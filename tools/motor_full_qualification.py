#!/usr/bin/env python3
"""One entrypoint for all five high-speed Micromouse Motor Driver qualification gates.

`--audit` analyses code + already captured logs without flashing or rotating.
`--run-integrated` first audits, then builds/uploads exactly ONE opt-in
motor+MPU Policy B+IR image; cal occurs before UART READY/motor START.
High PWM requires *external electrical evidence*; no hardware current
sensor exists, and neither this host script nor STM32 measures motor amps.
Brake, stall, energized reverse and 80-100% remain intentionally BLOCKED.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BASELINE = ROOT / "motor_logs" / "auto_20261013_105353"
# Toshiba operating-range max is 1.0A at VM >=4.5V without PWM;
# 1.2A is an absolute maximum, not a continuous design target.
# This conservative evidence prefilter is NOT current limitation.
MAX_TB6612_CONTINUOUS_A = 1.0
# The 3.2A absolute peak applies to a *single* 10ms pulse, NOT 120ms.
MAX_ALLOWED_EVIDENCE_PEAK_A = 1.0


def gate(code, label, status, explanation, evidence=None):
    result = {"gate":code,"label":label,"status":status,"explanation":explanation}
    if evidence is not None: result["evidence"] = evidence
    return result


def inspect_driver():
    motor=(ROOT/"src/motor.c").read_text(encoding="utf-8")
    pins=(ROOT/"include/pinout.h").read_text(encoding="utf-8")
    checks = {
        "stop_zero_ccr": "TIM2->CCR1 = TIM2->CCR2 = 0;" in motor,
        "stop_both_inputs_low": "AddDirection(&bsrr, MOTOR_L_IN1_PIN, MOTOR_L_IN2_PIN, 0);" in motor,
        "active_brake_implemented": "void Motor_Brake(void)" in motor and
            "TIM2->CCR1 = pwm_period + 1U;" in motor,
        "preload_enabled": "TIM_CCMR1_OC1PE" in motor and "TIM_CCMR1_OC2PE" in motor,
        "reverse_gap_30ms": "#define MOTOR_REVERSE_GAP_MS 30U" in motor,
        "hardware_pwm_30khz": "#define MOTOR_PWM_FREQUENCY     30000" in pins,
        "stby_controllable_from_firmware": False,
        "pinout_comment_vs_macro_mismatch":
            "IN1=PB14, IN2=PB15" in pins and
            "#define MOTOR_R_IN1_PIN         15" in pins,
    }
    blockers=[]
    if not all(checks[k] for k in (
        "stop_zero_ccr","stop_both_inputs_low","preload_enabled",
        "reverse_gap_30ms","hardware_pwm_30khz")):
        blockers.append("Software STOP/PWM/pin mapping invariants require repair.")
    blockers += [
        "TB6612 OUT1/OUT2 coast/short-brake cannot be confirmed by GPIO/CCR register log alone.",
        "IN1/IN2 change immediately but timer CCR is preloaded, requiring scope transition check.",
        "STBY tied HIGH; no independent MCU-controlled power-stage disable.",
        "TB6612 VCC=5V requires VIH>=3.5V; MCU 3.3V HIGH lacks guaranteed margin.",
    ]
    return checks,blockers


def evaluate_electrical(evidence_file):
    if evidence_file is None:
        return gate("P0-ELECTRICAL","Motor current, waveform, temperature","BLOCKED",
                    "No qualified external current/thermal/logic/scope evidence. STM32 does not measure current.")
    try:
        e=json.loads(evidence_file.read_text(encoding="utf-8"))
    except (OSError,ValueError) as err:
        return gate("P0-ELECTRICAL","Motor current, waveform, temperature","BLOCKED",
                    f"Missing/invalid evidence file: {err}")
    required = ("motor_exact_part","vm_measured_v","left_peak_a","right_peak_a",
                "left_running_a","right_running_a","left_stall_a","right_stall_a",
                "tb6612_case_temp_c","measured_gpio_high_v","output_waveform_verified",
                "physical_vm_disconnect_verified","independent_current_cutoff_verified",
                "measurement_method","operator","date")
    missing=[k for k in required if k not in e or e[k] in (None,"",False)]
    if missing:
        return gate("P0-ELECTRICAL","Motor current, waveform, temperature","BLOCKED",
                    "Evidence incomplete: "+", ".join(missing),
                    {"supplied_keys":sorted(e)})
    try:
        peak=max(float(e[k]) for k in ("left_peak_a","right_peak_a",
                   "left_running_a","right_running_a","left_stall_a","right_stall_a"))
        vm=float(e["vm_measured_v"])
        high=float(e["measured_gpio_high_v"])
        temp=float(e["tb6612_case_temp_c"])
    except (TypeError,ValueError) as err:
        return gate("P0-ELECTRICAL","Motor current, waveform, temperature","BLOCKED",
                    f"Non-numeric measurement: {err}")
    if not(0<vm<=12) or peak<=0 or peak>=MAX_ALLOWED_EVIDENCE_PEAK_A or high<3.5 or temp>=65:
        return gate("P0-ELECTRICAL","Motor current, waveform, temperature","BLOCKED",
                    "Measured values do not satisfy conservative gate: all provided amps <1.0A, VM<=12V, GPIO HIGH>=3.5V, case<65C. This is a project-specific gate, not device certification.",
                    {"maximum_reported_current_a":peak,"vm_v":vm,"gpio_high_v":high,"case_temp_c":temp})
    return gate("P0-ELECTRICAL","Motor current, waveform, temperature","REVIEW",
                "User-supplied external measurements pass conservative numeric filters but are not independently verified. Review scope waveforms, 10/20ms peak limitations, thermal rise and VM transient before any high test.",
                {"maximum_reported_current_a":peak,"model":e["motor_exact_part"],"case_temp_c":temp})


def startup_reinterpretation(stats):
    """Independently repair the v6 '8% succeeds, 9% fails' inference."""
    result={}
    for wheel in ("l","r"):
        old=stats.get(wheel,{})
        counts=old.get("sustained_replicates_by_duty",{})
        try:
            pairs=sorted((int(k),int(v)) for k,v in counts.items() if v is not None)
        except (TypeError,ValueError):
            pairs=[]
        monotonic = [d for i,(d,n) in enumerate(pairs)
                     if all(value==3 for _,value in pairs[i:])]
        inversions=[{"lower_duty_permille":a,"higher_duty_permille":b}
                    for (a,na),(b,nb) in zip(pairs,pairs[1:]) if na>nb]
        result[wheel]={
            "observed_repetitions":pairs,
            "naive_min_successful_permille":min(
                (d for d,n in pairs if n==3),default=None),
            "conservative_monotonic_suffix_candidate_permille":min(monotonic,default=None),
            "nonmonotonic_pairs":inversions,
            "calibrated_under_load":False,
        }
    return result


def build_report(reference, evidence_file, latest=None):
    checks,blockers=inspect_driver()
    gates=[
        gate("P0-STOP","STOP/COAST/BRAKE and CCR preload","REVIEW" if all([
            checks["stop_zero_ccr"],checks["stop_both_inputs_low"],checks["preload_enabled"]]) else "BLOCKED",
            "Source-level command states audited; live short-brake/reversal deliberately NOT triggered.",
            {"checks":checks,"blockers":blockers}),
        evaluate_electrical(evidence_file),
    ]
    try:
        report=json.loads((reference/"motor_auto_report.json").read_text(encoding="utf-8"))
        source_status=report.get("status","UNKNOWN")
        stats=report.get("phase_findings",{}).get("startup_threshold_heuristic",{})
        startup=gate("P1-ENCODER","Startup/encoder 8-12% and repeatability",
                     "REVIEW" if source_status=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" else "INCOMPLETE",
                     "Existing v6 87-pulse measurement reused; nonmonotonic startup and test-order effects cannot be resolved from this fixed-order run.",
                     {"source":str(reference.relative_to(ROOT)) if reference.is_relative_to(ROOT) else str(reference),
                      "original_startup":stats,
                      "corrected_startup_interpretation":startup_reinterpretation(stats)})
        high=gate("P2-HIGH","30-60% and 80-100%","REVIEW" if
                  source_status=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" else "INCOMPLETE",
                  "30-60% have only short, elevated-wheel 120ms pulses in v6, not continuous operation. 80-100% and loaded tests BLOCKED pending electrical limits.",
                  {"starts":report.get("starts"),"samples":report.get("sample_count"),
                   "60pct_verified_as_safe":False,"80_to_100pct_allowed":False})
    except (OSError,ValueError):
        startup=gate("P1-ENCODER","Startup/encoder","NOT_RUN",
                     "Missing prior v6 report; no startup threshold available.")
        high=gate("P2-HIGH","PWM 30-100%","BLOCKED",
                  "No reference bench report and no high-current approval.")
    gates.extend([startup])
    if latest is None:
        gates.append(gate("P1-INTEGRATION","Motor+MPU6050/I2C+IR+TIM11","NOT_RUN",
                          "Integrated v7/v8 firmware and analyzer provided; hardware run not yet executed."))
    else:
        try:
            merged=json.loads((latest/"motor_auto_report.json").read_text(encoding="utf-8"))
            si=merged.get("sensor_integration",{})
            trace_ok=(merged.get("status")=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" and
                      (si.get("gyro_fresh_delta") or 0)>0 and
                      (si.get("ir_scans_delta") or 0)>0)
            timing=merged.get("timing_integrity",{})
            sensor_anomaly=((si.get("i2c_error_delta") or 0)>0 or
                            (si.get("i2c_timeout_delta") or 0)>0 or
                            (timing.get("max_observed_tim11_late_ticks") or 0)>0 or
                            (timing.get("max_observed_tim11_gaps") or 0)>0 or
                            (timing.get("max_observed_uart_tx_drop") or 0)>0)
            status=("PASS_BENCH" if trace_ok and not sensor_anomaly else
                    "REVIEW" if trace_ok else "INCOMPLETE")
            gates.append(gate("P1-INTEGRATION","Motor+MPU6050/I2C+IR+TIM11",
                              status,
                              "Bench PASS requires samples and scans progressing, no new I2C timeout/errors and no TIM11 late/gaps or UART drops. Gyro-on/neutral differences still cannot prove EMI.",
                              {"source":str(latest),"report_status":merged.get("status"),
                               "sensor_integration":si,"timing":merged.get("timing_integrity")}))
        except (OSError,ValueError) as err:
            gates.append(gate("P1-INTEGRATION","Motor+MPU6050/I2C+IR+TIM11","INCOMPLETE",
                              f"Cannot read latest integrated output: {err}"))
    gates.append(high)
    gates.append(gate("P2-LOADED","Loaded travel and 80-100% PWM","BLOCKED",
                      "No independent current-limited instrumented setup, heat/VM scope review, loaded drivetrain data or qualified brake/reverse control. This script does not issue these motions."))
    return {
        "schema_version":1,
        "created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),
        "source_revision_note":"Source checks are code-level only; numeric measurements are user supplied.",
        "statuses":{p["gate"]:p["status"] for p in gates},
        "gates":gates,
        "next_action":"External electrical instrumentation and source/pin STOP audit before stronger drive modes.",
    }


def write_results(report):
    folder=ROOT/"motor_logs"/("qualification_"+dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    folder.mkdir(parents=True,exist_ok=True)
    (folder/"qualification_report.json").write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    lines=["MICROMOUSE MOTOR DRIVER FULL QUALIFICATION",f"UTC: {report['created_utc']}",""]
    for item in report["gates"]:
        lines.append(f"{item['gate']}: {item['status']} | {item['label']}")
        lines.append("  "+item["explanation"])
    lines += ["","Blocked != failed motor: it means required sensors/measurement or approvals are absent.",
              "No current, brake, reverse or loaded 80-100% test is automatically executed."]
    (folder/"qualification_summary.txt").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print("\n".join(lines))
    print("Output:",folder)
    return folder


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--self-test",action="store_true",help="Host-only gate logic test, no hardware")
    p.add_argument("--audit",action="store_true",help="Review code/evidence and prior logs without flashing")
    p.add_argument("--run-integrated",action="store_true",help="One STLINK flash + one sequential MPU/IR/motor diagnostic run")
    p.add_argument("--reference",type=Path,default=DEFAULT_BASELINE,
                   help="Existing high PWM v6 baseline log directory")
    p.add_argument("--electrical-evidence",type=Path,
                   help="Manually measured and independently reviewed current/temperature/waveform JSON")
    p.add_argument("--with-high",action="store_true",
                   help="Opt-in to 30-60 percent only after external electrical gate; never auto 80-100 percent")
    p.add_argument("--port",default="COM6")
    p.add_argument("--vm-state",choices=("powered","disconnected"),default="disconnected")
    p.add_argument("--meter-vm-volts",type=float)
    p.add_argument("--confirm-wheels-elevated",action="store_true")
    p.add_argument("--skip-flash",action="store_true")
    a=p.parse_args()
    if a.self_test:
        audit=build_report(a.reference,None)
        assert audit["statuses"]["P0-ELECTRICAL"]=="BLOCKED"
        sample={"l":{"sustained_replicates_by_duty":{
            "80":3,"90":0,"100":3,"110":3,"120":3}}}
        interpretation=startup_reinterpretation(sample)["l"]
        assert interpretation["naive_min_successful_permille"]==80
        assert interpretation["conservative_monotonic_suffix_candidate_permille"]==100
        assert interpretation["nonmonotonic_pairs"]
        assert audit["statuses"]["P2-LOADED"]=="BLOCKED"
        print("PASS: offline P0 hardware block, STOP audit, P1 pending, P2 80-100 block.")
        return 0
    if a.with_high and not a.run_integrated:
        p.error("--with-high requires --run-integrated")
    audit=build_report(a.reference,a.electrical_evidence)
    if not a.run_integrated:
        write_results(audit)
        return 0
    if a.vm_state!="powered" or not a.confirm_wheels_elevated:
        p.error("Motor run requires --vm-state powered and --confirm-wheels-elevated; independent physical VM disconnect required")
    if a.with_high and audit["statuses"]["P0-ELECTRICAL"]!="REVIEW":
        write_results(audit)
        p.error("Combined 30-60% mode BLOCKED until external electrical evidence passes. No flash or motor action performed.")
    before={x for x in (ROOT/"motor_logs").glob("auto_*") if x.is_dir()}
    cmd=[sys.executable,str(ROOT/"tools/motor_auto_pipeline.py"),"--run",
         "--sensor-integration","--confirm-wheels-elevated","--port",a.port,
         "--vm-state","powered","--timeout","210"]
    if a.with_high: cmd += ["--high-pwm"]
    if a.skip_flash: cmd += ["--skip-flash"]
    if a.meter_vm_volts is not None: cmd+=["--meter-vm-volts",str(a.meter_vm_volts)]
    # All motor start, watchdog and serial guards remain owned by the
    # existing verified host script + opt-in STM32 firmware.
    proc=subprocess.run(cmd,cwd=ROOT,check=False)
    created=sorted((x for x in (ROOT/"motor_logs").glob("auto_*")
                    if x.is_dir() and x not in before),key=lambda x:x.stat().st_mtime)
    latest=created[-1] if created else None
    report=build_report(a.reference,a.electrical_evidence,latest)
    report["hardware_process_exit_code"]=proc.returncode
    report["hardware_output_folder"]=str(latest) if latest else None
    write_results(report)
    return 0 if proc.returncode==0 else 1


if __name__=="__main__":
    raise SystemExit(main())
