#!/usr/bin/env python3
"""One-command host-controlled PWM/encoder diagnostic.

The same Python process builds/flashes opt-in firmware, verifies bidirectional
UART communication, sends the explicit START command, captures 3 rounds of
21 stages covering startup sweep, open-loop response and slew sensitivity; collects 400ms neutral stop traces and statistics.
NO KEY/button, no automatic motion on reset, no current/thermal safety claim.

IMPORTANT: MCU PA10 / USART1 RX must physically connect to the USB-UART TX
used by this COM port. If it is not wired, fail closed without START.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time

FIRMWARE_ENV = "motor_auto_pipeline_96m_trim25"
NUM_ROUNDS = 3
PULSE_MS = 200
POST_MS = 400
# Firmware owns this fixed capped matrix; host cannot submit arbitrary duty.
MATRIX = (
    (80,0),(0,80),(90,0),(0,90),(100,0),(0,100),
    (110,0),(0,110),(120,0),(0,120),
    (120,0),(0,120),(160,0),(0,160),(200,0),(0,200),(160,160),
    (160,0),(0,160),(160,0),(0,160)
)
TESTS = ("startup",)*10 + ("response",)*7 + ("slew_slow",)*2 + ("slew_fast",)*2
RISE_RATES = (10,)*17 + (5,)*2 + (20,)*2
MAX_CMD = 200
DUAL_MAX_CMD = 160
MAX_CCR = 640  # TIM2 96MHz / 30kHz = 3200 counts, 20% = 640
EXPECTED = NUM_ROUNDS * len(MATRIX)
CSV_FIELDS = ("round", "stage", "phase", "test", "rise", "ms", "elapsed",
              "applied_l", "applied_r", "enc_l", "enc_r", "ccr_l",
              "ccr_r", "tim11_late", "tim11_max_cycles", "tim11_gaps", "tx_drop")


def split_record(line: str):
    if "\t" in line:
        line = line.split("\t", 1)[1]
    name, sep, rest = line.strip().partition(",")
    if not sep:
        return name, {}
    values = {}
    for chunk in rest.split(","):
        if "=" in chunk:
            k, v = chunk.split("=", 1)
            values[k] = v
    return name, values


def intval(obj, key, default=-999999):
    try:
        return int(obj[key])
    except (KeyError, TypeError, ValueError):
        return default


def phase_findings(stats):
    """Summaries derived only from encoder counts, not physical voltage/current."""
    startup = {}
    for wheel in ("l", "r"):
        subset = sorted((x for x in stats if x["test"] == "startup" and
                         x["wheel"] == wheel), key=lambda x:x["cmd_permille"])
        reliable = [x["cmd_permille"] for x in subset if x["sustained_3of3"]]
        startup[wheel] = {
            "tested_permille": [x["cmd_permille"] for x in subset],
            "sustained_replicates_by_duty": {
                str(x["cmd_permille"]): x["sustained_replicates"] for x in subset},
            "lowest_tested_3of3_permille": min(reliable) if reliable else None,
        }
    response = []
    for duty in (120,160,200):
        left = next((x for x in stats if x["test"] == "response" and
                     x["wheel"] == "l" and x["cmd_permille"] == duty and
                     x["stage"] != 16), None)
        right = next((x for x in stats if x["test"] == "response" and
                      x["wheel"] == "r" and x["cmd_permille"] == duty and
                      x["stage"] != 16), None)
        if left is not None and right is not None and left["mean_count"] is not None and right["mean_count"] is not None:
            base = right["mean_count"]
            response.append({
                "pwm_permille":duty,"mean_left_encoder_count":left["mean_count"],
                "mean_right_encoder_count":base,
                "relative_count_difference_pct":round(100*(left["mean_count"]-base)/base,2) if base else None,
                "mean_left_post_stop_counts":round(statistics.mean(left["three_post_stop_deltas"]),2),
                "mean_right_post_stop_counts":round(statistics.mean(right["three_post_stop_deltas"]),2),
            })
    slew = []
    for wheel in ("l","r"):
        slow = next((x for x in stats if x["test"] == "slew_slow" and x["wheel"] == wheel),None)
        fast = next((x for x in stats if x["test"] == "slew_fast" and x["wheel"] == wheel),None)
        if slow is not None and fast is not None:
            slew.append({
                "wheel":wheel, "duty_permille":160,
                "rise_slow_permille_per_ms":5,
                "rise_fast_permille_per_ms":20,
                "mean_count_slow":slow["mean_count"],
                "mean_count_fast":fast["mean_count"],
                "mean_count_fast_minus_slow":round(fast["mean_count"]-slow["mean_count"],2) if
                    fast["mean_count"] is not None and slow["mean_count"] is not None else None,
            })
    return {
        "startup_threshold_heuristic":startup,
        "open_loop_response":response,
        "rise_slew_comparison":slew,
        "limitations":"Based on encoder count only; startup is a heuristic at the tested 8-12% settings and 3 repeats. Nonzero post-STOP counts do not establish exact electrical coast/brake mode."
    }


def analyze(lines, *, setup=None):
    records = [split_record(x) for x in lines]
    by_kind = {}
    for name, values in records:
        by_kind.setdefault(name, []).append(values)
    errors = []
    profile = by_kind.get("MOTOR_AUTO_PROFILE", [])
    ready = by_kind.get("MOTOR_AUTO_READY", [])
    begins = by_kind.get("MOTOR_AUTO_BEGIN", [])
    starts = by_kind.get("MOTOR_AUTO_START", [])
    ends = by_kind.get("MOTOR_AUTO_END", [])
    posts = by_kind.get("MOTOR_AUTO_POST_END", [])
    done = by_kind.get("MOTOR_AUTO_DONE", [])
    samples = by_kind.get("MOTOR_AUTO_SAMPLE", [])
    faults = by_kind.get("MOTOR_AUTO_FAULT", [])
    r_ends = by_kind.get("MOTOR_AUTO_ROUND_END", [])
    if not profile:
        errors.append("Missing firmware profile / wrong firmware loaded")
    else:
        p = profile[-1]
        for key, value in {
            "version": 3, "nominal_hz": 96000000,
            "pwm_hz": 30000, "rounds": NUM_ROUNDS,
            "steps": len(MATRIX), "hard_cap_permille": MAX_CMD,
            "both_cap_permille": DUAL_MAX_CMD, "families": 4,
            "pulse_ms": PULSE_MS, "post_ms": POST_MS,
            "boot_neutral": 1,
        }.items():
            if intval(p, key) != value:
                errors.append(f"Firmware profile mismatch: {key}")
    if not ready:
        errors.append("Missing UART RX handshake: MOTOR_AUTO_READY")
    if len(begins) != 1:
        errors.append("Expected exactly one host-triggered MOTOR_AUTO_BEGIN")
    if begins and (intval(begins[-1], "rounds") != NUM_ROUNDS or
                   intval(begins[-1], "steps") != len(MATRIX) or
                   intval(begins[-1], "boot_neutral") != 1):
        errors.append("Host START acknowledgment has incompatible run profile")
    if len(starts) != EXPECTED or len(ends) != EXPECTED or len(posts) != EXPECTED:
        errors.append(f"Missing/repeated stages (start/end/post counts): {len(starts)}/{len(ends)}/{len(posts)} expected {EXPECTED}")
    if len(r_ends) != NUM_ROUNDS:
        errors.append("Missing/repeated per-round completion signatures")
    if len(done) != 1 or (done and intval(done[-1], "pulses") != EXPECTED):
        errors.append("Missing/invalid full experiment completion record")
    if faults:
        errors.append("Firmware reported MOTOR_AUTO_FAULT: " + ",".join(x.get("reason", "?") for x in faults))

    rows = []
    individual = []
    for i, s in enumerate(samples):
        try:
            row = {k: str(s[k]) for k in CSV_FIELDS}
            for k in CSV_FIELDS:
                if k not in ("phase", "test"):
                    int(row[k])
            rows.append(row)
        except (KeyError, ValueError):
            errors.append(f"Malformed MOTOR_AUTO_SAMPLE at row {i}")
    seen = [(intval(x, "round"), intval(x, "stage"), intval(x, "l"), intval(x, "r"))
            for x in starts]
    expected = [(r, st, *pair) for r in range(NUM_ROUNDS)
                for st, pair in enumerate(MATRIX)]
    if seen != expected:
        errors.append("Start order or command matrix differs from three expected rounds")
    for x in starts:
        stage = intval(x, "stage")
        if not (0 <= stage < len(MATRIX)):
            errors.append("Invalid MCU START stage index")
        elif x.get("test") != TESTS[stage] or intval(x, "rise") != RISE_RATES[stage]:
            errors.append(f"Diagnostic test/slew mismatch in MCU START stage {stage}")
    if [(intval(x, "round"), intval(x, "stage")) for x in ends] != [x[:2] for x in expected]:
        errors.append("END order differs from start matrix")
    if [(intval(x, "round"), intval(x, "stage")) for x in posts] != [x[:2] for x in expected]:
        errors.append("POST_END order differs from start matrix")
    if [intval(x,"round") for x in r_ends] != [1,2,3]:
        errors.append("Round end ordering mismatch")
    for i, end in enumerate(ends):
        if intval(end, "reason") != 0 or intval(end, "elapsed") != PULSE_MS:
            errors.append(f"Pulse #{i} exceeded/failed intended 200ms timeout")
        if intval(end, "ccr_l") != 0 or intval(end, "ccr_r") != 0:
            errors.append(f"Pulse #{i} did not clear TIM2 CCR on stop")
    for i, p in enumerate(posts):
        if intval(p, "elapsed") != POST_MS or intval(p, "ccr_l") != 0 or intval(p, "ccr_r") != 0:
            errors.append(f"Post-stop observation #{i} invalid")
    for i, s in enumerate(samples):
        if s.get("phase") not in ("drive", "post"):
            errors.append(f"Unknown sample phase {i}")
        stage = intval(s, "stage")
        if not (0 <= stage < len(MATRIX)) or \
           s.get("test") != TESTS[stage] or intval(s, "rise") != RISE_RATES[stage]:
            errors.append(f"Invalid test/slew telemetry at sample {i}")
        if intval(s, "tim11_gaps") != 0 or intval(s, "tim11_late") != 0:
            errors.append("TIM11 deadline/jitter issue")
        if intval(s, "tx_drop") != 0:
            errors.append("UART DMA message loss")
        for k in ("applied_l", "applied_r"):
            if abs(intval(s, k)) > MAX_CMD:
                errors.append("Applied PWM exceeded hard cap")
        for k in ("ccr_l", "ccr_r"):
            if not (0 <= intval(s, k) <= MAX_CCR):
                errors.append("Invalid CCR (PWM > 20% at 3200 counts)")
        if s.get("phase") == "post" and any(intval(s, k) != 0 for k in
                                             ("applied_l", "applied_r", "ccr_l", "ccr_r")):
            errors.append("Nonneutral PWM detected during post-stop observation")

    for r in range(NUM_ROUNDS):
        for stage, cmd in enumerate(MATRIX):
            key = (str(r), str(stage))
            drives = [x for x in samples if (x.get("round"), x.get("stage")) == key
                      and x.get("phase") == "drive" and intval(x,"elapsed") < PULSE_MS]
            after = [x for x in samples if (x.get("round"), x.get("stage")) == key
                     and x.get("phase") == "post"]
            e = next((x for x in ends if (x.get("round"),x.get("stage")) == key), {})
            p = next((x for x in posts if (x.get("round"),x.get("stage")) == key), {})
            if len(drives) < 6 or len(after) < 10:
                errors.append(f"Insufficient 20ms samples round={r} stage={stage} drive={len(drives)} post={len(after)}")
            if any(intval(x, "ccr_l") != 0 for x in drives if cmd[0] == 0) or \
               any(intval(x, "ccr_r") != 0 for x in drives if cmd[1] == 0):
                errors.append(f"Noncommanded wheel CCR nonzero r={r} stage={stage}")
            if drives and any(abs(intval(x,"applied_l")) > cmd[0] or
                              abs(intval(x,"applied_r")) > cmd[1] for x in drives):
                errors.append(f"Requested PWM cap exceeded r={r} stage={stage}")
            if cmd[0] and cmd[1] and (cmd[0] > DUAL_MAX_CMD or cmd[1] > DUAL_MAX_CMD):
                errors.append(f"Dual-wheel duty beyond allowed cap r={r} stage={stage}")
            # Compare actual timer CCR peak against commanded levels. A zero
            # or truncated PWM implementation must not pass just because
            # all 21/27 START/END records exist.
            if drives:
                for wheel, duty in (("l", cmd[0]), ("r", cmd[1])):
                    peak = max(intval(x, f"ccr_{wheel}") for x in drives)
                    if duty > 0 and peak != duty * 3200 // 1000:
                        errors.append(f"PWM CCR mismatch r={r} stage={stage} wheel={wheel}: peak={peak}")
            driven_counts = []
            for wheel, request in (("l", cmd[0]), ("r", cmd[1])):
                final = intval(e, f"enc_{wheel}", 0)
                after_count = intval(p, f"enc_{wheel}", 0)
                tail = [x for x in drives if intval(x,"elapsed") >= 99]
                tail_delta = (intval(tail[-1],f"enc_{wheel}",0) -
                              intval(tail[0],f"enc_{wheel}",0)) if len(tail)>1 else None
                sustained = request > 0 and abs(final) >= 5 and \
                            (tail_delta is not None and abs(tail_delta) >= 5)
                first5 = next((intval(x,"elapsed") for x in drives
                               if abs(intval(x,f"enc_{wheel}",0)) >= 5), None)
                driven_counts.append({
                    "wheel":wheel, "requested_permille": request, "encoder_drive": final,
                    "encoder_after_post_ms": after_count,
                    "post_stop_delta": after_count-final,
                    "final_100ms_delta":tail_delta, "first_five_counts_ms":first5,
                    "sustained_count_signal": bool(sustained),
                })
            individual.append({
                "round":r, "stage":stage,"test":TESTS[stage],
                "rise_rate_permille_per_ms":RISE_RATES[stage],
                "cmd_left":cmd[0],"cmd_right":cmd[1],
                "drive_samples":len(drives),"post_samples":len(after),
                "wheel_readings":driven_counts,
            })
    stats = []
    for stage, cmd in enumerate(MATRIX):
        for wheel, duty in (("l",cmd[0]),("r",cmd[1])):
            if duty == 0: continue
            vals = [next(x for x in trial["wheel_readings"] if x["wheel"]==wheel)
                    for trial in individual if trial["stage"]==stage]
            counts = [x["encoder_drive"] for x in vals]
            stats.append({
                "stage":stage, "test":TESTS[stage],
                "rise_rate_permille_per_ms":RISE_RATES[stage],
                "wheel":wheel, "cmd_permille":duty,
                "three_drive_counts":counts,
                "mean_count":round(statistics.mean(counts),2) if counts else None,
                "sample_std_count":round(statistics.stdev(counts),2) if len(counts)>1 else None,
                "sustained_3of3":all(x["sustained_count_signal"] for x in vals) and len(vals)==3,
                "sustained_replicates":sum(x["sustained_count_signal"] for x in vals),
                "three_post_stop_deltas":[x["post_stop_delta"] for x in vals],
                "first_five_counts_ms":[x["first_five_counts_ms"] for x in vals]
            })
    if done and (intval(done[-1],"ccr_l") != 0 or intval(done[-1],"ccr_r") != 0 or
                 intval(done[-1],"tim11_gaps") != 0 or intval(done[-1],"tx_drop") != 0):
        errors.append("Nonneutral/invalid final MCU status")
    errors = sorted(set(errors))
    return {
        "status": "TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" if not errors else "INCOMPLETE",
        "rounds_requested": NUM_ROUNDS, "pulses_expected": EXPECTED,
        "starts":len(starts), "ends":len(ends), "post_ends":len(posts),
        "sample_count":len(samples),"errors":errors,
        "test_families":["startup","response","slew_slow","slew_fast"],
        "stages_per_round":len(MATRIX),"post_stop_observation_ms":POST_MS,
        "repeat_statistics":stats,"trials":individual,
        "phase_findings":phase_findings(stats),
        "timing_integrity":{
            "max_tim11_dwt_cycles_seen": max((intval(x,"tim11_max_cycles",0) for x in samples), default=0),
            "max_observed_tim11_late_ticks": max((intval(x,"tim11_late",0) for x in samples), default=0),
            "max_observed_tim11_gaps": max((intval(x,"tim11_gaps",0) for x in samples), default=0),
            "max_observed_uart_tx_drop": max((intval(x,"tx_drop",0) for x in samples), default=0),
        },
        "experiment_setup":setup or {},
        "electrical_limitation":"3.3V MCU GPIO HIGH is below guaranteed TB6612 VIH with logic VCC=5V; STBY tied 5V; no measured current/temperature/physical PWM waveform. Encoder signal is not verified wheel RPM. 20pct duty is not a current limit.",
        "interpretation":"Sustained_count_signal is an empirical >=5 encoder-count and >=5 tail-count heuristic, NOT a certified deadband threshold.",
    }, rows


def self_test():
    synthetic = [
        "[BOOT] Motor auto baseline: neutral, USART1 host ARM required",
        f"MOTOR_AUTO_PROFILE,version=3,nominal_hz=96000000,pwm_hz=30000,rounds=3,steps={len(MATRIX)},hard_cap_permille=200,both_cap_permille=160,families=4,pulse_ms=200,post_ms=400,boot_neutral=1",
        f"MOTOR_AUTO_READY,version=3,rounds=3,steps={len(MATRIX)},hard_cap_permille=200,both_cap_permille=160,families=4,pulse_ms=200,post_ms=400,one_shot=1,boot_neutral=1",
        f"MOTOR_AUTO_BEGIN,rounds=3,steps={len(MATRIX)},settle_ms=1000,boot_neutral=1"
    ]
    for r in range(NUM_ROUNDS):
        for i,(left,right) in enumerate(MATRIX):
            test, rise = TESTS[i], RISE_RATES[i]
            synthetic.append(f"MOTOR_AUTO_START,round={r},stage={i},l={left},r={right},test={test},rise={rise},pulse_ms=200")
            for n in range(1,11):
                t=n*20-1
                synthetic.append(f"MOTOR_AUTO_SAMPLE,round={r},stage={i},phase=drive,test={test},rise={rise},ms={t},elapsed={t},applied_l={left},applied_r={right},enc_l={int(left>0)*n*5},enc_r={int(right>0)*n*5},ccr_l={left*32//10},ccr_r={right*32//10},tim11_late=0,tim11_max_cycles=1000,tim11_gaps=0,tx_drop=0")
            synthetic.append(f"MOTOR_AUTO_END,round={r},stage={i},elapsed=200,reason=0,enc_l={int(left>0)*50},enc_r={int(right>0)*50},ccr_l=0,ccr_r=0")
            for n in range(1,POST_MS//20+1):
                t=n*20
                synthetic.append(f"MOTOR_AUTO_SAMPLE,round={r},stage={i},phase=post,test={test},rise={rise},ms={t},elapsed={t},applied_l=0,applied_r=0,enc_l={int(left>0)*50},enc_r={int(right>0)*50},ccr_l=0,ccr_r=0,tim11_late=0,tim11_max_cycles=1000,tim11_gaps=0,tx_drop=0")
            synthetic.append(f"MOTOR_AUTO_POST_END,round={r},stage={i},elapsed={POST_MS},enc_l={int(left>0)*50},enc_r={int(right>0)*50},ccr_l=0,ccr_r=0")
        synthetic.append(f"MOTOR_AUTO_ROUND_END,round={r+1},completed={(r+1)*len(MATRIX)}")
    synthetic.append(f"MOTOR_AUTO_DONE,rounds=3,pulses={EXPECTED},ccr_l=0,ccr_r=0,tim11_gaps=0,tx_drop=0")
    report,_ = analyze(synthetic)
    assert report["status"]=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL", report["errors"]
    assert len(report["repeat_statistics"])==22
    def incomplete(lines):
        x,_=analyze(lines)
        assert x["status"]=="INCOMPLETE", x
    incomplete([x for x in synthetic if not x.startswith("MOTOR_AUTO_PROFILE")])
    incomplete([x for x in synthetic if "MOTOR_AUTO_END,round=1,stage=3" not in x])
    incomplete(synthetic + ["MOTOR_AUTO_FAULT,reason=HOST_TIMEOUT"])
    incomplete([x.replace("ccr_l=0,ccr_r=0,tim11_late=0","ccr_l=384,ccr_r=0,tim11_late=0")
                if "phase=post" in x else x for x in synthetic])
    print(f"SELF-TEST PASSED: {NUM_ROUNDS}-round {EXPECTED}-pulse four-phase capture; bad profile, missing stage, fault, nonneutral post rejected.")
    print("Host synthetic test only; not hardware validation.")


def store(folder, raw, setup):
    folder.mkdir(parents=True,exist_ok=True)
    (folder/"raw_serial.log").write_text("\n".join(raw)+"\n",encoding="utf-8")
    (folder/"session_meta.json").write_text(
        json.dumps(setup,indent=2)+"\n",encoding="utf-8")
    report,rows=analyze(raw,setup=setup)
    with (folder/"motor_auto_samples.csv").open("w",newline="",encoding="utf-8-sig") as fp:
        writer=csv.DictWriter(fp,fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    (folder/"motor_auto_report.json").write_text(
        json.dumps(report,indent=2)+"\n",encoding="utf-8")
    lines=["Motor auto 3-round diagnostic",f"Status: {report['status']}",
           f"Stages: {report['starts']}/{EXPECTED}; samples: {report['sample_count']}",
           "Encoder counts are not RPM. Electrical current/thermal limits were NOT measured.",
           ""]
    findings = report["phase_findings"]
    lines.append("PHASE A: startup 8-12% (3/3 sustained encoder count heuristic)")
    for wheel in ("l","r"):
        th = findings["startup_threshold_heuristic"][wheel]
        lines.append(f"  {wheel}: confirmed-per-duty={th['sustained_replicates_by_duty']}; lowest-test-3of3={th['lowest_tested_3of3_permille']}")
    lines.append("PHASE B: PWM response and post-zero encoder movement")
    for x in findings["open_loop_response"]:
        lines.append(f"  {x['pwm_permille']} permille: L/R={x['mean_left_encoder_count']}/{x['mean_right_encoder_count']}, "
                     f"delta={x['relative_count_difference_pct']}%, postzero-L/R={x['mean_left_post_stop_counts']}/{x['mean_right_post_stop_counts']}")
    lines.append("PHASE C: rise slew 5 vs 20 permille/ms at 16%")
    for x in findings["rise_slew_comparison"]:
        lines.append(f"  {x['wheel']}: slow={x['mean_count_slow']}, fast={x['mean_count_fast']}, "
                     f"fast-minus-slow={x['mean_count_fast_minus_slow']} counts")
    lines.append("PHASE D: per-condition 400ms post-zero observations (details below)")
    for x in report["repeat_statistics"]:
        lines.append(f"[{x['test']}] Stage {x['stage']} {x['wheel']} {x['cmd_permille']}permille, rise={x['rise_rate_permille_per_ms']}: "
                     f"counts={x['three_drive_counts']}, mean={x['mean_count']}, "
                     f"std={x['sample_std_count']}, sustained={x['sustained_replicates']}/3, "
                     f"post-stop-delta={x['three_post_stop_deltas']}")
    if report["errors"]:
        lines.extend(["","ERRORS:"]+report["errors"])
    (folder/"motor_auto_summary.txt").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print("\n".join(lines))
    print("Output:",folder.resolve())
    return report


def execute(args):
    if args.vm_state != "powered":
        raise SystemExit("--run requires explicit --vm-state powered. For unpowered host protocol validation use --check-link.")
    if args.meter_vm_volts is not None and not (0 <= args.meter_vm_volts <= 12):
        raise SystemExit("Measured VM DC voltage must be in the range 0..12V")
    folder=Path("motor_logs")/("auto_"+dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    setup={"vm_state_declared":args.vm_state,
           "dc_vm_volts_hand_measured":args.meter_vm_volts,
           "firmware_env":FIRMWARE_ENV,"host_control":"USART1 RX PA10",
           "rounds":NUM_ROUNDS,"stages_per_round":len(MATRIX),
           "diagnostic_phases":["startup","response","slew_slow","slew_fast"],
           "note":"VM state user-supplied, not measured by STM32; duty is not current limit."}
    if not args.skip_flash:
        print("Build + flash opt-in firmware via STLINK:", FIRMWARE_ENV,flush=True)
        subprocess.run([sys.executable,"-m","platformio","run","-e",FIRMWARE_ENV,"-t","upload"],check=True)
    else:
        print("SKIPPING FLASH: firmware/profile must already match; the handshake will verify profile.")
    try:
        import serial
    except ImportError as e:
        raise SystemExit("Install dependency: python -m pip install pyserial") from e
    raw=[]
    began=False
    complete=False
    requested_start=False
    try:
        with serial.Serial(args.port,115200,timeout=0.12,write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            start=time.monotonic()
            ping_at=0.0
            keep_at=0.0
            handshake_deadline=start+10.0
            print("Connected:",args.port,"- host must receive MOTOR_AUTO_READY before motion.")
            print("Wheels MUST be elevated; have a physical VM disconnect ready. STOP via UART is not a hardware kill switch.")
            while time.monotonic()-start<args.timeout:
                now=time.monotonic()
                if not requested_start and now>=ping_at:
                    ser.write(b"MOTOR PING\n")
                    ping_at=now+0.4
                if requested_start and now>=keep_at and not complete:
                    ser.write(b"MOTOR KEEP\n")
                    keep_at=now+0.35
                if not requested_start and now>handshake_deadline:
                    print("NO BIDIRECTIONAL UART HANDSHAKE: PA10 RX may not be connected to USB-UART TX. No START was sent.")
                    break
                packet=ser.readline()
                if not packet:
                    continue
                frame=packet.decode("utf-8",errors="replace").strip()
                raw.append(f"{now-start:.6f}\t{frame}")
                kind,v=split_record(frame)
                if kind=="MOTOR_AUTO_READY" and not requested_start:
                    if any(intval(v,k)!=value for k,value in (
                        ("version",3),("rounds",NUM_ROUNDS),
                        ("steps",len(MATRIX)),("hard_cap_permille",MAX_CMD),
                        ("both_cap_permille",DUAL_MAX_CMD),("families",4),
                        ("pulse_ms",PULSE_MS),("post_ms",POST_MS),
                        ("boot_neutral",1))) or intval(v,"state")!=0:
                        print("Unsafe/incompatible firmware or motor already active. REFUSING START:",frame)
                        break
                    print("RX handshake verified. Explicit host START issued.")
                    ser.write(b"MOTOR START\n")
                    requested_start=True
                    keep_at=0.0
                elif kind=="MOTOR_AUTO_BEGIN":
                    began=True
                    print(frame)
                elif kind in ("MOTOR_AUTO_START","MOTOR_AUTO_END",
                              "MOTOR_AUTO_ROUND_END","MOTOR_AUTO_FAULT","MOTOR_AUTO_DONE"):
                    print(frame,flush=True)
                    if kind=="MOTOR_AUTO_FAULT": break
                    if kind=="MOTOR_AUTO_DONE":
                        complete=True
                        # Keep capturing small trailing UART frames without active outputs.
                        deadline=time.monotonic()+0.35
                        while time.monotonic()<deadline:
                            b=ser.readline()
                            if b:
                                raw.append(f"{time.monotonic()-start:.6f}\t{b.decode('utf-8',errors='replace').strip()}")
                        break
            if requested_start and not complete:
                try: ser.write(b"MOTOR STOP\n")
                except Exception: pass
    except KeyboardInterrupt:
        # The with-block closes the port before propagating KeyboardInterrupt.
        # Reopen once to send a best-effort stop. This does NOT replace VM cut.
        try:
            with serial.Serial(args.port,115200,timeout=0.1,write_timeout=0.3) as stopser:
                stopser.write(b"MOTOR STOP\n")
        except (serial.SerialException, OSError):
            pass
        print("Interrupted: best-effort UART STOP requested. Physically cut VM if unexpected motion continues.")
    except serial.SerialException as e:
        print("UART failure:",e,"- physically disconnect VM if there is unexpected motion.")
    except subprocess.CalledProcessError as e:
        raise SystemExit(f"Firmware upload failed: {e}") from e

    report=store(folder,raw,setup)
    if not began:
        print("No motor sequence began. If RX handshake failed, check COM6 USB-UART TX -> STM32 PA10 and common ground; board wiring must not be changed if prohibited.")
    return 0 if report["status"]=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" else 1


def check_link(args):
    print("Link test: no START will be sent. Validates STM32 UART RX only.")
    if not args.skip_flash:
        subprocess.run([sys.executable,"-m","platformio","run","-e",FIRMWARE_ENV,"-t","upload"],check=True)
    try:
        import serial
    except ImportError as e:
        raise SystemExit("pip install pyserial") from e
    with serial.Serial(args.port,115200,timeout=0.2,write_timeout=1) as ser:
        ser.reset_input_buffer()
        start=time.monotonic()
        while time.monotonic()-start < 8.0:
            ser.write(b"MOTOR PING\n")
            for _ in range(3):
                data=ser.readline().decode(errors="replace").strip()
                if data:
                    print(data)
                if split_record(data)[0]=="MOTOR_AUTO_READY":
                    print("BIDIRECTIONAL UART PASS; no motor drive commands sent.")
                    return 0
        print("BIDIRECTIONAL UART NOT CONFIRMED. No motor drive command sent.")
        return 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument("--run",action="store_true",help="Explicitly authorize one 3-round experiment")
    group.add_argument("--check-link",action="store_true",help="UART RX-only test; no motor drive")
    group.add_argument("--self-test",action="store_true",help="Offline host parser regression")
    group.add_argument("--folder",type=Path,help="Re-analyze a saved raw log directory")
    p.add_argument("--port",help="USB-UART port e.g. COM6")
    p.add_argument("--vm-state",choices=["powered","disconnected"])
    p.add_argument("--meter-vm-volts",type=float)
    p.add_argument("--skip-flash",action="store_true")
    p.add_argument("--timeout",type=float,default=150.0)
    args=p.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.folder:
        raw_path=args.folder/"raw_serial.log"
        if not raw_path.is_file():
            p.error(f"Missing {raw_path}")
        meta_path=args.folder/"session_meta.json"
        meta=json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
        report=store(args.folder,raw_path.read_text(encoding="utf-8").splitlines(),meta)
        return 0 if report["status"]=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" else 1
    if not args.port:
        p.error("--port is required for --run or --check-link")
    if args.timeout < 80:
        p.error("--timeout must be at least 80 seconds for the full four-phase suite")
    if args.check_link: return check_link(args)
    return execute(args)


if __name__=="__main__":
    sys.exit(main())
