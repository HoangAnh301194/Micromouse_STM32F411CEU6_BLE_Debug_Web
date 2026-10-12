#!/usr/bin/env python3
"""Validate production-policy MPU integration (motor OFF, diagnostic OFF).

This analyzer cannot prove hardware or ISR WCET from static/synthetic tests.
Usage:
  python tools/mpu_integration_regression.py --self-test
  python tools/mpu_integration_regression.py --source-check
  python tools/mpu_integration_regression.py --folder mpu_integration_logs/SESSION
"""
from __future__ import annotations
import argparse
import datetime as dt
import json
from pathlib import Path
import time

BASELINE = {
    "gyro_reused_pct": 0.0441,
    "tim11_stage2_max_us_nominal": 22.844,
    "fresh_gaps_over_2ms": 2,
    "gyro_age_gt1500": 99,
}
REQUIRED_PROFILE = {
    "policy": "B", "diag": "0", "ab_diag": "0", "load_diag": "0",
    "motor_off": "1", "nominal_clock_hz": "96000000", "trim": "25",
    "retry_us": "220", "retry_start_limit_us": "600",
    "retry_end_limit_us": "850"
}


def decode(s: str):
    s = s.split("\t", 1)[-1].strip()
    tag, _, data = s.partition(",")
    fields = {}
    for f in data.split(","):
        if "=" in f:
            key, val = f.split("=", 1)
            fields[key] = val
    return tag, fields


def analyze(lines, min_samples=150):
    boot_indices = [i for i, x in enumerate(lines) if "[MINT_BOOT]" in x]
    errors, warnings = [], []
    if len(boot_indices) != 1:
        errors.append("Expected exactly one verified MINT_BOOT (reset duplicates invalidate run)")
    if not boot_indices:
        return {"status": "INCOMPLETE", "failures": errors, "warnings": warnings}
    lines = lines[boot_indices[0]:]
    d = [decode(x) for x in lines]
    def tagged(tag):
        return [v for t, v in d if t == tag]
    profile = tagged("MINT_PROFILE")
    i2c = tagged("MINT_I2C")
    cal = tagged("MINT_CAL_DONE")
    frames = tagged("MINT")
    if len(profile) != 1 or any(profile[0].get(k) != v
                                for k, v in REQUIRED_PROFILE.items()):
        errors.append("Motor-off / Policy B build identity not verified")
    if len(i2c) != 1 or i2c[0].get("init_status") != "0" or (
        i2c[0].get("whoami") != "0x68" or i2c[0].get("selected") != "0"
        or i2c[0].get("scl_nominal") != "400000"
    ):
        errors.append("MPU I2C1 400kHz WHO_AM_I startup signature invalid")
    if len(cal) != 1 or any(cal[0].get(k) != v for k, v in
                            (("n","4000"),("retries","0"),("blocks","8"))):
        errors.append("Calibration evidence missing/mismatched")
    if any(tag in ("MINT_BOOT_FAIL", "MINT_CAL_FAIL") for tag, _ in d):
        errors.append("MCU reported boot/calibration failure")
    fresh = [x for x in frames if x.get("cal") == "1" and x.get("valid") == "1"]
    if len(fresh) < min_samples:
        errors.append(f"Insufficient valid postcal 1Hz MINT frames ({len(fresh)} < {min_samples})")

    def count(v, key):
        try:
            return int(v[key])
        except (KeyError, ValueError, TypeError):
            errors.append("Missing/non-integer field: " + key)
            return 0
    if fresh:
        seqs = [count(v,"seq") for v in fresh]
        if any(b != a+1 for a,b in zip(seqs,seqs[1:])):
            errors.append("Missing or duplicated telemetry sequences")
        for field in ("i2c_err","timeout","stale","timing_late","timing_gap","tx_drop"):
            if any(count(v,field) != 0 for v in fresh):
                errors.append("Nonzero " + field)
        for field in ("ticks","reused","aged1500","missing","good","fresh","sample_seq"):
            vals = [count(v,field) for v in fresh]
            if any(b < a for a,b in zip(vals,vals[1:])):
                errors.append("Counter decreased: " + field)
        ticks=count(fresh[-1],"ticks")-count(fresh[0],"ticks")
        reused=count(fresh[-1],"reused")-count(fresh[0],"reused")
        aged=count(fresh[-1],"aged1500")-count(fresh[0],"aged1500")
        expired=count(fresh[-1],"missing")-count(fresh[0],"missing")
        accepted=count(fresh[-1],"fresh")-count(fresh[0],"fresh")
        status_notready=count(fresh[-1],"notready")-count(fresh[0],"notready")
        if ticks <= 0:
            errors.append("No verified postcal 1kHz control ticks")
        reused_pct=round(reused*100/max(1,ticks),5)
        if reused_pct > 0.15:
            warnings.append("Reuse > 0.15%; investigate vs reference 0.0441%")
        if expired:
            warnings.append("Some 1kHz ticks had invalid/expired (>3ms) snapshots")
        if accepted <= 0:
            errors.append("No postcal accepted samples")
        metrics={"frames":len(fresh),"control_ticks":ticks,"reused":reused,
                 "reused_pct":reused_pct,"gyro_age_gt1500_ticks":aged,
                 "invalid_or_expired_ticks":expired,"fresh_gyro_reads":accepted,
                 "status_notready":status_notready,
                 "max_ISR_cycles_observed":count(fresh[-1],"timing_max_cycles")}
    else:
        metrics={}
    return {"status":"PASS_INTEGRATION_MOTOR_OFF" if not errors else "FAIL",
            "failures":sorted(set(errors)),"warnings":warnings,
            "metrics":metrics,"baseline_diagnostic_rc1":BASELINE,
            "note":"Standalone motor-OFF acceptance, not proof of production motion reliability or formal WCET."}


def check_source(repo: Path):
    env=(repo/"platformio.ini").read_text(encoding="utf-8")
    marker="[env:mpu_integration_policy_b_96m_trim25]"
    if marker not in env:
        return ["Missing dedicated integration build profile"]
    section=env.split(marker,1)[1].split("\n[env:",1)[0]
    errors=[]
    for flag in ("-DMPU_SAMPLING_POLICY_B=1","-DMPU_INTEGRATION_BENCH=1"):
        if flag not in section: errors.append("Missing: "+flag)
    for flag in ("MPU_FRESHNESS_DIAG","MPU_FRESHNESS_AB_B","CONTROL_LOAD_DIAG"):
        if flag in section: errors.append("Diagnostic flag in integration build: "+flag)
    h=(repo/"include/mpu_policy.h").read_text(encoding="utf-8")
    s=(repo/"src/mpu6050.c").read_text(encoding="utf-8")
    t=(repo/"src/system_timer.c").read_text(encoding="utf-8")
    for filename, contents, flag in (
        ("mpu_policy.h",h,"MPU_SAMPLING_POLICY_B"),
        ("mpu6050.c",s,"MPU_SAMPLING_POLICY_B"),
        ("system_timer.c",t,"MPU_SAMPLING_POLICY_B")):
        if flag not in contents: errors.append("Policy missing: "+filename)
    for symbol in ("MPU6050_GetSnapshot","published_valid","published_sequence"):
        if symbol not in s: errors.append("Snapshot missing: "+symbol)
    return errors


def self_test():
    def line(tag, **kv):
        return tag + "," + ",".join(f"{k}={v}" for k,v in kv.items())
    data=["[MINT_BOOT] MPU production sampling Policy B, motors disabled",
          line("MINT_PROFILE",**REQUIRED_PROFILE),
          line("MINT_I2C",whoami="0x68",selected=0,scl_nominal=400000,init_status=0),
          line("MINT_CAL_DONE",n=4000,retries=0,blocks=8,bias_mdps=0)]
    for k in range(151):
        data.append(line("MINT",seq=k+1,cal=1,valid=1,ticks=1000*k,
            reused=k//10,aged1500=k//10,age_max=800,missing=0,
            sample_seq=1000*k+1,good=1000*k+1,fresh=1000*k+1,
            stale=0,notready=0,i2c_err=0,timeout=0,busy=0,
            timing_late=0,timing_gap=0,timing_max_cycles=2100,tx_drop=0))
    assert analyze(data)["status"]=="PASS_INTEGRATION_MOTOR_OFF"
    assert analyze(data[:-100])["status"]=="FAIL"
    assert analyze([data[0],data[1].replace("motor_off=1","motor_off=0")]+data[2:])["status"]=="FAIL"
    assert analyze(data+["MINT_CAL_FAIL,retries=3"])["status"]=="FAIL"
    assert analyze(data+[data[0]])["status"]=="FAIL"
    print("SELF-TEST PASSED: policy attestation, calibration, timeout, completeness and safe rejects.")
    print("Only host-side synthetic data tested; no firmware compilation or hardware access.")


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    group=ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--folder",type=Path)
    group.add_argument("--port",help="COM6 / USB-TTL port to capture fresh integration run")
    group.add_argument("--self-test",action="store_true")
    group.add_argument("--source-check",action="store_true")
    ap.add_argument("--seconds",type=float,default=240.0)
    ap.add_argument("--baud",type=int,default=115200)
    args=ap.parse_args()
    if args.self_test:
        self_test()
    elif args.source_check:
        errors=check_source(Path(__file__).resolve().parents[1])
        print(json.dumps({"status":"PASS_STATIC_SOURCE_CHECK" if not errors else "FAIL",
                          "failures":errors},indent=2))
        if errors: raise SystemExit(1)
    else:
        if args.port:
            if args.seconds < 180:
                ap.error("Integration regression requires >=180 seconds of capture")
            try:
                import serial
            except ImportError:
                ap.error("Missing pyserial: python -m pip install pyserial")
            folder=Path("mpu_integration_logs")/("session_"+dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
            folder.mkdir(parents=True,exist_ok=True)
            raw=folder/"raw_serial.log"
            print("Open",args.port,"at",args.baud,"baud. Press STM32 RESET once after opening.")
            print("Motor-OFF only. Capturing",args.seconds,"s in",folder)
            try:
                with serial.Serial(args.port,args.baud,timeout=0.25) as ser:
                    ser.reset_input_buffer()
                    t0=time.perf_counter()
                    with raw.open("w",encoding="utf-8") as fh:
                        while time.perf_counter()-t0<args.seconds:
                            rx=ser.readline()
                            if not rx: continue
                            line=rx.decode("utf-8",errors="replace").strip()
                            fh.write(f"{time.perf_counter()-t0:.9f}\t{line}\n")
                            if line.startswith(("MINT_BOOT_FAIL","MINT_CAL_FAIL")):
                                print("MCU FAULT:",line)
            except serial.SerialException as exc:
                raise SystemExit("Serial capture failed: "+str(exc))
            except KeyboardInterrupt:
                print("Capture interrupted; processing available partial logs.")
        else:
            folder=args.folder
            raw=folder/"raw_serial.log"
        if not raw.exists(): raise SystemExit("Missing raw_serial.log")
        report=analyze(raw.read_text(encoding="utf-8",errors="replace").splitlines())
        (folder/"mpu_integration_report.json").write_text(
            json.dumps(report,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(report,indent=2))
        print("Output:",folder)
        if report["status"]!="PASS_INTEGRATION_MOTOR_OFF": raise SystemExit(1)


if __name__=="__main__": main()
