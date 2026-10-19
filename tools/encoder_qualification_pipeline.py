#!/usr/bin/env python3
"""Encoder all-in-one qualification. Safe default: no motor PWM and no flash.

Offline: native/static tests, build, review historical motor evidence.
Full: above plus read-only UART capture from verified motor-OFF firmware.
Motor-on: explicitly gated 33-stage existing OFF/ON benchmark (0..100% PWM).
All modes write new result directories and preserve prior evidence.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
GATES = (
    ("integration_audit", ["tools/system_integration_gate.py", "--audit"]),
    ("integration_negative", ["tools/system_integration_gate.py", "--self-test"]),
    ("encoder_probe_parser", ["tools/encoder_manual_probe.py", "--self-test"]),
    ("encoder_native", ["tools/encoder_native_test.py"]),
    ("pipeline_regression", ["tools/test_encoder_qualification_pipeline.py"]),
    ("estimator_selftest", ["tools/encoder_estimator_benchmark.py", "--self-test"]),
    ("motor_driver_native", ["tools/motor_driver_native_test.py"]),
    ("sweep_native", ["tools/system_sweep_native_test.py"]),
    ("ir_native", ["tools/ir_sensor_native_test.py"]),
    ("ir_parser", ["tools/test_ir_sensor_timing_capture.py"]),
)
HEALTH = ("SYS_E", "SYS_M", "SYS_T", "SYS_U")

def parse(line):
    head, *parts = line.strip().split(",")
    if not head.startswith(("SYS_", "CMD_", "CLOCK_")):
        return None
    result = {"type": head}
    for part in parts:
        key, sep, val = part.partition("=")
        if not sep or not key or key in result:
            return None
        try:
            result[key] = int(val)
        except ValueError:
            result[key] = val
    return result

def modulo_delta(new, old):
    return ((new - old + 32768) & 0xFFFF) - 32768

def self_test():
    assert modulo_delta(5, 65530) == 11
    assert modulo_delta(65530, 5) == -11
    assert modulo_delta(21, 20) == 1
    assert parse("SYS_E,ms=4,encL=-3,encR=5")["encL"] == -3
    assert parse("BAD,encL=7") is None
    return {"status": "PASS", "rollover_cases": 3, "parser_cases": 2}

def git_state():
    def git(*args):
        try:
            p = subprocess.run(["git", *args], cwd=ROOT, capture_output=True,
                               text=True, check=False, timeout=10)
            return p.stdout.strip() if p.returncode == 0 else "UNKNOWN"
        except (OSError, subprocess.TimeoutExpired):
            return "UNKNOWN"
    return {"branch": git("branch","--show-current"),
            "commit": git("rev-parse","HEAD"),
            "tracked_dirty": bool(git("status","--porcelain","--untracked-files=no"))}

def source_audit():
    enc = (ROOT/"src/encoder.c").read_text(encoding="utf-8")
    ctrl = (ROOT/"src/control.c").read_text(encoding="utf-8")
    cfg = (ROOT/"platformio.ini").read_text(encoding="utf-8")
    main = (ROOT/"src/main.c").read_text(encoding="utf-8")
    checks = {
        "quadrature_x4": "(3UL << 0)" in enc,
        "counter_16bit": "tim->ARR = 0xFFFFU;" in enc,
        "encoder_on_tim11": "Encoder_Update(CONTROL_DT_S);" in ctrl,
        "motor_off_flag": "-DINTEGRATION_DIAG_MOTOR_OFF=1" in cfg,
        "encoder_motor_off_profile": "integration_diag_encoder_motor_off_96m" in cfg and
            "-DENCODER_PROFILE_DIAG=1" in cfg,
        "encoder_telemetry_locked": "Encoder high-rate diagnostic requires" in main,
        "motor_off_sticky_lock": "Motor_LockOutputs();" in main,
        "positive_forward_unchanged": "ENCODER_LEFT_DIRECTION  -1" in
            (ROOT/"include/encoder.h").read_text(encoding="utf-8"),
    }
    return {"checks":checks, "pass":all(checks.values()),
            "limits": [
                "No explicit TIM3/TIM4 IC1F/IC2F hardware glitch filter configuration.",
                "Raw PPS from one-millisecond delta: quantization 1000 counts/s.",
                "EMA alpha 0.30 at 1 kHz is not equivalent to encoder electrical filtering.",
                "1 Hz SYS_E cannot detect high-rate glitches or independently prove zero missed edges.",
            ]}

def run(command, label, directory, timeout=300):
    path = directory/(label+".log")
    print("RUN:",label,flush=True)
    try:
        with path.open("x",encoding="utf-8") as log:
            cp = subprocess.run(command, cwd=ROOT, stdout=log,
                                stderr=subprocess.STDOUT, timeout=timeout,
                                check=False)
        return {"pass":cp.returncode==0, "returncode":cp.returncode,"log":path.name}
    except (OSError,subprocess.TimeoutExpired) as ex:
        with path.open("a",encoding="utf-8") as log:
            log.write(str(ex))
        return {"pass":False,"error":str(ex),"log":path.name}

def gates(directory, skip_build):
    d={"git":git_state(),"source":source_audit(),
       "self_test":self_test(),"checks":{}}
    for name,args in GATES:
        d["checks"][name]=run([sys.executable,*args],name,directory,180)
    d["checks"]["estimator_study"]=run(
        [sys.executable,"tools/encoder_estimator_benchmark.py",
         "--outdir",str(directory)],"estimator_study",directory,90)
    if not skip_build:
        d["checks"]["build_locked"]=run(
            [sys.executable,"-m","platformio","run","-e",
             "integration_diag_encoder_motor_off_96m"],"build_locked",directory,420)
    d["pass"]=d["source"]["pass"] and all(
        v["pass"] for v in d["checks"].values())
    return d

def summarize_encoder(records, directory, title="locked_capture"):
    by={}
    for x in records:
        if x is not None:
            by.setdefault(x["type"],[]).append(x)
    e=[x for x in by.get("SYS_E",[]) if all(
        isinstance(x.get(k),int) for k in
        ("ms","encL","encR","ppsL","ppsR","appL","appR"))]
    if len(e)<2:
        raise RuntimeError("Need at least 2 complete SYS_E records")
    if any(x["appL"]!=0 or x["appR"]!=0 for x in e):
        raise RuntimeError("Unexpected nonzero applied motor command in locked capture")
    first=e[0]
    rows=[]
    for x in e:
        rows.append({"ms":x["ms"],
                     "countL":x["encL"],"countR":x["encR"],
                     "relativeL":x["encL"]-first["encL"],
                     "relativeR":x["encR"]-first["encR"],
                     "ppsL":x["ppsL"],"ppsR":x["ppsR"]})
    with (directory/(title+".csv")).open("x",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    def diff(key,typ):
        arr=by.get(typ,[])
        if len(arr)<2 or not isinstance(arr[0].get(key),int) or not isinstance(arr[-1].get(key),int):
            return None
        return (arr[-1][key]-arr[0][key]) & 0xFFFFFFFF
    wheels={}
    for side in ("L","R"):
        diffs=[e[i]["enc"+side]-e[i-1]["enc"+side] for i in range(1,len(e))]
        wheels[side]={"net_counts":e[-1]["enc"+side]-first["enc"+side],
                      "nonzero_1s_windows":sum(x!=0 for x in diffs),
                      "max_abs_1s_delta":max(abs(x) for x in diffs),
                      "last_filtered_pps":e[-1]["pps"+side]}
    fast=[x for x in by.get("SYS_EN",[]) if all(
        isinstance(x.get(k),int) for k in ("ms","l","r","vL","vR"))]
    fast_stats={"status":"NOT_MEASURED","reason":"Encoder diagnostic profile not active"}
    if len(fast)>=3:
        samples=[]
        for i,x in enumerate(fast):
            previous=fast[i-1] if i else x
            delta_ms=x["ms"]-previous["ms"]
            dL=x["l"]-previous["l"]
            dR=x["r"]-previous["r"]
            samples.append({
                "ms":x["ms"],"countL":x["l"],"countR":x["r"],
                "delta_ms":delta_ms,"deltaL":dL,"deltaR":dR,
                "pps_20ms_L":round(dL*1000/delta_ms,2) if delta_ms>0 else 0,
                "pps_20ms_R":round(dR*1000/delta_ms,2) if delta_ms>0 else 0,
                "pps_ema_L":x["vL"],"pps_ema_R":x["vR"],
            })
        with (directory/"encoder_50hz.csv").open("x",newline="",encoding="utf-8") as f:
            writer=csv.DictWriter(f,fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
        gaps=[x["delta_ms"] for x in samples[1:] if x["delta_ms"]>0]
        mismatch={}
        for side in ("L","R"):
            diffs=[abs(x["pps_20ms_"+side]-x["pps_ema_"+side])
                   for x in samples[1:] if x["delta_ms"]>0]
            mismatch[side]={
                "mean_abs_ema_vs_window_pps":round(statistics.mean(diffs),2) if diffs else None,
                "max_abs_ema_vs_window_pps":round(max(diffs),2) if diffs else None,
            }
        fast_stats={"status":"OBSERVED","samples":len(fast),
                    "median_interval_ms":statistics.median(gaps) if gaps else None,
                    "max_interval_ms":max(gaps) if gaps else None,
                    "intervals_over_40ms":sum(x>40 for x in gaps),
                    "pps_comparison":mismatch,
                    "limitations":"50Hz is decimated; these differences are not solely measurement noise or proof of missed pulses."}
    return {"high_rate_encoder":fast_stats,
            "frame_counts":{k:len(v) for k,v in by.items()},
            "seconds_between_encoder_frames":(e[-1]["ms"]-e[0]["ms"])/1000,
            "wheels":wheels, "tim11_late_delta":diff("late","SYS_T"),
            "tim11_gap_delta":diff("gap","SYS_T"),
            "mpu_i2c_error_delta":diff("i2cerr","SYS_M"),
            "mpu_timeout_delta":diff("to","SYS_M"),
            "uart_tx_drop_delta":diff("tx_drop","SYS_U"),
            "motor_lock_frames":len(by.get("SYS_M",[])),
            "all_reported_locked":bool(by.get("SYS_M")) and
                all(x.get("locked")==1 for x in by["SYS_M"]),
            "note":"1Hz snapshots do not qualify high-rate PPS jitter or missed edges."}

def serial_capture(port,seconds,directory):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial missing; install with python -m pip install pyserial") from exc
    out=directory/"uart_raw_motor_off.log"
    records=[]
    confirmed_e=confirmed_lock=False
    begin=time.monotonic()
    with serial.Serial(port,115200,timeout=0.3) as uart, out.open("x",encoding="utf-8") as file:
        while time.monotonic()-begin<seconds:
            raw=uart.readline()
            if not raw:
                continue
            line=raw.decode("ascii",errors="replace").rstrip("\r\n")
            file.write(line+"\n")
            row=parse(line)
            if row is None:
                continue
            records.append(row)
            if row["type"]=="SYS_M":
                if row.get("locked")!=1:
                    raise RuntimeError("ABORT: SYS_M shows unlocked motor driver")
                confirmed_lock=True
            if row["type"]=="SYS_E":
                if row.get("appL")!=0 or row.get("appR")!=0:
                    raise RuntimeError("ABORT: SYS_E shows motor command")
                confirmed_e=True
            if time.monotonic()-begin>6 and not (confirmed_e and confirmed_lock):
                raise RuntimeError("Motor lock / encoder telemetry unverified within 6s")
    if not (confirmed_e and confirmed_lock):
        raise RuntimeError("Incomplete motor-off telemetry")
    return summarize_encoder(records,directory)

def sweep_summary(path,directory,tag="historical"):
    if not path.exists():
        return {"status":"NOT_AVAILABLE","source":str(path)}
    info=json.loads(path.read_text(encoding="utf-8"))
    rows=[]
    for mode in info.get("results",[]):
        for stage in mode.get("extended",{}).get("stages",[]):
            duration=stage.get("drive_ms",0)
            if not duration:
                continue
            l=stage.get("drive_left_counts",0)
            r=stage.get("drive_right_counts",0)
            rows.append({"mode":mode.get("mode"),"group":stage.get("wheel_group"),
                         "pwm_percent":stage.get("pwm_percent"),
                         "duration_ms":duration,"countL":l,"countR":r,
                         "mean_pps_L":round(1000*l/duration,2),
                         "mean_pps_R":round(1000*r/duration,2),
                         "coastL":stage.get("coast_left_counts"),
                         "coastR":stage.get("coast_right_counts")})
    if rows:
        with (directory/(f"sweep_encoder_stages_{tag}.csv")).open("x",encoding="utf-8",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    both=[x for x in rows if x["mode"]=="ON" and x["group"]==2 and
          x["pwm_percent"]>=20 and x["countL"]>0 and x["countR"]>0]
    mismatch=[abs(x["countL"]-x["countR"])*200/(x["countL"]+x["countR"]) for x in both]
    return {"status":"REVIEWED" if rows else "EMPTY","source":str(path),
            "stage_count":len(rows),"motor_on_stages":sum(x["mode"]=="ON" for x in rows),
            "both_wheels_mismatch_mean_pct":round(statistics.mean(mismatch),2)
                if mismatch else None,
            "qualified_electrically":False,
            "note":"Stage-average counts only; not 1ms dynamics or ground-truth RPM."}

def render_plots(directory):
    """Optional matplotlib plots; never affects pass/fail."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return "SKIP: matplotlib not installed (CSVs are still available)"
    saved=[]
    csvpath=directory/"encoder_50hz.csv"
    if not csvpath.exists():
        csvpath=directory/"locked_capture.csv"
    if csvpath.exists():
        with csvpath.open(newline="",encoding="utf-8") as file:
            rows=list(csv.DictReader(file))
        if rows:
            t=[(int(x["ms"])-int(rows[0]["ms"]))/1000 for x in rows]
            plt.figure(figsize=(10,4))
            keys=("countL","countR")
            for key in keys:
                plt.plot(t,[int(x[key]) for x in rows],label=key)
            plt.legend();plt.grid(alpha=.25);plt.xlabel("t (s)")
            plt.ylabel("Decoded timer counts");plt.title("Encoder counters")
            plt.tight_layout();plt.savefig(directory/"encoder_counts.png",dpi=150)
            plt.close();saved.append("encoder_counts.png")
            vel=("pps_20ms_L","pps_20ms_R","pps_ema_L","pps_ema_R") if "pps_20ms_L" in rows[0] else ("ppsL","ppsR")
            plt.figure(figsize=(10,4))
            for key in vel:
                plt.plot(t,[float(x[key]) for x in rows],label=key,linewidth=1.1)
            plt.legend();plt.grid(alpha=.25);plt.xlabel("t (s)")
            plt.ylabel("counts / second");plt.title("Measured window-PPS vs firmware EMA")
            plt.tight_layout();plt.savefig(directory/"encoder_speed.png",dpi=150)
            plt.close();saved.append("encoder_speed.png")
    for name in ("sweep_encoder_stages_motor_on.csv","sweep_encoder_stages_historical.csv"):
        path=directory/name
        if not path.exists():
            continue
        with path.open(newline="",encoding="utf-8") as file:
            rows=list(csv.DictReader(file))
        both=[x for x in rows if x["mode"]=="ON" and x["group"]=="2"]
        if not both:
            continue
        plt.figure(figsize=(10,4))
        duty=[int(x["pwm_percent"]) for x in both]
        for key in ("mean_pps_L","mean_pps_R"):
            plt.plot(duty,[float(x[key]) for x in both],marker="o",label=key)
        plt.legend();plt.grid(alpha=.25);plt.xlabel("Commanded PWM (%)")
        plt.ylabel("Mean counts / second during drive")
        plt.title("Both wheels (stage-averaged encoder counts, not RPM ground truth)")
        plt.tight_layout();plt.savefig(directory/"sweep_response.png",dpi=150)
        plt.close();saved.append("sweep_response.png")
        break
    return saved

def report(directory,data):
    lines=["# Encoder unified pipeline report","",
           "Firmware changes: none by this pipeline.",
           "Safety: MOTOR_ON only by explicit multi-flag opt-in; includes 100% PWM.",
           "Git branch: "+data["software"]["git"]["branch"],
           "Git commit: "+data["software"]["git"]["commit"],
           "",
           "## Software gates"]
    for name,item in data["software"]["checks"].items():
        lines.append("- "+name+": "+("PASS" if item["pass"] else "FAIL"))
    lines+=["","## Static code audit"]
    lines.extend("- "+x for x in data["software"]["source"]["limits"])
    lines+=["","## Hardware / sweep",json.dumps(data.get("hardware"),indent=2,ensure_ascii=False),
            json.dumps(data.get("sweep"),indent=2,ensure_ascii=False),
            "","## Remaining limitations",
            "- Input capture filter IC1F/IC2F and 10-ms speed estimator remain candidates, NOT auto-applied.",
            "- In encoder motor-off profile SYS_EN is ~50 Hz; remaining high-frequency glitches/missed edges require per-tick samples and independent reference.",
            "- estimator_metrics.csv and estimator_samples.csv (if present) are SYNTHETIC, NOT measured MCU response.",
            "- 1430 counts per wheel revolution is retained as the user's current engineering approximation.",
            "- Historical stage response is not a calibration of closed-loop speed PID."]
    (directory/"REPORT.md").write_text("\n".join(lines)+"\n",encoding="utf-8")

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode",choices=("offline","capture","full"),default="full")
    p.add_argument("--port",help="USART1 adapter such as COM7")
    p.add_argument("--seconds",type=float,default=90)
    p.add_argument("--skip-build",action="store_true")
    p.add_argument("--outdir",type=Path)
    p.add_argument("--review-summary",type=Path)
    p.add_argument("--flash-motor-off",action="store_true")
    p.add_argument("--confirm-vm-disconnected",action="store_true")
    p.add_argument("--motor-on-sweep",action="store_true",
                   help="DANGER: actual 0..100%% PWM OFF/ON sweep")
    for flag in ("lifted","cutoff","high-pwm","operator-present"):
        p.add_argument("--confirm-"+flag,action="store_true")
    p.add_argument("--self-test",action="store_true")
    args=p.parse_args(argv)
    if args.self_test:
        print(json.dumps(self_test(),indent=2))
        return 0
    if args.seconds<10:
        p.error("--seconds must be >=10")
    if args.mode=="offline" and (args.port or args.motor_on_sweep or args.flash_motor_off):
        p.error("offline cannot use port, flash or motor")
    if args.mode=="capture" and (args.motor_on_sweep or args.flash_motor_off):
        p.error("capture is read-only; use full for flashing or sweep")
    if args.mode!="offline" and not args.port:
        p.error("--port required")
    if args.motor_on_sweep:
        if args.mode!="full" or args.flash_motor_off or args.confirm_vm_disconnected:
            p.error("motor-on sweep requires full without motor-off options")
        if not all((args.confirm_lifted,args.confirm_cutoff,
                    args.confirm_high_pwm,args.confirm_operator_present)):
            p.error("DANGER: motor-on requires 4 explicit --confirm-* flags")
    elif args.mode!="offline" and not args.confirm_vm_disconnected:
        p.error("motor-off serial capture/flash requires --confirm-vm-disconnected")
    path=args.outdir or ROOT/"results"/("encoder_qual_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    path.mkdir(parents=True,exist_ok=False)
    data={"created":datetime.now().isoformat(timespec="seconds"),
          "mode":args.mode,"software":None,
          "hardware":{"status":"NOT_RUN"},"sweep":None}
    try:
        if args.mode=="capture":
            a=source_audit()
            data["software"]={"git":git_state(),"source":a,"self_test":self_test(),
                              "checks":{},"pass":a["pass"]}
        else:
            data["software"]=gates(path,args.skip_build)
        source=args.review_summary or ROOT/"results/system_uart_20261019_104529/summary.json"
        data["sweep"]=sweep_summary(source,path)
        if not data["software"]["pass"]:
            raise RuntimeError("Software gates failed; no hardware actions performed")
        if args.motor_on_sweep:
            if data["software"]["git"]["tracked_dirty"]:
                raise RuntimeError("Tracked tree modified; motor-on flash blocked")
            print("MOTOR-ON: REAL PWM 0..100%, 33 stages x OFF/ON; operator must remain present.")
            out=path/"system_uart_hardware"
            try:
                runinfo=run([sys.executable,"tools/system_uart_pipeline.py",
                             "--port",args.port,"--flash","--motor-on",
                             "--outdir",str(out)],"motor_on_automated",path,950)
                data["hardware"]={"status":"PASS" if runinfo["pass"] else "FAIL",
                                  "execution":runinfo}
                new=out/"summary.json"
                if new.exists():
                    data["sweep"]=sweep_summary(new,path,"motor_on")
                if not runinfo["pass"]:
                    raise RuntimeError("Motor sweep failed; check raw logs and STOP state")
            finally:
                # Attempt restoration even when a motor-on test fails or is interrupted.
                restored=run([sys.executable,"-m","platformio","run","-e",
                              "integration_diag_encoder_motor_off_96m","-t","upload"],
                             "restore_motor_off",path,360)
                data["restored_motor_off"]=restored
                if not restored["pass"]:
                    data["error"]="Motor-off restoration failed; independently disconnect VM and verify firmware"
                    print("WARNING: motor-off restoration failed; disconnect VM and verify firmware.",
                          file=sys.stderr)

        elif args.mode!="offline":
            if args.flash_motor_off:
                if data["software"]["git"]["tracked_dirty"]:
                    raise RuntimeError("Tracked tree modified; motor-off flashing blocked")
                f=run([sys.executable,"-m","platformio","run","-e",
                       "integration_diag_encoder_motor_off_96m","-t","upload"],
                      "motor_off_upload",path,360)
                data["upload"]=f
                if not f["pass"]:
                    raise RuntimeError("Motor-off firmware upload failed")
            data["hardware"]=serial_capture(args.port,args.seconds,path)
    except (RuntimeError,OSError,ValueError) as ex:
        data["error"]=str(ex)
        data["hardware"]={"status":"ABORTED_OR_INCOMPLETE","details":str(ex)}
        print("PIPELINE FAILED:",ex,file=sys.stderr)
    finally:
        (path/"summary.json").write_text(
            json.dumps(data,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
        report(path,data)
    try:
        data["plots"]=render_plots(path)
    except (OSError,ValueError,RuntimeError) as ex:
        data["plots"]="SKIP: "+str(ex)
    (path/"summary.json").write_text(
        json.dumps(data,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    report(path,data)
    print("Results:",path)
    return 2 if "error" in data else 0

if __name__=="__main__":
    raise SystemExit(main())
