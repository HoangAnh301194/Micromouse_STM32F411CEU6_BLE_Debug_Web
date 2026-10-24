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
    ("shadow_ab_regression", ["tools/test_encoder_shadow_ab.py"]),
    ("encoder_native", ["tests/regression/encoder_native_test.py"]),
    ("pipeline_regression", ["tools/test_encoder_qualification_pipeline.py"]),
    ("estimator_selftest", ["tools/encoder_estimator_benchmark.py", "--self-test"]),
    ("motor_driver_native", ["tests/regression/motor_driver_native_test.py"]),
    ("sweep_native", ["tests/regression/system_sweep_native_test.py"]),
    ("ir_native", ["tests/regression/ir_sensor_native_test.py"]),
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
        "encoder_telemetry_diagnostic_guard": "Encoder telemetry requires an explicit integration diagnostic mode" in main,
        "encoder_motor_on_profile": "integration_diag_encoder_motor_on_96m" in cfg,
        "shadow_alpha_optin": "-DENCODER_ALPHA_AB_DIAG=1" in cfg and
            "Encoder_GetShadowSnapshot" in enc and "SYS_AM,ms=" in main,
        "shadow_ema_offloaded_from_irq": (
            "ShadowEnqueue((int16_t)" in enc and
            "void Encoder_ShadowService" not in enc and
            "uint16_t Encoder_ShadowService" in enc and
            "Encoder_ShadowService(12U)" in main and
            "SYS_AQ,ms=" in main),

        "i2c_boot_probe_logged": "SYS_IPB,ms=" in main,
        "i2c_busy_status_logged": "SYS_IB,ms=" in main and
            "I2C_GetBusyDiag" in (ROOT/"src/i2c.c").read_text(encoding="utf-8"),
        "motor_off_sticky_lock": "Motor_LockOutputs();" in main,
        "positive_forward_unchanged": "ENCODER_LEFT_DIRECTION  -1" in
            (ROOT/"include/encoder.h").read_text(encoding="utf-8"),
    }
    return {"checks":checks, "pass":all(checks.values()),
            "limits": [
                "No explicit TIM3/TIM4 IC1F/IC2F hardware glitch filter configuration.",
                "Raw PPS from one-millisecond delta: quantization 1000 counts/s.",
                "EMA alpha 0.30 at 1 kHz is not equivalent to encoder electrical filtering.",
                "50 Hz SYS_EN during motor OFF/ON resolves short-term PPS trends but cannot prove zero missed edges.",
            ]}

def run(command, label, directory, timeout=300, live_output=False):
    path = directory/(label+".log")
    print("RUN:",label,flush=True)
    try:
        with path.open("x",encoding="utf-8") as log:
            if not live_output:
                cp = subprocess.run(command, cwd=ROOT, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=timeout,
                                    check=False)
                code=cp.returncode
            else:
                import threading
                proc=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT,text=True,
                                      bufsize=1,errors="replace")
                def relay():
                    for line in proc.stdout:
                        log.write(line)
                        log.flush()
                        print(line.rstrip(),flush=True)
                worker=threading.Thread(target=relay,daemon=True)
                worker.start()
                try:
                    code=proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                    raise
                finally:
                    worker.join(timeout=5)
        outcome={"pass":code==0, "returncode":code,"log":path.name}
        if code:
            # The full compiler log remains on disk. Surface a bounded
            # diagnostic excerpt to the user and persist it in summary.json.
            entries=path.read_text(encoding="utf-8",errors="replace").splitlines()
            anchors=[i for i,line in enumerate(entries)
                     if any(token in line.lower() for token in
                            ("error:", "fatal error:", "failed", "exception",
                             "undefined reference", "configurationerror"))]
            chosen=set()
            for idx in anchors[-8:]:
                chosen.update(range(max(0,idx-2),min(len(entries),idx+3)))
            chosen.update(range(max(0,len(entries)-18),len(entries)))
            excerpt="\n".join(entries[i] for i in sorted(chosen))
            outcome["diagnostic_excerpt"]=excerpt[-6000:]
            print("GATE FAILED:",label,"(exit",code,")",flush=True)
            print(outcome["diagnostic_excerpt"],flush=True)
        return outcome
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
        d["checks"]["build_encoder_on"]=run(
            [sys.executable,"-m","platformio","run","-e",
             "integration_diag_encoder_motor_on_96m"],"build_encoder_on",directory,420)
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

def analyze_sweep_telemetry(run_directory, report_directory):
    """Turn existing OFF/ON raw UART log into aligned 50-Hz encoder evidence.

    SYS_STEP transitions tag subsequent SYS_EN samples with drive/coast stage.
    A 20-ms PPS estimate is valid only when both frames share the same stage
    and state; crossing a transition is intentionally marked unavailable.
    """
    reports={}
    for phase in ("OFF", "ON"):
        logfile=run_directory/(phase.lower()+"_uart.log")
        if not logfile.exists():
            raise RuntimeError("Missing raw "+phase+" UART log; cannot analyze new motor capture")
        records=[]
        current={"stage":-1,"state":0,"group":-1,"pwm":0,"since":None}
        all_records=[]
        with logfile.open(encoding="utf-8",errors="replace") as handle:
            for line in handle:
                _, tab, frame=line.partition("\t")
                data=parse(frame if tab else line)
                if data is None:
                    continue
                all_records.append(data)
                if data["type"]=="SYS_STEP":
                    current={"stage":data.get("id",-1),
                             "state":data.get("state",-1),
                             "group":data.get("group",-1),
                             "pwm":data.get("duty",0)//10,
                             "since":data.get("ms")}
                if data["type"]!="SYS_EN":
                    continue
                if not all(isinstance(data.get(k),int)
                           for k in ("ms","l","r","vL","vR")):
                    continue
                item={"ms":data["ms"],"countL":data["l"],"countR":data["r"],
                      "emaL":data["vL"],"emaR":data["vR"],
                      "stage":current["stage"],"state":current["state"],
                      "group":current["group"],"pwm_percent":current["pwm"],
                      "since_stage_ms":(data["ms"]-current["since"]
                                        if current["since"] is not None else -1)}
                prior=records[-1] if records else None
                same_segment=(prior is not None and
                              (item["stage"],item["state"]) ==
                              (prior["stage"],prior["state"]))
                dt=item["ms"]-prior["ms"] if same_segment else 0
                item["dt_ms"]=dt
                for wheel in ("L","R"):
                    item["delta"+wheel]=(item["count"+wheel]-prior["count"+wheel]
                                         if dt>0 else "")
                    item["pps"+wheel]=(round(item["delta"+wheel]*1000/dt,2)
                                       if dt>0 else "")
                records.append(item)
        if len(records)<100:
            raise RuntimeError(phase+": fewer than 100 complete 50-Hz SYS_EN samples")
        with (report_directory/("encoder_motor_"+phase.lower()+"_50hz.csv")).open(
                "x",newline="",encoding="utf-8") as output:
            writer=csv.DictWriter(output,fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
        drive=[x for x in records if x["state"]==1 and
               x["since_stage_ms"]>=300 and x["pwm_percent"]>=20
               and x["dt_ms"]>0]
        results=[]
        for wheel in ("L","R"):
            values=[float(x["pps"+wheel]) for x in drive
                    if x["pps"+wheel]!=""]
            residuals=[abs(float(x["pps"+wheel])-x["ema"+wheel])
                       for x in drive if x["pps"+wheel]!=""]
            results.append({"wheel":wheel,"steady_drive_samples":len(values),
                            "mean_20ms_pps":round(statistics.mean(values),2)
                                if values else None,
                            "std_20ms_pps":round(statistics.pstdev(values),2)
                                if len(values)>1 else None,
                            "mae_pps_vs_1ms_ema":round(statistics.mean(residuals),2)
                                if residuals else None})
        def counter_delta(kind,key):
            d=[x for x in all_records if x["type"]==kind and
               isinstance(x.get(key),int)]
            return ((d[-1][key]-d[0][key]) & 0xFFFFFFFF) if len(d)>=2 else None
        health={"tim11_late_delta":counter_delta("SYS_T","late"),
                "tim11_gap_delta":counter_delta("SYS_T","gap"),
                "uart_tx_drop_delta":counter_delta("SYS_U","tx_drop"),
                 "uart_rx_error_delta":counter_delta("SYS_U","rx_err"),
                 "uart_rx_overflow_delta":counter_delta("SYS_U","rx_ov"),
                "uart_dma_error_delta":counter_delta("SYS_U","dma_err"),
                "mpu_i2c_error_delta":counter_delta("SYS_M","i2cerr"),
                "mpu_timeout_delta":counter_delta("SYS_M","to")}
        rx_details={key:counter_delta("SYS_UR",key)
                    for key in ("ore","fe","ne")}
        shadow_queue_entries=[x for x in all_records if x["type"]=="SYS_AQ"]
        shadow_queue=shadow_queue_entries[-1] if shadow_queue_entries else None
        shadow_queue_bad=bool(shadow_queue and (
            shadow_queue.get("lost",0)!=0 or shadow_queue.get("edge",0)!=0))
        status="PASS"
        if (shadow_queue_bad or any(v is None for v in health.values())
                or any(v!=0 for v in health.values())):
            status="FAIL_HEALTH"
        if not drive or any(x["steady_drive_samples"]<50 for x in results):
            status="INSUFFICIENT_DRIVE_SAMPLES"
        if phase=="ON":
            positive=[x for x in drive if x["pwm_percent"]>=40 and
                      ((x["group"] in (0,2) and x["ppsL"]!="" and x["ppsL"]>0) or
                       (x["group"] in (1,2) and x["ppsR"]!="" and x["ppsR"]>0))]
            if len(positive)<50:
                status="NO_ENCODER_RESPONSE"
        probe_entries=[x for x in all_records if x["type"]=="SYS_IPB" and
                       isinstance(x.get("idx"),int)]
        startup_probe=None
        if probe_entries:
            q=probe_entries[-1]
            startup_probe={
                "selected_speed_index":q["idx"],
                "selected_speed_mode":q.get("mode"),
                "configured_scl_hz":next((
                    x["iclk"] for x in reversed(all_records)
                    if x["type"]=="SYS_U" and isinstance(x.get("iclk"),int)
                ),None),
                "attempt_400k":{"address_status":q.get("a0"),
                                "who_read_status":q.get("w0"),
                                "who_am_i":q.get("id0")},
                "attempt_300k":{"address_status":q.get("a1"),
                                "who_read_status":q.get("w1"),
                                "who_am_i":q.get("id1")},
                "fallback_occurred":q["idx"]>0,
                "explanation":"Status 255 means unattempted. Probe status 0 indicates driver success; WHO_AM_I must equal 0x68. The configured clock is nominal, not an SCL oscilloscope measurement.",
            }
        busy_samples=[x for x in all_records if x["type"]=="SYS_IB"]
        busy_diag=None
        if busy_samples:
            d=busy_samples[-1]
            pins=d.get("pin",-1)
            after=d.get("after",-1)
            busy_diag={"busy_timeouts":d.get("n"),
                       "bus_recoveries":d.get("rec"),
                       "sr1_at_last_timeout":d.get("sr1"),
                       "sr2_at_last_timeout":d.get("sr2"),
                       "PB8_SCL_high_at_timeout":bool(pins & 1) if pins>=0 else None,
                       "PB9_SDA_high_at_timeout":bool(pins & 2) if pins>=0 else None,
                       "PB8_SCL_high_after_recovery":bool(after & 1) if after>=0 else None,
                       "PB9_SDA_high_after_recovery":bool(after & 2) if after>=0 else None,
                       "caveat":"GPIO input register sampled during/after recovery, not an oscilloscope waveform."}
        reports[phase]={"status":status,"samples":len(records),
                         "steady_drive_samples":len(drive),"wheels":results,
                         "i2c_startup_probe":startup_probe,
                         "i2c_busy_debug":busy_diag,
                         "health":health,
                         "uart_rx_subtypes":rx_details,
                         "shadow_queue":shadow_queue,
                         "shadow_queue_bad":shadow_queue_bad,
                         "note":"PWM-to-count response, not an independent proof of encoder accuracy/no missed edges."}
    statuses={k:v["status"] for k,v in reports.items()}
    if any(v not in ("PASS","FAIL_HEALTH") for v in statuses.values()):
        raise RuntimeError("Encoder motor telemetry incomplete or no response: "+
                           repr(statuses))
    # Health failures remain visible but MUST NOT suppress the independent
    # alpha A/B analysis of collected 1-kHz evidence.
    return reports


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
    for phase in ("off","on"):
        path=directory/("encoder_motor_"+phase+"_50hz.csv")
        if not path.exists():
            continue
        with path.open(newline="",encoding="utf-8") as file:
            rows=list(csv.DictReader(file))
        if not rows:
            continue
        t=[(int(x["ms"])-int(rows[0]["ms"]))/1000 for x in rows]
        fig=plt.figure(figsize=(11,4))
        for key in ("ppsL","ppsR","emaL","emaR"):
            data=[float(x[key]) if x[key] else float("nan") for x in rows]
            plt.plot(t,data,label=key,linewidth=0.9)
        plt.title("Encoder "+phase.upper()+" • 20ms PPS versus firmware EMA")
        plt.xlabel("Time (seconds)");plt.ylabel("counts/second")
        plt.grid(alpha=.25);plt.legend(ncol=4);plt.tight_layout()
        png="encoder_motor_"+phase+"_speed.png"
        fig.savefig(directory/png,dpi=150)
        plt.close(fig)
        saved.append(png)
    alpha_csv=directory/"encoder_alpha_samples_10hz.csv"
    if alpha_csv.exists():
        with alpha_csv.open(newline="",encoding="utf-8") as file:
            alpha_samples=list(csv.DictReader(file))
        for wheel in ("L","R"):
            # Stage 28 is BOTH wheels at 60% PWM, 3.5s DRIVE.
            section=[x for x in alpha_samples if x["phase"]=="ON" and
                     x.get("id")=="28" and x.get("state")=="1" and
                     x["wheel"]==wheel]
            if len(section)<8:
                continue
            t=[(int(x["ms"])-int(section[0]["ms"]))/1000 for x in section]
            fig=plt.figure(figsize=(10,4))
            for key in ("a05","a08","a10","a12","a15",
                        "a20","a30","a40","a60","a80","b20"):
                if key not in section[0] or not all(x.get(key) for x in section):
                    continue
                plt.plot(t,[float(x[key]) for x in section],label=key,
                         linewidth=1.0)
            plt.title("Shadow EMA alpha A/B: wheel "+wheel+" at PWM 60%")
            plt.xlabel("Stage elapsed time (s)")
            plt.ylabel("Encoder counts/second")
            plt.grid(alpha=.25);plt.legend(ncol=4);plt.tight_layout()
            png="encoder_alpha_overlay_60pct_"+wheel+".png"
            fig.savefig(directory/png,dpi=150)
            plt.close(fig)
            saved.append(png)
    alpha_report=directory/"encoder_alpha_ab.json"
    if alpha_report.exists():
        ranking=json.loads(alpha_report.read_text(encoding="utf-8")).get("ranking",[])
        if ranking:
            fig=plt.figure(figsize=(9,4))
            plt.bar([str(x["alpha"]) for x in ranking],
                    [x["heuristic_score_vs_alpha_0_30"] for x in ranking])
            plt.title("Diagnostic proxy score versus alpha=0.30 (NOT true RPM accuracy)")
            plt.xlabel("Shadow EMA alpha")
            plt.ylabel("Relative weighted score (lower is better)")
            plt.tight_layout()
            png="encoder_alpha_ranking.png"
            fig.savefig(directory/png,dpi=150)
            plt.close(fig)
            saved.append(png)
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
        if not item["pass"] and item.get("diagnostic_excerpt"):
            lines.extend(["", "### "+name+" error excerpt", "", "```text",
                          item["diagnostic_excerpt"], "```", ""])
    lines+=["","## Static code audit"]
    lines.extend("- "+x for x in data["software"]["source"]["limits"])
    lines+=["","## Hardware / sweep",json.dumps(data.get("hardware"),indent=2,ensure_ascii=False),
            json.dumps(data.get("sweep"),indent=2,ensure_ascii=False),
            "## Encoder OFF/ON high-rate telemetry",
            json.dumps(data.get("encoder_motor_on"),indent=2,ensure_ascii=False),
             "## Alpha A/B estimator comparison (1kHz MCU metrics)",
             json.dumps(data.get("alpha_ab"),indent=2,ensure_ascii=False),
            "## Qualification warnings (kept separate from Encoder measurements)",
            str(data.get("warning","None")),
            "","## Remaining limitations",
            "- Input capture filter IC1F/IC2F and 10-ms speed estimator remain candidates, NOT auto-applied.",
            "- Encoder diagnostic OFF/ON builds emit SYS_EN ~50 Hz; missing edges require independent reference; synthetic estimator study is not board qualification.",
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
                   help="Run automatic OFF/ON 33-stage motor sweep (0..100%% PWM)")
    p.add_argument("--fixture-ready",action="store_true",
                   help="Wheels securely lifted/guarded, independent VM cutoff available, operator present")
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
        if not args.fixture_ready:
            p.error("Motor test requires --fixture-ready (lifted wheels, independent VM cutoff and operator present)")
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
                             "--encoder-telemetry",
                             "--outdir",str(out)],"motor_on_automated",path,950,
                             live_output=True)
                data["hardware"]={"status":"PASS" if runinfo["pass"] else "FAIL",
                                  "execution":runinfo}
                new=out/"summary.json"
                if new.exists():
                    data["sweep"]=sweep_summary(new,path,"motor_on")
                data["encoder_motor_on"]=analyze_sweep_telemetry(out,path)
                from encoder_shadow_ab import analyze as analyze_alpha_ab
                data["alpha_ab"]=analyze_alpha_ab(out,path)
                # Complete sweeps with UART/I2C qualification faults are not
                # physically accepted, but their alpha/encoder evidence should
                # be analyzed and retained. No safety gate is relaxed.
                nested=(json.loads(new.read_text(encoding="utf-8"))
                        if new.exists() else {})
                phases=nested.get("results",[])
                failed={phase.get("mode"): [
                    key for key,ok in phase.get("checks",{}).items() if not ok
                ] for phase in phases}
                actual_faults=[
                    {"mode":phase.get("mode"),"fault":item}
                    for phase in phases for item in phase.get("faults",[])
                ]
                if actual_faults:
                    data["hardware"]["firmware_faults"]=actual_faults
                    data["hardware"]["status"]="FIRMWARE_SAFETY_ABORT"
                all_stages=(len(phases)==2 and
                            all(phase.get("stages")==33 for phase in phases))
                failures=set(item for items in failed.values() for item in items)
                health_failed=any(
                    x["status"]!="PASS" for x in data["encoder_motor_on"].values()
                )
                warnings=[]
                if not runinfo["pass"] or health_failed:
                    allowed={"i2c_400k_nominal","uart_no_rx_error","uart_no_tx_drop"}
                    if actual_faults:
                        raise RuntimeError(
                            "FIRMWARE_SAFETY_ABORT: "+repr(actual_faults)+
                            "; incomplete sweep; health checks: "+repr(failed))
                    if not all_stages or not failures or not failures.issubset(allowed):
                        raise RuntimeError(
                            "Incomplete/unsafe motor sweep; check raw logs and STOP state: "+
                            repr(failed))
                    data["hardware"]["status"]="SWEEP_COMPLETE_HEALTH_UNQUALIFIED"
                    data["hardware"]["failed_checks"]=failed
                    warnings.append(
                        "All 33 OFF/ON stages completed, but full system "
                        "acceptance FAILED: "+repr(failed)+
                        ". Encoder/alpha evidence is preserved for review.")
                if data["alpha_ab"]["status"]!="COMPLETE_PROVISIONAL":
                    if all_stages:
                        if not failures:
                            data["hardware"]["status"]="SWEEP_COMPLETE_ALPHA_INCOMPLETE"
                        warnings.append(
                            "Alpha A/B data are incomplete: missing "+
                            str(data["alpha_ab"]["missing_metric_frames_count"])+
                            " metric frames; any ranking is exploratory, not accepted.")
                    else:
                        raise RuntimeError("Alpha A/B incomplete after unfinished sweep")
                if warnings:
                    data["warning"]=" ".join(warnings)
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
    except (OSError,ValueError,RuntimeError,KeyError,TypeError) as ex:
        # Plotting is best effort. Never suppress an earlier firmware FAULT,
        # successful locked-OFF restoration or any evidence files.
        data["plots"]="SKIP: "+str(ex)
    (path/"summary.json").write_text(
        json.dumps(data,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    report(path,data)
    print("Results:",path)
    if data.get("warning"):
        print("I2C RATE WARNING:",data["warning"],flush=True)
    return 2 if ("error" in data or "warning" in data) else 0

if __name__=="__main__":
    raise SystemExit(main())
