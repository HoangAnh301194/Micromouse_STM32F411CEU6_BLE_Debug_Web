#!/usr/bin/env python3
"""PA0-only 90/180-degree turn PID capture, offline metrics and PNG plots.

Serial NEVER starts a motor. It may only select idle angle/profile and STOP.
No numpy required. Requires pyserial for live capture; matplotlib is optional.
Typical:
  python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile base --runs 3
  python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile fast --runs 3
  python tools/turn_pid_pipeline.py --analyze results/turn_pid_YYYYmmdd_HHMMSS
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import sys
import time
import zipfile

FIELD_NAMES = (
    "run_id", "t_ms", "target_deg", "profile", "phase",
    "yaw_deg", "error_deg", "gyro_dps", "ref_dps",
    "pid_pm", "ff_pm", "cmd_pm", "enc_l", "enc_r", "imu_age_us"
)
REPORT_FIELDS = (
    "run_id", "target_deg", "profile", "reason",
    "t10_ms", "t90_ms", "rise_10_90_ms", "t_first_target_ms",
    "stop_ms", "settling_ms", "peak_yaw_deg", "overshoot_deg",
    "overshoot_percent", "max_abs_gyro_dps", "final_yaw_deg",
    "final_abs_error_deg", "coast_drift_deg", "trace_samples",
    "sampling_gap_max_ms", "ring_drops", "uart_drops",
    "meets_trace_thresholds", "imu_effective_hz", "imu_intervals_max_us",
    "imu_intervals_mean_us", "imu_gap_gt1500", "imu_gap_gt2000",
    "imu_observer_missed", "imu_status_notready", "imu_i2c_errors",
    "imu_age_mean_us", "imu_age_max_us"
)

IMU_FIELDS = (
    "run_id", "t_ms", "window_ms", "new_samples", "effective_hz",
    "interval_mean_us", "interval_min_us", "interval_max_us",
    "gap_gt1500", "gap_gt2000", "observer_missed", "status_notready",
    "i2c_errors", "last_age_us", "last_seq", "is_final"
)
def parse_imu(line: str) -> dict | None:
    """Decode one MCU timing window; no inference from UART arrival gaps."""
    if not line.startswith("TP,IMU,"):
        return None
    data=parse_kv(line,"TP,IMU,")
    keys=("id","t","win","new","hz10","dtavg","dtmin","dtmax",
          "g1500","g2000","miss","nr","err","age","seq","last")
    try:
        v={k:int(data[k]) for k in keys}
    except (KeyError,ValueError):
        return None
    if v["id"]<=0 or v["win"]<=0 or v["new"]<0 or v["last"] not in (0,1):
        return None
    return dict(run_id=v["id"],t_ms=v["t"],window_ms=v["win"],
                new_samples=v["new"],effective_hz=v["hz10"]/10.0,
                interval_mean_us=v["dtavg"],interval_min_us=v["dtmin"],
                interval_max_us=v["dtmax"],gap_gt1500=v["g1500"],
                gap_gt2000=v["g2000"],observer_missed=v["miss"],
                status_notready=v["nr"],i2c_errors=v["err"],
                last_age_us=v["age"],last_seq=v["seq"],
                is_final=v["last"])




def parse_sample(line: str) -> dict | None:
    """One producer-timestamped gyro/PWM sample; rejects partial UART frames."""
    parts = line.strip().split(",")
    if len(parts) != 17 or parts[:2] != ["TP", "D"]:
        return None
    try:
        raw = [int(i) for i in parts[2:]]
    except ValueError:
        return None
    (run_id, t_ms, target, profile, phase, yaw_cd, err_cd,
     gyro_tenth, ref_tenth, pid_pm, ff_pm, cmd_pm, left, right,
     age_us) = raw
    if run_id <= 0 or not 0 <= t_ms <= 100_000 or target not in (90, 180):
        return None
    if profile not in (0, 1) or phase not in (1, 2):
        return None
    if abs(cmd_pm) > 300 or age_us < 0:
        return None
    return dict(run_id=run_id, t_ms=t_ms, target_deg=target,
                profile=profile, phase=phase, yaw_deg=yaw_cd / 100,
                error_deg=err_cd / 100, gyro_dps=gyro_tenth / 10,
                ref_dps=ref_tenth / 10, pid_pm=pid_pm, ff_pm=ff_pm,
                cmd_pm=cmd_pm, enc_l=left, enc_r=right,
                imu_age_us=age_us)


def parse_kv(line: str, prefix: str) -> dict:
    if not line.startswith(prefix):
        return {}
    result = {}
    for item in line[len(prefix):].strip().split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value
    return result


def interpolate_crossing(samples: list[dict], threshold: float) -> float | None:
    """First upward threshold crossing, interpolated between 50Hz points."""
    previous = None
    for item in sorted(samples, key=lambda x: x["t_ms"]):
        if previous is not None:
            a, b = previous["yaw_deg"], item["yaw_deg"]
            if a < threshold <= b and b > a:
                return previous["t_ms"] + (item["t_ms"] - previous["t_ms"]) * (
                    threshold - a) / (b - a)
        elif item["yaw_deg"] >= threshold:
            return float(item["t_ms"])
        previous = item
    return None


def analyze(samples: list[dict], result: dict,
            imu_records: list[dict] | None = None) -> dict:
    if not samples:
        return {"error": "No valid TP,D frames", **result}
    rows = sorted((s for s in samples if s["run_id"] == samples[0]["run_id"]),
                  key=lambda s: s["t_ms"])
    target = int(rows[0]["target_deg"])
    t10 = interpolate_crossing(rows, target * 0.10)
    t90 = interpolate_crossing(rows, target * 0.90)
    tfull = interpolate_crossing(rows, float(target))
    peak = max(s["yaw_deg"] for s in rows)
    max_rate = max(abs(s["gyro_dps"]) for s in rows)
    final_yaw = rows[-1]["yaw_deg"]
    overshoot = max(0.0, peak - target)
    stop_ms = int(result["stop_ms"]) if result.get("stop_ms") else None
    at_stop = min(rows, key=lambda s: abs(s["t_ms"] - stop_ms)) if stop_ms is not None else None
    coast_drift = round(final_yaw - at_stop["yaw_deg"], 3) if at_stop else None
    gaps = [b["t_ms"]-a["t_ms"] for a,b in zip(rows,rows[1:])]
    imu_records = imu_records or []
    valid_imu = [w for w in imu_records if w["run_id"]==rows[0]["run_id"]]
    duration_ms=sum(w["window_ms"] for w in valid_imu)
    new_samples=sum(w["new_samples"] for w in valid_imu)
    mean_gyro_dt=(sum(w["interval_mean_us"]*w["new_samples"] for w in valid_imu)
                  /new_samples) if new_samples else None
    sample_ages=[row["imu_age_us"] for row in rows]
    settling = None
    # "settled" means ALL remaining samples stay within +-2 degrees
    # for at least the final 150 ms. An isolated one-sample crossing
    # is not a genuine settling event.
    if rows[-1]["t_ms"] - rows[0]["t_ms"] >= 150:
        for i,s in enumerate(rows):
            if rows[-1]["t_ms"]-s["t_ms"] < 150:
                break
            if all(abs(t["yaw_deg"]-target) <= 2.0 for t in rows[i:]):
                settling = float(s["t_ms"])
                break
    def rnd(x):
        return None if x is None else round(float(x), 3)
    return {
        "run_id": rows[0]["run_id"],
        "target_deg": target,
        "profile": "FAST" if rows[0]["profile"] else "BASE",
        "reason": result.get("reason", "PARTIAL"),
        "t10_ms": rnd(t10),
        "t90_ms": rnd(t90),
        "rise_10_90_ms": rnd(t90-t10) if t10 is not None and t90 is not None else None,
        "t_first_target_ms": rnd(tfull),
        "stop_ms": stop_ms,
        "settling_ms": rnd(settling),
        "peak_yaw_deg": rnd(peak),
        "overshoot_deg": rnd(overshoot),
        "overshoot_percent": rnd(overshoot*100.0/target),
        "max_abs_gyro_dps": rnd(max_rate),
        "final_yaw_deg": rnd(final_yaw),
        "final_abs_error_deg": rnd(abs(target-final_yaw)),
        "coast_drift_deg": coast_drift,
        "imu_effective_hz": rnd(new_samples*1000.0/duration_ms) if duration_ms else None,
        "imu_intervals_max_us": max((w["interval_max_us"] for w in valid_imu),default=None),
        "imu_intervals_mean_us": rnd(mean_gyro_dt),
        "imu_gap_gt1500": sum(w["gap_gt1500"] for w in valid_imu) if valid_imu else None,
        "imu_gap_gt2000": sum(w["gap_gt2000"] for w in valid_imu) if valid_imu else None,
        "imu_observer_missed": sum(w["observer_missed"] for w in valid_imu) if valid_imu else None,
        "imu_status_notready": sum(w["status_notready"] for w in valid_imu) if valid_imu else None,
        "imu_i2c_errors": sum(w["i2c_errors"] for w in valid_imu) if valid_imu else None,
        "imu_age_mean_us": rnd(statistics.mean(sample_ages)),
        "imu_age_max_us": max(sample_ages),
        "trace_samples": len(rows),
        "sampling_gap_max_ms": max(gaps) if gaps else 0,
        "ring_drops": int(result.get("qdrop", 0)),
        "uart_drops": int(result.get("uartdrop", 0)),
        # Software trace quality only, NOT physical floor-motion acceptance.
        "meets_trace_thresholds": bool(
            result.get("reason") == "DONE" and overshoot <= 3.0 and
            abs(target-final_yaw) <= 2.0 and settling is not None and
            stop_ms is not None and rows[-1]["t_ms"]>=stop_ms+650 and
            max(gaps, default=0) <= 60 and len(rows) >= 12 and
            int(result.get("qdrop", 0)) == 0 and
            int(result.get("uartdrop", 0)) == 0),
    }


def save_csv(path: Path, samples: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELD_NAMES)
        writer.writeheader()
        for sample in samples:
            writer.writerow({k: sample.get(k) for k in FIELD_NAMES})


def plot_run(path: Path, samples: list[dict], title: str) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    if not samples:
        return False
    t=[s["t_ms"]/1000 for s in samples]
    target=samples[0]["target_deg"]
    # One four-panel engineering trace, same timestamps on all channels.
    fig, axes=plt.subplots(4,1,figsize=(11,10),sharex=True)
    axes[0].plot(t,[s["yaw_deg"] for s in samples],label="yaw IMU")
    axes[0].axhline(target,linestyle="--",label=f"target {target} deg")
    axes[0].axhline(target+2,linestyle=":",alpha=0.5)
    axes[0].axhline(target-2,linestyle=":",alpha=0.5)
    axes[0].set_ylabel("Angle [deg]")
    axes[0].legend(loc="best")
    axes[1].plot(t,[s["error_deg"] for s in samples],label="error")
    axes[1].axhline(0,linestyle="--")
    axes[1].set_ylabel("Error [deg]")
    axes[2].plot(t,[s["gyro_dps"] for s in samples],label="gyro rate")
    axes[2].plot(t,[s["ref_dps"] for s in samples],label="rate reference")
    axes[2].set_ylabel("Rate [deg/s]")
    axes[2].legend(loc="best")
    axes[3].plot(t,[s["cmd_pm"] for s in samples],label="commanded PWM")
    axes[3].plot(t,[s["pid_pm"] for s in samples],label="rate PID",alpha=0.65)
    axes[3].plot(t,[s["ff_pm"] for s in samples],label="feedforward",alpha=0.65)
    axes[3].set_ylabel("Motor permille")
    axes[3].set_xlabel("TIM11 relative time [s]")
    axes[3].legend(loc="best")
    for ax in axes:
        ax.grid(True,alpha=0.25)
        coast = next((s["t_ms"]/1000 for s in samples if s["phase"]==2),None)
        if coast is not None:
            ax.axvline(coast,linestyle="--",alpha=0.5)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path,dpi=130)
    plt.close(fig)
    return True


def write_report(root: Path, runs: dict[int,list[dict]],
                 results: dict[int,dict],
                 imu_windows: dict[int,list[dict]] | None = None) -> list[dict]:
    imu_windows = imu_windows or {}
    root.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for run_id in sorted(runs):
        samples=runs[run_id]
        if not samples:
            continue
        folder=root/f"run_{run_id:03d}"
        folder.mkdir(exist_ok=True)
        save_csv(folder/"samples.csv",samples)
        subset=imu_windows.get(run_id,[])
        if subset:
            with (folder/"imu_windows.csv").open("w",newline="",encoding="utf-8") as f:
                w=csv.DictWriter(f,fieldnames=IMU_FIELDS)
                w.writeheader()
                for item in subset:w.writerow({k:item.get(k) for k in IMU_FIELDS})
        summary=analyze(samples,results.get(run_id,{}),subset)
        (folder/"summary.json").write_text(
            json.dumps(summary,indent=2,ensure_ascii=False)+"\n",
            encoding="utf-8")
        plot_run(folder/"trace.png",samples,
                 f"Run {run_id} / {summary.get('target_deg')} deg / "
                 f"{summary.get('profile')} / {summary.get('reason')}")
        summaries.append(summary)
    with (root/"summary.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=REPORT_FIELDS,extrasaction="ignore")
        writer.writeheader()
        for item in summaries:
            writer.writerow({k:item.get(k) for k in REPORT_FIELDS})
    (root/"summary.json").write_text(
        json.dumps(summaries,indent=2,ensure_ascii=False)+"\n",
        encoding="utf-8")
    stats={}
    groups={}
    for item in summaries:
        if "error" in item:continue
        key=f"{item['target_deg']}deg_{item['profile']}"
        groups.setdefault(key,[]).append(item)
    for key,items in groups.items():
        entry={"runs":len(items),"trace_pass":sum(bool(z.get("meets_trace_thresholds")) for z in items)}
        for field in ("rise_10_90_ms","stop_ms","settling_ms",
                      "overshoot_deg","max_abs_gyro_dps",
                      "final_abs_error_deg","coast_drift_deg",
                      "imu_effective_hz","imu_intervals_max_us",
                      "imu_intervals_mean_us","imu_age_mean_us",
                      "imu_age_max_us"):
            numbers=[float(z[field]) for z in items if z.get(field) is not None]
            if numbers:
                entry[field]={"mean":round(statistics.mean(numbers),3),
                              "stdev":round(statistics.stdev(numbers),3) if len(numbers)>1 else None}
            else:
                entry[field]={"mean":None,"stdev":None}
        stats[key]=entry
    (root/"group_stats.json").write_text(
        json.dumps(stats,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    return summaries


def zip_session(root: Path) -> Path:
    destination=root/"turn_pid_bundle.zip"
    with zipfile.ZipFile(destination,"w",compression=zipfile.ZIP_DEFLATED) as zf:
        for source in root.rglob("*"):
            if source.is_file() and source!=destination:
                zf.write(source,arcname=str(source.relative_to(root)))
    return destination


def capture(args: argparse.Namespace) -> int:
    try:
        import serial
    except ImportError:
        print("Missing pyserial. Install: python -m pip install pyserial",
              file=sys.stderr)
        return 2
    root=Path(args.output) if args.output else Path("results")/(
        "turn_pid_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    root.mkdir(parents=True,exist_ok=True)
    runs: dict[int,list[dict]]={}
    results: dict[int,dict]={}
    imu_windows: dict[int,list[dict]]={}
    completed=set()
    mode_byte=b"1" if args.mode==90 else b"2"
    profile_byte=b"b" if args.profile=="base" else b"f"
    pending=bytearray()
    last_serial_activity=time.monotonic()
    last_config=0.0
    configured=False
    print(f"Results directory: {root.resolve()}")
    print(f"Requested: {args.mode} degrees / {args.profile.upper()} / "
          f"{args.runs} run(s)")
    print("IMPORTANT: UART cannot start motors. Wait for TP,CAL_OK / "
          "TP,CONFIG then physically PRESS and RELEASE PA0.")
    print("A second PA0 press during motion stops the motors; VM power "
          "cutoff must remain available.")
    try:
        ser=serial.Serial(args.port,args.baud,timeout=0.15,
                          write_timeout=1.0)
    except Exception as exc:
        print(f"Cannot open {args.port}: {exc}",file=sys.stderr)
        return 2
    began=time.monotonic()
    try:
        with ser, (root/"raw_uart.log").open("w",encoding="utf-8") as raw:
            def configure():
                nonlocal last_config
                # Idle-only configuration; no start command exists.
                ser.write(mode_byte+profile_byte+b"?")
                ser.flush()
                last_config=time.monotonic()
                print(f"[CONFIG] sent angle={args.mode}, profile={args.profile}; "
                      "PA0 is the ONLY start.")
            configure()
            while len(completed)<args.runs:
                if args.duration and time.monotonic()-began>args.duration:
                    print("[STOP] Total session time limit reached; commanding UART stop.")
                    try:ser.write(b"S")
                    except Exception:pass
                    break
                try:
                    data=ser.readline()
                except (serial.SerialException,OSError) as exc:
                    print(f"[ERROR] COM read failed: {exc}",file=sys.stderr)
                    break
                if not data:
                    continue
                line=data.decode("utf-8",errors="replace").strip()
                raw.write(line+"\n")
                raw.flush()
                last_serial_activity=time.monotonic()
                if line.startswith("TP,D,"):
                    sample=parse_sample(line)
                    if sample:
                        runs.setdefault(sample["run_id"],[]).append(sample)
                    else:
                        print(f"[WARN] Invalid trace frame: {line}")
                elif line.startswith("TP,IMU,"):
                    w=parse_imu(line)
                    if w:
                        imu_windows.setdefault(w["run_id"],[]).append(w)
                        print(f"[IMU] run={w['run_id']} published={w['effective_hz']:.1f} Hz "
                              f"gap_avg={w['interval_mean_us']}us "
                              f"gap_max={w['interval_max_us']}us "
                              f"notready={w['status_notready']} "
                              f"i2c_err={w['i2c_errors']}")
                    else:
                        print(f"[WARN] Invalid IMU timing record: {line}")
                elif line.startswith("TP,RESULT,"):
                    kv=parse_kv(line,"TP,RESULT,")
                    try:rid=int(kv["id"])
                    except (KeyError,ValueError):
                        print(f"[WARN] Invalid result: {line}")
                        continue
                    results[rid]=kv
                    completed.add(rid)
                    print(f"[RESULT] {line}")
                    summaries=write_report(root,runs,results,imu_windows)
                    if summaries: print("[METRICS] "+json.dumps(summaries[-1],
                                                     ensure_ascii=False))
                elif line.startswith("TP,HELLO,") or line.startswith("TP,BOOT,"):
                    print("[BOOT] "+line)
                    if time.monotonic()-last_config>1.0:
                        configure()
                elif line.startswith("TP,CAL,"):
                    print("[CAL] "+line)
                elif line.startswith("TP,CAL_OK,"):
                    print("[CAL OK] "+line)
                elif line.startswith("TP,READY,"):
                    print("[READY] "+line)
                    if not configured:
                        configure()
                        configured=True
                elif line.startswith("TP,START,"):
                    print("[START] "+line)
                elif line.startswith("TP,CONFIG,"):
                    print("[CONFIG OK] "+line)
                elif line.startswith("TP,") and not line.startswith("TP,IDLE,"):
                    print("[MCU] "+line)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Sending serial STOP as a precaution.")
        try:ser.write(b"S")
        except Exception:pass
    finally:
        summaries=write_report(root,runs,results,imu_windows)
        (root/"session.json").write_text(
            json.dumps({"port":args.port,"baud":args.baud,"requested_mode":args.mode,
                        "requested_profile":args.profile,"requested_runs":args.runs,
                        "completed_runs":len(completed),"valid_samples":sum(map(len,runs.values()))},
                       indent=2)+"\n",encoding="utf-8")
        archive=zip_session(root)
        print(f"[SAVED] {root.resolve()}")
        print(f"[UPLOAD THIS ZIP] {archive.resolve()}")
        print(f"[COUNT] {len(completed)} complete result(s); "
              f"{len(summaries)} run(s) with trace data.")
    return 0 if len(completed)>=args.runs else 1


def offline(folder: Path) -> int:
    path=folder/"raw_uart.log"
    if not path.exists():
        print(f"Missing {path}",file=sys.stderr)
        return 2
    runs={}
    results={}
    imu_windows={}
    for line in path.read_text(encoding="utf-8",errors="replace").splitlines():
        sample=parse_sample(line)
        if sample:
            runs.setdefault(sample["run_id"],[]).append(sample)
        elif line.startswith("TP,IMU,"):
            w=parse_imu(line)
            if w:imu_windows.setdefault(w["run_id"],[]).append(w)
        elif line.startswith("TP,RESULT,"):
            kv=parse_kv(line,"TP,RESULT,")
            try:results[int(kv["id"])]=kv
            except (KeyError,ValueError):pass
    summaries=write_report(folder,runs,results,imu_windows)
    for item in summaries:
        print(json.dumps(item,ensure_ascii=False))
    archive=zip_session(folder)
    print(f"[SAVED] {folder.resolve()}")
    print(f"[UPLOAD THIS ZIP] {archive.resolve()}")
    return 0 if summaries else 1


def main(argv=None) -> int:
    parser=argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port",default="COM7",help="USB-TTL USART1 port")
    parser.add_argument("--baud",type=int,default=115200)
    parser.add_argument("--mode",type=int,choices=[90,180],default=90)
    parser.add_argument("--profile",choices=["base","fast"],default="base")
    parser.add_argument("--runs",type=int,default=3)
    parser.add_argument("--output",type=str,help="destination results folder")
    parser.add_argument("--duration",type=float,default=360,
                        help="maximum capture session seconds, 0 means unlimited")
    parser.add_argument("--analyze",type=Path,help="offline reprocess previous raw UART folder")
    args=parser.parse_args(argv)
    if args.runs<1:parser.error("--runs must be >=1")
    return offline(args.analyze) if args.analyze else capture(args)


if __name__=="__main__":
    raise SystemExit(main())
