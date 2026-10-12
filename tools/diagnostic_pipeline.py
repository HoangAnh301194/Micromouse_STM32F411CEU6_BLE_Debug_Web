#!/usr/bin/env python3
"""One-session, motor-OFF IMU/clock/control diagnostic pipeline.

Capture raw UART and analyze boot, clock, MPU freshness, and 30-s stability.
Requires firmware with CLK, SYNC_SENSOR, SYNC_CTRL, SYNC_AB telemetry.
No motor commands or firmware flashing are performed.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent


def run_tool(script, args):
    cmd = [sys.executable, str(HERE / script), *map(str, args)]
    print("+", " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=ROOT, check=False).returncode


def read_telemetry(path):
    result = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.rstrip("\n").split("\t", 1)[-1].strip()
            if line:
                result.append(line)
    boots = [i for i, line in enumerate(result) if "[BOOT] Isolated MPU6050/I2C test" in line]
    if boots:
        result = result[boots[-1]:]
    return result, bool(boots)


def kv(line):
    fields = {}
    for part in re.split(r"[,\s]+", line)[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            fields[k.strip()] = v.strip()
    return fields


def phases(lines, boot):
    tags = ("CLOCK_PROFILE", "SYNC_POLICY", "CAL_DONE", "CLK", "IMU",
            "TIMING", "SYNC_SENSOR", "SYNC_CTRL", "SYNC_AB")
    grouped = {tag: [] for tag in tags}
    for line in lines:
        for tag in tags:
            if line.startswith(tag + ","):
                grouped[tag].append(kv(line))
                break
    def n(record, key):
        try:
            return int(record[key])
        except (KeyError, TypeError, ValueError):
            return None
    policy = grouped["SYNC_POLICY"][-1].get("policy") if grouped["SYNC_POLICY"] else None
    checks = {
        "boot_present": boot,
        "motor_off_profile": bool(grouped["CLOCK_PROFILE"] and
                                  grouped["CLOCK_PROFILE"][-1].get("motor_off") == "1"),
        "motor_off_sampling": bool(grouped["SYNC_POLICY"] and
                                   grouped["SYNC_POLICY"][-1].get("motor_off") == "1"),
        "calibration_4000_no_retries": bool(grouped["CAL_DONE"] and
            n(grouped["CAL_DONE"][-1], "n") == 4000 and
            n(grouped["CAL_DONE"][-1], "retries") == 0),
        "clock_samples_at_least_150": len(grouped["CLK"]) >= 150,
        "telemetry_complete": all(grouped[k] for k in ("TIMING", "IMU", "SYNC_SENSOR", "SYNC_CTRL", "SYNC_AB")),
        "no_boot_fail": not any("BOOT_FAIL" in line for line in lines),
    }
    for tag, columns in (("IMU", ("err", "timeout", "uart_drop", "uart_dma_err")),
                         ("TIMING", ("gap_over_1500us", "late_50us"))):
        checks["zero_" + tag.lower() + "_errors"] = bool(grouped[tag]) and all(
            n(row, col) == 0 for row in grouped[tag] for col in columns)
    # Cumulative counters are differenced; comparison across RESET is invalid.
    windows = []
    ctrl = grouped["SYNC_CTRL"]
    # Exclude pre-calibration windows; cumulative counters stay monotonic.
    cal_pos = max((i for i, item in enumerate(lines) if item.startswith("CAL_DONE,")), default=-1)
    if cal_pos >= 0:
        ctrl = [kv(item) for item in lines[cal_pos + 1:] if item.startswith("SYNC_CTRL,")]
    if len(ctrl) > 30:
        baseline = ctrl[0]
        for end_idx in range(30, len(ctrl), 30):
            last = ctrl[end_idx]
            ticks = n(last, "ticks")
            first_ticks = n(baseline, "ticks")
            repeated = n(last, "repeat")
            first_repeated = n(baseline, "repeat")
            old = n(last, "age_gt1500")
            first_old = n(baseline, "age_gt1500")
            if None not in (ticks, first_ticks, repeated, first_repeated, old, first_old):
                delta = ticks - first_ticks
                if delta > 0 and repeated >= first_repeated and old >= first_old:
                    windows.append({
                        "window_index": len(windows) + 1,
                        "ticks": delta,
                        "reused": repeated - first_repeated,
                        "reused_pct": round(100 * (repeated-first_repeated)/delta, 5),
                        "age_gt1500": old-first_old,
                    })
            baseline = last
    return {"policy": policy, "checks": checks, "counts": {k: len(v) for k,v in grouped.items()},
            "windows_approximately_30_frames": windows,
            "note": "30 consecutive SYNC_CTRL frames are approximately 30s only if emitted at 1Hz."}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=int, default=240)
    ap.add_argument("--folder", type=pathlib.Path, help="Offline existing clock_logs session")
    args = ap.parse_args()
    if args.folder:
        folder = args.folder.resolve()
    else:
        if args.seconds < 180:
            ap.error("At least 180s is required for freshness validation")
        folder = ROOT / "clock_logs" / ("pipeline_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
        folder.mkdir(parents=True, exist_ok=True)
        print("Motor-OFF only. Start capture, then press hardware RESET once.", flush=True)
        rc = run_tool("clock_serial_test.py", ["--port", args.port,
            "--baud", args.baud, "--seconds", args.seconds, "--out", folder])
        if rc:
            print("Capture/clock analysis failed; see raw log if present.", file=sys.stderr)
            return rc
    raw = folder / "raw_serial.log"
    if not raw.exists():
        print("Missing raw_serial.log", file=sys.stderr)
        return 2
    lines, boot = read_telemetry(raw)
    report = phases(lines, boot)
    rc = run_tool("imu_freshness_analyze.py", ["--folder", folder])
    report["freshness_analyzer_exit"] = rc
    report_path = folder / "pipeline_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("Pipeline report:", report_path)
    if rc or not all(report["checks"].values()) or report["policy"] not in ("A", "B"):
        print("INCOMPLETE/FAIL: do not authorize motor operation.")
        return 1
    print("DIAGNOSTIC CAPTURE PASS (MOTOR OFF ONLY); NOT a production/motion approval")
    return 0


if __name__ == "__main__":
    sys.exit(main())
