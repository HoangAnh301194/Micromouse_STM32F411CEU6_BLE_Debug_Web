#!/usr/bin/env python3
"""Offline maze-transition readiness suite, never moves motors or flashes MCU.

preflight   Execute safe existing software regression tests; optional compile only.
motion      Analyze several exported BLE wall-PID terminal logs together.
wall-events Calibrate one threshold per IR channel on labelled TRAIN CSV and
            evaluate on a separate labelled TEST CSV with hysteresis and debounce.

No threshold here is wired into the Wall PID motion controller.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import importlib.util
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CHANNELS = ("L90", "L45", "L0", "R0", "R45", "R90")
LABELS = tuple("wall_" + c for c in CHANNELS)
MOTION_FIELDS = ("file", "complete", "reason", "stop_mm", "drift_mm",
                 "trace_rows", "peak_speed_mm_s", "mae_norm",
                 "p95_norm", "fault", "review_flags")
REGRESSION = (
    "tests/regression/ir_wall_signal_native_test.py",
    "tests/regression/wall_pid_4cell_native_test.py",
    "tests/regression/wall_pid_ble_report_test.py",
    "tests/regression/ir_full_qualification_test.py",
    "tests/regression/platformio_profile_cleanup_test.py",
)
ENVIRONMENTS = ("wall_pid_4cell_96m", "ir_burst_diag_96m",
                "turn_pid_diag_96m", "flash_map_diag_96m")


def write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")


def write_csv(path: Path, columns: tuple[str, ...], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def load_wall_pid_parser():
    p = ROOT / "tools" / "wall_pid_ble_report.py"
    spec = importlib.util.spec_from_file_location("wall_pid_ble_report", p)
    if not spec or not spec.loader:
        raise RuntimeError("Cannot import " + str(p))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.parse_lines


def preflight(out: Path, build: bool = False) -> dict:
    steps = []
    commands = [[sys.executable, str(ROOT / p)] for p in REGRESSION]
    if build:
        commands.extend([[sys.executable, "-m", "platformio", "run", "-e", e]
                         for e in ENVIRONMENTS])
    for command in commands:
        try:
            proc = subprocess.run(command, cwd=ROOT, capture_output=True,
                                  text=True, timeout=180,
                                  check=False)
            result = {"command": command, "exit_code": proc.returncode,
                      "stdout_tail": proc.stdout[-1800:],
                      "stderr_tail": proc.stderr[-1800:]}
        except (subprocess.TimeoutExpired, OSError) as exc:
            result = {"command": command, "exit_code": -1,
                      "error": str(exc)}
        steps.append(result)
    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "offline_software_only",
        "hardware_tested": False, "firmware_uploaded": False,
        "software_checks_ok": all(s["exit_code"] == 0 for s in steps),
        "steps": steps,
    }
    write_json(out / "preflight.json", report)
    return report


def analyze_motion(logs: list[Path], out: Path, target_mm: float = 720.0,
                   tolerance_mm: float = 15.0,
                   drift_review_mm: float = 30.0) -> dict:
    if not logs:
        raise ValueError("Provide at least one exported BLE terminal log")
    parser = load_wall_pid_parser()
    rows = []
    for file in logs:
        raw = file.read_text(encoding="utf-8", errors="replace")
        trace, report = parser(raw.splitlines())
        end = report.get("end") or {}
        flags = []
        if not report["data_complete"]:
            flags.append("MISSING_OR_INVALID_BLE_LOG")
        if report["reason"] != "DONE":
            flags.append("NOT_720MM_DONE")
        if report["stop_distance_mm"] is None or abs(report["stop_distance_mm"] - target_mm) > tolerance_mm:
            flags.append("DISTANCE_OUTSIDE_REVIEW_RANGE")
        if report["drift_mm"] is None or abs(report["drift_mm"]) > drift_review_mm:
            flags.append("POST_STOP_DRIFT_REVIEW")
        if report.get("fault"):
            flags.append("FAULT_CAPTURED")
        rows.append({
            "file": str(file),
            "complete": report["data_complete"],
            "reason": report["reason"],
            "stop_mm": report["stop_distance_mm"],
            "drift_mm": report["drift_mm"],
            "trace_rows": len(trace),
            "peak_speed_mm_s": end.get("vpk", report["max_speed_mm_s"]),
            "mae_norm": report.get("mae_wall_error_norm"),
            "p95_norm": report.get("p95_abs_wall_error_norm"),
            "fault": (report.get("fault") or {}).get("why", ""),
            "review_flags": ";".join(flags),
        })
    summary = {
        "mode": "offline_motion_comparison",
        "hardware_tested_by_tool": False,
        "runs": len(rows),
        "complete_logs": sum(int(r["complete"]) for r in rows),
        "done_runs": sum(int(r["reason"] == "DONE") for r in rows),
        "review_no_flags": sum(not bool(r["review_flags"]) for r in rows),
        "max_drift_mm": max((abs(r["drift_mm"]) for r in rows
                             if r["drift_mm"] is not None), default=None),
        "target_mm": target_mm,
        "review_limits_not_acceptance": {
            "position_tolerance_mm": tolerance_mm,
            "post_stop_drift_mm": drift_review_mm,
        },
        "physical_wall_pid_qualified": False,
        "limitations": "No independent centre-distance ground truth, no measured braking clearance, no hardware test.",
        "results": rows,
    }
    write_csv(out / "motion_runs.csv", MOTION_FIELDS, rows)
    write_json(out / "motion_summary.json", summary)
    return summary


def read_labeled_csv(path: Path) -> list[dict]:
    required = ("ms",) + CHANNELS + LABELS
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or any(c not in reader.fieldnames for c in required):
            raise ValueError(f"{path}: expected columns: {', '.join(required)}")
        rows = []
        last_time = None
        for lineno, item in enumerate(reader, 2):
            try:
                ms = float(item["ms"])
                if not math.isfinite(ms) or (last_time is not None and ms <= last_time):
                    raise ValueError("timestamps must increase strictly")
                sample = {"ms": ms}
                for c in CHANNELS:
                    x = float(item[c])
                    if not math.isfinite(x) or not 0 <= x <= 4095:
                        raise ValueError(f"{c} is not a valid 12-bit ADC reading")
                    sample[c] = x
                    t = item["wall_" + c].strip()
                    if t not in ("0", "1"):
                        raise ValueError(f"wall_{c} must be 0 or 1")
                    sample["wall_" + c] = int(t)
                rows.append(sample)
                last_time = ms
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}: row {lineno}: {exc}") from exc
    if len(rows) < 4:
        raise ValueError(f"{path}: at least four labelled samples needed")
    return rows


def percentile(values: list[float], fraction: float = .95):
    if not values:
        return None
    values = sorted(values)
    # nearest rank: a conservative p95 on small event sets
    return values[min(len(values)-1, max(0, math.ceil(fraction*len(values))-1))]


def calibrate(train: list[dict]) -> dict:
    model = {}
    for c in CHANNELS:
        neg = sorted(r[c] for r in train if r["wall_"+c] == 0)
        pos = sorted(r[c] for r in train if r["wall_"+c] == 1)
        if len(pos) < 2 or len(neg) < 2:
            raise ValueError(f"{c}: need >=2 labelled wall and >=2 no-wall samples in TRAIN")
        med0, med1 = statistics.median(neg), statistics.median(pos)
        if med1 <= med0:
            raise ValueError(f"{c}: positive-wall median <= negative median; review optical geometry/labels")
        candidates = sorted(set(neg + pos))
        thresholds = [(a+b)/2 for a,b in zip(candidates, candidates[1:])]
        if not thresholds:
            raise ValueError(f"{c}: overlapping single-valued classes")
        midpoint = (med0 + med1)/2
        def score(threshold):
            tpr = sum(x >= threshold for x in pos)/len(pos)
            tnr = sum(x < threshold for x in neg)/len(neg)
            return (tpr + tnr)/2
        best = min(thresholds, key=lambda t: (-score(t), abs(t-midpoint)))
        separation = med1 - med0
        width = max(1.0, min(separation*.10, separation*.40))
        model[c] = {
            "threshold": best, "hysteresis_adc": width,
            "median_wall": med1, "median_no_wall": med0,
            "train_balanced_accuracy": score(best),
        }
    return model


def apply_hysteresis(rows: list[dict], channel: str, model: dict,
                     confirm_frames: int) -> list[int]:
    if confirm_frames < 1:
        raise ValueError("confirm_frames must be >= 1")
    params = model[channel]
    midpoint = params["threshold"]
    half_width = params["hysteresis_adc"]*.5
    on = midpoint + half_width
    off = midpoint - half_width
    state = 0
    pending = 0
    consecutive = 0
    outputs = []
    for row in rows:
        sample = row[channel]
        desired = 1 if sample >= on else (0 if sample <= off else state)
        if desired == state:
            consecutive = 0
        else:
            if desired != pending:
                pending, consecutive = desired, 1
            else:
                consecutive += 1
            if consecutive >= confirm_frames:
                state = pending
                consecutive = 0
        outputs.append(state)
    return outputs


def transitions(rows: list[dict], states: list[int]) -> list[tuple[float, int]]:
    return [(rows[i]["ms"], states[i]) for i in range(1, len(states))
            if states[i] != states[i-1]]


def event_lag(reference: list[tuple[float, int]],
              measured: list[tuple[float, int]],
              max_lag_ms: float) -> tuple[list[float], int]:
    lags = []
    missed = 0
    used = set()
    for t, direction in reference:
        matches = [(idx, mt-t) for idx, (mt, d) in enumerate(measured)
                   if idx not in used and d == direction and 0 <= mt-t <= max_lag_ms]
        if not matches:
            missed += 1
        else:
            idx, delay = min(matches, key=lambda q: q[1])
            used.add(idx)
            lags.append(delay)
    return lags, missed


EVENT_FIELDS = ("channel", "precision", "recall", "f1", "specificity",
                "true_positive", "false_positive", "false_negative",
                "true_negative", "gt_transitions", "missed_transitions",
                "p95_detection_delay_ms", "threshold_adc", "hysteresis_adc")


def analyze_wall_events(train_file: Path, test_file: Path, out: Path,
                        confirm_frames: int = 3,
                        max_transition_lag_ms: float = 250.0) -> dict:
    if train_file.resolve() == test_file.resolve():
        raise ValueError("TRAIN and TEST must be different files (holdout evaluation)")
    if confirm_frames < 1 or max_transition_lag_ms <= 0:
        raise ValueError("confirm_frames >=1 and max_transition_lag_ms >0 required")
    train = read_labeled_csv(train_file)
    test = read_labeled_csv(test_file)
    model = calibrate(train)
    rows = []
    for c in CHANNELS:
        pred = apply_hysteresis(test, c, model, confirm_frames)
        truth = [r["wall_"+c] for r in test]
        tp = sum(p == 1 and y == 1 for p, y in zip(pred, truth))
        tn = sum(p == 0 and y == 0 for p, y in zip(pred, truth))
        fp = sum(p == 1 and y == 0 for p, y in zip(pred, truth))
        fn = sum(p == 0 and y == 1 for p, y in zip(pred, truth))
        precision = tp/(tp+fp) if tp+fp else None
        recall = tp/(tp+fn) if tp+fn else None
        specificity = tn/(tn+fp) if tn+fp else None
        f1 = 2*precision*recall/(precision+recall) if (
            precision is not None and recall is not None and precision+recall
        ) else None
        reference_events = transitions(test, truth)
        measured_events = transitions(test, pred)
        lags, missed = event_lag(reference_events, measured_events,
                                 max_transition_lag_ms)
        rows.append({
            "channel": c, "precision": precision, "recall": recall,
            "f1": f1, "specificity": specificity,
            "true_positive": tp, "false_positive": fp,
            "false_negative": fn, "true_negative": tn,
            "gt_transitions": len(reference_events),
            "missed_transitions": missed,
            "p95_detection_delay_ms": percentile(lags),
            "threshold_adc": model[c]["threshold"],
            "hysteresis_adc": model[c]["hysteresis_adc"],
        })
    report = {
        "mode": "offline_labelled_wall_detection",
        "train_file": str(train_file), "test_file": str(test_file),
        "train_frames": len(train), "test_frames": len(test),
        "confirm_frames": confirm_frames, "max_transition_lag_ms": max_transition_lag_ms,
        "channels": list(CHANNELS),
        "physical_ground_truth_independent": "USER_MUST_VERIFY",
        "geometry": {
            "L0_R0": "side-facing wall beams",
            "L45_R45": "diagonal front-side beams",
            "L90_R90": "rear-mounted, forward-facing beams",
        },
        "model": model, "results": rows,
        "maze_wall_detection_qualified": False,
        "limitations": (
            "Labels must come from independent wall/edge measurements. "
            "Per-channel classifier is an offline benchmark, not a motor safety cutoff. "
            "An entire maze cell's wall map additionally needs pose/heading gating."
        ),
    }
    write_csv(out / "sensor_confusion.csv", EVENT_FIELDS, rows)
    write_json(out / "wall_detection_report.json", report)
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("preflight", help="Software tests and optional compile; never flash")
    a.add_argument("--build", action="store_true")
    a.add_argument("--out", type=Path, default=ROOT/"results/maze_readiness/preflight")
    b = sub.add_parser("motion", help="Aggregate BLE exported logs; offline only")
    b.add_argument("--logs", type=Path, nargs="+", required=True)
    b.add_argument("--out", type=Path, default=ROOT/"results/maze_readiness/motion")
    b.add_argument("--target-mm", type=float, default=720.0)
    b.add_argument("--tolerance-mm", type=float, default=15.0)
    b.add_argument("--drift-review-mm", type=float, default=30.0)
    c = sub.add_parser("wall-events", help="Holdout per-sensor labelled wall/no-wall benchmark")
    c.add_argument("--train", type=Path, required=True)
    c.add_argument("--test", type=Path, required=True)
    c.add_argument("--out", type=Path, default=ROOT/"results/maze_readiness/wall_events")
    c.add_argument("--confirm-frames", type=int, default=3)
    c.add_argument("--max-transition-lag-ms", type=float, default=250.0)
    args = p.parse_args(argv)
    try:
        if args.cmd == "preflight":
            result = preflight(args.out, args.build)
            print("PREFLIGHT:", "OK" if result["software_checks_ok"] else "FAILED",
                  "|", str(args.out/"preflight.json"))
            return 0 if result["software_checks_ok"] else 2
        if args.cmd == "motion":
            if args.target_mm <= 0 or args.tolerance_mm < 0 or args.drift_review_mm < 0:
                raise ValueError("Target must be positive; review tolerances nonnegative")
            result = analyze_motion(args.logs, args.out, args.target_mm,
                                    args.tolerance_mm, args.drift_review_mm)
            print(f"MOTION: {result['runs']} trials, {result['complete_logs']} complete logs, "
                  f"{result['done_runs']} DONE, {result['review_no_flags']} without review flags")
            print("OUTPUT:", args.out)
            return 0
        result = analyze_wall_events(args.train, args.test, args.out,
                                     args.confirm_frames, args.max_transition_lag_ms)
        print("WALL-EVENTS:", result["test_frames"], "frames, 6 channels")
        for row in result["results"]:
            print(f"  {row['channel']}: F1={row['f1']}, FN={row['false_negative']}, "
                  f"FP={row['false_positive']}, p95_lag={row['p95_detection_delay_ms']}ms")
        print("OUTPUT:", args.out)
        return 0
    except (ValueError, OSError, RuntimeError) as exc:
        print("ERROR:", exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
