#!/usr/bin/env python3
"""Single-command, no-power P0 STOP/Coast/Brake qualification.

Executes all existing source/host model regressions, optionally validates a
bundle of *measured* GPIO/PWM CSV traces, and writes JSON plus a text summary.
NEVER flashes, opens UART, sends START, or powers a motor.

GPIO/PWM captures establish pin-level sequences only. They cannot qualify
TB6612 OUT pins, short-brake current, temperature, or loaded reverse behavior.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import statistics
import subprocess
import sys
from collections import Counter
from pathlib import Path
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parent.parent
CASES = (
    "drive_to_coast", "coast_to_drive", "drive_to_brake",
    "brake_to_coast", "forward_to_reverse", "reverse_to_forward",
)
WHEELS = ("left", "right")
REQUIRED = tuple(f"{case}_{wheel}.csv" for case in CASES for wheel in WHEELS)
SUITE = (
    ("truth_table", ("tools/motor_stop_truth_table_audit.py", "--self-test")),
    ("source_audit", ("tools/motor_stop_truth_table_audit.py",)),
    ("state_transition", ("tools/motor_stop_transition_regression.py",)),
    ("v14_63", ("tools/motor_auto_pipeline.py", "--self-test")),
    ("v16_87", ("tools/motor_auto_pipeline.py", "--self-test", "--high-pwm")),
    ("v17_63_integrated", ("tools/motor_auto_pipeline.py", "--self-test", "--sensor-integration")),
    ("v18_87_integrated", ("tools/motor_auto_pipeline.py", "--self-test", "--sensor-integration", "--high-pwm")),
    ("qualification_gate", ("tools/motor_full_qualification.py", "--self-test")),
)


class CaptureError(ValueError):
    """CSV data cannot establish the requested transition."""


def read_capture(path: Path) -> list[tuple[float, int, int, int]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        assert reader.fieldnames is not None
        if not {"t_us", "in1", "in2", "pwm"}.issubset(reader.fieldnames):
            raise CaptureError("Required CSV columns: t_us,in1,in2,pwm")
        out = []
        for number, row in enumerate(reader, start=2):
            try:
                t = float(row["t_us"])
                digits = tuple(int(row[k]) for k in ("in1", "in2", "pwm"))
            except (TypeError, ValueError) as exc:
                raise CaptureError(f"Bad numeric data at CSV line {number}") from exc
            if not (-1e12 < t < 1e12) or any(v not in (0, 1) for v in digits):
                raise CaptureError(f"Invalid timestamp/logic value at line {number}")
            if out and t <= out[-1][0]:
                raise CaptureError(f"Timestamps not strictly increasing at line {number}")
            out.append((t, *digits))
    if len(out) < 100:
        raise CaptureError("Too few samples; need at least 100")
    elapsed = out[-1][0] - out[0][0]
    if elapsed < 4000:
        raise CaptureError("Capture shorter than 4 ms; cannot establish both settled states")
    gaps = [b[0] - a[0] for a, b in zip(out, out[1:])]
    max_gap = max(gaps)
    if max_gap > 2.0:
        raise CaptureError(f"Sampling gap {max_gap:g} us exceeds 2 us; cannot resolve PWM transients")
    return out


def pin_state(row: tuple[float, int, int, int]) -> str:
    _, in1, in2, pwm = row
    if in1 and not in2:
        return "forward"
    if in2 and not in1:
        return "reverse"
    if in1 and in2:
        return "brake"
    return "coast" if pwm else "low_low_pwm_low"


def inspect_capture(rows: list[tuple[float, int, int, int]], case: str) -> dict:
    if case not in CASES:
        raise ValueError("Unknown P0 test case")
    states = [pin_state(r) for r in rows]
    start, end = {
        "drive_to_coast": (("forward", "reverse"), "coast"),
        "coast_to_drive": (("coast",), "forward"),
        "drive_to_brake": (("forward", "reverse"), "brake"),
        "brake_to_coast": (("brake",), "coast"),
        "forward_to_reverse": (("forward",), "reverse"),
        "reverse_to_forward": (("reverse",), "forward"),
    }[case]
    # Require 1 ms of stable evidence at each boundary, rather than a
    # single favorable sample or a short noisy spike.
    first = [s for row, s in zip(rows, states) if row[0] < rows[0][0] + 1000]
    last = [s for row, s in zip(rows, states) if row[0] > rows[-1][0] - 1000]
    start_ok = len(first) >= 3 and sum(s in start for s in first) / len(first) >= 0.98
    end_ok = len(last) >= 3 and last.count(end) / len(last) >= 0.98
    issues = []
    if not start_ok:
        issues.append(f"Initial 1-ms window is not >=98% {start}")
    if not end_ok:
        issues.append(f"Final 1-ms window is not >=98% {end}")
    # A direction reversal must pass through a neutral interval. Assess
    # the pin command, never mistake a PWM-LOW drive subcycle for Coast.
    neutral_gap_us = None
    if case.endswith("_to_reverse") or case.endswith("_to_forward"):
        old, new = start[0], end
        old_times = [row[0] for row, state in zip(rows, states) if state == old]
        new_times = [row[0] for row, state in zip(rows, states) if state == new]
        if not old_times or not new_times:
            issues.append("Both drive directions not observed")
        else:
            first_new = new_times[0]
            old_before = [t for t in old_times if t < first_new]
            if not old_before:
                issues.append("Opposite drive direction starts before original direction")
            else:
                neutral_gap_us = first_new - old_before[-1]
                if neutral_gap_us < 30000:
                    issues.append(f"Measured reversal interval {neutral_gap_us:g} us < 30000 us")
                if any(s == "brake" for row, s in zip(rows, states)
                       if old_before[-1] < row[0] < first_new):
                    issues.append("Unexpected active brake between reverse directions")
    # Coast requires PWM HIGH in the steady state. During a transition
    # LOW/LOW/PWM LOW can occur while timer preload is being committed;
    # report occurrences rather than labeling them electrically safe.
    unknown = sum(s == "low_low_pwm_low" for s in states)
    if case in ("coast_to_drive", "drive_to_coast", "brake_to_coast"):
        if "coast" not in states:
            issues.append("No defined LOW/LOW/HIGH coast input observed")
    if case in ("coast_to_drive", "drive_to_coast", "drive_to_brake"):
        if not any(s in ("forward", "reverse") and row[3] == 0
                   for row, s in zip(rows, states)):
            issues.append("No PWM LOW chopping observed in drive portion")
        if not any(s in ("forward", "reverse") and row[3] == 1
                   for row, s in zip(rows, states)):
            issues.append("No PWM HIGH drive portion observed")
    if case == "coast_to_drive":
        # If direction becomes active before PWM preload updates, a full
        # 30-kHz period HIGH could energize the bridge at full duty.
        active_rows = [(row[0], row[3]) for row, s in zip(rows, states)
                       if s == "forward"]
        if active_rows:
            first_time = active_rows[0][0]
            later_low = next((t for t, pwm in active_rows if t >= first_time and pwm == 0), None)
            if later_low is None or later_low - first_time > 34:
                issues.append("Initial DRIVE PWM remains high >34 us or no LOW found")
    return {
        "status": "PIN_SEQUENCE_REVIEW" if not issues else "FAIL",
        "start_window_ok": start_ok,
        "end_window_ok": end_ok,
        "samples": len(rows),
        "span_us": round(rows[-1][0] - rows[0][0], 3),
        "sampling_median_us": round(statistics.median(
            b[0] - a[0] for a, b in zip(rows, rows[1:])), 3),
        "state_counts": dict(Counter(states)),
        "low_low_pwm_low_samples": unknown,
        "reverse_gap_us": round(neutral_gap_us, 3) if neutral_gap_us is not None else None,
        "issues": issues,
        "physical_bridge_out_current_qualified": False,
    }


def capture_bundle(folder: Path | None) -> dict:
    if folder is None:
        return {
            "status": "NOT_MEASURED",
            "expected_files": list(REQUIRED),
            "coverage": "0/12",
            "traces": {},
        }
    result: dict = {}
    for filename in REQUIRED:
        path = folder / filename
        if not path.is_file():
            result[filename] = {"status": "MISSING", "issues": ["Required scope CSV not supplied"]}
            continue
        try:
            rows = read_capture(path)
            result[filename] = inspect_capture(rows, filename.rsplit("_", 1)[0])
        except (CaptureError, OSError, UnicodeError, csv.Error) as exc:
            result[filename] = {"status": "FAIL", "issues": [str(exc)]}
    passed = sum(x["status"] == "PIN_SEQUENCE_REVIEW" for x in result.values())
    return {
        "status": "PIN_SEQUENCE_REVIEW" if passed == len(REQUIRED) else "INCOMPLETE_OR_FAIL",
        "expected_files": list(REQUIRED),
        "coverage": f"{passed}/{len(REQUIRED)}",
        "traces": result,
    }


def run_suite() -> list[dict]:
    records: list[dict] = []
    for name, command in SUITE:
        try:
            proc = subprocess.run(
                [sys.executable, str(ROOT / command[0]), *command[1:]],
                cwd=ROOT, capture_output=True, text=True, timeout=120, check=False,
            )
            records.append({
                "id": name, "status": "PASS_OFFLINE" if proc.returncode == 0 else "FAIL",
                "exit_code": proc.returncode, "stdout": proc.stdout.strip()[-3500:],
                "stderr": proc.stderr.strip()[-3500:],
            })
        except (OSError, subprocess.TimeoutExpired) as exc:
            records.append({"id": name, "status": "FAIL", "error": str(exc)})
    return records


def report_text(report: dict) -> str:
    lines = [
        "MICROMOUSE P0 STOP/COAST/BRAKE — MULTI-GATE PIPELINE",
        f"UTC: {report['utc']}",
        f"Source/model qualification: {report['source_model_status']}",
        f"Pin captures: {report['pin_capture']['status']} ({report['pin_capture']['coverage']})",
        "TB6612 physical OUT/current/thermal: BLOCKED — no independent instrumentation evidence",
        "Previously flashed v7 board: not changed; UART START interlock belongs to new firmware",
        "",
        "OFFLINE BATCH:",
    ]
    for item in report["offline"]:
        lines.append(f"  {item['id']}: {item['status']}")
        if item["status"] != "PASS_OFFLINE":
            lines.append("    " + str(item.get("stderr", item.get("error", "")))[:500])
    lines.extend(["", "PIN-LEVEL OSCILLOSCOPE CSV:"])
    for filename in REQUIRED:
        entry = report["pin_capture"]["traces"].get(filename, {})
        lines.append(f"  {filename}: {entry.get('status','NOT_MEASURED')}")
        if entry.get("issues"):
            lines.append("    " + "; ".join(entry["issues"])[:500])
    lines.extend([
        "", "FINAL: " + report["overall"],
        "PASS_OFFLINE is NOT motor electrical approval. No MCU flashed, no UART START sent.",
    ])
    return "\n".join(lines) + "\n"


def run_batch(evidence: Path | None) -> dict:
    offline = run_suite()
    source_ok = all(x["status"] == "PASS_OFFLINE" for x in offline)
    scope = capture_bundle(evidence)
    overall = ("BLOCKED_SOURCE" if not source_ok else
               "FAIL_PIN_CAPTURE" if scope["status"] == "INCOMPLETE_OR_FAIL" else
               "REVIEW_PIN_LEVEL_ONLY" if scope["status"] == "PIN_SEQUENCE_REVIEW" else
               "REVIEW_AWAITING_PIN_CAPTURE")
    return {
        "schema_version": 1, "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "branch": "feature/motor-driver-optimization",
        "execution_mode": "NO_FLASH_NO_UART_NO_MOTOR",
        "source_model_status": "PASS_OFFLINE" if source_ok else "FAIL",
        "offline": offline, "pin_capture": scope, "overall": overall,
        "p0_source_qualified": source_ok,
        "p0_electrical_status": "BLOCKED",
        "safety": {
            "no_automatic_brake_reverse_or_pwm_drive": True,
            "firmware_upload_performed": False,
            "physical_waveform_or_current_measurement_performed_by_script": False,
        },
    }


def self_test() -> None:
    report = run_batch(None)
    assert report["source_model_status"] == "PASS_OFFLINE", [
        (x["id"], x.get("stderr", "")) for x in report["offline"] if x["status"] != "PASS_OFFLINE"
    ]
    assert report["overall"] == "REVIEW_AWAITING_PIN_CAPTURE"
    # Synthesized CSV serves only to exercise the host parser. It is NOT
    # written to the user's measurement folder or used as hardware evidence.
    with TemporaryDirectory(prefix="motor_p0_synthetic_") as temp:
        path = Path(temp) / "drive_to_coast_left.csv"
        # 0-2ms forward PWM chopping, 2-4ms steady Coast at 1us sample.
        with path.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(("t_us", "in1", "in2", "pwm"))
            for t in range(5000):
                writer.writerow((t, 1 if t < 2000 else 0,
                                 0, int(t % 33 < 7) if t < 2000 else 1))
        good = inspect_capture(read_capture(path), "drive_to_coast")
        assert good["status"] == "PIN_SEQUENCE_REVIEW", good
        with path.open("a", encoding="utf-8") as fp:
            fp.write("5000,2,0,1\n")
        try:
            read_capture(path)
        except CaptureError:
            pass
        else:
            raise AssertionError("Bad logic pin level was accepted")
        partial = capture_bundle(Path(temp))
        assert partial["status"] == "INCOMPLETE_OR_FAIL"
        assert partial["coverage"] == "0/12"
    print("PASS: eight offline P0 gates, waveform CSV guards and missing-evidence gate.")
    print("SYNTHETIC WAVEFORMS ARE NOT HARDWARE EVIDENCE. No flash, START or serial access.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run", action="store_true", help="Full no-power batch and JSON/TXT results")
    group.add_argument("--self-test", action="store_true", help="Host-only pipeline regression")
    parser.add_argument("--waveforms", type=Path, help="Folder containing all 12 scope CSV captures")
    parser.add_argument("--output", type=Path, help="Report folder; defaults to timestamped motor_logs/p0_batch")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    out = args.output or ROOT / "motor_logs" / (
        "p0_batch_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    )
    report = run_batch(args.waveforms)
    out.mkdir(parents=True, exist_ok=True)
    (out / "p0_batch_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    summary = report_text(report)
    (out / "p0_batch_summary.txt").write_text(summary, encoding="utf-8")
    print(summary)
    print("Output:", out.resolve())
    # Missing scope captures (without --waveforms) are an expected REVIEW,
    # not a source regression. Supplied malformed/partial captures do fail.
    return 1 if report["overall"] in ("BLOCKED_SOURCE", "FAIL_PIN_CAPTURE") else 0


if __name__ == "__main__":
    raise SystemExit(main())
