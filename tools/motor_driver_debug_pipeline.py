#!/usr/bin/env python3
"""Motor-only checklist runner: offline tests, builds, locked UART and saved-log metrics.

Never uploads, STARTs, brakes, reverses or unlocks motors. Missing physical
measurements remain explicit review items, never a successful optimization.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import motor_driver_probe as driver_probe
from motor_driver_metrics import MetricOptions, analyze_logs, trace_metrics
import motor_functional_trace_pipeline as functional_trace
import motor_p0_batch_pipeline as p0_batch

ROOT = Path(__file__).resolve().parent.parent
BUILD_ENVS = (
    "motor_pwm_key_bench_96m_trim25", "motor_auto_pipeline_96m_trim25",
    "motor_auto_high_96m_trim25", "motor_auto_sensor_96m_trim25",
    "motor_auto_sensor_high_96m_trim25",
)


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict]) -> None:
    columns = list(dict.fromkeys(column for row in rows for column in row))
    if not columns:
        return
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def command_gate(output: Path, name: str, command: list[str], timeout: int = 180) -> dict:
    try:
        process = subprocess.run(command, cwd=ROOT, capture_output=True, encoding="utf-8",
                                 errors="replace", timeout=timeout, check=False)
        (output / f"{name}.log").write_text(process.stdout + process.stderr, encoding="utf-8")
        return {"id": name, "status": "PASS" if process.returncode == 0 else "FAIL",
                "exit_code": process.returncode, "log": f"{name}.log", "command": command}
    except (OSError, subprocess.TimeoutExpired) as error:
        partial = getattr(error, "stdout", None) or b""
        if isinstance(partial, bytes):
            partial = partial.decode("utf-8", errors="replace")
        (output / f"{name}.log").write_text(partial + "\n" + str(error), encoding="utf-8")
        return {"id": name, "status": "FAIL", "error": str(error), "log": f"{name}.log", "command": command}


def finalize(gates: list[dict], require_complete: bool) -> tuple[str, int]:
    if any(gate["status"].startswith("FAIL") for gate in gates):
        return "FAIL", 1
    pending = any(gate.get("required", True) and gate["status"] != "PASS" for gate in gates)
    return ("REVIEW_PENDING_EVIDENCE", 2 if require_complete else 0) if pending else ("FUNCTIONAL_REVIEW_ONLY", 0)


def parse_args(arguments: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--run", action="store_true")
    mode.add_argument("--self-test", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--port", help="Optional locked-idle probe, for example COM6")
    parser.add_argument("--samples", type=int, default=12)
    trace = parser.add_mutually_exclusive_group()
    trace.add_argument("--capture-trace", action="store_true")
    trace.add_argument("--trace-file", type=Path)
    parser.add_argument("--confirm-vm-disconnected", action="store_true")
    parser.add_argument("--trace-timeout", type=int, default=65)
    parser.add_argument("--build", action="store_true", help="Compile all five motor bench environments; never upload")
    parser.add_argument("--motor-log", type=Path, action="append", default=[])
    parser.add_argument("--waveforms", type=Path, help="Optional already captured pin-level CSV directory")
    parser.add_argument("--min-repeats", type=int, default=10)
    parser.add_argument("--plateau-min-ms", type=int, default=300)
    parser.add_argument("--plateau-start-ms", type=int, default=0)
    parser.add_argument("--plateau-max-drift-pct", type=float, default=5.0)
    parser.add_argument("--max-sample-gap-ms", type=int, default=40)
    parser.add_argument("--min-counts-per-interval", type=int, default=10)
    parser.add_argument("--require-complete", action="store_true", help="Exit 2 when required evidence remains missing/review-only")
    args = parser.parse_args(arguments)
    if args.capture_trace and (not args.port or not args.confirm_vm_disconnected):
        parser.error("--capture-trace requires --port and physically disconnected VM confirmation")
    if args.confirm_vm_disconnected and not args.capture_trace:
        parser.error("--confirm-vm-disconnected is only valid with --capture-trace")
    if not 2 <= args.samples <= 120 or args.min_repeats < 2:
        parser.error("samples must be 2..120; min-repeats must be >=2")
    if (args.plateau_min_ms <= 0 or args.plateau_start_ms < 0 or
        args.max_sample_gap_ms <= 0 or args.min_counts_per_interval < 1 or
        args.trace_timeout < 25 or not math.isfinite(args.plateau_max_drift_pct) or
        args.plateau_max_drift_pct < 0):
        parser.error("Invalid metric window, resolution, drift or trace timeout")
    if args.self_test and (args.port or args.capture_trace or args.trace_file or args.motor_log or
                           args.waveforms or args.build or args.output):
        parser.error("--self-test never builds, writes capture artifacts or accesses hardware")
    if args.output and args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error("Output must be a new or empty directory; existing evidence is never overwritten")
    return args


def report_markdown(report: dict) -> str:
    lines = ["# Motor Driver debug checklist", "", f"UTC: {report['utc']}",
             f"Overall: **{report['overall']}**", "", "| Gate | Status | Detail |",
             "| --- | --- | --- |"]
    for gate in report["gates"]:
        detail = str(gate.get("detail", gate.get("error", gate.get("log", ""))))
        lines.append(f"| {gate['id']} | {gate['status']} | {detail.replace('|', '/').replace(chr(10), ' ')} |")
    lines.extend(["", "Encoder polarity: owner confirmed; not retested.",
                  "Logic-level compatibility: excluded by owner; not a gate.",
                  "Current, temperature and supply droop: NOT_MEASURED.",
                  "No upload, START, brake, reverse, or P0-unlock action is implemented.",
                  "Saved motor logs do not verify the currently flashed image or electrical safety.",
                  "Missing evidence is review, not proof that the driver is fully optimized.", ""])
    return "\n".join(lines)


def run(args: argparse.Namespace) -> tuple[dict, int]:
    output = args.output or ROOT / "motor_logs" / ("driver_debug_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=True)
    gates = [command_gate(output, "native_driver_watchdog", [sys.executable, str(ROOT / "tools/motor_driver_native_test.py")]),
             command_gate(output, "probe_parser", [sys.executable, str(ROOT / "tools/motor_driver_probe.py"), "--self-test"]),
             command_gate(output, "metrics_pipeline_tests", [sys.executable, str(Path(__file__).resolve()), "--self-test"])]
    offline = p0_batch.run_batch(args.waveforms)
    write_json(output / "offline.json", offline)
    gates.append({"id": "offline_source_model", "status": "PASS" if offline["source_model_status"] == "PASS_OFFLINE" else "FAIL",
                  "detail": f"{sum(item['status'] == 'PASS_OFFLINE' for item in offline['offline'])}/{len(offline['offline'])} offline gates"})
    if args.waveforms:
        status = offline["pin_capture"]["status"]
        gates.append({"id": "pin_waveforms", "status": "FAIL" if status == "INCOMPLETE_OR_FAIL" else "MEASURED_PIN_LEVEL_ONLY",
                      "required": False, "detail": status})
    else:
        gates.append({"id": "pin_waveforms", "status": "NOT_MEASURED", "required": False})
    build_artifacts = []
    if args.build:
        for environment in BUILD_ENVS:
            gate = command_gate(output, f"build_{environment}",
                                [sys.executable, "-m", "platformio", "run", "-e", environment], 600)
            gates.append(gate)
            artifacts = {}
            if gate["status"] == "PASS":
                for filename in ("firmware.bin", "firmware.elf"):
                    path = ROOT / ".pio/build" / environment / filename
                    if path.is_file():
                        artifacts[filename] = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                               "size_bytes": path.stat().st_size}
            build_artifacts.append({"environment": environment, "status": gate["status"],
                                    "artifacts": artifacts, "uploaded": False})
    else:
        gates.append({"id": "firmware_builds", "status": "NOT_RUN", "required": False})
    source_ok = not any(gate["status"].startswith("FAIL") for gate in gates)
    probe = None
    if args.port and source_ok:
        try:
            probe = driver_probe.capture(argparse.Namespace(port=args.port, samples=args.samples,
                output=output / "uart", vm_volts=None, expect_revision=3))
            good = probe["status"] == "PASS_LOCKED_IDLE"
            late = [row["MOTOR_DRIVER_BENCH"].get("late", 0) for row in probe["snapshots"]]
            if late and late[-1] != late[0]:
                good = False
            gates.append({"id": "locked_idle_uart", "status": "PASS" if good else "FAIL",
                          "detail": {"status": probe["status"], "mean_cycles": probe.get("mean_cycles"),
                                     "error": probe.get("error"), "snapshot_count": len(probe["snapshots"])}})
        except (ImportError, OSError, ValueError) as error:
            gates.append({"id": "locked_idle_uart", "status": "FAIL", "error": str(error)})
    else:
        gates.append({"id": "locked_idle_uart", "status": "BLOCKED_SOURCE" if args.port else "NOT_MEASURED"})
    trace_result = None
    if args.capture_trace or args.trace_file:
        try:
            if args.capture_trace:
                if (not source_ok or probe is None or probe["status"] != "PASS_LOCKED_IDLE" or
                    any(gate["id"] == "locked_idle_uart" and gate["status"] != "PASS" for gate in gates)):
                    raise ValueError("Locked idle source/probe must pass before trace capture")
                content = functional_trace.capture_device(args.port, args.trace_timeout)
            else:
                content = args.trace_file.read_text(encoding="utf-8-sig")
            (output / "functional_trace_raw.log").write_text(content, encoding="utf-8")
            trace_result = trace_metrics(content)
            good = trace_result["status"] == "PASS_IDLE_REGISTERS"
            good = good and trace_result["encoder_delta_left"] == trace_result["encoder_delta_right"] == 0
            gates.append({"id": "tim11_raw_trace", "status": "PASS" if good else "REVIEW_TIMING_OR_MOVEMENT", "detail": trace_result})
        except (OSError, RuntimeError, ValueError) as error:
            gates.append({"id": "tim11_raw_trace", "status": "FAIL", "error": str(error)})
    else:
        gates.append({"id": "tim11_raw_trace", "status": "NOT_MEASURED",
                      "detail": "Live capture requires physically disconnected VM confirmation"})
    options = MetricOptions(args.min_repeats, args.plateau_min_ms, args.plateau_start_ms,
                            args.plateau_max_drift_pct, args.max_sample_gap_ms, args.min_counts_per_interval)
    motor = analyze_logs(args.motor_log, options)
    for index, path in enumerate(args.motor_log):
        if path.is_file():
            (output / f"motor_input_{index + 1:03d}.log").write_bytes(path.read_bytes())
    raw_samples, speeds = motor.pop("samples"), motor.pop("speed_samples")
    write_csv(output / "motor_samples.csv", raw_samples)
    write_csv(output / "speed_samples.csv", speeds)
    gates.append({"id": "motor_log_integrity", "status": "FAIL" if motor["errors"] else "PASS" if motor["sources"] else "NOT_MEASURED",
                  "detail": motor["errors"] or f"{len(motor['sources'])} saved captures; image identity unverified"})
    present = bool(motor["sources"])
    saved = "MEASURED_SAVED_LOG" if present else "NOT_MEASURED"
    repeats = motor["repeat_statistics"]
    plateau = [trial["steady_speed"]["status"] for trial in motor["trials"]]
    gates.extend([
        {"id": "startup_deadband", "status": saved, "detail": "Startup heuristics only; no automatic deadband assignment"},
        {"id": "holding_deadband", "status": "NOT_MEASURED", "detail": "Existing pulse protocol has no descending-duty hold sweep"},
        {"id": "repeatability", "status": "LOW_REPEATS" if repeats and any(group["status"] == "LOW_REPEATS" for group in repeats) else saved,
         "detail": f"{len(repeats)} groups; minimum {options.min_repeats} distinct trials per group"},
        {"id": "steady_speed", "status": "MEASURED_SAVED_LOG" if plateau and all(status == "MEASURED_PLATEAU_CANDIDATE" for status in plateau) else "INSUFFICIENT_PLATEAU" if plateau else "NOT_MEASURED",
         "detail": {status: plateau.count(status) for status in sorted(set(plateau))}},
        {"id": "rise_slew", "status": "MEASURED_SAVED_LOG" if motor["rise_slew_comparisons"] else "NOT_MEASURED"},
        {"id": "fall_slew", "status": "NOT_MEASURED", "detail": "Fall-rate alternatives are not advertised in current pulse protocol"},
        {"id": "software_stop_tail", "status": saved, "detail": "Neutral register checks, net and absolute observed stop-tail counts"},
        {"id": "live_watchdog", "status": "NOT_MEASURED", "detail": "Actual C watchdog tested on host; powered heartbeat-loss trace not captured"},
        {"id": "single_dual_wheel", "status": "MEASURED_SAVED_LOG" if motor["dual_wheel_comparisons"] else "NOT_MEASURED"},
        {"id": "energized_brake_reverse_stall", "status": "NOT_AUTOMATED", "required": False},
        {"id": "pwm_frequency_comparison", "status": "NOT_MEASURED", "required": False},
    ])
    overall, exit_code = finalize(gates, args.require_complete)
    def git_value(*arguments: str) -> str | None:
        process = subprocess.run(["git", *arguments], cwd=ROOT, capture_output=True, text=True, check=False)
        return process.stdout.strip() if process.returncode == 0 else None
    sources = ("src/motor.c", "src/motor_test.c", "include/motor.h", "include/motor_test.h",
               "platformio.ini", "tools/motor_driver_debug_pipeline.py", "tools/motor_driver_metrics.py",
               "tools/motor_driver_native_test.py", "tools/motor_driver_probe.py", "tools/motor_auto_pipeline.py",
               "tools/motor_functional_trace_pipeline.py", "tools/motor_p0_batch_pipeline.py",
               "tests/motor_driver/test_motor_driver.c", "tests/motor_driver/test_debug_pipeline.py")
    report = {"schema_version": 1, "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "overall": overall, "complete_checklist": overall == "FUNCTIONAL_REVIEW_ONLY",
              "git_head": git_value("rev-parse", "HEAD"), "git_branch": git_value("branch", "--show-current"),
              "tracked_diff_sha256": hashlib.sha256((git_value("diff", "--", "src", "include", "tools", "tests", ".github") or "").encode()).hexdigest(),
              "host_source_sha256": {source: hashlib.sha256((ROOT / source).read_bytes()).hexdigest() for source in sources},
              "flashed_image_hash": None, "encoder_polarity": "USER_CONFIRMED_NOT_RETESTED",
              "python_version": sys.version, "build_artifacts": build_artifacts,
              "logic_level_compatibility": "EXCLUDED_BY_OWNER", "current_temperature_supply_droop": "NOT_MEASURED",
              "safety": {"firmware_uploaded": False, "motor_started": False, "p0_lock_changed": False,
                         "vm_disconnected_confirmation": args.confirm_vm_disconnected,
                         "serial_command_allowlist": ["MOTOR PING", "MOTOR DRIVER", "MOTOR TRACE"]},
              "metric_options": asdict(options), "gates": gates, "trace": trace_result, "motor": motor}
    write_json(output / "report.json", report)
    (output / "report.md").write_text(report_markdown(report), encoding="utf-8")
    print(f"{overall}: {output.resolve()}")
    return report, exit_code


def main() -> int:
    args = parse_args()
    if args.self_test:
        return subprocess.run([sys.executable, "-m", "unittest", "discover", "-s",
            str(ROOT / "tests/motor_driver"), "-p", "test_debug_pipeline.py", "-v"], cwd=ROOT, check=False).returncode
    return run(args)[1]


if __name__ == "__main__":
    raise SystemExit(main())
