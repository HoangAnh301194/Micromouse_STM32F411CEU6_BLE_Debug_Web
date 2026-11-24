#!/usr/bin/env python3
"""Headless automated Motion Control test runner over Classic SPP.

Shares MotionSppClient, protocol, CSV logger, metrics analyzer, and plotter
with tools/classic_spp_dashboard.py. Never opens a second COM connection.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from typing import Any

from motion_experiment import (
    MotionCsvLogger,
    MotionSppClient,
    analyze_motion_csv,
    detect_spp_outgoing_port,
    generate_motion_plots,
    get_git_commit_sha,
    list_serial_ports,
    write_summary_md,
)

ROOT_DIR = Path(__file__).resolve().parent.parent
RESULTS_BASE = ROOT_DIR / "results" / "motion_test"


def run_single_experiment(
    client: MotionSppClient,
    port: str,
    test_name: str,
    test_id: str,
    verb: str,
    args: tuple[Any, ...],
    expected_timeout_s: float,
    pwm_max_pm: float = 300.0,
    extra_params: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Execute one bounded motion experiment with CSV logging, metrics & plots."""
    client.send_emergency_stop()
    time.sleep(0.20)

    ack_rst = client.send_command("RESET_ENC", timeout_s=2.0)
    if ack_rst.get("kind") != "ACK":
        return {"test_name": test_name, "verdict": "BLOCKED", "reason": f"RESET_ENC failed: {ack_rst}"}

    gains = client.read_gains(timeout_s=2.0)
    metadata = {
        "test_name": test_name,
        "test_id": test_id,
        "git_commit": get_git_commit_sha(ROOT_DIR),
        "port": port,
        "command": [verb, *args],
        "gains_confirmed": gains,
        "pwm_limit_pm": pwm_max_pm,
    }
    if extra_params:
        metadata.update(extra_params)

    logger = MotionCsvLogger(RESULTS_BASE, test_name, test_id, metadata)
    latest_telem: dict[str, Any] | None = None
    fault_seen = False

    def _on_line(line: str) -> None:
        logger.log_raw(line)

    def _on_telem(t: dict[str, Any]) -> None:
        nonlocal latest_telem
        latest_telem = t
        logger.log_telemetry(t)

    def _on_ext(ext: dict[str, Any]) -> None:
        logger.update_ext(ext)

    prev_on_line = client.on_line
    prev_on_telem = client.on_telemetry
    prev_on_ext = client.on_ext

    client.on_line = _on_line
    client.on_telemetry = _on_telem
    client.on_ext = _on_ext

    finish_result = "UNKNOWN"
    stop_latency_ms: float | None = None

    try:
        client.clear_done_event()
        client.start_heartbeat(0.30)
        ack = client.send_command(verb, *args, timeout_s=2.0)
        if ack.get("kind") != "ACK":
            finish_result = f"NACK:{ack.get('status', 'ERR')}"
            logger.close({"finish_result": finish_result})
            return {"test_name": test_name, "verdict": "FAIL", "reason": finish_result, "run_dir": str(logger.run_dir)}

        done_evt = client.wait_done_event(expected_timeout_s)
        if done_evt is not None:
            finish_result = done_evt.get("fields", {}).get("RES", "DONE")
        else:
            # Explicit safety STOP if event didn't arrive within expected timeout
            t_stop = time.monotonic()
            client.send_emergency_stop()
            done_evt2 = client.wait_done_event(1.5)
            if done_evt2 is not None:
                stop_latency_ms = (time.monotonic() - t_stop) * 1000.0
                finish_result = done_evt2.get("fields", {}).get("RES", "HOST_STOPPED")
            else:
                finish_result = "HOST_TIMEOUT"

        # Capture brief trailing idle samples (0.3s) to verify PWM=0 after finish
        time.sleep(0.30)
    finally:
        client.stop_heartbeat()
        client.send_emergency_stop()
        time.sleep(0.15)
        client.on_line = prev_on_line
        client.on_telemetry = prev_on_telem
        client.on_ext = prev_on_ext
        logger.close({"finish_result": finish_result})

    metrics = analyze_motion_csv(
        logger.csv_path,
        pwm_max_pm=pwm_max_pm,
        stop_latency_ms=stop_latency_ms,
        finish_result=finish_result,
    )
    (logger.run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    generate_motion_plots(logger.csv_path, logger.run_dir, title_prefix=test_name)
    write_summary_md(logger.run_dir, test_name, metadata, metrics)

    # Evaluate safety & preliminary acceptance checks
    checks = {
        "coverage_ge_95": metrics["coverage_pct"] >= 95.0,
        "no_fault": metrics["fault_count"] == 0 and finish_result != "FAULT",
        "clean_finish": finish_result in ("SUCCESS", "ABORTED"),
        "zero_pwm_when_ref_zero": (
            metrics["left"].get("nonzero_pwm_when_ref_zero", 0) == 0
            and metrics["right"].get("nonzero_pwm_when_ref_zero", 0) == 0
        ),
    }

    # Check steady-state error <= 5% and overshoot <= 10% when applicable
    for side in ("left", "right"):
        m = metrics[side]
        ss = m.get("steady_state_error_pct")
        if isinstance(ss, (int, float)):
            checks[f"{side}_ss_err_le_5pct"] = ss <= 5.0
        ov = m.get("overshoot_pct")
        if isinstance(ov, (int, float)):
            checks[f"{side}_overshoot_le_10pct"] = ov <= 10.0

    verdict = "PASS" if all(checks.values()) else "FAIL"
    return {
        "test_name": test_name,
        "test_id": test_id,
        "verdict": verdict,
        "finish_result": finish_result,
        "checks": checks,
        "metrics": metrics,
        "run_dir": str(logger.run_dir),
    }


def run_suite_b_connectivity(client: MotionSppClient, port: str) -> dict[str, Any]:
    """Stage B: Hardware connectivity, MPU ready, idle telemetry, STOP, gains read."""
    client.send_emergency_stop()
    time.sleep(0.25)

    # Wait up to 12s for MPU calibration ready
    ready = False
    ping_ack: dict[str, Any] = {}
    for _ in range(24):
        ping_ack = client.send_command("PING", timeout_s=2.0)
        fields = ping_ack.get("fields", {})
        if fields.get("READY") == "1" and fields.get("STATE") == "0":
            ready = True
            break
        time.sleep(0.5)

    if not ready:
        return {"test_name": "stage_b_connectivity", "verdict": "BLOCKED", "reason": f"MCU not ready: {ping_ack}"}

    gains = client.read_gains(timeout_s=2.0)
    # Idle capture for 2.0s
    idle_samples: list[dict[str, Any]] = []
    prev_t = client.on_telemetry
    client.on_telemetry = lambda t: idle_samples.append(t)
    time.sleep(2.0)
    client.on_telemetry = prev_t

    stop_t0 = time.monotonic()
    stop_ack = client.send_command("STOP", timeout_s=2.0)
    stop_lat_ms = (time.monotonic() - stop_t0) * 1000.0

    nonzero_idle_pwm = sum(
        1 for s in idle_samples if abs(s["pwm_cmd_l_pm"]) > 0 or abs(s["pwm_cmd_r_pm"]) > 0
    )
    verdict = "PASS" if (len(idle_samples) >= 70 and nonzero_idle_pwm == 0 and stop_ack.get("kind") == "ACK") else "FAIL"
    return {
        "test_name": "stage_b_connectivity",
        "verdict": verdict,
        "port": port,
        "ping": ping_ack,
        "gains": gains,
        "idle_samples": len(idle_samples),
        "idle_rate_hz": round(len(idle_samples) / 2.0, 1),
        "nonzero_idle_pwm": nonzero_idle_pwm,
        "stop_latency_ms": round(stop_lat_ms, 1),
    }


def run_suite_c_wheel_pi(client: MotionSppClient, port: str, pwm_limit: float = 300.0) -> list[dict[str, Any]]:
    """Stage C: No-load Wheel PI experiments C1-C9."""
    pmax = max(50.0, min(550.0, float(pwm_limit)))
    cases = [
        ("C1_left_500pps", "C1", "WTEST", (500, 0, 1.5, pmax, 0), 3.0, pmax),
        ("C2_right_500pps", "C2", "WTEST", (0, 500, 1.5, pmax, 0), 3.0, pmax),
        ("C3_both_1000pps", "C3", "WTEST", (1000, 1000, 1.5, pmax, 0), 3.0, pmax),
        ("C4_both_1500pps", "C4", "WTEST", (1500, 1500, 1.5, pmax, 0), 3.0, pmax),
        ("C5_both_2000pps", "C5", "WTEST", (2000, 2000, 1.5, pmax, 0), 3.0, pmax),
        ("C7_ramp_1200pps", "C7", "WTEST", (1200, 1200, 1.8, pmax, 1), 3.5, pmax),
        ("C8_diff_800_1600pps", "C8", "WTEST", (800, 1600, 1.5, pmax, 0), 3.0, pmax),
    ]
    results: list[dict[str, Any]] = []
    for name, tid, verb, args, tout, pm in cases:
        res = run_single_experiment(client, port, name, tid, verb, args, tout, pwm_max_pm=pm)
        results.append(res)
        time.sleep(0.5)
        if res.get("finish_result") == "FAULT":
            break

    # C9: Start -> Stop -> Start test
    res_c9 = run_single_experiment(client, port, "C9_start_stop_start", "C9", "WTEST", (1000, 1000, 1.2, pmax, 0), 2.5, pwm_max_pm=pmax)
    results.append(res_c9)
    return results


def run_suite_d_straight(client: MotionSppClient, port: str) -> list[dict[str, Any]]:
    """Stage D: Lifted-wheel Straight Profile tests (165mm, 330mm, hold_heading=0)."""
    cases = [
        ("D1_straight_165mm_nolh", "D1", "STRAIGHT", (165.0, 100.0, 60.0, 0.0, 300.0, 300.0, 120.0, 0), 5.0),
        ("D2_straight_330mm_nolh", "D2", "STRAIGHT", (330.0, 100.0, 60.0, 0.0, 300.0, 300.0, 120.0, 0), 7.0),
    ]
    results = []
    for name, tid, verb, args, tout in cases:
        results.append(run_single_experiment(client, port, name, tid, verb, args, tout, pwm_max_pm=1000.0))
        time.sleep(0.5)
    return results


def run_suite_e_vel_and_qstep(client: MotionSppClient, port: str) -> list[dict[str, Any]]:
    """Stage E: Timed Velocity angular kinematics & QuickStep LOW (never auto-run HIGH)."""
    results = []
    results.append(
        run_single_experiment(
            client, port, "E1_vel_arc_left_low", "E1",
            "VEL", (100.0, 90.0, 0, 0.0, 1.5), 3.0, pwm_max_pm=1000.0,
        )
    )
    time.sleep(0.5)
    results.append(
        run_single_experiment(
            client, port, "E2_qstep_low", "E2",
            "QSTEP", (0,), 6.5, pwm_max_pm=1000.0,
        )
    )
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description="Headless SPP Motion Control automated test runner.")
    parser.add_argument("--list-ports", action="store_true", help="List available COM ports and exit")
    parser.add_argument("--port", default=None, help="Bluetooth Outgoing COM port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--suite", choices=["B", "C", "D", "E", "all"], default="B", help="Experiment suite to run")
    parser.add_argument("--pwm-limit", type=float, default=300.0, help="PWM ceiling in permille for Stage C (default: 300, max 550)")
    parser.add_argument("--report", default=None, help="Optional JSON report output path")
    args = parser.parse_args()

    if args.list_ports:
        for p in list_serial_ports():
            print(f"{p['device']:8s} | {p['direction']:8s} | {p['description']}")
        return 0

    port = detect_spp_outgoing_port(args.port)
    if not port:
        print("[BLOCKED] Could not auto-detect Bluetooth Outgoing COM port. Pass --port COMx.", file=sys.stderr)
        return 2

    client = MotionSppClient(on_error=lambda msg: print(f"[SPP ERR] {msg}", file=sys.stderr, flush=True))
    print(f"[SPP] Connecting to {port} @ {args.baud}...", flush=True)
    connected = False
    for attempt in range(2):
        try:
            client.connect(port, args.baud, timeout_s=12.0)
            connected = True
            break
        except Exception as exc:
            print(f"[SPP] Attempt {attempt + 1} failed: {exc}", file=sys.stderr, flush=True)
            time.sleep(2.0)
    if not connected:
        print(f"[BLOCKED] Connection failed on {port}", file=sys.stderr)
        return 2

    all_results: list[dict[str, Any]] = []
    try:
        res_b = run_suite_b_connectivity(client, port)
        all_results.append(res_b)
        print(f"[STAGE B] {res_b['verdict']} ({res_b.get('idle_rate_hz', 0)} Hz, idle_pwm={res_b.get('nonzero_idle_pwm')})", flush=True)
        if res_b["verdict"] != "PASS" and args.suite != "B":
            print("[ABORT] Stage B did not pass; halting motor tests.", file=sys.stderr)
            return 1

        if args.suite in ("C", "all"):
            for r in run_suite_c_wheel_pi(client, port, pwm_limit=args.pwm_limit):
                all_results.append(r)
                print(f"[{r.get('test_id')}] {r['verdict']} -> {r.get('run_dir', '')}", flush=True)

        if args.suite in ("D", "all"):
            for r in run_suite_d_straight(client, port):
                all_results.append(r)
                print(f"[{r.get('test_id')}] {r['verdict']} -> {r.get('run_dir', '')}", flush=True)

        if args.suite in ("E", "all"):
            for r in run_suite_e_vel_and_qstep(client, port):
                all_results.append(r)
                print(f"[{r.get('test_id')}] {r['verdict']} -> {r.get('run_dir', '')}", flush=True)
    finally:
        client.send_emergency_stop()
        time.sleep(0.15)
        client.disconnect()

    report_path = Path(args.report) if args.report else (
        RESULTS_BASE / f"suite_{args.suite}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(all_results, indent=2), encoding="utf-8")
    print(f"[DONE] Suite report saved: {report_path}", flush=True)

    overall_ok = all(r["verdict"] == "PASS" for r in all_results)
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
