#!/usr/bin/env python3
"""Inspection-only STM32 startup-v4 UART attestation. Never ARM, PULSE, STOP, upload or flash."""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import sys
import time

from motor_startup_pipeline import Capture, HIGH_PROFILE

ROOT = Path(__file__).resolve().parent.parent
ALLOWED_COMMAND = "STARTUP PING"
# PING emits BOTH a PROFILE and a STATUS in the MCU Command() handler.
# STATUS is allowed exactly once and must obey the same neutral invariants.
ALLOWED_FRAMES = frozenset(("STARTUP_PROFILE", "STARTUP_STATUS", "STARTUP_SAMPLE", "STARTUP_HEALTH"))


def audit_frames(frames: list[tuple[str, dict]], minimum_samples: int = 5) -> dict:
    """Pure offline P0 check. Reject incomplete, stale, armed or reset traces."""
    if minimum_samples < 2:
        raise ValueError("At least two idle samples are required")
    profiles = [row for name, row in frames if name == "STARTUP_PROFILE"]
    samples = [row for name, row in frames if name == "STARTUP_SAMPLE"]
    statuses = [row for name, row in frames if name == "STARTUP_STATUS"]
    health = [row for name, row in frames if name == "STARTUP_HEALTH"]
    if not profiles or not samples or len(samples) < minimum_samples:
        raise ValueError("Missing v4 profile or adequate idle samples")
    if any(name not in ALLOWED_FRAMES for name, _ in frames):
        raise ValueError("Unexpected test/ARM/FAULT frame during inspection-only check")
    # The checker sends only one PING. More than one STATUS implies a
    # separate command source or an unintended response: fail closed.
    if len(statuses) != 1:
        raise ValueError("Expected exactly one STARTUP_STATUS acknowledgement for a single PING")
    if any(name == "STARTUP_STATUS" and not any(
        earlier == "STARTUP_PROFILE" for earlier, _ in frames[:i]
    ) for i, (name, _) in enumerate(frames)):
        raise ValueError("STARTUP_STATUS before verified PROFILE")
    # Firmware may print PROFILE at boot and in response to PING, before the sample stream.
    from motor_startup_pipeline import validate_profile, validate_row
    for profile in profiles:
        validate_profile(profile, high_speed=True)
    if len({(p.get("build"), p.get("reset_flags")) for p in profiles}) != 1:
        raise ValueError("Mixed or resetting firmware profiles")
    # UART samples are asynchronous: a solicited PING PROFILE can arrive after
    # an idle sample. Detect resets by firmware identity and monotonic ticks,
    # rather than incorrectly treating a valid late PING as a reboot.
    # Validate the PING acknowledgement as strictly as each periodic sample.
    # In particular, a STATUS from a FAULT/READY/DRIVE state is never benign.
    for sample in samples + statuses:
        validate_row(sample, high_speed=True)
        if sample["state"] != 0 or sample["fault"] or sample["id"] or sample["duty"]:
            raise ValueError("MCU is not disarmed and without prior pulses")
        if (sample["al"], sample["ar"], sample["odr"], sample["cl"], sample["cr"]) != (0, 0, 0, 3200, 3200):
            raise ValueError("Non-neutral MCU outputs")
        if sample["late"] or sample["gaps"] or sample["tx"] or sample["rx"]:
            raise ValueError("TIM11/UART health counter not zero")
    if statuses and (statuses[0]["l"], statuses[0]["r"]) != (samples[0]["l"], samples[0]["r"]):
        raise ValueError("Encoder counters differ between PING and idle telemetry")
    for prev, curr in zip(samples, samples[1:]):
        dt_ms = (curr["ms"] - prev["ms"]) & 0xFFFFFFFF
        ticks = (curr["ticks"] - prev["ticks"]) & 0xFFFFFFFF
        if not 0 < dt_ms <= 200 or abs(ticks - dt_ms) > 2:
            raise ValueError("TIM11 not advancing with millisecond time")
        if (curr["l"], curr["r"]) != (prev["l"], prev["r"]):
            raise ValueError("Encoder moved during VM-disconnected inspection")
    if (samples[-1]["ms"] - samples[0]["ms"]) & 0xFFFFFFFF < 150:
        raise ValueError("Idle observation shorter than 150 ms")
    for row in health:
        if row.get("err", 0) or row.get("timeout", 0) or row.get("age_max", 0) > 15000:
            raise ValueError("Unexpected sensor error while idle")
    return {
        "status": "PASS_INSPECTION_ONLY_IDLE_V4",
        "firmware_version": 4,
        "experimental": True,
        "electrical_qualified": False,
        "idle_samples": len(samples),
        "ping_status_frames": len(statuses),
        "elapsed_ms": (samples[-1]["ms"] - samples[0]["ms"]) & 0xFFFFFFFF,
        "first_tick": samples[0]["ticks"],
        "last_tick": samples[-1]["ticks"],
        "raw_qualification": "NOT_MEASURED",
        "motor_vm": "USER_CONFIRMED_PHYSICALLY_DISCONNECTED",
    }


def run(port_name: str, output: Path, duration: float) -> int:
    import serial  # must not be required in self-test
    if output.exists():
        raise ValueError("Refusing to overwrite an existing evidence folder")
    output.mkdir(parents=True)
    report: dict = {
        "status": "FAIL", "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "port": port_name, "allowed_host_commands": [ALLOWED_COMMAND],
        "flash_attempted": False, "motor_arm_attempted": False,
        "motor_vm": "USER_CONFIRMED_PHYSICALLY_DISCONNECTED",
        "electrical_qualification": "NOT_MEASURED",
    }
    capture = None
    frames: list[tuple[str, dict]] = []
    try:
        with serial.Serial(port_name, 115200, timeout=0.04, write_timeout=0.25) as port:
            capture = Capture(port, output, high_speed=True)
            # Deliberately call only PING; never use the Capture's ARM/STOP flows.
            capture.send(ALLOWED_COMMAND)
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                result = capture.read()
                if result is None:
                    continue
                name, row = result
                if name not in ALLOWED_FRAMES:
                    raise ValueError(f"Unexpected frame: {name}")
                frames.append((name, row))
                if (sum(n == "STARTUP_SAMPLE" for n, _ in frames) >= 6 and
                        any(n == "STARTUP_STATUS" for n, _ in frames)):
                    break
            report.update(audit_frames(frames))
    except Exception as exc:
        report["status"] = "FAIL"
        report["error"] = str(exc)
    finally:
        if capture is not None:
            capture.close()
        raw = output / "raw_uart.log"
        report["raw_uart_sha256"] = hashlib.sha256(raw.read_bytes()).hexdigest() if raw.exists() else None
        report["source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({k: report.get(k) for k in ("status", "firmware_version", "idle_samples", "elapsed_ms", "error")}, indent=2))
    return 0 if report["status"] == "PASS_INSPECTION_ONLY_IDLE_V4" else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--run", action="store_true")
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--confirm-vm-disconnected", action="store_true",
                        help="Physically disconnect TB6612 VM; MCU/UART may stay powered")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.self_test:
        if args.confirm_vm_disconnected or args.output is not None:
            parser.error("--self-test is offline only")
        import unittest
        return int(not unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.discover(str(ROOT / "tests/motor_driver"),
                                                pattern="test_motor_startup_readonly.py")).wasSuccessful())
    if not args.confirm_vm_disconnected:
        parser.error("Physical motor VM disconnect confirmation required; no USB/ST-Link command cuts VM")
    if not 1.0 <= args.seconds <= 10.0:
        parser.error("--seconds must be 1..10")
    output = args.output or ROOT / "motor_logs" / ("startup_readonly_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    try:
        return run(args.port, output, args.seconds)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    sys.exit(main())
