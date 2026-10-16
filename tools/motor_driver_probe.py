#!/usr/bin/env python3
"""Inspect a locked motor bench and measure neutral driver calls; never flash or START."""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import hashlib
import json
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parent.parent
COMMANDS = (b"MOTOR PING\n", b"MOTOR DRIVER\n")
RECORDS = ("MOTOR_DRIVER", "MOTOR_DRIVER_REG", "MOTOR_DRIVER_STATE", "MOTOR_DRIVER_BENCH")


def send_command(port, command: bytes) -> None:
    if command not in COMMANDS:
        raise ValueError("Only MOTOR PING and MOTOR DRIVER are allowed")
    port.write(command)


def parse_line(raw: bytes) -> tuple[str, dict]:
    parts = raw.decode("ascii", errors="strict").strip().split(",")
    fields = {}
    for item in parts[1:]:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        if key in fields:
            raise ValueError(f"Duplicate field: {key}")
        try:
            fields[key] = int(value)
        except ValueError:
            fields[key] = value
    return parts[0], fields


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_handshake(profile: dict, ready: dict) -> None:
    require(profile.get("version") in (14, 16, 17, 18), "Unsupported/archived firmware")
    require(ready.get("version") == profile["version"], "Mixed firmware versions")
    require(profile.get("p0_locked") == ready.get("p0_locked") == 1, "Missing P0 lock")
    require(ready.get("state") == 0, "Firmware is not idle")


def validate_snapshot(records: dict) -> None:
    require(all(name in records for name in RECORDS), "Incomplete driver response")
    identity, registers, state, bench = (records[name] for name in RECORDS)
    require(identity.get("revision") in (1, 2, 3), "Unknown driver revision")
    require(isinstance(identity.get("ticks"), int) and identity["ticks"] >= 0, "Invalid scheduler counter")
    require(isinstance(identity.get("core_hz"), int) and 0 < identity["core_hz"] <= 100000000, "Invalid core clock")
    require(identity.get("locked") == 1 and identity.get("state") == 0, "Driver not locked idle")
    clock = identity.get("timer_hz", 0)
    period = identity.get("period", 0)
    require(isinstance(clock, int) and 30000 <= clock <= 100000000, "Unexpected timer clock")
    require(period == clock // 30000, "PWM period does not match 30 kHz source setting")
    expected = {"cr1": 129, "smcr": 0, "dier": 0, "ccmr1": 0x6868, "ccer": 0x11,
                "ccr_l": period, "ccr_r": period}
    if identity.get("revision", 0) >= 2:
        expected["cr2"] = 0
    require(all(registers.get(key) == value for key, value in expected.items()), "TIM2 register mismatch")
    require(isinstance(registers.get("odr"), int) and registers["odr"] & 0xF000 == 0, "Direction asserted")
    require(all(state.get(key) == 0 for key in ("target_l", "target_r", "ramped_l", "ramped_r", "brake")),
            "Non-neutral driver software state")
    require(bench.get("samples") == 256, "Unexpected benchmark sample count")
    minimum, maximum, total = (bench.get(key, 0) for key in ("min_cycles", "max_cycles", "total_cycles"))
    require(all(isinstance(value, int) for value in (minimum, maximum, total)), "Non-integer cycle measurement")
    require(0 < minimum <= maximum and minimum * 256 <= total <= maximum * 256, "Invalid cycle measurement")
    require(bench.get("gaps") == 0 and bench.get("tx_drop") == 0, "Timing gap or UART loss")


def capture(args) -> dict:
    import serial

    output = args.output or ROOT / "motor_logs" / ("driver_probe_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=True)
    report = {"status": "FAIL", "utc": dt.datetime.now(dt.timezone.utc).isoformat(), "port": args.port,
              "vm_reported_volts": args.vm_volts, "vm_measured_by_mcu": False,
              "source_sha256": hashlib.sha256((ROOT / "src/motor.c").read_bytes()).hexdigest(),
              "tx_commands": [], "snapshots": [], "limits": "Locked neutral microbenchmark, not drive/current/thermal qualification"}
    raw_lines = []
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, 115200, 0.2
    port.dtr = port.rts = False

    def send(command):
        send_command(port, command)
        report["tx_commands"].append(command.decode("ascii").strip())

    def receive():
        raw = port.readline()
        if not raw:
            return "", {}
        raw_lines.append(raw)
        return parse_line(raw)

    try:
        with port:
            profile, ready = {}, {}
            deadline, next_ping = time.monotonic() + 15.0, 0.0
            while time.monotonic() < deadline:
                if time.monotonic() >= next_ping:
                    send(COMMANDS[0])
                    next_ping = time.monotonic() + 1.0
                name, fields = receive()
                if name == "MOTOR_AUTO_PROFILE":
                    profile = fields
                elif name == "MOTOR_AUTO_READY":
                    ready = fields
                    validate_handshake(profile, ready)
                    break
            else:
                raise TimeoutError("No complete locked-idle handshake")
            report.update(profile=profile, ready=ready)
            for _ in range(args.samples):
                records = {}
                send(COMMANDS[1])
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    name, fields = receive()
                    require(name != "MOTOR_DRIVER_REJECTED", "Firmware refused driver probe")
                    if name in RECORDS:
                        require(name not in records, "Duplicate driver record")
                        records[name] = fields
                    if name == "MOTOR_DRIVER_BENCH":
                        break
                validate_snapshot(records)
                if args.expect_revision is not None:
                    require(records["MOTOR_DRIVER"].get("revision") == args.expect_revision, "Wrong driver revision")
                report["snapshots"].append(records)
                time.sleep(0.5)
            snapshots = report["snapshots"]
            require(len({item["MOTOR_DRIVER"].get("revision") for item in snapshots}) == 1, "Driver revision changed")
            require(all(later["MOTOR_DRIVER"]["ticks"] > earlier["MOTOR_DRIVER"]["ticks"]
                        for earlier, later in zip(snapshots, snapshots[1:])), "Scheduler stopped or reset")
            require(all(item["MOTOR_DRIVER_BENCH"].get("encoder_l") == snapshots[0]["MOTOR_DRIVER_BENCH"].get("encoder_l")
                        and item["MOTOR_DRIVER_BENCH"].get("encoder_r") == snapshots[0]["MOTOR_DRIVER_BENCH"].get("encoder_r")
                        for item in snapshots), "Encoder movement during neutral probe")
            report["mean_cycles"] = sum(item["MOTOR_DRIVER_BENCH"]["total_cycles"] for item in snapshots) / (256 * len(snapshots))
            report["status"] = "PASS_LOCKED_IDLE"
    except (OSError, ValueError, TimeoutError) as error:
        report["error"] = str(error)
    finally:
        (output / "raw_uart.log").write_bytes(b"".join(raw_lines))
        (output / "driver_probe.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("status", "mean_cycles", "error") if key in report}))
    print("Output:", output)
    return report


def self_test() -> None:
    profile = {"version": 14, "p0_locked": 1}
    ready = {"version": 14, "p0_locked": 1, "state": 0}
    validate_handshake(profile, ready)
    records = {
        "MOTOR_DRIVER": {"revision": 1, "locked": 1, "state": 0, "timer_hz": 96000000,
                         "core_hz": 96000000, "period": 3200, "ticks": 1000},
        "MOTOR_DRIVER_REG": {"cr1": 129, "smcr": 0, "dier": 0, "ccmr1": 0x6868, "ccer": 0x11,
                             "ccr_l": 3200, "ccr_r": 3200, "odr": 0},
        "MOTOR_DRIVER_STATE": {"target_l": 0, "target_r": 0, "ramped_l": 0, "ramped_r": 0, "brake": 0},
        "MOTOR_DRIVER_BENCH": {"samples": 256, "min_cycles": 100, "max_cycles": 200,
                               "total_cycles": 38400, "gaps": 0, "tx_drop": 0},
    }
    validate_snapshot(records)
    revision_two = copy.deepcopy(records)
    revision_two["MOTOR_DRIVER"]["revision"] = 2
    revision_two["MOTOR_DRIVER_REG"]["cr2"] = 0
    validate_snapshot(revision_two)
    revision_three = copy.deepcopy(revision_two)
    revision_three["MOTOR_DRIVER"]["revision"] = 3
    validate_snapshot(revision_three)
    revision_two["MOTOR_DRIVER_REG"]["cr2"] = 1
    try:
        validate_snapshot(revision_two)
    except ValueError:
        pass
    else:
        raise AssertionError("Accepted stale TIM2 CR2")
    for record, field, bad_value in (
        ("MOTOR_DRIVER", "revision", 4),
        ("MOTOR_DRIVER", "locked", 0), ("MOTOR_DRIVER", "state", 1),
        ("MOTOR_DRIVER", "period", 3199), ("MOTOR_DRIVER_REG", "ccr_l", 0),
        ("MOTOR_DRIVER_REG", "odr", 0x1000), ("MOTOR_DRIVER_REG", "dier", 1),
        ("MOTOR_DRIVER_STATE", "target_l", 10), ("MOTOR_DRIVER_STATE", "brake", 1),
        ("MOTOR_DRIVER_BENCH", "min_cycles", 0), ("MOTOR_DRIVER_BENCH", "gaps", 1),
        ("MOTOR_DRIVER_BENCH", "tx_drop", 1),
    ):
        invalid = copy.deepcopy(records)
        invalid[record][field] = bad_value
        try:
            validate_snapshot(invalid)
        except ValueError:
            continue
        raise AssertionError(f"Accepted invalid {record}.{field}")
    for invalid_profile, invalid_ready in (
        ({"version": 7}, {"version": 7, "state": 0}),
        (profile, {"version": 14, "state": 0}),
        (profile, {"version": 14, "p0_locked": 1, "state": 1}),
    ):
        try:
            validate_handshake(invalid_profile, invalid_ready)
        except ValueError:
            continue
        raise AssertionError("Accepted unlocked/old/armed firmware")
    class FakePort:
        def __init__(self):
            self.writes = []

        def write(self, command):
            self.writes.append(command)

    port = FakePort()
    for command in COMMANDS:
        send_command(port, command)
    try:
        send_command(port, b"MOTOR START\n")
    except ValueError:
        pass
    else:
        raise AssertionError("START must be rejected")
    assert port.writes == list(COMMANDS)
    print("PASS: locked-idle UART validation, register checks, cycle checks and command allowlist.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--vm-volts", type=float)
    parser.add_argument("--expect-revision", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not 2 <= args.samples <= 120:
        parser.error("--samples must be between 2 and 120")
    return 0 if capture(args)["status"] == "PASS_LOCKED_IDLE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
