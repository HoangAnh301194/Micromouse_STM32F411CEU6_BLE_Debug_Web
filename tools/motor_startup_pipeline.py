#!/usr/bin/env python3
"""Bounded, forward-only encoder startup experiment; never tests full PWM."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from motor_driver_metrics import distribution

ROOT = Path(__file__).resolve().parent.parent
ENVIRONMENT = "motor_startup_96m_trim25"
PROFILE = dict(version=2, cap=200, pulse_ms=250, rest_ms=750, quiet_ms=300,
               link_ms=400, max_pulses=128, session_ms=180000, rise=20, fall=30,
               deadband=0, wdg_ms=500, core_hz=96000000, pwm_hz=30000)


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def parse_frame(raw: bytes) -> tuple[str, dict]:
    text = raw.decode("ascii", errors="strict").strip()
    parts = text.split(",")
    fields = {}
    for part in parts[1:]:
        key, value = part.split("=", 1)
        if key in fields:
            raise ValueError("Duplicate UART field")
        try:
            fields[key] = int(value)
        except ValueError:
            fields[key] = value
    return parts[0], fields


def validate_profile(profile: dict) -> None:
    if any(profile.get(key) != value for key, value in PROFILE.items()):
        raise ValueError("Unknown startup firmware or safety limits")


def validate_row(row: dict, stopping: bool = False) -> None:
    required = {"id", "state", "fault", "ms", "t", "rest", "q", "wheel", "duty",
                "l", "r", "al", "ar", "cl", "cr", "odr", "ticks", "late", "gaps", "tx", "rx"}
    if not required <= row.keys() or any(type(row[key]) is not int for key in required):
        raise ValueError("Incomplete/noninteger startup sample")
    if row["state"] not in range(5) or row["wheel"] not in (0, 1):
        raise ValueError("Invalid state/wheel")
    if (row["state"] == 4) != (row["fault"] != 0):
        raise ValueError("Fault/state mismatch")
    if any(row[key] for key in ("gaps", "tx", "rx")):
        raise ValueError("Timing gap or UART error")
    if row["fault"] and not (stopping and row["fault"] == 5):
        raise ValueError(f"Firmware fault {row['fault']}")
    if not 0 <= row["id"] <= 128 or not 0 <= row["duty"] <= 200 or not 0 <= row["t"] <= 250:
        raise ValueError("Pulse bounds exceeded")
    for wheel in ("l", "r"):
        applied = row["a" + wheel]
        expected = applied * 3200 // 1000 if applied else 3200
        if not 0 <= applied <= row["duty"] or row["c" + wheel] != expected:
            raise ValueError("PWM/register contract mismatch")
    expected_odr = (0x2000 if row["al"] else 0) | (0x8000 if row["ar"] else 0)
    if row["odr"] != expected_odr:
        raise ValueError("Direction/register contract mismatch")
    if row["state"] != 2:
        if row["al"] or row["ar"] or row["odr"]:
            raise ValueError("Non-neutral outputs outside pulse")
    else:
        passive = "r" if row["wheel"] == 0 else "l"
        if row["a" + passive] or abs(row[passive]) > 2:
            raise ValueError("Passive wheel changed")


def trial_metrics(rows: list[dict], wheel: str) -> dict:
    if len(rows) < 8 or rows[0]["t"] > 5 or rows[-1]["t"] != 250:
        raise ValueError("Missing pulse endpoints/samples")
    passive = "r" if wheel == "l" else "l"
    positive = 0
    for earlier, later in zip(rows, rows[1:]):
        elapsed = later["t"] - earlier["t"]
        period = (later["ms"] - earlier["ms"]) & 0xFFFFFFFF
        if not 0 < elapsed <= 40 or abs(period - elapsed) > 2:
            raise ValueError("Missing, duplicated or inconsistent pulse timestamps")
        delta = later[wheel] - earlier[wheel]
        if delta < 0 or abs(later[passive]) > 2:
            raise ValueError("Reverse/noisy counts or passive wheel movement")
        positive += delta > 0
    counts = rows[-1][wheel]
    anchor = max((row for row in rows if row["t"] <= 150), key=lambda row: row["t"])
    tail = counts - anchor[wheel]
    return {"counts": counts, "first_motion": counts > 0,
            "sustained_start": counts >= 5 and positive >= 3 and tail >= 2,
            "positive_intervals": positive, "last_100ms_counts": tail,
            "first_five_counts_observed_ms": next((row["t"] for row in rows if row[wheel] >= 5), None)}


class Capture:
    def __init__(self, port, output: Path):
        self.port = port
        self.raw = (output / "raw_uart.log").open("wb")
        self.commands = (output / "commands.log").open("w", encoding="utf-8")
        self.armed = self.stopping = False
        self.next_keep = 0.0
        self.profile = None
        self.latest = None
        self.records = []
        self.baseline_late = None
        self.watchdog_pending = self.watchdog_ack = self.watchdog_observed = False

    def send(self, command: str) -> None:
        if command not in ("STARTUP PING", "STARTUP ARM", "STARTUP KEEP", "STARTUP STOP", "STARTUP WATCHDOG"):
            parts = command.split()
            if (len(parts) != 5 or parts[:2] != ["STARTUP", "PULSE"] or parts[3] not in ("l", "r") or
                not parts[2].isdigit() or not parts[4].isdigit() or not 1 <= int(parts[2]) <= 128 or
                not 5 <= int(parts[4]) <= 200 or int(parts[4]) % 5):
                raise ValueError("Command outside startup-only allowlist")
        payload = (command + "\n").encode("ascii")
        if self.port.write(payload) != len(payload):
            raise OSError("Incomplete UART command")
        self.commands.write(f"{time.monotonic():.6f}\t{command}\n")
        self.commands.flush()

    def read(self) -> tuple[str, dict] | None:
        if self.armed and not self.stopping and time.monotonic() >= self.next_keep:
            self.send("STARTUP KEEP")
            self.next_keep = time.monotonic() + 0.08
        raw = self.port.readline()
        if not raw:
            return None
        self.raw.write(f"{time.monotonic():.6f}\t".encode("ascii") + raw)
        self.raw.flush()
        if self.watchdog_pending and self.watchdog_ack:
            marker = raw.find(b"STARTUP_PROFILE,")
            if marker < 0:
                return None
            raw = raw[marker:]
        if not raw.endswith(b"\n"):
            raise ValueError("Truncated UART frame")
        if not raw.startswith(b"STARTUP_"):
            raise ValueError("Unexpected firmware/frame")
        name, row = parse_frame(raw)
        if name == "STARTUP_PROFILE":
            if self.armed:
                raise ValueError("Unexpected firmware reset during armed session")
            validate_profile(row)
            self.profile = row
            if self.watchdog_pending and self.watchdog_ack:
                if not row.get("reset_flags", 0) & (1 << 29):
                    raise ValueError("Watchdog probe reset not confirmed by RCC flag")
                self.watchdog_observed = True
                self.watchdog_pending = False
                self.latest = None
        elif name == "STARTUP_WATCHDOG" and self.watchdog_pending and row == {"requested": 1}:
            self.watchdog_ack = True
        elif name in ("STARTUP_SAMPLE", "STARTUP_STATUS", "STARTUP_BEGIN", "STARTUP_END", "STARTUP_FAULT"):
            validate_row(row, self.stopping)
            if self.armed and self.baseline_late is not None and row["late"] != self.baseline_late:
                raise ValueError("TIM11 late counter increased")
            self.latest = row
            self.records.append((name, row))
        else:
            raise ValueError(f"Rejected/unknown device response: {name} {row}")
        return name, row

    def wait(self, predicate, timeout: float = 3.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read()
            if predicate():
                return
        raise TimeoutError("Startup device response/rest timeout")

    def pulse(self, wheel: str, duty: int) -> dict:
        self.wait(lambda: self.latest is not None and self.latest["state"] == 1 and self.latest["q"] >= 300)
        sequence = self.latest["id"] + 1
        offset = len(self.records)
        self.send(f"STARTUP PULSE {sequence} {wheel} {duty}")
        self.wait(lambda: any(name == "STARTUP_END" and row["id"] == sequence
                             for name, row in self.records[offset:]))
        selected = [(name, row) for name, row in self.records[offset:] if row["id"] == sequence]
        if sum(name == "STARTUP_BEGIN" for name, row in selected) != 1:
            raise ValueError("Missing/duplicate pulse acknowledgement")
        rows = []
        for name, row in selected:
            if name not in ("STARTUP_BEGIN", "STARTUP_END") and row["state"] != 2:
                continue
            if row["wheel"] != (0 if wheel == "l" else 1) or row["duty"] != duty:
                raise ValueError("Pulse identity mismatch")
            if rows and row["t"] == rows[-1]["t"]:
                if (row["l"], row["r"]) != (rows[-1]["l"], rows[-1]["r"]):
                    raise ValueError("Different counts at duplicate elapsed time")
                continue
            rows.append(row)
        result = {"id": sequence, "wheel": wheel, "duty_permille": duty,
                  **trial_metrics(rows, wheel), "samples": rows}
        print(f"{wheel.upper()} {duty / 10:.1f}%: {result['counts']} counts; sustained={result['sustained_start']}", flush=True)
        return result

    def stop(self) -> bool:
        self.stopping = True
        self.send("STARTUP STOP")
        self.wait(lambda: self.latest is not None and self.latest["state"] == 4 and self.latest["fault"] == 5, 1.5)
        return True

    def close(self) -> None:
        self.raw.close()
        self.commands.close()


def parse_args(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--run", action="store_true")
    parser.add_argument("--upload", action="store_true")
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--vm-volts", type=float)
    parser.add_argument("--confirm-wheels-elevated", action="store_true")
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(arguments)
    if args.run and (not args.confirm_wheels_elevated or args.vm_volts is None or not 0 < args.vm_volts <= 8.4):
        parser.error("Run requires elevated-wheel confirmation and user-reported VM in (0, 8.4] V")
    if not 10 <= args.repeats <= 20:
        parser.error("repeats must be 10..20")
    if args.self_test and (args.upload or args.confirm_wheels_elevated or args.vm_volts is not None or args.output):
        parser.error("Self-test never builds, uploads or accesses hardware")
    if args.output and args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error("Never overwrite existing evidence")
    return args


def run(args) -> int:
    import serial
    output = args.output or ROOT / "motor_logs" / ("startup_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=True)
    report = {"status": "FAIL", "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "environment": ENVIRONMENT, "vm_volts_user_reported": args.vm_volts,
              "load": "wheels_elevated_user_confirmed", "current_temperature_supply_droop": "NOT_MEASURED",
              "encoder_polarity": "USER_CONFIRMED_NOT_RETESTED", "trials": [], "wheels": {},
              "upload_requested": args.upload, "stop_confirmed": False,
              "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()}
    sources = ("src/motor.c", "src/motor_test.c", "src/motor_startup.c", "src/motor_startup_bench.c",
               "include/motor_startup.h", "src/main.c", "src/control.c", "src/uart.c", "include/uart.h",
               "platformio.ini", "tools/motor_startup_pipeline.py")
    report["source_sha256"] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in sources}
    client = None
    try:
        checks = [("native", [sys.executable, str(ROOT / "tools/motor_driver_native_test.py")]),
                  ("self_test", [sys.executable, str(Path(__file__).resolve()), "--self-test"])]
        if args.upload:
            checks.extend((name, [sys.executable, "-m", "platformio", "run", "-e", ENVIRONMENT, *extra])
                          for name, extra in (("build", []), ("upload", ["-t", "upload"])))
        for name, command in checks:
            process = subprocess.run(command, cwd=ROOT, capture_output=True, encoding="utf-8", errors="replace", timeout=600)
            (output / f"{name}.log").write_text(process.stdout + process.stderr, encoding="utf-8")
            if process.returncode:
                raise RuntimeError(f"{name} failed before UART arming")
        image = ROOT / ".pio/build" / ENVIRONMENT / "firmware.bin"
        report["built_image_sha256"] = hashlib.sha256(image.read_bytes()).hexdigest() if image.exists() else None
        with serial.Serial(args.port, 115200, timeout=0.04, write_timeout=0.25) as port:
            client = Capture(port, output)
            try:
                client.send("STARTUP PING")
                client.wait(lambda: client.profile is not None and client.latest is not None and
                            client.latest["state"] == 0 and client.latest["q"] >= 300, 5.0)
                report["device_profile"] = client.profile
                client.watchdog_pending = True
                client.send("STARTUP WATCHDOG")
                client.wait(lambda: client.watchdog_observed and client.latest is not None and
                            client.latest["state"] == 0 and client.latest["q"] >= 300, 3.0)
                report["watchdog_idle_reset_observed"] = client.watchdog_observed
                report["device_profile_after_watchdog"] = client.profile
                client.baseline_late = client.latest["late"]
                client.send("STARTUP ARM")
                client.armed = True
                client.wait(lambda: client.latest["state"] == 1)
                for wheel in ("l", "r"):
                    found = False
                    for duty in range(5, 201, 5):
                        trial = client.pulse(wheel, duty)
                        trial["phase"] = "search"
                        report["trials"].append(trial)
                        write_json(output / "report.json", report)
                        if not trial["sustained_start"]:
                            continue
                        confirmations = []
                        for repeat in range(args.repeats):
                            trial = client.pulse(wheel, duty)
                            trial.update(phase="confirmation", repeat=repeat + 1)
                            confirmations.append(trial)
                            report["trials"].append(trial)
                            write_json(output / "report.json", report)
                        successful = sum(trial["sustained_start"] for trial in confirmations)
                        if successful == args.repeats:
                            report["wheels"][wheel] = {"candidate_permille": duty, "successes": successful,
                                "attempts": args.repeats, "count_statistics": distribution([trial["counts"] for trial in confirmations]),
                                "first_five_counts_statistics_ms": distribution([trial["first_five_counts_observed_ms"] for trial in confirmations])}
                            found = True
                            break
                    if not found:
                        report["wheels"][wheel] = {"status": "NOT_FOUND_WITHIN_20_PERCENT"}
                client.wait(lambda: client.latest["state"] == 1)
                report["status"] = "MEASURED_STARTUP_CANDIDATES" if all("candidate_permille" in value for value in report["wheels"].values()) else "REVIEW_STARTUP_NOT_FOUND"
            except Exception as error:
                report["experiment_error"] = str(error)
                raise
            finally:
                try:
                    report["stop_confirmed"] = client.stop()
                except Exception as error:
                    report["stop_error"] = str(error)
                    raise
                finally:
                    client.close()
    except Exception as error:
        report["status"] = "FAIL"
        report["error"] = str(error)
    finally:
        report["limitations"] = "Startup under reported elevated-wheel conditions only; not holding PWM, loaded calibration, current/thermal qualification or full-speed approval. Driver deadband unchanged."
        for wheel, result in report["wheels"].items():
            measured = [trial for trial in report["trials"] if trial["wheel"] == wheel]
            result["first_motion_observed_permille"] = min((trial["duty_permille"] for trial in measured if trial["first_motion"]), default=None)
            result["lowest_sustained_observed_permille"] = min((trial["duty_permille"] for trial in measured if trial["sustained_start"]), default=None)
        raw = output / "raw_uart.log"
        report["raw_uart_sha256"] = hashlib.sha256(raw.read_bytes()).hexdigest() if raw.exists() else None
        write_json(output / "report.json", report)
        columns = ("id", "wheel", "phase", "duty_permille", "counts", "first_motion", "sustained_start",
                   "positive_intervals", "last_100ms_counts", "first_five_counts_observed_ms")
        with (output / "trials.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(report["trials"])
        lines = ["# Startup encoder experiment", "", f"Status: {report['status']}",
                 f"STOP confirmed: {report['stop_confirmed']}",
                 f"VM: {args.vm_volts} V (user reported, not measured)",
                 f"Trials: {len(report['trials'])}", "", "| Wheel | Candidate PWM | Confirmation |", "| --- | --- | --- |"]
        for wheel, result in report["wheels"].items():
            duty = result.get("candidate_permille")
            percent = f"{duty / 10:.1f}%" if duty is not None else "NOT_FOUND"
            lines.append(f"| {wheel} | {percent} | {result.get('successes', 0)}/{result.get('attempts', 0)} |")
        lines.extend(["", report["limitations"], "", str(report.get("error", "")), ""])
        (output / "report.md").write_text("\n".join(lines), encoding="utf-8")
        print(json.dumps({key: report[key] for key in ("status", "stop_confirmed", "wheels")}, ensure_ascii=False), flush=True)
        print(output.resolve(), flush=True)
    return 1 if report["status"] == "FAIL" or not report["stop_confirmed"] else 0


def main() -> int:
    args = parse_args()
    if args.self_test:
        return subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", str(ROOT / "tests/motor_driver"),
                               "-p", "test_startup_pipeline.py", "-v"], cwd=ROOT).returncode
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
