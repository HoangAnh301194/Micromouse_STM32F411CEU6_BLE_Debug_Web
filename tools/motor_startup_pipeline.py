#!/usr/bin/env python3
"""Bounded forward-only startup or opt-in elevated-wheel high-speed experiment."""
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
HIGH_ENVIRONMENT = "motor_speed_96m_trim25"
HIGH_PROFILE = {**PROFILE, "version": 4, "experimental": 1, "qualified": 0, "cap": 1000, "pulse_ms": 600,
                "rest_ms": 900, "max_pulses": 32, "session_ms": 90000}


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


def validate_profile(profile: dict, high_speed: bool = False) -> None:
    expected = HIGH_PROFILE if high_speed else PROFILE
    if any(profile.get(key) != value for key, value in expected.items()):
        raise ValueError("Unknown startup firmware or safety limits")


def validate_row(row: dict, stopping: bool = False, high_speed: bool = False) -> None:
    limits = HIGH_PROFILE if high_speed else PROFILE
    required = {"id", "state", "fault", "ms", "t", "rest", "q", "wheel", "duty",
                "l", "r", "al", "ar", "cl", "cr", "odr", "ticks", "late", "gaps", "tx", "rx"}
    if not required <= row.keys() or any(type(row[key]) is not int for key in required):
        raise ValueError("Incomplete/noninteger startup sample")
    if row["state"] not in range(5) or row["wheel"] not in ((0, 1, 2) if high_speed else (0, 1)):
        raise ValueError("Invalid state/wheel")
    if (row["state"] == 4) != (row["fault"] != 0):
        raise ValueError("Fault/state mismatch")
    if any(row[key] for key in ("gaps", "tx", "rx")):
        raise ValueError("Timing gap or UART error")
    if row["fault"] and not (stopping and 1 <= row["fault"] <= 10):
        raise ValueError(f"Firmware fault {row['fault']}")
    if not 0 <= row["id"] <= limits["max_pulses"] or not 0 <= row["duty"] <= limits["cap"] or not 0 <= row["t"] <= limits["pulse_ms"]:
        raise ValueError("Pulse bounds exceeded")
    for wheel in ("l", "r"):
        applied = row["a" + wheel]
        expected = applied * 3200 // 1000 if applied else 3200
        if not 0 <= applied <= row["duty"] or row["c" + wheel] != expected:
            raise ValueError("PWM/register contract mismatch")
    expected_odr = (0x2000 if row["al"] else 0) | (0x8000 if row["ar"] else 0)
    startup_guard = row["state"] == 2 and row["t"] <= 1 and row["odr"] == 0 and max(row["al"], row["ar"]) <= 20
    if row["odr"] != expected_odr and not startup_guard:
        raise ValueError("Direction/register contract mismatch")
    if row["state"] != 2:
        if row["al"] or row["ar"] or row["odr"]:
            raise ValueError("Non-neutral outputs outside pulse")
    elif row["wheel"] < 2:
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


def speed_metrics(rows: list[dict], wheel: str) -> dict:
    if len(rows) < 12 or rows[0]["t"] > 5 or rows[-1]["t"] != 600:
        raise ValueError("Missing high-speed pulse endpoints/samples")
    active = ("l", "r") if wheel == "b" else (wheel,)
    speeds = {key: [] for key in active}
    for earlier, later in zip(rows, rows[1:]):
        elapsed = later["t"] - earlier["t"]
        period = (later["ms"] - earlier["ms"]) & 0xFFFFFFFF
        if not 0 < elapsed <= 80 or abs(period - elapsed) > 2:
            raise ValueError("High-speed timestamps missing/inconsistent")
        for key in active:
            delta = later[key] - earlier[key]
            if delta < 0:
                raise ValueError("Reverse high-speed counts")
            if earlier["t"] >= 300:
                speeds[key].append(delta * 1000 / elapsed)
    metrics = {}
    for key, rates in speeds.items():
        if len(rates) < 5 or min(rates) <= 0:
            raise ValueError("Insufficient sustained high-speed encoder data")
        stats = distribution(rates)
        trend = abs(sum(rates[-3:]) - sum(rates[:3])) / (3 * stats["mean"]) * 100
        metrics[key] = {**stats, "drift_pct": trend,
                        "steady": stats["cv_pct"] <= 10 and trend <= 10}
    return {"counts": sum(rows[-1][key] for key in active), "first_motion": True,
            "sustained_start": True, "positive_intervals": len(rows) - 1,
            "last_100ms_counts": None, "first_five_counts_observed_ms": None,
            "speed_pps": metrics}

class Capture:
    def __init__(self, port, output: Path, high_speed: bool = False):
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
        self.high_speed = high_speed
        self.experimental_ack_pending = self.experimental_ack_accepted = False
        self.limits = HIGH_PROFILE if high_speed else PROFILE
        self.health = None
        self.health_records = []
        self.first_frame = True

    def send(self, command: str) -> None:
        if command == "STARTUP ARM" and self.high_speed and not self.experimental_ack_accepted:
            raise ValueError("High-speed firmware experimental ACK required before ARM")
        if command == "STARTUP ACK EXPERIMENT":
            if not self.high_speed or self.profile is None or self.armed or self.experimental_ack_pending:
                raise ValueError("Experimental ACK allowed once per verified disarmed high-speed profile")
            self.experimental_ack_pending = True
        allowed = ("STARTUP PING", "STARTUP ARM", "STARTUP KEEP", "STARTUP STOP", "STARTUP WATCHDOG")
        if self.high_speed:
            allowed += ("STARTUP ACK EXPERIMENT",)
        if command not in allowed:
            parts = command.split()
            wheels = ("l", "r", "b") if self.high_speed else ("l", "r")
            if (len(parts) != 5 or parts[:2] != ["STARTUP", "PULSE"] or parts[3] not in wheels or
                not parts[2].isdigit() or not parts[4].isdigit() or not 1 <= int(parts[2]) <= self.limits["max_pulses"] or
                not 5 <= int(parts[4]) <= self.limits["cap"] or int(parts[4]) % 5):
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
        if self.first_frame:
            self.first_frame = False
            if not raw.startswith(b"STARTUP_"):
                return None
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
            validate_profile(row, self.high_speed)
            self.profile = row
            if self.watchdog_pending and self.watchdog_ack:
                if not row.get("reset_flags", 0) & (1 << 29):
                    raise ValueError("Watchdog probe reset not confirmed by RCC flag")
                self.watchdog_observed = True
                self.watchdog_pending = False
                self.latest = None
                self.health = None
        elif name == "STARTUP_WATCHDOG" and self.watchdog_pending and row == {"requested": 1}:
            self.watchdog_ack = True
        elif name == "STARTUP_EXPERIMENT_ACK" and self.high_speed:
            if not self.experimental_ack_pending or self.experimental_ack_accepted or row != {"accepted": 1}:
                raise ValueError("Unexpected/invalid experimental acknowledgement")
            self.experimental_ack_accepted = True
        elif name in ("STARTUP_SAMPLE", "STARTUP_STATUS", "STARTUP_BEGIN", "STARTUP_END", "STARTUP_FAULT"):
            validate_row(row, self.stopping, self.high_speed)
            if self.armed and self.baseline_late is not None and row["late"] != self.baseline_late:
                raise ValueError("TIM11 late counter increased")
            self.latest = row
            self.records.append((name, row))
        elif self.high_speed and name == "STARTUP_HEALTH":
            required = ("ready", "good", "fresh", "err", "timeout", "ir", "age_max", "isr", "adc_div")
            if any(type(row.get(key)) is not int for key in required) or row["ready"] not in (0, 1):
                raise ValueError("Incomplete high-speed sensor health")
            if row["err"] or row["timeout"] or row["age_max"] > 15000 or row["adc_div"] != 4:
                raise ValueError("Sensor errors/staleness during high-speed bench")
            self.health = row
            self.health_records.append(row)
        elif self.high_speed and name == "STARTUP_PERF":
            if any(type(row.get(key)) is not int for key in ("id", "n", "cycles", "max", "rev")) or not 400 <= row["n"] <= 600 or row["cycles"] <= 0 or row["rev"] not in (2, 3):
                raise ValueError("Invalid driver cycle measurement")
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
                             for name, row in self.records[offset:]) and
                  (not self.high_speed or any(name == "STARTUP_PERF" and row["id"] == sequence
                                              for name, row in self.records[offset:])))
        selected = [(name, row) for name, row in self.records[offset:] if row["id"] == sequence and name != "STARTUP_PERF"]
        if sum(name == "STARTUP_BEGIN" for name, row in selected) != 1:
            raise ValueError("Missing/duplicate pulse acknowledgement")
        rows = []
        for name, row in selected:
            if name not in ("STARTUP_BEGIN", "STARTUP_END") and row["state"] != 2:
                continue
            if row["wheel"] != {"l": 0, "r": 1, "b": 2}[wheel] or row["duty"] != duty:
                raise ValueError("Pulse identity mismatch")
            if rows and row["t"] == rows[-1]["t"]:
                if (row["l"], row["r"]) != (rows[-1]["l"], rows[-1]["r"]):
                    raise ValueError("Different counts at duplicate elapsed time")
                continue
            rows.append(row)
        result = {"id": sequence, "wheel": wheel, "duty_permille": duty,
                  **(speed_metrics(rows, wheel) if self.high_speed else trial_metrics(rows, wheel)), "samples": rows}
        if self.high_speed:
            result["driver_cycles"] = next(row for name, row in self.records[offset:] if name == "STARTUP_PERF" and row["id"] == sequence)
        print(f"{wheel.upper()} {duty / 10:.1f}%: {result['counts']} counts; sustained={result['sustained_start']}", flush=True)
        return result

    def stop(self) -> bool:
        self.stopping = True
        self.send("STARTUP STOP")
        self.wait(lambda: self.latest is not None and self.latest["state"] == 4 and self.latest["fault"] != 0, 1.5)
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
    parser.add_argument("--high-speed", action="store_true")
    parser.add_argument("--ack-experimental-unqualified", action="store_true",
                        help="Explicit research-only high-speed bench consent, NOT production qualification")
    parser.add_argument("--confirm-only", action="store_true")
    parser.add_argument("--repeats", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(arguments)
    if args.ack_experimental_unqualified and (not args.run or not args.high_speed):
        parser.error("Experimental acknowledgement requires --run --high-speed")
    if args.run and args.high_speed and not args.ack_experimental_unqualified:
        parser.error("P0 EXPERIMENTAL GATE: high-speed bench requires --ack-experimental-unqualified")
    if args.confirm_only and (not args.run or not args.high_speed):
        parser.error("confirm-only requires a high-speed hardware run")
    if args.run and (not args.confirm_wheels_elevated or args.vm_volts is None or not 0 < args.vm_volts <= 8.4):
        parser.error("Run requires elevated-wheel confirmation and user-reported VM in (0, 8.4] V")
    if args.repeats is None:
        args.repeats = 2 if args.high_speed else 10
    if not ((2 <= args.repeats <= 5) if args.high_speed else (10 <= args.repeats <= 20)):
        parser.error("repeats must be 2..5 for high-speed, otherwise 10..20")
    if args.self_test and (args.upload or args.high_speed or args.ack_experimental_unqualified or args.confirm_wheels_elevated or args.vm_volts is not None or args.output):
        parser.error("Self-test never builds, uploads or accesses hardware")
    if args.output and args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error("Never overwrite existing evidence")
    return args


def collect_trials(client: Capture, args, report: dict, output: Path) -> None:
    if args.high_speed:
        sequence = [(wheel, duty) for duty in (200, 400, 600, 800, 1000) for wheel in ("l", "r")]
        sequence += [("b", duty) for duty in (400, 600, 800, 1000)]
        if args.confirm_only:
            sequence = [("l", 200), ("r", 200), ("b", 400), ("b", 800), ("b", 1000)]
        sequence += [("b", 1000)] * (args.repeats - 1)
        previous = {}
        for wheel, duty in sequence:
            trial = client.pulse(wheel, duty)
            trial["phase"] = "high_speed"
            report["trials"].append(trial)
            write_json(output / "report.json", report)
            for key, stats in trial["speed_pps"].items():
                identity = wheel, key
                if identity in previous and stats["mean"] < previous[identity] * 0.8:
                    raise ValueError("Encoder speed fell over 20 percent; no higher-duty test allowed")
                previous[identity] = stats["mean"]
        full = [trial for trial in report["trials"] if trial["wheel"] == "b" and trial["duty_permille"] == 1000]
        for wheel in ("l", "r"):
            stats = [trial["speed_pps"][wheel] for trial in full]
            report["wheels"][wheel] = {
                "duty_permille": 1000, "speed_statistics_pps": distribution([row["mean"] for row in stats]),
                "plateau_cv_pct_max": max(row["cv_pct"] for row in stats),
                "plateau_drift_pct_max": max(row["drift_pct"] for row in stats),
                "steady_confirmations": sum(row["steady"] for row in stats), "attempts": len(stats)}
        perf = [trial["driver_cycles"] for trial in report["trials"]]
        report["driver_cycles_mean"] = sum(row["cycles"] for row in perf) / sum(row["n"] for row in perf)
        report["driver_revision"] = perf[-1]["rev"]
        report["status"] = "MEASURED_HIGH_SPEED" if all(value["steady_confirmations"] == args.repeats for value in report["wheels"].values()) else "REVIEW_SPEED_NOT_SETTLED"
        return
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
    report["status"] = "MEASURED_STARTUP_CANDIDATES" if all("candidate_permille" in value for value in report["wheels"].values()) else "REVIEW_STARTUP_NOT_FOUND"

def run(args) -> int:
    import serial
    environment = HIGH_ENVIRONMENT if args.high_speed else ENVIRONMENT
    prefix = "startup_speed_" if args.high_speed else "startup_"
    output = args.output or ROOT / "motor_logs" / (prefix + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=True)
    report = {"status": "FAIL", "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "environment": environment, "high_speed": args.high_speed, "confirm_only": args.confirm_only,
              "vm_volts_user_reported": args.vm_volts,
              "load": "wheels_elevated_user_confirmed", "current_temperature_supply_droop": "NOT_MEASURED",
              "encoder_polarity": "USER_CONFIRMED_NOT_RETESTED", "trials": [], "wheels": {},
              "upload_requested": args.upload, "stop_confirmed": False,
              "qualification_status": "NOT_QUALIFIED_ELECTRICAL_OR_LOADED" if args.high_speed else "STARTUP_CHARACTERIZATION_ONLY",
              "experiment_acknowledged_by_operator": args.ack_experimental_unqualified,
              "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()}
    sources = ("src/motor.c", "src/motor_test.c", "src/motor_startup.c", "src/motor_startup_bench.c",
               "include/motor_startup.h", "src/main.c", "src/control.c", "src/uart.c", "include/uart.h",
               "platformio.ini", "tools/motor_startup_pipeline.py")
    if args.high_speed:
        sources += ("src/encoder.c", "src/ir_sensor.c", "src/mpu6050.c", "src/bt_debug.c")
    report["source_sha256"] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in sources}
    client = None
    try:
        checks = [("native", [sys.executable, str(ROOT / "tools/motor_driver_native_test.py")]),
                  ("self_test", [sys.executable, str(Path(__file__).resolve()), "--self-test"])]
        if args.upload:
            checks.append(("upload", [sys.executable, "-m", "platformio", "run", "-e", environment, "-t", "upload"]))
        for name, command in checks:
            process = subprocess.run(command, cwd=ROOT, capture_output=True, encoding="utf-8", errors="replace", timeout=600)
            (output / f"{name}.log").write_text(process.stdout + process.stderr, encoding="utf-8")
            if process.returncode:
                raise RuntimeError(f"{name} failed before UART arming")
        image = ROOT / ".pio/build" / environment / "firmware.bin"
        report["built_image_sha256"] = hashlib.sha256(image.read_bytes()).hexdigest() if image.exists() else None
        with serial.Serial(args.port, 115200, timeout=0.04, write_timeout=0.25) as port:
            client = Capture(port, output, args.high_speed)
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
                if args.high_speed:
                    client.wait(lambda: client.health is not None and client.health["ready"] == 1, 15.0)
                client.baseline_late = client.latest["late"]
                if args.high_speed:
                    client.send("STARTUP ACK EXPERIMENT")
                    client.wait(lambda: client.experimental_ack_accepted, 2.0)
                    report["firmware_experiment_acknowledged"] = True
                client.send("STARTUP ARM")
                client.armed = True
                client.wait(lambda: client.latest["state"] == 1)
                collect_trials(client, args, report, output)
                client.wait(lambda: client.latest["state"] == 1)
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
                    if args.high_speed:
                        report["sensor_health"] = client.health_records
                    client.close()
    except Exception as error:
        report["status"] = "FAIL"
        report["error"] = str(error)
    finally:
        report["limitations"] = ("Bounded elevated-wheel pulses only; not continuous/full-load approval or current/thermal/energy-efficiency qualification. BLE radio link and Motion/PID workload not tested. ADC /4 is bench-local; production IR clock still needs separate review."
                                 if args.high_speed else "Startup under reported elevated-wheel conditions only; not holding PWM, loaded calibration, current/thermal qualification or full-speed approval. Driver deadband unchanged.")
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
            if args.high_speed:
                lines.append(f"| {wheel} | 100%: {result['speed_statistics_pps']['mean']:.1f} counts/s | {result['steady_confirmations']}/{result['attempts']} steady |")
            else:
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
