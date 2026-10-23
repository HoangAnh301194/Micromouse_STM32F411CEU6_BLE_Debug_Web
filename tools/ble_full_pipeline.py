#!/usr/bin/env python3
"""Single-command JDY-33 PA2/PA3 BLE qualification (no USB-TTL while running).

Defaults: flash motor-locked 115200 and 38400 builds, sweep sizes/rates, save
raw BLE bytes, parsed frames, CSV and JSON. Optional explicit guarded motor-on
30% closed-loop speed-PID phase, with continuous BLE heartbeat and auto STOP.
Do not open the browser Web BLE console while this program holds the GATT link.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
SYSCLK = 96_000_000
OFF115 = "ble_profile_motor_off_115200_96m"
OFF384 = "ble_profile_motor_off_38400_96m"
ON115 = "ble_profile_motor_on_pid_115200_96m"
# id, UART packet bytes (all framing included), requested Hz
MATRIX = [
    ("T01", 16, 10), ("T02", 20, 25), ("T03", 20, 50),
    ("T04", 32, 50), ("T05", 20, 100), ("T06", 20, 150),
    ("T07", 16, 200), ("T08", 20, 160), ("T09", 32, 100),
    ("T10", 64, 50), ("T11", 128, 25), ("T12", 20, 200),
    ("T13", 32, 125), ("T14", 64, 60), ("T15", 128, 30),
    ("T16", 20, 250), ("T17", 64, 80), ("T18", 16, 400),
]
BAUD384 = [
    ("B01", 16, 25), ("B02", 20, 50), ("B03", 20, 100),
    ("B04", 32, 50), ("B05", 64, 30), ("B06", 128, 15),
]
MOTOR = [
    ("M01", 32, 50), ("M02", 20, 100), ("M03", 64, 40),
    ("M04", 20, 150), ("M05", 32, 100), ("M06", 16, 200),
]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def percent(values, p):
    if not values:
        return None
    s = sorted(values)
    return round(s[max(0, (len(s) * p + 99) // 100 - 1)], 3)


def fields(line: str) -> dict:
    parts = line.strip().split(",")
    item = {"type": parts[0]}
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        try:
            item[key] = int(value)
        except ValueError:
            item[key] = value
    return item


class Decoder:
    """Unframed BLE notifications are a byte stream: split, join, resync.

    Binary frame: B5 62 01 D length session seq(u32) micros(u32) data crc16.
    Control lines begin ! and end LF. No framing assumptions about ATT MTU.
    """

    def __init__(self):
        self.pending = bytearray()
        self.frames = []
        self.events = []
        self.crc_failures = 0
        self.discarded = 0
        self.notifications = 0
        self.raw_bytes = 0

    def feed(self, data: bytes, time_ns: int | None = None):
        if time_ns is None:
            time_ns = time.perf_counter_ns()
        self.notifications += 1
        self.raw_bytes += len(data)
        self.pending.extend(data)
        while self.pending:
            if self.pending.startswith(b"\xb5\x62"):
                if len(self.pending) < 16:
                    break
                size = self.pending[4]
                if not (16 <= size <= 128) or self.pending[2] != 1:
                    self.discarded += 1
                    del self.pending[0]
                    continue
                if len(self.pending) < size:
                    break
                frame = bytes(self.pending[:size])
                if crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                    self.crc_failures += 1
                    del self.pending[0]
                    continue
                del self.pending[:size]
                self.frames.append({
                    "host_monotonic_ns": time_ns,
                    "session": frame[5], "seq": int.from_bytes(frame[6:10], "little"),
                    "mcu_us": int.from_bytes(frame[10:14], "little"),
                    "size": size, "kind": chr(frame[3]),
                    "payload_hex": frame[14:-2].hex(),
                })
                continue
            if self.pending[0] == ord("!"):
                eol = self.pending.find(b"\n")
                sof = self.pending.find(b"\xb5\x62")
                if 0 <= sof < eol or (eol < 0 and sof >= 0):
                    self.discarded += sof
                    del self.pending[:sof]
                    continue
                if eol < 0:
                    if len(self.pending) > 512:
                        self.discarded += 1
                        del self.pending[0]
                        continue
                    break
                raw = bytes(self.pending[:eol]).decode("ascii", errors="replace")
                del self.pending[:eol + 1]
                self.events.append({"host_monotonic_ns": time_ns, **fields(raw)})
                continue
            self.discarded += 1
            del self.pending[0]


def diff(a: dict, b: dict, key: str):
    x, y = a.get(key), b.get(key)
    return (y - x) & 0xFFFFFFFF if isinstance(x, int) and isinstance(y, int) else None


def score_test(test, frames, begin, end, before, after, crc_delta, seconds):
    test_id, size, hz = test
    session = end.get("s")
    selected = [f for f in frames if f["session"] == session and f["kind"] == "D"]
    seqs = [f["seq"] for f in selected]
    unique = set(seqs)
    generated, ok = end.get("gen"), end.get("ok")
    expected = hz * seconds
    absent = generated - len(unique) if isinstance(generated, int) else None
    arrival_ms = [
        (b["host_monotonic_ns"] - a["host_monotonic_ns"]) / 1e6
        for a, b in zip(selected, selected[1:])
    ]
    cpu = (end.get("ecy", 0) + end.get("scy", 0) +
           (diff(before, after, "txcy") or 0))
    cpu_pct = 100 * cpu / (SYSCLK * seconds)
    checks = {
        "end_marker": end.get("type") == "!E" and end.get("s") == begin.get("s"),
        "rate_over_95pct": hz == 0 or (isinstance(generated, int) and
                                      generated >= 0.95 * expected),
        "no_generator_skip": end.get("skip") == 0,
        "no_queue_reject": end.get("reject") == 0 and end.get("drop") == 0,
        "host_receives_all_accepted": isinstance(ok, int) and len(unique) == ok,
        "crc_zero": crc_delta == 0,
        "uart_rx_zero": end.get("rxerr") == 0,
        "queue_peak_below_75pct": end.get("peak", 4096) <= 3072,
        "cpu_below_3pct": cpu_pct <= 3.0,
        "tim11_no_late": diff(before, after, "late") == 0,
        "tim11_no_gap": diff(before, after, "gap") == 0,
        "mpu_no_stale": diff(before, after, "stale") == 0,
        "i2c_no_error": diff(before, after, "i2cerr") == 0,
        "i2c_no_timeout": diff(before, after, "i2cto") == 0,
        "ir_no_failure": diff(before, after, "irfail") == 0,
        "tim11_active": (diff(before, after, "tick") or 0) >= 0.9 * 1000 * seconds,
        "ir_scans_active": (diff(before, after, "ir") or 0) > 0,
    }
    return {
        "id": test_id, "size_B": size, "target_Hz": hz,
        "offered_Bps": size * hz, "duration_s": seconds,
        "generated": generated, "accepted": ok, "received": len(unique),
        "missing_vs_generated": absent, "duplicates": len(seqs) - len(unique),
        "reject": end.get("reject"), "skipped_schedules": end.get("skip"),
        "crc_failures": crc_delta, "uart_queue_peak_B": end.get("peak"),
        "ble_goodput_Bps": round(len(unique) * size / seconds, 1),
        "arrival_gap_p50_ms": percent(arrival_ms, 50),
        "arrival_gap_p95_ms": percent(arrival_ms, 95),
        "arrival_gap_p99_ms": percent(arrival_ms, 99),
        "uart_cpu_measured_subset_percent": round(cpu_pct, 4),
        "tim11_max_irq_us": round(after.get("maxcy", 0) / 96, 3),
        "tim11_late_delta": diff(before, after, "late"),
        "tim11_gap_delta": diff(before, after, "gap"),
        "mpu_stale_delta": diff(before, after, "stale"),
        "ir_scans_delta": diff(before, after, "ir"),
        "checks": checks, "pass": all(checks.values()),
    }


def ensure_module(package: str):
    if importlib.util.find_spec(package) is None:
        print(f"Installing {package} (one-shot prerequisite)...", flush=True)
        subprocess.run([sys.executable, "-m", "pip", "install",
                        "bleak" if package == "bleak" else "platformio"],
                       check=True)


def flash(env: str, output: Path):
    ensure_module("platformio")
    cmd = [sys.executable, "-m", "platformio", "run", "-e", env, "-t", "upload"]
    print("FLASH", env, flush=True)
    with (output / f"flash_{env}.log").open("w", encoding="utf8") as log:
        proc = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    if proc.returncode:
        tail = (output / f"flash_{env}.log").read_text(
            encoding="utf8", errors="replace").splitlines()[-45:]
        raise RuntimeError(f"Firmware flash FAILED: {env}\n" + "\n".join(tail))


async def discover(name: str, address: str | None):
    from bleak import BleakScanner
    devices = await BleakScanner.discover(timeout=12.0)
    if address:
        selected = [d for d in devices if d.address.lower() == address.lower()]
    else:
        selected = [d for d in devices if d.name and name.lower() in d.name.lower()]
    if not selected:
        names = ", ".join(f"{d.name or '?'} ({d.address})" for d in devices)
        raise RuntimeError(f"BLE device not found: {name or address}. Scanned: {names}")
    if len(selected) > 1:
        raise RuntimeError("Multiple matching BLE devices; specify --address: " +
                           ", ".join(d.address for d in selected))
    return selected[0]


class Session:
    def __init__(self, raw_file: Path):
        self.decoder = Decoder()
        self.raw = raw_file.open("wb")
        self.client = None
        self.rx_exc = None
        self.rtts_ms = []
        self.last_ping_ns = None

    def on_notify(self, _sender, data: bytearray):
        now = time.perf_counter_ns()
        try:
            chunk = bytes(data)
            self.raw.write(chunk)
            before = len(self.decoder.events)
            self.decoder.feed(chunk, now)
            for event in self.decoder.events[before:]:
                if event["type"] == "!P" and self.last_ping_ns is not None:
                    self.rtts_ms.append((now - self.last_ping_ns) / 1e6)
                    self.last_ping_ns = None
        except Exception as exc:
            self.rx_exc = exc

    async def connect(self, device):
        from bleak import BleakClient
        self.client = BleakClient(device, timeout=20.0)
        await self.client.connect()
        char = self.client.services.get_characteristic(UUID)
        if not char or "notify" not in char.properties:
            raise RuntimeError("JDY-33 FFE1 notify characteristic not found")
        if "write" in char.properties:
            self.with_response = True
        elif "write-without-response" in char.properties:
            self.with_response = False
        else:
            raise RuntimeError("JDY-33 FFE1 has no GATT write permission")
        await self.client.start_notify(UUID, self.on_notify)

    async def send(self, command: str):
        if self.rx_exc:
            raise RuntimeError(f"BLE data receive failed: {self.rx_exc}")
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE disconnected")
        value = (command + "\n").encode("ascii")
        if len(value) > 20:
            raise ValueError(f"Control write >20B (default GATT payload): {command}")
        await self.client.write_gatt_char(UUID, value, response=self.with_response)

    async def wait(self, kind: str, after: int, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.rx_exc:
                raise RuntimeError(f"BLE notification handler: {self.rx_exc}")
            if not self.client.is_connected:
                raise ConnectionError("JDY-33 BLE link disconnected")
            for event in self.decoder.events[after:]:
                if event.get("type") == kind:
                    return event
                if event.get("type") == "!ERR":
                    raise RuntimeError("Firmware refused command: " + repr(event))
            await asyncio.sleep(0.02)
        raise TimeoutError(f"No {kind} within {timeout}s; device may have dropped BLE output")

    async def ready(self, mode: str):
        deadline = time.monotonic() + 65
        while time.monotonic() < deadline:
            mark = len(self.decoder.events)
            await self.send("HELLO")
            hello = await self.wait("!H", mark, 6)
            if hello.get("mode") != mode:
                raise RuntimeError(f"Incorrect flashed BLE mode: {hello}")
            if hello.get("ready") == 1 and hello.get("cal") == 3:
                return hello
            if hello.get("cal") == 4:
                raise RuntimeError("MPU calibration failed: " + repr(hello))
            await asyncio.sleep(1.5)
        raise TimeoutError("MPU did not calibrate within 65s; motor stays OFF")

    async def health(self):
        mark = len(self.decoder.events)
        await self.send("STATUS")
        return await self.wait("!S", mark, 8)

    async def run_case(self, test, seconds: int, run_mode: str):
        test_id, size, hz = test
        start_health = await self.health()
        begin_index = len(self.decoder.events)
        frame_index = len(self.decoder.frames)
        crc0 = self.decoder.crc_failures
        await self.send(f"RUN {size} {hz} {seconds*1000}")
        begin = await self.wait("!B", begin_index, 7)
        end = await self.wait("!E", begin_index + 1, seconds + 12)
        await asyncio.sleep(0.35)  # GATT/OS notification drain
        end_health = await self.health()
        result = score_test(test, self.decoder.frames[frame_index:],
                            begin, end, start_health, end_health,
                            self.decoder.crc_failures-crc0, seconds)
        result["mode"] = run_mode
        print(f" {run_mode}/{test_id} {size:3} B x {hz:3} Hz "
              f"host={result['received']}/{result['generated']} "
              f"CPU={result['uart_cpu_measured_subset_percent']:.3f}% "
              f"TIM11 late={result['tim11_late_delta']} "
              f"{'PASS' if result['pass'] else 'LIMIT'}", flush=True)
        return result

    async def heartbeat(self):
        while self.client and self.client.is_connected:
            self.last_ping_ns = time.perf_counter_ns()
            await self.send("PING")
            await asyncio.sleep(0.15)

    async def close(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.write_gatt_char(UUID, b"!", response=self.with_response)
            except Exception:
                pass
            try:
                await self.client.disconnect()
            except Exception:
                pass
        self.raw.close()


def save_phase(session: Session, output: Path, phase_name: str):
    with (output / f"{phase_name}_events.json").open("w", encoding="utf8") as f:
        json.dump(session.decoder.events, f, indent=2)
    with (output / f"{phase_name}_frames.csv").open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=["host_monotonic_ns", "session",
                "seq", "mcu_us", "size", "kind", "payload_hex"])
        writer.writeheader()
        writer.writerows(session.decoder.frames)
    return {
        "ble_notifications": session.decoder.notifications,
        "received_raw_B": session.decoder.raw_bytes,
        "crc_failures": session.decoder.crc_failures,
        "desync_discarded_B": session.decoder.discarded,
        "rtt_p50_ms": percent(session.rtts_ms, 50),
        "rtt_p95_ms": percent(session.rtts_ms, 95),
        "rtt_p99_ms": percent(session.rtts_ms, 99),
    }


async def run_phase(env, cases, out, args, mode):
    if not args.no_flash:
        flash(env, out)
    device = await discover(args.name, args.address)
    session = Session(out / f"{env}_raw.bin")
    rows = []
    error = None
    hb_task = None
    completion = {}
    try:
        await session.connect(device)
        hello = await session.ready(mode)
        print(f"CONNECTED {device.name} mode={mode} baud={hello.get('baud')}", flush=True)
        if mode == "ON":
            mark = len(session.decoder.events)
            await session.send("ARM")
            arm = await session.wait("!ARM", mark, 8)
            if arm.get("pwm_max") != 30 or arm.get("stages") != 12:
                raise RuntimeError("Dangerous motor profile: " + repr(arm))
            hb_task = asyncio.create_task(session.heartbeat())
            # Heartbeat must already be active before running first load case.
        for test in cases:
            if hb_task and hb_task.done():
                raise RuntimeError("Motor heartbeat task exited")
            rows.append(await session.run_case(test, args.seconds, mode))
        if mode == "ON":
            deadline = time.monotonic() + 95
            while time.monotonic() < deadline:
                index = len(session.decoder.events)
                await session.send("STATUS")
                detail = await session.wait("!D", index, 7)
                completion = detail
                if detail.get("sweep") in (3, 4):
                    break
                await asyncio.sleep(0.75)
            if completion.get("sweep") != 3 or completion.get("fault") != 0:
                raise RuntimeError("Motor sweep incomplete/fault: " + repr(completion))
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print("PHASE ABORT:", error, file=sys.stderr, flush=True)
    finally:
        if hb_task:
            hb_task.cancel()
            try:
                await hb_task
            except asyncio.CancelledError:
                pass
        await session.close()
        meta = save_phase(session, out, env)
        meta.update({"environment": env, "mode": mode, "error": error,
                     "motor_end": completion, "cases": rows})
    return meta


def write_results(out, phases, argv):
    rows = [t for p in phases for t in p.get("cases", [])]
    if rows:
        keys = ["mode", "id", "size_B", "target_Hz", "offered_Bps",
                "generated", "accepted", "received", "missing_vs_generated",
                "reject", "skipped_schedules", "crc_failures",
                "ble_goodput_Bps", "arrival_gap_p50_ms",
                "arrival_gap_p95_ms", "arrival_gap_p99_ms",
                "uart_cpu_measured_subset_percent", "tim11_max_irq_us",
                "tim11_late_delta", "tim11_gap_delta", "mpu_stale_delta",
                "ir_scans_delta", "pass"]
        with (out / "ble_test_matrix.csv").open("w", newline="", encoding="utf8") as f:
            writer = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
    report = {
        "utc_timestamp": datetime.now(timezone.utc).isoformat(),
        "argv": argv, "clock_hz_nominal": SYSCLK,
        "hardware_verified": False,
        "notes": [
            "PASS requires actual board + JDY-33 GATT notifications; not CI.",
            "UART DR writes do not prove over-air delivery.",
            "Arrival gaps and Ping RTT include host/OS BLE scheduling.",
            "One-way sensor-to-PC latency is NOT measured without clock sync.",
            "Speed PID in motor-on integration sweep is 10ms, TIM11 is 1ms.",
            "motor-on remains subject to 30pct PWM firmware + 750ms BLE lease "
            "and independent VM cutoff.",
        ],
        "phases": phases,
        "overall": "PASS" if phases and all(not p.get("error") and
                   p.get("cases") and all(c["pass"] for c in p["cases"])
                   for p in phases) else "FAIL_OR_INCOMPLETE",
    }
    (out / "ble_summary.json").write_text(json.dumps(report, indent=2), encoding="utf8")
    doc = ["# BLE one-shot qualification report", "",
           f"Result: **{report['overall']}**", ""]
    for p in phases:
        doc += [f"## {p['environment']}",
                f"Error: {p.get('error') or 'none'}", "",
                "| Test | Packet | Hz | BLE received | CPU subset | TIM11 late | Result |",
                "|---|---:|---:|---:|---:|---:|---|"]
        for t in p.get("cases", []):
            doc.append(f"| {t['id']} | {t['size_B']} B | {t['target_Hz']} | "
                       f"{t['received']}/{t['generated']} | "
                       f"{t['uart_cpu_measured_subset_percent']:.3f}% | "
                       f"{t['tim11_late_delta']} | "
                       f"{'PASS' if t['pass'] else 'LIMIT'} |")
        doc.append("")
    (out / "ble_qualification.md").write_text("\n".join(doc), encoding="utf8")
    return report


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", default="Nezumi_Chan",
                   help="BLE advertised name substring; default Nezumi_Chan")
    p.add_argument("--address", help="BLE adapter-specific address/identifier")
    p.add_argument("--seconds", type=int, default=5,
                   help="Seconds for each case, 2..10, default 5")
    p.add_argument("--motor-on", action="store_true",
                   help="Opt-in lifted-wheel guarded PID motor test")
    p.add_argument("--fixture-ready", action="store_true")
    p.add_argument("--independent-vm-cutoff", action="store_true")
    p.add_argument("--allow-30-percent", action="store_true")
    p.add_argument("--no-flash", action="store_true",
                   help="Use currently flashed OFF115 test firmware; OFF115 only")
    p.add_argument("--output", type=Path, help="Results folder")
    return p.parse_args()


async def main():
    args = parse_args()
    if not (2 <= args.seconds <= 10):
        raise SystemExit("--seconds must be between 2 and 10")
    if args.motor_on and not (args.fixture_ready and args.independent_vm_cutoff
                             and args.allow_30_percent):
        raise SystemExit("Motor ON requires --fixture-ready "
                         "--independent-vm-cutoff --allow-30-percent")
    ensure_module("bleak")
    out = args.output or ROOT / "results" / (
        "ble_one_shot_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    phases = []
    jobs = [(OFF115, [("BASE", 16, 0)] + MATRIX, "OFF")]
    if not args.no_flash:
        jobs.append((OFF384, BAUD384, "OFF"))
        if args.motor_on:
            jobs.append((ON115, MOTOR, "ON"))
    for env, cases, mode in jobs:
        phase = await run_phase(env, cases, out, args, mode)
        phases.append(phase)
        write_results(out, phases, sys.argv[1:])
        if phase["error"]:
            print("Aborting remaining profiles after fault.", flush=True)
            break
        if mode == "OFF" and not all(
                row["pass"] for row in phase["cases"] if row["target_Hz"] <= 100):
            print("Basic BLE control/sensor gate failed. No motor-on phase.", flush=True)
            break
    report = write_results(out, phases, sys.argv[1:])
    print(f"FINAL {report['overall']}  results={out}", flush=True)
    if report["overall"] != "PASS":
        raise SystemExit(2)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted; motor-on firmware heartbeat expires and locks outputs.",
              file=sys.stderr)
        raise SystemExit(130)
