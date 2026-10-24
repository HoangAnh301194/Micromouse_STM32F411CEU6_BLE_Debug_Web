#!/usr/bin/env python3
"""Motor-OFF OLED/MPU I2C1 qualification: UART1 A/B, raw log + JSON evidence.

pip install pyserial
python tools/oled_mpu_pipeline.py --port COM5 --baseline-seconds 15 --oled-seconds 15
python tools/oled_mpu_pipeline.py --self-test
Never treats a build or synthetic --self-test as hardware acceptance.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

PARTS = {"OLED_C", "OLED_G", "OLED_O", "OLED_E"}
REQUIRED = {
    "ms", "on", "ready", "cal", "locked", "tick", "late", "gap",
    "valid", "invalid", "uart_drop", "good", "err", "to",
    "mpubusy", "age", "cmd", "chunks", "frames", "bytes",
    "skip", "wait", "oled_err", "tx_start", "tx_done", "tx_err",
    "tx_to", "tx_max", "recoveries", "maxirq",
    "af", "berr", "arlo", "ovr", "fe", "dme", "te",
    "last_sr1", "last_dma", "last_phase", "ndtr",
}


def parse_record(line: str):
    items = line.strip().split(",")
    if not items or items[0] not in PARTS:
        return None
    values = {}
    for chunk in items[1:]:
        if "=" not in chunk:
            return None
        key, raw = chunk.split("=", 1)
        try:
            values[key] = int(raw)
        except ValueError:
            return None
    if "seq" not in values:
        return None
    return items[0], values


class Assembler:
    """Only complete matching sequence IDs become a measurement."""
    def __init__(self):
        self.pending = defaultdict(dict)
        self.malformed = 0
        self.missing = 0

    def feed(self, line: str):
        parsed = parse_record(line)
        if parsed is None:
            if line.startswith(("OLED_C,", "OLED_G,", "OLED_O,", "OLED_E,")):
                self.malformed += 1
            return None
        kind, item = parsed
        seq = item.pop("seq")
        self.pending[seq][kind] = item
        # Bounded recovery after corrupt or dropped UART frames.
        for old in list(self.pending):
            if old < seq - 4:
                self.missing += 1
                del self.pending[old]
        if len(self.pending[seq]) != len(PARTS):
            return None
        sample = {"seq": seq}
        for segment in sorted(PARTS):
            sample.update(self.pending[seq][segment])
        del self.pending[seq]
        if not REQUIRED.issubset(sample):
            self.malformed += 1
            return None
        return sample


def delta(end, start, key):
    return (end[key] - start[key]) % (2**32)


def segment(samples, expected_on):
    if len(samples) < 6:
        raise ValueError("Not enough complete UART frames (need >= 6)")
    for row in samples:
        if row["on"] != expected_on or row["cal"] != 1 or row["locked"] != 1:
            raise ValueError("Wrong mode / uncalibrated / motor not locked")
        if row["ready"] != 1:
            raise ValueError("OLED was not detected (check PB8/PB9/0x3C)")
    start, end = samples[0], samples[-1]
    dt_ms = delta(end, start, "ms")
    if dt_ms < 3500:
        raise ValueError("Insufficient wall time in telemetry")
    dt = dt_ms / 1000.0
    count = delta(end, start, "tick")
    keys = ["tick", "late", "gap", "valid", "invalid", "uart_drop",
            "good", "err", "to", "mpubusy", "cmd", "chunks",
            "frames", "bytes", "skip", "wait", "oled_err",
            "tx_start", "tx_done", "tx_err", "tx_to", "recoveries",
            "af", "berr", "arlo", "ovr", "fe", "dme", "te"]
    diff = {key: delta(end, start, key) for key in keys}
    diff.update(
        seconds=dt,
        tick_rate=count / dt,
        gyro_rate=diff["good"] / dt,
        gyro_per_tick=diff["good"] / count if count else 0,
        late_ratio=diff["late"] / count if count else 1,
        max_age_us=max(r["age"] for r in samples),
        tx_max_us=max(r["tx_max"] for r in samples),
        max_irq_cycles=max(r["maxirq"] for r in samples),
        last_failure={key: end[key] for key in
                      ("last_sr1", "last_dma", "last_phase", "ndtr")},
        rows=len(samples),
    )
    return diff


def evaluate(off, on, minimum_gyro_hz=850, min_ratio=0.97):
    tests = {
        "tick_1khz_off": 970 <= off["tick_rate"] <= 1030,
        "tick_1khz_on": 970 <= on["tick_rate"] <= 1030,
        "no_lost_irq_intervals": off["gap"] == 0 and on["gap"] == 0,
        "no_stale_gyro_at_control": off["invalid"] == 0 and on["invalid"] == 0,
        "mpu_no_new_error": all(x["err"] == 0 and x["to"] == 0 for x in [off, on]),
        "gyro_rate_baseline": off["gyro_rate"] >= minimum_gyro_hz,
        "gyro_rate_coexistence": on["gyro_rate"] >= minimum_gyro_hz and
                                  on["gyro_rate"] >= off["gyro_rate"] * min_ratio,
        "irq_late_increase": on["late_ratio"] <= off["late_ratio"] + 0.005,
        "gyro_age_under_3ms": max(off["max_age_us"], on["max_age_us"]) <= 3000,
        "oled_actual_refresh": on["frames"] >= 1 and on["chunks"] > 0 and
                                 on["bytes"] > 0 and on["tx_done"] > 0,
        "oled_traffic_off_during_baseline": off["tx_start"] == 0,
        "no_oled_dma_errors": on["oled_err"] == 0 and on["tx_err"] == 0 and
                              on["tx_to"] == 0,
        "no_i2c_oled_nack_or_bus_fault": all(on[k] == 0 for k in
                                               ("af", "berr", "arlo", "ovr")),
        "no_dma_fifo_direct_transfer_fault": all(on[k] == 0 for k in
                                                   ("fe", "dme", "te")),
        "no_bus_recovery": off["recoveries"] == 0 and on["recoveries"] == 0,
        "isr_bounded_under_500us": max(off["max_irq_cycles"], on["max_irq_cycles"]) < 48000,
        "short_oled_transactions": 0 < on["tx_max_us"] <= 900,
        "serial_complete": off["uart_drop"] == 0 and on["uart_drop"] == 0,
    }
    return {"verdict": "PASS" if all(tests.values()) else "FAIL",
            "checks": tests, "baseline": off, "oled_on": on,
            "note": "Hardware-only verdict; cannot infer electrical timing from UART alone."}


def self_test():
    def build(seq, mode, tick, good, completed=0):
        return dict(seq=seq, ms=seq * 1000, on=mode, ready=1,
                    cal=1, locked=1, tick=tick, late=0, gap=0,
                    valid=tick, invalid=0, uart_drop=0, good=good,
                    err=0, to=0, mpubusy=0, age=900,
                    cmd=completed, chunks=completed, frames=completed,
                    bytes=completed * 8, skip=10, wait=0, oled_err=0,
                    tx_start=completed * 2, tx_done=completed * 2,
                    tx_err=0, tx_to=0, recoveries=0, maxirq=5000,
                    af=0, berr=0, arlo=0, ovr=0, fe=0, dme=0, te=0,
                    last_sr1=0, last_dma=0, last_phase=0, ndtr=0,
                    tx_max=250 if mode else 0)
    a = [build(i, 0, i*1000, i*980) for i in range(6)]
    b = [build(i, 1, i*1000, i*980, completed=i-5) for i in range(6,12)]
    assert evaluate(segment(a,0), segment(b,1))["verdict"] == "PASS"
    b[-1]["tx_to"] = 1
    assert evaluate(segment(a,0), segment(b,1))["verdict"] == "FAIL"
    ass = Assembler()
    r = a[0]
    for tag,keys in [
        ("OLED_C",("ms on ready cal locked tick late gap valid invalid maxirq uart_drop")),
        ("OLED_G",("good err to mpubusy age")),
        ("OLED_O",("cmd chunks frames bytes skip wait oled_err tx_start tx_done tx_err tx_to tx_max recoveries")),
        ("OLED_E",("af berr arlo ovr fe dme te last_sr1 last_dma last_phase ndtr")),
    ]:
        fields = "seq=%d," % r["seq"] + ",".join(
            "%s=%d" % (k,r[k]) for k in keys.split())
        val = ass.feed(tag+","+fields)
    assert val is not None and val["tick"] == 0
    assert parse_record("OLED_G,seq=1,good=bad") is None
    print("SELF_TEST_PASS (parser, frame assembly, PASS/FAIL guard)")
    return 0


def capture(port, baud, baseline_seconds, oled_seconds, timeout, log):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Install pyserial: python -m pip install pyserial") from exc
    serial_port = serial.Serial(port, baudrate=baud, timeout=0.4)
    ass = Assembler()
    samples_all = []
    log.write("# OLED/MPU physical UART1 evidence\n")
    def read_one():
        line = serial_port.readline().decode("utf-8", "replace").strip()
        if line:
            log.write(line+"\n")
            log.flush()
        return ass.feed(line)
    try:
        boot_deadline = time.monotonic() + timeout
        print("Waiting for calibrated gyro and motor lock (no movement).", flush=True)
        while time.monotonic() < boot_deadline:
            s=read_one()
            if s is not None and s["cal"] == 1 and s["locked"] == 1:
                if s["ready"] != 1:
                    raise RuntimeError("OLED not detected; cannot assess coexistence")
                break
        else:
            raise RuntimeError("Boot/calibration telemetry timeout")
        result = []
        for mode, seconds, label in ((0, baseline_seconds, "baseline"),
                                     (1, oled_seconds, "oled_on")):
            serial_port.write(bytes([ord("0") + mode]))
            serial_port.flush()
            print("Capturing %s for %ds" % (label, seconds), flush=True)
            start = time.monotonic()
            rows=[]
            while time.monotonic()-start < seconds+1.8:
                s=read_one()
                if s is None:
                    continue
                if s["on"] != mode:
                    continue
                if time.monotonic()-start < 1.0:
                    continue # transition and residual DMA
                rows.append(s)
                samples_all.append({"phase": label, **s})
            result.append(segment(rows, mode))
        serial_port.write(b"0")
        serial_port.flush()
        return result, samples_all, {"malformed": ass.malformed, "missing": ass.missing}
    finally:
        try:
            serial_port.write(b"0")
            serial_port.flush()
        except Exception:
            pass
        serial_port.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="UART1 USB-TTL serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--baseline-seconds", type=int, default=15)
    ap.add_argument("--oled-seconds", type=int, default=15)
    ap.add_argument("--boot-timeout", type=int, default=65)
    ap.add_argument("--output", type=Path, default=Path("results"))
    ap.add_argument("--flash", action="store_true",
                    help="Flash motor-locked oled_mpu_diag_96m first via PlatformIO/ST-Link")
    ap.add_argument("--self-test", action="store_true", help="Host parser only; NOT MCU evidence")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if not args.port:
        ap.error("--port required for physical acceptance")
    if args.baseline_seconds < 8 or args.oled_seconds < 8:
        ap.error("Both phases must be >=8s for meaningful deltas")
    if args.flash:
        subprocess.run([sys.executable, "-m", "platformio", "run", "-e",
                        "oled_mpu_diag_96m", "-t", "upload"], check=True)
    folder=args.output / ("oled_mpu_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    folder.mkdir(parents=True,exist_ok=False)
    raw=folder/"uart_raw.log"
    try:
        with raw.open("w",encoding="utf-8") as fp:
            (off,on), rows, transport = capture(
                args.port,args.baud,args.baseline_seconds,args.oled_seconds,
                args.boot_timeout,fp)
        report=evaluate(off,on)
        report["host_uart"] = transport
        if transport["malformed"] or transport["missing"]:
            report["checks"]["complete_uart_framing"] = False
            report["verdict"] = "FAIL"
        else:
            report["checks"]["complete_uart_framing"] = True
        (folder/"report.json").write_text(
            json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
        with (folder/"telemetry.csv").open("w",newline="",encoding="utf-8") as fp:
            w=csv.DictWriter(fp,fieldnames=["phase"]+sorted(REQUIRED)+["seq"])
            w.writeheader()
            for row in rows:
                w.writerow({k:row.get(k,"") for k in w.fieldnames})
        print(json.dumps(report,indent=2,ensure_ascii=False))
        print("Evidence:",folder)
        return 0 if report["verdict"]=="PASS" else 2
    except (RuntimeError,ValueError,KeyboardInterrupt) as exc:
        failed={"verdict":"INCOMPLETE","reason":str(exc),
                "note":"Never claim hardware PASS from incomplete telemetry."}
        (folder/"report.json").write_text(json.dumps(failed,indent=2))
        print(json.dumps(failed),file=sys.stderr)
        return 2


if __name__=="__main__":
    sys.exit(main())
