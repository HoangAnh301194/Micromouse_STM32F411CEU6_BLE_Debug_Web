#!/usr/bin/env python3
"""Read-only encoder hand-turn probe using existing 1-Hz SYS_E telemetry.

Run with the motor VM physically disconnected and a verified motor-locked
INTEGRATION_DIAG firmware. No serial writes, commands, flashing or PWM.
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys
import time
from typing import Iterator

REQUIRED = ("ms", "encL", "encR", "ppsL", "ppsR", "appL", "appR")
CSV_COLUMNS = REQUIRED + ("relativeL", "relativeR", "deltaL", "deltaR")


def parse_sys_e(line: str) -> dict[str, int] | None:
    if not line.startswith("SYS_E,"):
        return None
    values: dict[str, int] = {}
    for fragment in line.strip().split(",")[1:]:
        key, separator, val = fragment.partition("=")
        if not separator or key in values:
            return None
        try:
            values[key] = int(val)
        except ValueError:
            return None
    return values if all(key in values for key in REQUIRED) else None


def reader_from_file(path: Path) -> Iterator[str]:
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        yield from stream


def reader_from_port(port: str, seconds: float) -> Iterator[str]:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Install pyserial: python -m pip install pyserial") from exc
    try:
        stream = serial.Serial(port, 115200, timeout=0.5)
    except (OSError, serial.SerialException) as exc:
        raise RuntimeError(f"Cannot open {port}: {exc}") from exc
    with stream:
        start = time.monotonic()
        last_notice = start
        while not seconds or time.monotonic() - start < seconds:
            try:
                raw = stream.readline()
            except serial.SerialException as exc:
                raise RuntimeError(f"Serial read failed: {exc}") from exc
            if raw:
                last_notice = time.monotonic()
                yield raw.decode("ascii", errors="replace")
            elif time.monotonic() - last_notice > 6:
                print("No UART data for 6s; check COM port and firmware.",
                      file=sys.stderr, flush=True)
                last_notice = time.monotonic()


def display_row(record: dict[str, int], baseline: dict[str, int],
                previous: dict[str, int], index: int) -> dict[str, int]:
    row = dict(record)
    for side in ("L", "R"):
        row["relative" + side] = record["enc" + side] - baseline["enc" + side]
        row["delta" + side] = record["enc" + side] - previous["enc" + side]
    print(f"#{index:03d}  t={record['ms']/1000:9.3f}s  "
          f"L count={row['relativeL']:+9d}  Δ={row['deltaL']:+7d}  "
          f"pps={record['ppsL']:+8d} | "
          f"R count={row['relativeR']:+9d}  Δ={row['deltaR']:+7d}  "
          f"pps={record['ppsR']:+8d}", flush=True)
    return row


def self_test() -> None:
    good = "SYS_E,ms=1000,encL=-20,encR=35,ppsL=-40,ppsR=70,appL=0,appR=0"
    parsed = parse_sys_e(good)
    assert parsed is not None and parsed["encL"] == -20
    assert parse_sys_e("SYS_IR,ms=1000,new=1") is None
    assert parse_sys_e(good.replace(",appR=0", "")) is None
    assert parse_sys_e(good + ",encL=9") is None
    assert parse_sys_e(good.replace("ppsR=70", "ppsR=xxx")) is None
    base = {**parsed, "encL": -25, "encR": 30}
    row = dict(parsed)
    assert row["encL"] - base["encL"] == 5
    assert row["encR"] - base["encR"] == 5
    print("SELF-TEST PASS: SYS_E parsing, malformed rejects, signed counts.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--port", help="COM7, or another USB-UART port (read only)")
    sources.add_argument("--input", type=Path, help="Replay existing uart.log")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="Live duration; 0 runs until Ctrl+C")
    parser.add_argument("--count", type=int, default=0,
                        help="Stop after N complete SYS_E records")
    parser.add_argument("--csv", type=Path,
                        help="Save NEW CSV with absolute and relative counts")
    parser.add_argument("--uart-log", type=Path,
                        help="Save NEW raw UART log, including other telemetry")
    args = parser.parse_args(argv)
    if args.self_test:
        self_test()
        return 0
    if not args.port and not args.input:
        parser.error("specify --port or --input (or --self-test)")
    if args.seconds < 0 or args.count < 0:
        parser.error("--seconds and --count cannot be negative")
    if args.input and args.seconds:
        parser.error("--seconds is only valid for live serial captures")

    handles = []
    try:
        csv_writer = None
        raw_output = None
        paths = [p for p in (args.csv, args.uart_log) if p is not None]
        if len(paths) == 2 and paths[0].resolve() == paths[1].resolve():
            parser.error("--csv and --uart-log must be different paths")
        if args.input and any(p.resolve() == args.input.resolve() for p in paths):
            parser.error("output cannot overwrite the input")
        for p in paths:
            p.parent.mkdir(parents=True, exist_ok=True)
        if args.csv:
            csv_file = args.csv.open("x", newline="", encoding="utf-8")
            handles.append(csv_file)
            csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS,
                                        extrasaction="ignore")
            csv_writer.writeheader()
        if args.uart_log:
            raw_output = args.uart_log.open("x", encoding="utf-8")
            handles.append(raw_output)

        print("ENCODER | read-only 115200 8N1 | press Ctrl+C to stop")
        print("Safety: motor VM disconnected; appL/appR MUST be 0.")
        print("Baseline = first SYS_E. Turn each wheel manually, count full turns.")
        print("PPS is the existing firmware's filtered value, NOT calibrated RPM.\n")
        src = (reader_from_port(args.port, args.seconds) if args.port
               else reader_from_file(args.input))
        first = prev = last = None
        frames = 0
        for line in src:
            if raw_output is not None:
                raw_output.write(line)
            parsed = parse_sys_e(line)
            if parsed is None:
                continue
            if parsed["appL"] != 0 or parsed["appR"] != 0:
                print("STOP: nonzero applied motor PWM observed. "
                      "This test requires motor locked and VM disconnected.",
                      file=sys.stderr)
                return 3
            if first is None:
                first = prev = parsed
            frames += 1
            row = display_row(parsed, first, prev, frames)
            if csv_writer is not None:
                csv_writer.writerow(row)
                handles[0].flush()
            prev = last = parsed
            if args.count and frames >= args.count:
                break
        if first is None or last is None:
            print("No valid SYS_E records; check INTEGRATION_DIAG motor-off "
                  "firmware and serial port.", file=sys.stderr)
            return 1
        print(f"\nSummary: {frames} valid samples; total since baseline: "
              f"L={last['encL']-first['encL']:+d}, "
              f"R={last['encR']-first['encR']:+d} timer counts.")
        print("If a wheel was manually rotated N full revolutions, "
              "use abs(delta counts)/N to estimate decoded counts/wheel rev.")
        return 0
    except KeyboardInterrupt:
        print("\nStopped by operator; no serial commands sent.")
        return 0
    except (OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        for handle in handles:
            handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
