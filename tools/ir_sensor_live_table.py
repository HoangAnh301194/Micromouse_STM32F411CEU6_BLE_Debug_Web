#!/usr/bin/env python3
"""Read six IR receiver channels as aligned columns from SYS_IRV telemetry.

No firmware flashing, motor commands, serial writes, or GPIO changes.
The current INTEGRATION_DIAG firmware emits SYS_IRV about once per second,
although the ADC acquisition loop runs at 1 kHz.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Iterator

SENSORS = ("L90", "L45", "L0", "R0", "R45", "R90")
COLUMNS = ("ms", "scan", "age_us") + tuple(
    f"{sensor}_{quantity}"
    for sensor in SENSORS
    for quantity in ("ambient", "led_on", "signed_diff", "ir_value")
)


def parse_irv(line: str) -> dict[str, int] | None:
    """Reject partial/malformed SYS_IRV; ignore all other telemetry."""
    if not line.startswith("SYS_IRV,"):
        return None
    parsed: dict[str, int] = {}
    for field in line.strip().split(",")[1:]:
        key, sep, value = field.partition("=")
        if not sep or key in parsed:
            return None
        try:
            parsed[key] = int(value)
        except ValueError:
            return None
    required = ("ms", "scan", "age_us") + tuple(
        f"{prefix}{i}" for prefix in ("a", "l", "d") for i in range(6)
    )
    return parsed if all(key in parsed for key in required) else None


def normalized(raw: dict[str, Any]) -> dict[str, int] | None:
    """Validate the same required fields in the saved summary format."""
    return parse_irv(
        "SYS_IRV," + ",".join(
            f"{key}={value}" for key, value in raw.items() if key != "type"
        )
    )


def rows(record: dict[str, int]) -> list[list[int]]:
    ambient = [record[f"a{i}"] for i in range(6)]
    active = [record[f"l{i}"] for i in range(6)]
    return [
        ambient,
        active,
        [active[i] - ambient[i] for i in range(6)],
        [record[f"d{i}"] for i in range(6)],
    ]


def formatted_table(record: dict[str, int], index: int) -> str:
    labels = ("LED OFF", "LED ON", "ON - OFF", "IR OUTPUT")
    data = rows(record)
    cell_width = 9
    header = f"{'':<11}" + "".join(f"{name:>{cell_width}}" for name in SENSORS)
    lines = [
        f"IR RX monitor | record {index} | MCU {record['ms']/1000:.3f} s "
        f"| scan {record['scan']} | age {record['age_us']} us",
        "Values: raw ADC counts; signed ON-OFF can be negative, "
        "IR OUTPUT is clipped at 0 by firmware.",
        header,
        "-" * len(header),
    ]
    for label, values in zip(labels, data):
        lines.append(f"{label:<11}" + "".join(f"{value:>{cell_width}d}" for value in values))
    mismatches = [
        SENSORS[i] for i in range(6)
        if data[3][i] != max(data[2][i], 0)
    ]
    if mismatches:
        lines.append("WARNING: ON-OFF vs IR OUTPUT mismatch: " + ", ".join(mismatches))
    return "\n".join(lines)


def csv_row(record: dict[str, int]) -> dict[str, int]:
    row = {key: record[key] for key in ("ms", "scan", "age_us")}
    values = rows(record)
    for i, sensor in enumerate(SENSORS):
        for value_key, data in zip(("ambient", "led_on", "signed_diff", "ir_value"), values):
            row[f"{sensor}_{value_key}"] = data[i]
    return row


def replay(path: Path) -> Iterator[dict[str, int]]:
    if path.suffix.lower() == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
        for obj in data.get("raw_value_samples", []):
            parsed = normalized(obj)
            if parsed is not None:
                yield parsed
    else:
        with path.open(encoding="utf-8", errors="replace") as source:
            for line in source:
                parsed = parse_irv(line)
                if parsed is not None:
                    yield parsed


def live(port: str, seconds: float = 0) -> Iterator[dict[str, int]]:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Requires pyserial: python -m pip install pyserial") from exc

    try:
        connection = serial.Serial(port, 115200, timeout=0.5)
    except (OSError, serial.SerialException) as exc:
        raise RuntimeError(f"Cannot open {port}: {exc}") from exc

    with connection:
        last_valid = time.monotonic()
        deadline = last_valid + seconds if seconds else None
        while deadline is None or time.monotonic() < deadline:
            try:
                raw = connection.readline()
            except serial.SerialException as exc:
                raise RuntimeError(f"Read failed on {port}: {exc}") from exc
            parsed = parse_irv(raw.decode("ascii", errors="replace").strip())
            if parsed is not None:
                last_valid = time.monotonic()
                yield parsed
            elif time.monotonic() - last_valid >= 6:
                print("No complete SYS_IRV for 6 s. Check COM port and "
                      "INTEGRATION_DIAG motor-off firmware.", file=sys.stderr, flush=True)
                last_valid = time.monotonic()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="USB-UART device for PA9 telemetry, e.g. COM7")
    source.add_argument("--input", type=Path, help="Replay summary.json or uart.log")
    p.add_argument("--seconds", type=float, default=0,
                   help="Live duration; 0 (default) runs until Ctrl+C")
    p.add_argument("--count", type=int, default=0, help="Stop after N valid records")
    p.add_argument("--csv", type=Path, help="Write rows to a NEW CSV file (never overwrite)")
    p.add_argument("--scroll", action="store_true",
                   help="Print successive tables instead of refreshing terminal")
    args = p.parse_args(argv)
    if args.seconds < 0 or args.count < 0:
        p.error("--seconds and --count must be nonnegative")
    if args.input and args.seconds:
        p.error("--seconds is only for live serial mode")

    csv_handle = None
    try:
        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            csv_handle = args.csv.open("x", encoding="utf-8", newline="")
            csv_writer = csv.DictWriter(csv_handle, fieldnames=COLUMNS)
            csv_writer.writeheader()

        print("Read-only monitor: no motor commands, no firmware upload.", flush=True)
        if args.port:
            print("SYS_IRV output is ~1 Hz; ADC scans at 1 kHz. Ctrl+C to stop.", flush=True)
        stream = live(args.port, args.seconds) if args.port else replay(args.input)
        count = 0
        for record in stream:
            count += 1
            if not args.scroll and sys.stdout.isatty():
                os.system("cls" if os.name == "nt" else "clear")
            print(formatted_table(record, count), end="\n\n", flush=True)
            if csv_handle is not None:
                csv_writer.writerow(csv_row(record))
                csv_handle.flush()
            if args.count and count >= args.count:
                break
        if count == 0:
            print("No complete SYS_IRV records.", file=sys.stderr)
            return 1
        print(f"Read {count} complete SYS_IRV records.")
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        if csv_handle is not None:
            csv_handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
