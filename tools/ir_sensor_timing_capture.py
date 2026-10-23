#!/usr/bin/env python3
"""Capture motor-off IR timing diagnostics from USART1 PA9/PA10."""
from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
NOMINAL_SYSCLK_HZ = 96_000_000
REQUIRED_FIELDS = {
    "SYS_IR": ("ms", "new", "old", "age_us", "age_max_us"),
    "SYS_IRD": (
        "ms", "start", "done", "fail", "dma_start", "dma_done", "dma_err",
        "dma_to", "adc_ovr", "scan_us", "scan_max_us", "dma_us", "dma_max_us",
    ),
    "SYS_IRC": (
        "ms", "tim_irq", "tim_cy", "tim_maxcy", "dma_irq", "dma_cy",
        "dma_maxcy", "adc_clk",
    ),
    "SYS_IRV": (
        "ms", "scan", "age_us", "ambient_reads", "pair1_reads", "pair2_reads",
        "pair3_reads", "a0", "a1", "a2", "a3", "a4", "a5", "l0", "l1",
        "l2", "l3", "l4", "l5", "d0", "d1", "d2", "d3", "d4", "d5",
    ),
    "SYS_T": ("ms", "ticks", "late", "gap", "maxcy"),
    "SYS_M": ("ms", "i2cok", "i2cerr", "to"),
    "SYS_U": ("ms", "tx_drop", "dma_err", "rx_err", "rx_ov"),
    "SYS_UQ": (
        "ms", "q", "peak", "accb", "doneb", "prncy", "dmacy", "rxcy", "rxb",
    ),
}


def parse_record(line: str) -> dict[str, int | str] | None:
    fields = line.strip().split(",")
    if not fields or fields[0] not in {
        "SYS_IR", "SYS_IRD", "SYS_IRC", "SYS_IRV", "SYS_T", "SYS_M",
        "SYS_U", "SYS_UQ"
    }:
        return None
    record: dict[str, int | str] = {"type": fields[0]}
    for field in fields[1:]:
        key, separator, value = field.partition("=")
        if not separator:
            record["_malformed"] = 1
            continue
        try:
            record[key] = int(value)
        except ValueError:
            continue
    return record


def counter_delta(first: dict, last: dict, key: str) -> int:
    return (int(last.get(key, 0)) - int(first.get(key, 0))) & 0xFFFFFFFF


def summarize(records: list[dict], captured_seconds: float, port: str) -> dict:
    by_type: dict[str, list[dict]] = {}
    for record in records:
        by_type.setdefault(str(record["type"]), []).append(record)

    result: dict = {
        "port": port,
        "baud": 115200,
        "capture_seconds": round(captured_seconds, 3),
        "record_counts": {key: len(value) for key, value in by_type.items()},
        "raw_value_samples": by_type.get("SYS_IRV", []),
        "malformed_records": sum(
            1 for record in records
            if record.get("_malformed", 0)
            or any(field not in record
                   for field in REQUIRED_FIELDS.get(str(record["type"]), ()))
        ),
    }
    for category in (
        "SYS_IRD", "SYS_IRC", "SYS_IR", "SYS_T", "SYS_M", "SYS_U", "SYS_UQ"
    ):
        rows = by_type.get(category, [])
        if len(rows) < 2:
            continue
        first, last = rows[0], rows[-1]
        elapsed_ms = counter_delta(first, last, "ms")
        delta_records = {"elapsed_ms": elapsed_ms}
        if category == "SYS_IRD":
            delta_records.update({
                "scans_started": counter_delta(first, last, "start"),
                "scans_completed": counter_delta(first, last, "done"),
                "scans_failed": counter_delta(first, last, "fail"),
                "dma_started": counter_delta(first, last, "dma_start"),
                "dma_completed": counter_delta(first, last, "dma_done"),
                "dma_errors": counter_delta(first, last, "dma_err"),
                "dma_timeouts": counter_delta(first, last, "dma_to"),
                "adc_overruns": counter_delta(first, last, "adc_ovr"),
            })
            if elapsed_ms:
                delta_records["scan_rate_hz"] = round(
                    delta_records["scans_completed"] * 1000.0 / elapsed_ms, 2
                )
            completed = delta_records["scans_completed"]
            if completed:
                delta_records["mean_scan_time_us"] = round(
                    counter_delta(first, last, "scan_us") / completed, 2
                )
            dma_completed = delta_records["dma_completed"]
            if dma_completed:
                delta_records["mean_dma_completion_us"] = round(
                    counter_delta(first, last, "dma_us") / dma_completed, 2
                )
            delta_records["scan_max_us_observed"] = int(last.get("scan_max_us", 0))
            delta_records["dma_max_us_observed"] = int(last.get("dma_max_us", 0))
        elif category == "SYS_IRC":
            cycle_delta = (
                counter_delta(first, last, "tim_cy")
                + counter_delta(first, last, "dma_cy")
            )
            delta_records["ir_isr_cycles"] = cycle_delta
            if elapsed_ms:
                delta_records["ir_isr_cpu_percent_of_nominal_sysclk"] = round(
                    100.0 * cycle_delta
                    / (NOMINAL_SYSCLK_HZ * elapsed_ms / 1000.0),
                    4,
                )
            delta_records["tim10_irq_max_cycles"] = int(last.get("tim_maxcy", 0))
            delta_records["dma_irq_max_cycles"] = int(last.get("dma_maxcy", 0))
            delta_records["adc_clock_nominal_hz"] = int(last.get("adc_clk", 0))
        elif category == "SYS_IR":
            delta_records["fresh_control_ticks"] = counter_delta(first, last, "new")
            delta_records["ticks_without_new_scan"] = counter_delta(first, last, "old")
            delta_records["max_sample_age_us"] = int(last.get("age_max_us", 0))
            total_ticks = (
                delta_records["fresh_control_ticks"]
                + delta_records["ticks_without_new_scan"]
            )
            if total_ticks:
                delta_records["fresh_control_tick_percent"] = round(
                    100.0 * delta_records["fresh_control_ticks"] / total_ticks, 2
                )
        elif category == "SYS_T":
            delta_records["tim11_late_delta"] = counter_delta(first, last, "late")
            delta_records["tim11_gap_delta"] = counter_delta(first, last, "gap")
            delta_records["tim11_max_irq_cycles"] = int(last.get("maxcy", 0))
        elif category == "SYS_M":
            delta_records["mpu_i2c_errors"] = counter_delta(first, last, "i2cerr")
            delta_records["mpu_i2c_timeouts"] = counter_delta(first, last, "to")
            delta_records["mpu_fresh_gyro_reads"] = counter_delta(first, last, "i2cok")
        elif category == "SYS_U":
            delta_records["uart_tx_drops"] = counter_delta(first, last, "tx_drop")
            delta_records["uart_dma_errors"] = counter_delta(first, last, "dma_err")
            delta_records["uart_rx_errors"] = counter_delta(first, last, "rx_err")
            delta_records["uart_rx_overflows"] = counter_delta(first, last, "rx_ov")
        elif category == "SYS_UQ":
            delta_records["uart_bytes_accepted"] = counter_delta(first, last, "accb")
            delta_records["uart_bytes_completed"] = counter_delta(first, last, "doneb")
            delta_records["uart_printf_cycles"] = counter_delta(first, last, "prncy")
            delta_records["uart_queue_peak_bytes"] = int(last.get("peak", 0))
            delta_records["uart_queue_bytes_at_end"] = int(last.get("q", 0))
        result[category] = delta_records

    result["has_ir_values"] = len(by_type.get("SYS_IRV", [])) >= 2
    result["has_timing_counters"] = (
        len(by_type.get("SYS_IRD", [])) >= 2
        and len(by_type.get("SYS_IRC", [])) >= 2
    )
    result["has_uart_health"] = len(by_type.get("SYS_U", [])) >= 2
    uart_stats = result.get("SYS_U", {})
    result["uart_transport_clean"] = (
        result["has_uart_health"]
        and uart_stats.get("uart_tx_drops", 1) == 0
        and uart_stats.get("uart_dma_errors", 1) == 0
        and uart_stats.get("uart_rx_errors", 1) == 0
        and uart_stats.get("uart_rx_overflows", 1) == 0
    )
    result["valid_capture"] = (
        result["has_ir_values"]
        and result["has_timing_counters"]
        and result["uart_transport_clean"]
        and result["malformed_records"] == 0
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB-UART COM port, e.g. COM7")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.seconds < 3.0:
        parser.error("--seconds must be at least 3")

    try:
        import serial
    except ImportError:
        print("pyserial is required; install it in the selected Python environment.",
              file=sys.stderr)
        return 2

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or ROOT / "results" / f"ir_timing_{stamp}"
    try:
        connection = serial.Serial(args.port, 115200, timeout=0.25)
    except (OSError, serial.SerialException) as exc:
        print(f"Unable to open {args.port}: {exc}", file=sys.stderr)
        return 2

    records: list[dict] = []
    raw_lines: list[str] = []
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            raw = connection.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            raw_lines.append(line)
            record = parse_record(line)
            if record is not None:
                records.append(record)
    finally:
        connection.close()

    if not raw_lines:
        print(f"No UART data received from {args.port}; no result files written.",
              file=sys.stderr)
        return 1

    elapsed = time.monotonic() - start
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "uart.log").write_text(
        "".join(line + "\n" for line in raw_lines), encoding="utf-8"
    )
    summary = summarize(records, elapsed, args.port)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Saved {len(raw_lines)} UART lines to {output_dir}")
    print(json.dumps({key: value for key, value in summary.items()
                      if key != "raw_value_samples"}, indent=2))
    if not summary["valid_capture"]:
        print("Capture is incomplete: need multiple complete SYS_IRD/SYS_IRC, "
              "SYS_IRV and SYS_U records, with no malformed telemetry or "
              "UART errors/drops.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
