#!/usr/bin/env python3
"""Compare STM32 TIM5/TIM11 time with Windows perf_counter via USART1 USB-TTL.

Firmware must emit once per second:
    CLK,seq=1,us=1234567,ticks=1234,tx_drop=0

Usage:
    python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 240
    python tools/clock_serial_test.py --input clock_logs/SESSION/clock_samples.csv
    python tools/clock_serial_test.py --self-test

The PC monotonic clock provides an independent *approximate* reference. UART,
USB buffering and OS scheduling add receipt-time jitter; estimate long-term
slope, not one individual inter-arrival period. Never use these measurements
to tune PID until the clock discrepancy has been investigated.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import pathlib
import statistics
import sys
import tempfile
import time

EXPECTED_PREFIX = "CLK,"
U32 = 2**32
FIELDS = ["host_iso", "host_perf_s", "session", "seq", "mcu_us",
          "ticks", "tx_drop"]


def parse_clk(line: str):
    if not line.startswith(EXPECTED_PREFIX):
        return None
    parts = {}
    for field in line[4:].split(","):
        if "=" not in field:
            return None
        name, value = field.strip().split("=", 1)
        parts[name] = value
    try:
        return {k: int(parts[k]) for k in ("seq", "us", "ticks", "tx_drop")}
    except (KeyError, ValueError):
        return None


def sessionize(rows):
    """Separate resets without mistaking TIM5's 2^32 wrap for a reset."""
    groups = []
    current = []
    previous = None
    for item in rows:
        v = item["mcu_us"]
        if previous is not None:
            seq_restart = item["seq"] <= previous["seq"]
            backward = v < previous["mcu_us"]
            rollover = backward and previous["mcu_us"] > 0xF0000000 and v < 0x10000000
            discontinuity = backward and not rollover
            if seq_restart or discontinuity:
                if current:
                    groups.append(current)
                current = []
        current.append(item)
        previous = item
    if current:
        groups.append(current)
    return groups


def unwrap(group):
    offset = 0
    previous = None
    data = []
    for item in group:
        if previous is not None and item["mcu_us"] < previous:
            offset += U32
        copy = dict(item)
        copy["unwrapped_us"] = copy["mcu_us"] + offset
        data.append(copy)
        previous = item["mcu_us"]
    return data


def percentile(data, pct):
    data = sorted(data)
    if not data:
        return float("nan")
    position = (len(data) - 1) * pct / 100
    lower = math.floor(position)
    upper = math.ceil(position)
    return data[lower] + (data[upper] - data[lower]) * (position - lower)


def least_squares(x, y):
    mx = statistics.mean(x)
    my = statistics.mean(y)
    cov = sum((a - mx) * (b - my) for a, b in zip(x, y))
    var = sum((a - mx) ** 2 for a in x)
    if var == 0:
        raise ValueError("All MCU timestamps are identical")
    slope = cov / var
    intercept = my - slope * mx
    return slope, intercept


def analyze(rows):
    groups = sessionize(rows)
    if not groups:
        raise ValueError("No valid CLK records. Pull/build/upload new firmware.")
    # The user can reset after opening the port: analyze the longest uninterrupted
    # sequence rather than concatenating old and new sessions.
    group = max(groups, key=lambda g: len(g))
    if len(group) < 30:
        raise ValueError(f"Only {len(group)} CLK records in the longest session; "
                         "need >=30 (prefer 180+) for a reliable clock comparison.")
    group = unwrap(group)
    # Discard the first few readings, where a reset/USB backlog is more likely.
    used = group[3:] if len(group) >= 40 else group
    first = used[0]
    device_s = [(r["unwrapped_us"] - first["unwrapped_us"]) / 1e6 for r in used]
    host_s = [r["host_perf_s"] - first["host_perf_s"] for r in used]
    tick_s = [r["ticks"] - first["ticks"] for r in used]
    slope, intercept = least_squares(device_s, host_s)

    # Long-baseline overlapping intervals reduce the effect of timestamp
    # quantization/USB packet arrival jitter; use windows >=~30 MCU seconds.
    strides = [30, 60]
    robust = {}
    for stride in strides:
        if len(used) > stride + 12:
            ratios = [(host_s[i + stride] - host_s[i]) /
                      (device_s[i + stride] - device_s[i])
                      for i in range(len(used) - stride)
                      if device_s[i + stride] > device_s[i]]
            if ratios:
                robust[stride] = statistics.median(ratios)
    predicted = [intercept + slope * val for val in device_s]
    abs_residual_ms = [abs((h - p) * 1000) for h, p in zip(host_s, predicted)]
    interval_s = [host_s[i] - host_s[i - 1] for i in range(1, len(used))]
    seq_gaps = sum(max(0, used[i]["seq"] - used[i - 1]["seq"] - 1)
                   for i in range(1, len(used)))
    drop_delta = max(0, used[-1]["tx_drop"] - used[0]["tx_drop"])
    device_rate = 1.0 / slope
    host_span = host_s[-1]
    device_span = device_s[-1]
    tick_span = tick_s[-1]
    statistics_out = {
        "sessions": len(groups),
        "selected_session_records": len(group),
        "fit_records": len(used),
        "seq_first": used[0]["seq"],
        "seq_last": used[-1]["seq"],
        "host_span_s": host_span,
        "device_span_s": device_span,
        "pc_seconds_per_mcu_second": slope,
        "pc_minus_mcu_pct": (slope - 1.0) * 100.0,
        "effective_tim5_hz": device_rate * 1_000_000,
        "mcu_clock_rate_error_pct": (device_rate - 1.0) * 100,
        "effective_tim11_hz": tick_span / host_span if host_span > 0 else float("nan"),
        "tim11_ticks_per_mcu_second": tick_span / device_span,
        "fit_residual_p50_ms": percentile(abs_residual_ms, 50),
        "fit_residual_p95_ms": percentile(abs_residual_ms, 95),
        "host_interval_median_s": statistics.median(interval_s),
        "host_interval_p95_s": percentile(interval_s, 95),
        "seq_missing_count": seq_gaps,
        "tx_drop_delta": drop_delta,
        "robust_30s_pc_per_mcu": robust.get(30),
        "robust_60s_pc_per_mcu": robust.get(60),
    }
    return statistics_out, used, device_s, host_s, predicted


def format_summary(result):
    s = []
    s.append("STM32 vs Windows monotonic clock comparison")
    s.append("=" * 50)
    for k, value in result.items():
        if value is None:
            continue
        if isinstance(value, float):
            s.append(f"{k}: {value:.6f}")
        else:
            s.append(f"{k}: {value}")
    s += [
        "",
        "Interpretation:",
        "  PC seconds / MCU second > 1 means the MCU's nominal microsecond",
        "  counter advances SLOWER than the host monotonic clock.",
        "  A 4.5% time-ratio error corresponds to about -4.3% MCU tick rate,",
        "  not +4.5% MCU frequency. These denominators are different.",
        "  TIM5 and TIM11 share the MCU clock; matching both is not a proof",
        "  that either has the correct *physical* frequency.",
        "  The p95 fit residual is dominated by host USB/serial scheduling",
        "  jitter; it is NOT TIM11 ISR jitter.",
    ]
    if result["seq_missing_count"] or result["tx_drop_delta"]:
        s += ["", "WARNING: Missing CLK sequences or UART message drops;",
              "repeat with an unloaded PC and verify 115200 baud."]
    if result["fit_records"] < 120:
        s += ["", "WARNING: Prefer at least 120 CLK samples for confirmation."]
    if abs(result["mcu_clock_rate_error_pct"]) > 1:
        s += ["", "CHECK: Significant apparent clock mismatch (>1%).",
              "Re-run twice and consider a reference measurement before",
              "changing PLL, prescalers, gyro integration or PID dt."]
    return "\n".join(s) + "\n"


def plots(outdir: pathlib.Path, used, device_s, host_s, predicted):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; CSV and summary are still saved.")
        return
    difference = [h - d for h, d in zip(host_s, device_s)]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(host_s, difference, marker=".", markersize=2)
    ax.set(xlabel="PC monotonic elapsed time (s)",
           ylabel="PC elapsed - nominal MCU elapsed (s)",
           title="Accumulated difference between host and STM32 time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(outdir / "01_clock_drift.png", dpi=150)
    plt.close(fig)

    residual = [(h - p) * 1000 for h, p in zip(host_s, predicted)]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(host_s, residual, marker=".", markersize=2, linestyle="none")
    ax.axhline(0, linewidth=1)
    ax.set(xlabel="PC monotonic elapsed time (s)",
           ylabel="Host receipt residual from best-fit line (ms)",
           title="UART/USB/PC timestamp jitter (not MCU ISR jitter)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(outdir / "02_serial_timestamp_jitter.png", dpi=150)
    plt.close(fig)


def read_csv(path: pathlib.Path):
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    parsed = []
    for row in rows:
        try:
            parsed.append({
                "host_iso": row.get("host_iso", ""),
                "host_perf_s": float(row["host_perf_s"]),
                "session": int(row.get("session", 0)),
                "seq": int(row["seq"]),
                "mcu_us": int(row["mcu_us"]),
                "ticks": int(row["ticks"]),
                "tx_drop": int(row["tx_drop"]),
            })
        except (KeyError, ValueError, TypeError):
            continue
    return parsed


def save_report(outdir, rows):
    result, used, device_s, host_s, predicted = analyze(rows)
    with (outdir / "clock_summary.txt").open("w", encoding="utf-8") as f:
        f.write(format_summary(result))
    plots(outdir, used, device_s, host_s, predicted)
    print("\n" + format_summary(result))
    print("Output:", outdir.resolve())


def self_test():
    import random
    rng = random.Random(100)
    fake = []
    # Simulate a short pre-RESET segment, followed by a longer new segment.
    for session, count in [(0, 8), (1, 200)]:
        for i in range(count):
            fake.append({
                "host_iso": "",
                "host_perf_s": (i * 1.04535 + rng.uniform(-0.004, 0.004))
                               + (100 if session == 1 else 0),
                "session": session,
                "seq": i + 1,
                "mcu_us": 1_500_000 + i * 1_000_000,
                "ticks": 1000 + i * 1000,
                "tx_drop": 0,
            })
    result, *_ = analyze(fake)
    assert result["sessions"] == 2, result
    assert abs(result["pc_seconds_per_mcu_second"] - 1.04535) < 0.0002, result
    assert abs(result["effective_tim11_hz"] - (1000 / 1.04535)) < 0.4, result
    print("Self-test passed: RESET split, 4.535% host/MCU time ratio, "
          "regression and TIM11 estimation.")
    print(format_summary(result))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--port", help="Serial port, e.g. COM7 (close PlatformIO monitor)")
    mode.add_argument("--input", type=pathlib.Path,
                      help="Existing clock_samples.csv for offline reanalysis")
    mode.add_argument("--self-test", action="store_true",
                      help="Run deterministic synthetic test without hardware")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=240,
                    help="PC wall-time capture duration; default 240 seconds")
    ap.add_argument("--out", type=pathlib.Path,
                    help="Output directory (default clock_logs/timestamp)")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    outdir = args.out or pathlib.Path("clock_logs") / dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir.mkdir(parents=True, exist_ok=True)
    if args.input:
        if not args.input.is_file():
            ap.error("--input must point to clock_samples.csv, not a folder")
        rows = read_csv(args.input)
        save_report(outdir, rows)
        return
    if args.seconds < 30:
        ap.error("Please capture >=30 seconds; 240 seconds recommended.")

    try:
        import serial
    except ImportError:
        ap.error("Missing pyserial. Install with: python -m pip install pyserial")
    rows = []
    session = 0
    last_seq = None
    t0_ns = time.perf_counter_ns()
    print(f"Opening {args.port} at {args.baud} baud.")
    print("After port opens, press STM32 RESET once, then leave board untouched.")
    print(f"Recording {args.seconds:g} PC seconds; outputs in {outdir}.")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            ser.reset_input_buffer()
            t0_ns = time.perf_counter_ns()
            with (outdir / "raw_serial.log").open("w", encoding="utf-8") as raw:
                while (time.perf_counter_ns() - t0_ns) / 1e9 < args.seconds:
                    payload = ser.readline()
                    received_ns = time.perf_counter_ns()
                    if not payload:
                        continue
                    host_s = (received_ns - t0_ns) / 1e9
                    line = payload.decode("utf-8", errors="replace").strip()
                    raw.write(f"{host_s:.9f}\t{line}\n")
                    value = parse_clk(line)
                    if value is None:
                        continue
                    if last_seq is not None and value["seq"] <= last_seq:
                        session += 1
                        print(f"Detected STM32 restart; now tracking session {session}.")
                    last_seq = value["seq"]
                    rows.append({
                        "host_iso": dt.datetime.now(dt.timezone.utc).isoformat(),
                        "host_perf_s": host_s,
                        "session": session,
                        "seq": value["seq"],
                        "mcu_us": value["us"],
                        "ticks": value["ticks"],
                        "tx_drop": value["tx_drop"],
                    })
                    if len(rows) % 20 == 0:
                        print(f"CLK records={len(rows)}, session={session}, "
                              f"seq={value['seq']}, PC elapsed={host_s:.1f}s")
    except serial.SerialException as exc:
        print(f"Cannot use {args.port}: {exc}. Close PlatformIO Serial Monitor.",
              file=sys.stderr)
        sys.exit(2)
    except KeyboardInterrupt:
        print("\nInterrupted; saving collected records.")
    with (outdir / "clock_samples.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    if not rows:
        print("No CLK frames received. Did you rebuild/upload firmware with CLK?",
              file=sys.stderr)
        sys.exit(3)
    try:
        save_report(outdir, rows)
    except ValueError as exc:
        print(f"Insufficient or invalid data: {exc}. Raw log/CSV were saved.",
              file=sys.stderr)
        sys.exit(4)


if __name__ == "__main__":
    main()
