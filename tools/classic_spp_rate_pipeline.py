#!/usr/bin/env python3
"""Windows COM / JDY-33 Classic SPP throughput benchmark (RX-only).

Firmware: classic_spp_rate_diag_96m (motor locked, physical PA0 start).
Reuses the existing mixed binary + ASCII CRC16 decoder from ble_full_pipeline.
Does not send bytes, start motors, tune PID, or flash MCU.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import statistics
import sys
import time

from ble_full_pipeline import Decoder
from classic_spp_ports import classify_port

BAUD = 115200
WIRE_BYTES_PER_S = BAUD / 10.0  # UART 8N1: start + 8 data + stop
SENSOR_MIN_HZ = 980.0
MAX_UTILIZATION = 0.40
HEADER = ("session", "size_B", "target_Hz", "payload_Bps", "uart_pct",
          "generated", "accepted", "host_received", "host_receive_pct",
          "crc_global", "tx_rejected", "scheduler_skipped", "uart_drop",
          "ctrl_Hz", "imu_fresh_Hz", "ir_Hz", "ir_fresh_Hz", "ir_fail",
          "control_late", "control_gap", "irq_gt48_per_k", "tx_queue_peak",
          "elapsed_s", "verdict")


def delta(before: dict, after: dict, key: str):
    a, b = before.get(key), after.get(key)
    if not isinstance(a, int) or not isinstance(b, int):
        return None
    # Hardware diagnostic counters are unsigned 32-bit.
    return (b - a) & 0xFFFFFFFF


def divide(numerator, denominator):
    if numerator is None or not denominator:
        return None
    return round(numerator / denominator, 4)


def percent(values, q: int):
    if not values:
        return None
    v = sorted(values)
    return round(v[max(0, min(len(v) - 1, (len(v) * q + 99) // 100 - 1))], 3)


def assess(events: list[dict], frames: list[dict], crc_global: int) -> list[dict]:
    """Score only sessions with matching START/END markers and MCU snapshots.

    Receive timestamps are PC arrival time, not MCU->PC one-way latency.
    """
    groups: dict[int, dict] = {}
    for event in events:
        session = event.get("s")
        if not isinstance(session, int):
            continue
        row = groups.setdefault(session, {"markers": {}, "hist": {}})
        kind = event.get("type")
        if kind in ("!B", "!E"):
            row["markers"][kind] = event
        elif kind == "!R" and event.get("p") in ("B", "E"):
            row["markers"]["!R" + event["p"]] = event
        elif kind == "!T" and event.get("p") in ("B", "E"):
            row["hist"][event["p"]] = event
    by_session: dict[int, list[dict]] = {}
    for f in frames:
        if f.get("kind") == "D":
            by_session.setdefault(f["session"], []).append(f)
    results = []
    for session in sorted(groups):
        group = groups[session]["markers"]
        begin, end = group.get("!B"), group.get("!E")
        before, after = group.get("!RB"), group.get("!RE")
        if not begin and not end:
            continue
        hz = begin.get("hz") if begin else None
        size = begin.get("size") if begin else None
        elapsed_ms = delta(before, after, "ms") if before and after else None
        seconds = elapsed_ms / 1000 if elapsed_ms else None
        if seconds is None and end:
            seconds = end.get("ms", 0) / 1000
        if not seconds:
            seconds = None
        rec = by_session.get(session, [])
        gaps = [
            (rec[i]["host_monotonic_ns"] - rec[i-1]["host_monotonic_ns"]) / 1e6
            for i in range(1, len(rec))
        ]
        generated = end.get("gen") if end else None
        accepted = end.get("ok") if end else None
        rejected = end.get("reject") if end else None
        skipped = end.get("skip") if end else None
        hist = groups[session]["hist"]
        h0, h1 = hist.get("B"), hist.get("E")
        count = delta(before, after, "tick") if before and after else None
        ir_fail = delta(before, after, "irfail") if before and after else None
        late = delta(before, after, "late") if before and after else None
        ctrl_gap = delta(before, after, "gap") if before and after else None
        irq_high = delta(h0, h1, "b7") if h0 and h1 else None
        lossless = (generated is not None and accepted is not None and
                    generated == accepted + (rejected or 0))
        rec_count = len({f["seq"] for f in rec})
        reception = divide(rec_count * 100, accepted)
        # No PASS from host records without two MCU snapshots, even if COM worked.
        complete = (begin is not None and end is not None and
                    before is not None and after is not None and
                    h0 is not None and h1 is not None and
                    lossless and count is not None and seconds is not None and
                    isinstance(hz, int) and isinstance(size, int))
        utilization = round(100 * size * hz / WIRE_BYTES_PER_S, 2) if isinstance(hz, int) and isinstance(size, int) else None
        row = {
            "session": session, "size_B": size, "target_Hz": hz,
            "payload_Bps": size * hz if size is not None and hz is not None else None,
            "uart_pct": utilization,
            "generated": generated, "accepted": accepted,
            "host_received": rec_count, "host_receive_pct": reception,
            "crc_global": crc_global,
            "tx_rejected": rejected,
            "scheduler_skipped": skipped,
            "uart_drop": end.get("drop") if end else None,
            "ctrl_Hz": divide(count, seconds),
            "imu_fresh_Hz": divide(delta(before, after, "gyro"), seconds) if before and after else None,
            "ir_Hz": divide(delta(before, after, "ir"), seconds) if before and after else None,
            "ir_fresh_Hz": divide(delta(before, after, "irfresh"), seconds) if before and after else None,
            "ir_fail": ir_fail,
            "control_late": late,
            "control_gap": ctrl_gap,
            "irq_gt48_per_k": divide(irq_high * 1000, count) if irq_high is not None and count else None,
            "tx_queue_peak": end.get("peak") if end else None,
            "elapsed_s": seconds,
            "host_gap_p50_ms": percent(gaps, 50),
            "host_gap_p95_ms": percent(gaps, 95),
            "host_gap_p99_ms": percent(gaps, 99),
            "mcu_snapshot_ok": bool(complete),
            "verdict": "INCOMPLETE",
        }
        if complete:
            if hz == 0:
                row["verdict"] = "BASELINE"
            elif any((
                rejected != 0,
                skipped is None or skipped > max(1, generated * 0.01),
                end.get("drop") != 0,
                ctrl_gap != 0,
                row["ctrl_Hz"] is None or abs(row["ctrl_Hz"]-1000) > 10,
                row["imu_fresh_Hz"] is None or row["imu_fresh_Hz"] < SENSOR_MIN_HZ,
                row["ir_Hz"] is None or row["ir_Hz"] < SENSOR_MIN_HZ,
                row["ir_fresh_Hz"] is None or row["ir_fresh_Hz"] < SENSOR_MIN_HZ,
                ir_fail != 0,
                reception is None or reception < 99.0,
                crc_global != 0,
                row["tx_queue_peak"] is None or row["tx_queue_peak"] > 3072,
            )):
                row["verdict"] = "LIMIT_OR_FAIL"
            else:
                row["verdict"] = ("STRESS_CLEAN_ABOVE_BUDGET"
                                  if utilization > MAX_UTILIZATION * 100 else "CANDIDATE")
        results.append(row)
    return results


def refine_against_baseline(rows: list[dict]) -> None:
    baseline = next((x for x in rows if x["verdict"] == "BASELINE"), None)
    if not baseline:
        for row in rows:
            if row["verdict"] in ("CANDIDATE", "STRESS_CLEAN_ABOVE_BUDGET"):
                row["verdict"] = "NO_BASELINE"
        return
    for row in rows:
        if row["verdict"] not in ("CANDIDATE", "STRESS_CLEAN_ABOVE_BUDGET"):
            continue
        for key in ("imu_fresh_Hz", "ir_Hz", "ir_fresh_Hz"):
            value, base = row.get(key), baseline.get(key)
            if value is None or base is None or value < base * 0.98:
                row["verdict"] = "SENSOR_DEGRADED"
                break
        if row["verdict"] != "CANDIDATE":
            continue
        # Compare phase-matched 1kHz IRQ counters with the quiet stage.
        # Keep strictly <1 extra late tick per 1000 ticks; this is
        # an engineering acceptance screen, not a hard real-time proof.
        baseline_late_per_k = ((baseline.get("control_late") or 0) /
                               max(1.0, (baseline.get("elapsed_s") or 4) * 1.0))
        allowed_late = max(2, int(baseline_late_per_k * (row.get("elapsed_s") or 4)) + 2)
        base_irq = baseline.get("irq_gt48_per_k")
        test_irq = row.get("irq_gt48_per_k")
        if (row.get("control_late") or 0) > allowed_late or (
            base_irq is not None and test_irq is not None and
            test_irq > base_irq + 1.0
        ):
            row["verdict"] = "CONTROL_JITTER"
    if any(baseline.get(k) is None or baseline[k] < SENSOR_MIN_HZ
           for k in ("ctrl_Hz", "imu_fresh_Hz", "ir_Hz", "ir_fresh_Hz")):
        baseline["verdict"] = "BASELINE_INSUFFICIENT"
        for row in rows:
            if row["verdict"] in ("CANDIDATE", "STRESS_CLEAN_ABOVE_BUDGET"):
                row["verdict"] = "BASELINE_INSUFFICIENT"


def save(out: Path, rows: list[dict], events: list[dict], frames: list[dict],
         decoder: Decoder, duration_s: float, complete: bool, port: str):
    out.mkdir(parents=True, exist_ok=True)
    with (out / "cases.csv").open("w", newline="", encoding="utf-8") as f:
        columns = list(HEADER) + ["host_gap_p50_ms", "host_gap_p95_ms",
                                  "host_gap_p99_ms", "mcu_snapshot_ok"]
        w = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    with (out / "frames.csv").open("w", newline="", encoding="utf-8") as f:
        columns = ["host_monotonic_ns", "session", "seq", "mcu_us", "size", "kind", "payload_hex"]
        w = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        w.writeheader()
        w.writerows(frames)
    with (out / "events.json").open("w", encoding="utf-8") as f:
        json.dump(events, f, indent=2)
    recommended = {}
    observed = {}
    for row in rows:
        if row["verdict"] in ("CANDIDATE", "STRESS_CLEAN_ABOVE_BUDGET"):
            k = str(row["size_B"])
            observed[k] = max(observed.get(k, 0), row["target_Hz"])
            if row["verdict"] == "CANDIDATE":
                recommended[k] = max(recommended.get(k, 0), row["target_Hz"])
    summary = {
        "environment": "JDY-33 Classic SPP Windows outgoing COM",
        "port": port, "baud": BAUD, "uart_8n1_ceiling_Bps": WIRE_BYTES_PER_S,
        "elapsed_capture_s": round(duration_s, 3),
        "suite_done": complete, "motor": "LOCKED (no motor PWM applied)",
        "packet_crc_failures": decoder.crc_failures,
        "unframed_bytes": decoder.discarded,
        "raw_received_bytes": decoder.raw_bytes,
        "rows": rows,
        "candidate_max_hz_by_packet_size": recommended,
        "observed_clean_max_hz_by_packet_size": observed,
        "no_one_way_latency_claim": True,
        "qualification_limitations": [
            "SPP PC receive path only; no Windows->MCU commands",
            "No motor-on, maze exploration motion, vibration, electrical motor noise",
            "Host serial arrival gap is not one-way latency without clock synchronization",
            "All pass limits are engineering trial gates, not physical qualification until collected",
            "Rates >40pct of 115200 8N1 link may be STRESS_CLEAN_ABOVE_BUDGET but are not recommended",
            "A baseline/control verdict is a 4-second test window; repeat multiple trials and validate live navigation"
        ]
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def run(port: str, seconds: int, out: Path, baud: int):
    import serial
    from serial.tools import list_ports
    candidate = next((p for p in list_ports.comports()
                      if p.device.upper() == port.upper()), None)
    if candidate is None:
        raise RuntimeError("COM not listed: verify outgoing JDY-33 COM Ports")
    direction, remote = classify_port(candidate.hwid or "")
    if direction == "incoming":
        raise RuntimeError("Refusing incoming Bluetooth COM; select Outgoing JDY-33 port")
    if direction in ("unknown", "other"):
        print("WARNING: cannot verify COM direction; check bthprops.cpl", flush=True)
    out.mkdir(parents=True, exist_ok=True)
    decoder = Decoder()
    start = time.monotonic()
    done = False
    seen_events = 0
    with (out / "raw.bin").open("wb") as raw, serial.Serial(
            port=port, baudrate=baud, timeout=0.15) as link:
        print("Connected COM. Wait for !Q,state=READY,ready=1 then press-and-release PA0.", flush=True)
        while time.monotonic() - start < seconds:
            data = link.read(link.in_waiting or 1)
            if not data:
                continue
            raw.write(data)
            decoder.feed(data, time.perf_counter_ns())
            for event in decoder.events[seen_events:]:
                kind = event.get("type")
                if kind in ("!Q", "!B", "!E", "!R"):
                    print(event, flush=True)
                if kind == "!Q" and event.get("state") == "DONE":
                    done = True
            seen_events = len(decoder.events)
            if done:
                break
    rows = assess(decoder.events, decoder.frames, decoder.crc_failures)
    refine_against_baseline(rows)
    summary = save(out, rows, decoder.events, decoder.frames,
                   decoder, time.monotonic() - start, done, port)
    print("\n SIZE   HZ  UART%    MCUHz   IMUHz    IRHz  RX/OK  verdict")
    for row in rows:
        print(f'{str(row["size_B"]):>5} {str(row["target_Hz"]):>4} '
              f'{str(row["uart_pct"]):>6} '
              f'{str(row["ctrl_Hz"]):>8} {str(row["imu_fresh_Hz"]):>7} '
              f'{str(row["ir_Hz"]):>7} '
              f'{row["host_received"]}/{row["accepted"]}  {row["verdict"]}')
    print(f'\nCapture suite complete: {done}; outputs: {out}', flush=True)
    return 0 if done and rows and all(r["mcu_snapshot_ok"] for r in rows) else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM16", help="Windows outgoing Classic SPP serial port")
    p.add_argument("--seconds", type=int, default=180, help="Capture timeout incl. PA0 wait")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    if args.baud != 115200:
        p.error("This firmware bench is fixed at UART2 115200; --baud must be 115200")
    if args.seconds < 30:
        p.error("--seconds must be >=30 to include sweep")
    out = args.output or (Path("results") / ("classic_spp_rate_" +
          datetime.now().strftime("%Y%m%d_%H%M%S")))
    try:
        return run(args.port, args.seconds, out, args.baud)
    except (OSError, RuntimeError) as exc:
        print("SPP RATE ERROR:", exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
