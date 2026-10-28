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
# No fixed 40% qualification ceiling. Test up to overload, then derive
# recommendation from measured first failure and a 20% throughput headroom.
RECOMMEND_HEADROOM = 0.20
EXPECTED_CASES = 64
HEADER = ("session", "size_B", "target_Hz", "payload_Bps", "uart_pct",
          "generated", "generated_Hz", "schedule_achieved_pct",
          "accepted", "host_received", "host_receive_pct",
          "crc_global", "crc_case", "tx_rejected", "scheduler_skipped", "uart_drop",
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


def assess(events: list[dict], frames: list[dict], crc_global: int,
           crc_by_session: dict[int, int] | None = None) -> list[dict]:
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
        case_crc = crc_by_session.get(session, 0) if crc_by_session is not None else crc_global
        # SPP queue may take extra time to drain after the requested TX
        # interval, so use !B's requested active duration, not delayed !E.
        active_s = (begin.get("ms") / 1000.0) if begin and begin.get("ms") else None
        generated_hz = divide(generated, active_s) if active_s else None
        scheduled_pct = divide(100 * generated, hz * active_s) if active_s and hz and generated is not None else None
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
            "generated": generated,
            "generated_Hz": generated_hz,
            "schedule_achieved_pct": scheduled_pct,
            "accepted": accepted,
            "host_received": rec_count, "host_receive_pct": reception,
            "crc_global": crc_global, "crc_case": case_crc,
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
                # 8N1 UART cannot sustain traffic above its wire ceiling,
                # even if a 4s burst temporarily fits in the TX FIFO.
                size * hz > WIRE_BYTES_PER_S,
                rejected != 0,
                scheduled_pct is None or scheduled_pct < 98.0,
                skipped is None or skipped > max(1, generated * 0.01),
                end.get("drop") != 0,
                ctrl_gap != 0,
                row["ctrl_Hz"] is None or abs(row["ctrl_Hz"]-1000) > 10,
                row["imu_fresh_Hz"] is None or row["imu_fresh_Hz"] < SENSOR_MIN_HZ,
                row["ir_Hz"] is None or row["ir_Hz"] < SENSOR_MIN_HZ,
                row["ir_fresh_Hz"] is None or row["ir_fresh_Hz"] < SENSOR_MIN_HZ,
                ir_fail != 0,
                reception is None or reception < 99.0,
                case_crc != 0,
                row["tx_queue_peak"] is None or row["tx_queue_peak"] > 3072,
            )):
                row["verdict"] = "LIMIT_OR_FAIL"
            else:
                row["verdict"] = "CANDIDATE"
        results.append(row)
    return results


def refine_against_baseline(rows: list[dict]) -> None:
    baseline = next((x for x in rows if x["verdict"] == "BASELINE"), None)
    if not baseline:
        for row in rows:
            if row["verdict"] == "CANDIDATE":
                row["verdict"] = "NO_BASELINE"
        return
    for row in rows:
        if row["verdict"] != "CANDIDATE":
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
            if row["verdict"] == "CANDIDATE":
                row["verdict"] = "BASELINE_INSUFFICIENT"



def measured_recommendations(rows: list[dict], headroom: float = RECOMMEND_HEADROOM) -> dict:
    """Return a tested rate, not a guessed link bandwidth.

    Rates for each size are ordered low to high. Only a contiguous CLEAN
    prefix may establish a safe frontier; a later isolated pass does not
    reverse an earlier failure. The recommended tested rate must lie below
    80% of the first observed failing rate AND below 80% of physical 8N1
    bandwidth. No fixed 40% ceiling is used.
    """
    if not 0 < headroom < 1:
        raise ValueError("headroom must be between zero and one")
    by_size = {}
    for row in rows:
        size, hz = row.get("size_B"), row.get("target_Hz")
        if not isinstance(size, int) or not isinstance(hz, int) or not hz:
            continue
        by_size.setdefault(size, []).append(row)
    report = {}
    for size, group in sorted(by_size.items()):
        group.sort(key=lambda row: row["target_Hz"])
        first_failure = next((row for row in group
                              if row["verdict"] != "CANDIDATE"), None)
        clean_prefix = []
        for row in group:
            if row["verdict"] != "CANDIDATE":
                break
            clean_prefix.append(row)
        first_failure_hz = first_failure["target_Hz"] if first_failure else None
        measured_frontier_hz = clean_prefix[-1]["target_Hz"] if clean_prefix else None
        # For a frame size of 64 B, the ~11.5k B/s physical 8N1 limit
        # is at 180 Hz; the radio may fail MUCH sooner.
        physical_cap_hz = WIRE_BYTES_PER_S / size
        measured_bound_hz = min(physical_cap_hz,
                                first_failure_hz or physical_cap_hz)
        advised_max_hz = measured_bound_hz * (1.0 - headroom)
        eligible = [row for row in clean_prefix
                    if row["target_Hz"] <= advised_max_hz]
        chosen = eligible[-1] if eligible else None
        report[str(size)] = {
            "first_failure_Hz": first_failure_hz,
            "first_failure_reason": first_failure["verdict"] if first_failure else None,
            "clean_frontier_Hz": measured_frontier_hz,
            "recommended_tested_Hz": chosen["target_Hz"] if chosen else None,
            "recommended_Bps": chosen["payload_Bps"] if chosen else None,
            "recommended_uart_pct": chosen["uart_pct"] if chosen else None,
            "headroom_fraction": headroom,
            "qualifier": "PRELIMINARY_MOTOR_LOCKED" if chosen else "NO_RECOMMENDATION",
            "tests": len(group),
        }
    return report


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
    selection = measured_recommendations(rows)
    observed = {}
    for row in rows:
        if row["verdict"] == "CANDIDATE":
            k = str(row["size_B"])
            observed[k] = max(observed.get(k, 0), row["target_Hz"])
    recommended = {size: info["recommended_tested_Hz"]
                   for size, info in selection.items()
                   if info["recommended_tested_Hz"] is not None}
    summary = {
        "environment": "JDY-33 Classic SPP Windows outgoing COM",
        "port": port, "baud": BAUD, "uart_8n1_ceiling_Bps": WIRE_BYTES_PER_S,
        "elapsed_capture_s": round(duration_s, 3),
        "suite_done": complete, "motor": "LOCKED (no motor PWM applied)",
        "packet_crc_failures": decoder.crc_failures,
        "crc_unattributed_to_case": max(0, decoder.crc_failures -
                                       sum(int(row["crc_case"] or 0) for row in rows)),
        "unframed_bytes": decoder.discarded,
        "raw_received_bytes": decoder.raw_bytes,
        "rows": rows,
        "candidate_max_hz_by_packet_size": recommended,
        "observed_clean_max_hz_by_packet_size": observed,
        "recommendation_analysis": selection,
        "recommendation_basis": "First failing measured step per packet size, 20% tested-rate headroom; no 40% cap",
        "no_one_way_latency_claim": True,
        "qualification_limitations": [
            "SPP PC receive path only; no Windows->MCU commands",
            "No motor-on, maze exploration motion, vibration, electrical motor noise",
            "Host serial arrival gap is not one-way latency without clock synchronization",
            "All pass limits are engineering trial gates, not physical qualification until collected",
            "The upper sweep tests past 100% UART capacity as deliberate overload; cannot be sustained",
            "Recommendation is a tested rate below 80% of the first failing load or physical UART 8N1 limit",
            "A missing/corrupt marker or unattributed CRC error must be reviewed before using a recommendation",
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
    crc_by_session = {}
    active_session = None
    with (out / "raw.bin").open("wb") as raw, serial.Serial(
            port=port, baudrate=baud, timeout=0.15) as link:
        print("Connected COM. Wait for !Q,state=READY,ready=1 then press-and-release PA0.", flush=True)
        while time.monotonic() - start < seconds:
            data = link.read(link.in_waiting or 1)
            if not data:
                continue
            raw.write(data)
            old_crc = decoder.crc_failures
            decoder.feed(data, time.perf_counter_ns())
            if decoder.crc_failures > old_crc and active_session is not None:
                # Assign CRC failures to the currently streaming session.
                # Markers may share a read chunk; always keep global count.
                crc_by_session[active_session] = (crc_by_session.get(active_session, 0)
                                                  + decoder.crc_failures - old_crc)
            for event in decoder.events[seen_events:]:
                kind = event.get("type")
                if kind == "!B" and isinstance(event.get("s"), int):
                    active_session = event["s"]
                if kind == "!E" and event.get("s") == active_session:
                    active_session = None
                if kind in ("!Q", "!B", "!E", "!R"):
                    print(event, flush=True)
                if kind == "!Q" and event.get("state") == "DONE":
                    done = True
            seen_events = len(decoder.events)
            if done:
                break
    rows = assess(decoder.events, decoder.frames, decoder.crc_failures,
                  crc_by_session)
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
    print("\\n MEASURED RECOMMENDATIONS (20% reserve from first failure / wire ceiling):")
    for size, info in summary["recommendation_analysis"].items():
        print(f'  {size:>3} B: clean={info["clean_frontier_Hz"]} Hz '
              f'fail={info["first_failure_Hz"]} Hz '
              f'recommend={info["recommended_tested_Hz"]} Hz')
    print(f'\\nCapture suite complete: {done}; outputs: {out}', flush=True)
    # Overload cases are allowed to FAIL by design; incomplete markers are not.
    return 0 if (done and len(rows) == EXPECTED_CASES and
                 all(r["mcu_snapshot_ok"] for r in rows)) else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM16", help="Windows outgoing Classic SPP serial port")
    p.add_argument("--seconds", type=int, default=600,
                   help="Capture timeout incl. PA0/calibration wait; full 64-case suite takes about 6min")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    if args.baud != 115200:
        p.error("This firmware bench is fixed at UART2 115200; --baud must be 115200")
    if args.seconds < 400:
        p.error("--seconds must be >=400 for full expanded sweep and PA0 wait")
    out = args.output or (Path("results") / ("classic_spp_rate_" +
          datetime.now().strftime("%Y%m%d_%H%M%S")))
    try:
        return run(args.port, args.seconds, out, args.baud)
    except (OSError, RuntimeError) as exc:
        print("SPP RATE ERROR:", exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
