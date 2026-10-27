#!/usr/bin/env python3
"""Guided motor-OFF IR qualification (static 40Hz + PA0 1210Hz burst).
Run from repository root. Never sends motion or UART control commands.
No physical tests or PASS claims are inferred without actual captures.
"""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path
import statistics as st
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
CHANNELS = ("L90", "L45", "L0", "R0", "R45", "R90")
FILTERS = ("raw", "median3", "ema10", "ema25", "hybrid")
WIRE_KEYS = ("d", "med", "e10", "e25", "hy")
CSV_HEADER = ("id", "seq", "us", "scan", "channel", *FILTERS)
STAGES = (
    ("A", "NO_WALL", "Remove reflective surfaces from all six beams; 30s static"),
    ("B", "SIDE_STATIC", "Two parallel white matte walls; measure L0 and R0 from each sensor; three independent placements per distance"),
    ("C", "SIDE_EDGE", "Move one side wall in/out manually, or move secured unpowered robot past a wall edge; >=10 rise and >=10 fall per side"),
    ("D", "SIDE_YAW", "Gently yaw secured unpowered robot +/-5 and +/-10 deg between walls; capture both rotation directions"),
    ("E", "SINGLE_WALL", "Leave only LEFT, then only RIGHT wall; repeat no-wall and side measurements to separate geometry from cross-talk"),
    ("F", "DIAGONAL_FRONT", "Place wall in L45/R45 viewing directions, then front L90/R90; calibrate geometry separately"),
    ("G", "ROBUSTNESS", "Repeat static/dynamic tests under changed illumination/material, with declared fixture uncertainty"),
    ("H", "LUT_HOLDOUT", "Use independent distance/placement holdout; do not fit LUT until coverage is complete"),
)

def save_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))

def session_root(name: str) -> Path:
    return ROOT / "results" / "ir_full_qualification" / name

def require_session(name: str) -> Path:
    p = session_root(name)
    if not (p / "session.json").is_file():
        raise ValueError("Unknown session. Run init first: " + str(p))
    return p

def stamp() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")

def plan() -> None:
    for code, title, instructions in STAGES:
        print(f"{code}: {title}: {instructions}")
    print("\nPA0: debounced press-release freezes the preceding ~0.85s of IR data.")
    print("PC13: slow pulse=READY; steady=UART DUMP; fast blink=scan or safety fault.")
    print("MOTORS ARE NEVER COMMANDED. Physically disconnect VM before flashing or capture.")
    print("Burst firmware: python -m platformio run -e ir_burst_diag_96m -t upload")
    print("Static firmware: python -m platformio run -e ir_filter_diag_96m -t upload")

def init(args: argparse.Namespace) -> None:
    p = session_root(args.session)
    if p.exists():
        raise ValueError("Session already exists; choose another name")
    p.mkdir(parents=True)
    save_json(p / "session.json", {
        "session": args.session, "created": dt.datetime.now().isoformat(),
        "status": "PREPARED_NOT_QUALIFIED", "vm_disconnected_operator_confirmed": False,
        "protocol": "ir_guided_v1", "channels": CHANNELS,
        "notes": "Motor-locked manual translation/yaw only; no powered trials.",
    })
    print("Created:", p)
    plan()

def preflight(args: argparse.Namespace) -> None:
    commands = [
        [sys.executable, "tests/regression/ir_filter_native_test.py"],
        [sys.executable, "tools/test_ir_filter_pipeline.py"],
    ]
    if args.build:
        commands.append([sys.executable, "-m", "platformio", "run",
                         "-e", "ir_burst_diag_96m"])
        commands.append([sys.executable, "-m", "platformio", "run",
                         "-e", "ir_filter_diag_96m"])
    for cmd in commands:
        print("RUN:", " ".join(cmd), flush=True)
        res = subprocess.run(cmd, cwd=ROOT, check=False)
        if res.returncode:
            raise RuntimeError(f"Preflight failed: {cmd} => {res.returncode}")
    print("Software preflight passed. This does NOT prove board operation.")

def read_line(raw: str) -> tuple[str, dict[str, str]] | None:
    parts = raw.strip().split(",")
    if not parts or parts[0] not in ("IB", "IBD"):
        return None
    if parts[0] == "IB":
        kind = parts[1] if len(parts) > 1 else ""
        pairs = parts[2:]
    else:
        kind = "DATA"
        pairs = parts[1:]
    fields = {}
    for p in pairs:
        key, sep, value = p.partition("=")
        if not sep or not key or key in fields:
            return None
        fields[key] = value
    return kind, fields

def parse_sample(fields: dict[str, str]) -> tuple[int, int, int, int, list[list[int]]]:
    ids = [int(fields[name]) for name in ("id", "seq", "us", "scan")]
    values = []
    for name in WIRE_KEYS:
        val = [int(z) for z in fields[name].split("|")]
        if len(val) != 6 or any(x < 0 or x > 4095 for x in val):
            raise ValueError("Invalid six channel ADC field " + name)
        values.append(val)
    return (*ids, values)

def parse_burst(lines: list[str]) -> tuple[list[dict], dict]:
    begin = None
    end = None
    samples = []
    malformed = 0
    for line in lines:
        parsed = read_line(line)
        if parsed is None:
            if line.startswith(("IB,", "IBD,")):
                malformed += 1
            continue
        typ, fields = parsed
        try:
            if typ == "BEGIN":
                if begin is not None: raise ValueError("Duplicate begin")
                begin = {k: int(fields[k]) for k in ("id", "count", "first_scan", "last_scan", "locked")}
            elif typ == "DATA":
                sid, seq, us, scan, arr = parse_sample(fields)
                samples.append((sid, seq, us, scan, arr))
            elif typ == "END":
                if end is not None: raise ValueError("Duplicate end")
                end = {k: int(fields[k]) for k in
                       ("id", "count", "uart_drop_delta", "dma_err", "dma_to", "ovr", "fail", "locked")}
        except (ValueError, KeyError):
            malformed += 1
    if not begin or not end:
        return [], {"valid": False, "reason": "MISSING_BEGIN_OR_END", "malformed": malformed}
    expected = begin["count"]
    good = (malformed == 0 and 128 <= expected <= 1024 and
            begin["id"] == end["id"] and begin["locked"] == 1 and end["locked"] == 1 and
            end["count"] == expected and len(samples) == expected and
            end["uart_drop_delta"] == 0 and all(end[k] == 0 for k in ("dma_err", "dma_to", "ovr", "fail")))
    rows = []
    for i, item in enumerate(samples):
        sid, seq, us, scan, array = item
        if sid != begin["id"] or seq != i:
            good = False
        if i and (scan <= samples[i-1][3] or (us - samples[i-1][2]) % (2**32) > 20000):
            good = False
        for ch_index, ch in enumerate(CHANNELS):
            rows.append({"id": sid, "seq": seq, "us": us, "scan": scan, "channel": ch,
                         **{f: array[j][ch_index] for j, f in enumerate(FILTERS)}})
    if samples and (begin["first_scan"] != samples[0][3] or
                    begin["last_scan"] != samples[-1][3]):
        good = False
    rate = None
    if len(samples) > 1:
        interval = (samples[-1][2] - samples[0][2]) % 2**32
        rate = round((len(samples) - 1) * 1e6 / interval, 2) if interval else None
    quality = {
        "valid": good, "reason": "PASS" if good else "SEQUENCE_OR_TRANSPORT_FAILURE",
        "frames": len(samples), "expected": expected,
        "scan_hz_from_trace": rate, "malformed": malformed, "begin": begin, "end": end,
        "timestamp_basis": "MCU TIM5 microseconds; trace before PA0 release",
        "absolute_wall_event_latency_supported": False,
    }
    return rows, quality

def med(vals: list[float]) -> float:
    return st.median(vals)

def transition(rows: list[dict], channel: str, direction: str) -> dict:
    by_ch = [r for r in rows if r["channel"] == channel]
    if len(by_ch) < 128:
        return {"status": "INCONCLUSIVE", "reason": "TOO_FEW_SAMPLES"}
    k = max(20, len(by_ch) // 10)
    start = med([float(r["raw"]) for r in by_ch[:k]])
    end = med([float(r["raw"]) for r in by_ch[-k:]])
    delta = end - start
    if abs(delta) < 20 or (direction == "rising" and delta <= 0) or (direction == "falling" and delta >= 0):
        return {"status": "INCONCLUSIVE", "reason": "EDGE_NOT_IN_BUFFER_OR_WRONG_DIRECTION",
                "delta_adc": round(delta, 2), "start_adc": start, "end_adc": end}
    threshold = start + 0.5 * delta
    def crossed(row: dict, name: str) -> bool:
        return row[name] >= threshold if delta > 0 else row[name] <= threshold
    found = {}
    for f in FILTERS:
        idx = next((i for i, r in enumerate(by_ch) if crossed(r, f)), None)
        found[f] = idx
    if any(v is None for v in found.values()):
        return {"status": "INCONCLUSIVE", "reason": "FILTER_MISSED_EDGE", "indices": found}
    def lag_us(f: str) -> int:
        a = int(by_ch[found[f]]["us"])
        b = int(by_ch[found["raw"]]["us"])
        d = (a - b) % (2**32)
        return d if d < 2**31 else d - 2**32
    return {
        "status": "COARSE_RELATIVE_EVENT_ACCEPTED", "direction": direction,
        "delta_adc": round(delta, 2), "start_adc": start, "end_adc": end,
        "relative_lag_us": {f: lag_us(f) for f in FILTERS if f != "raw"},
        "raw_crossing_us": by_ch[found["raw"]]["us"],
        "reference": "RAW_THRESHOLD_NOT_AN_EXTERNAL_EVENT",
    }

def summarize(rows: list[dict], channel: str, direction: str) -> dict:
    out = {}
    for ch in CHANNELS:
        part = [r for r in rows if r["channel"] == ch]
        if not part:
            continue
        stds = {f: st.stdev(float(r[f]) for r in part) for f in FILTERS}
        out[ch] = {
            "mean_raw_adc": round(st.fmean(float(r["raw"]) for r in part), 4),
            "std_adc": {k: round(v, 4) for k, v in stds.items()},
            "noise_reduction_pct": {k: round(100 * (1-v/stds["raw"]), 2)
                                    if stds["raw"] else None for k, v in stds.items()},
            "mean_bias_vs_raw_adc": {
                k: round(st.fmean(float(r[k]-r["raw"]) for r in part), 4)
                for k in FILTERS if k != "raw"},
        }
        if ch == channel and direction in ("rising", "falling"):
            out[ch]["edge"] = transition(rows, ch, direction)
    return out

def capture_burst(args: argparse.Namespace, dest: Path) -> None:
    try:
        import serial
    except ImportError as e:
        raise RuntimeError("Install pyserial: python -m pip install pyserial") from e
    lines = []
    started = None
    last_print = 0.0
    print(f"Opening {args.port} at 115200; PA0 press-release after the manual event.")
    print("Do the movement while VM disconnected; press PA0 immediately after the edge/turn.")
    with serial.Serial(args.port, 115200, timeout=0.3) as port:
        until = time.monotonic() + args.timeout
        while time.monotonic() < until:
            raw = port.readline()
            if not raw:
                continue
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            if time.monotonic() - last_print > 2:
                print("UART:", line[:130], flush=True)
                last_print = time.monotonic()
            parsed = read_line(line)
            if parsed and parsed[0] == "BEGIN" and started is None:
                started = time.monotonic()
                print("Captured PA0 trigger; receiving trace ...", flush=True)
            if started is not None:
                lines.append(line)
                if parsed and parsed[0] == "END":
                    break
                if time.monotonic() - started > 75:
                    break
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "raw_uart.log").write_text("\n".join(lines) + "\n", encoding="utf-8")
    rows, quality = parse_burst(lines)
    if rows:
        with (dest / "ir_burst_trace.csv").open("w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=CSV_HEADER)
            wr.writeheader()
            wr.writerows(rows)
    quality["metadata"] = vars(args)
    quality["channels"] = summarize(rows, args.channel, args.direction) if rows else {}
    save_json(dest / "summary.json", quality)
    print(f"Captured {quality['frames'] if 'frames' in quality else 0} samples. Valid={quality['valid']}")
    if not quality["valid"]:
        raise RuntimeError(f"Invalid trace: {quality['reason']}; raw retained in {dest}")
    ch = quality["channels"].get(args.channel, {})
    if "edge" in ch:
        print("Edge:", json.dumps(ch["edge"], ensure_ascii=False))
    print("Saved:", dest)

def capture_static(args: argparse.Namespace, dest: Path) -> None:
    cmd = [sys.executable, str(ROOT / "tools" / "ir_filter_pipeline.py"),
           "--port", args.port, "--seconds", str(args.seconds),
           "--scenario", args.case if args.case in
           ("static", "no-wall", "ambient", "crosstalk", "step") else "static",
           "--surface", args.surface, "--light", args.light,
           "--trial", str(args.trial), "--notes", args.notes,
           "--output", str(dest), "--plot"]
    if args.distance is not None:
        cmd += ["--distance-mm", str(args.distance)]
    print("RUN", " ".join(cmd), flush=True)
    result = subprocess.run(cmd, cwd=ROOT, check=False)
    if result.returncode:
        raise RuntimeError("Static capture failed; inspect existing raw/summary output")

def capture(args: argparse.Namespace) -> None:
    p = require_session(args.session)
    if not args.vm_off:
        raise ValueError("Physical VM disconnect confirmation required: --vm-off")
    if args.kind == "burst" and (not args.channel or args.channel not in CHANNELS):
        raise ValueError("--channel L0/R0/L45/R45/L90/R90 required for burst")
    if args.kind == "burst" and args.direction in ("rising", "falling"):
        print("Edge must be completed before PA0 button release and within ~0.85s buffer.")
    if args.distance is not None and (args.distance_left is not None or args.distance_right is not None):
        raise ValueError("Do not mix generic --distance with per-side distances")
    run_name = stamp() + "_" + args.kind + "_" + args.case.replace("/", "_")
    dest = p / "runs" / run_name
    (p / "runs").mkdir(exist_ok=True)
    meta = {"case": args.case, "kind": args.kind, "channel": args.channel,
            "direction": args.direction, "trial": args.trial, "distance_mm": args.distance,
            "distance_left_mm": args.distance_left, "distance_right_mm": args.distance_right,
            "light": args.light, "surface": args.surface, "notes": args.notes,
            "vm_disconnected_operator_confirmed": True, "created": dt.datetime.now().isoformat()}
    dest.mkdir(parents=True)
    save_json(dest / "setup.json", meta)
    print("\nSETUP:", json.dumps(meta, ensure_ascii=False, indent=2))
    if input("Robot secured, VM physically OFF, geometry measured? Type READY: ").strip() != "READY":
        print("Canceled; setup metadata retained.")
        return
    if args.kind == "burst":
        capture_burst(args, dest)
    else:
        capture_static(args, dest)

def status(args: argparse.Namespace) -> list[dict]:
    p = require_session(args.session)
    runs = []
    for folder in sorted((p / "runs").glob("*")) if (p / "runs").exists() else []:
        meta = folder / "setup.json"
        summary = folder / "summary.json"
        if not meta.exists():
            continue
        item = load_json(meta)
        item["folder"] = str(folder.relative_to(p))
        item["valid"] = load_json(summary).get("valid", load_json(summary).get("valid_capture", False)) if summary.exists() else False
        item["missing_summary"] = not summary.exists()
        runs.append(item)
    print(f"Session: {p.name}; runs {len(runs)}; valid {sum(bool(x['valid']) for x in runs)}")
    for item in runs:
        print(f" {'PASS' if item['valid'] else 'HOLD'} {item['folder']} "
              f"{item['channel']} {item['direction']} trial={item['trial']}")
    return runs

def report(args: argparse.Namespace) -> None:
    p = require_session(args.session)
    runs = status(args)
    groups = {}
    candidates = {}
    for r in runs:
        if not r["valid"]:
            continue
        s = load_json(p / r["folder"] / "summary.json")
        if r["kind"] != "burst":
            continue
        channel = r["channel"]
        edge = s.get("channels", {}).get(channel, {}).get("edge", {})
        if edge.get("status") != "COARSE_RELATIVE_EVENT_ACCEPTED":
            continue
        key = (channel, r["direction"], r["case"])
        groups.setdefault(key, []).append(edge)
        for f, val in edge["relative_lag_us"].items():
            candidates.setdefault((channel, f), []).append(val)
    table = []
    for (ch, direction, case), records in sorted(groups.items()):
        table.append({"channel": ch, "direction": direction, "case": case,
                      "valid_edges": len(records), "required_edges": 10,
                      "status": "PASS_COUNT_ONLY" if len(records) >= 10 else "INCONCLUSIVE"})
    ranking = []
    for (ch, f), delays in sorted(candidates.items()):
        ranking.append({"channel": ch, "filter": f, "n_relative_edges": len(delays),
                        "median_relative_lag_ms": round(st.median(delays) / 1000, 3),
                        "p95_relative_lag_ms": round(sorted(delays)[math.ceil(.95 * len(delays))-1]/1000, 3),
                        "qualified_for_wall_pid": False})
    report_data = {
        "status": "INCONCLUSIVE_NOT_PRODUCTION_QUALIFIED",
        "safety": "Manual-unpowered diagnostic only, based on operator confirmation",
        "valid_runs": sum(1 for r in runs if r["valid"]),
        "gates": {"G0": "OPERATOR_REPORTED_ONLY", "G1": "PER_RUN_ONLY",
                  "G2": "INCONCLUSIVE", "G3": "INCONCLUSIVE",
                  "G4": "INCONCLUSIVE_NO_INDEPENDENT_EVENT_TIMESTAMP",
                  "G5": "INCONCLUSIVE", "G6": "INCONCLUSIVE"},
        "edges": table, "relative_filter_lag": ranking,
        "decision": "Do not select production filter without distance coverage, "
                    "repeatability, independently labeled events and robustness.",
    }
    save_json(p / "acceptance.json", report_data)
    for name, fields, data in (
            ("edge_coverage.csv", ("channel", "direction", "case", "valid_edges", "required_edges", "status"), table),
            ("relative_filter_lag.csv", ("channel", "filter", "n_relative_edges",
                                         "median_relative_lag_ms", "p95_relative_lag_ms", "qualified_for_wall_pid"), ranking)):
        with (p / name).open("w", encoding="utf-8", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=fields)
            wr.writeheader()
            wr.writerows(data)
    print("Report:", p / "acceptance.json")
    print("Relative lags only; no automatic filter approval.")

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("plan")
    i = sub.add_parser("init")
    i.add_argument("--session", required=True)
    pf = sub.add_parser("preflight")
    pf.add_argument("--build", action="store_true")
    ca = sub.add_parser("capture")
    ca.add_argument("--session", required=True)
    ca.add_argument("--kind", choices=("static", "burst"), required=True)
    ca.add_argument("--port", required=True)
    ca.add_argument("--case", required=True)
    ca.add_argument("--vm-off", action="store_true",
                    help="Operator asserts VM motor power physically disconnected")
    ca.add_argument("--channel", choices=CHANNELS, default=None)
    ca.add_argument("--direction", choices=("rising", "falling", "yaw_left", "yaw_right", "static"), default="static")
    ca.add_argument("--trial", type=int, default=1)
    ca.add_argument("--distance", type=float, default=None)
    ca.add_argument("--distance-left", type=float, default=None)
    ca.add_argument("--distance-right", type=float, default=None)
    ca.add_argument("--seconds", type=float, default=30)
    ca.add_argument("--timeout", type=float, default=180)
    ca.add_argument("--surface", default="white_matte")
    ca.add_argument("--light", default="room")
    ca.add_argument("--notes", default="")
    sta = sub.add_parser("status")
    sta.add_argument("--session", required=True)
    rep = sub.add_parser("report")
    rep.add_argument("--session", required=True)
    args = ap.parse_args()
    try:
        if args.command == "plan": plan()
        elif args.command == "init": init(args)
        elif args.command == "preflight": preflight(args)
        elif args.command == "capture": capture(args)
        elif args.command == "status": status(args)
        elif args.command == "report": report(args)
    except (ValueError, RuntimeError, OSError) as e:
        print("ERROR:", e, file=sys.stderr)
        raise SystemExit(2) from e

if __name__ == "__main__":
    main()
