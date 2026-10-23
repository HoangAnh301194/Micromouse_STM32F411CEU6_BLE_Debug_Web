#!/usr/bin/env python3
"""Analyze real STM32 1-kHz shadow EMA measurements with conservative ranking.

Never treat the trailing 20-ms count window as independent speed ground truth.
The six stage-level SYS_AM frames are generated from 1-kHz measurements on MCU;
SYS_AB samples at 5 Hz (legacy 10 Hz) visualize only, not 1-ms lag.
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
import statistics

ALPHAS = (0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.30, 0.40, 0.60, 0.80)
LEGACY_ALPHAS = (0.05, 0.10, 0.20, 0.30, 0.40, 0.60, 0.80)
METRICS = {0: "steady_abs_error_vs_box20", 1: "first_500ms_abs_error_vs_box20",
           2: "steady_total_variation"}
SAMPLE_KEYS = ("a05", "a08", "a10", "a12", "a15", "a20", "a30",
               "a40", "a60", "a80", "b5", "b10", "b20")
LEGACY_SAMPLES = ("a05", "a10", "a20", "a30", "a40", "a60", "a80",
                  "b5", "b10", "b20")


def parse_frame(line):
    frame = line.partition("\t")[2] if "\t" in line else line
    parts = frame.strip().split(",")
    if not parts or parts[0] not in ("SYS_AM", "SYS_AB", "SYS_STEP"):
        return None
    record = {"type": parts[0]}
    for token in parts[1:]:
        name, eq, value = token.partition("=")
        if not eq or not name or name in record:
            return None
        try:
            record[name] = int(value)
        except ValueError:
            record[name] = value
    return record


def load_shadow(run_directory):
    stage_rows, samples, issues = [], [], []
    for phase in ("OFF", "ON"):
        path = Path(run_directory) / (phase.lower() + "_uart.log")
        if not path.is_file():
            raise ValueError("Missing raw UART evidence: " + str(path))
        stage = {"id": -1, "state": 0, "group": -1, "pwm_percent": 0}
        with path.open(encoding="utf-8", errors="replace") as stream:
            for line in stream:
                row = parse_frame(line)
                if row is None:
                    continue
                if row["type"] == "SYS_STEP":
                    if all(isinstance(row.get(key), int) for key in
                           ("id", "state", "group", "duty")):
                        stage = {"id": row["id"], "state": row["state"],
                                 "group": row["group"],
                                 "pwm_percent": row["duty"] // 10}
                elif row["type"] == "SYS_AB":
                    if (row.get("w") in (0, 1) and
                        all(isinstance(row.get(k), int)
                            for k in ("ms", *LEGACY_SAMPLES))):
                        samples.append({"phase": phase, **stage,
                                        "wheel": "L" if row["w"] == 0 else "R",
                                        "ms": row["ms"],
                                        **{key: row.get(key, "") for key in SAMPLE_KEYS}})
                elif row["type"] == "SYS_AM":
                    new_keys = tuple("x" + str(i) for i in range(len(ALPHAS)))
                    old_keys = tuple("x" + str(i) for i in range(len(LEGACY_ALPHAS)))
                    has_new = any(k in row for k in new_keys[len(old_keys):])
                    keys = new_keys if has_new else old_keys
                    active = ALPHAS if has_new else LEGACY_ALPHAS
                    if (not all(isinstance(row.get(k), int)
                                for k in ("id", "state", "w", "m", "n", *keys))
                        or row["w"] not in (0, 1)
                        or row["m"] not in METRICS or not 0 <= row["id"] < 33
                        or row["state"] not in (1, 2)
                        or row["n"] < 0
                        or any(row[k] < 0 for k in keys)):
                        issues.append({"phase": phase, "issue": "malformed_SYS_AM"})
                        continue
                    stage_rows.append({
                        "phase": phase, "stage": row["id"],
                        "group": row["id"] // 11,
                        "pwm_percent": (row["id"] % 11) * 10,
                        "state": row["state"],
                        "wheel": "L" if row["w"] == 0 else "R",
                        "metric": METRICS[row["m"]], "n": row["n"],
                        "alpha_count": len(active),
                        **{"alpha_" + str(alpha).replace(".", "_"):
                           round(row[key] / row["n"], 5)
                           if row["n"] > 0 else None
                           for alpha, key in zip(active, keys)}
                    })
    return stage_rows, samples, issues

def candidate_rows(stage_rows, phase="ON"):
    # Only active motor and middle PWM steps; 90-100% runs are deliberately
    # shorter and separately reviewed rather than optimizing against them.
    return [
        row for row in stage_rows
        if row["phase"] == phase and row["state"] in (1, 2)
        and (row["state"] == 1 or
             row["metric"] == METRICS[1])  # coast onset only
        and 20 <= row["pwm_percent"] <= 80
        and row["n"] >= 100
        and (row["group"] == 2 or
             (row["group"] == 0 and row["wheel"] == "L") or
             (row["group"] == 1 and row["wheel"] == "R"))
    ]


def summarize(stage_rows, samples, issues):
    seen = set()
    duplicated = []
    for row in stage_rows:
        key = (row["phase"], row["stage"], row["state"],
               row["wheel"], row["metric"])
        if key in seen:
            duplicated.append(key)
        seen.add(key)
    # Includes both wheels even when only one is active; 33 x 2 intervals
    # x two wheels x three metrics, per OFF or ON phase.
    expected = {
        (phase, stage, state, wheel, metric)
        for phase in ("OFF", "ON") for stage in range(33)
        for state in (1, 2) for wheel in ("L", "R")
        for metric in METRICS.values()
    }
    missing = sorted(expected - seen)
    counts = set(row["alpha_count"] for row in stage_rows)
    if len(counts) > 1:
        issues.append({"issue": "mixed_shadow_bank_versions", "counts": sorted(counts)})
    active_alphas = (ALPHAS if counts == {len(ALPHAS)} else
                     LEGACY_ALPHAS if counts == {len(LEGACY_ALPHAS)} else ())
    points = candidate_rows(stage_rows)
    per_metric = {}
    for metric in METRICS.values():
        basis = [row for row in points if row["metric"] == metric]
        totals = {}
        for alpha in active_alphas:
            name = "alpha_" + str(alpha).replace(".", "_")
            used = [row for row in basis
                    if row[name] is not None and row["n"] > 0]
            n = sum(row["n"] for row in used)
            totals[name] = (round(sum(row[name] * row["n"] for row in used) / n, 5)
                            if n else None)
        per_metric[metric] = {"intervals": len(basis), "weighted_ticks":
                              sum(row["n"] for row in basis), "values": totals}

    # Multicriterion proxy ranking, not physical-accuracy validation.
    base_name = "alpha_0_3"
    weights = {METRICS[0]: 0.50, METRICS[1]: 0.35, METRICS[2]: 0.15}
    ranking = []
    for alpha in active_alphas:
        name = "alpha_" + str(alpha).replace(".", "_")
        terms = {}
        for metric, weight in weights.items():
            x = per_metric[metric]["values"][name]
            baseline = per_metric[metric]["values"][base_name]
            if x is None or baseline is None or baseline <= 0:
                terms = {}
                break
            terms[metric] = x / baseline
        if terms:
            ranking.append({"alpha": alpha,
                            "heuristic_score_vs_alpha_0_30": round(
                                sum(weights[k] * terms[k] for k in weights), 5),
                            "relative_metrics": {
                                key: round(val, 5) for key, val in terms.items()}})
    ranking.sort(key=lambda x: x["heuristic_score_vs_alpha_0_30"])
    def region_summary(region):
        out = {}
        for metric in METRICS.values():
            target = [row for row in region if row["metric"] == metric and row["n"] > 0]
            n = sum(row["n"] for row in target)
            out[metric] = {
                "intervals": len(target), "ticks": n,
                **{str(alpha): round(
                    sum(row["alpha_" + str(alpha).replace(".", "_")] * row["n"]
                        for row in target) / n, 5) if n else None
                   for alpha in active_alphas}
            }
        return out
    by_wheel = {wheel: region_summary([row for row in points
                                      if row["wheel"] == wheel])
                for wheel in ("L", "R")}
    by_pwm = {
        str(pwm): {
            wheel: region_summary([row for row in points if
                                   row["pwm_percent"] == pwm and
                                   row["wheel"] == wheel])
            for wheel in ("L", "R")
        } for pwm in range(20, 81, 10)
    }
    def wheel_ranking(wheel):
        summary = by_wheel[wheel]
        if not active_alphas or 0.30 not in active_alphas:
            return []
        base = {key: summary[key].get("0.3") for key in weights}
        if any(value is None or value <= 0 for value in base.values()):
            return []
        rows = []
        for alpha in active_alphas:
            terms = [summary[key][str(alpha)] / base[key] for key in weights]
            if any(v is None for v in terms):
                continue
            rows.append({"alpha": alpha,
                         "score": round(sum(t * v for t, v in
                                            zip(weights.values(), terms)), 5)})
        return sorted(rows, key=lambda x: x["score"])
    rank_by_wheel = {wheel: wheel_ranking(wheel) for wheel in ("L", "R")}
    passes = not missing and not duplicated and not issues and len(samples) >= 500
    return {
        "status": "COMPLETE_PROVISIONAL" if passes else "INSUFFICIENT_SHADOW_DATA",
        "expected_metric_frames": len(expected),
        "observed_metric_frames": len(stage_rows),
        "missing_metric_frames_count": len(missing),
        "missing_first_12": [list(x) for x in missing[:12]],
        "duplicated_frames_count": len(duplicated),
        "issues": issues[:20],
        "sample_frames": len(samples),
        "candidate_alphas": list(active_alphas),
        "shadow_bank_version": ("ten_alpha" if len(active_alphas) == 10 else
                                "legacy_seven_alpha" if len(active_alphas) == 7 else "unknown"),
        "reference": "Trailing 20-ms integrated encoder counts, not external truth.",
        "sampling": "Metrics accumulated at 1 kHz; SYS_AB visualization 5 Hz (legacy 10 Hz).",
        "optimization_scope": "ON active wheels PWM 20-80%; steady noise/tracking from DRIVE only, early transient from DRIVE and COAST; 90-100% reserved for separate risk analysis.",
        "metrics": per_metric,
        "by_wheel": by_wheel,
        "by_pwm": by_pwm,
        "ranking_by_wheel": rank_by_wheel,
        "ranking": ranking,
        "provisional_top_two": [row["alpha"] for row in ranking[:2]],
        "decision": ("Shortlist only; do not change production alpha without "
                     "repeatability and independent velocity/plant assessment."),
        "caveats": [
            "A lower score means closer to the SAME ENCODER's trailing 20ms count proxy; it does not prove true RPM accuracy.",
            "The speed response and inertia of the physical motor affect step and coast error.",
            "Per-interval first 500ms errors are cumulative signed-magnitude errors, not time-to-settle.",
            "High-PWM runs are short. Independent rpm and mechanical load remain unqualified.",
            "UART timing and ISR overhead must be compared to historical baseline.",
        ],
    }


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def analyze(run_directory, report_directory):
    report_directory = Path(report_directory)
    rows, samples, issues = load_shadow(run_directory)
    report = summarize(rows, samples, issues)
    write_csv(report_directory / "encoder_alpha_stage_metrics.csv", rows)
    write_csv(report_directory / "encoder_alpha_samples_10hz.csv", samples)
    (report_directory / "encoder_alpha_ab.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if not args.run_dir or not args.output_dir:
        parser.error("--run-dir and --output-dir required")
    report = analyze(args.run_dir, args.output_dir)
    print("Shadow A/B status:", report["status"])
    print("Provisional alpha shortlist:", report["provisional_top_two"])
    return 0 if report["status"] == "COMPLETE_PROVISIONAL" else 2


if __name__ == "__main__":
    raise SystemExit(main())
