#!/usr/bin/env python3
"""Compare matched motor-OFF MPU6050 policy A and policy B tests.

First analyze each run with imu_freshness_analyze.py --folder ..., then:
  python tools/imu_freshness_ab_compare.py --a clock_logs/A/imu_freshness_report.json --b clock_logs/B/imu_freshness_report.json

A/B/A repeats are encouraged if the difference is marginal.
This tool does not approve deploying either policy into motor firmware.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys


def load(path: pathlib.Path, mode: str) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("status") != "PASS_DIAGNOSTIC" or data.get("failures"):
        raise ValueError(f"{mode}: diagnostic failed; compare only valid runs.")
    m = data.get("metrics", {})
    if m.get("sampling_policy") != mode:
        raise ValueError(f"Expected mode {mode} in {path}, got {m.get('sampling_policy')!r}")
    if m.get("postcal_TIM11_ticks", 0) < 150000:
        raise ValueError(f"{mode}: too few post-calibration TIM11 ticks")
    if m.get("uart_tx_drop") or m.get("clk_seq_missing"):
        raise ValueError(f"{mode}: UART/CLK integrity failure")
    if abs(m.get("clock_error_pct", 99)) > 0.5:
        raise ValueError(f"{mode}: clock outside pre-agreed tolerance")
    return m


def compare(a: dict, b: dict) -> dict:
    fields = {
        "postcal_status_not_ready_pct": "DATA_RDY=0 per status poll (%)",
        "gyro_reused_tick_pct": "TIM11 repeated gyro sample (%)",
        "fresh_gap_gt2000_count": "Accepted gyro gaps over 2ms (count)",
        "fresh_gap_gt1500_count": "Accepted gyro gaps over 1.5ms (count)",
        "gyro_completion_age_gt1500_ticks": "Control sample age over 1.5ms (count)",
        "gyro_completion_age_max_us_observed": "Maximum completion age (us)",
        "age_p95_bin_us": "Age p95 histogram bound (us)",
        "age_p99_bin_us": "Age p99 histogram bound (us)",
        "postcal_fresh_gyro_reads": "Accepted fresh gyro reads (count)",
        "clock_error_pct": "Clock error (%)",
    }
    # Rates normalize for small differences in test duration.
    ar = a["postcal_TIM11_ticks"]
    br = b["postcal_TIM11_ticks"]
    rows = []
    for field, label in fields.items():
        left, right = a.get(field), b.get(field)
        rows.append({"key": field, "label": label, "a": left, "b": right,
                     "delta": None if left is None or right is None else round(right-left, 6)})
    extra = {}
    for name in ("gyro_started", "gyro_skipped", "retry_started", "retry_ready",
                 "retry_nr", "retry_expired", "retry_busy",
                 "estimated_i2c_transactions_per_1k_ticks"):
        left, right = a.get("ab", {}).get(name, 0), b.get("ab", {}).get(name, 0)
        extra[name] = {"a": left, "b": right}
    def rate(m, key):
        return 1000.0 * m[key] / m["postcal_TIM11_ticks"]
    derived = {
        "gap_gt2000_per_1000_ticks_a": round(rate(a, "fresh_gap_gt2000_count"), 5),
        "gap_gt2000_per_1000_ticks_b": round(rate(b, "fresh_gap_gt2000_count"), 5),
        "old_sample_age_gt1500_per_1000_ticks_a": round(rate(a, "gyro_completion_age_gt1500_ticks"), 5),
        "old_sample_age_gt1500_per_1000_ticks_b": round(rate(b, "gyro_completion_age_gt1500_ticks"), 5),
        "fresh_gyro_per_1000_ticks_a": round(rate(a, "postcal_fresh_gyro_reads"), 5),
        "fresh_gyro_per_1000_ticks_b": round(rate(b, "postcal_fresh_gyro_reads"), 5),
    }
    improve_reuse = b["gyro_reused_tick_pct"] < a["gyro_reused_tick_pct"]
    improve_old_age = (derived["old_sample_age_gt1500_per_1000_ticks_b"]
                       < derived["old_sample_age_gt1500_per_1000_ticks_a"])
    clock_diff = abs(a["clock_error_pct"] - b["clock_error_pct"])
    checks = {
        "same_nominal_clock": clock_diff <= 0.5,
        "no_more_fresh_gaps_gt2ms": derived["gap_gt2000_per_1000_ticks_b"]
                                   <= derived["gap_gt2000_per_1000_ticks_a"],
        "no_more_old_control_samples": derived["old_sample_age_gt1500_per_1000_ticks_b"]
                                       <= derived["old_sample_age_gt1500_per_1000_ticks_a"],
        "no_reuse_regression": b["gyro_reused_tick_pct"] <= a["gyro_reused_tick_pct"],
        "bounded_retry": b.get("ab", {}).get("retry_started", 0)
                         <= b.get("ab", {}).get("initial_nr", 0) + 3,
    }
    if not checks["same_nominal_clock"]:
        conclusion = "INCOMPARABLE: observed clock rates differ materially"
    elif not all(checks.values()):
        conclusion = "NOT PROVEN: B regressed at least one guardrail"
    elif improve_reuse or improve_old_age:
        conclusion = "PROMISING: B improved one or more freshness metrics, verify with A/B/A"
    else:
        conclusion = "NO CLEAR BENEFIT: B met guardrails but did not improve freshness"
    return {"conclusion": conclusion, "checks": checks, "metrics": rows,
            "transactions": extra, "normalized_rates": derived,
            "note": "No deployment decision: PC-UART rate is approximate, "
                    "and 1kHz TIM11 here runs diagnostic-only, motors OFF."}


def self_test():
    base = {
        "postcal_TIM11_ticks": 227000, "postcal_status_not_ready_pct": .50,
        "gyro_reused_tick_pct": .50, "fresh_gap_gt2000_count": 1,
        "fresh_gap_gt1500_count": 1358,
        "gyro_completion_age_gt1500_ticks": 1135,
        "gyro_completion_age_max_us_observed": 1772,
        "age_p95_bin_us": 1000, "age_p99_bin_us": 1500,
        "postcal_fresh_gyro_reads": 225865, "clock_error_pct": -.09,
        "ab": {"gyro_started": 227000, "gyro_skipped": 0,
               "retry_started": 0, "retry_ready": 0, "retry_nr": 0,
               "retry_expired": 0, "retry_busy": 0,
               "estimated_i2c_transactions_per_1k_ticks": 2000},
    }
    improved = dict(base, gyro_reused_tick_pct=.25,
                    gyro_completion_age_gt1500_ticks=600,
                    ab=dict(base["ab"], initial_nr=1131, retry_started=1131,
                            retry_ready=1130, gyro_skipped=1131))
    result = compare(base, improved)
    assert result["conclusion"].startswith("PROMISING"), result
    regressed = dict(improved, fresh_gap_gt2000_count=10)
    assert compare(base, regressed)["conclusion"].startswith("NOT PROVEN")
    print("SELF-TEST PASSED: metric normalization and A/B safety checks.")
    print("No board was touched.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--a", type=pathlib.Path, help="A report JSON")
    parser.add_argument("--b", type=pathlib.Path, help="B report JSON")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.a is None or args.b is None:
        parser.error("Supply --a and --b or --self-test")
    try:
        a = load(args.a, "A")
        b = load(args.b, "B")
        result = compare(a, b)
    except (ValueError, KeyError, OSError, TypeError) as exc:
        print("FAIL:", exc, file=sys.stderr)
        sys.exit(2)
    print("MPU6050 DATA_RDY A/B comparison (motor OFF)")
    print(result["conclusion"])
    print(f"{'Metric':46s} {'A':>12s} {'B':>12s} {'B-A':>12s}")
    for row in result["metrics"]:
        print(f"{row['label'][:46]:46s} "
              f"{str(row['a']):>12s} {str(row['b']):>12s} "
              f"{str(row['delta']):>12s}")
    print("Bounded retry and measurement guards:", result["checks"])
    print("Additional metrics:", result["transactions"], result["normalized_rates"])
    print("No production deployment is authorized based on one A/B pair.")


if __name__ == "__main__":
    main()
