#!/usr/bin/env python3
"""Compare two STM32 clock_serial_test.py reports (baseline vs HSI trim).

Example:
    python tools/clock_trim_compare.py --baseline clock_logs/20261011_202835/clock_summary.txt --candidate clock_logs/NEW_SESSION/clock_summary.txt
"""
import argparse
from pathlib import Path


def load_report(path: Path):
    if not path.is_file():
        raise ValueError(f"Not a file: {path}")
    result = {}
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        key, sep, value = line.partition(":")
        if not sep:
            continue
        key = key.strip()
        try:
            result[key] = float(value.strip())
        except ValueError:
            pass
    must = ("pc_seconds_per_mcu_second", "effective_tim5_hz",
            "effective_tim11_hz", "seq_missing_count", "tx_drop_delta",
            "fit_records")
    for key in must:
        if key not in result:
            raise ValueError(f"Missing {key} in {path}")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    args = parser.parse_args()

    baseline = load_report(args.baseline)
    candidate = load_report(args.candidate)
    print("STM32 HSI trim comparison (PC-monotonic estimate; not scope-grade)")
    print("-" * 72)
    print(f"{'Metric':35s} {'Baseline':>15s} {'Candidate':>15s}")
    for field, unit in (
        ("pc_seconds_per_mcu_second", ""),
        ("effective_tim5_hz", " Hz"),
        ("effective_tim11_hz", " Hz"),
        ("mcu_clock_rate_error_pct", " %"),
        ("fit_residual_p95_ms", " ms"),
    ):
        a = baseline.get(field, float("nan"))
        b = candidate.get(field, float("nan"))
        print(f"{field:35s} {a:15.5f} {b:15.5f} {unit}")

    freq0, freq1 = baseline["effective_tim5_hz"], candidate["effective_tim5_hz"]
    delta = 100 * (freq1 / freq0 - 1)
    base_error = abs(1_000_000 - freq0)
    new_error = abs(1_000_000 - freq1)
    print(f"\nFrequency change vs baseline: {delta:+.4f}%")
    print(f"Error from nominal TIM5 1MHz: {base_error:.1f} Hz -> {new_error:.1f} Hz")
    print(f"Approximate SYSCLK: {freq0 * 0.0001:.3f} MHz -> {freq1 * 0.0001:.3f} MHz")

    if candidate["fit_records"] < 90:
        print("WARNING: only a short record; prefer >=120 seconds.")
    if candidate["seq_missing_count"] or candidate["tx_drop_delta"]:
        print("WARNING: missing CLK records/UART drops; repeat before tuning.")
    if freq1 > 1_000_000:
        print("SAFETY: inferred SYSCLK exceeds nominal STM32F411 100 MHz.")
        print("Do not continue increasing trim. Restore the default build.")
    elif new_error < base_error:
        print("RESULT: candidate is CLOSER to nominal time; repeat to confirm.")
    else:
        print("RESULT: candidate did not improve clock accuracy; stop and review.")


if __name__ == "__main__":
    main()
