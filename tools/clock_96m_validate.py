#!/usr/bin/env python3
"""Validate an isolated 96 MHz nominal PLL / HSITRIM=25 Micromouse run.

Usage:
    python tools/clock_96m_validate.py --self-test
    python tools/clock_96m_validate.py --folder clock_logs/20261011_XXXXXX

Uses existing TIM5-vs-PC regression plus firmware boot/IMU/control telemetry.
PC perf_counter is an approximate independent clock, not a calibrated counter.
"""
from __future__ import annotations

import argparse
import pathlib
import sys

from clock_serial_test import analyze, read_csv

NOMINAL_SYSCLK_HZ = 96_000_000
EXPECTED_TRIM = 25
EXPECTED_PLLN = 192
MAX_SAFE_MHZ = 98.0   # practical review threshold; hardware max is 100 MHz
MAX_ERROR_PCT = 0.5
MIN_FIT_RECORDS = 150
MIN_HOST_SPAN_S = 160.0
MAX_P95_RESIDUAL_MS = 30.0


def payloads(raw_log: str) -> list[str]:
    return [line.split("\t", 1)[-1].strip()
            for line in raw_log.splitlines() if line.strip()]


def fields(message: str) -> dict[str, str]:
    """Firmware mixes comma-separated and space-separated key=value tokens."""
    result = {}
    # CLOCK_PROFILE is comma-separated; CLOCK_TRIM/CAL_DONE/IMU combine
    # commas with spaces. Split BOTH, or boot and sensor checks silently fail.
    import re
    for item in re.split(r"[,\\s]+", message):
        key, sep, value = item.partition("=")
        if sep:
            result[key.strip()] = value.strip()
    return result


def check(report: dict, messages: list[str]) -> tuple[list[str], list[str], dict]:
    failures = []
    warnings = []

    def fail_if(condition: bool, why: str):
        if condition:
            failures.append(why)

    def select(prefix: str):
        return [m for m in messages if m.startswith(prefix)]

    def require_one(prefix: str):
        matches = select(prefix)
        if not matches:
            failures.append(f"Missing telemetry: {prefix}")
            return {}
        return fields(matches[-1])

    boot = select("[BOOT] Isolated MPU6050/I2C test (motors disabled)")
    fail_if(not boot, "Cannot verify motor-off diagnostic firmware.")
    fail_if(bool(select("BOOT_FAIL")), "Firmware reported BOOT_FAIL.")

    trim = require_one("CLOCK_TRIM,")
    profile = require_one("CLOCK_PROFILE,")
    diag = require_one("CLOCK_DIAG,")
    fail_if(trim.get("mode") != "override" or trim.get("active") != "25"
            or trim.get("requested") != "25",
            "HSITRIM not confirmed as 25 at boot.")
    for key, expected in (
        ("mode", "validate96"), ("trim", "25"),
        ("plln", "192"), ("nominal_hz", "96000000"),
        ("motor_off", "1"),
    ):
        fail_if(profile.get(key) != expected,
                f"CLOCK_PROFILE {key}: {profile.get(key)!r} != {expected!r}")
    fail_if(diag.get("fault") != "0", "RCC clock fault reported at boot.")

    clk = select("CLK,")
    fail_if(not clk, "No raw CLK records to verify nominal SYSCLK.")
    for message in clk:
        data = fields(message)
        if any(data.get(k) != expected for k, expected in (
            ("trim", "25"), ("fault", "0"),
            ("sysclk_nominal", "96000000"),
        )):
            failures.append("CLK readback mismatch (trim/fault/nominal clock).")
            break

    hz = float(report.get("effective_tim5_hz", float("nan")))
    mhz = hz * NOMINAL_SYSCLK_HZ / 1e12
    error = float(report.get("mcu_clock_rate_error_pct", float("nan")))
    ratio = float(report.get("pc_seconds_per_mcu_second", float("nan")))
    robust60 = report.get("robust_60s_pc_per_mcu")
    fail_if(report.get("fit_records", 0) < MIN_FIT_RECORDS,
            "Too few regression samples; use a 240s capture.")
    fail_if(report.get("host_span_s", 0) < MIN_HOST_SPAN_S,
            "Capture is too short for long-run validation.")
    fail_if(report.get("seq_missing_count", 1) != 0,
            "Missing CLK sequence(s).")
    fail_if(report.get("tx_drop_delta", 1) != 0,
            "Firmware UART queue dropped a message.")
    fail_if(not (0.0 < mhz < MAX_SAFE_MHZ),
            f"Estimated SYSCLK {mhz:.4f}MHz is outside safe validation window "
            f"(0, {MAX_SAFE_MHZ:g})MHz.")
    fail_if(not (-MAX_ERROR_PCT <= error <= MAX_ERROR_PCT),
            f"Clock error {error:+.4f}% outside +/-{MAX_ERROR_PCT}% target.")
    fail_if(report.get("fit_residual_p95_ms", float("inf")) > MAX_P95_RESIDUAL_MS,
            "High host serial receipt jitter; repeat under lower PC load.")
    fail_if(robust60 is None or abs(robust60 - ratio) > 0.002,
            "60-second robust estimate differs from global slope.")

    cal = require_one("CAL_DONE,")
    fail_if(cal.get("n") != "4000" or cal.get("blocks") != "8"
            or cal.get("retries") != "0",
            "MPU6050 stationary calibration did not finish as expected.")
    checks = select("CAL_CHECK,")
    fail_if(not any(fields(item).get("elapsed_ms") == "120000"
                    for item in checks),
            "Missing MPU6050 120s drift observation.")

    imus = select("IMU,")
    timings = select("TIMING,")
    fail_if(len(imus) < MIN_FIT_RECORDS,
            "Too few IMU health records (did the full diagnostic mode run?).")
    fail_if(len(timings) < MIN_FIT_RECORDS,
            "Too few TIMING records.")
    for metric in ("err", "timeout", "uart_drop", "uart_dma_err"):
        if any(fields(item).get(metric) not in (None, "0") for item in imus):
            failures.append(f"Nonzero IMU {metric} detected.")
    if any(fields(item).get("gap_over_1500us") not in (None, "0")
           for item in timings):
        failures.append("TIM11 reported gaps >1500 nominal microseconds.")
    if any(fields(item).get("late_50us") not in (None, "0")
           for item in timings):
        warnings.append("TIM11 has some +/-50us intervals (MCU-local timings).")

    probes = select("I2C_PROBE,")
    if probes:
        selected = fields(probes[-1]).get("selected")
        if selected != "0":
            warnings.append(f"I2C probe selected lower bus-speed index {selected}.")
    else:
        warnings.append("I2C probe startup line missing; inspect UART boot log.")

    fresh = select("FRESH,")
    if not fresh:
        warnings.append("FRESH gyro telemetry missing.")
    elif any(int(fields(item).get("stale", "0")) > 0 for item in fresh):
        warnings.append("Gyro FRESH telemetry includes stale sensor samples.")

    metrics = {
        "trim": EXPECTED_TRIM,
        "pll_n": EXPECTED_PLLN,
        "nominal_sysclk_mhz": NOMINAL_SYSCLK_HZ / 1e6,
        "measured_sysclk_mhz_estimate": round(mhz, 6),
        "clock_error_pct": round(error, 6),
        "effective_tim11_hz": round(float(report.get("effective_tim11_hz", 0.0)), 3),
        "fit_records": int(report.get("fit_records", 0)),
        "imu_records": len(imus),
        "timing_records": len(timings),
        "seq_missing_count": int(report.get("seq_missing_count", 0)),
        "tx_drop_delta": int(report.get("tx_drop_delta", 0)),
    }
    return failures, warnings, metrics


def self_test():
    # Independent synthetic safety-path tests, no hardware connection.
    good = {
        "effective_tim5_hz": 998_500.0,
        "effective_tim11_hz": 998.5,
        "mcu_clock_rate_error_pct": -0.15,
        "pc_seconds_per_mcu_second": 1.00150225,
        "robust_60s_pc_per_mcu": 1.001502,
        "fit_records": 220, "host_span_s": 220.3,
        "seq_missing_count": 0, "tx_drop_delta": 0,
        "fit_residual_p95_ms": 5.0,
    }
    lines = [
        "[BOOT] Isolated MPU6050/I2C test (motors disabled)",
        "CLOCK_TRIM,mode=override requested=25 active=25 hsical=127",
        "CLOCK_PROFILE,mode=validate96,trim=25,plln=192,nominal_hz=96000000,motor_off=1",
        "CLOCK_DIAG,fault=0 rcc_cr=0x0300809B",
        "CLK,seq=1,us=1153613,ticks=1000,tx_drop=0,trim=25,fault=0,sysclk_nominal=96000000",
        "CAL_DONE,n=4000 blocks=8 retries=0",
        "CAL_CHECK,elapsed_ms=120000 raw_drift_mdeg=20",
    ]
    lines += ["IMU,read_hz=1000 good=2000 err=0 busy=0 timeout=0 "
              "uart_drop=0 uart_dma_err=0"] * 220
    lines += ["TIMING,period_min_us=1000 late_50us=0 gap_over_1500us=0"] * 220
    bad, _, _ = check(good, lines)
    assert not bad, bad
    wrong = dict(good, seq_missing_count=1)
    bad, _, _ = check(wrong, lines)
    assert any("Missing CLK" in s for s in bad)
    wrong = dict(good, effective_tim5_hz=1_040_000.0)
    bad, _, _ = check(wrong, lines)
    assert any("SYSCLK" in s for s in bad)
    changed = [m.replace("trim=25", "trim=26") if m.startswith("CLK,") else m
               for m in lines]
    bad, _, _ = check(good, changed)
    assert any("CLK readback mismatch" in s for s in bad)
    print("SELF-TEST PASSED: nominal profile, UART/CLK integrity, "
          "overclock and firmware mismatch rejection.")
    print("No hardware was touched.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--folder", type=pathlib.Path,
                        help="Directory containing clock_samples.csv and raw_serial.log")
    action.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return

    folder = args.folder
    try:
        csv_file = folder / "clock_samples.csv"
        raw_file = folder / "raw_serial.log"
        rows = read_csv(csv_file)
        report, *_ = analyze(rows)
        messages = payloads(raw_file.read_text(encoding="utf-8"))
        failures, warnings, metrics = check(report, messages)
    except (ValueError, OSError, ZeroDivisionError) as exc:
        print(f"FAIL: cannot evaluate the clock log: {exc}", file=sys.stderr)
        sys.exit(2)

    print("96 MHz / HSITRIM=25 validation")
    for key, value in metrics.items():
        print(f"  {key}: {value}")
    for item in warnings:
        print("WARNING:", item)
    for item in failures:
        print("FAIL:", item)
    print("RESULT:", "PASS (diagnostic only)" if not failures else "FAIL")
    if not failures:
        print("Do not deploy as production clock based on this alone; "
              "repeat under realistic temperature and supply voltage.")
    sys.exit(0 if not failures else 2)


if __name__ == "__main__":
    main()
