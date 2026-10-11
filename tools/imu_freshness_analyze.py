#!/usr/bin/env python3
"""Analyze a motor-OFF STM32 MPU6050 DATA_RDY/1kHz TIM11 synchronization test.

Usage:
    python tools/imu_freshness_analyze.py --self-test
    python tools/imu_freshness_analyze.py --folder clock_logs/SESSION

Measures what firmware actually observes: INT_STATUS DATA_RDY polls, successful
gyro-read completion spacing, and repeated samples visible at TIM11 ticks.
It cannot measure the MPU6050 internal sample timestamp without a DATA_RDY pin.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys

from clock_serial_test import analyze, read_csv

MIN_FRAMES = 150
NOMINAL_SYSCLK_MHZ = 96
MAX_TEST_SYSCLK_MHZ = 98


def parse_fields(line: str) -> dict[str, str]:
    return dict(x.split("=", 1) for x in re.split(r"[,\s]+", line)
                if "=" in x)


def extract(raw_text: str) -> list[str]:
    lines = [x.split("\t", 1)[-1].strip() for x in raw_text.splitlines()
             if x.strip()]
    boot_ids = [i for i, x in enumerate(lines)
                if x.startswith("[BOOT] Isolated MPU6050/I2C test (motors disabled)")]
    return lines[boot_ids[-1]:] if boot_ids else []


def validate(report: dict, lines: list[str]):
    failures, cautions = [], []

    def selected(tag):
        return [parse_fields(x) for x in lines if x.startswith(tag + ",")]

    def require(tag):
        values = selected(tag)
        if not values:
            failures.append("Missing " + tag + " log")
        return values

    if not lines:
        return ["No post-RESET motor-off BOOT marker found"], [], {}
    trim = require("CLOCK_TRIM")
    clock_profile = require("CLOCK_PROFILE")
    clock_fault = require("CLOCK_DIAG")
    sync_profile = require("MPU_SYNC_DIAG")
    probes = require("I2C_PROBE")
    sensor = require("SYNC_SENSOR")
    ctrl = require("SYNC_CTRL")
    imu = require("IMU")
    timing = require("TIMING")
    clk = require("CLK")
    cal = require("CAL_DONE")
    if any(x.startswith("BOOT_FAIL") for x in lines):
        failures.append("MCU reported boot failure")
    if trim and (trim[-1].get("active") != "25" or trim[-1].get("requested") != "25"):
        failures.append("Wrong HSITRIM, expected 25")
    if clock_profile and any(clock_profile[-1].get(k) != v for k, v in
                             (("mode", "validate96"), ("plln", "192"),
                              ("nominal_hz", "96000000"), ("motor_off", "1"))):
        failures.append("Wrong PLL/clock profile")
    if clock_fault and clock_fault[-1].get("fault") != "0":
        failures.append("RCC startup fault")
    if sync_profile and any(sync_profile[-1].get(k) != v for k, v in
                            (("mode", "status_poll"), ("trim", "25"),
                             ("sysclk_nominal", "96000000"), ("motors_off", "1"))):
        failures.append("Wrong MPU synchronization diagnostic firmware")
    if probes and probes[-1].get("selected") != "0":
        cautions.append("I2C did not select 400kHz fast mode")
    if not cal or cal[-1].get("n") != "4000" or cal[-1].get("retries") != "0":
        failures.append("4k-sample gyro calibration not confirmed")
    if report["fit_records"] < MIN_FRAMES:
        failures.append("Need at least 150 CLK regression records (run 240s)")
    if report["seq_missing_count"] or report["tx_drop_delta"]:
        failures.append("Missing CLK sequence / firmware UART TX overrun")
    if not (0 < (report["effective_tim5_hz"] / 1e6) * NOMINAL_SYSCLK_MHZ
            < MAX_TEST_SYSCLK_MHZ):
        failures.append("SYSCLK estimate outside safety review window")
    if abs(report["mcu_clock_rate_error_pct"]) > 0.5:
        failures.append("Clock differs from nominal by more than 0.5%")
    for frame in clk:
        if any(frame.get(k) != v for k, v in
               (("trim", "25"), ("fault", "0"), ("sysclk_nominal", "96000000"))):
            failures.append("CLK firmware signature mismatch")
            break
    if len(sensor) < 150 or len(ctrl) < 150:
        failures.append("Not enough SYNC_SENSOR / SYNC_CTRL records")
    for tag, arr, keys in [
        ("IMU", imu, ("err", "timeout", "uart_drop", "uart_dma_err")),
        ("TIMING", timing, ("gap_over_1500us", "late_50us")),
    ]:
        for key in keys:
            if any(x.get(key) != "0" for x in arr):
                failures.append(tag + " nonzero/missing " + key)
                break

    # Work only AFTER calibration, to avoid treating initialization as
    # normal operation. Counters are cumulative from MCU boot.
    sensor_after, ctrl_after = sensor, ctrl
    if cal and lines:
        end_cal = max(i for i, x in enumerate(lines) if x.startswith("CAL_DONE,"))
        sids = [i for i, x in enumerate(lines) if x.startswith("SYNC_SENSOR,")]
        cids = [i for i, x in enumerate(lines) if x.startswith("SYNC_CTRL,")]
        sensor_after = [sensor[j] for j, i in enumerate(sids) if i > end_cal]
        ctrl_after = [ctrl[j] for j, i in enumerate(cids) if i > end_cal]
    if len(sensor_after) < 2 or len(ctrl_after) < 2:
        failures.append("Not enough post-calibration SYNC data")
        return failures, cautions, {}

    def count_delta(items, field):
        try:
            first = int(items[0][field])
            last = int(items[-1][field])
        except (KeyError, ValueError):
            failures.append("Missing/non-numeric counter " + field)
            return 0
        if last < first:
            failures.append("Counter decreased unexpectedly: " + field)
            return 0
        return last - first

    def last_num(items, field):
        try:
            return int(items[-1][field])
        except (KeyError, ValueError):
            failures.append("Missing/non-numeric field " + field)
            return 0

    ready = count_delta(sensor_after, "ready")
    notready = count_delta(sensor_after, "notready")
    fresh = count_delta(sensor_after, "fresh")
    stale = count_delta(sensor_after, "stale")
    fresh_gap_over1500 = count_delta(sensor_after, "fgap_gt1500")
    fresh_gap_over2000 = count_delta(sensor_after, "fgap_gt2000")
    total_ticks = count_delta(ctrl_after, "ticks")
    repeated_ticks = count_delta(ctrl_after, "repeat")
    aged_ticks = count_delta(ctrl_after, "age_gt1500")
    if abs(ready - fresh) > 3 or abs(notready - stale) > 3:
        cautions.append("DATA_RDY status count differs from gyro-read completion counts")
    if total_ticks <= 0 or ready + notready <= 0:
        failures.append("Insufficient accepted post-calibration sensor/control counters")
    if fresh_gap_over2000:
        cautions.append(f"{fresh_gap_over2000} accepted-sample gaps exceeded 2ms")
    if repeated_ticks:
        cautions.append("TIM11 sometimes reuses the latest accepted gyro sample")
    if aged_ticks:
        cautions.append("Some TIM11 ticks observed a gyro read completed over 1.5ms earlier")
    metrics = {
        "clock_error_pct": round(report["mcu_clock_rate_error_pct"], 5),
        "sysclk_mhz_estimate": round(report["effective_tim5_hz"] * NOMINAL_SYSCLK_MHZ / 1e6, 5),
        "postcal_status_ready": ready,
        "postcal_status_not_ready": notready,
        "postcal_status_not_ready_pct": round(100 * notready / max(1, ready + notready), 4),
        "postcal_fresh_gyro_reads": fresh,
        "postcal_status_zero_gyro_reads": stale,
        "fresh_gap_min_us_observed": last_num(sensor_after, "fgap_min"),
        "fresh_gap_max_us_observed": last_num(sensor_after, "fgap_max"),
        "fresh_gap_gt1500_count": fresh_gap_over1500,
        "fresh_gap_gt2000_count": fresh_gap_over2000,
        "status_poll_gap_max_us_observed": last_num(sensor_after, "poll_gap_max"),
        "consecutive_notready_max": last_num(sensor_after, "nr_run_max"),
        "postcal_TIM11_ticks": total_ticks,
        "postcal_reused_gyro_ticks": repeated_ticks,
        "gyro_reused_tick_pct": round(100 * repeated_ticks / max(1, total_ticks), 4),
        "gyro_completion_age_max_us_observed": last_num(ctrl_after, "age_max"),
        "gyro_completion_age_gt1500_ticks": aged_ticks,
        "host_fit_records": report["fit_records"],
        "uart_tx_drop": report["tx_drop_delta"],
        "clk_seq_missing": report["seq_missing_count"],
    }
    return failures, cautions, metrics


def self_test():
    fake = {
        "fit_records": 230, "seq_missing_count": 0, "tx_drop_delta": 0,
        "effective_tim5_hz": 998000.0, "mcu_clock_rate_error_pct": -0.2,
    }
    lines = [
        "[BOOT] Isolated MPU6050/I2C test (motors disabled)",
        "CLOCK_TRIM,mode=override requested=25 active=25 hsical=134",
        "CLOCK_PROFILE,mode=validate96,trim=25,plln=192,nominal_hz=96000000,motor_off=1",
        "CLOCK_DIAG,fault=0",
        "MPU_SYNC_DIAG,mode=status_poll,trim=25,sysclk_nominal=96000000,motors_off=1",
        "I2C_PROBE,selected=0 selected_scl=400000",
        "CAL_DONE,n=4000 blocks=8 retries=0",
    ]
    for i in range(155):
        lines += [
            f"CLK,seq={i+1},us={1000000*(i+1)},ticks={1000*(i+1)},tx_drop=0,"
            "trim=25,fault=0,sysclk_nominal=96000000",
            "IMU,read_hz=1000 err=0 timeout=0 uart_drop=0 uart_dma_err=0",
            "TIMING,late_50us=0 gap_over_1500us=0",
            f"SYNC_SENSOR,ready={1000*(i+1)},notready={4*(i+1)},"
            f"fresh={1000*(i+1)},stale={4*(i+1)},fgap_min=999,"
            "fgap_max=2001,fgap_gt1500=4,fgap_gt2000=0,"
            "poll_gap_max=1010,nr_run_max=1",
            f"SYNC_CTRL,ticks={1000*(i+1)},repeat={4*(i+1)},"
            "age_max=2000,age_gt1000=6,age_gt1500=1",
        ]
    failures, _, metrics = validate(fake, lines)
    assert not failures, failures
    assert metrics["postcal_status_not_ready_pct"] > 0
    assert metrics["gyro_reused_tick_pct"] > 0
    bad = dict(fake, seq_missing_count=1)
    assert any("Missing CLK" in x for x in validate(bad, lines)[0])
    bad_lines = [x.replace("trim=25", "trim=26") if x.startswith("CLK,")
                 else x for x in lines]
    assert any("signature mismatch" in x for x in validate(fake, bad_lines)[0])
    assert not extract("1.0\tOLD\n2.0\t[BOOT] Isolated MPU6050/I2C test (motors disabled)\n")[0].startswith("OLD")
    print("SELF-TEST PASSED: 96MHz profile, post-calibration deltas, repeat rate, and safe rejects.")
    print("No firmware build or hardware access was performed.")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--folder", type=pathlib.Path)
    group.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    if args.self_test:
        self_test()
        return
    path = args.folder
    try:
        rows = read_csv(path / "clock_samples.csv")
        report, *_ = analyze(rows)
        messages = extract((path / "raw_serial.log").read_text(encoding="utf-8"))
        failures, cautions, metrics = validate(report, messages)
    except (OSError, ValueError, ZeroDivisionError) as exc:
        print("FAIL: unable to parse logs:", exc, file=sys.stderr)
        sys.exit(2)
    result = {"status": "PASS_DIAGNOSTIC" if not failures else "FAIL",
              "metrics": metrics, "warnings": cautions, "failures": failures}
    (path / "imu_freshness_report.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("\nInterpretation: status=0 is NOT proof of a dropped physical sample.")
    print("TIM11 'reuse' means an accepted gyro sample has not changed since")
    print("the prior control tick; it does not prove repeated identical raw codes.")
    print("Read-completion age is NOT an MPU internal sensor-data timestamp.")
    sys.exit(2 if failures else 0)


if __name__ == "__main__":
    main()
