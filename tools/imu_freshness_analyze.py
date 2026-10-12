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


BOOT_MARKER = "[BOOT] Isolated MPU6050/I2C test (motors disabled)"


def extract(raw_text: str) -> list[str]:
    """Recover RESET mid-UART-frame; preserve late-start captures as partial.

    If reset occurs halfway through a TX line, the host can receive e.g.
    SYNC_CTRL,ticks=0,re[BOOT] Isolated ...; keep the suffix after [BOOT].
    Without any boot marker we retain raw lines instead of returning [].
    Safety policy: a late-start run is always PARTIAL_DIAGNOSTIC.
    """
    lines = [x.split("\t", 1)[-1].strip() for x in raw_text.splitlines()
             if x.strip()]
    boot_ids = [i for i, x in enumerate(lines) if BOOT_MARKER in x]
    if not boot_ids:
        return lines
    index = boot_ids[-1]
    return [lines[index][lines[index].index(BOOT_MARKER):]] + lines[index + 1:]


def validate(report: dict, lines: list[str]):
    failures, cautions = [], []

    def selected(tag):
        return [parse_fields(x) for x in lines if x.startswith(tag + ",")]

    if not lines:
        return ["No UART diagnostic lines found"], [], {}
    has_boot = any(x == BOOT_MARKER for x in lines)
    if not has_boot:
        cautions.append("Partial capture: no [BOOT] marker; startup, motor-off "
                        "state and calibration history cannot be fully verified.")

    def require(tag, startup=False):
        values = selected(tag)
        if not values:
            if startup and not has_boot:
                cautions.append("Missing startup frame: " + tag)
            else:
                failures.append("Missing " + tag + " log")
        return values

    trim = require("CLOCK_TRIM", startup=True)
    clock_profile = require("CLOCK_PROFILE", startup=True)
    clock_fault = require("CLOCK_DIAG", startup=True)
    sync_profile = require("MPU_SYNC_DIAG", startup=True)
    probes = require("I2C_PROBE", startup=True)
    sensor = require("SYNC_SENSOR")
    ctrl = require("SYNC_CTRL")
    imu = require("IMU")
    timing = require("TIMING")
    clk = require("CLK")
    cal = require("CAL_DONE", startup=True)
    policy_rows = selected("SYNC_POLICY")
    ab_frames = selected("SYNC_AB")
    mode = policy_rows[-1].get("policy", "legacy") if policy_rows else "legacy"
    if not policy_rows and not has_boot and ab_frames and sensor:
        # Infer behavior, NOT proven build identity. Untrusted as production
        # attestation; caller marks results PARTIAL and comparison opt-in.
        last_ab = ab_frames[-1]
        last_sensor = sensor[-1]
        if (int(last_ab.get("gyro_skipped", "-1")) > 0 and
            int(last_ab.get("retry_started", "-1")) > 0 and
            last_sensor.get("stale") == "0"):
            mode = "B"
            cautions.append("Policy B inferred from skip/retry counters; "
                            "SYNC_POLICY startup frame was not recorded.")
        elif (last_ab.get("gyro_skipped") == "0" and
              last_ab.get("retry_started") == "0"):
            mode = "A"
            cautions.append("Policy A inferred from counters; "
                            "SYNC_POLICY startup frame was not recorded.")
    if not policy_rows and has_boot and ab_frames:
        failures.append("Missing SYNC_POLICY for new A/B firmware after observed boot")

    if mode not in ("A", "B", "legacy"):
        failures.append("Unknown SYNC_POLICY")
    if mode in ("A", "B"):
        if policy_rows and policy_rows[-1].get("motor_off") != "1":
            failures.append("A/B profile is not confirmed motor-off")
        if len(ab_frames) < 150:
            failures.append("Missing A/B diagnostic counters")
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
    if cal:
        if cal[-1].get("n") != "4000" or cal[-1].get("retries") != "0":
            failures.append("4k-sample gyro calibration failed validation")
    elif not has_boot:
        # Post-calibration telemetry is suggestive, not equivalent to
        # having observed CAL_DONE with accepted block/retry counts.
        if not all(x.get("cal") == "1" and x.get("cal_n") == "4000"
                   for x in imu):
            failures.append("Missing CAL_DONE and IMU calibration status invalid")
        else:
            cautions.append("CAL_DONE missed; IMU reports calibrated but "
                            "stationarity/retry history was not observed.")
    # Strictly validate clock identity even for a late-start capture.
    # This establishes nominal 96MHz/trim25 MCU readback, not an independent
    # guarantee of which motor-control source was flashed.
    if not clk:
        failures.append("Cannot validate MCU clock with no CLK data")
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
    sensor_after, ctrl_after, ab_after = sensor, ctrl, ab_frames
    if cal and lines:
        end_cal = max(i for i, x in enumerate(lines) if x.startswith("CAL_DONE,"))
        sids = [i for i, x in enumerate(lines) if x.startswith("SYNC_SENSOR,")]
        cids = [i for i, x in enumerate(lines) if x.startswith("SYNC_CTRL,")]
        aids = [i for i, x in enumerate(lines) if x.startswith("SYNC_AB,")]
        sensor_after = [sensor[j] for j, i in enumerate(sids) if i > end_cal]
        ctrl_after = [ctrl[j] for j, i in enumerate(cids) if i > end_cal]
        ab_after = [ab_frames[j] for j, i in enumerate(aids) if i > end_cal]
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
    if abs(ready - fresh) > 3:
        cautions.append("DATA_RDY ready count differs from gyro-read completion count")
    if mode != "B" and abs(notready - stale) > 3:
        cautions.append("DATA_RDY not-ready count differs from stale gyro-read count")
    if mode == "B" and stale != 0:
        failures.append("Policy B unexpectedly read gyro with DATA_RDY=0")
    if total_ticks <= 0 or ready + notready <= 0:
        failures.append("Insufficient accepted post-calibration sensor/control counters")
    if fresh_gap_over2000:
        cautions.append(f"{fresh_gap_over2000} accepted-sample gaps exceeded 2ms")
    if repeated_ticks:
        cautions.append("TIM11 sometimes reuses the latest accepted gyro sample")
    if aged_ticks:
        cautions.append("Some TIM11 ticks observed a gyro read completed over 1.5ms earlier")
    ab_metrics = {}
    if mode in ("A", "B"):
        if len(ab_after) < 2:
            failures.append("Not enough post-calibration SYNC_AB frames")
        else:
            names = ("initial_nr", "gyro_started", "gyro_skipped",
                     "retry_started", "retry_ready", "retry_nr",
                     "retry_expired", "retry_busy")
            ab_metrics = {name: count_delta(ab_after, name) for name in names}
            ab_metrics["status_checks_per_1k_ticks"] = round(
                1000.0 * (ready + notready) / max(1, total_ticks), 3)
            ab_metrics["gyro_read_starts_per_1k_ticks"] = round(
                1000.0 * ab_metrics["gyro_started"] / max(1, total_ticks), 3)
            ab_metrics["estimated_i2c_transactions_per_1k_ticks"] = round(
                1000.0 * (ready + notready + ab_metrics["gyro_started"])
                / max(1, total_ticks), 3)
            if mode == "A" and (
                ab_metrics["gyro_skipped"] != 0 or
                ab_metrics["retry_started"] != 0):
                failures.append("A control firmware unexpectedly skipped/retried gyro")
            if mode == "B":
                if abs(ab_metrics["gyro_skipped"] - notready) > 3:
                    cautions.append("B skipped-gyro count differs from DATA_RDY=0 statuses")
                if ab_metrics["retry_started"] > ab_metrics["initial_nr"] + 3:
                    failures.append("B retry count exceeded single-retry bound")
                if ab_metrics["retry_nr"] + ab_metrics["retry_ready"] > (
                    ab_metrics["retry_started"] + 3
                ):
                    failures.append("B retry outcome count exceeded starts")
                if ab_metrics["retry_busy"]:
                    cautions.append("Some B retries encountered a busy I2C bus")
    metrics = {
        "boot_verified": has_boot,
        "policy_inferred": not bool(policy_rows),
        "sampling_policy": mode,
        "age_p95_bin_us": last_num(ctrl_after, "p95_bin") if mode in ("A", "B") else None,
        "age_p99_bin_us": last_num(ctrl_after, "p99_bin") if mode in ("A", "B") else None,
        "ab": ab_metrics,
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

    # Regression check for both newly tagged A/B diagnostic variants.
    # Build the A/B synthetic captures with cumulative counters and the
    # approximate sample-age histogram fields.
    def with_policy(policy):
        data = [x.replace("age_gt1500=1",
                          "age_gt1500=1,p95_bin=750,p99_bin=1500")
                if x.startswith("SYNC_CTRL,") else x for x in lines]
        data.insert(5, "SYNC_POLICY,policy=" + policy +
                    ",skip_on_nr=" + ("1" if policy == "B" else "0") +
                    ",retry_us=220,retry_max=1,motor_off=1")
        for i in range(155):
            n = i + 1
            data.append(
                "SYNC_AB,initial_nr=" + str(4*n) +
                ",gyro_started=" + str((1000 if policy == "B" else 1004)*n) +
                ",gyro_skipped=" + str(4*n if policy == "B" else 0) +
                ",retry_started=" + str(4*n if policy == "B" else 0) +
                ",retry_ready=" + str(4*n if policy == "B" else 0) +
                ",retry_nr=0,retry_expired=0,retry_busy=0")
        if policy == "B":
            data = [x.split(",stale=")[0] + ",stale=0," +
                    x.split(",stale=", 1)[1].split(",", 1)[1]
                    if x.startswith("SYNC_SENSOR,") else x
                    for x in data]
        return data

    fa, _, ma = validate(fake, with_policy("A"))
    fb, _, mb = validate(fake, with_policy("B"))
    assert not fa, fa
    assert not fb, fb
    assert ma["sampling_policy"] == "A"
    assert mb["sampling_policy"] == "B"
    assert mb["ab"]["gyro_skipped"] > 0
    assert mb["ab"]["retry_started"] > 0

    # Real-world B captures may begin after firmware boot/calibration.
    # Preserve legitimate steady-state telemetry while explicitly refusing
    # to treat absent boot headers as complete firmware attestation.
    partial_b = []
    for item in with_policy("B"):
        if item.startswith((
            BOOT_MARKER, "CLOCK_TRIM,", "CLOCK_PROFILE,", "CLOCK_DIAG,",
            "MPU_SYNC_DIAG,", "I2C_PROBE,", "CAL_DONE,", "SYNC_POLICY,",
        )):
            continue
        if item.startswith("IMU,"):
            item = item.replace("IMU,read_hz=", "IMU,cal=1 cal_n=4000 read_hz=")
        partial_b.append(item)
    partial_bad, partial_warning, partial_metrics = validate(fake, partial_b)
    assert not partial_bad, partial_bad
    assert partial_metrics["sampling_policy"] == "B"
    assert partial_metrics["boot_verified"] is False
    assert partial_metrics["policy_inferred"] is True
    assert any("Partial capture" in x for x in partial_warning)

    bad = dict(fake, seq_missing_count=1)
    assert any("Missing CLK" in x for x in validate(bad, lines)[0])
    bad_lines = [x.replace("trim=25", "trim=26") if x.startswith("CLK,")
                 else x for x in lines]
    assert any("signature mismatch" in x for x in validate(fake, bad_lines)[0])
    assert extract("1.0\tOLD\n2.0\tSYNC_CTRL,ticks=0,re" + BOOT_MARKER +
                   "\n")[0] == BOOT_MARKER
    assert extract("1.0\tCLK,seq=25,us=1000\n") == ["CLK,seq=25,us=1000"]
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
    # Never turn missing [BOOT]/CAL_DONE into a full PASS. Metrics remain
    # available for an explicitly provisional comparison.
    partial = not any(x == BOOT_MARKER for x in messages)
    status = ("FAIL" if failures else
              "PARTIAL_DIAGNOSTIC" if partial else "PASS_DIAGNOSTIC")
    result = {"status": status,
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
