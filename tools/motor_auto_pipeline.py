#!/usr/bin/env python3
"""One-command host-controlled PWM/encoder diagnostic.

The same Python process builds/flashes opt-in firmware, verifies bidirectional
UART communication, sends the explicit START command, captures 3 rounds of
21 stages covering startup sweep, open-loop response and slew sensitivity; collects 400ms neutral stop traces and statistics.
NO KEY/button, no automatic motion on reset, no current/thermal safety claim.

IMPORTANT: MCU PA10 / USART1 RX must physically connect to the USB-UART TX
used by this COM port. If it is not wired, fail closed without START.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import re
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
FIRMWARE_ENV = "motor_auto_pipeline_96m_trim25"
PROFILE_VERSION = 14
STOP_CCR = 3200 # Hi-Z settled PWM HIGH when both IN inputs LOW
FAMILIES = 4
HIGH_PROFILE = False
SENSOR_PROFILE = False
BASE_STEPS = 21
HIGH_PULSE_MS = 120
HIGH_POST_MS = 800
HIGH_STAGES = (
    (300,0),(0,300),(400,0),(0,400),
    (500,0),(0,500),(600,0),(0,600)
)
NUM_ROUNDS = 3
PULSE_MS = 200
POST_MS = 400
# Firmware owns this fixed capped matrix; host cannot submit arbitrary duty.
MATRIX = (
    (80,0),(0,80),(90,0),(0,90),(100,0),(0,100),
    (110,0),(0,110),(120,0),(0,120),
    (120,0),(0,120),(160,0),(0,160),(200,0),(0,200),(160,160),
    (160,0),(0,160),(160,0),(0,160)
)
TESTS = ("startup",)*10 + ("response",)*7 + ("slew_slow",)*2 + ("slew_fast",)*2
RISE_RATES = (10,)*17 + (5,)*2 + (20,)*2
MAX_CMD = 200
DUAL_MAX_CMD = 160
MAX_CCR = 640  # TIM2 96MHz / 30kHz = 3200 counts, 20% = 640
EXPECTED = NUM_ROUNDS * len(MATRIX)
CSV_FIELDS = ("round", "stage", "phase", "test", "rise", "ms", "elapsed",
              "applied_l", "applied_r", "enc_l", "enc_r", "ccr_l",
              "ccr_r", "tim11_late", "tim11_max_cycles", "tim11_gaps", "tx_drop")


def configure_profile(high=False, sensors=False, *, legacy=False):
    """One strict mutually identified MCU firmware image per invocation."""
    global FIRMWARE_ENV, PROFILE_VERSION, FAMILIES, HIGH_PROFILE, SENSOR_PROFILE
    global MATRIX, TESTS, RISE_RATES, MAX_CMD, MAX_CCR, EXPECTED, STOP_CCR
    HIGH_PROFILE = bool(high)
    SENSOR_PROFILE = bool(sensors)
    FIRMWARE_ENV = (
        "motor_auto_sensor_high_96m_trim25" if sensors and high else
        "motor_auto_sensor_96m_trim25" if sensors else
        "motor_auto_high_96m_trim25" if high else
        "motor_auto_pipeline_96m_trim25"
    )
    PROFILE_VERSION = ((8 if high else 7) if sensors else (6 if high else 4)) + (0 if legacy else 10)
    STOP_CCR = 0 if legacy else 3200
    FAMILIES = 5 if high else 4
    MATRIX = (
        ((80,0),(0,80),(90,0),(0,90),(100,0),(0,100),
         (110,0),(0,110),(120,0),(0,120),
         (120,0),(0,120),(160,0),(0,160),(200,0),(0,200),(160,160),
         (160,0),(0,160),(160,0),(0,160)) +
        (HIGH_STAGES if high else ())
    )
    TESTS = (("startup",)*10 + ("response",)*7 + ("slew_slow",)*2 +
             ("slew_fast",)*2 + (("high",)*8 if high else ()))
    RISE_RATES = (10,)*17 + (5,)*2 + (20,)*2 + ((10,)*8 if high else ())
    MAX_CMD = 600 if high else 200
    MAX_CCR = 1920 if high else 640
    EXPECTED = NUM_ROUNDS * len(MATRIX)



def configure_from_saved_log(raw: list[str], *, high: bool, sensors: bool):
    """Select archived/new telemetry by on-wire version, not today's source."""
    versions = {
        intval(values, "version")
        for line in raw
        for name, values in [split_record(line)]
        if name == "MOTOR_AUTO_PROFILE"
    }
    old_version = (8 if high else 7) if sensors else (6 if high else 4)
    new_version = old_version + 10
    if versions == {old_version}:
        configure_profile(high, sensors, legacy=True)
    elif versions == {new_version}:
        configure_profile(high, sensors, legacy=False)
    else:
        raise ValueError(
            f"Missing/mixed/incorrect MOTOR_AUTO_PROFILE versions {sorted(versions)}; "
            f"expected only {old_version} (archived CCR=0) or "
            f"{new_version} (STOP_HIZ CCR=3200)"
        )

def stage_pulse_ms(stage):
    return HIGH_PULSE_MS if stage >= BASE_STEPS else PULSE_MS


def stage_post_ms(stage):
    return HIGH_POST_MS if stage >= BASE_STEPS else POST_MS


def split_record(line: str):
    if "\t" in line:
        line = line.split("\t", 1)[1]
    name, sep, rest = line.strip().partition(",")
    if not sep:
        return name, {}
    values = {}
    for chunk in rest.split(","):
        if "=" in chunk:
            k, v = chunk.split("=", 1)
            values[k] = v
    return name, values


def intval(obj, key, default=-999999):
    try:
        return int(obj[key])
    except (KeyError, TypeError, ValueError):
        return default


def phase_findings(stats):
    """Summaries derived only from encoder counts, not physical voltage/current."""
    startup = {}
    for wheel in ("l", "r"):
        subset = sorted((x for x in stats if x["test"] == "startup" and
                         x["wheel"] == wheel), key=lambda x:x["cmd_permille"])
        reliable = [x["cmd_permille"] for x in subset if x["sustained_3of3"]]
        # Do NOT interpret a nonmonotonic series (8% works, 9% fails)
        # as a calibrated startup threshold. A conservative *observed*
        # candidate requires all higher tested PWM to succeed 3/3 too.
        stable_up = [
            x["cmd_permille"] for i,x in enumerate(subset)
            if all(z["sustained_3of3"] for z in subset[i:]) and
               len(subset[i:]) > 0
        ]
        anomalies = [
            {"lower_permille":x["cmd_permille"],
             "higher_permille":y["cmd_permille"],
             "lower_successful_repeats":x["sustained_replicates"],
             "higher_successful_repeats":y["sustained_replicates"]}
            for x,y in zip(subset, subset[1:])
            if x["executed_replicates"]==3 and y["executed_replicates"]==3
            and x["sustained_replicates"] > y["sustained_replicates"]
        ]
        startup[wheel] = {
            "planned_permille": [x["cmd_permille"] for x in subset],
            "measured_permille": [x["cmd_permille"] for x in subset if x["executed_replicates"] > 0],
            "sustained_replicates_by_duty": {
                str(x["cmd_permille"]): x["sustained_replicates"] if
                x["executed_replicates"] > 0 else None for x in subset},
            "lowest_tested_3of3_permille": min(reliable) if reliable else None,
            "lowest_monotonic_suffix_3of3_permille": min(stable_up) if stable_up else None,
            "nonmonotonic_success_decreases": anomalies,
        }
    response = []
    for duty in (120,160,200):
        left = next((x for x in stats if x["test"] == "response" and
                     x["wheel"] == "l" and x["cmd_permille"] == duty and
                     x["stage"] != 16), None)
        right = next((x for x in stats if x["test"] == "response" and
                      x["wheel"] == "r" and x["cmd_permille"] == duty and
                      x["stage"] != 16), None)
        if left is not None and right is not None and left["mean_count"] is not None and right["mean_count"] is not None:
            base = right["mean_count"]
            response.append({
                "pwm_permille":duty,"mean_left_encoder_count":left["mean_count"],
                "mean_right_encoder_count":base,
                "relative_count_difference_pct":round(100*(left["mean_count"]-base)/base,2) if base else None,
                "mean_left_post_stop_counts":round(statistics.mean(
                    [n for n in left["three_post_stop_deltas"] if n is not None]),2)
                    if any(n is not None for n in left["three_post_stop_deltas"]) else None,
                "mean_right_post_stop_counts":round(statistics.mean(
                    [n for n in right["three_post_stop_deltas"] if n is not None]),2)
                    if any(n is not None for n in right["three_post_stop_deltas"]) else None,
            })
    slew = []
    for wheel in ("l","r"):
        slow = next((x for x in stats if x["test"] == "slew_slow" and x["wheel"] == wheel),None)
        fast = next((x for x in stats if x["test"] == "slew_fast" and x["wheel"] == wheel),None)
        if slow is not None and fast is not None:
            slew.append({
                "wheel":wheel, "duty_permille":160,
                "rise_slow_permille_per_ms":5,
                "rise_fast_permille_per_ms":20,
                "mean_count_slow":slow["mean_count"],
                "mean_count_fast":fast["mean_count"],
                "mean_count_fast_minus_slow":round(fast["mean_count"]-slow["mean_count"],2) if
                    fast["mean_count"] is not None and slow["mean_count"] is not None else None,
            })
    return {
        "startup_threshold_heuristic":startup,
        "open_loop_response":response,
        "rise_slew_comparison":slew,
        "limitations":"Based on encoder count only; startup is a heuristic at the tested 8-12% settings and 3 repeats. Nonzero post-STOP counts do not establish exact electrical coast/brake mode."
    }


def analyze(lines, *, setup=None):
    records = [split_record(x) for x in lines]
    by_kind = {}
    for name, values in records:
        by_kind.setdefault(name, []).append(values)
    errors = []
    profile = by_kind.get("MOTOR_AUTO_PROFILE", [])
    ready = by_kind.get("MOTOR_AUTO_READY", [])
    begins = by_kind.get("MOTOR_AUTO_BEGIN", [])
    starts = by_kind.get("MOTOR_AUTO_START", [])
    ends = by_kind.get("MOTOR_AUTO_END", [])
    posts = by_kind.get("MOTOR_AUTO_POST_END", [])
    done = by_kind.get("MOTOR_AUTO_DONE", [])
    samples = by_kind.get("MOTOR_AUTO_SAMPLE", [])
    faults = by_kind.get("MOTOR_AUTO_FAULT", [])
    r_ends = by_kind.get("MOTOR_AUTO_ROUND_END", [])
    links = by_kind.get("MOTOR_AUTO_LINK", [])
    if not profile:
        errors.append("Missing firmware profile / wrong firmware loaded")
    else:
        p = profile[-1]
        for key, value in {
            "version": PROFILE_VERSION, "nominal_hz": 96000000,
            "pwm_hz": 30000, "rounds": NUM_ROUNDS,
            "steps": len(MATRIX), "hard_cap_permille": MAX_CMD,
            "both_cap_permille": DUAL_MAX_CMD, "families": FAMILIES,
            "pulse_ms": PULSE_MS, "post_ms": POST_MS,
            "boot_neutral": 1,
        }.items():
            if intval(p, key) != value:
                errors.append(f"Firmware profile mismatch: {key}")
        if intval(p,"sensor",0) != int(SENSOR_PROFILE):
            errors.append("Firmware sensor mode mismatch")
        if p.get("rx_mode") != "irq256":
            errors.append("Firmware missing USART1 RX interrupt ring profile")
        if HIGH_PROFILE and (intval(p,"high_pulse_ms") != HIGH_PULSE_MS or
                             intval(p,"high_post_ms") != HIGH_POST_MS):
            errors.append("High PWM pulse/settle protection profile mismatch")
    if not ready:
        errors.append("Missing UART RX handshake: MOTOR_AUTO_READY")
    if ready and intval(ready[-1],"sensor") != int(SENSOR_PROFILE):
        errors.append("READY sensor integration profile mismatch")
    static_regs = by_kind.get("MOTOR_AUTO_STATIC", [])
    if SENSOR_PROFILE and not static_regs:
        errors.append("Missing read-only MCU STOP pin/CCR/preload snapshot")
    for rec in static_regs:
        expected_static = {
            "ccr_l":STOP_CCR,"ccr_r":STOP_CCR,
            "in_l1":0,"in_l2":0,"in_r1":0,"in_r2":0,
            "oc1pe":1,"oc2pe":1,"arr":3199,
        }
        if any(intval(rec,key)!=value for key,value in expected_static.items()):
            errors.append("MCU STOP/static direction pins or CCR preload incorrect at PING")
    sensor_ready = by_kind.get("MOTOR_AUTO_SENSOR_READY", [])
    sensor_rows = by_kind.get("MOTOR_AUTO_SENSORS", [])
    if SENSOR_PROFILE:
        if not sensor_ready or max((intval(x,"ir",0) for x in sensor_ready),default=0) < 10:
            errors.append("Integrated IMU calibration/IR scan readiness not confirmed")
        if len(sensor_rows) != len(starts)*2:
            errors.append(f"Missing integrated sensor telemetry: {len(sensor_rows)}/{len(starts)*2}")
        previous = None
        for x in sensor_rows:
            if x.get("phase") not in ("drive","post"):
                errors.append("Invalid sensor telemetry phase")
            if (intval(x,"gyro_n",0) < 1 or
                intval(x,"gyro_abs_avg",-1) < 0 or
                intval(x,"gyro_abs_peak",-1) < intval(x,"gyro_abs_avg",0)):
                errors.append("Missing/invalid encoder-synchronized gyro vibration/noise proxy")
            if previous is not None:
                for key in ("good","fresh","err","timeout","ir","reused","aged","missing"):
                    if intval(x,key) < intval(previous,key):
                        errors.append(f"Integrated {key} counter decreased")
            previous = x
        if len(sensor_rows) >= 2:
            if intval(sensor_rows[-1],"fresh") <= intval(sensor_rows[0],"fresh"):
                errors.append("No new gyro samples were collected during combined run")
            if intval(sensor_rows[-1],"ir") <= intval(sensor_rows[0],"ir"):
                errors.append("IR scanning did not progress during combined run")
    elif sensor_rows:
        errors.append("Unexpected integrated sensor telemetry in motor-only image")
    if len(begins) != 1:
        errors.append("Expected exactly one host-triggered MOTOR_AUTO_BEGIN")
    if begins and (intval(begins[-1], "rounds") != NUM_ROUNDS or
                   intval(begins[-1], "steps") != len(MATRIX) or
                   intval(begins[-1], "boot_neutral") != 1):
        errors.append("Host START acknowledgment has incompatible run profile")
    if len(starts) != EXPECTED or len(ends) != EXPECTED or len(posts) != EXPECTED:
        errors.append(f"Missing/repeated stages (start/end/post counts): {len(starts)}/{len(ends)}/{len(posts)} expected {EXPECTED}")
    if len(r_ends) != NUM_ROUNDS:
        errors.append("Missing/repeated per-round completion signatures")
    if len(links) != len(posts):
        errors.append(f"Heartbeat/RX telemetry missing: link={len(links)}, post={len(posts)}")
    if links:
        counts = [intval(x, "keep_ok") for x in links]
        if counts != sorted(counts):
            errors.append("Firmware KEEP count decreased during the run")
        if len(starts) == EXPECTED and (counts[0] < 1 or counts[-1] < 10):
            errors.append("Insufficient decoded host heartbeats for complete experiment")
        if any(intval(x,"rx_ore") > 0 or intval(x,"rx_fe") > 0 or
               intval(x,"rx_ring_drop") > 0 for x in links):
            errors.append("USART1 RX hardware/ring reported dropped or invalid bytes")
    if len(done) != 1 or (done and intval(done[-1], "pulses") != EXPECTED):
        errors.append("Missing/invalid full experiment completion record")
    if faults:
        errors.append("Firmware reported MOTOR_AUTO_FAULT: " + ",".join(x.get("reason", "?") for x in faults))

    rows = []
    individual = []
    for i, s in enumerate(samples):
        try:
            row = {k: str(s[k]) for k in CSV_FIELDS}
            for k in CSV_FIELDS:
                if k not in ("phase", "test"):
                    int(row[k])
            rows.append(row)
        except (KeyError, ValueError):
            errors.append(f"Malformed MOTOR_AUTO_SAMPLE at row {i}")
    seen = [(intval(x, "round"), intval(x, "stage"), intval(x, "l"), intval(x, "r"))
            for x in starts]
    expected = [(r, st, *pair) for r in range(NUM_ROUNDS)
                for st, pair in enumerate(MATRIX)]
    if seen != expected:
        errors.append("Start order or command matrix differs from three expected rounds")
    for x in starts:
        stage = intval(x, "stage")
        if not (0 <= stage < len(MATRIX)):
            errors.append("Invalid MCU START stage index")
        elif x.get("test") != TESTS[stage] or intval(x, "rise") != RISE_RATES[stage]:
            errors.append(f"Diagnostic test/slew mismatch in MCU START stage {stage}")
    if [(intval(x, "round"), intval(x, "stage")) for x in ends] != [x[:2] for x in expected]:
        errors.append("END order differs from start matrix")
    if [(intval(x, "round"), intval(x, "stage")) for x in posts] != [x[:2] for x in expected]:
        errors.append("POST_END order differs from start matrix")
    if [intval(x,"round") for x in r_ends] != [1,2,3]:
        errors.append("Round end ordering mismatch")
    for i, end in enumerate(ends):
        expected_ms = stage_pulse_ms(intval(end,"stage"))
        if intval(end, "reason") != 0 or intval(end, "elapsed") != expected_ms:
            errors.append(f"Pulse #{i} failed configured {expected_ms}ms timeout")
        if intval(end, "ccr_l") != STOP_CCR or intval(end, "ccr_r") != STOP_CCR:
            errors.append(f"Pulse #{i} STOP CCR disagrees with protocol (expected {STOP_CCR})")
    for i, p in enumerate(posts):
        if intval(p, "elapsed") != stage_post_ms(intval(p,"stage")) or intval(p, "ccr_l") != STOP_CCR or intval(p, "ccr_r") != STOP_CCR:
            errors.append(f"Post-stop observation #{i} invalid")
    for i, s in enumerate(samples):
        if s.get("phase") not in ("drive", "post"):
            errors.append(f"Unknown sample phase {i}")
        stage = intval(s, "stage")
        if not (0 <= stage < len(MATRIX)) or \
           s.get("test") != TESTS[stage] or intval(s, "rise") != RISE_RATES[stage]:
            errors.append(f"Invalid test/slew telemetry at sample {i}")
        if intval(s, "tim11_gaps") != 0 or intval(s, "tim11_late") != 0:
            errors.append("TIM11 deadline/jitter issue")
        if intval(s, "tx_drop") != 0:
            errors.append("UART DMA message loss")
        for k in ("applied_l", "applied_r"):
            if abs(intval(s, k)) > MAX_CMD:
                errors.append("Applied PWM exceeded hard cap")
        for wheel in ("l", "r"):
            applied = intval(s, f"applied_{wheel}")
            ccr = intval(s, f"ccr_{wheel}")
            if applied == 0:
                if ccr != STOP_CCR:
                    errors.append(f"Zero-applied wheel {wheel} STOP CCR mismatch")
            elif not (0 < ccr <= MAX_CCR):
                errors.append(f"Active wheel {wheel} CCR outside capped drive duty")
        if s.get("phase") == "post" and any(
                intval(s, f"applied_{wheel}") != 0 or
                intval(s, f"ccr_{wheel}") != STOP_CCR
                for wheel in ("l", "r")):
            errors.append("Nonneutral wheel state during post-stop observation")

    for r in range(NUM_ROUNDS):
        for stage, cmd in enumerate(MATRIX):
            key = (str(r), str(stage))
            drives = [x for x in samples if (x.get("round"), x.get("stage")) == key
                      and x.get("phase") == "drive" and intval(x,"elapsed") < stage_pulse_ms(stage)]
            after = [x for x in samples if (x.get("round"), x.get("stage")) == key
                     and x.get("phase") == "post"]
            e = next((x for x in ends if (x.get("round"),x.get("stage")) == key), {})
            p = next((x for x in posts if (x.get("round"),x.get("stage")) == key), {})
            if not drives and not after and not e and not p:
                # Never fabricate 0-count measurements for unexecuted stages
                # when HOST_TIMEOUT or another fail-safe aborts the sequence.
                continue
            if (len(drives) < (4 if stage >= BASE_STEPS else 6) or
                len(after) < (30 if stage >= BASE_STEPS else 10)):
                errors.append(f"Insufficient 20ms samples round={r} stage={stage} drive={len(drives)} post={len(after)}")
            if any(intval(x, "ccr_l") != STOP_CCR for x in drives if cmd[0] == 0) or \
               any(intval(x, "ccr_r") != STOP_CCR for x in drives if cmd[1] == 0):
                errors.append(f"Noncommanded wheel STOP CCR mismatch r={r} stage={stage}")
            if drives and any(abs(intval(x,"applied_l")) > cmd[0] or
                              abs(intval(x,"applied_r")) > cmd[1] for x in drives):
                errors.append(f"Requested PWM cap exceeded r={r} stage={stage}")
            if cmd[0] and cmd[1] and (cmd[0] > DUAL_MAX_CMD or cmd[1] > DUAL_MAX_CMD):
                errors.append(f"Dual-wheel duty beyond allowed cap r={r} stage={stage}")
            # Compare actual timer CCR peak against commanded levels. A zero
            # or truncated PWM implementation must not pass just because
            # all 21/27 START/END records exist.
            if drives:
                for wheel, duty in (("l", cmd[0]), ("r", cmd[1])):
                    peak = max(intval(x, f"ccr_{wheel}") for x in drives)
                    if duty > 0 and peak != duty * 3200 // 1000:
                        errors.append(f"PWM CCR mismatch r={r} stage={stage} wheel={wheel}: peak={peak}")
            driven_counts = []
            for wheel, request in (("l", cmd[0]), ("r", cmd[1])):
                # Absence of END or POST_END is missing data, never a zero
                # count, zero stop-distance or proof of no motor response.
                final = intval(e, f"enc_{wheel}", None)
                after_count = intval(p, f"enc_{wheel}", None)
                # Short 120ms high-duty pulses have only ~5 pre-stop
                # samples; choose last-half window rather than >=99ms.
                tail_start = (stage_pulse_ms(stage)//2
                              if stage >= BASE_STEPS else 99)
                tail = [x for x in drives if intval(x,"elapsed") >= tail_start]
                tail_delta = (intval(tail[-1],f"enc_{wheel}",0) -
                              intval(tail[0],f"enc_{wheel}",0)) if len(tail)>1 else None
                sustained = request > 0 and final is not None and abs(final) >= 5 and \
                            (tail_delta is not None and abs(tail_delta) >= 5)
                first5 = next((intval(x,"elapsed") for x in drives
                               if abs(intval(x,f"enc_{wheel}",0)) >= 5), None)
                post_delta = ((after_count-final) if
                              after_count is not None and final is not None else None)
                # Post-stop response is a position trajectory, NOT a brake
                # current measurement or exact instant of mechanical rest.
                end_window = stage_post_ms(stage)
                neutral_tail = [x for x in after if
                                intval(x,"elapsed") >= end_window-100]
                post_last100_net = (
                    intval(neutral_tail[-1],f"enc_{wheel}") -
                    intval(neutral_tail[0],f"enc_{wheel}")
                    if len(neutral_tail) >= 2 else None)
                def post_reached(frac):
                    if post_delta is None or post_delta <= 5:
                        return None
                    goal = final + post_delta*frac
                    return next((intval(x,"elapsed") for x in after
                                 if intval(x,f"enc_{wheel}") >= goal),None)
                driven_counts.append({
                    "wheel":wheel, "requested_permille": request, "encoder_drive": final,
                    "encoder_after_post_ms": after_count,
                    "post_stop_delta": post_delta,
                    "post_t95_ms": post_reached(0.95),
                    "post_t99_ms": post_reached(0.99),
                    "post_last100ms_net_counts": post_last100_net,
                    "final_100ms_delta":tail_delta, "first_five_counts_ms":first5,
                    "sustained_count_signal": bool(sustained),
                })
            individual.append({
                "round":r, "stage":stage,"test":TESTS[stage],
                "rise_rate_permille_per_ms":RISE_RATES[stage],
                "cmd_left":cmd[0],"cmd_right":cmd[1],
                "drive_samples":len(drives),"post_samples":len(after),
                "wheel_readings":driven_counts,
            })
    stats = []
    for stage, cmd in enumerate(MATRIX):
        for wheel, duty in (("l",cmd[0]),("r",cmd[1])):
            if duty == 0: continue
            vals = [next((x for x in trial["wheel_readings"]
                          if x["wheel"]==wheel), None)
                    for trial in individual if trial["stage"]==stage]
            vals = [x for x in vals if x is not None]
            counts = [x["encoder_drive"] for x in vals if x["encoder_drive"] is not None]
            stats.append({
                "stage":stage, "test":TESTS[stage],
                "rise_rate_permille_per_ms":RISE_RATES[stage],
                "wheel":wheel, "cmd_permille":duty,
                "three_drive_counts":counts,
                "executed_replicates":len(vals),
                "measurement_status":"MEASURED" if len(vals)==3 else
                     ("PARTIAL" if vals else "NOT_TESTED"),
                "mean_count":round(statistics.mean(counts),2) if counts else None,
                "sample_std_count":round(statistics.stdev(counts),2) if len(counts)>1 else None,
                "sustained_3of3":all(x["sustained_count_signal"] for x in vals) and len(vals)==3,
                "sustained_replicates":sum(x["sustained_count_signal"] for x in vals),
                "three_post_stop_deltas":[x["post_stop_delta"] for x in vals],
                "complete_post_stop_replicates":sum(x["post_stop_delta"] is not None for x in vals),
                "post_t95_ms":[x["post_t95_ms"] for x in vals],
                "post_t99_ms":[x["post_t99_ms"] for x in vals],
                "post_last100ms_net_counts":[x["post_last100ms_net_counts"] for x in vals],
                "first_five_counts_ms":[x["first_five_counts_ms"] for x in vals]
            })
    if done and (intval(done[-1],"ccr_l") != STOP_CCR or intval(done[-1],"ccr_r") != STOP_CCR or
                 intval(done[-1],"tim11_gaps") != 0 or intval(done[-1],"tx_drop") != 0 or
                 intval(done[-1],"rx_ore") != 0 or intval(done[-1],"rx_ring_drop") != 0):
        errors.append("Nonneutral/invalid final MCU status")
    errors = sorted(set(errors))
    return {
        "status": "TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" if not errors else "INCOMPLETE",
        "rounds_requested": NUM_ROUNDS, "pulses_expected": EXPECTED,
        "not_executed_pulses":max(0,EXPECTED-len(starts)),
        "starts":len(starts), "ends":len(ends), "post_ends":len(posts),
        "sample_count":len(samples),"errors":errors,
        "rx_heartbeat_diagnostics":{
            "link_records":len(links),
            "keep_decoded_last":intval(links[-1],"keep_ok",0) if links else None,
            "max_uart_rx_overrun":max((intval(x,"rx_ore",0) for x in links), default=None),
            "max_uart_rx_framing":max((intval(x,"rx_fe",0) for x in links), default=None),
            "max_uart_rx_ring_drop":max((intval(x,"rx_ring_drop",0) for x in links), default=None),
            "last_firmware_reported_host_age_ms":intval(links[-1],"host_age_ms",0) if links else None,
            "faults":faults,
        },
        "test_families":["startup","response","slew_slow","slew_fast"]+
                        (["high"] if HIGH_PROFILE else []),
        "profile_version":PROFILE_VERSION, "high_profile":HIGH_PROFILE,
        "sensor_profile":SENSOR_PROFILE,
        "stop_register_snapshot":{
            "records":len(static_regs),
            "last":static_regs[-1] if static_regs else None,
            "note":"GPIO ODR/CCR != measured TB6612 output or brake current"
        },
        "sensor_integration":{
            "sensor_ready_records":len(sensor_ready),
            "sensor_phase_records":len(sensor_rows),
            "sensor_first":sensor_rows[0] if sensor_rows else None,
            "sensor_last":sensor_rows[-1] if sensor_rows else None,
            "gyro_fresh_delta":(intval(sensor_rows[-1],"fresh")-
                                intval(sensor_rows[0],"fresh")) if len(sensor_rows)>=2 else None,
            "ir_scans_delta":(intval(sensor_rows[-1],"ir")-
                              intval(sensor_rows[0],"ir")) if len(sensor_rows)>=2 else None,
            "i2c_error_delta":(intval(sensor_rows[-1],"err")-
                               intval(sensor_rows[0],"err")) if len(sensor_rows)>=2 else None,
            "i2c_timeout_delta":(intval(sensor_rows[-1],"timeout")-
                                 intval(sensor_rows[0],"timeout")) if len(sensor_rows)>=2 else None,
            # Timing/IMU counters also run while the stationary MPU is
            # calibrating before READY. Assess the CHANGE during motor
            # stages, never reject because an accumulated baseline != 0.
            "missing_before_motor_phase":intval(sensor_rows[0],"missing") if sensor_rows else None,
            "missing_delta":(intval(sensor_rows[-1],"missing")-
                             intval(sensor_rows[0],"missing")) if len(sensor_rows)>=2 else None,
            "reused_delta":(intval(sensor_rows[-1],"reused")-
                            intval(sensor_rows[0],"reused")) if len(sensor_rows)>=2 else None,
            "aged_over1500_delta":(intval(sensor_rows[-1],"aged")-
                                   intval(sensor_rows[0],"aged")) if len(sensor_rows)>=2 else None,
            "reused_per_1000_fresh":round(
                1000.0*(intval(sensor_rows[-1],"reused")-intval(sensor_rows[0],"reused"))/
                (intval(sensor_rows[-1],"fresh")-intval(sensor_rows[0],"fresh")),3)
                if len(sensor_rows)>=2 and intval(sensor_rows[-1],"fresh")>intval(sensor_rows[0],"fresh") else None,
            "gyro_abs_avg_drive_mdps":round(statistics.mean(
                [intval(x,"gyro_abs_avg") for x in sensor_rows if x.get("phase")=="drive"]),2)
                if any(x.get("phase")=="drive" for x in sensor_rows) else None,
            "gyro_abs_avg_post_mdps":round(statistics.mean(
                [intval(x,"gyro_abs_avg") for x in sensor_rows if x.get("phase")=="post"]),2)
                if any(x.get("phase")=="post" for x in sensor_rows) else None,
            "gyro_abs_peak_drive_mdps":max(
                (intval(x,"gyro_abs_peak") for x in sensor_rows if x.get("phase")=="drive"),
                default=None),
            "gyro_interpretation":"20ms sampling of gyro angular rate includes real mechanical vibration; a difference between PWM-on and neutral is not by itself proof of electrical EMI.",
        },
        "stages_per_round":len(MATRIX),"post_stop_observation_ms":POST_MS,
        "repeat_statistics":stats,"trials":individual,
        "phase_findings":phase_findings(stats),
        "timing_integrity":{
            "max_tim11_dwt_cycles_seen": max((intval(x,"tim11_max_cycles",0) for x in samples), default=0),
            "max_observed_tim11_late_ticks": max((intval(x,"tim11_late",0) for x in samples), default=0),
            "max_observed_tim11_gaps": max((intval(x,"tim11_gaps",0) for x in samples), default=0),
            "max_observed_uart_tx_drop": max((intval(x,"tx_drop",0) for x in samples), default=0),
        },
        "experiment_setup":setup or {},
        "electrical_limitation":"No current, stall-current, thermal, oscilloscope or physical output-mode measurement. STM32 GPIO=3.3V vs TB6612 VCC=5V outside VIH guarantee; STBY fixed high. 60pct command on high profile is NOT a current limit or electrical safety approval.",
        "interpretation":"Sustained_count_signal is an empirical >=5 encoder-count and >=5 tail-count heuristic, NOT a certified deadband threshold.",
    }, rows


def test_firmware_uart_profile_frame():
    """Source-based guard for C UART_Printf's 256-byte record limit.

    This specifically reproduces the v5 bug: an overlong PROFILE frame lost
    CRLF, merging it with the following READY and blocking host handshake.
    """
    source = (Path(__file__).resolve().parent.parent /
              "src" / "motor_auto_bench.c").read_text(encoding="utf-8")
    section = source.split("static void ReplyReady(void)",1)[1].split(
        "static void ReceiveLine(",1)[0]
    match = re.search(r'UART_Printf\(\s*((?:"(?:\\.|[^"\\])*"\s*)+),',
                      section, re.S)
    assert match, "Missing firmware PROFILE UART_Printf frame"
    literals = re.findall(r'"((?:\\.|[^"\\])*)"', match.group(1))
    fmt = "".join(bytes(chunk,"utf-8").decode("unicode_escape")
                  for chunk in literals)
    profile = fmt % (PROFILE_VERSION, 96000000, 30000, NUM_ROUNDS,
                     len(MATRIX), MAX_CMD, FAMILIES,
                     HIGH_PULSE_MS if HIGH_PROFILE else 0,
                     HIGH_POST_MS if HIGH_PROFILE else 0,
                     int(SENSOR_PROFILE))
    assert profile.startswith("MOTOR_AUTO_PROFILE,")
    assert profile.endswith("\r\n"), "Firmware PROFILE must end in CRLF"
    assert len(profile.encode("ascii")) < 256, (
        f"Firmware PROFILE would truncate at UART_Printf 256B: "
        f"{len(profile.encode('ascii'))} bytes")
    print(f"UART framing guard: v{PROFILE_VERSION} PROFILE "
          f"{len(profile.encode('ascii'))}/255 bytes; CRLF intact.")


def self_test():
    test_firmware_uart_profile_frame()
    synthetic = [
        "[BOOT] Motor auto baseline: neutral, USART1 host ARM required",
        f"MOTOR_AUTO_PROFILE,version={PROFILE_VERSION},nominal_hz=96000000,pwm_hz=30000,rounds=3,steps={len(MATRIX)},hard_cap_permille={MAX_CMD},both_cap_permille=160,families={FAMILIES},rx_mode=irq256,high_pulse_ms={HIGH_PULSE_MS if HIGH_PROFILE else 0},high_post_ms={HIGH_POST_MS if HIGH_PROFILE else 0},pulse_ms=200,post_ms=400,boot_neutral=1,sensor={int(SENSOR_PROFILE)}",
        f"MOTOR_AUTO_READY,version={PROFILE_VERSION},rounds=3,steps={len(MATRIX)},hard_cap_permille={MAX_CMD},both_cap_permille=160,families={FAMILIES},rx_mode=irq256,pulse_ms=200,post_ms=400,one_shot=1,boot_neutral=1,sensor={int(SENSOR_PROFILE)}",
        f"MOTOR_AUTO_BEGIN,rounds=3,steps={len(MATRIX)},settle_ms=1000,boot_neutral=1"
    ]
    synthetic.insert(2,f"MOTOR_AUTO_STATIC,ccr_l={STOP_CCR},ccr_r={STOP_CCR},in_l1=0,in_l2=0,in_r1=0,in_r2=0,oc1pe=1,oc2pe=1,arr=3199")
    if SENSOR_PROFILE:
        synthetic.append("MOTOR_AUTO_SENSOR_READY,good=9300,ir=100,bias_mdps=0")
    for r in range(NUM_ROUNDS):
        for i,(left,right) in enumerate(MATRIX):
            test, rise = TESTS[i], RISE_RATES[i]
            pulse, post = stage_pulse_ms(i), stage_post_ms(i)
            synthetic.append(f"MOTOR_AUTO_START,round={r},stage={i},l={left},r={right},test={test},rise={rise},pulse_ms={pulse}")
            for n in range(1,pulse//20+1):
                t=n*20-1
                synthetic.append(f"MOTOR_AUTO_SAMPLE,round={r},stage={i},phase=drive,test={test},rise={rise},ms={t},elapsed={t},applied_l={left},applied_r={right},enc_l={int(left>0)*n*5},enc_r={int(right>0)*n*5},ccr_l={left*32//10 if left else STOP_CCR},ccr_r={right*32//10 if right else STOP_CCR},tim11_late=0,tim11_max_cycles=1000,tim11_gaps=0,tx_drop=0")
            if SENSOR_PROFILE:
                n = r*len(MATRIX)+i
                synthetic.append(f"MOTOR_AUTO_SENSORS,round={r},stage={i},phase=drive,good={9300+n*50},fresh={9250+n*50},err=0,timeout=0,busy=2,reused=10,aged=2,missing=0,ir={100+n*30},gyro_n=6,gyro_abs_avg=200,gyro_abs_peak=350")
            synthetic.append(f"MOTOR_AUTO_END,round={r},stage={i},elapsed={pulse},reason=0,enc_l={int(left>0)*50},enc_r={int(right>0)*50},ccr_l={STOP_CCR},ccr_r={STOP_CCR}")
            for n in range(1,post//20+1):
                t=n*20
                synthetic.append(f"MOTOR_AUTO_SAMPLE,round={r},stage={i},phase=post,test={test},rise={rise},ms={t},elapsed={t},applied_l=0,applied_r=0,enc_l={int(left>0)*50},enc_r={int(right>0)*50},ccr_l={STOP_CCR},ccr_r={STOP_CCR},tim11_late=0,tim11_max_cycles=1000,tim11_gaps=0,tx_drop=0")
            if SENSOR_PROFILE:
                n = r*len(MATRIX)+i
                synthetic.append(f"MOTOR_AUTO_SENSORS,round={r},stage={i},phase=post,good={9350+n*50},fresh={9300+n*50},err=0,timeout=0,busy=2,reused=10,aged=2,missing=0,ir={120+n*30},gyro_n=20,gyro_abs_avg=100,gyro_abs_peak=220")
            synthetic.append(f"MOTOR_AUTO_POST_END,round={r},stage={i},elapsed={post},enc_l={int(left>0)*50},enc_r={int(right>0)*50},ccr_l={STOP_CCR},ccr_r={STOP_CCR}")
            synthetic.append(f"MOTOR_AUTO_LINK,round={r},stage={i},keep_ok={1+r*len(MATRIX)+i},host_age_ms=100,rx_bytes={50+r*len(MATRIX)+i},rx_ore=0,rx_fe=0,rx_noise=0,rx_ring_drop=0,bad_commands=0")
        synthetic.append(f"MOTOR_AUTO_ROUND_END,round={r+1},completed={(r+1)*len(MATRIX)}")
    synthetic.append(f"MOTOR_AUTO_DONE,rounds=3,pulses={EXPECTED},ccr_l={STOP_CCR},ccr_r={STOP_CCR},tim11_gaps=0,tx_drop=0,keep_ok={EXPECTED},rx_ore=0,rx_ring_drop=0")
    report,_ = analyze(synthetic)
    assert report["status"]=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL", report["errors"]
    wrong_stop = 0 if STOP_CCR == 3200 else 3200
    wrong_mode = [
        x.replace(f"ccr_l={STOP_CCR},ccr_r={STOP_CCR},in_l1",
                  f"ccr_l={wrong_stop},ccr_r={wrong_stop},in_l1")
        if x.startswith("MOTOR_AUTO_STATIC") else x for x in synthetic
    ]
    wrong_report, _ = analyze(wrong_mode)
    assert wrong_report["status"] == "INCOMPLETE"
    if SENSOR_PROFILE:
        assert report["sensor_integration"]["gyro_fresh_delta"]>0
        assert report["sensor_integration"]["ir_scans_delta"]>0
        assert report["sensor_integration"]["missing_delta"]==0
        assert report["sensor_integration"]["missing_before_motor_phase"]==0
        assert report["sensor_integration"]["reused_delta"]==0
        assert report["sensor_integration"]["aged_over1500_delta"]==0
    assert len(report["repeat_statistics"])==(30 if HIGH_PROFILE else 22)
    # Regression from powered v6 dataset: left 8% works 3/3, 9% fails
    # 0/3, 10% works 3/3. "lowest successful" 8% is NOT a robust
    # deadband threshold in this nonmonotonic situation.
    test_stats = [
        {"test":"startup", "wheel":"l", "cmd_permille":duty,
         "sustained_3of3":ok, "sustained_replicates":3 if ok else 0,
         "executed_replicates":3}
        for duty,ok in ((80,True),(90,False),(100,True),(110,True),(120,True))
    ]
    threshold = phase_findings(test_stats)["startup_threshold_heuristic"]["l"]
    assert threshold["lowest_tested_3of3_permille"] == 80
    assert threshold["lowest_monotonic_suffix_3of3_permille"] == 100
    assert threshold["nonmonotonic_success_decreases"][0]["higher_permille"] == 90
    def incomplete(lines):
        x,_=analyze(lines)
        assert x["status"]=="INCOMPLETE", x
    incomplete([x for x in synthetic if not x.startswith("MOTOR_AUTO_PROFILE")])
    incomplete([x for x in synthetic if "MOTOR_AUTO_END,round=1,stage=3" not in x])
    incomplete(synthetic + ["MOTOR_AUTO_FAULT,reason=HOST_TIMEOUT"])
    # v5 regression: a truncated PROFILE without CRLF merged into
    # MOTOR_AUTO_READY, so a correct host must reject handshake.
    merged = list(synthetic)
    merged[1] = merged[1].removesuffix("boot_neutral=1") + (
        "stby_fixed_MOTOR_AUTO_READY,version=5,rounds=3")
    merged.pop(2)
    incomplete(merged)
    interrupted_post = [x for x in synthetic if
                        "MOTOR_AUTO_POST_END,round=0,stage=1" not in x]
    partial_report,_ = analyze(interrupted_post)
    assert partial_report["status"] == "INCOMPLETE"
    st1 = next(x for x in partial_report["trials"]
               if x["round"]==0 and x["stage"]==1)
    assert st1["wheel_readings"][1]["post_stop_delta"] is None
    incomplete([x.replace("rx_ore=0,rx_fe=0","rx_ore=1,rx_fe=0")
                if x.startswith("MOTOR_AUTO_LINK") else x for x in synthetic])
    aborted = [x for x in synthetic if not ("round=1,stage=" in x or
               "round=2,stage=" in x)]
    partial,_ = analyze(aborted)
    assert partial["status"] == "INCOMPLETE"
    assert partial["repeat_statistics"][0]["measurement_status"] == "PARTIAL"
    assert partial["repeat_statistics"][0]["three_drive_counts"] == [50]
    assert partial["repeat_statistics"][3]["measurement_status"] == "PARTIAL"
    assert all(x["mean_count"] is None and x["three_drive_counts"] == []
               for x in partial["repeat_statistics"]
               if x["test"] == "slew_fast" and x["measurement_status"] == "NOT_TESTED")
    incomplete([x.replace("in_l1=0,in_l2=0","in_l1=1,in_l2=0")
                if x.startswith("MOTOR_AUTO_STATIC") else x for x in synthetic])
    incomplete([x.replace(f"ccr_l={STOP_CCR},ccr_r={STOP_CCR},tim11_late=0", f"ccr_l=384,ccr_r={STOP_CCR},tim11_late=0")
                if "phase=post" in x else x for x in synthetic])
    if HIGH_PROFILE and not SENSOR_PROFILE:
        archived = ROOT / "motor_logs" / "auto_20261013_105353" / "raw_serial.log"
        if archived.is_file():
            old_raw = archived.read_text(encoding="utf-8").splitlines()
            configure_from_saved_log(old_raw, high=True, sensors=False)
            assert PROFILE_VERSION == 6 and STOP_CCR == 0
            old_report, _ = analyze(old_raw)
            assert old_report["status"] == "TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL", old_report["errors"]
            assert old_report["starts"] == 87 and old_report["ends"] == 87
            configure_profile(True, False)
            assert PROFILE_VERSION == 16 and STOP_CCR == 3200
            print("PASS: archived v6 87-stage CCR=0 data remains valid; v16 STOP_HIZ is separate.")
    if SENSOR_PROFILE and not HIGH_PROFILE:
        archived = ROOT / "motor_logs" / "auto_20261013_113713" / "raw_serial.log"
        if archived.is_file():
            old_raw = archived.read_text(encoding="utf-8").splitlines()
            configure_from_saved_log(old_raw, high=False, sensors=True)
            assert PROFILE_VERSION == 7 and STOP_CCR == 0
            old_report, _ = analyze(old_raw)
            assert old_report["status"] == "TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL", old_report["errors"]
            assert old_report["starts"] == 63 and old_report["ends"] == 63
            configure_profile(False, True)
            assert PROFILE_VERSION == 17 and STOP_CCR == 3200
            print("PASS: archived v7 63-stage CCR=0 data remains valid; v17 STOP_HIZ is separate.")
    print(f"SELF-TEST PASSED: {NUM_ROUNDS}-round {EXPECTED}-pulse four-phase capture; RX fault and incomplete stage measurements are correctly classified.")
    print("Host synthetic test only; not hardware validation.")


def store(folder, raw, setup):
    folder.mkdir(parents=True,exist_ok=True)
    (folder/"raw_serial.log").write_text("\n".join(raw)+"\n",encoding="utf-8")
    (folder/"session_meta.json").write_text(
        json.dumps(setup,indent=2)+"\n",encoding="utf-8")
    report,rows=analyze(raw,setup=setup)
    with (folder/"motor_auto_samples.csv").open("w",newline="",encoding="utf-8-sig") as fp:
        writer=csv.DictWriter(fp,fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    (folder/"motor_auto_report.json").write_text(
        json.dumps(report,indent=2)+"\n",encoding="utf-8")
    lines=["Motor auto 3-round diagnostic",f"Status: {report['status']}",
           f"Stages: {report['starts']}/{EXPECTED}; samples: {report['sample_count']}",
           "Encoder counts are not RPM. Electrical current/thermal limits were NOT measured.",
           ""]
    findings = report["phase_findings"]
    lines.append("PHASE A: startup 8-12% (3/3 sustained encoder count heuristic)")
    for wheel in ("l","r"):
        th = findings["startup_threshold_heuristic"][wheel]
        lines.append(f"  {wheel}: confirmed-per-duty={th['sustained_replicates_by_duty']}; "
                     f"naive-lowest-3of3={th['lowest_tested_3of3_permille']}; "
                     f"monotonic-suffix-candidate={th['lowest_monotonic_suffix_3of3_permille']}; "
                     f"nonmonotonic={th['nonmonotonic_success_decreases']}")
    lines.append("PHASE B: PWM response and post-zero encoder movement")
    for x in findings["open_loop_response"]:
        lines.append(f"  {x['pwm_permille']} permille: L/R={x['mean_left_encoder_count']}/{x['mean_right_encoder_count']}, "
                     f"delta={x['relative_count_difference_pct']}%, postzero-L/R={x['mean_left_post_stop_counts']}/{x['mean_right_post_stop_counts']}")
    lines.append("PHASE C: rise slew 5 vs 20 permille/ms at 16%")
    for x in findings["rise_slew_comparison"]:
        lines.append(f"  {x['wheel']}: slow={x['mean_count_slow']}, fast={x['mean_count_fast']}, "
                     f"fast-minus-slow={x['mean_count_fast_minus_slow']} counts")
    lines.append("PHASE D: executed-condition 400ms base / 800ms high neutral observations (details below)")
    if HIGH_PROFILE:
        lines.append("PHASE E: opt-in PWM 30/40/50/60% for each wheel; 120ms drive/800ms neutral, 3 rounds")
        high = [x for x in report["repeat_statistics"] if x["test"] == "high"]
        for x in high:
            lines.append(f"  {x['wheel']} duty={x['cmd_permille']/10:.0f}% counts={x['three_drive_counts']}, "
                         f"post-neutral={x['three_post_stop_deltas']}, post-t95-ms={x['post_t95_ms']}, "
                         f"post-t99-ms={x['post_t99_ms']}, last100ms-net={x['post_last100ms_net_counts']}, "
                         f"repeats={x['executed_replicates']}")
    lines.append(f"NOT_EXECUTED: {report['not_executed_pulses']} of {EXPECTED} pulses; missing stages are excluded, not treated as 0 counts")
    lines.append("Host/Microcontroller link: " + json.dumps(report["rx_heartbeat_diagnostics"]))
    if SENSOR_PROFILE:
        lines.append("IMU/I2C/IR combined telemetry: " + json.dumps(report["sensor_integration"]))
    for x in report["repeat_statistics"]:
        if x["measurement_status"] == "NOT_TESTED":
            continue
        lines.append(f"[{x['test']}] Stage {x['stage']} {x['wheel']} {x['cmd_permille']}permille, rise={x['rise_rate_permille_per_ms']}: "
                     f"counts={x['three_drive_counts']}, mean={x['mean_count']}, "
                     f"std={x['sample_std_count']}, sustained={x['sustained_replicates']}/3, "
                     f"post-stop-delta={x['three_post_stop_deltas']}")
    if report["errors"]:
        lines.extend(["","ERRORS:"]+report["errors"])
    (folder/"motor_auto_summary.txt").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print("\n".join(lines))
    print("Output:",folder.resolve())
    return report


def execute(args):
    if args.vm_state != "powered":
        raise SystemExit("--run requires explicit --vm-state powered. For unpowered host protocol validation use --check-link.")
    if args.meter_vm_volts is not None and not (0 <= args.meter_vm_volts <= 12):
        raise SystemExit("Measured VM DC voltage must be in the range 0..12V")
    # One-command gate: verify the selected v4/v5 host parser (including
    # missing/faulted traces) BEFORE flashing/arming a motor.
    self_test()
    folder=Path("motor_logs")/("auto_"+dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    setup={"vm_state_declared":args.vm_state,
           "dc_vm_volts_hand_measured":args.meter_vm_volts,
           "firmware_env":FIRMWARE_ENV,"host_control":"USART1 RX PA10",
           "rounds":NUM_ROUNDS,"stages_per_round":len(MATRIX),
           "diagnostic_phases":["startup","response","slew_slow","slew_fast"]+
                                  (["high"] if HIGH_PROFILE else []),
           "high_profile": HIGH_PROFILE, "sensor_profile": SENSOR_PROFILE,
           "note":"VM state user-supplied, not measured by STM32; duty is not current limit."}
    if not args.skip_flash:
        print("Build + flash opt-in firmware via STLINK:", FIRMWARE_ENV,flush=True)
        subprocess.run([sys.executable,"-m","platformio","run","-e",FIRMWARE_ENV,"-t","upload"],check=True)
    else:
        print("SKIPPING FLASH: firmware/profile must already match; the handshake will verify profile.")
    try:
        import serial
    except ImportError as e:
        raise SystemExit("Install dependency: python -m pip install pyserial") from e
    raw=[]
    keep_sent=0
    began=False
    complete=False
    requested_start=False
    verified_profile=False
    handshake_error=None
    try:
        with serial.Serial(args.port,115200,timeout=0.12,write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            start=time.monotonic()
            ping_at=0.0
            keep_at=0.0
            handshake_deadline=start+(60.0 if SENSOR_PROFILE else 10.0)
            print("Connected:",args.port,"- host must receive MOTOR_AUTO_READY before motion.")
            print("Wheels MUST be elevated; have a physical VM disconnect ready. STOP via UART is not a hardware kill switch.")
            while time.monotonic()-start<args.timeout:
                now=time.monotonic()
                if not requested_start and now>=ping_at:
                    ser.write(b"MOTOR PING\n")
                    ping_at=now+0.4
                if requested_start and now>=keep_at and not complete:
                    ser.write(b"MOTOR KEEP\n")
                    keep_sent += 1
                    keep_at=now+0.25
                if not requested_start and now>handshake_deadline:
                    handshake_error = (
                        "No complete/valid PROFILE + READY before calibration/handshake timeout. "
                        "Inspect raw_serial.log for truncated or merged records; "
                        "check USB-UART wiring only if no MCU bytes are received.")
                    print("HANDSHAKE FAILED:",handshake_error,
                          "No START was sent.")
                    break
                packet=ser.readline()
                if not packet:
                    continue
                frame=packet.decode("utf-8",errors="replace").strip()
                raw.append(f"{now-start:.6f}\t{frame}")
                kind,v=split_record(frame)
                if not requested_start and "MOTOR_AUTO_READY" in frame and kind != "MOTOR_AUTO_READY":
                    handshake_error = "UART framing corruption: READY merged into another record (old v5 PROFILE truncation)."
                    print("HANDSHAKE FAILED:",handshake_error)
                    break
                if kind=="MOTOR_AUTO_PROFILE" and not requested_start:
                    profile_expected = (
                        ("version",PROFILE_VERSION),("rounds",NUM_ROUNDS),
                        ("steps",len(MATRIX)),("hard_cap_permille",MAX_CMD),
                        ("both_cap_permille",DUAL_MAX_CMD),("families",FAMILIES),
                        ("pulse_ms",PULSE_MS),("post_ms",POST_MS),
                        ("boot_neutral",1),("pwm_hz",30000),
                        ("sensor",int(SENSOR_PROFILE)),
                        ("high_pulse_ms",HIGH_PULSE_MS if HIGH_PROFILE else 0),
                        ("high_post_ms",HIGH_POST_MS if HIGH_PROFILE else 0),
                    )
                    if any(intval(v,k)!=want for k,want in profile_expected) or v.get("rx_mode")!="irq256":
                        handshake_error = "Firmware PROFILE fields/version do not match selected host mode."
                        print("HANDSHAKE FAILED:",handshake_error,frame)
                        break
                    verified_profile=True
                elif kind=="MOTOR_AUTO_READY" and not requested_start:
                    ready_expected = (
                        ("version",PROFILE_VERSION),("rounds",NUM_ROUNDS),
                        ("steps",len(MATRIX)),("hard_cap_permille",MAX_CMD),
                        ("both_cap_permille",DUAL_MAX_CMD),("families",FAMILIES),
                        ("pulse_ms",PULSE_MS),("post_ms",POST_MS),
                        ("boot_neutral",1),("sensor",int(SENSOR_PROFILE)))
                    if not verified_profile:
                        handshake_error = "READY received without a complete validated PROFILE."
                        print("HANDSHAKE FAILED:",handshake_error)
                        break
                    if any(intval(v,k)!=want for k,want in ready_expected) or v.get("rx_mode")!="irq256" or intval(v,"state")!=0:
                        handshake_error = "Unsafe/incompatible firmware or motor already active."
                        print("HANDSHAKE FAILED:",handshake_error,frame)
                        break
                    print("Bidirectional UART PROFILE+READY validated. Explicit host START issued.")
                    ser.write(b"MOTOR START HIGH 600\n" if HIGH_PROFILE else b"MOTOR START\n")
                    requested_start=True
                    keep_at=0.0
                elif kind=="MOTOR_AUTO_BEGIN":
                    began=True
                    print(frame)
                elif kind in ("MOTOR_AUTO_START","MOTOR_AUTO_END",
                              "MOTOR_AUTO_ROUND_END","MOTOR_AUTO_FAULT","MOTOR_AUTO_DONE"):
                    print(frame,flush=True)
                    if kind=="MOTOR_AUTO_FAULT": break
                    if kind=="MOTOR_AUTO_DONE":
                        complete=True
                        # Keep capturing small trailing UART frames without active outputs.
                        deadline=time.monotonic()+0.35
                        while time.monotonic()<deadline:
                            b=ser.readline()
                            if b:
                                raw.append(f"{time.monotonic()-start:.6f}\t{b.decode('utf-8',errors='replace').strip()}")
                        break
            if requested_start and not complete:
                try: ser.write(b"MOTOR STOP\n")
                except Exception: pass
    except KeyboardInterrupt:
        # The with-block closes the port before propagating KeyboardInterrupt.
        # Reopen once to send a best-effort stop. This does NOT replace VM cut.
        try:
            with serial.Serial(args.port,115200,timeout=0.1,write_timeout=0.3) as stopser:
                stopser.write(b"MOTOR STOP\n")
        except (serial.SerialException, OSError):
            pass
        print("Interrupted: best-effort UART STOP requested. Physically cut VM if unexpected motion continues.")
    except serial.SerialException as e:
        print("UART failure:",e,"- physically disconnect VM if there is unexpected motion.")
    except subprocess.CalledProcessError as e:
        raise SystemExit(f"Firmware upload failed: {e}") from e

    setup["host_keep_sent_count"] = keep_sent
    setup["handshake_profile_validated"] = verified_profile
    setup["handshake_error"] = handshake_error
    report=store(folder,raw,setup)
    print("Heartbeat link: host KEEP sent =",keep_sent,
          "; firmware last decoded =",report["rx_heartbeat_diagnostics"]["keep_decoded_last"])
    if not began:
        print("No motor sequence began. Inspect raw_serial.log first. If the MCU sends no bytes, then check COM6 and UART wiring; no hardware changes assumed.")
    return 0 if report["status"]=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" else 1


def check_link(args):
    print("Link test: no START will be sent. Validates STM32 UART RX only.")
    if not args.skip_flash:
        subprocess.run([sys.executable,"-m","platformio","run","-e",FIRMWARE_ENV,"-t","upload"],check=True)
    try:
        import serial
    except ImportError as e:
        raise SystemExit("pip install pyserial") from e
    with serial.Serial(args.port,115200,timeout=0.2,write_timeout=1) as ser:
        ser.reset_input_buffer()
        start=time.monotonic()
        profile_ok = False
        while time.monotonic()-start < (60.0 if SENSOR_PROFILE else 8.0):
            ser.write(b"MOTOR PING\n")
            for _ in range(3):
                data=ser.readline().decode(errors="replace").strip()
                if data:
                    print(data)
                kind, values = split_record(data)
                if "MOTOR_AUTO_READY" in data and kind != "MOTOR_AUTO_READY":
                    print("UART FRAME ERROR: READY merged into PROFILE or another frame.")
                    return 1
                if kind == "MOTOR_AUTO_PROFILE":
                    requirements = (
                        ("version",PROFILE_VERSION),("steps",len(MATRIX)),
                        ("hard_cap_permille",MAX_CMD),("families",FAMILIES),
                        ("high_pulse_ms",HIGH_PULSE_MS if HIGH_PROFILE else 0),
                        ("high_post_ms",HIGH_POST_MS if HIGH_PROFILE else 0),
                        ("sensor",int(SENSOR_PROFILE)))
                    if any(intval(values,k)!=v for k,v in requirements):
                        print("PROFILE mismatch; refusing to validate link.")
                        return 1
                    profile_ok = True
                elif kind == "MOTOR_AUTO_READY":
                    if (not profile_ok or intval(values,"version") != PROFILE_VERSION
                        or intval(values,"state") != 0
                        or intval(values,"steps") != len(MATRIX)
                        or intval(values,"sensor",0) != int(SENSOR_PROFILE)):
                        print("READY without matching PROFILE/idle state; link not validated.")
                        return 1
                    print("BIDIRECTIONAL UART PROFILE + READY PASS; no motor drive commands sent.")
                    return 0
        print("BIDIRECTIONAL UART PROFILE + READY NOT CONFIRMED. No motor drive command sent.")
        return 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument("--run",action="store_true",help="Explicitly authorize one 3-round experiment")
    group.add_argument("--check-link",action="store_true",help="UART RX-only test; no motor drive")
    group.add_argument("--self-test",action="store_true",help="Offline host parser regression")
    group.add_argument("--folder",type=Path,help="Re-analyze a saved raw log directory")
    p.add_argument("--port",help="USB-UART port e.g. COM6")
    p.add_argument("--vm-state",choices=["powered","disconnected"])
    p.add_argument("--meter-vm-volts",type=float)
    p.add_argument("--skip-flash",action="store_true")
    p.add_argument("--sensor-integration",action="store_true",
                   help="Opt-in combined motor+MPU RC2/IR single-flash image; calibration before READY")
    p.add_argument("--high-pwm",action="store_true",
                   help="Opt-in additional 30/40/50/60 percent elevated-wheel 120ms per-motor pulses; no current measurement")
    p.add_argument("--confirm-wheels-elevated",action="store_true",
                   help="Confirm elevated, free-spinning wheels and immediate physical VM disconnect")
    p.add_argument("--timeout",type=float,default=150.0)
    args=p.parse_args()
    configure_profile(args.high_pwm,args.sensor_integration)
    if args.high_pwm and args.run and not args.confirm_wheels_elevated:
        p.error("High PWM requires --confirm-wheels-elevated; duty cannot substitute for current limit")
    if args.self_test:
        self_test()
        return 0
    if args.folder:
        raw_path=args.folder/"raw_serial.log"
        if not raw_path.is_file():
            p.error(f"Missing {raw_path}")
        meta_path=args.folder/"session_meta.json"
        meta=json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
        raw = raw_path.read_text(encoding="utf-8").splitlines()
        try:
            configure_from_saved_log(raw,
                high=bool(meta.get("high_profile", False)),
                sensors=bool(meta.get("sensor_profile", False)))
        except ValueError as exc:
            p.error(str(exc))
        report=store(args.folder,raw,meta)
        return 0 if report["status"]=="TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" else 1
    if args.run:
        p.error(
            "P0 STOP_HIZ firmware changed neutral CCR to ARR+1. "
            "Powered motor runs are locked until the UART v7/v8 neutral "
            "telemetry protocol, analyzer, and waveform safety gate are "
            "requalified. Offline --folder and --self-test remain available."
        )
    if not args.port:
        p.error("--port is required for --run or --check-link")
    if args.timeout < 80:
        p.error("--timeout must be at least 80 seconds for the full four-phase suite")
    if args.check_link: return check_link(args)
    return execute(args)


if __name__=="__main__":
    sys.exit(main())
