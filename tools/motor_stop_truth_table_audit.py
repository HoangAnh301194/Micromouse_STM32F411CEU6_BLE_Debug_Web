#!/usr/bin/env python3
"""Source-only TB6612FNG state audit. NEVER flashes, powers, or moves a motor.

Pin truth table: Toshiba TB6612FNG datasheet, H-SW Control Function, p.4.
'Unspecified' means not guaranteed by the cited table, NOT proven hazardous.
Software CCR registers are NOT measured bridge output voltages/currents.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent.parent

# Truth table entries only: PWM is a digital H/L pin level, not duty percent.
TB6612 = {
    (0, 0, 1): "STOP_HIZ",
    (1, 1, 0): "SHORT_BRAKE",
    (1, 1, 1): "SHORT_BRAKE",
    (1, 0, 1): "CW",
    (0, 1, 1): "CCW",
    (1, 0, 0): "SHORT_BRAKE",
    (0, 1, 0): "SHORT_BRAKE",
}


def mode(in1: int, in2: int, pwm: int, stby: int = 1) -> str:
    if stby == 0:
        return "STANDBY_HIZ"
    return TB6612.get((in1, in2, pwm), "UNSPECIFIED_BY_DATASHEET")


def body(code: str, func: str) -> str:
    """Extract function body, ignoring declarative signatures in headers."""
    match = re.search(r"\bvoid\s+" + re.escape(func) +
                      r"\s*\([^;{}]*\)\s*\{", code)
    if not match:
        raise ValueError(f"Missing definition: {func}")
    start = match.end()
    depth = 1
    for i in range(start, len(code)):
        if code[i] == "{":
            depth += 1
        elif code[i] == "}":
            depth -= 1
            if depth == 0:
                return code[start:i]
    raise ValueError(f"Unbalanced braces in: {func}")


def inspect_driver(root: Path = ROOT):
    motor = (root / "src/motor.c").read_text(encoding="utf-8")
    header = (root / "include/pinout.h").read_text(encoding="utf-8")
    stop = body(motor, "Motor_Stop")
    brake = body(motor, "Motor_Brake")
    release = body(motor, "Motor_SetPairPermille")
    update = body(motor, "Motor_Update1ms")
    init = body(motor, "Motor_Init")

    def pin(name):
        m = re.search(r"(?m)^\s*#define\s+" + re.escape(name) +
                      r"\s+(\d+)\b", header)
        return int(m.group(1)) if m else None

    stop_pins_low = all(
        f"AddDirection(&bsrr, MOTOR_{w}_IN1_PIN, MOTOR_{w}_IN2_PIN, 0);"
        in stop for w in ("L", "R")
    )
    stop_ccr_high = "TIM2->CCR1 = TIM2->CCR2 = pwm_period + 1U;" in stop
    # Both PWM channels use PWM mode 1, non-inverted output CCxP reset at init.
    pwm_mode1 = "(6UL << 4)" in init and "(6UL << 12)" in init
    # PWM=H is specified to coast only in IN1=IN2=L state.
    stop_expected = mode(0, 0, 1) if stop_pins_low and stop_ccr_high and pwm_mode1 else "UNKNOWN"
    active_brake = (
        all(f"AddPinLevel(&bsrr, MOTOR_{w}_IN{n}_PIN, 1);" in brake
            for w in ("L", "R") for n in (1, 2)) and
        "TIM2->CCR1 = pwm_period + 1U;" in brake and
        "TIM2->CCR2 = pwm_period + 1U;" in brake
    )
    preload = all(x in init for x in ("TIM_CCMR1_OC1PE", "TIM_CCMR1_OC2PE"))
    reverse_gap = "#define MOTOR_REVERSE_GAP_MS 30U" in motor
    brake_release = (all(x in release for x in (
        "if (brake_latched)", "TIM2->CCR1 = TIM2->CCR2 = pwm_period + 1U;",
        "brake_release_ticks = 2U",
        "wheel_l.reverse_gap_ms = MOTOR_REVERSE_GAP_MS",
        "wheel_r.reverse_gap_ms = MOTOR_REVERSE_GAP_MS"))
        and "if (brake_release_ticks > 0U)" in update)
    runtime_pwm_hz = pin("MOTOR_PWM_FREQUENCY")
    pins = {
        "left_in1_pb": pin("MOTOR_L_IN1_PIN"),
        "left_in2_pb": pin("MOTOR_L_IN2_PIN"),
        "right_in1_pb": pin("MOTOR_R_IN1_PIN"),
        "right_in2_pb": pin("MOTOR_R_IN2_PIN"),
        "left_pwm_pb": pin("MOTOR_L_PWM_PIN"),
        "right_pwm_pa": pin("MOTOR_R_PWM_PIN"),
    }
    pin_comment_disagrees = ("IN1=PB14, IN2=PB15" in header and
                             pins["right_in1_pb"] == 15 and
                             pins["right_in2_pb"] == 14)
    direct_snapshot = all(marker in update for marker in (
        "was_stopped_l = (wheel_l.applied == 0);", "was_stopped_r = (wheel_r.applied == 0);"))
    cached_snapshot = all(marker in update for marker in (
        "previous_left = wheel_l.applied;", "previous_right = wheel_r.applied;",
        "was_stopped_l = (previous_left == 0);", "was_stopped_r = (previous_right == 0);"))
    start_guard = (direct_snapshot or cached_snapshot) and all(marker in update for marker in (
        "was_stopped_l ? 0 : wheel_l.applied", "was_stopped_r ? 0 : wheel_r.applied"))
    init_safe = all(x in init for x in ("TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;", "TIM2->CCR1 = pwm_period + 1U;", "TIM2->CCR2 = pwm_period + 1U;"))
    checks = {
        "stop_high_ccr": stop_ccr_high,
        "stop_drive_preload_guard": start_guard,
        "init_coast_and_active_high": init_safe,
        "stop_both_inputs_low": stop_pins_low,
        "software_coast_defined": stop_expected == "STOP_HIZ",
        "software_stop_truth_table_mode": stop_expected,
        "active_brake_implemented": active_brake,
        "active_brake_truth_table_mode": mode(1,1,1) if active_brake else "UNKNOWN",
        "preload_enabled": preload,
        "reverse_gap_30ms": reverse_gap,
        "brake_release_neutral_guard": brake_release,
        "hardware_pwm_30khz": runtime_pwm_hz == 30000,
        "timer_frequency_hz": runtime_pwm_hz,
        "pinout": pins,
        "stby_controllable_from_firmware": False,
        "pinout_comment_vs_macro_mismatch": pin_comment_disagrees,
    }
    blockers = []
    if not stop_pins_low or not stop_ccr_high or not pwm_mode1:
        blockers.append("Motor_Stop register/PWM mode 1 state not fully recognized; inspect source before using hardware.")
    if stop_expected != "STOP_HIZ":
        blockers.append(
            "P0 FAIL: Motor_Stop() does not command Toshiba-defined LOW/LOW/HIGH Hi-Z Stop."
        )
    if not start_guard or not init_safe:
        blockers.append("P0 FAIL: safe startup polarity or stop-to-drive preload guard not recognized.")
    if not active_brake:
        blockers.append("Short-brake source cannot be validated against both IN pins HIGH.")
    if not preload or not brake_release or not reverse_gap:
        blockers.append("PWM preload or brake release/reversal neutral timing requires review.")
    if pin_comment_disagrees:
        blockers.append("Right motor pinout comment conflicts with compiled pin macros (PB14/PB15 reversed).")
    blockers += [
        "Transient switching while PWM preload updates requires oscilloscope validation; LOW/LOW/PWM-low is not characterized.",
        "STBY is fixed HIGH on this board: host STOP or MCU GPIO cannot independently disable power stage.",
        "At VCC=5.0V, VIH(min)=3.5V but STM32 GPIO nominal HIGH=3.3V; actual noise margin not guaranteed.",
        "No measured motor current, thermal rise, PWM pin waveform or bridge OUT1/OUT2 states. Active brake/reverse not powered-tested.",
    ]
    return checks, blockers


def audit_result(root: Path = ROOT):
    checks, blockers = inspect_driver(root)
    return {
        "status": "REVIEW" if checks["software_coast_defined"] and
                  checks["active_brake_implemented"] and
                  checks["brake_release_neutral_guard"] and
                  checks["stop_drive_preload_guard"] and
                  checks["init_coast_and_active_high"] else "BLOCKED",
        "software_stop_mode": checks["software_stop_truth_table_mode"],
        "software_brake_mode": checks["active_brake_truth_table_mode"],
        "checks": checks, "blockers": blockers,
        "datasheet": "https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG",
        "physical_test_performed": False,
        "no_automatic_brake_reverse_or_high_pwm": True,
    }


def self_test():
    assert mode(0,0,1) == "STOP_HIZ"
    assert mode(0,0,0) == "UNSPECIFIED_BY_DATASHEET"
    assert mode(1,1,0) == mode(1,1,1) == "SHORT_BRAKE"
    assert mode(1,0,0) == mode(0,1,0) == "SHORT_BRAKE"
    assert mode(1,0,1) == "CW" and mode(0,1,1) == "CCW"
    for in1 in (0,1):
        for in2 in (0,1):
            for pwm in (0,1):
                assert mode(in1,in2,pwm,stby=0) == "STANDBY_HIZ"
    audit = audit_result()
    assert audit["status"] == "REVIEW", audit
    assert audit["software_stop_mode"] == "STOP_HIZ"
    assert audit["checks"]["stop_drive_preload_guard"]
    assert audit["checks"]["init_coast_and_active_high"]
    assert audit["checks"]["active_brake_truth_table_mode"] == "SHORT_BRAKE"
    assert audit["checks"]["preload_enabled"]
    assert audit["checks"]["brake_release_neutral_guard"]
    print("PASS: datasheet STOP_HIZ, initialization and preload guard; physical measurement still required.")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--self-test", action="store_true")
    p.add_argument("--json", action="store_true", help="Print source audit as JSON (no hardware access)")
    a = p.parse_args()
    if a.self_test:
        self_test()
        return 0
    result = audit_result()
    if a.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        print("TB6612FNG P0 STATIC AUDIT:", result["status"])
        print("Motor_Stop software state:", result["software_stop_mode"])
        print("Motor_Brake software state:", result["software_brake_mode"])
        for issue in result["blockers"]:
            print(" -", issue)
        print("No firmware flashed or motors powered.")
    # BLOCKED is an informative verdict, not Python program failure:
    # CI must fail only if the actual code/datasheet expectations regress.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
