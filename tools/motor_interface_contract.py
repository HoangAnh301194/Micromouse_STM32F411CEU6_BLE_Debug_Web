#!/usr/bin/env python3
"""Offline Motor Driver/Control/Motion API integration contract gate.

Inspects the frozen C source. No motor power, upload, UART, or encoder-speed
measurement. This is an integration source regression, not a physical waveform
or real-time scheduler performance certificate.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
FILES = ("include/motor.h", "include/control.h", "include/pinout.h",
         "src/motor.c", "src/control.c", "src/motion.c")


class ContractError(ValueError):
    pass


def require(text: str, needle: str, reason: str) -> None:
    if needle not in text:
        raise ContractError(reason)


def ordered(text: str, terms: tuple[str, ...], reason: str) -> None:
    pos = -1
    for term in terms:
        pos = text.find(term, pos + 1)
        if pos < 0:
            raise ContractError(reason)


def inspect_contract(s: dict[str, str]) -> dict:
    for name in FILES:
        if name not in s:
            raise ContractError(f"Missing source: {name}")
    header, timer, pin = s["include/motor.h"], s["include/control.h"], s["include/pinout.h"]
    motor, control, motion = s["src/motor.c"], s["src/control.c"], s["src/motion.c"]

    # The high-level controller must command the driver, not its registers.
    for token in ("TIM2->", "GPIOB->BSRR", "MOTOR_R_IN1_PIN", "MOTOR_L_IN2_PIN",
                  "Motor_KeepAlive()"):
        if token in motion:
            raise ContractError(f"Motion bypasses Driver ownership / keepalive: {token}")
    require(motion, "Motor_SetPair(ClampPwm(left_pwm), ClampPwm(right_pwm));",
            "Straight Motion no longer issues signed driver commands")
    require(motion, "Motor_SetPair(ClampPwm(-output), ClampPwm(output));",
            "Turn Motion no longer issues differential signed driver commands")
    require(motion, "Motor_Stop();", "Motion lacks the explicit neutral STOP")

    # Control owns 1kHz scheduling; Motor owns the actual PWM timer registers.
    require(timer, "#define CONTROL_FREQUENCY_HZ  1000UL",
            "Control 1ms tick frequency changed")
    for token in ("TIM2->", "GPIOB->BSRR", "Motor_KeepAlive()"):
        if token in control:
            raise ContractError(f"Control bypasses Motor Driver authority: {token}")
    stop = control.split("void Control_Stop(void)", 1)[-1].split(
        "uint8_t Control_IsRunning(void)", 1)[0]
    ordered(stop, ("MotorTest_Stop();", "Motion_Stop();",
                   "TIM11->CR1 &= ~TIM_CR1_CEN;"),
            "Control_Stop must neutralize both owners before stopping TIM11")
    irq = control.split("void TIM1_TRG_COM_TIM11_IRQHandler(void)", 1)[-1]
    require(irq, "Motor_Update1ms();", "Control interrupt no longer runs the driver")
    ordered(irq, ("Motor_Update1ms();", "MotorTest_Tick1ms();",
                  "if (MotorTest_IsRunning())", "Motion_Update("),
            "Control interrupt no longer preserves Motor/MotorTest/Motion arbitration")
    require(irq, "Motion_Stop();", "Expired IMU path no longer calls Motion_Stop")

    for api in ("Motor_SetPairPermille(", "Motor_Update1ms(", "Motor_Stop(",
                "Motor_Brake(", "Motor_LockOutputs(", "Motor_GetDiagnostics(",
                "Motor_KeepAlive("):
        require(header, api, f"Motor public API removed: {api}")
    require(header, "#define MOTOR_COMMAND_TIMEOUT_MS 300U", "300ms lease changed")
    require(motor, "command_timeout_latched = 1U;", "Driver lacks timeout fault latch")
    require(motor, "Motor_LockOutputs();", "Driver lacks lock after timeout")
    for alias, pin_no in {
        "MOTOR_R_PWM_PIN": "15", "MOTOR_L_PWM_PIN": "3",
        "MOTOR_R_IN1_PIN": "15", "MOTOR_R_IN2_PIN": "14",
        "MOTOR_L_IN1_PIN": "13", "MOTOR_L_IN2_PIN": "12",
    }.items():
        import re
        if re.search(r"(?m)^#define\s+" + alias + r"\s+" + pin_no + r"\b", pin) is None:
            raise ContractError(f"Motor pin mapping drifted: {alias}")
    require(pin, "#define MOTOR_PWM_FREQUENCY     30000U",
            "PWM timer frequency contract changed")
    return {"status": "PASS_MOTOR_INTERFACE_CONTRACT",
            "scope": "STATIC_INTEGRATION_SOURCE_CHECK_ONLY",
            "motion_owns_duty_decision": True,
            "driver_owns_pwm_and_direction": True,
            "scheduler_services_driver_before_motion": True,
            "scheduler_stop_neutralizes_first": True,
            "battery_or_encoder_speed_required": False,
            "powered_hardware_verified": False}


def audit(root: Path = ROOT) -> dict:
    try:
        sources = {name: (root / name).read_text(encoding="utf-8") for name in FILES}
    except OSError as e:
        raise ContractError(f"Source file unavailable: {e}") from e
    return inspect_contract(sources)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--audit", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        import unittest
        suite = unittest.defaultTestLoader.discover(
            str(ROOT / "tests/motor_driver"), pattern="test_motor_interface_contract.py")
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    try:
        print(json.dumps(audit(), indent=2))
    except ContractError as e:
        print(f"FAIL_MOTOR_INTERFACE_CONTRACT: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
