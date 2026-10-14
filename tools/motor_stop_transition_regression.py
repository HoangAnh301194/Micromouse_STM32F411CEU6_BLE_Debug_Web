#!/usr/bin/env python3
"""Unpowered Motor Driver state-transition regression.

This is a *model* of the current 1 kHz wheel-state logic and the TIM2
preload boundary, not execution of STM32 firmware or a bridge waveform.
It deliberately does not certify the undefined LOW/LOW/PWM-low transient.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from motor_stop_truth_table_audit import body, inspect_driver, mode

ROOT = Path(__file__).resolve().parent.parent
PERIOD_COUNTS = 3200
REVERSE_GAP_MS = 30
RISE = 20
FALL = 30


def require_order(source: str, *parts: str) -> None:
    """Fail on absent or reordered source operations."""
    last = -1
    for part in parts:
        pos = source.find(part, last + 1)
        assert pos >= 0, f"Absent/out-of-order source operation: {part}"
        last = pos


def check_source_order() -> None:
    driver = (ROOT / "src/motor.c").read_text(encoding="utf-8")
    checks, _ = inspect_driver(ROOT)
    for key in ("software_coast_defined", "active_brake_implemented",
                "preload_enabled", "stop_drive_preload_guard",
                "init_coast_and_active_high", "brake_release_neutral_guard",
                "reverse_gap_30ms"):
        assert checks[key], f"P0 source regression: {key}"
    init = body(driver, "Motor_Init")
    require_order(init, "GPIOB->BSRR =", "TIM2->CCR1 = pwm_period + 1U;",
                  "TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;",
                  "TIM2->EGR = TIM_EGR_UG;", "GPIO_AF(MOTOR_R_PWM_PORT",
                  "TIM2->CR1 |= TIM_CR1_CEN;")
    stop = body(driver, "Motor_Stop")
    require_order(stop, "GPIOB->BSRR = bsrr;",
                  "TIM2->CCR1 = TIM2->CCR2 = pwm_period + 1U;")
    brake = body(driver, "Motor_Brake")
    require_order(brake, "GPIOB->BSRR = bsrr;",
                  "TIM2->CCR1 = pwm_period + 1U;",
                  "TIM2->CCR2 = pwm_period + 1U;")
    update = body(driver, "Motor_Update1ms")
    require_order(update, "was_stopped_l = (wheel_l.applied == 0);",
                  "UpdateWheel(&wheel_l);",
                  "was_stopped_l ? 0 : wheel_l.applied",
                  "GPIOB->BSRR = bsrr;",
                  "TIM2->CCR2 = DutyFromCmd")
    release = body(driver, "Motor_SetPairPermille")
    require_order(release, "if (brake_latched)",
                  "TIM2->CCR1 = TIM2->CCR2 = pwm_period + 1U;",
                  "GPIOB->BSRR = neutral;", "brake_release_ticks = 2U")
    fault = (ROOT / "src/board.c").read_text(encoding="utf-8")
    require_order(body(fault, "HardFault_Handler"),
                  "TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);",
                  "TIM2->CCR1 = 0;", "GPIOB->BSRR =")


def sign(value: int) -> int:
    return (value > 0) - (value < 0)


@dataclass
class Wheel:
    target: int = 0
    applied: int = 0
    last_direction: int = 0
    reverse_gap_ms: int = 0
    direction_gpio: int = 0   # 0 = both LOW, 1 = forward, -1 = backward, 2 = brake
    ccr_shadow: int = PERIOD_COUNTS
    ccr_active: int = PERIOD_COUNTS
    brake_latched: bool = False
    brake_release_ticks: int = 0

    def timer_update(self) -> None:
        """Model just the register preload transfer, not terminal voltage."""
        self.ccr_active = self.ccr_shadow

    def command(self, target: int) -> None:
        if self.brake_latched:
            self.ccr_shadow = PERIOD_COUNTS
            self.direction_gpio = 0
            self.reverse_gap_ms = REVERSE_GAP_MS
            self.brake_release_ticks = 2
        self.brake_latched = False
        self.target = max(-1000, min(1000, target))

    def stop(self) -> None:
        if self.applied:
            self.last_direction = sign(self.applied)
            self.reverse_gap_ms = REVERSE_GAP_MS
        self.target = self.applied = 0
        self.brake_latched = False
        self.brake_release_ticks = 0
        self.direction_gpio = 0
        self.ccr_shadow = PERIOD_COUNTS

    def brake(self) -> None:
        self.target = self.applied = 0
        self.reverse_gap_ms = REVERSE_GAP_MS
        self.brake_latched = True
        self.direction_gpio = 2
        self.ccr_shadow = PERIOD_COUNTS

    def tick_1ms(self) -> None:
        if self.brake_latched:
            return
        if self.brake_release_ticks:
            self.brake_release_ticks -= 1
            return
        current = self.applied
        current_sign = sign(current)
        desired = self.target
        desired_sign = sign(desired)
        was_stopped = current == 0
        if self.reverse_gap_ms:
            self.reverse_gap_ms -= 1
        if current_sign and desired_sign and desired_sign != current_sign:
            desired = 0
        rate = (FALL if desired == 0 or
                (current_sign and sign(desired) != current_sign) or
                abs(current) > abs(desired) else RISE)
        delta = max(-rate, min(rate, desired - current))
        if (not current_sign and desired_sign and self.last_direction and
                desired_sign != self.last_direction and self.reverse_gap_ms):
            delta = 0
        self.applied = current + delta
        if current_sign and self.applied == 0:
            self.reverse_gap_ms = REVERSE_GAP_MS
        if self.applied:
            self.last_direction = sign(self.applied)
        # For newly nonzero applied, direction remains low for this 1-ms
        # boundary, while CCR is waiting for a TIM2 preload transfer.
        self.direction_gpio = 0 if was_stopped else sign(self.applied)
        self.ccr_shadow = (PERIOD_COUNTS if not self.applied else
                           abs(self.applied) * PERIOD_COUNTS // 1000)


def self_test() -> None:
    check_source_order()
    w = Wheel()
    assert (w.direction_gpio, w.ccr_active) == (0, PERIOD_COUNTS)
    assert mode(0, 0, 1) == "STOP_HIZ"
    w.command(160)
    w.tick_1ms()
    assert w.applied == 20 and w.direction_gpio == 0
    assert w.ccr_active == PERIOD_COUNTS and w.ccr_shadow == 64
    w.timer_update()
    w.tick_1ms()
    assert w.direction_gpio == 1 and w.ccr_active == 64
    for _ in range(8):
        w.timer_update()
        w.tick_1ms()
    assert w.applied == 160 and w.direction_gpio == 1
    # Immediate Stop makes GPIO LOW before a new high CCR takes effect.
    # In PWM LOW subcycles, LOW/LOW/LOW is explicitly unqualified.
    w.stop()
    assert w.direction_gpio == 0 and w.ccr_shadow == PERIOD_COUNTS
    assert mode(0, 0, 0) == "UNSPECIFIED_BY_DATASHEET"
    w.timer_update()
    assert w.ccr_active == PERIOD_COUNTS and mode(0, 0, 1) == "STOP_HIZ"
    # Opposite direction must not appear before full 30-ms gap AND
    # the first limited CCR transfer after the gap.
    w.command(-160)
    for _ in range(29):
        w.tick_1ms()
        assert w.direction_gpio == 0 and w.applied == 0
        w.timer_update()
    w.tick_1ms()
    assert w.applied == -20 and w.direction_gpio == 0
    w.timer_update()
    w.tick_1ms()
    assert w.direction_gpio == -1 and w.ccr_active == 64
    w.brake()
    assert w.direction_gpio == 2 and w.brake_latched
    w.timer_update()
    w.command(-100)  # same direction: isolate brake-release wait from reversal gap
    assert w.direction_gpio == 0 and w.brake_release_ticks == 2
    for remaining in (1, 0):
        w.tick_1ms()
        assert w.direction_gpio == 0 and w.brake_release_ticks == remaining
        w.timer_update()
    w.tick_1ms()
    assert w.direction_gpio == 0 and w.ccr_shadow == 64
    w.timer_update()
    w.tick_1ms()
    assert w.direction_gpio == -1 and w.ccr_active == 64
    print("PASS: source order + modeled STOP/DRIVE/reverse/brake-release guards.")
    print("UNQUALIFIED: physical transients, LOW/LOW/PWM-low, real brake current and levels.")


if __name__ == "__main__":
    self_test()
