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
    pinout = (ROOT / "include/pinout.h").read_text(encoding="utf-8")
    assert "#define MOTOR_PWM_FREQUENCY     30000U" in pinout
    assert "#define MOTOR_PWM_PERIOD" not in pinout, (
        "TIM2 ARR must be derived from runtime APB timer clock, not stale pinout constant")
    assert "#define MOTOR_PWM_PRESCALER" not in pinout
    init_check = body(driver, "Motor_Init")
    require_order(init_check,
                  "Board_GetAPB1TimerClockHz()",
                  "arr = timer_clock / MOTOR_PWM_FREQUENCY;",
                  "pwm_period = arr - 1U;",
                  "TIM2->PSC = 0;",
                  "TIM2->ARR = pwm_period;")
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



def check_firmware_uart_interlock() -> None:
    """Check the MCU build blocks BOTH UART START aliases, not just the host CLI."""
    src = (ROOT / "src/motor_auto_bench.c").read_text(encoding="utf-8")
    assert "#define MOTOR_AUTO_P0_ARM_LOCKED 1U" in src
    assert '#error "P0 electrical gate must remain locked until a reviewed release"' in src
    start = src.index("static void ReceiveLine(")
    end = src.index("static void PollHost(", start)
    receive = src[start:end]
    assert "#if MOTOR_AUTO_P0_ARM_LOCKED" in receive
    locked = receive.split("#if MOTOR_AUTO_P0_ARM_LOCKED", 1)[1].split("#else", 1)[0]
    unlocked = receive.split("#else", 1)[1].split("#endif /* MOTOR_AUTO_P0_ARM_LOCKED */", 1)[0]
    assert 'strcmp(line,"MOTOR START")' in locked
    assert 'strcmp(line,"MOTOR START HIGH 600")' in locked
    assert 'MOTOR_AUTO_ARM_BLOCKED,reason=P0_ELECTRICAL_UNQUALIFIED' in locked
    assert "Motor_Stop();" in locked
    assert "state = AUTO_SETTLE;" not in locked
    assert "state = AUTO_SETTLE;" in unlocked
    assert "StartStep(now);" in src
    # Preprocessor-locked firmware must not compile the auto-drive arm branch.
    assert "#if !MOTOR_AUTO_P0_ARM_LOCKED\nstatic uint8_t CanArm(void)" in src



def check_readonly_uart_lock_handshake() -> None:
    """Mock UART transport: link PASS must require P0 lock in BOTH messages.

    skip_flash=True means this test cannot upload firmware or drive motors.
    """
    import contextlib
    import io
    import sys
    from types import SimpleNamespace
    from unittest.mock import patch

    import motor_auto_pipeline as pipeline

    pipeline.configure_profile(high=False, sensors=False)
    profile = (
        "MOTOR_AUTO_PROFILE,version=14,steps=21,hard_cap_permille=200,"
        "families=4,high_pulse_ms=0,high_post_ms=0,sensor=0,p0_locked=1\n"
    ).encode("ascii")
    ready = (
        "MOTOR_AUTO_READY,version=14,state=0,steps=21,sensor=0,p0_locked=1\n"
    ).encode("ascii")

    def handshake(records: list[bytes]) -> int:
        class DummySerial:
            def __init__(self) -> None:
                self.lines = iter(records)
                self.writes = []

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return False

            def reset_input_buffer(self) -> None:
                pass

            def write(self, data: bytes) -> None:
                self.writes.append(data)
                assert data == b"MOTOR PING\n", "Read-only check must never send START"

            def readline(self) -> bytes:
                return next(self.lines, b"")

        test_port = DummySerial()
        serial_stub = SimpleNamespace(Serial=lambda *_a, **_kw: test_port)
        args = SimpleNamespace(skip_flash=True, port="MOCK-PORT")
        with patch.dict(sys.modules, {"serial": serial_stub}):
            with contextlib.redirect_stdout(io.StringIO()):
                result = pipeline.check_link(args)
        assert all(w == b"MOTOR PING\n" for w in test_port.writes)
        return result

    assert handshake([profile, ready]) == 0
    assert handshake([profile.replace(b"p0_locked=1", b"p0_locked=0"), ready]) == 1
    assert handshake([profile, ready.replace(b"p0_locked=1", b"p0_locked=0")]) == 1
    assert handshake([profile, ready.replace(b"state=0", b"state=1")]) == 1


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
    check_firmware_uart_interlock()
    check_readonly_uart_lock_handshake()
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
    print("PASS: source order, locked MCU UART START, modeled STOP/DRIVE/reverse/brake guards.")
    print("UNQUALIFIED: physical transients, LOW/LOW/PWM-low, real brake current and levels.")


if __name__ == "__main__":
    self_test()
