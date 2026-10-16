#!/usr/bin/env python3
"""Clock 96 MHz / HSI trim25 integrated firmware contract.

Offline gate only: validates selected RCC source/profile, derived peripheral
divider contract, fail-closed clock startup, and unchanged Motor Rev3 blobs.
No physical frequency/EMI/PWM waveform measurement is performed.
"""
from __future__ import annotations

import argparse
import configparser
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent.parent

MOTOR_FROZEN_BLOBS = {
    "src/motor.c": "57fc04072cd9d206e22e54e987a9b63b450739f0",
    "include/motor.h": "9ed969ef6e46d682dbfd7a0607bff629be2330fc",
    "src/motor_test.c": "fdd1be9cb4ae353e76b289f765b206424a62cb74",
    "include/motor_test.h": "1cf62c21e2e611770bc18d719693b71dde165b95",
    "src/motor_startup.c": "2b6438ee93305a7d6d6e2e5dba1d83b65288a25c",
    "include/motor_startup.h": "12638ac6a5749609c7dc283c89af0a0ddefa8e79",
}
FILES = tuple(MOTOR_FROZEN_BLOBS) + (
    "src/board.c", "include/board.h", "src/main.c", "platformio.ini",
    "src/control.c", "include/control.h", "src/system_timer.c",
    "src/motor.c", "include/pinout.h", "src/i2c.c", "src/uart.c",
    "src/bt_debug.c",
)


class ClockContractError(ValueError):
    pass


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ClockContractError(reason)


def git_blob(b: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(b)).encode() + b"\x00" + b).hexdigest()


def verify(files: dict[str, bytes]) -> dict:
    def read(path: str) -> str:
        if path not in files:
            raise ClockContractError("Source missing: " + path)
        return files[path].decode("utf-8")

    for name, sha in MOTOR_FROZEN_BLOBS.items():
        data = files[name]
        require(sha in (git_blob(data), git_blob(data.replace(b"\r\n", b"\n"))),
                "Frozen Motor Driver modified: " + name)

    ini = read("platformio.ini")
    config = configparser.ConfigParser(interpolation=None)
    config.read_string(ini)
    flags = config.get("env:genericSTM32F411CE", "build_flags").split()
    expected = {
        "-DCLOCK_VALIDATE_96M=1",
        "-DCLOCK_HSI_TRIM=25",
        "-DMPU_SAMPLING_POLICY_B=1",
        "-DMPU_TIM11_TICK_SCHED=1",
    }
    require(expected.issubset(flags), "Default firmware flags missing RC2 96MHz profile")
    require(not any(k in flags for k in (
        "-DMPU_INTEGRATION_BENCH=1", "-DMPU_PHASE_AB=1",
        "-DCLOCK_AUTOTUNE=1", "-DMPU_FRESHNESS_DIAG=1")),
        "Default production build accidentally enables experimental benches")

    board = read("src/board.c")
    hdr = read("include/board.h")
    require(bool(re.search(
        r"#elif defined\(CLOCK_VALIDATE_96M\).*?"
        r"#define BOARD_PLLN 192UL\s+"
        r"#define BOARD_EXPECTED_SYSCLK 96000000UL", board, flags=re.S)),
        "Board RCC PLLN=192 / expected 96MHz profile missing")
    require("CLOCK_HSI_TRIM != 25" in board,
            "Board must refuse non-25 trim in 96MHz profile")
    require("(16UL << 0) | (BOARD_PLLN << 6)" in board and
            "(0UL << 16) | (5UL << 24)" in board,
            "HSI PLLM=16 / PLLP=2 / PLLQ=5 mapping drifted")
    require("RCC_CFGR_PPRE1_DIV2" in board,
            "APB1 divider /2 missing")
    require("RCC->CR = (RCC->CR & ~(0x1FUL << 3)) |" in board,
            "HSI trim is not programmed before PLL")
    require("void Board_Init(void)" in board and
            "    Board_ConfigureHsiPllClock();" in board,
            "Board_Init no longer applies the clock configuration")
    for sign in ("board_clock_fault = 1U", "board_clock_fault = 2U",
                 "board_clock_fault = 5U", "board_clock_fault = 7U",
                 "board_clock_fault = 8U",
                 "SystemCoreClockUpdate();",
                 "uint32_t Board_GetClockFault(void)",
                 "uint32_t Board_GetHsiTrim(void)"):
        require(sign in board, "Missing PLL stage/fault guard: " + sign)
    for sign in ("Board_GetClockFault(void)", "Board_GetHsiTrim(void)",
                 "Board_GetHsiCal(void)"):
        require(sign in hdr, "Missing clock diagnostics public API: " + sign)
    require("TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);" in board
            and "GPIOB->BSRR = (1UL << (MOTOR_L_IN1_PIN + 16U))" in board,
            "Motor HardFault immediate disable/direction guard lost")

    main = read("src/main.c")
    seq = ("    Board_Init();", "    SystemTimer_Init();", "    UART_Init();",
           "if (Board_GetClockFault() != 0U)", "while (1) { __WFI(); }",
           "UART_Printf(\"CLOCK_PROFILE,", "    Motor_Init();",
           "    Encoder_Init();", "    BT_Init();", "    IR_Init();")
    at = -1
    for token in seq:
        nxt = main.find(token, at + 1)
        require(nxt > at, "Clock fail-closed startup order broken: " + token)
        at = nxt
    require("BOOT_FAIL,CLOCK" in main and "Board_StatusLed(1U)" in main,
            "No clock fault diagnostic and visible failure state")
    require("MPU6050_Update(&mpu);" in main,
            "MPU RC2 IRQ-driven sensor acquisition no longer serviced from main")
    control = read("src/control.c")
    require(control.count("Motor_Update1ms();") == 1 and
            "MPU6050_NotifyControlTick(control_mpu, micros());" in control and
            "MPU6050_Update(control_mpu);" not in control,
            "TIM11 Motor/MPU scheduler regression")
    require(control.index("Motor_Update1ms();") <
            control.index("MPU6050_NotifyControlTick("),
            "Motor driver service must precede MPU tick notification")
    require("Motion_Stop(); /* Invalid, uncalibrated or stale gyro. */" in control,
            "Stale MPU motion fail-closed guard missing")
    require("CONTROL_FREQUENCY_HZ  1000UL" in read("include/control.h"),
            "TIM11 tick base must remain 1kHz")
    require("uint32_t divider = timer_hz / 1000000UL;" in read("src/system_timer.c"),
            "TIM5 1MHz timer derivation regressed")
    require("MOTOR_PWM_FREQUENCY     30000U" in read("include/pinout.h") and
            "arr = timer_clock / MOTOR_PWM_FREQUENCY;" in read("src/motor.c"),
            "Motor TIM2 30kHz runtime calculation regression")
    require("pclk_hz = Board_GetAPB1ClockHz();" in read("src/i2c.c") and
            "400000UL" in read("src/i2c.c"),
            "I2C1 400kHz peripheral clock derivation regressed")
    require("USART1->BRR = (Board_GetAPB2ClockHz()" in read("src/uart.c"),
            "USART1 baud rate no longer tracks APB2")
    require("USART2->BRR = brr_value;" in read("src/bt_debug.c") and
            "plln = (RCC->PLLCFGR" in read("src/bt_debug.c"),
            "BLE USART2 baud configuration no longer tracks PLL")

    # Nominal RCC-derived values, explicitly not external measurements.
    hsi = 16_000_000
    sysclk = (hsi // 16) * 192 // 2
    apb1, apb2 = sysclk // 2, sysclk
    tim_apb1 = apb1 * 2
    tim_apb2 = apb2
    pwm_arr = tim_apb1 // 30_000 - 1
    tim5_psc = tim_apb1 // 1_000_000 - 1
    tim11_psc = tim_apb2 // 1_000_000 - 1
    tim11_arr = 1_000_000 // 1_000 - 1
    i2c_fast_ccr = apb1 // (3 * 400_000)
    usart1_brr = (apb2 + 115_200 // 2) // 115_200
    require((sysclk, apb1, apb2, tim_apb1, tim_apb2, pwm_arr,
             tim5_psc, tim11_psc, tim11_arr, i2c_fast_ccr, usart1_brr) ==
            (96_000_000, 48_000_000, 96_000_000, 96_000_000, 96_000_000,
             3199, 95, 95, 999, 40, 833),
            "Nominal 96MHz RCC/divider arithmetic regression")
    return {
        "status": "PASS_SELECTED_CLOCK_96M_HSI_TRIM25_SOFTWARE_CONTRACT",
        "firmware_env": "genericSTM32F411CE",
        "nominal_sysclk_hz": sysclk,
        "hsi_trim_register_setting": 25,
        "apb1_hz": apb1,
        "apb2_hz": apb2,
        "tim2_arr_for_30khz": pwm_arr,
        "tim5_psc_for_1mhz": tim5_psc,
        "tim11_psc_for_1khz": tim11_psc,
        "tim11_arr": tim11_arr,
        "i2c1_fast_ccr": i2c_fast_ccr,
        "usart1_brr": usart1_brr,
        "motor_driver_frozen_blobs": len(MOTOR_FROZEN_BLOBS),
        "motor_clock_fault_guard": "FAIL_CLOSED_BEFORE_MOTOR_INIT",
        "electrical_frequency_measured": False,
        "motor_on_clock_qualified": False,
    }


def gather(root: Path = ROOT) -> dict[str, bytes]:
    return {p: (root / p).read_bytes() for p in set(FILES)}


def self_test() -> None:
    original = gather()
    verify(original)
    mutations = [
        ("platformio.ini", b"-DCLOCK_VALIDATE_96M=1",
         b"-DCLOCK_VALIDATE_96M=0", "Default firmware flags"),
        ("platformio.ini", b"-DCLOCK_HSI_TRIM=25",
         b"-DCLOCK_HSI_TRIM=20", "Default firmware flags"),
        ("src/board.c", b"#define BOARD_PLLN 192UL",
         b"#define BOARD_PLLN 200UL", "Board RCC"),
        ("src/board.c", b"RCC_CFGR_PPRE1_DIV2",
         b"RCC_CFGR_PPRE1_DIV1", "APB1 divider"),
        ("src/board.c", b"    Board_ConfigureHsiPllClock();",
         b"    /* missing startup */", "Board_Init"),
        ("src/board.c", b"TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);",
         b"/* no HardFault CCR disable */", "HardFault"),
        ("src/main.c", b"if (Board_GetClockFault() != 0U)",
         b"if (0)", "Clock fail-closed"),
        ("src/motor.c", b"#include \"motor.h\"",
         b"#include \"motion.h\"", "Frozen Motor Driver"),
        ("src/control.c", b"    Motor_Update1ms();",
         b"    /* removed */", "TIM11 Motor"),
        ("src/i2c.c", b"pclk_hz = Board_GetAPB1ClockHz();",
         b"pclk_hz = 16000000;", "I2C1 400kHz"),
    ]
    for path, old, new, msg in mutations:
        require(old in original[path], "Bad mutation fixture: " + path)
        mutant = dict(original)
        mutant[path] = original[path].replace(old, new, 1)
        try:
            verify(mutant)
        except ClockContractError as e:
            require(msg in str(e), "Mutation detected for wrong reason: " + str(e))
        else:
            raise ClockContractError("Unsafe mutant not detected: " + path + " / " + msg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--audit", action="store_true")
    modes.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    try:
        if args.self_test:
            self_test()
            print("PASS_CLOCK96_INTEGRATION_NEGATIVE_TESTS: 10 mutants rejected")
        else:
            print(json.dumps(verify(gather()), indent=2))
    except (ClockContractError, OSError, ValueError, UnicodeError) as e:
        print(f"FAIL_CLOCK96_INTEGRATION: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
