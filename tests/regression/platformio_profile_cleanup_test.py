#!/usr/bin/env python3
"""Regression: retired PIO test modes are opt-in and live environments unchanged.
Software-only; never identifies old hardware test firmware as qualified for motion.
"""
import configparser
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
ACTIVE = (
    "genericSTM32F411CE",
    "integration_diag_motor_off_96m",
    "integration_diag_motor_on_96m",
    "flash_map_diag_96m",
    "ble_profile_motor_off_115200_96m",
    "ble_profile_motor_off_38400_96m",
    "ble_profile_motor_on_pid_115200_96m",
    "ble_rx_diag_96m",
    "ir_filter_diag_96m",
    "system_test_96m",
    "turn_pid_diag_96m",
    "ir_burst_diag_96m",
)
RETIRED = (
    "integration_diag_uart_96m",
    "integration_diag_encoder_motor_off_96m",
    "integration_diag_encoder_motor_on_96m",
    "integration_diag_encoder_motor_on_limited_30_96m",
    "speed_pid_alpha_008_96m",
    "speed_pid_alpha_010_96m",
    "speed_pid_alpha_030_96m",
    "ir_oled_test_96m",
    "oled_mpu_diag_96m",
)

def read(path):
    config = configparser.ConfigParser(interpolation=None)
    config.read(ROOT/path, encoding="utf-8")
    return config

def effective_flags(cfg, name, stack=()):
    assert name not in stack, ("Cycle in PIO environment inheritance", stack, name)
    section = "env:" + name
    assert cfg.has_section(section), "Missing section: " + section
    raw = cfg.get(section, "build_flags", fallback="")
    # PIO uses dollar references to expand build_flags from parent sections.
    def expand(match):
        parent = match.group(1)
        return " ".join(effective_flags(cfg, parent, stack+(name,)))
    raw = re.sub(r"\$\{env:([^}.]+)\.build_flags\}", expand, raw)
    assert "${" not in raw, "Unresolved PIO reference in " + name
    return tuple(raw.split())

def test_cleanup():
    active = read("platformio.ini")
    legacy = read("platformio_legacy.ini")
    active_set = {s[4:] for s in active.sections() if s.startswith("env:")}
    legacy_set = {s[4:] for s in legacy.sections() if s.startswith("env:")}
    assert active_set == set(ACTIVE), (sorted(active_set-set(ACTIVE)), sorted(set(ACTIVE)-active_set))
    assert set(RETIRED).isdisjoint(active_set)
    assert set(ACTIVE+RETIRED) == legacy_set, "Archive lost old environments"
    # Guard the actual expanded firmware flags, not merely parent names.
    for name in ACTIVE:
        assert effective_flags(active, name) == effective_flags(legacy, name), (
            "Active environment changed effective flags: " + name
        )
        parent = active.get("env:"+name,"extends",fallback="")
        if parent:
            assert parent.startswith("env:") and parent[4:] in active_set, (
                name, "dangling active extends", parent
            )
    main = effective_flags(active, "genericSTM32F411CE")
    assert "-DIR_FILTER_DIAG=1" not in main
    assert "-DBLE_PROFILE_DIAG=1" not in main
    assert "-DINTEGRATION_DIAG_MOTOR_ON=1" not in main
    assert "-DENCODER_SPEED_PID_DIAG=1" not in main
    assert "-DCLOCK_VALIDATE_96M=1" in main
    assert "-DIR_FILTER_DIAG=1" in effective_flags(active, "ir_burst_diag_96m")
    assert "-DIR_BURST_DIAG=1" in effective_flags(active, "ir_burst_diag_96m")
    assert "-DBLE_RX_DIAG=1" in effective_flags(active, "ble_rx_diag_96m")
    print("PLATFORMIO_MODES_CLEANUP_PASS: %d active, %d archived-only, identical flags for retained profiles" %
          (len(ACTIVE), len(RETIRED)))

if __name__ == "__main__":
    test_cleanup()
