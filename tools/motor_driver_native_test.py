#!/usr/bin/env python3
"""Compile and execute the actual motor.c with host register mocks; no hardware."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    compiler = shutil.which(os.environ.get("CC", "gcc"))
    if compiler is None:
        raise SystemExit("A native C compiler is required: install GCC or set CC.")
    with tempfile.TemporaryDirectory(prefix="motor-native-") as directory:
        executable = Path(directory) / ("motor_test.exe" if os.name == "nt" else "motor_test")
        command = [
            compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "tests/motor_driver/stubs"),
            "-I", str(ROOT / "include"),
            str(ROOT / "src/motor.c"), str(ROOT / "src/motor_test.c"),
            str(ROOT / "tests/motor_driver/test_motor_driver.c"),
            "-o", str(executable),
        ]
        subprocess.run(command, cwd=ROOT, check=True, timeout=60)
        subprocess.run([str(executable)], cwd=ROOT, check=True, timeout=30)
        subprocess.run([str(executable), "--watchdog"], cwd=ROOT, check=True, timeout=30)
        subprocess.run([str(executable), "--handoff-matrix"], cwd=ROOT, check=True, timeout=30)
        startup_executable = Path(directory) / ("startup_test.exe" if os.name == "nt" else "startup_test")
        startup_command = command[:-3] + [str(ROOT / "src/motor_startup.c"),
            str(ROOT / "tests/motor_driver/test_motor_startup.c"), "-o", str(startup_executable)]
        subprocess.run(startup_command, cwd=ROOT, check=True, timeout=60)
        for scenario in ("normal", "budget", "link", "wrong_wheel", "reverse", "session", "stop"):
            subprocess.run([str(startup_executable), scenario], cwd=ROOT, check=True, timeout=30)
        speed_command = startup_command[:1] + ["-DMOTOR_STARTUP_HIGH_SPEED=1"] + startup_command[1:]
        subprocess.run(speed_command, cwd=ROOT, check=True, timeout=60)
        for scenario in ("high_stall", "high_both", "high_link"):
            subprocess.run([str(startup_executable), scenario], cwd=ROOT, check=True, timeout=30)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
