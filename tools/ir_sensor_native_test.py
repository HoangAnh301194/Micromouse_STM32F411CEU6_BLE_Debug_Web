#!/usr/bin/env python3
"""Compile and run the real IR driver against host register mocks."""
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
    with tempfile.TemporaryDirectory(prefix="ir-sensor-native-") as directory:
        executable = Path(directory) / ("ir_sensor_test.exe" if os.name == "nt"
                                        else "ir_sensor_test")
        command = [
            compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-DINTEGRATION_DIAG=1",
            "-I", str(ROOT / "tests/ir_sensor/stubs"),
            "-I", str(ROOT / "include"),
            str(ROOT / "src/ir_sensor.c"),
            str(ROOT / "tests/ir_sensor/test_ir_sensor.c"),
            "-o", str(executable),
        ]
        subprocess.run(command, cwd=ROOT, check=True, timeout=60)
        subprocess.run([str(executable)], cwd=ROOT, check=True, timeout=30)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
