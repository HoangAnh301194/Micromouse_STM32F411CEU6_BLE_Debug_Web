#!/usr/bin/env python3
"""Compile actual 145s integration_sweep.c twice with MCU stubs, never touch hardware."""
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

def main():
    compiler = shutil.which(os.environ.get("CC", "gcc"))
    if compiler is None:
        raise SystemExit("Native C compiler (gcc) is required")
    with tempfile.TemporaryDirectory(prefix="stm32-sweep-") as work:
        for mode in ("OFF", "ON", "LIMITED"):
            exe = Path(work) / ("sweep_" + mode.lower() + (".exe" if os.name == "nt" else ""))
            cmd = [
                compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-DINTEGRATION_DIAG=1",
                "-DINTEGRATION_DIAG_MOTOR_ON=1" if mode=="LIMITED" else
                f"-DINTEGRATION_DIAG_MOTOR_{mode}=1",
                *(["-DINTEGRATION_DIAG_LIMITED_PWM_30=1", "-DENCODER_ALPHA_AB_DIAG=1"]
                  if mode=="LIMITED" else []),
                "-I", str(ROOT / "tests" / "integration" / "stubs"),
                str(ROOT / "src" / "integration_sweep.c"),
                str(ROOT / "tests" / "integration" / "test_sweep_native.c"),
                "-o", str(exe),
            ]
            subprocess.run(cmd, cwd=ROOT, check=True, timeout=60)
            subprocess.run([str(exe)], cwd=ROOT, check=True, timeout=60)
            print(f"PASS {mode}: actual C sweep "
                  f"{12 if mode=='LIMITED' else 33} stages + STOP/stale MPU "
                  "and TIM11 fault" + (" + UART loss" if mode!="OFF" else ""))
if __name__ == "__main__":
    main()
