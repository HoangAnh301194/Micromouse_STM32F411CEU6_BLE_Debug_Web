#!/usr/bin/env python3
"""Compile the actual src/encoder.c against isolated STM32 register stubs."""
from __future__ import annotations
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]

def main():
    compiler=shutil.which(os.environ.get("CC","gcc"))
    if not compiler:
        print("FAIL: GCC required for native encoder regression")
        return 2
    with tempfile.TemporaryDirectory(prefix="encoder-native-") as directory:
        exe=Path(directory)/("encoder_native.exe" if os.name=="nt" else "encoder_native")
        command=[
            compiler,"-std=c11","-O2","-Wall","-Wextra","-Werror",
            "-I",str(ROOT/"tests/encoder/stubs"),
            "-I",str(ROOT/"include"),
            str(ROOT/"src/encoder.c"),
            str(ROOT/"tests/encoder/test_encoder.c"),
            "-lm","-o",str(exe),
        ]
        subprocess.run(command,cwd=ROOT,check=True,timeout=90)
        subprocess.run([str(exe)],cwd=ROOT,check=True,timeout=90)
    return 0

if __name__=="__main__":
    raise SystemExit(main())
