"""Regression entry point retained for existing tooling."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).with_name("ir_calibration_live_test.py")), run_name="__main__")
