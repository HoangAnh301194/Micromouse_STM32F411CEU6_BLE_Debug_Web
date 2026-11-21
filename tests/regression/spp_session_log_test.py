"""Regression for automatic SPP run/session log separation."""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from spp_session_log import SessionLog

def main():
    with TemporaryDirectory() as d:
        x=SessionLog(Path(d))
        x.open_session("COM8")
        x.add_line("CALIB OK")
        x.add_line("MPU,ms=1000")
        x.add_line("W,START,mm=720")
        run=x.run_path
        x.add_line("IR,ms=2000,L0=100")
        x.add_line("W,END,DONE")
        assert x.run is None
        x.add_line("Mode 1: Calibration")
        connection=x.connection_path
        x.close()
        assert "CALIB OK" in connection.read_text(encoding="utf-8")
        assert "Mode 1:" in connection.read_text(encoding="utf-8")
        assert "IR,ms=2000" in run.read_text(encoding="utf-8")
        assert "Mode 1:" not in run.read_text(encoding="utf-8")
        x.open_session("COM8")
        x.add_line("W,START,mm=720")
        incomplete=x.run_path
        x.close()
        assert "# DISCONNECTED" in incomplete.read_text(encoding="utf-8")
    print("spp_session_log: PASS")

if __name__ == "__main__":
    main()
