"""Tests for event/data split and automatic dual log files."""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from spp_dual_log import DualSessionLog, classify_spp_line

def main():
    for line in ("MPU,ms=1", "IR,ms=1", "ENC,ms=1,l=2", "ENC_CSV,1,2,3",
                 "CSV,1,2", "W,1,2,3,4,5,6,7,8,9"):
        assert classify_spp_line(line) == "data", line
    for line in ("SPP CONNECTED", "Mode 7: System Test",
                 "Selected mode 7: System Test", "W,MENU_READY", "W,END,DONE",
                 "STATUS,TEST START", "UNKNOWN"):
        assert classify_spp_line(line) == "status", line
    with TemporaryDirectory() as temp:
        log = DualSessionLog(Path(temp))
        log.open_session("COM16")
        paths = dict(log.paths)
        log.write("Mode 7: System Test")
        log.write("ENC_CSV,1,2,3")
        log.close()
        assert "Mode 7" in paths["status"].read_text(encoding="utf-8")
        assert "ENC_CSV" not in paths["status"].read_text(encoding="utf-8")
        assert "ENC_CSV" in paths["data"].read_text(encoding="utf-8")
        log.open_session("COM16")
        assert log.paths["status"] != paths["status"]
        log.close()
    print("spp_dual_log: PASS")

if __name__ == "__main__":
    main()
