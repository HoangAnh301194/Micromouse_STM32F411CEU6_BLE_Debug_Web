"""Tests for event/data split and automatic dual log files."""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from spp_dual_log import DualSessionLog, classify_spp_line, data_to_csv_row
import csv

def main():
    for line in ("MPU,ms=1", "IR,ms=1", "ENC,ms=1,l=2", "ENC_CSV,1,2,3",
                 "CSV,1,2", "W,1,2,3,4,5,6,7,8,9"):
        assert classify_spp_line(line) == "data", line
    for line in ("SPP CONNECTED", "Mode 7: System Test",
                 "Selected mode 7: System Test", "W,MENU_READY", "W,END,DONE",
                 "System Test 2: Encoder Test", "Selected test 2: Encoder Test",
                 "System Test: back to main menu", "STATUS,TEST START", "UNKNOWN"):
        assert classify_spp_line(line) == "status", line
    row = data_to_csv_row("MPU,ms=1000,valid=1,yaw_cd=250,gz_cd=-40")
    assert row["yaw_cd"] == "250" and row["L0"] == ""
    row = data_to_csv_row("IR,ms=1000,L0=12,L45=20,L90=0,R0=13,R45=1,R90=0")
    assert row["L45"] == "20" and row["R0"] == "13"
    row = data_to_csv_row("ENC,ms=1200,dt_ms=50,left=12,right=-13,delta_l=4,delta_r=-5,pps_l=80,pps_r=-100,window_l=80,window_r=-100")
    assert row["delta_l"] == "4" and row["delta_r"] == "-5"
    assert row["window_l"] == "80" and row["dt_ms"] == "50"
    row = data_to_csv_row("ENC_CSV,1100,12,13,120,125")
    assert row["left"] == "12" and row["pps_r"] == "125"
    with TemporaryDirectory() as temp:
        log = DualSessionLog(Path(temp))
        log.open_session("COM16")
        paths = dict(log.paths)
        log.write("Mode 7: System Test")
        log.write("ENC_CSV,1,2,3")
        log.close()
        assert "Mode 7" in paths["status"].read_text(encoding="utf-8")
        assert "ENC_CSV" not in paths["status"].read_text(encoding="utf-8")
        with paths["data"].open(newline="", encoding="utf-8") as f:
            records = list(csv.DictReader(f))
        assert len(records) == 1 and records[0]["type"] == "ENC_CSV"
        assert records[0]["raw"] == "ENC_CSV,1,2,3"
        log.open_session("COM16")
        assert log.paths["status"] != paths["status"]
        log.close()
    print("spp_dual_log: PASS")

if __name__ == "__main__":
    main()
