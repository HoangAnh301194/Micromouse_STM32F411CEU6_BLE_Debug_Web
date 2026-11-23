"""Regression: isolated per-sensor System Test CSV and legacy dual SPP logs."""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
import csv

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from spp_dual_log import (DualSessionLog, classify_spp_line, data_to_csv_row,
                          sensor_test_csv, TEST_CSV_COLUMNS)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.reader(f))


def main():
    for line in ("MPU,ms=1", "IR,ms=1", "ENC,12,-13",
                 "ENC,ms=1,l=2", "ENC_CSV,1,2,3",
                 "CSV,1,2", "W,1,2,3,4,5,6,7,8,9"):
        assert classify_spp_line(line) == "data", line
    for line in ("SPP CONNECTED", "Mode 7: System Test",
                 "Selected mode 7: System Test", "W,MENU_READY", "W,END,DONE",
                 "System Test 2: Encoder Test", "Selected test 2: Encoder Test",
                 "System Test: back to main menu", "STATUS,TEST START", "UNKNOWN"):
        assert classify_spp_line(line) == "status", line

    legacy = data_to_csv_row("MPU,ms=1000,valid=1,yaw_cd=250,gz_cd=-40")
    assert legacy["yaw_cd"] == "250" and legacy["L0"] == ""
    legacy = data_to_csv_row("IR,ms=1000,L0=12,L45=20,L90=0,R0=13,R45=1,R90=0")
    assert legacy["L45"] == "20" and legacy["R0"] == "13"
    legacy = data_to_csv_row("ENC,ms=1200,dt_ms=50,left=12,right=-13,delta_l=4,delta_r=-5,pps_l=80,pps_r=-100,window_l=80,window_r=-100")
    assert legacy["delta_l"] == "4" and legacy["delta_r"] == "-5"
    assert legacy["window_l"] == "80" and legacy["dt_ms"] == "50"
    legacy = data_to_csv_row("ENC_CSV,1100,12,13,120,125")
    assert legacy["left"] == "12" and legacy["pps_r"] == "125"

    # New compact encoder frames contain only the signed quadrature counters.
    record = sensor_test_csv("ENC,123,-45")
    assert record == ("ENC", ("ENC_L", "ENC_R"), ("123", "-45"))
    assert data_to_csv_row("ENC,123,-45")["left"] == "123"
    assert data_to_csv_row("ENC,123,-45")["right"] == "-45"
    assert sensor_test_csv("ENC,-2147483648,2147483647")[2] == (
        "-2147483648", "2147483647")
    assert sensor_test_csv("ENC,abc,10") is None
    assert sensor_test_csv("ENC,12,13,14") is None
    assert sensor_test_csv("ENC,12") is None
    assert sensor_test_csv("ENC,ms=1200,left=123,right=-45,pps_l=999")[2] == ("123", "-45")

    # The sensor-specific test tables cannot contain columns from other tests.
    ir = sensor_test_csv("IR,ms=1,valid=1,age_us=20,scan=3,L0=1,L45=2,L90=3,R0=4,R45=5,R90=6")
    assert ir == ("IR", TEST_CSV_COLUMNS["IR"], ("1", "1", "2", "3", "4", "5", "6"))
    mpu = sensor_test_csv("MPU,ms=1,valid=1,age_us=10,yaw_cd=123,gz_cd=-99")
    assert mpu == ("MPU", TEST_CSV_COLUMNS["MPU"], ("1", "123", "-99"))
    invalid_mpu = sensor_test_csv("MPU,ms=5,valid=0,age_us=9999")
    assert invalid_mpu[2] == ("0", "", "")
    assert sensor_test_csv("MPU,valid=1") is None
    assert sensor_test_csv("W,1,2,3,4,5,6,7,8,9") is None
    assert TEST_CSV_COLUMNS["ENC"] == ("ENC_L", "ENC_R")
    assert not any("IR" in c or "MPU" in c for c in TEST_CSV_COLUMNS["ENC"])

    with TemporaryDirectory() as temp:
        log = DualSessionLog(Path(temp))
        log.open_session("COM16")
        base = dict(log.paths)
        log.write("Selected test 2: Encoder Test")
        log.write("ENC,11,-12")
        log.write("ENC,13,-15")
        log.write("System Test 3: MPU Test")
        log.write("Selected test 3: MPU Test")
        log.write("MPU,ms=50,valid=1,age_us=25,yaw_cd=12,gz_cd=-2")
        log.write("MPU,ms=100,valid=0,age_us=10000")
        log.write("Selected test 1: IR Test")
        log.write("IR,ms=150,valid=1,age_us=25,scan=11,L0=10,L45=11,L90=12,R0=13,R45=14,R90=15")
        log.write("ENC_CSV,1,2,3")  # Old wire protocol still supported.
        encoder, ir_path, mpu_path = (log.paths["encoder"], log.paths["ir"],
                                       log.paths["mpu"])
        log.close()

        assert "Selected test 2" in base["status"].read_text(encoding="utf-8")
        assert "ENC,11" not in base["status"].read_text(encoding="utf-8")
        assert read_csv(encoder) == [
            ["ENC_L", "ENC_R"], ["11", "-12"], ["13", "-15"]]
        assert read_csv(ir_path) == [
            ["valid", "L0", "L45", "L90", "R0", "R45", "R90"],
            ["1", "10", "11", "12", "13", "14", "15"]]
        assert read_csv(mpu_path) == [
            ["valid", "yaw_cd", "gz_cd"], ["1", "12", "-2"], ["0", "", ""]]
        with base["data"].open(newline="", encoding="utf-8") as f:
            legacy_rows = list(csv.DictReader(f))
        assert len(legacy_rows) == 1
        assert legacy_rows[0]["type"] == "ENC_CSV"
        assert legacy_rows[0]["raw"] == "ENC_CSV,1,2,3"
        assert "IR" not in ",".join(read_csv(encoder))

        log.open_session("COM16")
        assert log.paths["status"] != base["status"]
        assert "encoder" not in log.paths  # Lazy: no file before test starts.
        log.close()
    print("spp_dual_log: PASS")


if __name__ == "__main__":
    main()
