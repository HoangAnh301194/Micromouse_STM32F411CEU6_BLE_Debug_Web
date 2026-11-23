"""Regression for per-run Mode 7 DATA-only CSV storage."""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
import csv

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from spp_dual_log import (
    DualSessionLog, classify_spp_line, sensor_test_csv, TEST_CSV_COLUMNS,
)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.reader(file))


def main():
    old_frame = (
        "ENC,ms=128650,dt_ms=50,left=2620,right=4230,delta_l=0,"
        "delta_r=0,pps_l=0,pps_r=0,window_l=0,window_r=0"
    )
    t0 = "2026-11-23T15:33:40.852"
    t1 = "2026-11-23T15:33:56.848"
    assert classify_spp_line("Selected test 2: Encoder Test") == "status"
    assert classify_spp_line("ENC,0,-5") == "data"
    assert sensor_test_csv(old_frame, host_time=t0) == (
        "ENC", ("timestamp", "ENC_L", "ENC_R"), (t0, "2620", "4230"))
    assert sensor_test_csv("ENC,0,-5", host_time=t0)[2] == (t0, "0", "-5")
    assert TEST_CSV_COLUMNS["ENC"] == ("timestamp", "ENC_L", "ENC_R")
    assert sensor_test_csv("ENC,bad,1") is None

    with TemporaryDirectory() as temp:
        root = Path(temp) / "logs" / "spp_runs"
        log = DualSessionLog(root)
        log.open_session("COM16")
        # CONNECT without a test must create NO files.
        log.write("SPP CONNECTED")
        log.write("BOOT,READY,MPU=OK")
        log.write("Selected mode 7: System Test")
        log.write("System Test: choose IR / Encoder / MPU / Back")
        assert not root.exists()

        log.write("System Test 2: Encoder Test")  # browsing only
        assert log.write("ENC,33,44", host_time=t0) is None
        assert not root.exists()
        log.write("Selected test 2: Encoder Test")
        assert not root.exists()

        e1 = log.write("ENC,0,-5", host_time=t0)
        assert e1 and e1.name == "continuous_data.csv"
        assert e1.parent.name.startswith("Mode 7 - Encoder Test - 20261123_153340_852000_COM16")
        assert len(list(root.iterdir())) == 1
        assert log.write("IR,valid=1,L0=0,L45=1,L90=2,R0=3,R45=4,R90=5") is None
        assert log.write("MPU,valid=1,yaw_cd=5,gz_cd=6") is None
        assert log.write("STATUS,anything") is None
        assert log.write("ENC,bad,10") is None
        log.write("TEST STREAM STOPPED: Encoder Test")
        assert log.write("ENC,777,888", host_time=t0) is None
        log.write("TEST STREAM STARTED: Encoder Test")
        assert log.write(old_frame, host_time=t1) == e1
        assert read_csv(e1) == [
            ["timestamp", "ENC_L", "ENC_R"],
            [t0, "0", "-5"],
            [t1, "2620", "4230"],
        ]
        assert list(e1.parent.iterdir()) == [e1]  # no status log, no empty data.csv

        # Next focus closes the encoder run. Different test gets new folder.
        log.write("System Test 3: MPU Test")
        assert log.write("ENC,111,222", host_time=t1) is None
        log.write("Selected test 3: MPU Test")
        m1 = log.write(
            "MPU,ms=50,valid=1,age_us=25,yaw_cd=12,gz_cd=-2", host_time=t1)
        assert m1.parent.name.startswith("Mode 7 - MPU Test - ")
        assert read_csv(m1) == [
            ["valid", "yaw_cd", "gz_cd"], ["1", "12", "-2"]]
        log.write("MPU,ms=100,valid=0,age_us=10000")
        assert read_csv(m1)[-1] == ["0", "", ""]

        log.write("System Test 1: IR Test")
        log.write("Selected test 1: IR Test")
        i1 = log.write(
            "IR,ms=150,valid=1,age_us=25,scan=11,"
            "L0=10,L45=11,L90=12,R0=13,R45=14,R90=15", host_time=t1)
        assert i1.parent.name.startswith("Mode 7 - IR Test - ")
        assert read_csv(i1) == [
            ["valid", "L0", "L45", "L90", "R0", "R45", "R90"],
            ["1", "10", "11", "12", "13", "14", "15"]]

        # Reselect the same Encoder Test: it is a distinct experiment/run,
        # even when first-sample time would collide with a previous folder.
        log.write("System Test 2: Encoder Test")
        log.write("Selected test 2: Encoder Test")
        e2 = log.write("ENC,1,2", host_time=t0)
        assert e2 != e1
        assert e2.parent != e1.parent
        assert read_csv(e2) == [
            ["timestamp", "ENC_L", "ENC_R"], [t0, "1", "2"]]
        log.write("System Test: back to main menu")
        assert log.write("ENC,9,9", host_time=t0) is None
        assert len(list(root.iterdir())) == 4
        assert not list(root.rglob("*_status.log"))
        assert all(p.name == "continuous_data.csv" for p in root.rglob("*.csv"))
        assert not list(root.rglob("connections"))
        log.close()

        # A new COM session without a 'Selected test' event may be an
        # existing active MCU stream: detect the sensor from its data tag.
        log.open_session("COM16")
        assert len(list(root.iterdir())) == 4
        inferred = log.write("ENC,100,-100", host_time=t0)
        assert inferred.parent.name.startswith("Mode 7 - Encoder Test - ")
        assert inferred != e1 and inferred != e2
        log.close()
        assert len(list(root.iterdir())) == 5

        # An idle connection must leave the directory count unchanged.
        log.open_session("COM16")
        log.write("SPP CONNECTED")
        log.write("BOOT,DEGRADED,MPU_STATUS=2,STEP=1,I2C=1")
        log.close()
        assert len(list(root.iterdir())) == 5
    print("spp_dual_log: PASS")


if __name__ == "__main__":
    main()
