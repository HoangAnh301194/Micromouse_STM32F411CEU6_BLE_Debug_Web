"""Regression for explicit Encoder Log Receive Start/Stop and DATA-only CSV."""
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
    t0 = "2026-11-23T15:33:40.852"
    t1 = "2026-11-23T15:33:56.848"
    t2 = "2026-11-23T15:34:05.123"
    t3 = "2026-11-23T15:34:20.123"
    old_frame = (
        "ENC,ms=128650,dt_ms=50,left=2620,right=4230,delta_l=0,"
        "delta_r=0,pps_l=0,pps_r=0,window_l=0,window_r=0"
    )
    assert classify_spp_line("Selected encoder option 1: Log Receive") == "status"
    assert classify_spp_line("TEST STREAM STARTED: Encoder Test") == "status"
    assert classify_spp_line("ENC,0,-5") == "data"
    assert sensor_test_csv(old_frame, host_time=t0) == (
        "ENC", ("timestamp", "ENC_L", "ENC_R"), (t0, "2620", "4230"))
    assert sensor_test_csv("ENC,0,-5", host_time=t0)[2] == (t0, "0", "-5")
    assert TEST_CSV_COLUMNS["ENC"] == ("timestamp", "ENC_L", "ENC_R")
    assert sensor_test_csv("ENC,bad,1") is None
    onebox_frame = "ONEBOX_CSV,ms=500,left=2400,right=2390,pwm_l=175,pwm_r=160"
    assert classify_spp_line(onebox_frame) == "data"
    assert classify_spp_line("ONEBOX,START,target_l=2400") == "status"
    assert sensor_test_csv(onebox_frame, host_time=t0) == (
        "ONEBOX", ("timestamp", "t_ms", "ENC_L", "ENC_R", "PWM_L", "PWM_R", "REF_L_PPS", "REF_R_PPS", "SPEED_L_PPS", "SPEED_R_PPS"),
        (t0, "500", "2400", "2390", "175", "160", "", "", "", ""))
    assert sensor_test_csv(onebox_frame + ",ref_l=1100,ref_r=1100,speed_l=1050,speed_r=1090", host_time=t0)[2][-4:] == ("1100", "1100", "1050", "1090")
    assert sensor_test_csv("ONEBOX_CSV,ms=bad,left=1,right=2,pwm_l=3,pwm_r=4") is None

    with TemporaryDirectory() as temp:
        root = Path(temp) / "logs" / "spp_runs"
        log = DualSessionLog(root)
        log.open_session("COM16")
        for msg in (
            "SPP CONNECTED",
            "BOOT,READY,MPU=OK",
            "Selected mode 7: System Test",
            "System Test: choose IR / Encoder / MPU / Back",
            "System Test 2: Encoder Test",
            "Encoder Test: choose Log Receive / motion tests / Back",
            "Selected test 2: Encoder Test",
            "Encoder Test option 2: 1 Box Forward",
            "Selected encoder option 2: 1 Box Forward (NOT IMPLEMENTED; MOTOR LOCKED)",
        ):
            log.write(msg, host_time=t0)
        assert log.write("ENC,9,9", host_time=t0) is None
        assert not root.exists()  # No status files, no accidental CSV.

        log.write("Encoder Test option 1: Log Receive")
        log.write("Selected encoder option 1: Log Receive")
        assert log.write("ENC,10,11", host_time=t0) is None
        assert not root.exists()  # Select = ARMED, not RECORDING.

        # Start #1 immediately creates a fresh timestamp-named CSV.
        log.write("TEST STREAM STARTED: Encoder Test", host_time=t0)
        folder = root / "mode 7 - encoder test - log receiver"
        e1 = folder / "20261123_153340_852000_COM16.csv"
        assert e1.is_file()
        assert read_csv(e1) == [["timestamp", "ENC_L", "ENC_R"]]
        assert log.write("ENC,0,-5", host_time=t0) == e1
        assert log.write("ENC,12,13", host_time=t1) == e1
        assert log.write("IR,ms=1,valid=1,L0=1,L45=2,L90=3,R0=4,R45=5,R90=6") is None
        assert log.write("MPU,valid=1,yaw_cd=10,gz_cd=20") is None
        assert log.write("ENC,bad,10") is None

        # Stop closes and saves the first take. Repeating Stop is inert.
        log.write("TEST STREAM STOPPED: Encoder Test", host_time=t1)
        assert log.last_saved_path == e1
        assert log._active_file is None
        assert log.write("ENC,777,888", host_time=t1) is None
        log.write("TEST STREAM STOPPED: Encoder Test", host_time=t1)
        assert list(folder.glob("*.csv")) == [e1]
        assert read_csv(e1) == [
            ["timestamp", "ENC_L", "ENC_R"],
            [t0, "0", "-5"],
            [t1, "12", "13"],
        ]

        # Start #2 produces another FILE inside the SAME static folder.
        log.write("TEST STREAM STARTED: Encoder Test", host_time=t2)
        e2 = folder / "20261123_153405_123000_COM16.csv"
        assert e2.is_file() and e2 != e1
        assert log.write(old_frame, host_time=t2) == e2
        log.write("TEST STREAM STOPPED: Encoder Test", host_time=t3)
        assert log.last_saved_path == e2
        assert read_csv(e2) == [
            ["timestamp", "ENC_L", "ENC_R"],
            [t2, "2620", "4230"],
        ]
        assert len(list(folder.glob("*.csv"))) == 2
        assert read_csv(e1)[-1] == [t1, "12", "13"]  # Must not append.

        # Quick Start->Stop with zero samples still saves a header-only CSV.
        log.write("TEST STREAM STARTED: Encoder Test", host_time=t2)
        e3 = folder / "20261123_153405_123000_COM16_01.csv"
        assert e3.is_file()
        log.write("TEST STREAM STOPPED: Encoder Test", host_time=t2)
        assert read_csv(e3) == [["timestamp", "ENC_L", "ENC_R"]]
        assert len(list(folder.glob("*.csv"))) == 3

        # Leaving Log Receive closes the current take and blocks stray ENC.
        log.write("Encoder Test option 3: 5 Box Forward")
        log.write("Selected encoder option 3: 5 Box Forward (NOT IMPLEMENTED; MOTOR LOCKED)")
        assert log.write("ENC,100,100", host_time=t2) is None
        assert len(list(folder.glob("*.csv"))) == 3
        # A one-box selection is idle; no file until an MCU START ACK.
        log.write("Encoder Test option 2: 1 Box Forward")
        log.write("Selected encoder option 2: 1 Box Forward (ARMED; press Start)")
        assert log.write(onebox_frame, host_time=t0) is None
        assert log.write("ONEBOX,REJECT,reason=MOTOR_NOT_READY", host_time=t0) is None
        motion_dir = root / "mode 7 - encoder test - 1 box forward"
        assert not motion_dir.exists()
        log.write("ONEBOX,START,target_l=2400,target_r=2390,pwm_max=200",
                  host_time=t0)
        motion1 = motion_dir / "20261123_153340_852000_COM16.csv"
        assert motion1.exists()
        assert read_csv(motion1) == [
            ["timestamp", "t_ms", "ENC_L", "ENC_R", "PWM_L", "PWM_R", "REF_L_PPS", "REF_R_PPS", "SPEED_L_PPS", "SPEED_R_PPS"]]
        assert log.write("ONEBOX_CSV,ms=0,left=100,right=150,pwm_l=0,pwm_r=0",
                         host_time=t0) == motion1
        assert log.write(onebox_frame, host_time=t1) == motion1
        assert log.write("ENC,999,999", host_time=t1) is None
        assert log.write("MPU,valid=0", host_time=t1) is None
        assert log.write("ONEBOX_CSV,ms=601,left=2501,right=2491,pwm_l=0,pwm_r=0",
                         host_time=t2) == motion1
        log.write("ONEBOX,RESULT,reason=DONE,L=2401,R=2391,ms=501,locked=1",
                  host_time=t2)
        assert log.last_saved_path == motion1 and log._active_file is None
        assert log.write(onebox_frame, host_time=t2) is None
        assert read_csv(motion1) == [
            ["timestamp", "t_ms", "ENC_L", "ENC_R", "PWM_L", "PWM_R", "REF_L_PPS", "REF_R_PPS", "SPEED_L_PPS", "SPEED_R_PPS"],
            [t0, "0", "100", "150", "0", "0", "", "", "", ""],
            [t1, "500", "2400", "2390", "175", "160", "", "", "", ""],
            [t2, "601", "2501", "2491", "0", "0", "", "", "", ""]]

        # Second motor Start must produce ANOTHER file in the same folder,
        # even when the previous take ended due to emergency Stop.
        log.write("ONEBOX,START,target_l=2400,target_r=2390,pwm_max=200",
                  host_time=t3)
        motion2 = motion_dir / "20261123_153420_123000_COM16.csv"
        assert motion2.is_file() and motion2 != motion1
        log.write("ONEBOX_CSV,ms=15,left=-5,right=-9,pwm_l=120,pwm_r=125",
                  host_time=t3)
        log.write("ONEBOX,RESULT,reason=STOP,L=-5,R=-9,ms=15,locked=1",
                  host_time=t3)
        assert log.last_saved_path == motion2
        assert read_csv(motion2) == [
            ["timestamp", "t_ms", "ENC_L", "ENC_R", "PWM_L", "PWM_R", "REF_L_PPS", "REF_R_PPS", "SPEED_L_PPS", "SPEED_R_PPS"],
            [t3, "15", "-5", "-9", "120", "125", "", "", "", ""]]
        assert read_csv(motion1)[-1][4:6] == ["0", "0"]
        assert len(list(motion_dir.glob("*.csv"))) == 2
        log.write("Encoder Test: back to System Test")

        # IR/MPU retained their original, isolated per-test folder behavior.
        log.write("System Test 3: MPU Test")
        log.write("Selected test 3: MPU Test")
        assert log.write("MPU,valid=1,yaw_cd=9000,gz_cd=0", host_time=t2) is None
        log.write("TEST STREAM STARTED: MPU Test", host_time=t2)
        m1 = log.write("MPU,ms=50,valid=1,age_us=25,yaw_cd=12,gz_cd=-2", host_time=t2)
        assert m1.parent.name == "mode 7 - mpu test - log receiver"
        assert read_csv(m1) == [["timestamp", "mcu_ms", "valid", "yaw_deg", "age_us"], [t2, "50", "1", "0.12", "25"]]
        log.write("MPU,ms=100,valid=0,age_us=10000")
        assert read_csv(m1)[-1][1:] == ["100", "0", "", "10000"]
        log.write("TEST STREAM STOPPED: MPU Test", host_time=t2)
        assert log.last_saved_path == m1
        log.write("TEST STREAM STARTED: MPU Test", host_time=t3)
        m2 = log.write("MPU,ms=200,valid=1,age_us=30,yaw_cd=-9000,gz_cd=0", host_time=t3)
        assert m2 != m1 and m2.parent == m1.parent and read_csv(m2)[-1][3] == "-90.00"
        log.write("TEST STREAM STOPPED: MPU Test", host_time=t3)
        log.write("System Test 1: IR Test")
        log.write("Selected test 1: IR Test")
        i1 = log.write(
            "IR,ms=150,valid=1,age_us=25,scan=11,"
            "L0=10,L45=11,L90=12,R0=13,R45=14,R90=15", host_time=t2)
        assert i1.parent.name.startswith("Mode 7 - IR Test - ")
        assert read_csv(i1) == [
            ["valid", "L0", "L45", "L90", "R0", "R45", "R90"],
            ["1", "10", "11", "12", "13", "14", "15"],
        ]

        log.write("MPU Test: choose MPU Log / Pivot / Back")
        log.write("Selected mpu option 2: Pivot 90 Left")
        pivot="PIVOT_CSV,ms=100,valid=1,age_us=850,yaw_cd=9500,angle_cd=9000,gz_cd=100,target=90,left=-900,right=920,pwm_l=-188,pwm_r=188,ref_l=-1100,ref_r=1100,speed_l=-1050,speed_r=1090"
        assert log.write(pivot,host_time=t0) is None
        log.write("PIVOT,START,target=90",host_time=t0)
        p1=log.write(pivot,host_time=t0)
        assert p1 and p1.parent.name == "mode 7 - mpu test - pivot 90 left"
        assert read_csv(p1)[-1][3:10]==["95.00","850","90.00","1.00","90","-900","920"]
        log.write("PIVOT,RESULT,reason=DONE",host_time=t1)
        assert log.last_saved_path==p1 and log.write(pivot,host_time=t1) is None

        # Combined sensor stream is allowed only in the All Sensors mode
        # and is stored as one CSV, never mixed into Encoder/IR/MPU files.
        log.write("System Test 4: All Sensors Test")
        log.write("Selected test 4: All Sensors Test")
        all_line = ("ALL,ms=200,enc_l=2400,enc_r=2390,ir_valid=1,"
                    "L0=10,L45=11,L90=12,R0=13,R45=14,R90=15,"
                    "mpu_valid=0,yaw_cd=0,gz_cd=0")
        combined = log.write(all_line, host_time=t3)
        assert combined and combined.parent.name.startswith(
            "Mode 7 - All Sensors Test - ")
        assert read_csv(combined) == [
            ["timestamp","ENC_L","ENC_R","ir_valid","L0","L45","L90",
             "R0","R45","R90","mpu_valid","yaw_cd","gz_cd"],
            [t3,"2400","2390","1","10","11","12","13","14","15","0","0","0"]]
        log.write("System Test: back to main menu")
        assert log.write(all_line, host_time=t3) is None

        assert not list(root.rglob("*_status.log"))
        assert not list(root.rglob("connections"))
        log.close()

        # New COM session cannot resume an encoder log until Log Receive
        # and a brand-new Start have been acknowledged.
        log.open_session("COM16")
        assert log.write("ENC,100,-100", host_time=t0) is None
        log.write("Selected encoder option 1: Log Receive")
        assert log.write("ENC,100,-100", host_time=t0) is None
        log.write("TEST STREAM STARTED: Encoder Test", host_time=t3)
        e4 = log.write("ENC,100,-100", host_time=t3)
        assert e4.parent == folder and e4 != e1 and e4 != e2 and e4 != e3
        log.write("TEST STREAM STOPPED: Encoder Test")
        log.close()
        assert len(list(folder.glob("*.csv"))) == 4

    print("spp_dual_log: PASS — Encoder Start/Stop creates independent timestamped CSV files")


if __name__ == "__main__":
    main()
