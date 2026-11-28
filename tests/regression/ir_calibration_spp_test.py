"""Stationary wall/open Mode-1 SPP routing and CSV regression."""
import queue
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from spp_dual_log import (
    DualSessionLog, sensor_test_csv, classify_spp_line,
    is_menu_navigation_line, provisional_wall_threshold,
)
from classic_spp_dashboard import SPPReader


def main():
    wall = "CAL_IR,stage=0,ms=123,L0=120,L45=230,L90=340,R0=450,R45=560,R90=670"
    opened = "CAL_IR,stage=1,ms=3123,L0=20,L45=30,L90=40,R0=50,R45=60,R90=70"
    parsed = sensor_test_csv(wall)
    assert parsed == (
        "CAL",
        ("t_ms","stage","L0","L45","L90","R0","R45","R90"),
        ("123","0","120","230","340","450","560","670"),
    )
    assert sensor_test_csv(opened)[2] == ("3123","1","20","30","40","50","60","70")
    assert sensor_test_csv("CAL_IR,stage=2,ms=1,L0=1,L45=1,L90=1,R0=1,R45=1,R90=1") is None
    assert classify_spp_line(wall) == "data"
    assert classify_spp_line("CAL,REVIEW,samples_wall=150,samples_open=150,quality=CANDIDATE_ONLY") == "status"

    assert is_menu_navigation_line("Mode 1: Calibration")
    assert is_menu_navigation_line("Selected mode 1: Calibration")
    for event in (
        "Encoder Test: choose Log Receive / motion tests / Back",
        "Encoder Test option 1: Log Receive",
        "MPU Test: choose MPU Log / Pivot / Back",
        "MPU Test option 1: MPU Log",
        "Selected encoder option 1: Log Receive",
        "Selected mpu option 1: MPU Log",
        "Menu snapshot: Encoder Test | focus=Log Receive | selected=None | stream=0 | cal=0",
        "CAL,START,mode=1,stages=2", "CAL,WAIT,stage=OPEN",
        "CAL,REVIEW,quality=CANDIDATE_ONLY", "CAL,COUNTDOWN,seconds=5",
        "CAL,RESTART_ACCEPTED", "CAL,SAVED,flash=OK", "CAL,SAVE_REJECT,reason=NOT_READY",
        "CAL,END,result=SUCCESS", "CAL,EXIT",
    ):
        assert is_menu_navigation_line(event), event
        assert classify_spp_line(event) == "status", event
    assert not is_menu_navigation_line(wall)
    assert not is_menu_navigation_line("CAL_LIVE,valid=1")
    assert not is_menu_navigation_line("CAL,THRESH,index=0")
    assert not is_menu_navigation_line("CAL,STAGE_DONE,stage=WALL,samples=150")
    assert provisional_wall_threshold([100] * 25) is None
    assert provisional_wall_threshold(list(range(100, 200))) == (133, 165)
    assert provisional_wall_threshold([4096] * 30) is None

    with TemporaryDirectory() as tmp:
        logger = DualSessionLog(tmp)
        logger.open_session("COM16")
        assert logger.write(wall) is None
        logger.write("Selected mode 1: Calibration")
        assert logger.write(wall) is None
        logger.write("CAL,START,mode=1,stages=2,samples=150")
        first = logger.write(wall)
        assert first and first.read_text().splitlines() == [
            "t_ms,stage,L0,L45,L90,R0,R45,R90",
            "123,0,120,230,340,450,560,670",
        ]
        # WAIT_OPEN is a pause for the operator, not the end of the same run.
        logger.write("CAL,STAGE_DONE,stage=WALL,samples=150")
        logger.write("CAL,WAIT,stage=OPEN,action=MOVE_TO_OPEN_SPACE,delay_ms=5000")
        assert logger.write(opened) == first
        assert len(first.read_text().splitlines()) == 3
        logger.write("CAL,REVIEW,samples_wall=150,samples_open=150,quality=CANDIDATE_ONLY")
        assert logger.write(wall) is None
        logger.write("CAL,END,result=SUCCESS,quality=WALL_OPEN,flash=SAVED")
        assert logger.write(opened) is None

        logger.write("Selected mode 1: Calibration")
        logger.write("CAL,START,mode=1,stages=2,samples=150")
        second = logger.write(wall)
        assert second and second != first
        logger.write("CAL,END,result=FAILED,reason=KEY_STOP")
        assert logger.write(opened) is None
        # Old 360-degree protocol is not accepted as a Mode-1 run.
        logger.write("Selected mode 1: Calibration")
        logger.write("CAL,START,mode=1,segments=4")
        assert logger.write(wall) is None
        logger.close()

    class ActiveThread:
        def is_alive(self):
            return True

    reader = SPPReader(queue.Queue())
    reader.thread = ActiveThread()
    assert reader.send_menu(b"R") is True
    assert reader.commands.get_nowait() == b"R"
    assert reader.send_menu(b"O") is True
    assert reader.commands.get_nowait() == b"O"
    assert reader.send_menu(b"V") is True
    assert reader.commands.get_nowait() == b"V"
    assert reader.send_menu(b"X") is False
    print("ir_calibration_spp_test PASS")


if __name__ == "__main__":
    main()
