"""Mode 1 SPP routing, CSV isolation and retry-command regression."""
import queue
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from spp_dual_log import (
    DualSessionLog,
    sensor_test_csv,
    classify_spp_line,
    is_menu_navigation_line,
)
from classic_spp_dashboard import SPPReader


def main():
    line = "CAL_IR,ms=123,L0=120,L45=230,L90=340,R0=450,R45=560,R90=670"
    parsed = sensor_test_csv(line)
    assert parsed == (
        "CAL",
        ("t_ms", "L0", "L45", "L90", "R0", "R45", "R90"),
        ("123", "120", "230", "340", "450", "560", "670"),
    )
    assert classify_spp_line(line) == "data"
    assert classify_spp_line("CAL,REVIEW,samples=123,quality=CANDIDATE_ONLY") == "status"

    # Right terminal contains menu navigation only, not runtime telemetry.
    assert is_menu_navigation_line("Mode 1: Calibration")
    assert is_menu_navigation_line("Selected mode 1: Calibration")
    assert is_menu_navigation_line("Selected test 1: IR Test")
    assert not is_menu_navigation_line("CAL,START,mode=1,segments=4")
    assert not is_menu_navigation_line("CAL,PIVOT_START,segment=1,target=90")
    assert not is_menu_navigation_line("BOOT,READY,MPU=OK")
    assert provisional_wall_threshold([100] * 25) is None
    assert provisional_wall_threshold(list(range(100, 200))) == (129, 169)
    assert provisional_wall_threshold([4096] * 30) is None

    with TemporaryDirectory() as tmp:
        logger = DualSessionLog(tmp)
        logger.open_session("COM16")
        assert logger.write(line) is None
        logger.write("Selected mode 1: Calibration")
        assert logger.write(line) is None
        logger.write("CAL,START,mode=1,segments=4")
        first = logger.write(line)
        assert first and first.read_text().splitlines() == [
            "t_ms,L0,L45,L90,R0,R45,R90",
            "123,120,230,340,450,560,670",
        ]
        logger.write("CAL,REVIEW,samples=123,quality=CANDIDATE_ONLY")
        assert logger.write(line) is None
        logger.write("CAL,END,result=SUCCESS,quality=CANDIDATE_ONLY,flash=SAVED")
        assert logger.write(line) is None
        logger.write("Selected mode 1: Calibration")
        logger.write("CAL,START,mode=1,segments=4")
        second = logger.write(line)
        assert second and second != first
        logger.write("CAL,END,result=FAILED,reason=KEY_STOP")
        assert logger.write(line) is None
        # Legacy static calibration must not open a v3 360-degree CSV.
        logger.write("Selected mode 1: Calibration")
        logger.write("CAL,START,stage=1,samples=200")
        assert logger.write(line) is None
        logger.close()

    class ActiveThread:
        def is_alive(self):
            return True

    reader = SPPReader(queue.Queue())
    reader.thread = ActiveThread()
    assert reader.send_menu(b"R") is True
    assert reader.commands.get_nowait() == b"R"
    assert reader.send_menu(b"V") is True
    assert reader.commands.get_nowait() == b"V"
    assert reader.send_menu(b"X") is False
    print("ir_calibration_spp_test PASS")


if __name__ == "__main__":
    main()
