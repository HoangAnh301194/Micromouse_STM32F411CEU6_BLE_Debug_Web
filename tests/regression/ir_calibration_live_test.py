"""Regression: Mode-1 CSV isolation and live 6-channel firmware/UI contracts.

Run from repository root. Does not replace the embedded build or bench test.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools"))
from spp_dual_log import DualSessionLog, classify_spp_line, sensor_test_csv

CAL="CAL_IR,ms=123,L0=120,L45=230,L90=340,R0=450,R45=560,R90=670"
LIVE="CAL_LIVE,valid=1,L0=12,L45=23,L90=34,R0=45,R45=56,R90=67"
HEADER="t_ms,L0,L45,L90,R0,R45,R90"
assert sensor_test_csv(CAL)[1]==tuple(HEADER.split(","))
assert classify_spp_line(LIVE)=="status"  # GUI consumes it before this dispatcher
with TemporaryDirectory() as d:
    logger=DualSessionLog(d)
    logger.open_session("COM16")
    logger.write("Selected mode 1: Calibration")
    logger.write("CAL,START,mode=1,segments=4")
    first=logger.write(CAL)
    assert first and first.read_text().splitlines()==[HEADER,"123,120,230,340,450,560,670"]
    logger.write("CAL,REVIEW,samples=120,quality=CANDIDATE_ONLY")
    logger.write("CAL,SAVED,flash=OK,auto_apply=0")
    assert logger.write(LIVE) is None
    assert first.read_text().splitlines()==[HEADER,"123,120,230,340,450,560,670"]
    logger.write("CAL,START,mode=1,segments=4")
    second=logger.write(CAL)
    assert second and second!=first
    assert len(second.read_text().splitlines())==2
    logger.close()
driver=(ROOT/"src/encoder_onebox.c").read_text(encoding="utf-8")
cal=(ROOT/"src/ir_calibration_360.c").read_text(encoding="utf-8")
runtime=(ROOT/"src/main_runtime.c").read_text(encoding="utf-8")
dashboard=(ROOT/"tools/classic_spp_dashboard.py").read_text(encoding="utf-8")
assert "CAL_PIVOT_SPEED_PPS 800" in driver
assert "StartMotion(mpu,target,CAL_PIVOT_SPEED_PPS)" in driver
assert "EncoderOneBox_StartCalibrationPivot(mpu,90)" in cal
assert "if(c->valid && !IRC_Save(c))" in cal
assert "event == MODE_MENU_EVENT_FOCUS" in runtime
assert "IRC_Arm(&ir_calibration,now,1U)" in runtime
assert "CAL_LIVE,valid=%u" in runtime
assert "if line.startswith(\"CAL_LIVE,\"):" in dashboard
assert "self.cal_wall_state=[False]*6" in dashboard
assert "self.cal_live_valid" in dashboard
print("ir_calibration_live_test PASS")
