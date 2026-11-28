"""Regression for stationary Mode-1 wall/open capture and SPP data isolation."""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools"))
from spp_dual_log import DualSessionLog, classify_spp_line, sensor_test_csv

WALL="CAL_IR,stage=0,ms=123,L0=120,L45=230,L90=340,R0=450,R45=560,R90=670"
OPEN="CAL_IR,stage=1,ms=3123,L0=20,L45=30,L90=40,R0=50,R45=60,R90=70"
HEADER="t_ms,stage,L0,L45,L90,R0,R45,R90"
assert sensor_test_csv(WALL)[1]==tuple(HEADER.split(","))
assert sensor_test_csv(OPEN)[2][1]=="1"
assert classify_spp_line("CAL_LIVE,valid=1,L0=12,L45=23,L90=34,R0=45,R45=56,R90=67")=="status"

with TemporaryDirectory() as d:
    logger=DualSessionLog(d)
    logger.open_session("COM16")
    logger.write("Selected mode 1: Calibration")
    logger.write("CAL,START,mode=1,stages=2,samples=150")
    first=logger.write(WALL)
    assert first and first.read_text().splitlines()==[HEADER,"123,0,120,230,340,450,560,670"]
    logger.write("CAL,STAGE_DONE,stage=WALL,samples=150")
    logger.write("CAL,WAIT,stage=OPEN,action=MOVE_TO_OPEN_SPACE,confirm=O_OR_KEY")
    assert logger.write(OPEN)==first
    logger.write("CAL,REVIEW,samples_wall=150,samples_open=150,quality=CANDIDATE_ONLY")
    logger.write("CAL,SAVED,flash=OK,auto_apply=0,has_open_calib=1")
    logger.write("CAL,END,result=SUCCESS,quality=WALL_OPEN,flash=SAVED,auto_apply=0")
    assert logger.write("CAL_LIVE,valid=1,L0=12,L45=23,L90=34,R0=45,R45=56,R90=67") is None
    assert len(first.read_text().splitlines())==3
    logger.close()

cal=(ROOT/"src/ir_calibration_360.c").read_text(encoding="utf-8")
header=(ROOT/"include/ir_calibration_360.h").read_text(encoding="utf-8")
runtime=(ROOT/"src/main_runtime.c").read_text(encoding="utf-8")
resources=(ROOT/"src/mode_test_resources.c").read_text(encoding="utf-8")
dashboard=(ROOT/"tools/classic_spp_dashboard.py").read_text(encoding="utf-8")
assert "IRC_CAPTURE_WALL" in cal and "IRC_CAPTURE_OPEN" in cal
assert "IRC_ConfirmOpen" in cal and "CAL,WAIT,stage=OPEN" in cal
assert "IRC_MIN_SEPARATION_ADC" in cal and "wall_p10" in cal and "open_p90" in cal
assert "data.has_open_calib = 1U" in cal
finish = cal.split("static void IRC_FinishOpenStage", 1)[1].split("void IRC_Update", 1)[0]
assert "IRC_Save(" not in finish
assert "CAL,WAIT_SAVE" in finish
assert "save=USER_CONFIRM" in cal
assert "CAL,END,result=SUCCESS" in cal.split("uint8_t IRC_Save(", 1)[1]
assert 'self.cal_phase="REVIEW"' in dashboard
assert 'self.cal_phase in ("REVIEW", "END")' in dashboard
assert "was_calibration && event == MODE_MENU_EVENT_SELECT" in runtime
assert "CAL,RESTART_ACCEPTED" in runtime
assert "EncoderOneBox_StartCalibrationPivot" not in cal
assert "MPU6050_GetSnapshot" not in cal
assert "IRC_Update(&ir_calibration,now)" in runtime
assert "IRC_ConfirmOpen(&ir_calibration, now)" in runtime
assert "command == 'O'" in runtime
assert "return TEST_RES_IR;" in resources
assert "Confirm open-space capture (O)" in dashboard
assert 'line.startswith("CAL,START,mode=1,stages=2")' in dashboard
assert 'line.startswith("CAL,WAIT,stage=OPEN")' in dashboard
assert "self.cal_phase=\"WAIT_OPEN\"" in dashboard
assert "cal_phase not in (\"WALL\", \"OPEN\")" in dashboard
assert "provisional_wall_threshold" not in dashboard
print("ir_calibration_live_test PASS")
