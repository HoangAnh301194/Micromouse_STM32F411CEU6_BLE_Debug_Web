"""Mode 1 is a reserved menu entry; never start calibration or write CSV."""
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools"))
from spp_dual_log import DualSessionLog
runtime=(ROOT/"src/main_runtime.c").read_text(encoding="utf-8")
dashboard=(ROOT/"tools/classic_spp_dashboard.py").read_text(encoding="utf-8")
resources=(ROOT/"src/mode_test_resources.c").read_text(encoding="utf-8")
assert "IRC_Arm(" not in runtime
assert "IRC_Update(" not in runtime
assert "IRC_Save(" not in runtime
assert "self.cal_pane" not in dashboard
assert "menu->selected == 0U" not in resources
with TemporaryDirectory() as d:
    logger=DualSessionLog(d)
    logger.open_session("COM16")
    logger.write("Selected mode 1: Calibration")
    logger.write("CAL,START,mode=1,stages=2,samples=150")
    assert logger.write("CAL_IR,stage=0,ms=1,L0=1,L45=1,L90=1,R0=1,R45=1,R90=1") is None
    assert not list(Path(d).rglob("*.csv"))
    logger.close()
print("mode1_placeholder PASS")
