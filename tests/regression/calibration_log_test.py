"""Verify three independent calibration captures and retry isolation."""
import csv,sys
from pathlib import Path
from tempfile import TemporaryDirectory
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from spp_dual_log import DualSessionLog
with TemporaryDirectory() as d:
    log=DualSessionLog(d); log.open_session("COM16")
    for stage in (1,2,3,1):
        log.write(f"CAL,START,stage={stage},samples=200")
        for idx in range(1,201):
            path=log.write(f"CAL_SAMPLE,stage={stage},idx={idx},ms={idx*10},scan={idx},L0=1,L45=2,L90=3,R0=4,R45=5,R90=6")
            assert path
        log.write(f"CAL,STAGE_OK,stage={stage}")
        with path.open() as f: rows=list(csv.reader(f))
        assert len(rows)==201 and rows[-1][2]=="200"
        assert f"stage {stage}" in path.parent.name
        assert log.write(f"CAL_SAMPLE,stage={stage},idx=1,ms=0,scan=1,L0=1,L45=2,L90=3,R0=4,R45=5,R90=6") is None
    assert len(list(Path(d).rglob("*.csv")))==4
    log.close()
print("calibration_log: PASS")
