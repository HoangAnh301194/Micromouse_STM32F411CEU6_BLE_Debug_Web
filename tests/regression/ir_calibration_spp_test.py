"""Mode 1 isolated CSV regression."""
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from spp_dual_log import DualSessionLog,sensor_test_csv,classify_spp_line
def main():
    line="CAL_IR,ms=123,L0=120,L45=230,L90=340,R0=450,R45=560,R90=670"
    r=sensor_test_csv(line)
    assert r==("CAL",("t_ms","L0","L45","L90","R0","R45","R90"),("123","120","230","340","450","560","670"))
    assert classify_spp_line(line)=="data"
    assert classify_spp_line("CAL,REVIEW,samples=123,quality=CANDIDATE_ONLY")=="status"
    with TemporaryDirectory() as tmp:
        s=DualSessionLog(tmp);s.open_session("COM16")
        assert s.write(line) is None
        s.write("Selected mode 1: Calibration")
        assert s.write(line) is None
        s.write("CAL,START,mode=1,segments=4")
        p=s.write(line)
        assert p and p.read_text().splitlines()==["t_ms,L0,L45,L90,R0,R45,R90","123,120,230,340,450,560,670"]
        s.write("CAL,REVIEW,samples=123,quality=CANDIDATE_ONLY")
        assert s.write(line) is None
        s.write("Selected mode 1: Calibration");s.write("CAL,START,mode=1,segments=4")
        p2=s.write(line);assert p2 and p2!=p
        s.write("CAL,FAULT,reason=KEY_STOP");assert s.write(line) is None
    print("ir_calibration_spp_test PASS")
if __name__=="__main__":main()
