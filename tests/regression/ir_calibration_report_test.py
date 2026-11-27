import csv
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from ir_calibration_report import write_ir_report

def main():
    with TemporaryDirectory() as d:
        p=Path(d)/"run.csv"
        with p.open("w",newline="",encoding="utf8") as f:
            w=csv.writer(f)
            w.writerow(["t_ms","stage","L0","L45","L90","R0","R45","R90"])
            for n in range(150):
                w.writerow([20*n,0]+[1200+n%5+i for i in range(6)])
            for n in range(150):
                w.writerow([3000+20*n,1]+[100+n%5+i for i in range(6)])
        out=write_ir_report(p,{i:(300+i,500+i) for i in range(6)})
        assert out.exists()
        assert p.with_name("run_ir_plot.svg").exists()
        report=json.loads(out.read_text())
        assert report["calibration_method"]=="stationary_two_condition_wall_open"
        assert report["samples"]=={"WALL":150,"OPEN":150}
        assert report["channels"]["L0"]["WALL"]["n"]==150
        assert report["channels"]["L0"]["OPEN"]["n"]==150
        assert report["channels"]["L0"]["wall_on"]==500
        assert "CANDIDATE_ONLY" in out.read_text()
    print("ir_calibration_report_test PASS")

if __name__=="__main__":main()
