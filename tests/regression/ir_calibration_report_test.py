import csv
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
            w.writerow(["t_ms","L0","L45","L90","R0","R45","R90"])
            for n in range(10):w.writerow([20*n]+[n*100+i for i in range(6)])
        out=write_ir_report(p,{i:(110+i,450+i) for i in range(6)})
        assert out.exists()
        assert p.with_name("run_ir_plot.svg").exists()
        assert "CANDIDATE_ONLY" in out.read_text()
    print("ir_calibration_report_test PASS")
if __name__=="__main__":main()
