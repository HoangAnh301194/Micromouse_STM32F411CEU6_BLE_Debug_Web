"""Native synthetic tests for the no-motor 90/180 turn trace analysis."""
import csv
import json
import tempfile
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from turn_pid_pipeline import parse_sample,parse_kv,parse_imu,analyze,write_report,zip_session


def sample(t,yaw,target=90,phase=1):
    return {
        "run_id":1,"t_ms":t,"target_deg":target,"profile":0,"phase":phase,
        "yaw_deg":yaw,"error_deg":target-yaw,"gyro_dps":0.0,
        "ref_dps":0.0,"pid_pm":0,"ff_pm":0,"cmd_pm":0,
        "enc_l":0,"enc_r":0,"imu_age_us":500
    }


class TurnTraceTests(unittest.TestCase):
    def test_parse_exact_firmware_csv(self):
        line="TP,D,1,20,90,0,1,100,8900,50,60,70,80,90,10,-10,500"
        data=parse_sample(line)
        self.assertEqual(data["run_id"],1)
        self.assertAlmostEqual(data["yaw_deg"],1.0)
        self.assertEqual(data["cmd_pm"],90)
        self.assertEqual(data["enc_r"],-10)
        self.assertEqual(data["imu_age_us"],500)

    def test_imu_frequency_and_gap_metrics(self):
        r=parse_imu("TP,IMU,id=2,t=1000,win=1000,new=976,hz10=9760,dtavg=1025,dtmin=930,dtmax=2100,g1500=7,g2000=1,miss=0,nr=12,err=2,age=620,seq=3021,last=0")
        self.assertEqual(r["run_id"],2)
        self.assertEqual(r["effective_hz"],976.0)
        self.assertEqual(r["interval_max_us"],2100)
        self.assertIsNone(parse_imu("TP,IMU,id=2,new=970"))
        rows=[dict(sample(t,40),run_id=2) for t in range(0,1200,20)]
        summary=analyze(rows,{"reason":"DONE","stop_ms":"400"},[r])
        self.assertEqual(summary["imu_effective_hz"],976.0)
        self.assertEqual(summary["imu_gap_gt2000"],1)
        self.assertEqual(summary["imu_i2c_errors"],2)

    def test_reject_partial_and_unsafe_value(self):
        self.assertIsNone(parse_sample("TP,D,1,20,90,0,1,1"))
        self.assertIsNone(parse_sample("TP,D,1,20,90,0,1,100,8900,50,60,70,80,900,10,-10,500"))
        self.assertIsNone(parse_sample("TP,D,1,20,90,0,1,NaN,8900,50,60,70,80,90,10,-10,500"))

    def test_overshoot_coast_drift_and_settle(self):
        times=[0,20,40,60,80,100,120,140,160,180,200,220,240,260,280,300,320,340,360]
        angles=[0,5,14,28,40,55,70,82,92,93,91,90.5,90.3,90.2,90.1,90.0,90.0,90.0,90.0]
        rows=[sample(t,y,phase=1 if t<180 else 2) for t,y in zip(times,angles)]
        report=analyze(rows,{"reason":"DONE","stop_ms":"180",
                             "qdrop":"0","uartdrop":"0"})
        self.assertEqual(report["overshoot_deg"],3)
        self.assertEqual(report["rise_10_90_ms"],report["rise_10_90_ms"])
        self.assertGreater(report["rise_10_90_ms"],0)
        self.assertAlmostEqual(report["coast_drift_deg"],-3)
        self.assertEqual(report["settling_ms"],200)
        self.assertEqual(report["final_abs_error_deg"],0)
        self.assertEqual(report["sampling_gap_max_ms"],20)

    def test_never_settles_and_missing_crossing(self):
        rows=[sample(t,yaw,target=180) for t,yaw in
              [(0,0),(20,20),(40,50),(60,80),(80,110),(100,145),
               (120,140),(140,150),(160,150),(180,150),(200,150)]]
        report=analyze(rows,{"reason":"TIMEOUT","stop_ms":"200"})
        self.assertIsNone(report["t_first_target_ms"])
        self.assertIsNone(report["settling_ms"])
        self.assertEqual(report["overshoot_deg"],0)
        self.assertEqual(report["reason"],"TIMEOUT")
        self.assertEqual(report["final_abs_error_deg"],30)

    def test_persist_complete_run(self):
        rows=[sample(t,0.5*t) for t in range(0,260,20)]
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            reports=write_report(folder,{1:rows},{1:{"reason":"DONE","stop_ms":"200"}})
            self.assertEqual(len(reports),1)
            self.assertTrue((folder/"raw_uart.log").exists() is False)
            self.assertTrue((folder/"group_stats.json").exists())
            self.assertTrue(zip_session(folder).exists())
            self.assertTrue((folder/"run_001"/"samples.csv").exists())
            self.assertTrue((folder/"run_001"/"summary.json").exists())
            with (folder/"summary.csv").open(newline="",encoding="utf-8") as f:
                records=list(csv.DictReader(f))
            self.assertEqual(len(records),1)
            self.assertEqual(records[0]["reason"],"DONE")
            self.assertEqual(json.loads((folder/"summary.json").read_text())[0]["trace_samples"],13)

    def test_kv_header(self):
        d=parse_kv("TP,RESULT,id=3,mode=90,reason=DONE,stop_ms=800","TP,RESULT,")
        self.assertEqual(d["id"],"3")
        self.assertEqual(d["stop_ms"],"800")


if __name__=="__main__":
    unittest.main(verbosity=2)
