"""Native synthetic tests for the no-motor 90/180 turn trace analysis."""
import csv
import json
import tempfile
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from turn_pid_pipeline import parse_sample,parse_kv,parse_imu,analyze,write_report,zip_session,offline,motor_sweep_steps


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

    def test_new_rev3_pwm_applied_trace_and_tuned(self):
        rec=parse_sample("TP,D,4,20,90,2,1,1090,7890,250,1200,90,110,330,-310,307,-210,190,751,0")
        self.assertIsNotNone(rec)
        self.assertEqual(rec["profile"],2)
        self.assertEqual(rec["applied_l_pm"],-310)
        self.assertEqual(rec["applied_r_pm"],307)
        self.assertEqual(rec["motor_stage"],0)
        self.assertEqual(rec["enc_l"],-210)
        self.assertIsNone(parse_sample("TP,D,4,20,90,2,1,1090,7890,250,1200,90,110,999,-310,307,-210,190,751,0"))

    def test_motor_sweep_step_metrics_and_archive(self):
        levels=[70,95,120,155,200,260]
        recs=[]
        for t in range(20,1500,20):
            stage=min(t//250,5)
            pwm=levels[stage] if t%250<150 else 0
            # Synthetic counter values increase only while command active.
            frame=("TP,D,1,%d,0,0,1,%d,%d,%d,0,0,0,%d,%d,%d,%d,%d,500,%d"
                   %(t,t*2,-t*2,pwm*3,pwm,-pwm,pwm,-(t//4),t//4,stage))
            item=parse_sample(frame)
            self.assertIsNotNone(item,frame)
            recs.append(item)
        steps=motor_sweep_steps(recs)
        self.assertEqual(len(steps),6)
        self.assertEqual(steps[0]["pwm_requested_pm"],70)
        self.assertEqual(steps[-1]["pwm_requested_pm"],260)
        summary=analyze(recs,{"reason":"DONE","stop_ms":"1500"})
        self.assertEqual(summary["profile"],"SWEEP")
        self.assertEqual(summary["motor_sweep_steps"],6)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            reports=write_report(root,{1:recs},{1:{"reason":"DONE","stop_ms":"1500"}})
            self.assertEqual(len(reports),1)
            self.assertTrue((root/"run_001"/"motor_steps.csv").exists())
            self.assertNotIn("0deg_SWEEP",json.loads((root/"group_stats.json").read_text()))

    def test_aborted_run_excluded_from_repeated_turn_stats(self):
        a=[sample(t,float(t)/20) for t in range(20,220,20)]
        b=[dict(z,run_id=2) for z in a]
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            write_report(root,{1:a,2:b},{
                1:{"reason":"PA0_STOP","stop_ms":"200"},
                2:{"reason":"DONE","stop_ms":"200"}})
            self.assertEqual(json.loads((root/"group_stats.json").read_text())["90deg_BASE"]["runs"],1)

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

    def test_offline_does_not_create_zip_unless_requested(self):
        line="TP,D,1,20,90,0,1,100,8900,50,60,70,80,90,10,-10,500"
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            (folder/"raw_uart.log").write_text(line+"\n",encoding="utf-8")
            self.assertEqual(offline(folder),0)
            self.assertFalse((folder/"turn_pid_bundle.zip").exists())
            self.assertTrue((folder/"run_001"/"samples.csv").exists())
            self.assertEqual(offline(folder,make_zip=True),0)
            self.assertTrue((folder/"turn_pid_bundle.zip").exists())

    def test_kv_header(self):
        d=parse_kv("TP,RESULT,id=3,mode=90,reason=DONE,stop_ms=800","TP,RESULT,")
        self.assertEqual(d["id"],"3")
        self.assertEqual(d["stop_ms"],"800")


if __name__=="__main__":
    unittest.main(verbosity=2)
