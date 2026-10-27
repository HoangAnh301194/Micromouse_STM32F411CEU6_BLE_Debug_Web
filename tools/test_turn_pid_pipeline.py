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
        kick=parse_sample("TP,D,4,200,90,2,1,8700,300,0,0,0,150,150,-150,150,-260,240,751,6")
        self.assertEqual(kick["motor_stage"],6)
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

    def test_v4_smooth_terminal_stage_and_metrics(self):
        def frame(t,yaw,cmd,stage,phase=1):
            return parse_sample(
                f"TP,D,7,{t},90,3,{phase},{int(yaw*100)},100,"
                f"10,20,0,0,{cmd},{-cmd},{cmd},-50,50,500,{stage}")
        rows=[frame(20,30,210,0),frame(40,80,110,7),
              frame(60,87,108,7),frame(80,88.3,0,8),
              frame(100,89.0,0,8),frame(280,89.0,0,8,phase=2)]
        self.assertTrue(all(z is not None for z in rows))
        report=analyze(rows,{"reason":"DONE","stop_ms":"100"})
        self.assertEqual(report["profile"],"SMOOTH")
        self.assertEqual(report["terminal_80_to_stop_ms"],60)
        self.assertEqual(report["smooth_approach_samples"],2)
        self.assertEqual(report["kick_samples"],0)
        self.assertEqual(report["reverse_pwm_samples"],0)
        self.assertIsNone(parse_sample("TP,D,7,20,90,3,1,100,200,10,20,0,0,100,-100,100,-50,50,500,9"))

    def test_v5_rapid_80pct_ceiling_braking_and_peak_from_1khz(self):
        def rec(t, yaw, cmd, stage, phase=1):
            cd=int(yaw*100)
            return parse_sample(
                f"TP,D,8,{t},90,4,{phase},{cd},{9000-cd},"
                f"500,2000,100,250,{cmd},{-cmd},{cmd},-50,50,500,{stage}")
        records=[rec(20,8,780,9),rec(40,40,550,9),
                 rec(60,72,-310,10),rec(80,83,100,11),
                 rec(100,88.5,0,12),rec(320,89.1,0,12,phase=2)]
        self.assertTrue(all(r is not None for r in records))
        summary=analyze(records,{"reason":"DONE","stop_ms":"100",
                                 "peak_dps10":"4805"})
        self.assertEqual(summary["profile"],"RAPID")
        self.assertEqual(summary["max_requested_pwm_pm"],780)
        self.assertEqual(summary["rapid_accel_samples"],2)
        self.assertEqual(summary["rapid_brake_samples"],1)
        self.assertEqual(summary["rapid_near_samples"],1)
        self.assertEqual(summary["rapid_coast_samples"],1)
        self.assertEqual(summary["peak_gyro_1khz_dps"],480.5)
        self.assertEqual(summary["reverse_pwm_samples"],1)
        self.assertIsNone(rec(20,8,801,9))
        self.assertIsNone(parse_sample(
            "TP,D,8,20,90,3,1,800,8200,500,2000,100,250,780,-780,780,-50,50,500,9"))
        self.assertIsNone(parse_sample(
            "TP,D,8,20,90,4,1,800,8200,500,2000,100,250,780,-780,780,-50,50,500,13"))

    def test_v6_flow_faster_90deg_monotonic_stages(self):
        def make(t,yaw,cmd,stage,phase=1):
            cd=int(yaw*100)
            return parse_sample(
                f"TP,D,10,{t},90,5,{phase},{cd},{9000-cd},"
                f"1200,2400,30,40,{cmd},{-cmd},{cmd},-50,50,540,{stage}")
        samples=[make(20,3,680,13),make(40,33,620,13),
                 make(60,59,-210,14),make(80,73,0,15),
                 make(100,83,110,16),make(120,88.8,0,17),
                 make(300,88.8,0,17,phase=2)]
        self.assertTrue(all(s is not None for s in samples))
        report=analyze(samples,{"reason":"DONE","stop_ms":"120",
                                "peak_dps10":"5412"})
        self.assertEqual(report["profile"],"FLOW")
        self.assertEqual(report["flow_accel_samples"],2)
        self.assertEqual(report["flow_brake_samples"],1)
        self.assertEqual(report["flow_coast_samples"],1)
        self.assertEqual(report["flow_approach_samples"],1)
        self.assertEqual(report["flow_finish_samples"],1)
        self.assertEqual(report["rapid_or_flow_brake_reaccelerations"],0)
        self.assertEqual(report["max_requested_pwm_pm"],680)
        self.assertEqual(report["peak_gyro_1khz_dps"],541.2)
        # Detection of the exact path that jerked during RAPID.
        damaged=[samples[0],samples[2],make(70,70,770,13),*samples[3:]]
        report2=analyze(damaged,{"reason":"DONE","stop_ms":"120"})
        self.assertGreater(report2["rapid_or_flow_brake_reaccelerations"],0)
        self.assertIsNone(make(20,3,801,13))
        self.assertIsNone(make(20,3,680,12))
        self.assertIsNone(parse_sample(
          "TP,D,10,20,90,4,1,300,8700,1200,2400,30,40,680,-680,680,-50,50,540,13"))

    def test_missing_initial_trace_and_true_terminal_1khz_yaw(self):
        # Run1 of the 2026-10-27 rapid session lost startup UART frames
        # but formerly met trace quality; do not fabricate a t10 crossing.
        rows=[sample(t,30.0+0.12*(t-140),phase=1 if t<780 else 2)
              for t in range(140,1600,20)]
        report=analyze(rows,{"reason":"DONE","stop_ms":"785",
                            "final_cd":"8856","peak_cd":"8858"})
        self.assertIsNone(report["t10_ms"])
        self.assertEqual(report["trace_first_ms"],140)
        self.assertFalse(report["meets_trace_thresholds"])
        self.assertEqual(report["final_yaw_deg"],88.56)
        # Run4: RESULT final is 74.12deg, not the previous 50Hz 76.38deg.
        motion=[sample(20,20.0),sample(2075,87.42,phase=2),
                sample(2855,76.38,phase=2)]
        m=analyze(motion,{"reason":"STALLED_NEAR_TARGET",
                          "stop_ms":"2075","final_cd":"7412",
                          "peak_cd":"8753"})
        self.assertEqual(m["final_yaw_deg"],74.12)
        self.assertEqual(m["final_abs_error_deg"],15.88)
        self.assertGreater(m["coast_drift_deg"].__abs__(),13)
        self.assertGreater(m["trace_terminal_disagreement_deg"],2.0)
        self.assertFalse(m["meets_trace_thresholds"])

    def test_v7_glide_stage_monotonicity_and_braking_metrics(self):
        def frame(t,yaw,cmd,stage,phase=1):
            cd=int(yaw*100)
            return parse_sample(
                f"TP,D,12,{t},90,6,{phase},{cd},{9000-cd},"
                f"1400,1600,0,0,{cmd},{-cmd},{cmd},-120,120,700,{stage}")
        rows=[frame(20,1,200,18),frame(40,10,420,18),
              frame(60,44,260,19),frame(80,67,-160,19),
              frame(100,76,0,20),frame(120,80,110,21),
              frame(140,88.5,0,22),frame(300,88.5,0,22,phase=2)]
        self.assertTrue(all(row is not None for row in rows))
        metrics=analyze(rows,{"reason":"DONE","stop_ms":"140",
                              "peak_dps10":"5101","final_cd":"8850"})
        self.assertEqual(metrics["profile"],"GLIDE")
        self.assertEqual(metrics["glide_accel_samples"],2)
        self.assertEqual(metrics["glide_brake_samples"],2)
        self.assertEqual(metrics["glide_coast_samples"],1)
        self.assertEqual(metrics["glide_approach_samples"],1)
        self.assertEqual(metrics["glide_finish_samples"],1)
        self.assertEqual(metrics["brake_first_yaw_deg"],44.0)
        self.assertEqual(metrics["max_cmd_delta_20ms"],420)
        self.assertEqual(metrics["rapid_or_flow_brake_reaccelerations"],0)
        self.assertEqual(metrics["peak_gyro_1khz_dps"],510.1)
        self.assertIsNone(frame(20,1,810,18))
        self.assertIsNone(frame(20,1,200,13))
        self.assertIsNone(parse_sample(
          "TP,D,12,20,90,5,1,100,8900,1400,1600,0,0,200,-200,200,-120,120,700,18"))

    def test_v8_sprint_full_pwm_and_180deg_telemetry(self):
        def frame(t, angle, cmd, stage, phase=1, target=180, profile=7):
            cd=int(angle*100)
            return parse_sample(
                f"TP,D,16,{t},{target},{profile},{phase},{cd},"
                f"{target*100-cd},2500,4500,80,100,{cmd},"
                f"{-cmd},{cmd},-80,80,730,{stage}")
        r=[frame(20,0,500,23),frame(40,42,1000,23),
           frame(60,106,690,23),frame(80,155,-240,24),
           frame(100,172,0,25),frame(120,177,125,26),
           frame(140,178.9,0,27),frame(300,178.9,0,27,phase=2)]
        self.assertTrue(all(z is not None for z in r))
        a=analyze(r,{"reason":"DONE","stop_ms":"140",
                     "final_cd":"17890","peak_cd":"17893",
                     "peak_dps10":"7100"})
        self.assertEqual(a["profile"],"SPRINT")
        self.assertEqual(a["target_deg"],180)
        self.assertEqual(a["max_requested_pwm_pm"],1000)
        self.assertEqual(a["sprint_accel_samples"],3)
        self.assertEqual(a["sprint_brake_samples"],1)
        self.assertEqual(a["sprint_coast_samples"],1)
        self.assertEqual(a["sprint_approach_samples"],1)
        self.assertEqual(a["sprint_finish_samples"],1)
        self.assertEqual(a["rapid_or_flow_brake_reaccelerations"],0)
        self.assertEqual(a["brake_first_yaw_deg"],155.0)
        self.assertIsNotNone(a["terminal_last10_to_stop_ms"])
        self.assertEqual(a["peak_gyro_1khz_dps"],710)
        self.assertIsNone(frame(20,0,1001,23))
        self.assertIsNone(frame(20,0,1000,19))
        self.assertIsNone(frame(20,0,1000,23,profile=6))

    def test_after_stop_heading_drift_must_fail(self):
        # A real GLIDE run in latest GitHub logs reported DONE at stop
        # but drifted 4.6deg before the 800ms post-stop record finished.
        rows=[sample(20,10),sample(220,80),
              sample(400,89,phase=1),sample(500,89,phase=2),
              sample(1200,85.4,phase=2)]
        a=analyze(rows,{"reason":"POST_STOP_DRIFT",
                        "stop_ms":"500","final_cd":"8540"})
        self.assertEqual(a["reason"],"POST_STOP_DRIFT")
        self.assertEqual(a["final_abs_error_deg"],4.6)
        self.assertFalse(a["meets_trace_thresholds"])
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp)
            write_report(d,{1:rows},{1:{"reason":"POST_STOP_DRIFT",
                                        "stop_ms":"500","final_cd":"8540"}})
            self.assertEqual(json.loads((d/"group_stats.json").read_text()),{})

    def test_track_continuous_rate_90_and_180_metrics(self):
        def frame(t,angle,rate,ref,cmd,stage,target=90,phase=1):
            cd=int(angle*100)
            return parse_sample(
               f"TP,D,22,{t},{target},8,{phase},{cd},{target*100-cd},"
               f"{int(rate*10)},{int(ref*10)},0,0,{cmd},"
               f"{-cmd},{cmd},-100,100,600,{stage}")
        rows=[frame(20,2,60,100,360,28),
              frame(40,21,350,500,840,28),
              frame(60,53,515,430,-160,29),
              frame(80,74,270,265,-80,29),
              frame(100,83,145,130,100,30),
              frame(120,88.6,14,20,0,31),
              frame(300,88.8,0,0,0,31,phase=2)]
        self.assertTrue(all(z is not None for z in rows))
        a=analyze(rows,{"reason":"DONE","stop_ms":"120","peak_dps10":"5300"})
        self.assertEqual(a["profile"],"TRACK")
        self.assertEqual(a["track_accel_samples"],2)
        self.assertEqual(a["track_brake_samples"],2)
        self.assertEqual(a["track_near_samples"],1)
        self.assertEqual(a["track_settle_samples"],1)
        self.assertEqual(a["midturn_low_speed_samples"],0)
        self.assertEqual(a["midturn_reverse_samples"],0)
        self.assertEqual(a["min_midturn_gyro_dps"],270.0)
        self.assertIsNone(frame(20,2,60,100,1001,28))
        self.assertIsNone(frame(20,2,60,100,360,26))
        rows180=[frame(20,0,30,100,360,28,target=180),
                 frame(40,75,670,700,1000,28,target=180),
                 frame(60,125,520,510,-180,29,target=180),
                 frame(80,161,300,280,50,30,target=180),
                 frame(100,179.1,10,0,0,31,target=180),
                 frame(280,179.1,0,0,0,31,target=180,phase=2)]
        b=analyze(rows180,{"reason":"DONE","stop_ms":"100",
                           "final_cd":"17910","peak_dps10":"7050"})
        self.assertEqual(b["target_deg"],180)
        self.assertEqual(b["profile"],"TRACK")
        self.assertEqual(b["max_requested_pwm_pm"],1000)
        self.assertEqual(b["track_brake_samples"],1)

    def test_track_metric_catches_actual_sprint_stall_at_72deg(self):
        # Regression based on uploaded SPRINT 20261027_024310:
        # gyro crossed zero near yaw 72deg and then relaunched.
        series=[dict(sample(20,1),profile=7,gyro_dps=40,motor_stage=23),
                dict(sample(120,32),profile=7,gyro_dps=570,motor_stage=24),
                dict(sample(200,70),profile=7,gyro_dps=280,motor_stage=24),
                dict(sample(240,72.9),profile=7,gyro_dps=-10.7,motor_stage=26),
                dict(sample(260,72.8),profile=7,gyro_dps=-1.5,motor_stage=26),
                dict(sample(300,74),profile=7,gyro_dps=74,motor_stage=26),
                dict(sample(540,88.6),profile=7,gyro_dps=0,motor_stage=27),
                dict(sample(800,88.6,phase=2),profile=7,gyro_dps=0,motor_stage=27)]
        s=analyze(series,{"reason":"DONE","stop_ms":"540"})
        self.assertEqual(s["midturn_low_speed_samples"],2)
        self.assertEqual(s["midturn_reverse_samples"],1)  # -10.7<-3, but -1.5 is not
        self.assertEqual(s["min_midturn_gyro_dps"],-10.7)

    def test_v10_guard_90_and_180_rate_sensitive_stopping(self):
        def rec(t,yaw,rate,cmd,stage,angle=90,phase=1):
            cd=round(yaw*100)
            return parse_sample(
              f"TP,D,24,{t},{angle},9,{phase},{cd},{angle*100-cd},"
              f"{round(rate*10)},0,0,0,{cmd},{-cmd},{cmd},"
              f"-110,110,710,{stage}")
        q=[rec(20,2,60,250,32),rec(40,40,380,740,32),
           rec(60,72,180,-90,33),rec(80,85,56,50,34),
           rec(100,88.4,33,-100,36),
           rec(120,88.9,7,0,35),
           rec(320,88.9,0,0,35,phase=2)]
        self.assertTrue(all(x is not None for x in q))
        a=analyze(q,{"reason":"DONE","stop_ms":"120",
                      "final_cd":"8890","peak_dps10":"4120"})
        self.assertEqual(a["profile"],"GUARD")
        self.assertEqual(a["guard_forward_samples"],2)
        self.assertEqual(a["guard_brake_samples"],1)
        self.assertEqual(a["guard_near_samples"],1)
        self.assertEqual(a["guard_active_band_brake_samples"],1)
        self.assertEqual(a["guard_stop_samples"],1)
        self.assertEqual(a["in_target_band_high_rate_samples"],1)
        self.assertEqual(a["first_band_rate_dps"],33.0)
        self.assertEqual(a["max_requested_pwm_pm"],740)
        self.assertIsNone(rec(20,2,60,751,32))
        self.assertIsNone(rec(20,2,60,250,29))
        v=[rec(20,4,80,300,32,angle=180),
           rec(40,110,525,850,32,angle=180),
           rec(60,168,160,-155,33,angle=180),
           rec(80,178.2,7,0,35,angle=180),
           rec(300,178.2,0,0,35,angle=180,phase=2)]
        self.assertTrue(all(x is not None for x in v))
        b=analyze(v,{"reason":"DONE","stop_ms":"80","final_cd":"17820"})
        self.assertEqual(b["profile"],"GUARD")
        self.assertEqual(b["target_deg"],180)
        self.assertEqual(b["max_requested_pwm_pm"],850)
        self.assertIsNone(rec(20,4,80,851,32,angle=180))

    def test_v11_pace_90_180_stage_metrics_and_pwm_limits(self):
        def record(t,yaw,rate,cmd,stage,target=90,phase=1):
            centi=round(100*yaw)
            return parse_sample(
               f"TP,D,28,{t},{target},10,{phase},{centi},"
               f"{target*100-centi},{round(rate*10)},"
               f"{round(rate*10)},0,0,{cmd},{-cmd},{cmd},"
               f"-100,100,700,{stage}")
        rows=[record(20,1,35,220,37),record(40,23,360,820,37),
              record(60,65,310,-120,38),record(80,83,70,130,39),
              record(100,88.5,7,0,40),
              record(300,88.5,0,0,40,phase=2)]
        self.assertTrue(all(x is not None for x in rows))
        s=analyze(rows,{"reason":"DONE","stop_ms":"100",
                        "final_cd":"8850","peak_dps10":"5320"})
        self.assertEqual(s["profile"],"PACE")
        self.assertEqual(s["pace_drive_samples"],2)
        self.assertEqual(s["pace_brake_samples"],1)
        self.assertEqual(s["pace_near_samples"],1)
        self.assertEqual(s["pace_stop_samples"],1)
        self.assertEqual(s["pace_band_brake_samples"],0)
        self.assertEqual(s["max_requested_pwm_pm"],820)
        self.assertEqual(s["final_abs_error_deg"],1.5)
        self.assertIsNone(record(20,1,35,851,37))
        self.assertIsNone(record(20,1,35,220,34))
        rows180=[record(20,1,35,240,37,target=180),
                 record(40,65,560,920,37,target=180),
                 record(60,155,230,-140,38,target=180),
                 record(80,178.1,44,-100,41,target=180),
                 record(100,178.7,6,0,40,target=180),
                 record(300,178.7,0,0,40,target=180,phase=2)]
        self.assertTrue(all(x is not None for x in rows180))
        a=analyze(rows180,{"reason":"DONE","stop_ms":"100",
                            "final_cd":"17870"})
        self.assertEqual(a["profile"],"PACE")
        self.assertEqual(a["pace_band_brake_samples"],1)
        self.assertEqual(a["first_band_rate_dps"],44.0)
        self.assertIsNone(record(40,65,560,921,37,target=180))

    def test_coast_v12_90_180_no_reverse_and_stage_decoder(self):
        def sample_frame(t,angle,gyro,cmd,stage,target=90,phase=1):
            centi=round(angle*100)
            return parse_sample(
               f"TP,D,31,{t},{target},11,{phase},{centi},"
               f"{target*100-centi},{round(gyro*10)},"
               f"{round(gyro*10)},0,0,{cmd},{-cmd},{cmd},"
               f"-100,100,650,{stage}")
        rows=[sample_frame(20,1,55,300,42),
              sample_frame(40,25,400,830,42),
              sample_frame(60,58,320,0,43),
              sample_frame(80,80,90,110,44),
              sample_frame(100,88.4,24,0,46),
              sample_frame(120,88.5,8,0,45),
              sample_frame(300,88.5,0,0,45,phase=2)]
        self.assertTrue(all(v is not None for v in rows))
        m=analyze(rows,{"reason":"DONE","stop_ms":"120",
                         "peak_dps10":"5550","final_cd":"8850"})
        self.assertEqual(m["profile"],"COAST")
        self.assertEqual(m["coast_drive_samples"],2)
        self.assertEqual(m["coast_neutral_samples"],1)
        self.assertEqual(m["coast_near_samples"],1)
        self.assertEqual(m["coast_band_neutral_samples"],1)
        self.assertEqual(m["coast_stop_samples"],1)
        self.assertEqual(m["coast_negative_command_samples"],0)
        self.assertEqual(m["command_polarity_flips"],0)
        self.assertIsNotNone(m["applied_pwm_max_step_20ms"])
        self.assertIsInstance(m["neutral_actual_high_gyro_samples"],int)
        self.assertEqual(m["first_band_rate_dps"],24.0)
        self.assertIsNotNone(sample_frame(20,1,55,1000,42))
        self.assertIsNone(sample_frame(20,1,55,1001,42))
        self.assertIsNone(sample_frame(20,1,55,-1,43))
        self.assertIsNone(sample_frame(20,1,55,300,39))
        long=[sample_frame(20,1,55,300,42,target=180),
              sample_frame(40,75,640,940,42,target=180),
              sample_frame(60,142,300,0,43,target=180),
              sample_frame(80,173,80,125,44,target=180),
              sample_frame(100,178.5,7,0,45,target=180),
              sample_frame(280,178.5,0,0,45,target=180,phase=2)]
        self.assertTrue(all(v is not None for v in long))
        lm=analyze(long,{"reason":"DONE","stop_ms":"100",
                         "final_cd":"17850"})
        self.assertEqual(lm["target_deg"],180)
        self.assertEqual(lm["profile"],"COAST")
        self.assertEqual(lm["max_requested_pwm_pm"],940)
        self.assertIsNotNone(sample_frame(40,75,640,1000,42,target=180))
        self.assertIsNone(sample_frame(40,75,640,1001,42,target=180))

    def test_coast_v13_fast_configuration_rate_and_pwm(self):
        firmware=(Path(__file__).resolve().parent.parent/"src"/"turn_pid_diag.c").read_text()
        for token in (
            "#define TP_COAST_RATE_90           700.0f",
            "#define TP_COAST_RATE_180          800.0f",
            "#define TP_COAST_PWM_90           1000.0f",
            "#define TP_COAST_PWM_180          1000.0f",
            "turn_pid_diag_v13_coast_fast",
        ):
            self.assertIn(token,firmware)

    def test_coast_uart_profile_selection_matches_index_11(self):
        # Regression: TP_COAST is already the motor coast phase (2).
        # UART 'c' must select the separate motion profile index 11.
        firmware=(Path(__file__).resolve().parent.parent/"src"/"turn_pid_diag.c").read_text()
        self.assertIn("TP_COAST_PROFILE=11",firmware)
        self.assertIn("if(c=='c')tp.profile=TP_COAST_PROFILE;",firmware)
        self.assertIn("profiles[TP_COAST_PROFILE]",firmware)
        self.assertNotIn("if(c=='c')tp.profile=TP_COAST;",firmware)

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
