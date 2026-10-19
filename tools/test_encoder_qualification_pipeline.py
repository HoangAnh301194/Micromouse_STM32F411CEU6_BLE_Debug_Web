#!/usr/bin/env python3
"""Regression tests for the unified Encoder host report (no board / motor)."""
import json
from pathlib import Path
import tempfile
import unittest
import encoder_qualification_pipeline as p

def fixture_records():
    return [
        p.parse("SYS_M,ms=0,locked=1,i2cerr=0,to=0"),
        p.parse("SYS_T,ms=0,late=0,gap=0"),
        p.parse("SYS_U,ms=0,tx_drop=0"),
        p.parse("SYS_E,ms=0,encL=10,encR=20,ppsL=0,ppsR=0,appL=0,appR=0"),
        p.parse("SYS_EN,ms=20,l=12,r=21,vL=50,vR=25"),
        p.parse("SYS_EN,ms=40,l=14,r=22,vL=90,vR=45"),
        p.parse("SYS_EN,ms=60,l=16,r=24,vL=100,vR=55"),
        p.parse("SYS_EN,ms=80,l=20,r=26,vL=180,vR=65"),
        p.parse("SYS_E,ms=1000,encL=20,encR=26,ppsL=3,ppsR=2,appL=0,appR=0"),
        p.parse("SYS_M,ms=1000,locked=1,i2cerr=0,to=0"),
        p.parse("SYS_T,ms=1000,late=0,gap=0"),
        p.parse("SYS_U,ms=1000,tx_drop=0"),
    ]

class EncoderHostTests(unittest.TestCase):
    def test_signed_16bit_wrap(self):
        self.assertEqual(p.modulo_delta(5,65530),11)
        self.assertEqual(p.modulo_delta(65530,5),-11)

    def test_high_rate_and_health_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            stats=p.summarize_encoder(fixture_records(),Path(directory))
            self.assertTrue(stats["all_reported_locked"])
            self.assertEqual(stats["tim11_gap_delta"],0)
            self.assertEqual(stats["high_rate_encoder"]["status"],"OBSERVED")
            self.assertEqual(stats["high_rate_encoder"]["samples"],4)
            self.assertEqual(stats["high_rate_encoder"]["max_interval_ms"],20)
            self.assertTrue((Path(directory)/"encoder_50hz.csv").exists())

    def test_pwm_does_not_pass_locked_capture(self):
        rows=fixture_records()
        for record in rows:
            if record["type"]=="SYS_E":
                record["appL"]=50
                break
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(RuntimeError):
                p.summarize_encoder(rows,Path(directory))

    def test_malformed_data_and_short_capture(self):
        self.assertIsNone(p.parse("SYS_E,encL"))
        self.assertIsNone(p.parse("SYS_EN,ms=20,ms=30"))
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(RuntimeError):
                p.summarize_encoder([p.parse("SYS_E,ms=1,encL=1,encR=2,ppsL=0,ppsR=0,appL=0,appR=0")],Path(directory))

    def test_motor_off_on_50hz_end_to_end(self):
        # Synthetic UART fixture: does not power motors or certify real signal.
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            run=folder/"new_motor_run"
            run.mkdir()
            for phase in ("off","on"):
                l_count=r_count=0
                frames=[
                    "0.0\tSYS_M,ms=0,locked=1,i2cerr=0,to=0",
                    "0.0\tSYS_T,ms=0,late=0,gap=0",
                    "0.0\tSYS_U,ms=0,tx_drop=0,dma_err=0",
                    "0.0\tSYS_STEP,ms=0,id=25,group=2,state=1,duty=700",
                ]
                for i in range(160):
                    if phase=="on":
                        l_count+=5
                        r_count+=4
                    ms=20*(i+1)
                    frames.append("0.01\tSYS_EN,ms=%d,l=%d,r=%d,vL=%d,vR=%d" %
                                  (ms,l_count,r_count,
                                   250 if phase=="on" else 0,
                                   200 if phase=="on" else 0))
                frames += [
                    "4.0\tSYS_T,ms=4000,late=0,gap=0",
                    "4.0\tSYS_U,ms=4000,tx_drop=0,dma_err=0",
                    "4.0\tSYS_M,ms=4000,locked=1,i2cerr=0,to=0",
                ]
                (run/(phase+"_uart.log")).write_text("\n".join(frames)+"\n",
                                                       encoding="utf-8")
            result=p.analyze_sweep_telemetry(run,folder)
            self.assertEqual(result["ON"]["status"],"PASS")
            self.assertEqual(result["OFF"]["status"],"PASS")
            self.assertTrue((folder/"encoder_motor_on_50hz.csv").exists())

    def test_motor_sweep_missing_telemetry_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            run=folder/"failed_motor"
            run.mkdir()
            (run/"off_uart.log").write_text("0.1\tSYS_EN,ms=1,l=0,r=0,vL=0,vR=0\n",
                                            encoding="utf-8")
            with self.assertRaises(RuntimeError):
                p.analyze_sweep_telemetry(run,folder)

    def test_historical_sweep_report(self):
        report={"results":[{"mode":"ON","extended":{"stages":[{
            "wheel_group":2,"pwm_percent":20,"drive_ms":2000,
            "drive_left_counts":300,"drive_right_counts":320,
            "coast_left_counts":100,"coast_right_counts":110
        }]}}]}
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            src=folder/"sample.json"
            src.write_text(json.dumps(report),encoding="utf-8")
            result=p.sweep_summary(src,folder)
            self.assertEqual(result["motor_on_stages"],1)
            self.assertTrue((folder/"sweep_encoder_stages_historical.csv").exists())

if __name__=="__main__":
    unittest.main()
