#!/usr/bin/env python3
"""Offline regression tests for SPP Motion v2 protocol parser, CSV logger, and metrics."""
from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from classic_spp_protocol import (
    parse_ext_telemetry_line,
    parse_motion_telemetry_line,
    parse_spp_control_line,
)
from motion_experiment import MotionCsvLogger, analyze_motion_csv


class SppMotionProtocolTest(unittest.TestCase):
    def test_telemetry_v1_and_v2_parsers(self) -> None:
        v1 = "t=1234 state=1 encL=100 encR=102 vL=500 vR=505 vRefL=500 vRefR=500 pL=120 pR=122 yaw=1.25 gz=-0.50 d=165.0 e=0.15"
        p1 = parse_motion_telemetry_line(v1)
        self.assertIsNotNone(p1)
        self.assertEqual(p1["t_ms"], 1234)
        self.assertEqual(p1["pwm_cmd_l_pm"], 120)
        self.assertEqual(p1["pwm_app_l_pm"], 120)

        v2 = "t=5000 state=4 ph=2 encL=200 encR=0 vL=995 vR=0 vRefL=1000 vRefR=0 pL=210 pR=0 aL=195 aR=0 yaw=0.10 gz=0.05 d=22.4 e=0.00 sat=1"
        p2 = parse_motion_telemetry_line(v2)
        self.assertIsNotNone(p2)
        self.assertEqual(p2["phase"], 2)
        self.assertEqual(p2["pwm_cmd_l_pm"], 210)
        self.assertEqual(p2["pwm_app_l_pm"], 195)
        self.assertEqual(p2["sat_flags"], 1)

        ext = parse_ext_telemetry_line("EXT,t=5000,vR=134.5,wR=0.0,vM=133.9,wM=-0.4,iL=42.1,iR=0.0,res=0")
        self.assertIsNotNone(ext)
        self.assertAlmostEqual(ext["integral_l"], 42.1, places=1)

    def test_control_parsers(self) -> None:
        ack = parse_spp_control_line("ACK,7,PING,READY=1,STATE=0,CAL=3,LOCK=0,RES=1")
        self.assertEqual(ack["kind"], "ACK")
        self.assertEqual(ack["cmd_id"], 7)
        self.assertEqual(ack["fields"]["READY"], "1")

        gains = parse_spp_control_line("GAINS,L,0.0900,0.2500,80.00,0.09800,0.00000,1250.0,1000.0")
        self.assertEqual(gains["kind"], "GAINS")
        self.assertAlmostEqual(gains["kp"], 0.09, places=4)

        done = parse_spp_control_line("EVENT,DONE,RES=SUCCESS,DIST=165.2,ERR=-0.35,T=8100")
        self.assertEqual(done["kind"], "EVENT")
        self.assertEqual(done["fields"]["RES"], "SUCCESS")

    def test_csv_logger_and_metrics_zero_ref_safety(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            logger = MotionCsvLogger(Path(tmp), "unit_step", "U1", {"port": "COM0"})
            for k in range(30):
                t_ms = 1000 + k * 20
                meas_l = 1000.0 * (1.0 - 0.5 ** (k + 1))
                telem = parse_motion_telemetry_line(
                    f"t={t_ms} state=4 ph=0 encL={k*20} encR=0 "
                    f"vL={meas_l:.0f} vR=0 vRefL=1000 vRefR=0 "
                    f"pL=210 pR=0 aL=210 aR=0 yaw=0.00 gz=0.00 d=10.0 e=0.00 sat=0"
                )
                logger.log_telemetry(telem)
            logger.close({"finish_result": "SUCCESS"})

            m = analyze_motion_csv(logger.csv_path, pwm_max_pm=300.0, finish_result="SUCCESS")
            self.assertEqual(m["samples_total"], 30)
            self.assertEqual(m["right"]["steady_state_error_pct"], "N/A (ref=0)")
            self.assertIsInstance(m["left"]["steady_state_error_pct"], float)
            self.assertLess(m["left"]["steady_state_error_pct"], 2.0)


if __name__ == "__main__":
    unittest.main()
