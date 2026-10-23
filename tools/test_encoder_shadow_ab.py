#!/usr/bin/env python3
"""Pure synthetic regression fixtures, never hardware qualification."""
import tempfile
from pathlib import Path
import unittest

from encoder_shadow_ab import (
    ALPHAS, LEGACY_ALPHAS, METRICS, analyze, parse_frame
)


class ShadowABTests(unittest.TestCase):
    def test_parser_and_legacy_compatibility(self):
        row = parse_frame("1.01\tSYS_AM,ms=5000,id=1,state=2,w=0,m=0,n=999,x0=1")
        self.assertEqual(row["id"], 1)
        self.assertEqual(row["n"], 999)
        self.assertIsNone(parse_frame("SYS_AM,bad"))
        self.assertIsNone(parse_frame("SYS_AB,ms=1,ms=2"))

    def test_missing_trace_not_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            for phase in ("off", "on"):
                (p / (phase + "_uart.log")).write_text(
                    "0.0\tSYS_STEP,id=0,state=1,group=0,duty=0\n"
                )
            report = analyze(p, p)
            self.assertEqual(report["status"], "INSUFFICIENT_SHADOW_DATA")
            self.assertEqual(report["provisional_top_two"], [])

    @staticmethod
    def generate(root, alphas, malformed=False):
        names = {
            0.05:"a05", 0.08:"a08", 0.10:"a10", 0.12:"a12",
            0.15:"a15", 0.20:"a20", 0.30:"a30",
            0.40:"a40", 0.60:"a60", 0.80:"a80"
        }
        for phase in ("off", "on"):
            lines = []
            for stage in range(33):
                lines.append(
                    "0.0\tSYS_STEP,ms=0,id=%d,state=1,group=%d,duty=%d"
                    % (stage, stage // 11, (stage % 11) * 100)
                )
                for tick in range(5):
                    for wheel in (0, 1):
                        data = ",".join(
                            names[alpha] + "=0" for alpha in alphas
                        ) + ",b5=0,b10=0,b20=0"
                        lines.append(
                            f"0.1\tSYS_AB,ms={tick*200},w={wheel},{data}"
                        )
                for state in (1, 2):
                    for wheel in (0, 1):
                        for metric in METRICS:
                            n = 300 if metric != 1 else 450
                            penalties = {
                                0.05: 90, 0.08: 50, 0.10: 32, 0.12: 15,
                                0.15: 35, 0.20: 55, 0.30: 90,
                                0.40: 120, 0.60: 180, 0.80: 230
                            }
                            if metric == 2:
                                # Less alpha -> smoother, but transition
                                # penalty for alpha 0.05 is much higher.
                                penalties = {
                                    0.05: 12, 0.08: 16, 0.10: 20,
                                    0.12: 26, 0.15: 35, 0.20: 45,
                                    0.30: 65, 0.40: 80, 0.60: 140, 0.80: 200
                                }
                            if metric == 1:
                                penalties = {
                                    0.05: 140, 0.08: 60, 0.10: 30,
                                    0.12: 15, 0.15: 38, 0.20: 60,
                                    0.30: 90, 0.40: 120, 0.60: 170, 0.80: 230
                                }
                            fields = ",".join(
                                "x%d=%d" % (i, penalties[alpha] * n)
                                for i, alpha in enumerate(alphas)
                            )
                            if malformed and phase == "on" and stage == 10 and (
                                state, wheel, metric
                            ) == (1, 0, 0):
                                fields = fields.replace("x9=", "bad9=")
                            lines.append(
                                "0.2\tSYS_AM,ms=0,id=%d,state=%d,w=%d,m=%d,n=%d,%s"
                                % (stage, state, wheel, metric, n, fields)
                            )
            (root / (phase + "_uart.log")).write_text(
                "\n".join(lines) + "\n", encoding="utf-8"
            )

    def test_ten_alpha_capture_and_ranking(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            self.generate(path, ALPHAS)
            report = analyze(path, path)
            self.assertEqual(report["status"], "COMPLETE_PROVISIONAL")
            self.assertEqual(report["shadow_bank_version"], "ten_alpha")
            self.assertEqual(report["expected_metric_frames"], 792)
            self.assertEqual(report["sample_frames"], 660)
            self.assertEqual(report["provisional_top_two"][0], 0.12)
            self.assertEqual(
                sum(report["metrics"][key]["intervals"]
                    for key in METRICS.values()), 112
            )
            self.assertTrue((path / "encoder_alpha_stage_metrics.csv").is_file())

    def test_legacy_seven_alpha_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            self.generate(path, LEGACY_ALPHAS)
            report = analyze(path, path)
            self.assertEqual(report["status"], "COMPLETE_PROVISIONAL")
            self.assertEqual(report["shadow_bank_version"], "legacy_seven_alpha")
            self.assertEqual(report["candidate_alphas"], list(LEGACY_ALPHAS))
            self.assertEqual(report["expected_metric_frames"], 792)

    def test_malformed_new_frame_is_not_treated_as_legacy(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            self.generate(path, ALPHAS, malformed=True)
            report = analyze(path, path)
            self.assertEqual(report["status"], "INSUFFICIENT_SHADOW_DATA")
            self.assertEqual(report["missing_metric_frames_count"], 1)
            self.assertTrue(report["issues"])


if __name__ == "__main__":
    unittest.main()
