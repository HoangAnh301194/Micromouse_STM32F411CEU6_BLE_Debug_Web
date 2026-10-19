#!/usr/bin/env python3
"""Synthetic host-level tests. Never treated as STM32 hardware acceptance."""
import tempfile
from pathlib import Path
import unittest

from encoder_shadow_ab import (ALPHAS, METRICS, analyze, parse_frame, summarize)


class ShadowABTests(unittest.TestCase):
    def test_parser(self):
        row = parse_frame("1.01\tSYS_AM,ms=5000,id=1,state=2,w=0,m=0,n=999,x0=1")
        self.assertEqual(row["id"], 1)
        self.assertEqual(row["n"], 999)
        self.assertIsNone(parse_frame("SYS_AM,bad"))
        self.assertIsNone(parse_frame("SYS_AB,ms=1,ms=2"))

    def test_missing_trace_not_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / "off_uart.log").write_text("0.0\tSYS_STEP,id=0,state=1,group=0,duty=0\n")
            (p / "on_uart.log").write_text("0.0\tSYS_STEP,id=0,state=1,group=0,duty=0\n")
            report = analyze(p, p)
            self.assertEqual(report["status"], "INSUFFICIENT_SHADOW_DATA")
            self.assertEqual(report["provisional_top_two"], [])
            self.assertTrue((p / "encoder_alpha_ab.json").exists())

    def test_full_synthetic_metric_capture_and_ranking(self):
        # 33 steps x 2 phases x DRIVE/COAST x 2 wheels x 3 metrics.
        # 10-Hz visualization is deliberately not used for latency ranking.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for phase in ("off", "on"):
                lines = []
                for stage in range(33):
                    lines.append("0.0\tSYS_STEP,ms=0,id=%d,state=1,group=%d,duty=%d" %
                                 (stage, stage // 11, (stage % 11) * 100))
                    for k in range(10):
                        for wheel in (0, 1):
                            lines.append("0.1\tSYS_AB,ms=%d,w=%d," % (k*100,wheel) +
                                ",".join(key+"=0" for key in
                                         ("a05","a10","a20","a30","a40","a60","a80",
                                          "b5","b10","b20")))
                    for state in (1, 2):
                        for wheel in (0, 1):
                            for metric in METRICS:
                                n = 300 if metric != 1 else 450
                                # Alpha 0.2 wins all three synthetic proxies.
                                penalties = (70, 40, 20, 50, 80, 110, 150)
                                if metric == 1:
                                    penalties = (120, 65, 25, 55, 75, 105, 140)
                                if metric == 2:
                                    penalties = (10, 18, 23, 32, 40, 65, 90)
                                lines.append("0.2\tSYS_AM,ms=0,id=%d,state=%d,w=%d,m=%d,n=%d," %
                                             (stage,state,wheel,metric,n) +
                                             ",".join("x%d=%d" % (i, penalties[i]*n)
                                                      for i in range(len(ALPHAS))))
                (root / (phase + "_uart.log")).write_text("\n".join(lines) + "\n")
            report = analyze(root, root)
            self.assertEqual(report["status"], "COMPLETE_PROVISIONAL")
            self.assertEqual(report["expected_metric_frames"], 792)
            self.assertEqual(report["sample_frames"], 1320)
            self.assertEqual(report["provisional_top_two"][0], 0.2)
            self.assertTrue((root / "encoder_alpha_stage_metrics.csv").is_file())
            self.assertTrue((root / "encoder_alpha_samples_10hz.csv").is_file())
            self.assertEqual(sum(report["metrics"][k]["intervals"] for k in METRICS.values()),
                             3 * 2 * 7 * 4 * 2)  # 20-80% x L/R/Both active wheels


if __name__ == "__main__":
    unittest.main()
