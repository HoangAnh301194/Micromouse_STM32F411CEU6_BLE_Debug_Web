#!/usr/bin/env python3
"""Host-only P0-P3 strict admission and alpha decision regression tests."""
import unittest

import encoder_p0_p3_pipeline as p
import system_uart_pipeline as uart


def good_phase(mode, stages=33, pwm=100):
    return {
        "mode": mode, "stages": stages, "expected_stages": stages,
        "max_pwm": pwm, "software_pass": True, "faults": [],
        "checks": {"no_sweep_fault": True, "uart_no_rx_error": True,
                   "shadow_spsc_no_loss": True},
        "timer": {"maxcy": 1105, "encmaxcy": 517},
        "uart": {"rx_err": 0, "tx_drop": 0, "dma_err": 0},
        "encoder_shadow_queue": {"in": 155000, "out": 155000, "lost": 0,
                                 "edge": 0, "peak": 1, "q": 0},
        "host_heartbeat": {"ping_sent": 900, "ping_ack_received": 900,
                           "max_inter_ping_gap_ms": 170},
    }


class PipelineGateTests(unittest.TestCase):
    def test_p0_good_and_failures(self):
        off = good_phase("OFF")
        self.assertEqual(p.check_p0(off)["status"], "PASS")
        for key in ("lost", "edge"):
            bad = good_phase("OFF")
            bad["encoder_shadow_queue"][key] = 1
            with self.assertRaises(RuntimeError):
                p.check_p0(bad)
        bad = good_phase("OFF")
        bad["timer"]["encmaxcy"] = 8000
        with self.assertRaises(RuntimeError):
            p.check_p0(bad)
        bad = good_phase("OFF")
        bad["uart"]["rx_err"] = 1
        with self.assertRaises(RuntimeError):
            p.check_p0(bad)

    def test_p1_limits(self):
        on = good_phase("ON", 12, 30)
        self.assertEqual(p.check_p1(on)["status"], "PASS")
        bad = good_phase("ON", 12, 100)
        with self.assertRaises(RuntimeError):
            p.check_p1(bad)
        bad = good_phase("ON", 12, 30)
        bad["host_heartbeat"]["max_inter_ping_gap_ms"] = 700
        with self.assertRaises(RuntimeError):
            p.check_p1(bad)

    def test_p2_no_shortcuts(self):
        self.assertEqual(p.check_p2(good_phase("ON"),good_phase("OFF"))["status"], "PASS")
        with self.assertRaises(RuntimeError):
            p.check_p2(good_phase("ON",12,30),good_phase("OFF"))
        bad = good_phase("ON")
        bad["checks"]["no_sweep_fault"] = False
        with self.assertRaises(RuntimeError):
            p.check_p2(bad,good_phase("OFF"))

    def test_alpha_does_not_change_production(self):
        alphas=[0.05,0.08,0.1,0.12,0.15,0.2,0.3,0.4,0.6,0.8]
        ranking=[{"alpha": x, "heuristic_score_vs_alpha_0_30": i}
                 for i,x in enumerate(alphas)]
        sample={
            "status": "COMPLETE_PROVISIONAL", "shadow_bank_version": "ten_alpha",
            "candidate_alphas": alphas, "missing_metric_frames_count": 0,
            "duplicated_frames_count": 0,
            "ranking": ranking,
            "ranking_by_wheel": {
                "L":[{"alpha":0.1},{"alpha":0.08},{"alpha":0.12}],
                "R":[{"alpha":0.12},{"alpha":0.1},{"alpha":0.15}],
            },
        }
        decision=p.decide_alpha(sample,True)
        self.assertEqual(decision["candidate_alpha"], 0.1)
        self.assertFalse(decision["automatic_pid_changes"])
        self.assertEqual(decision["production_alpha"], 0.3)
        with self.assertRaises(RuntimeError):
            p.decide_alpha(sample,False)
        sample["missing_metric_frames_count"] = 12
        with self.assertRaises(RuntimeError):
            p.decide_alpha(sample,True)

    def test_limited_30_stage_contract(self):
        # Deliberate P1-limited patterns, not a synthetic hardware PASS.
        logs=[]
        for stage in range(12):
            group, step = divmod(stage,4)
            logs.append({"type":"SYS_STEP","mode":1,"state":2,
                         "id":stage,"group":group,"step":step,
                         "duty":step*100,
                         "dl":70 if group!=1 else 0,
                         "dr":70 if group!=0 else 0})
        self.assertTrue(uart.inspect_limited_drive(logs)["qualified"])
        self.assertEqual(uart.inspect_limited_drive(logs)["drive_windows"],12)
        logs[-1]["dr"]=0
        self.assertFalse(uart.inspect_limited_drive(logs)["qualified"])


if __name__=="__main__":
    unittest.main()
