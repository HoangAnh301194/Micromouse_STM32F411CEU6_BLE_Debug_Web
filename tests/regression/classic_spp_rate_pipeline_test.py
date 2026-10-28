#!/usr/bin/env python3
"""Offline regression for Classic SPP 1-kHz/IR/IMU throughput scorer."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from classic_spp_rate_pipeline import assess, refine_against_baseline, measured_recommendations, EXPECTED_CASES
from ble_full_pipeline import Decoder, crc16


def session_events(s, size, hz, late=0, gap=0, gyro=3980, ir=4000,
                   irfresh=4000, accepted=None, rejected=0, skipped=0,
                   peak=600):
    gen = int(hz * 4)
    accepted = gen - rejected if accepted is None else accepted
    before = dict(s=s, ms=1000 + s * 10000, tick=1000, late=0, gap=0,
                  gyro=1000, valid=1000, stale=0, ir=1000, irfail=0,
                  irfresh=1000, drop=0, q=0, locked=1)
    after = dict(s=s, ms=before["ms"] + 4000, tick=5000, late=late,
                 gap=gap, gyro=1000+gyro, valid=5000, stale=0,
                 ir=1000+ir, irfail=0, irfresh=1000+irfresh,
                 drop=rejected, q=0, locked=1)
    return [
        {"type": "!R", "p": "B", **before},
        {"type": "!T", "p": "B", "s": s, **{"b"+str(i): 0 for i in range(8)}},
        {"type": "!B", "s": s, "size": size, "hz": hz, "ms": 4000},
        {"type": "!E", "s": s, "gen": gen, "ok": accepted, "reject": rejected,
         "skip": skipped, "size": size, "hz": hz, "ms": 4000,
         "peak": peak, "drop": rejected},
        {"type": "!R", "p": "E", **after},
        {"type": "!T", "p": "E", "s": s, **{"b"+str(i): (0 if i == 7 else 400)
                                           for i in range(8)}},
    ]


def session_frames(s, hz, size):
    return [{"session": s, "seq": i, "size": size, "kind": "D",
             "host_monotonic_ns": 1_000_000_000 + i * (1_000_000_000 // hz)}
            for i in range(hz * 4)] if hz else []


class SppRateTests(unittest.TestCase):
    def test_crc_decoder_split_binary_and_ascii(self):
        pkt = bytearray([0xb5, 0x62, 1, ord("D"), 20, 3])
        pkt += (13).to_bytes(4, "little") + (321).to_bytes(4, "little")
        pkt += bytes([0, 0, 0, 0])
        pkt += crc16(pkt).to_bytes(2, "little")
        dec = Decoder()
        dec.feed(b"!Q,state=START,cases=15\\n".replace(b"\\n",b"\n"))
        dec.feed(bytes(pkt[:1]))
        dec.feed(bytes(pkt[1:9]))
        dec.feed(bytes(pkt[9:]))
        self.assertEqual(dec.crc_failures, 0)
        self.assertEqual(dec.frames[0]["session"], 3)
        self.assertEqual(dec.frames[0]["seq"], 13)
        self.assertEqual(dec.events[0]["state"], "START")
        self.assertEqual(EXPECTED_CASES, 57)

    def test_good_case_and_baseline(self):
        events = session_events(1, 32, 0) + session_events(2, 64, 50)
        frames = session_frames(2, 50, 64)
        rows = assess(events, frames, 0)
        refine_against_baseline(rows)
        self.assertEqual(rows[0]["verdict"], "BASELINE")
        self.assertEqual(rows[1]["verdict"], "CANDIDATE")
        self.assertEqual(rows[1]["uart_pct"], 27.78)
        self.assertEqual(rows[1]["ctrl_Hz"], 1000)
        self.assertEqual(rows[1]["imu_fresh_Hz"], 995.0)

    def test_above_40pct_can_pass_if_tim11_imu_ir_and_delivery_hold(self):
        events = session_events(1, 32, 0) + session_events(2, 128, 80)
        rows = assess(events, session_frames(2, 80, 128), 0)
        refine_against_baseline(rows)
        self.assertEqual(rows[1]["verdict"], "CANDIDATE")
        self.assertGreater(rows[1]["uart_pct"], 80)

    def test_above_8n1_wire_ceiling_is_not_sustainable(self):
        events = session_events(1, 32, 0) + session_events(2, 128, 100)
        rows = assess(events, session_frames(2, 100, 128), 0)
        self.assertEqual(rows[1]["verdict"], "LIMIT_OR_FAIL")

    def test_selection_uses_first_failure_and_measured_headroom(self):
        rates = (10, 20, 30, 40, 50, 60, 70, 80, 100)
        rows = [{"size_B": 128, "target_Hz": hz,
                 "payload_Bps": 128 * hz,
                 "uart_pct": round(128 * hz * 100 / 11520, 2),
                 "verdict": "LIMIT_OR_FAIL" if hz >= 100 else "CANDIDATE"}
                for hz in rates]
        r = measured_recommendations(rows)["128"]
        self.assertEqual(r["clean_frontier_Hz"], 80)
        self.assertEqual(r["first_failure_Hz"], 100)
        self.assertEqual(r["recommended_tested_Hz"], 70)
        self.assertGreater(r["recommended_uart_pct"], 40)

    def test_one_bad_step_breaks_frontier_even_if_later_step_passes(self):
        rows = [{"size_B": 64, "target_Hz": hz,
                 "payload_Bps": 64 * hz, "uart_pct": 64 * hz / 115.2,
                 "verdict": verdict}
                for hz, verdict in ((20, "CANDIDATE"), (40, "LIMIT_OR_FAIL"),
                                    (60, "CANDIDATE"), (80, "CANDIDATE"))]
        r = measured_recommendations(rows)["64"]
        self.assertEqual(r["clean_frontier_Hz"], 20)
        self.assertIsNone(r["recommended_tested_Hz"])

    def test_missing_low_rate_baseline_does_not_invent_recommendation(self):
        rows = [{"size_B": 64, "target_Hz": 10,
                 "payload_Bps": 640, "uart_pct": 5.6,
                 "verdict": "INCOMPLETE"}]
        self.assertIsNone(measured_recommendations(rows)["64"]["recommended_tested_Hz"])

    def test_crc16_bad_frame_rejected(self):
        payload = bytearray([0xb5, 0x62, 1, ord("D"), 20, 1])
        payload += (1).to_bytes(4, "little") + (42).to_bytes(4, "little")
        payload += bytes([0, 0, 0, 0])
        payload += crc16(payload).to_bytes(2, "little")
        payload[10] ^= 0x01
        dec = Decoder()
        dec.feed(bytes(payload))
        self.assertEqual(len(dec.frames), 0)
        self.assertGreater(dec.crc_failures, 0)

    def test_control_gap_rejected(self):
        events = session_events(1, 32, 0) + session_events(2, 64, 40, gap=1)
        rows = assess(events, session_frames(2, 40, 64), 0)
        self.assertEqual(rows[1]["verdict"], "LIMIT_OR_FAIL")

    def test_missing_mcu_snapshot_is_incomplete(self):
        events = session_events(1, 32, 0) + session_events(2, 64, 40)
        events = [x for x in events if not (x.get("s") == 2 and
                                            x["type"] == "!R" and x["p"] == "E")]
        rows = assess(events, session_frames(2, 40, 64), 0)
        self.assertEqual(rows[1]["verdict"], "INCOMPLETE")

    def test_sensor_freshness_drop(self):
        events = session_events(1, 32, 0) + session_events(2, 64, 40, gyro=3500)
        rows = assess(events, session_frames(2, 40, 64), 0)
        self.assertEqual(rows[1]["verdict"], "LIMIT_OR_FAIL")

    def test_crc_failure_scoped_to_current_session(self):
        events = (session_events(1, 32, 0) + session_events(2, 64, 40)
                  + session_events(3, 64, 60))
        frames = session_frames(2, 40, 64) + session_frames(3, 60, 64)
        rows = assess(events, frames, 1, {2: 1})
        self.assertEqual(rows[1]["verdict"], "LIMIT_OR_FAIL")
        self.assertEqual(rows[2]["verdict"], "CANDIDATE")

    def test_crc_failure_blocks_qualification(self):
        events = session_events(1, 32, 0) + session_events(2, 64, 40)
        rows = assess(events, session_frames(2, 40, 64), 1)
        self.assertEqual(rows[1]["verdict"], "LIMIT_OR_FAIL")


if __name__ == "__main__":
    unittest.main(verbosity=2)
