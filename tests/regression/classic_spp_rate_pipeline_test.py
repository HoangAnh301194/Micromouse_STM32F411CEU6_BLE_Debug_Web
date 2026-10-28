#!/usr/bin/env python3
"""Offline regression for Classic SPP 1-kHz/IR/IMU throughput scorer."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from classic_spp_rate_pipeline import assess, refine_against_baseline
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

    def test_high_utilization_rejected_even_if_link_delivers(self):
        events = session_events(1, 32, 0) + session_events(2, 128, 80)
        frames = session_frames(2, 80, 128)
        rows = assess(events, frames, 0)
        refine_against_baseline(rows)
        self.assertEqual(rows[1]["verdict"], "LIMIT_OR_FAIL")
        self.assertGreater(rows[1]["uart_pct"], 80)

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

    def test_crc_failure_blocks_qualification(self):
        events = session_events(1, 32, 0) + session_events(2, 64, 40)
        rows = assess(events, session_frames(2, 40, 64), 1)
        self.assertEqual(rows[1]["verdict"], "LIMIT_OR_FAIL")


if __name__ == "__main__":
    unittest.main(verbosity=2)
