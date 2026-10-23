#!/usr/bin/env python3
"""Offline host-protocol regressions; no board, BLE adapter, ST-Link or motors."""
import unittest

from ble_full_pipeline import Decoder, crc16, score_test, MATRIX, BAUD384, MOTOR


def frame(size=20, seq=12, session=3, micros=123456):
    f = bytearray(size)
    f[:6] = bytes((0xB5, 0x62, 1, ord("D"), size, session))
    f[6:10] = seq.to_bytes(4, "little")
    f[10:14] = micros.to_bytes(4, "little")
    for i in range(14, size - 2):
        f[i] = (i + seq) & 0xFF
    f[-2:] = crc16(f[:-2]).to_bytes(2, "little")
    return bytes(f)


class DecoderTest(unittest.TestCase):
    def test_all_lengths_split_and_combined(self):
        for size in (16, 20, 32, 64, 128):
            with self.subTest(size=size):
                d = Decoder()
                payload = b"!B,s=3,size=%d,hz=50,ms=5000\n" % size
                raw = payload + frame(size) + frame(size, 13) + b"!E,s=3,gen=2,ok=2\n"
                for i in range(0, len(raw), 7):
                    d.feed(raw[i:i + 7], 123)
                self.assertEqual(len(d.frames), 2)
                self.assertEqual([p["seq"] for p in d.frames], [12, 13])
                self.assertEqual([e["type"] for e in d.events], ["!B", "!E"])
                self.assertEqual(d.crc_failures, 0)
                self.assertEqual(d.discarded, 0)

    def test_resync_after_corrupt_frame(self):
        d = Decoder()
        corrupt = bytearray(frame())
        corrupt[15] ^= 1
        d.feed(bytes(corrupt) + frame(seq=14))
        self.assertEqual([x["seq"] for x in d.frames], [14])
        self.assertGreater(d.crc_failures, 0)

    def test_incomplete_frame_never_passes(self):
        d = Decoder()
        d.feed(frame(128)[:20])
        self.assertFalse(d.frames)
        d.feed(frame(128)[20:])
        self.assertEqual(d.frames[0]["size"], 128)

    def test_protocol_checks(self):
        self.assertEqual(len(MATRIX), 18)
        self.assertEqual(len(BAUD384), 6)
        self.assertEqual(len(MOTOR), 6)
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_score_missing_reported(self):
        base = dict(tick=1000, late=0, gap=0, stale=0, i2cerr=0,
                    i2cto=0, ir=100, irfail=0, txcy=100)
        after = dict(base, tick=6000, ir=300, txcy=200)
        begin = dict(type="!B", s=3)
        end = dict(type="!E", s=3, gen=50, ok=50, reject=0,
                   skip=0, drop=0, rxerr=0, peak=10, ecy=100,
                   scy=300)
        frames = [dict(session=3, seq=i, kind="D", size=20,
                       host_monotonic_ns=i * 100_000_000)
                  for i in range(49)]
        r = score_test(("X", 20, 10), frames, begin, end,
                       base, after, 0, 5)
        self.assertEqual(r["missing_vs_generated"], 1)
        self.assertFalse(r["checks"]["host_receives_all_accepted"])
        self.assertFalse(r["pass"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
