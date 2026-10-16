from __future__ import annotations

import contextlib
import copy
import io
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import motor_startup_readonly_check as checker
import motor_startup_pipeline as startup


def fixture():
    profile = {**startup.HIGH_PROFILE, "build": "test_firmware", "reset_flags": 0}
    samples = [dict(id=0, state=0, fault=0, ms=1000 + i * 40, t=0,
                    rest=0, q=400, wheel=0, duty=0,
                    l=0, r=0, al=0, ar=0, cl=3200, cr=3200, odr=0,
                    ticks=900 + i * 40, late=0, gaps=0, tx=0, rx=0)
               for i in range(6)]
    return [("STARTUP_PROFILE", profile)] + [("STARTUP_SAMPLE", row) for row in samples]


class ReadonlyAuditTests(unittest.TestCase):
    def test_idle_v4_pass(self):
        result = checker.audit_frames(fixture())
        self.assertEqual(result["status"], "PASS_INSPECTION_ONLY_IDLE_V4")
        self.assertEqual(result["elapsed_ms"], 200)
        self.assertEqual(result["idle_samples"], 6)
        self.assertFalse(result["electrical_qualified"])

    def test_missing_old_or_mixed_profile_fail(self):
        cases = []
        cases.append(fixture()[1:])
        old = fixture()
        old[0][1]["version"] = 3
        cases.append(old)
        mixed = fixture()
        mixed.insert(1, ("STARTUP_PROFILE", {**startup.HIGH_PROFILE, "build": "other", "reset_flags": 0}))
        cases.append(mixed)
        reset_after = fixture()
        reset_after.append(("STARTUP_PROFILE", {**startup.HIGH_PROFILE, "build": "test_firmware", "reset_flags": 1}))
        cases.append(reset_after)
        for case in cases:
            with self.subTest(case=case[:1]), self.assertRaises(ValueError):
                checker.audit_frames(case)

    def test_late_solicted_profile_is_allowed(self):
        case = fixture()
        case.insert(3, ("STARTUP_PROFILE", {**startup.HIGH_PROFILE, "build": "test_firmware", "reset_flags": 0}))
        self.assertEqual(checker.audit_frames(case)["status"], "PASS_INSPECTION_ONLY_IDLE_V4")

    def test_never_accept_active_output_or_fault(self):
        for field, value in (("state", 1), ("id", 1), ("duty", 20),
                             ("al", 20), ("odr", 0x2000), ("cl", 64),
                             ("fault", 2), ("late", 1), ("gaps", 1)):
            case = fixture()
            case[2][1][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                checker.audit_frames(case)

    def test_missing_bad_timing_or_encoder_motion_fail(self):
        cases = []
        cases.append(fixture()[:4])
        moved = fixture()
        moved[-1][1]["l"] = 1
        cases.append(moved)
        stalled = fixture()
        stalled[-1][1]["ticks"] = stalled[-2][1]["ticks"]
        cases.append(stalled)
        skip = fixture()
        skip[-1][1]["ms"] += 300
        cases.append(skip)
        short = fixture()
        short[-1][1]["ms"] = short[1][1]["ms"] + 100
        cases.append(short)
        for case in cases:
            with self.subTest(length=len(case)), self.assertRaises(ValueError):
                checker.audit_frames(case)

    def test_unsolicited_arm_or_fault_frame_fails(self):
        for name in ("STARTUP_BEGIN", "STARTUP_STATUS", "STARTUP_FAULT", "STARTUP_EXPERIMENT_ACK"):
            case = fixture()
            case.insert(2, (name, {}))
            with self.subTest(frame=name), self.assertRaises(ValueError):
                checker.audit_frames(case)

    def test_hardware_run_requires_physical_vm_confirmation(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
            checker.main(["--run", "--port", "COM6"])
        self.assertEqual(exc.exception.code, 2)

    def test_readonly_allowlist_is_only_ping(self):
        self.assertEqual(checker.ALLOWED_COMMAND, "STARTUP PING")
        source = (ROOT / "tools/motor_startup_readonly_check.py").read_text(encoding="utf-8")
        self.assertIn("capture.send(ALLOWED_COMMAND)", source)
        self.assertNotIn('capture.send("STARTUP ARM")', source)
        self.assertNotIn('capture.send("STARTUP PULSE', source)
        self.assertNotIn("subprocess.run", source)


if __name__ == "__main__":
    unittest.main()
