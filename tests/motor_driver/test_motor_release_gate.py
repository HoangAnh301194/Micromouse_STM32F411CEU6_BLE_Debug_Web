from __future__ import annotations

import contextlib
import io
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import motor_release_gate as gate


class SoftwareRcAuditTests(unittest.TestCase):
    def test_archived_mcu_uart_replay_pass(self):
        result = gate.audit_evidence()
        self.assertEqual(result["evidence_status"], "PASS_ARCHIVED_MCU_IDLE_REPLAY")
        self.assertEqual(result["frames"], 9)
        self.assertEqual(result["samples"], 6)
        self.assertEqual(result["tim11_ticks"], 200)

    def test_source_snapshot_matches_pinned_blob_ids(self):
        result = gate.verify_sources()
        self.assertEqual(result["source_files"], len(gate.FROZEN_BLOBS))

    def test_full_offline_gate_does_not_claim_electrical_qualification(self):
        result = gate.audit()
        self.assertEqual(result["status"], "PASS_SOFTWARE_RC_EVIDENCE_AUDIT")
        self.assertEqual(result["powered_qualification"], "NOT_QUALIFIED")
        self.assertEqual(result["independent_electrical_measurements"], "NOT_MEASURED")

    def test_any_frozen_source_edit_is_rejected(self):
        p = next(iter(gate.FROZEN_BLOBS))
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp) / p
            f.parent.mkdir(parents=True, exist_ok=True)
            f.write_bytes((ROOT / p).read_bytes() + b"\n// unexpected\n")
            with self.assertRaisesRegex(gate.GateError, "FROZEN_SOURCE_DRIFT"):
                gate.verify_sources(Path(tmp), {p: gate.FROZEN_BLOBS[p]})

    def test_raw_uart_tamper_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = self.copy_evidence(Path(tmp))
            with (dest / "raw_uart.log").open("ab") as f:
                f.write(b"INJECTED_FRAME\n")
            with self.assertRaisesRegex(gate.GateError, "SHA-256"):
                gate.audit_evidence(dest)

    def test_missing_ping_or_an_arm_command_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = self.copy_evidence(Path(tmp))
            (dest / "commands.log").write_text("001\tSTARTUP ARM\n", encoding="utf-8")
            with self.assertRaisesRegex(gate.GateError, "exactly one PING"):
                gate.audit_evidence(dest)

    def test_report_forged_electrical_pass_is_rejected(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            dest = self.copy_evidence(Path(tmp))
            report = json.loads((dest / "report.json").read_text(encoding="utf-8"))
            report["electrical_qualified"] = True
            (dest / "report.json").write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaisesRegex(gate.GateError, "Unsafe electrical qualification"):
                gate.audit_evidence(dest)

    def test_missing_or_modified_checker_cannot_reuse_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "readonly.py"
            bad.write_text("print('same')", encoding="utf-8")
            with self.assertRaisesRegex(gate.GateError, "differs"):
                gate.audit_evidence(checker=bad)

    def test_non_initial_malformed_uart_frame_rejected(self):
        bad = (b"10.0\tSTARTUP_PROFILE,version=4\n"
               b"10.1\tgarbage\n")
        with self.assertRaisesRegex(gate.GateError, "Unexpected unframed"):
            gate.decode_uart_trace(bad)

    def test_windows_line_endings_for_sources_are_normalized_only(self):
        data = b"example\nsource\n"
        expected = gate.git_blob_sha(data)
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp) / "firmware.c"
            f.write_bytes(data.replace(b"\n", b"\r\n"))
            self.assertEqual(gate.verify_sources(Path(tmp), {"firmware.c": expected})[
                "windows_eol_normalized"], ["firmware.c"])

    @staticmethod
    def copy_evidence(destination):
        dest = destination / "evidence"
        dest.mkdir()
        for name in ("report.json", "raw_uart.log", "commands.log"):
            shutil.copyfile(ROOT / gate.EVIDENCE / name, dest / name)
        return dest


if __name__ == "__main__":
    unittest.main()
