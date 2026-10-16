#!/usr/bin/env python3
"""No-hardware release-candidate audit: frozen source + replay of archived MCU idle trace.

This does not flash firmware, access UART, or constitute electrical qualification.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

from motor_startup_pipeline import parse_frame
from motor_startup_readonly_check import audit_frames

ROOT = Path(__file__).resolve().parent.parent
EVIDENCE = Path("motor_logs/startup_readonly_20261017_010627_618666")
CODE_BASELINE = "b40228e85382e54f2487d8145fa3b0558802065b"
EVIDENCE_COMMIT = "a5b67de451702f2ce92613affdf28bd6ea348eaa"
EXPECTED_UART_SHA256 = "c3608449a62ec264827d7a5792dc67dba785ffb77655a57b3280ee3e395ebde1"
FROZEN_BLOBS = {
    "src/motor.c": "57fc04072cd9d206e22e54e987a9b63b450739f0",
    "include/motor.h": "9ed969ef6e46d682dbfd7a0607bff629be2330fc",
    "src/motor_test.c": "fdd1be9cb4ae353e76b289f765b206424a62cb74",
    "include/motor_test.h": "1cf62c21e2e611770bc18d719693b71dde165b95",
    "src/motor_startup.c": "2b6438ee93305a7d6d6e2e5dba1d83b65288a25c",
    "include/motor_startup.h": "12638ac6a5749609c7dc283c89af0a0ddefa8e79",
    "src/motor_startup_bench.c": "ec2a1c36c40c98d52d2cc8bcfae77fbf7620bd6b",
    "src/uart.c": "c82ef679f2f2a0cc0bcca414d8619fb11297481f",
    "include/uart.h": "a9c00efb3cf52a52eb5e445161609730a52a1bd3",
    "src/control.c": "bd07fb35c3018add548e6ded0c54212e0a15ea52",
    "include/control.h": "f4cd282ffc3a50a5a182032799e97aa23777199b",
    "src/motion.c": "d45285c154d7f830b36557ccd92a72b09b694bfc",
    "include/motion.h": "2fbf4dc80930afe6a60441b966c308d99b9f852d",
    "include/pinout.h": "9b517b99ef68a377a757990a62a27429eefd6f39",
    "src/main.c": "3a53da5d31cb4028b92e7039299849017a63c14d",
    "platformio.ini": "fe413177ad0a262da1091c4dc87caa4a27828422",
    "tools/motor_startup_readonly_check.py": "3e334ca24b0cbc6a928df42f1e9932cf0bd93b0d",
    "tools/motor_startup_pipeline.py": "48526d3ad8fa258826c2c9fe51634779564b1ea5",
}
EXPECTED_STATUS = "PASS_INSPECTION_ONLY_IDLE_V4"


class GateError(ValueError):
    pass


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def verify_sources(root: Path = ROOT, frozen: dict[str, str] = FROZEN_BLOBS) -> dict:
    """Compare Git blob IDs, not local commit SHA, to permit docs-only handoff commits."""
    normalized_eol = []
    for name, expected in frozen.items():
        file = root / name
        if not file.is_file():
            raise GateError(f"Frozen source missing: {name}")
        data = file.read_bytes()
        if git_blob_sha(data) == expected:
            continue
        # Windows Git checkout with autocrlf may have converted source LF to CRLF.
        if b"\r\n" in data and git_blob_sha(data.replace(b"\r\n", b"\n")) == expected:
            normalized_eol.append(name)
            continue
        raise GateError(f"FROZEN_SOURCE_DRIFT: {name}; compare against {CODE_BASELINE}")
    return {"source_status": "PASS_FROZEN_SOURCE_BLOBS",
            "source_files": len(frozen), "windows_eol_normalized": normalized_eol}


def decode_uart_trace(raw: bytes) -> list[tuple[str, dict]]:
    frames = []
    seen_partial = False
    prev_ts = None
    for index, line in enumerate(raw.splitlines()):
        try:
            timestamp, payload = line.split(b"\t", 1)
            stamp = float(timestamp.decode("ascii"))
            payload_text = payload.decode("ascii")
        except (ValueError, UnicodeDecodeError) as exc:
            raise GateError(f"Malformed UART capture line {index + 1}") from exc
        if prev_ts is not None and stamp < prev_ts:
            raise GateError("Host UART log timestamps moved backwards")
        prev_ts = stamp
        if not payload_text.startswith("STARTUP_"):
            # Capture.read() intentionally ignores only one partial *first* frame
            # after opening an already streaming UART.
            if index != 0 or seen_partial:
                raise GateError("Unexpected unframed UART data during capture")
            seen_partial = True
            continue
        try:
            name, fields = parse_frame(payload)
        except (ValueError, UnicodeDecodeError, IndexError) as exc:
            raise GateError(f"Malformed UART telemetry at line {index + 1}") from exc
        frames.append((name, fields))
    return frames


def audit_evidence(evidence: Path = ROOT / EVIDENCE,
                   checker: Path = ROOT / "tools/motor_startup_readonly_check.py") -> dict:
    try:
        report = json.loads((evidence / "report.json").read_text(encoding="utf-8"))
        raw = (evidence / "raw_uart.log").read_bytes()
        commands = (evidence / "commands.log").read_bytes()
    except (OSError, json.JSONDecodeError) as exc:
        raise GateError("Archived MCU evidence is missing or invalid") from exc

    digest = hashlib.sha256(raw).hexdigest()
    if digest != EXPECTED_UART_SHA256 or report.get("raw_uart_sha256") != digest:
        raise GateError("Archived UART trace SHA-256 does not match signed-off evidence")
    if not checker.is_file():
        raise GateError("Read-only checker used for MCU evidence is missing")
    # The live report was generated on Windows; Git can check out text with
    # CRLF while CI/Linux keeps LF. Verify both encodings of *identical*
    # content, never arbitrary changes. verify_sources() separately pins
    # the Git blob of this checker against the immutable RC baseline.
    checker_bytes = checker.read_bytes()
    canonical = checker_bytes.replace(b"\r\n", b"\n")
    equivalent_hashes = {
        hashlib.sha256(data).hexdigest()
        for data in (checker_bytes, canonical, canonical.replace(b"\n", b"\r\n"))
    }
    if report.get("source_sha256") not in equivalent_hashes:
        raise GateError("Read-only checker differs from the source used for MCU evidence")
    if report.get("status") != EXPECTED_STATUS or report.get("firmware_version") != 4:
        raise GateError("Archived report does not certify expected idle protocol")
    if report.get("motor_vm") != "USER_CONFIRMED_PHYSICALLY_DISCONNECTED":
        raise GateError("Archived VM user declaration is missing")
    if report.get("electrical_qualified") is not False or report.get("electrical_qualification") != "NOT_MEASURED":
        raise GateError("Unsafe electrical qualification claim in report")
    if report.get("motor_arm_attempted") is not False or report.get("flash_attempted") is not False:
        raise GateError("Archived host action was not read-only")
    if report.get("allowed_host_commands") != ["STARTUP PING"]:
        raise GateError("Archived host command allowlist differs from PING-only contract")
    lines = commands.decode("ascii").splitlines()
    if len(lines) != 1 or len(lines[0].split("\t", 1)) != 2 or lines[0].split("\t", 1)[1] != "STARTUP PING":
        raise GateError("Host command transcript is not exactly one PING")
    try:
        float(lines[0].split("\t", 1)[0])
    except ValueError as exc:
        raise GateError("Invalid command timestamp") from exc

    frames = decode_uart_trace(raw)
    result = audit_frames(frames, minimum_samples=5)
    for key in ("status", "firmware_version", "idle_samples", "elapsed_ms",
                "first_tick", "last_tick", "ping_status_frames"):
        if report.get(key) != result.get(key):
            raise GateError(f"Replay/report mismatch for {key}")
    if len(frames) != 9:
        raise GateError("Expected one PROFILE, one STATUS, one HEALTH, six SAMPLE frames")
    profiles = [row for name, row in frames if name == "STARTUP_PROFILE"]
    health = [row for name, row in frames if name == "STARTUP_HEALTH"]
    if len(profiles) != 1 or len(health) != 1 or health[0].get("ready") != 1:
        raise GateError("Unexpected profile or sensor-health count")
    if report.get("idle_samples") != 6 or report.get("elapsed_ms") != 200:
        raise GateError("Archived 6-sample 200ms attestation mismatch")
    return {"evidence_status": "PASS_ARCHIVED_MCU_IDLE_REPLAY",
            "raw_uart_sha256": digest,
            "frames": len(frames),
            "samples": result["idle_samples"],
            "tim11_ticks": result["last_tick"] - result["first_tick"],
            "motor_vm": "USER_DECLARED_DISCONNECTED_NOT_EXTERNALLY_MEASURED"}


def audit(root: Path = ROOT) -> dict:
    code = verify_sources(root)
    hardware = audit_evidence(root / EVIDENCE, root / "tools/motor_startup_readonly_check.py")
    return {"status": "PASS_SOFTWARE_RC_EVIDENCE_AUDIT",
            "code_baseline": CODE_BASELINE,
            "evidence_commit": EVIDENCE_COMMIT,
            **code, **hardware,
            "powered_qualification": "NOT_QUALIFIED",
            "independent_electrical_measurements": "NOT_MEASURED"}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true", help="Offline hostile-evidence regression")
    group.add_argument("--audit", action="store_true", help="Verify frozen code and replay committed MCU log")
    args = parser.parse_args(argv)
    if args.self_test:
        import unittest
        suite = unittest.defaultTestLoader.discover(
            str(ROOT / "tests/motor_driver"), pattern="test_motor_release_gate.py")
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    try:
        result = audit()
        print(json.dumps(result, indent=2))
        return 0
    except (GateError, ValueError) as exc:
        print(f"FAIL_SOFTWARE_RC_EVIDENCE_AUDIT: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
