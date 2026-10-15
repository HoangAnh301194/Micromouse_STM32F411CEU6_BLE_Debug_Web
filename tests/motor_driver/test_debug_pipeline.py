from __future__ import annotations

import contextlib
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import motor_auto_pipeline as motor_auto
import motor_driver_debug_pipeline as pipeline
import motor_driver_metrics as metrics
import motor_functional_trace_pipeline as functional_trace


def motor_fixture(offset: int = 0) -> str:
    motor_auto.configure_profile(legacy=True)
    matrix = motor_auto.MATRIX
    lines = [
        "MOTOR_AUTO_PROFILE,version=4,nominal_hz=96000000,pwm_hz=30000,rounds=3,steps=21,hard_cap_permille=200,both_cap_permille=160,families=4,rx_mode=irq256,high_pulse_ms=0,high_post_ms=0,pulse_ms=200,post_ms=400,boot_neutral=1,sensor=0",
        "MOTOR_AUTO_READY,version=4,sensor=0,state=0",
        "MOTOR_AUTO_BEGIN,rounds=3,steps=21,settle_ms=1000,boot_neutral=1",
    ]
    for round_index in range(3):
        for stage, (left, right) in enumerate(matrix):
            family = motor_auto.TESTS[stage]
            rise = motor_auto.RISE_RATES[stage]
            trial_index = round_index * len(matrix) + stage
            origin = offset + trial_index * 2000
            lines.append(f"MOTOR_AUTO_START,round={round_index},stage={stage},l={left},r={right},test={family},rise={rise},pulse_ms=200")
            for phase, duration in (("drive", 200), ("post", 400)):
                for elapsed in range(20, duration + 1, 20):
                    active = phase == "drive" and elapsed < 200
                    counts = elapsed if phase == "drive" else 200
                    applied_left, applied_right = (left, right) if active else (0, 0)
                    timestamp = origin + elapsed + (200 if phase == "post" else 0)
                    lines.append(f"MOTOR_AUTO_SAMPLE,round={round_index},stage={stage},phase={phase},test={family},rise={rise},ms={timestamp},elapsed={elapsed},applied_l={applied_left},applied_r={applied_right},enc_l={counts if left else 0},enc_r={counts if right else 0},ccr_l={applied_left * 3200 // 1000},ccr_r={applied_right * 3200 // 1000},tim11_late=0,tim11_max_cycles=1000,tim11_gaps=0,tx_drop=0")
                marker = "END" if phase == "drive" else "POST_END"
                reason = ",reason=0" if phase == "drive" else ""
                lines.append(f"MOTOR_AUTO_{marker},round={round_index},stage={stage},elapsed={duration}{reason},enc_l={200 if left else 0},enc_r={200 if right else 0},ccr_l=0,ccr_r=0")
            lines.append(f"MOTOR_AUTO_LINK,round={round_index},stage={stage},keep_ok={trial_index + 1},host_age_ms=100,rx_bytes={trial_index + 50},rx_ore=0,rx_fe=0,rx_noise=0,rx_ring_drop=0,bad_commands=0")
        lines.append(f"MOTOR_AUTO_ROUND_END,round={round_index + 1},completed={(round_index + 1) * len(matrix)}")
    lines.append("MOTOR_AUTO_DONE,rounds=3,pulses=63,ccr_l=0,ccr_r=0,tim11_gaps=0,tx_drop=0,keep_ok=63,rx_ore=0,rx_ring_drop=0")
    return "\n".join(lines) + "\n"


def speed_rows(count_step: int = 20, timestamp_origin: int = 1000) -> list[dict]:
    return [{"phase": "drive", "elapsed": 20 * index,
             "ms": (timestamp_origin + 20 * index) & 0xFFFFFFFF,
             "applied_l": 160, "enc_l": count_step * index}
            for index in range(1, 32)]


class DriverMetricsTests(unittest.TestCase):
    def test_distribution_and_zero_mean(self):
        self.assertEqual(metrics.distribution([100, 100, 100])["cv_pct"], 0)
        self.assertIsNone(metrics.distribution([0, 0])["cv_pct"])
        self.assertIsNone(metrics.distribution([10])["sample_std"])
        self.assertEqual(metrics.distribution([])["n"], 0)

    def test_non_finite_rejected(self):
        with self.assertRaises(ValueError):
            metrics.distribution([float("nan")])

    def test_percentile(self):
        self.assertEqual(metrics.percentile(list(range(101)), 95), 95)
        self.assertIsNone(metrics.percentile([], 99))

    def test_trace_quantiles(self):
        result = metrics.trace_metrics(functional_trace.synthetic_records())
        self.assertEqual(result["period_count"], 511)
        self.assertEqual(result["absolute_jitter_p99_us"], 0)

    def test_missing_trace_rows_rejected(self):
        content = functional_trace.synthetic_records()
        lines = [line for line in content.splitlines() if not line.startswith("MFD,i=20,")]
        with self.assertRaises(ValueError):
            metrics.trace_metrics("\n".join(lines))

    def test_constant_speed_plateau(self):
        result, rows = metrics.speed_metrics(speed_rows(), "l", 160, 700, metrics.MetricOptions())
        self.assertEqual(result["status"], "MEASURED_PLATEAU_CANDIDATE")
        self.assertEqual(result["cv_speed_pct"], 0)
        self.assertEqual(rows[0]["speed_pps"], 1000)

    def test_short_pulse_never_becomes_steady_speed(self):
        result, _ = metrics.speed_metrics(speed_rows(), "l", 160, 200, metrics.MetricOptions())
        self.assertEqual(result["status"], "INSUFFICIENT_PLATEAU")
        self.assertIsNone(result["cv_speed_pct"])

    def test_low_resolution_not_reported_as_steady_cv(self):
        result, _ = metrics.speed_metrics(speed_rows(1), "l", 160, 700, metrics.MetricOptions())
        self.assertEqual(result["status"], "LOW_COUNT_RESOLUTION")
        self.assertIsNone(result["cv_speed_pct"])

    def test_acceleration_is_not_stationary(self):
        rows = speed_rows()
        for index, row in enumerate(rows):
            row["enc_l"] = 10 * index * index
        result, _ = metrics.speed_metrics(rows, "l", 160, 700, metrics.MetricOptions())
        self.assertEqual(result["status"], "NONSTATIONARY_WINDOW")
        self.assertIsNone(result["cv_speed_pct"])

    def test_timestamp_wrap(self):
        result, _ = metrics.speed_metrics(speed_rows(timestamp_origin=0xFFFFFF00), "l", 160, 700, metrics.MetricOptions())
        self.assertEqual(result["status"], "MEASURED_PLATEAU_CANDIDATE")

    def test_disjoint_windows_cannot_be_combined(self):
        rows = speed_rows()[:25]
        rows[12]["applied_l"] = 100
        result, _ = metrics.speed_metrics(rows, "l", 160, 700, metrics.MetricOptions())
        self.assertEqual(result["status"], "INSUFFICIENT_PLATEAU")

    def test_timestamp_duplicate_rejected(self):
        row = {"round": "0", "stage": "0", "phase": "drive", "elapsed": "20", "ms": "20"}
        with self.assertRaises(ValueError):
            metrics.validate_samples([row, row], metrics.MetricOptions())

    def test_elapsed_disagreement_rejected(self):
        rows = [{"round": "0", "stage": "0", "phase": "drive", "elapsed": str(elapsed), "ms": str(timestamp)}
                for elapsed, timestamp in ((20, 20), (40, 25))]
        with self.assertRaises(ValueError):
            metrics.validate_samples(rows, metrics.MetricOptions())

    def test_complete_legacy_log_and_repeats(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for index in range(4):
                path = Path(directory) / f"capture_{index}.log"
                path.write_text(motor_fixture(index * 200000), encoding="utf-8")
                paths.append(path)
            result = metrics.analyze_logs(paths, metrics.MetricOptions())
            self.assertFalse(result["errors"], result["errors"])
            self.assertEqual(len(result["sources"]), 4)
            self.assertEqual(len(result["repeat_statistics"]), 22)
            self.assertTrue(all(group["count_statistics"]["n"] == 12 for group in result["repeat_statistics"]))
            self.assertTrue(all(group["count_statistics"]["cv_pct"] == 0 for group in result["repeat_statistics"]))
            self.assertTrue(result["rise_slew_comparisons"])
            self.assertTrue(result["dual_wheel_comparisons"])

    def test_duplicate_capture_is_not_an_independent_trial(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory) / "first.log", Path(directory) / "second.log"
            content = motor_fixture()
            first.write_text(content, encoding="utf-8")
            second.write_text("\n".join("1.000\t" + line for line in content.splitlines()), encoding="utf-8")
            result = metrics.analyze_logs([first, second], metrics.MetricOptions())
            self.assertEqual(len(result["sources"]), 1)
            self.assertEqual(len(result["errors"]), 1)

    def test_corrupt_motor_logs_do_not_produce_measurements(self):
        fixture = motor_fixture()
        cases = ["\n".join(line for line in fixture.splitlines() if not line.startswith("MOTOR_AUTO_DONE")),
                 fixture.replace("phase=drive", "phase=drive,phase=drive", 1),
                 fixture.replace("version=4", "version=4,p0_locked=1")]
        with tempfile.TemporaryDirectory() as directory:
            for index, content in enumerate(cases):
                path = Path(directory) / f"bad_{index}.log"
                path.write_text(content, encoding="utf-8")
                result = metrics.analyze_logs([path], metrics.MetricOptions())
                self.assertTrue(result["errors"])
                self.assertFalse(result["sources"])


class PipelineSafetyTests(unittest.TestCase):
    def test_trace_requires_physical_confirmation_before_execution(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as failure:
            pipeline.parse_args(["--run", "--port", "COM6", "--capture-trace"])
        self.assertEqual(failure.exception.code, 2)

    def test_self_test_never_accepts_hardware_arguments(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            pipeline.parse_args(["--self-test", "--port", "COM6"])

    def test_invalid_drift_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            pipeline.parse_args(["--run", "--plateau-max-drift-pct", "nan"])

    def test_existing_evidence_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / "evidence.txt").write_text("preserve", encoding="utf-8")
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                pipeline.parse_args(["--run", "--output", directory])

    def test_missing_evidence_is_review_not_pass(self):
        gates = [{"status": "PASS"}, {"status": "NOT_MEASURED"}]
        self.assertEqual(pipeline.finalize(gates, False), ("REVIEW_PENDING_EVIDENCE", 0))
        self.assertEqual(pipeline.finalize(gates, True), ("REVIEW_PENDING_EVIDENCE", 2))
        self.assertEqual(pipeline.finalize([{"status": "FAIL"}], False), ("FAIL", 1))

    def test_saved_logs_cannot_certify_current_firmware(self):
        self.assertEqual(pipeline.finalize([{"status": "MEASURED_SAVED_LOG"}], True), ("REVIEW_PENDING_EVIDENCE", 2))

    def test_failed_command_is_recorded(self):
        process = subprocess.CompletedProcess(["fake"], 1, "failure", "")
        with tempfile.TemporaryDirectory() as directory, patch.object(pipeline.subprocess, "run", return_value=process):
            gate = pipeline.command_gate(Path(directory), "failed_test", ["fake"])
            self.assertEqual(gate["status"], "FAIL")
            self.assertTrue((Path(directory) / "failed_test.log").exists())

    def test_command_timeout_is_recorded(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(pipeline.subprocess, "run", side_effect=subprocess.TimeoutExpired("fake", 1)):
            gate = pipeline.command_gate(Path(directory), "timeout_test", ["fake"])
            self.assertEqual(gate["status"], "FAIL")

    def test_failed_source_gates_prevent_all_serial_access(self):
        offline = {"source_model_status": "FAIL", "offline": [{"status": "FAIL"}],
                   "pin_capture": {"status": "NOT_MEASURED"}}
        with tempfile.TemporaryDirectory() as directory:
            args = pipeline.parse_args(["--run", "--port", "COM6", "--output", str(Path(directory) / "report")])
            with patch.object(pipeline, "command_gate", return_value={"id": "forced_failure", "status": "FAIL"}), \
                 patch.object(pipeline.p0_batch, "run_batch", return_value=offline), \
                 patch.object(pipeline.driver_probe, "capture") as capture, \
                 patch.object(pipeline.functional_trace, "capture_device") as trace, \
                 contextlib.redirect_stdout(io.StringIO()):
                report, exit_code = pipeline.run(args)
            capture.assert_not_called()
            trace.assert_not_called()
            self.assertEqual(exit_code, 1)
            self.assertEqual(report["overall"], "FAIL")
            self.assertFalse(report["safety"]["motor_started"])

    def test_no_motor_data_is_not_zero_motion_measurement(self):
        result = metrics.analyze_logs([], metrics.MetricOptions())
        self.assertEqual(result["trials"], [])
        self.assertEqual(result["repeat_statistics"], [])


if __name__ == "__main__":
    unittest.main()
