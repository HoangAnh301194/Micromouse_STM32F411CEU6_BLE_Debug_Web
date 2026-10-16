from __future__ import annotations

import contextlib
import io
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import motor_startup_pipeline as startup


def sample(elapsed=0, counts=0):
    return dict(id=1, state=2, fault=0, ms=1000 + elapsed, t=elapsed, rest=0, q=0,
                wheel=0, duty=100, l=counts, r=0, al=100, ar=0,
                cl=320, cr=3200, odr=0x2000, ticks=elapsed, late=0, gaps=0, tx=0, rx=0)


def pulse_rows(step=2):
    times = [0, *range(20, 250, 20), 250]
    return [sample(elapsed, index * step) for index, elapsed in enumerate(times)]


class FakePort:
    def __init__(self, frames=()):
        self.frames = list(frames)
        self.writes = []

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)

    def readline(self):
        return self.frames.pop(0) if self.frames else b""


class StartupMetricsTests(unittest.TestCase):
    def test_profile_requires_exact_limits(self):
        startup.validate_profile(startup.PROFILE)
        for field, value in (("cap", 1000), ("pulse_ms", 0), ("version", 14), ("quiet_ms", 0)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                startup.validate_profile({**startup.PROFILE, field: value})

    def test_sustained_rotation(self):
        result = startup.trial_metrics(pulse_rows(), "l")
        self.assertTrue(result["sustained_start"])
        self.assertEqual(result["first_five_counts_observed_ms"], 60)

    def test_no_counts_not_start(self):
        result = startup.trial_metrics(pulse_rows(0), "l")
        self.assertFalse(result["first_motion"])
        self.assertFalse(result["sustained_start"])

    def test_single_jump_not_sustained_rotation(self):
        rows = pulse_rows(0)
        for row in rows[1:]:
            row["l"] = 10
        self.assertFalse(startup.trial_metrics(rows, "l")["sustained_start"])

    def test_early_twitch_not_sustained_rotation(self):
        rows = pulse_rows()
        for row in rows:
            row["l"] = min(row["l"], 8)
        self.assertFalse(startup.trial_metrics(rows, "l")["sustained_start"])

    def test_missing_or_duplicate_sample_rejected(self):
        rows = pulse_rows()
        for corrupt in (rows[:3] + rows[6:], rows[:3] + [rows[2]] + rows[3:], rows[:-1]):
            with self.assertRaises(ValueError):
                startup.trial_metrics(corrupt, "l")

    def test_timestamp_wrap(self):
        rows = pulse_rows()
        for row in rows:
            row["ms"] = (0xFFFFFF80 + row["t"]) & 0xFFFFFFFF
        self.assertTrue(startup.trial_metrics(rows, "l")["sustained_start"])

    def test_reverse_and_passive_counts_rejected(self):
        for field, value in (("l", -5), ("r", 3)):
            rows = pulse_rows()
            rows[5][field] = value
            with self.assertRaises(ValueError):
                startup.trial_metrics(rows, "l")

    def test_register_error_and_over_duty_rejected(self):
        startup.validate_row(sample())
        for field, value in (("cl", 3200), ("ar", 100), ("duty", 1000), ("gaps", 1), ("tx", 1), ("rx", 1)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                startup.validate_row({**sample(), field: value})

    def test_neutral_and_host_stop(self):
        neutral = {**sample(), "state": 4, "fault": 5, "al": 0, "cl": 3200, "odr": 0}
        startup.validate_row(neutral, stopping=True)
        with self.assertRaises(ValueError):
            startup.validate_row(neutral)

    def test_duplicate_uart_fields_rejected(self):
        with self.assertRaises(ValueError):
            startup.parse_frame(b"STARTUP_SAMPLE,id=1,id=2\r\n")

    def test_direction_register_contract(self):
        for value in (0, 0x1000, 0x3000, 0xA000):
            with self.subTest(odr=value), self.assertRaises(ValueError):
                startup.validate_row({**sample(), "odr": value})
        startup.validate_row({**sample(), "wheel": 1, "l": 0, "r": 10,
                              "al": 0, "ar": 100, "cl": 3200, "cr": 320, "odr": 0x8000})

    def test_fault_state_consistency(self):
        with self.assertRaises(ValueError):
            startup.validate_row({**sample(), "state": 4})

class HighSpeedTests(unittest.TestCase):
    def test_high_speed_requires_matching_profile(self):
        startup.validate_profile(startup.HIGH_PROFILE, high_speed=True)
        with self.assertRaises(ValueError):
            startup.validate_profile(startup.HIGH_PROFILE)
        with self.assertRaises(ValueError):
            startup.validate_profile(startup.PROFILE, high_speed=True)

    def test_full_pwm_both_register_contract(self):
        row = {**sample(), "wheel": 2, "t": 400, "duty": 1000,
               "al": 1000, "ar": 1000, "cl": 3200, "cr": 3200, "odr": 0xA000}
        startup.validate_row(row, high_speed=True)
        with self.assertRaises(ValueError):
            startup.validate_row(row)
        with self.assertRaises(ValueError):
            startup.validate_row({**row, "cr": 3199}, high_speed=True)

    def test_first_tick_preload_guard_not_false_fault(self):
        row = {**sample(), "t": 1, "al": 20, "cl": 64, "odr": 0}
        startup.validate_row(row)
        with self.assertRaises(ValueError):
            startup.validate_row({**row, "t": 2})

    def test_speed_plateau_both_wheels(self):
        rows = [{**sample(elapsed), "l": elapsed * 4, "r": elapsed * 5}
                for elapsed in range(0, 601, 40)]
        result = startup.speed_metrics(rows, "b")
        self.assertEqual(result["speed_pps"]["l"]["mean"], 4000)
        self.assertEqual(result["speed_pps"]["r"]["mean"], 5000)
        self.assertTrue(result["speed_pps"]["l"]["steady"])
        with self.assertRaises(ValueError):
            startup.speed_metrics(rows[:-1], "b")

    def test_speed_missing_reverse_and_zero_counts_fail(self):
        rows = [{**sample(elapsed), "l": elapsed * 4} for elapsed in range(0, 601, 40)]
        for corrupt in (rows[:4] + rows[7:], [{**row, "l": 0} for row in rows],
                        rows[:8] + [{**rows[8], "l": -1}] + rows[9:]):
            with self.assertRaises(ValueError):
                startup.speed_metrics(corrupt, "l")

    def test_high_speed_allowlist_stays_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            port = FakePort()
            capture = startup.Capture(port, Path(directory), high_speed=True)
            try:
                capture.send("STARTUP PULSE 1 b 1000")
                for command in ("STARTUP PULSE 1 b 1005", "STARTUP PULSE 1 l -1000", "STARTUP PULSE 33 l 200"):
                    with self.assertRaises(ValueError):
                        capture.send(command)
            finally:
                capture.close()

    def test_sensor_error_fails_before_further_pulses(self):
        frame = b"STARTUP_HEALTH,ready=1,good=1000,fresh=1000,err=1,timeout=0,ir=10,age_max=1000,isr=1000,adc_div=4\r\n"
        with tempfile.TemporaryDirectory() as directory:
            capture = startup.Capture(FakePort([frame]), Path(directory), high_speed=True)
            try:
                with self.assertRaises(ValueError):
                    capture.read()
            finally:
                capture.close()

    def test_safety_fault_stop_can_confirm_neutral(self):
        row = {**sample(), "state": 4, "fault": 9, "al": 0, "cl": 3200, "odr": 0}
        startup.validate_row(row, stopping=True, high_speed=True)
        with self.assertRaises(ValueError):
            startup.validate_row(row, high_speed=True)

class StartupFirmwareTests(unittest.TestCase):
    def test_status_registers_and_state_are_sampled_before_irq_restore(self):
        source = (startup.ROOT / "src/motor_startup_bench.c").read_text(encoding="utf-8")
        function = source.split("static void Status(", 1)[1].split("static void Command(", 1)[0]
        restore = function.index("if (!mask) __enable_irq()")
        for operation in ("MotorStartup_GetSnapshot(&coherent)", "Control_GetTiming(&timing)",
                          "compare_left = TIM2->CCR2", "compare_right = TIM2->CCR1",
                          "direction = GPIOB->ODR", "sample_ticks = Control_GetTickCount()"):
            self.assertLess(function.index(operation), restore)

    def test_watchdog_start_precedes_register_update_wait(self):
        source = (startup.ROOT / "src/motor_startup_bench.c").read_text(encoding="utf-8")
        function = source.split("static uint8_t Watchdog_Init(void)", 1)[1].split("\n}", 1)[0]
        operations = ("IWDG->KR = 0xCCCCU", "IWDG->KR = 0x5555U", "IWDG->PR = 5U",
                      "IWDG->RLR = 124U", "while (IWDG->SR != 0U)", "IWDG->KR = 0xAAAAU")
        positions = [function.index(operation) for operation in operations]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("micros() - start", function)
        self.assertIn(">= 100000UL", function)

    def test_watchdog_probe_clears_sticky_reset_flags(self):
        source = (startup.ROOT / "src/motor_startup_bench.c").read_text(encoding="utf-8")
        function = source.split("static void Command(const char *line)", 1)[1]
        self.assertLess(function.index("RCC->CSR |= RCC_CSR_RMVF"),
                        function.index("watchdog_probe = 1U"))


class StartupSafetyTests(unittest.TestCase):
    def test_run_requires_physical_confirmation(self):
        for args in (["--run"], ["--run", "--vm-volts", "8.4"],
                     ["--run", "--vm-volts", "nan", "--confirm-wheels-elevated"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                startup.parse_args(args)

    def test_self_test_rejects_hardware_flags(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            startup.parse_args(["--self-test", "--upload"])

    def test_allowlist_refuses_full_pwm_and_reversal(self):
        with tempfile.TemporaryDirectory() as directory:
            port = FakePort()
            capture = startup.Capture(port, Path(directory))
            try:
                for command in ("MOTOR START", "STARTUP PULSE 1 l 1000", "STARTUP PULSE 1 l -100", "STARTUP PULSE 1 both 100"):
                    with self.assertRaises(ValueError):
                        capture.send(command)
                capture.send("STARTUP PULSE 1 l 100")
                self.assertEqual(port.writes, [b"STARTUP PULSE 1 l 100\n"])
            finally:
                capture.close()

    def test_device_rejection_preserves_raw_and_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            port = FakePort([b"STARTUP_REJECT,reason=pulse\r\n"])
            capture = startup.Capture(port, Path(directory))
            try:
                with self.assertRaises(ValueError):
                    capture.read()
            finally:
                capture.close()
            self.assertIn(b"STARTUP_REJECT", (Path(directory) / "raw_uart.log").read_bytes())

    def test_reset_after_arming_fails(self):
        frame = ("STARTUP_PROFILE," + ",".join(f"{key}={value}" for key, value in startup.PROFILE.items()) + "\r\n").encode()
        with tempfile.TemporaryDirectory() as directory:
            capture = startup.Capture(FakePort([frame]), Path(directory))
            capture.armed = True
            try:
                with self.assertRaises(ValueError):
                    capture.read()
            finally:
                capture.close()

    def test_watchdog_needs_ack_and_reset_flag(self):
        profile = "STARTUP_PROFILE," + ",".join(f"{key}={value}" for key, value in startup.PROFILE.items())
        frames = [b"STARTUP_WATCHDOG,requested=1\r\n", (profile + ",reset_flags=536870912\r\n").encode()]
        with tempfile.TemporaryDirectory() as directory:
            capture = startup.Capture(FakePort(frames), Path(directory))
            capture.watchdog_pending = True
            try:
                capture.read()
                self.assertFalse(capture.watchdog_observed)
                capture.read()
                self.assertTrue(capture.watchdog_observed)
            finally:
                capture.close()

    def test_intentional_idle_reset_resynchronizes_partial_frame(self):
        profile = "STARTUP_PROFILE," + ",".join(f"{key}={value}" for key, value in startup.PROFILE.items())
        frames = [b"STARTUP_WATCHDOG,requested=1\r\n", b"STARTUP_SAMPLE,id=0,",
                  ("garbage" + profile + ",reset_flags=536870912\r\n").encode()]
        with tempfile.TemporaryDirectory() as directory:
            capture = startup.Capture(FakePort(frames), Path(directory))
            capture.watchdog_pending = True
            try:
                capture.read()
                capture.read()
                capture.read()
                self.assertTrue(capture.watchdog_observed)
            finally:
                capture.close()


if __name__ == "__main__":
    unittest.main()
