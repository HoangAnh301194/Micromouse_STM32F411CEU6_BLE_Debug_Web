from __future__ import annotations

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import motor_interface_contract as contract


class MotorInterfaceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = {name: (ROOT / name).read_text(encoding="utf-8")
                        for name in contract.FILES}

    def case(self, filename, before, after, error):
        sources = self.original.copy()
        self.assertIn(before, sources[filename])
        sources[filename] = sources[filename].replace(before, after, 1)
        with self.assertRaisesRegex(contract.ContractError, error):
            contract.inspect_contract(sources)

    def test_frozen_integration_contract_pass(self):
        self.assertEqual(contract.audit()["status"], "PASS_MOTOR_INTERFACE_CONTRACT")

    def test_motion_direct_register_write_rejected(self):
        self.case("src/motion.c", "#include \"motor.h\"",
                  "#include \"motor.h\"\nTIM2->CCR1=100;", "bypasses")

    def test_motion_keepalive_cannot_mask_stalled_pid(self):
        self.case("src/motion.c", "Motor_Stop();", "Motor_KeepAlive(); Motor_Stop();",
                  "bypasses")

    def test_imu_stale_still_neutralizes(self):
        self.case("src/control.c",
                  "            Motion_Stop();\n", "            /* ignored */\n",
                  "Expired IMU")

    def test_scheduler_neutralizes_before_timer_stop(self):
        self.case("src/control.c", "    Motion_Stop();\n    TIM11->CR1 &= ~TIM_CR1_CEN;",
                  "    TIM11->CR1 &= ~TIM_CR1_CEN;\n    Motion_Stop();",
                  "Control_Stop")

    def test_motion_never_controls_raw_pwm_pins(self):
        self.case("src/motion.c", "Motor_SetPair(ClampPwm(-output), ClampPwm(output));",
                  "TIM2->CCR1 = 1;", "bypasses")

    def test_wrong_pin_mapping_rejected(self):
        self.case("include/pinout.h", "#define MOTOR_R_IN2_PIN        14",
                  "#define MOTOR_R_IN2_PIN        13", "pin mapping drifted")

    def test_wrong_watchdog_contract_rejected(self):
        self.case("include/motor.h", "#define MOTOR_COMMAND_TIMEOUT_MS 300U",
                  "#define MOTOR_COMMAND_TIMEOUT_MS 999U", "300ms")

    def test_wrong_schedule_rejected(self):
        self.case("include/control.h", "#define CONTROL_FREQUENCY_HZ  1000UL",
                  "#define CONTROL_FREQUENCY_HZ  500UL", "frequency changed")

    def test_motion_must_renew_real_command(self):
        self.case("src/motion.c",
                  "Motor_SetPair(ClampPwm(left_pwm), ClampPwm(right_pwm));",
                  "(void)left_pwm; (void)right_pwm;", "Straight Motion")


if __name__ == "__main__":
    unittest.main()
