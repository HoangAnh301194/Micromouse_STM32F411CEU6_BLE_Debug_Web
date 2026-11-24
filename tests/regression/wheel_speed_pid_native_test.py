"""Host C-native regression test for src/wheel_speed_pid.c."""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]


C_HARNESS = r"""
#include <math.h>
#include <stdio.h>
#include "wheel_speed_pid.h"
#include "pinout.h"

static int fail(const char *msg) {
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
}

int main(void) {
    const float expected_pulses_per_mm = (float)ENCODER_PPR / (3.14159265f * 34.0f);
    WheelSpeedController_t ctrl;
    WheelSpeedOutput_t out;
    float l_pps = 0.0f, r_pps = 0.0f;

    /* 1. Wheel diameter 34mm & straight cmd_vel conversion */
    if (fabsf(WHEEL_DIAMETER_MM - 34.0f) > 1e-5f) {
        return fail("WHEEL_DIAMETER_MM must be 34.0f");
    }
    WheelSpeedPid_CmdVelToWheelPps(250.0f, 0.0f, &l_pps, &r_pps);
    if (fabsf(l_pps - 250.0f * expected_pulses_per_mm) > 0.5f ||
        fabsf(r_pps - l_pps) > 1e-4f) {
        return fail("Straight cmd_vel conversion mismatch");
    }

    /* 2. +w_ref_dps = left turn -> right wheel faster than left wheel */
    WheelSpeedPid_CmdVelToWheelPps(0.0f, 90.0f, &l_pps, &r_pps);
    if (!(r_pps > 0.0f && l_pps < 0.0f && fabsf(r_pps + l_pps) < 1e-3f)) {
        return fail("Left turn sign convention mismatch");
    }

    /* 3. Saturation cap at 500 permille + anti-windup */
    WheelSpeedPid_Init(&ctrl, 0.050f, 0.080f, 80.0f, 0.080f, 500.0f, 1250.0f);
    for (int i = 0; i < 200; ++i) {
        WheelSpeedPid_UpdateCmdVel(&ctrl, 500.0f, 0.0f, 0.0f, 0.0f, 0.001f, &out);
        if (out.left_pm > 500 || out.right_pm > 500 ||
            out.left_pm < -500 || out.right_pm < -500) {
            return fail("Output exceeded 500 permille cap");
        }
    }
    if (ctrl.left.integral > 1.0f || ctrl.right.integral > 1.0f) {
        return fail("Anti-windup failed to freeze integral during saturation");
    }

    /* 4. Steering-priority desaturation preserves turning differential */
    WheelSpeedPid_Reset(&ctrl);
    WheelSpeedPid_UpdateCmdVel(&ctrl, 420.0f, 90.0f, 0.0f, 0.0f, 0.001f, &out);
    if (out.left_pm > 500 || out.right_pm > 500 ||
        out.left_pm < -500 || out.right_pm < -500) {
        return fail("Desaturation exceeded bounds");
    }
    if (!(out.right_pm > out.left_pm)) {
        return fail("Steering differential lost after desaturation");
    }

    /* 5. Reset clears integral & last state */
    WheelSpeedPid_Reset(&ctrl);
    if (ctrl.left.integral != 0.0f || ctrl.right.integral != 0.0f ||
        ctrl.left.last_output_pm != 0.0f || ctrl.right.last_output_pm != 0.0f) {
        return fail("Reset did not clear state");
    }

    /* 6. Default tuned config & independent left/right gains preserving integral */
    {
        WheelSpeedConfig_t def_cfg, l_cfg, r_cfg, got_l, got_r;
        WheelSpeedPid_GetDefaultConfig(&def_cfg);
        if (fabsf(def_cfg.kp - 0.090f) > 1e-5f ||
            fabsf(def_cfg.ki - 0.250f) > 1e-5f ||
            fabsf(def_cfg.ks_permille - 80.0f) > 1e-4f ||
            fabsf(def_cfg.kv_permille_per_pps - 0.098f) > 1e-5f) {
            return fail("Default tuned gains mismatch");
        }
        WheelSpeedPid_InitConfig(&ctrl, &def_cfg, NULL);
        WheelSpeedPid_UpdateCmdVel(&ctrl, 200.0f, 0.0f, 1500.0f, 1500.0f, 0.01f, &out);
        if (!(ctrl.left.integral > 0.0f)) {
            return fail("Expected positive integral accumulation before SetGains");
        }
        {
            float saved_int = ctrl.left.integral;
            l_cfg = def_cfg;
            r_cfg = def_cfg;
            l_cfg.ks_permille = 75.0f;
            r_cfg.ks_permille = 95.0f;
            WheelSpeedPid_SetGains(&ctrl, &l_cfg, &r_cfg);
            WheelSpeedPid_GetGains(&ctrl, &got_l, &got_r);
            if (fabsf(ctrl.left.integral - saved_int) > 1e-5f) {
                return fail("SetGains should preserve live integral state");
            }
            if (fabsf(got_l.ks_permille - 75.0f) > 1e-4f ||
                fabsf(got_r.ks_permille - 95.0f) > 1e-4f) {
                return fail("Independent left/right ks gains mismatch");
            }
        }
    }

    /* 7. Inverse kinematics round-trip */
    {
        float v_back = 0.0f, w_back = 0.0f;
        WheelSpeedPid_CmdVelToWheelPps(320.0f, -180.0f, &l_pps, &r_pps);
        WheelSpeedPid_WheelPpsToCmdVel(l_pps, r_pps, &v_back, &w_back);
        if (fabsf(v_back - 320.0f) > 0.1f || fabsf(w_back - (-180.0f)) > 0.1f) {
            return fail("Forward/inverse kinematics round-trip mismatch");
        }
    }

    /* 8. Battery compensation & acceleration feedforward */
    {
        WheelSpeedConfig_t cfg;
        WheelSpeedOutput_t out_nom, out_lowbat, out_acc;
        WheelSpeedPid_GetDefaultConfig(&cfg);
        cfg.ka_permille_per_pps2 = 0.010f;
        WheelSpeedPid_InitConfig(&ctrl, &cfg, NULL);
        WheelSpeedPid_UpdateCmdVel(&ctrl, 200.0f, 0.0f, 2677.0f, 2677.0f, 0.001f, &out_nom);

        WheelSpeedPid_Reset(&ctrl);
        WheelSpeedPid_SetBatteryScale(&ctrl, 7.4f, 6.8f);
        WheelSpeedPid_UpdateCmdVel(&ctrl, 200.0f, 0.0f, 2677.0f, 2677.0f, 0.001f, &out_lowbat);
        if (!(out_lowbat.left_pm > out_nom.left_pm)) {
            return fail("Low battery voltage should scale PWM upward");
        }

        WheelSpeedPid_Reset(&ctrl);
        WheelSpeedPid_SetBatteryScale(&ctrl, 7.4f, 7.4f);
        WheelSpeedPid_UpdateCmdVelWithAccel(&ctrl, 200.0f, 0.0f, 500.0f, 0.0f,
                                            2677.0f, 2677.0f, 0.001f, &out_acc);
        if (!(out_acc.left_pm > out_nom.left_pm)) {
            return fail("Positive acceleration feedforward should increase PWM");
        }
    }

    printf("ALL_WHEEL_SPEED_PID_TESTS_PASS\n");
    return 0;
}
"""

MOTION_C_HARNESS = r"""
#include <math.h>
#include <stdio.h>
#include "motion.h"
#include "encoder.h"
#include "motor.h"

static float stub_v_l = 0.0f, stub_v_r = 0.0f;
static int16_t last_pm_l = 0, last_pm_r = 0;
static int stop_count = 0;

void Encoder_Reset(void) { stub_v_l = 0.0f; stub_v_r = 0.0f; }
float Encoder_GetLeftSpeedPPS(void) { return stub_v_l; }
float Encoder_GetRightSpeedPPS(void) { return stub_v_r; }
void Motor_Stop(void) { last_pm_l = 0; last_pm_r = 0; stop_count++; }
void Motor_SetPair(int16_t l, int16_t r) { last_pm_l = (int16_t)(l * 10); last_pm_r = (int16_t)(r * 10); }
void Motor_SetPairPermille(int16_t l, int16_t r) { last_pm_l = l; last_pm_r = r; }
uint8_t Motor_OutputsLocked(void) { return 0U; }

static int fail(const char *msg) {
    fprintf(stderr, "MOTION_FAIL: %s\n", msg);
    return 1;
}

int main(void) {
    Motion_StraightProfile_t sp;
    Motion_TurnProfile_t tp;
    Motion_SpeedDebug_t dbg;
    float v_meas = 0.0f, w_meas = 0.0f;

    Motion_Init();

    /* 1. Configurable straight profile + steering bias + remaining distance */
    Motion_GetDefaultStraightProfile(&sp);
    sp.max_speed_mm_s = 400.0f;
    sp.end_speed_mm_s = 200.0f;
    Motion_CommandStraightProfile(180.0f, 0.0f, &sp);
    Motion_SetSteeringBiasDps(30.0f);
    Motion_Update(0.001f, 100, 100, 0.0f, 0.0f);
    Motion_GetSpeedDebug(&dbg);
    if (Motion_GetState() != MOTION_STRAIGHT || Motion_IsDone()) {
        return fail("StraightProfile should be running");
    }
    if (!(Motion_GetRemainingDistanceMm() > 160.0f && Motion_GetRemainingDistanceMm() < 180.0f)) {
        return fail("Remaining distance calculation mismatch");
    }
    if (!(dbg.right_ref_pps > dbg.left_ref_pps)) {
        return fail("Positive steering bias should steer left (right faster than left)");
    }

    /* 2. Continuous velocity streaming without resetting state */
    Motion_CommandVelocity(300.0f, 0.0f, 1U, 10.0f);
    Motion_Update(0.001f, 0, 0, 10.0f, 0.0f);
    Motion_CommandVelocity(350.0f, -90.0f, 0U, 0.0f);
    Motion_Update(0.001f, 0, 0, 10.0f, 0.0f);
    Motion_GetSpeedDebug(&dbg);
    if (Motion_GetState() != MOTION_VELOCITY || !(dbg.left_ref_pps > dbg.right_ref_pps)) {
        return fail("Velocity streaming right turn mismatch");
    }

    /* 3. Closed-loop gyro turn profile (+90 deg left) */
    Motion_GetDefaultTurnProfile(&tp);
    Motion_CommandTurnProfile(90.0f, 0.0f, &tp);
    Motion_Update(0.001f, 0, 0, 0.0f, 0.0f);
    Motion_GetSpeedDebug(&dbg);
    if (Motion_GetState() != MOTION_TURN_PROFILE || !(dbg.right_ref_pps > 0 && dbg.left_ref_pps < 0)) {
        return fail("TurnProfile +90 should command left rotation");
    }
    /* Reach target yaw and low gyro rate -> completes cleanly */
    Motion_Update(0.001f, 0, 0, 89.0f, 2.0f);
    if (!Motion_IsDone() || Motion_GetState() != MOTION_IDLE) {
        return fail("TurnProfile did not finish within tolerance");
    }

    /* 4. Chassis velocity readout */
    stub_v_l = 2000.0f;
    stub_v_r = 2000.0f;
    Motion_GetChassisVelocity(&v_meas, &w_meas);
    if (!(v_meas > 140.0f && v_meas < 160.0f && fabsf(w_meas) < 0.1f)) {
        return fail("Chassis velocity inverse kinematics readout mismatch");
    }

    printf("ALL_MOTION_API_TESTS_PASS\n");
    return 0;
}
"""


class WheelSpeedPidNativeTest(unittest.TestCase):
    def test_wheel_speed_pid_c(self) -> None:
        gcc = shutil.which("gcc")
        self.assertIsNotNone(gcc, "gcc not found on PATH")
        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)
            (tmp_dir / "stm32f4xx.h").write_text("/* host stub */\n", encoding="utf-8")
            harness_c = tmp_dir / "test_wheel_speed_pid.c"
            harness_exe = tmp_dir / "test_wheel_speed_pid.exe"
            harness_c.write_text(C_HARNESS, encoding="utf-8")
            compile_res = subprocess.run(
                [
                    gcc,
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    f"-I{tmp_dir}",
                    f"-I{REPO_ROOT / 'include'}",
                    str(REPO_ROOT / "src" / "wheel_speed_pid.c"),
                    str(harness_c),
                    "-lm",
                    "-o",
                    str(harness_exe),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                compile_res.returncode,
                0,
                f"Compile failed:\n{compile_res.stdout}\n{compile_res.stderr}",
            )
            run_res = subprocess.run(
                [str(harness_exe)], capture_output=True, text=True
            )
            self.assertEqual(
                run_res.returncode,
                0,
                f"Test failed:\n{run_res.stdout}\n{run_res.stderr}",
            )
            self.assertIn("ALL_WHEEL_SPEED_PID_TESTS_PASS", run_res.stdout)

    def test_motion_extended_apis_c(self) -> None:
        gcc = shutil.which("gcc")
        self.assertIsNotNone(gcc, "gcc not found on PATH")
        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)
            (tmp_dir / "stm32f4xx.h").write_text(
                "#ifndef STM32F4XX_H_STUB\n"
                "#define STM32F4XX_H_STUB\n"
                "#include <stdint.h>\n"
                "static inline uint32_t __get_PRIMASK(void) { return 0U; }\n"
                "static inline void __disable_irq(void) {}\n"
                "static inline void __enable_irq(void) {}\n"
                "#endif\n",
                encoding="utf-8",
            )
            harness_c = tmp_dir / "test_motion_api.c"
            harness_exe = tmp_dir / "test_motion_api.exe"
            harness_c.write_text(MOTION_C_HARNESS, encoding="utf-8")
            compile_res = subprocess.run(
                [
                    gcc,
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    f"-I{tmp_dir}",
                    f"-I{REPO_ROOT / 'include'}",
                    str(REPO_ROOT / "src" / "wheel_speed_pid.c"),
                    str(REPO_ROOT / "src" / "pid.c"),
                    str(REPO_ROOT / "src" / "motion_primitives.c"),
                    str(REPO_ROOT / "src" / "motion.c"),
                    str(harness_c),
                    "-lm",
                    "-o",
                    str(harness_exe),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                compile_res.returncode,
                0,
                f"Compile failed:\n{compile_res.stdout}\n{compile_res.stderr}",
            )
            run_res = subprocess.run(
                [str(harness_exe)], capture_output=True, text=True
            )
            self.assertEqual(
                run_res.returncode,
                0,
                f"Test failed:\n{run_res.stdout}\n{run_res.stderr}",
            )
            self.assertIn("ALL_MOTION_API_TESTS_PASS", run_res.stdout)


if __name__ == "__main__":
    unittest.main()
