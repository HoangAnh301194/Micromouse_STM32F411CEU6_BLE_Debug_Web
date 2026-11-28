"""Source and syntax regression checks for restored MPU Test."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "src" / "main_runtime.c"
APP = ROOT / "tools" / "classic_spp_dashboard.py"
TAB = ROOT / "tools" / "mpu_test_tab.py"


class MPURestoreTest(unittest.TestCase):
    def test_python_sources_compile(self):
        for path in (APP, TAB):
            with self.subTest(path=path.name):
                compile(path.read_text(encoding="utf-8"), str(path), "exec")

    def test_firmware_spp_commands_and_guards(self):
        source = FW.read_text(encoding="utf-8")
        for name in ("MPU_TEST_START", "MPU_TEST_STOP", "MPU_TEST_ZERO"):
            self.assertIn('strcmp(verb, "' + name + '")', source)
        self.assertIn("MPU_TEST_ACTIVE", source)
        self.assertIn("if (mpu_test_active && c != 's'", source)
        self.assertIn("if (ready && Motion_IsDone() && !mpu_test_active)", source)
        self.assertIn("MPU_TEST,STOP,EMERGENCY", source)
        self.assertIn("Motion_Stop(); /* PWM zero before any sampling. */", source)
        self.assertIn("(uint32_t)(now - mpu_test_last_ms) >= 50U", source)
        self.assertIn("age_us <= 3000U", source)
        self.assertIn("MPU,ms=%lu,valid=1", source)
        self.assertIn("MPU,ms=%lu,valid=0", source)

    def test_dashboard_wiring_and_csv(self):
        app = APP.read_text(encoding="utf-8")
        tab = TAB.read_text(encoding="utf-8")
        self.assertIn("from mpu_test_tab import MPUTestTab", app)
        self.assertEqual(app.count("self.mpu_test_tab.on_spp_line(line)"), 1)
        self.assertIn('self.tabs.add(self.mpu_test_tab, text="MPU Test")', app)
        self.assertIn('"mode 7 - mpu test - log receiver"', tab)
        self.assertIn('self._writer.writerow(("ms", "valid", "age_us", "yaw_deg", "gz_dps"))', tab)
        self.assertIn("MPU_TEST_START", tab)
        self.assertIn("MPU_TEST_STOP", tab)
        self.assertIn("MPU_TEST_ZERO", tab)
        self.assertIn("self._open_csv()", tab)
        self.assertIn("self._close_csv()", tab)


if __name__ == "__main__":
    unittest.main()
