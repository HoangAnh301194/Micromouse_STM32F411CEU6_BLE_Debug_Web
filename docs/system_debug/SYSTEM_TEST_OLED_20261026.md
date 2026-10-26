# PA0 System Test (no OLED) — 2026-10-26

The optional PlatformIO profile `system_test_96m` is now fully **headless**.
The SSD1306 OLED is not probed, initialized, drawn, or required for arming.
This change only affects the opt-in test profile; production/IR-OLED benchmarks
remain separate.

## Start and stop
1. Connect UART1 PA9 (TX) -> 3.3V USB-TTL RX, PA10 (RX) <- TX and common GND.
   Monitor at **115200 8N1**. Connect the physical independent motor VM cutoff.
2. Power/reset board without pressing PA0. Keep board stationary during
   automatic MPU calibration (nominal ~9s, may require retries).
   The firmware prints `ST,CAL` every second until finished and then
   `ST,CAL_OK` followed by `ST,READY`.
3. **One press and release of PA0** (>=40ms) starts the current mode.
   Each successful result advances: `TURN90` -> `TURN180` ->
   `STRAIGHT200` -> `TURN90`. No automatic re-run.
4. **Any new PA0 press during movement locks motor outputs from TIM11**.
   UART1 `S` is a second STOP method. No serial/BLE start command exists.
   After a fault/STOP/TIMEOUT the same mode is retained for a retry.
5. If a button is already held when boot finishes, release is ignored;
   pre-calibration presses never queue motion for later.

## Control
- Turn 90° or 180°: uses gyro-integrated yaw PID feedback **only** for angle;
  both wheels run opposite directions. A successful stop requires angle
  error <=2° and angular velocity <=12°/s for >=120ms.
- Straight 200mm: encoder average distance P control, IMU yaw PID heading.
  Test-only geometry: wheel **radius 34mm**, decoded quadrature 1430
  counts/wheel-revolution. => 0.14940 mm/count, ~1338.79 counts per 200mm.
  Production `include/pinout.h` diameter 35mm remains **unchanged**.
- Control tick 1kHz TIM11, PID sample 200Hz. PWM cap **260/1000**;
  exploratory gains are *not tuned*. Runtime watchdog 300ms, gyro stale
  >3ms, angle anomaly, encoder mismatch and mode timeouts lock motors.
- Timeout: 90° 4.5s, 180° 7.5s, straight 4.5s.
  Stopping at 200 mm on the *encoder estimate* cannot guarantee 200mm
  of actual floor travel; measure inertia/slip/overshoot externally.
- IR obstacle avoidance is not enabled in these tests.

## UART frames (actual format)
```text
ST,HELLO,profile=PA0_NO_OLED,locked=1
ST,BOOT,no_oled=1,locked=1,first=TURN90,PA0=PRESS_RELEASE,S=STOP
ST,CAL,ms=1000,state=1,ok=0,retry=0,reason=0,count=0,good=...,err=...,age_us=...,locked=1
ST,CAL_OK,ms=...,good=...,locked=1
ST,READY,mode=0,name=TURN90,action=PRESS_PA0
ST,START,mode=0,name=TURN90,target=90,pwm_limit=260,R_mm=34,CPR=1430
ST_DATA,ms=...,mode=0,yaw_cdeg=...,gz_0p1dps=...,dist_0p01mm=...,L=...,R=...,PWM_L=...,PWM_R=...
ST,RESULT,mode=0,reason=DONE,yaw_cdeg=...,dist_0p01mm=...,L=...,R=...,ticks=...,locked=1
ST,READY_NEXT,mode=1,name=TURN180,action=PRESS_PA0
```
Live `ST_DATA` is emitted at ~20Hz. Values `yaw_cdeg` and
`dist_0p01mm` are scaled integers (divide by 100), `gz_0p1dps`
divide by 10. A `?` serial input prints additional calibration status.
If calibration remains 0 indefinitely, STOP and diagnose IMU readings;
**do not bypass calibration to force movement**.

## Upload
```powershell
git pull origin giomuadongbac
python -m platformio run -e system_test_96m -t upload
python -m serial.tools.miniterm COM7 115200
```
Press RESET after opening miniterm to capture early boot messages.

## Safety / hardware acceptance
The CI build proves compilation, not physical navigation.
First test lifted/secured wheels under guarded VM supply, with operator
and independent motor power cutoff. Check PWM and yaw polarity against
robot motion. Validate emergency stop, calibration, 90°/180° measured
angles and repeat 200mm floor traversals before assuming accuracy.
