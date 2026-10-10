# Motor Debug — full PWM range, continuous test, BLE heartbeat

Branch: `feature/motor-debug-console` (based on `giomuadongbac`).

## What changed

- Signed motor test commands: **-100% to +100%** (full command range, no 30% software cap).
- Test duration is a 32-bit unsigned integer in milliseconds. **0 means continuous** until STOP, heartbeat loss or Control_Stop; positive values stop at their set duration (no 1000 ms cap).
- **Separate safety watchdog:** browser sends `MT HB` about every 400 ms while running. TIM11 stops motors if no heartbeat for **1500 ms**. This is not a test-duration limit.
- BLE telemetry remains **1–10 Hz**, default **5 Hz**, formatted in the **main loop**, not in the 1 kHz control ISR.
- `Control_Stop()` stops any running Motor Test before disabling TIM11.
- The Web Motor Debug tab includes a STOP button, auto-scrolling log, encoder/PPS/RPM readouts, and CSV export.

## Usage

1. Confirm TB6612FNG wiring, supply, motor rated voltage/current. A 2S lithium battery can supply **8.4 V when full**; unknown N20 motor voltage ratings mean 100% PWM may exceed the motor's intended voltage or driver current.
2. Lift both wheels clear of the ground, keep hands away and prepare a physical power disconnect. Start with modest PWM, then increase only if measured conditions justify it.
3. Build/flash the branch and open `tools/ble_debug_console.html` using a browser supporting Web Bluetooth and JavaScript from the chosen origin.
4. Open the **Motor Debug** tab, connect BLE, then **Verify firmware**. The updated firmware responds `MTREADY,2`.
5. Set Wheel, PWM L/R (%), Duration (ms; 0 = continuous), Log period (100–1000 ms) and encoder counts/rev. Press RUN and monitor telemetry.
6. Press STOP to terminate. Switching tabs, hiding the browser tab or disconnecting from the Motor Debug connect button requests STOP; loss of heartbeat is a fallback after 1.5 s.

## BLE protocol (ASCII, newline-delimited)

Commands:
```
MT PING
MT RUN <left_percent> <right_percent> <duration_ms> <log_interval_ms>
MT HB
MT STOP
```

Example full-duty continuous run for the left wheel alone:
```
MT RUN 100 0 0 200
```
The browser sends `MT HB` periodically while running. The firmware does **not** accept unlimited running without this keepalive.

Responses:
```
MTREADY,2
MTACK,RUN,<left_percent>,<right_percent>,<duration_ms>,<log_interval_ms>
MTACK,STOP
MTERR,<reason>
MTDATA,<elapsed_ms>,<pwm_left>,<pwm_right>,<encoder_left>,<encoder_right>,<left_pps>,<right_pps>,<delta_ms>
MTEND,TIMEOUT
MTEND,STOP
MTEND,HEARTBEAT_LOST
```

## Notes and limitations

- `src/motor.c` and `include/motor.h` have **not** been changed in this branch: the new test accepts the full existing percentage API. The user's separate permille/deadband patch is **not included** here.
- No direction reversal interlock, motor-current limit, VM reading, hardware watchdog, stall protection, or acceleration ramp has been added. Do not directly command high-speed reversal.
- Heartbeat depends on TIM11 and browser BLE availability. It cannot protect against all electrical or MCU faults.
- The existing firmware still requires successful MPU6050 initialization/calibration before entering the main loop.
- Firmware has been committed but **has not been verified by a complete PlatformIO build or physical STM32 test**.
