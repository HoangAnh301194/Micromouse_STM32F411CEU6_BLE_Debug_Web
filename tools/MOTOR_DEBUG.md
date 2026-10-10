# Motor Debug — bounded diagnostic mode

Branch: `feature/motor-debug-console`, based on `giomuadongbac`.

## Usage

1. Check TB6612FNG VM/GND/STBY wiring and confirm the motor's voltage/current rating before running sustained tests. A 2S battery reaches about 8.4 V fully charged. The firmware cannot measure VM or motor current.
2. Lift **both wheels clear of the ground** and keep hands away from rotating wheels.
3. Build and flash the feature branch with PlatformIO. Start `tools/ble_debug_console.html` through a browser environment supporting Web Bluetooth and local JavaScript file loading (Chrome/Edge, secure origin if required).
4. Select the **Motor Debug** top-level tab. Connect JDY-33 BLE and choose **Verify firmware**. RUN is disabled until `MTREADY,1` arrives.
5. Begin with a single wheel at low PWM and a short duration. Verify direction and counts. Negative duty commands reverse direction; do not reverse abruptly during a running test.
6. Observe the separate auto-scrolling motor terminal, export data as CSV, and test the other wheel. Do not run both wheels on the ground until the direction mapping has been verified.

## Inputs

- Wheel: Left/Right/Both. The unselected wheel gets PWM 0.
- Signed left and right PWM: integer -30..30 percent; at least one must be nonzero.
- Duration: 100..1000 ms. Firmware TIM11 controls the deadline.
- Telemetry interval: 100..1000 ms (default 200 ms, 5 Hz).
- Encoder counts per wheel revolution: default 1430 is **unverified** and affects only the browser RPM calculation, not firmware.

## BLE line protocol (ASCII, newline-delimited)

Commands:

```
MT PING
MT RUN <left_pwm> <right_pwm> <duration_ms> <log_interval_ms>
MT STOP
```

Responses:

```
MTREADY,1
MTACK,RUN,<left_pwm>,<right_pwm>,<duration_ms>,<log_interval_ms>
MTACK,STOP
MTERR,<reason>
MTDATA,<elapsed_ms>,<left_pwm>,<right_pwm>,<encoder_left>,<encoder_right>,<left_pps>,<right_pps>,<delta_ms>
MTEND,TIMEOUT
MTEND,STOP
```

Motor test is open-loop; it is mutually exclusive with motion primitives. TIM11 at 1 kHz updates encoder/IMU/IR as usual, executes a constant-time test deadline check, and skips `Motion_Update` while the test runs. Telemetry snapshot, integer formatting and non-blocking BLE queue writes happen in the **main loop**, never in the 1 kHz ISR. USART2 TX remains lower interrupt priority than TIM11. RX uses a small IRQ ring buffer to avoid losing packet bytes during main-loop work.

## Caveats

- BLE disconnect may prevent an immediate remote STOP; the currently active command is bounded to at most 1000 ms by firmware. The software cannot guarantee stopping on a stalled TIM11 ISR, lost MCU power, or driver electrical faults.
- STOP by itself is not a permanent inhibit: future valid commands may start new runs.
- The motor driver in the baseline is not yet protected against immediate direction reversal at the hardware PWM level. Run low-duty, single-direction tests first.
- The firmware still boots through MPU6050 initialization and calibration; a failed MPU boot prevents motor test mode.
- No battery voltage, motor current, temperature, stall current, or true duty waveform is measured.
- Real hardware testing and a full PlatformIO build have **not yet been performed**.
