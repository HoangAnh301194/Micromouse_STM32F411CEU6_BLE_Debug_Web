# Verification: 96 MHz nominal PLL + HSITRIM=25 (motor OFF)

This is a **one-profile hardware diagnostic**, NOT a production firmware
change. The HSI AutoTune scan with **nominal PLL 80 MHz** found that trim25
gave the smallest measured clock error, **-0.148764%**, at
`clock_logs/auto_hsi_20261011_225413`. Trim26 overshot to
+0.231764%. These PC-UART measurements suggest trim25 is worth testing under
a lower-than-maximum normal-speed PLL configuration.

## Firmware change, strictly opt-in

PlatformIO environment: **`clock_96m_trim25`**:

- `-DCLOCK_VALIDATE_96M=1 -DCLOCK_HSI_TRIM=25`
- PLL source HSI; PLLM=16, PLLN=192, PLLP=2, PLLQ=5
- SYSCLK/HCLK **96 MHz nominal**; APB1 **48 MHz nominal**; APB2 **96 MHz**
- APB1 timer clock **96 MHz** (APB1 /2 with timer ×2)
- TIM5 programmed for **1 MHz nominal** using 96-to-1 division
- TIM11 programmed for **1 kHz nominal** using 96-to-1 then 1000-to-1
- Motor initialization and drive code remain excluded by the **isolated
  diagnostic** `main.c` (`#if 0` around the old motor harness).
- Log output stays full enough for clock, I2C/IMU and TIM11 checks:
  `CLK, IMU, FRESH, TIMING, CAL_DONE, CAL_CHECK` are retained;
  periodic `NOISE/CAL` summaries are suppressed in this profile to reduce
  DMA UART ring pressure.
- Normal production-profile clock configuration is unchanged:
  `genericSTM32F411CE` still uses factory HSI trim and PLLN=200
  (100 MHz **nominal**).

The HSI scan suggests that **physical SYSCLK** may be near:

```text
96 MHz × 0.99851236 = 95.8572 MHz (prediction, not new measurement)
TIM5 ≈ 998512 Hz; TIM11 ≈ 998.5 Hz
```

An exact result must be measured on the **96MHz** build. External heating,
supply voltage, and board-specific HSI behavior can shift it. STM32F411
maximum specified SYSCLK is 100 MHz; the post-measurement validator uses
98 MHz as a conservative review cutoff, NOT a hardware protection device.

## Commands for Windows PowerShell

Use branch `feature/motor-debug-console`. Close other Serial
Monitors. Leave ST-Link and USB-TTL connected, motor drive power disabled.

```powershell
cd D:\Micromouse_STM32F411CEU6_BLE_Debug_Web
git switch feature/motor-debug-console
git status -sb
git fetch origin
git merge --ff-only origin/feature/motor-debug-console
```

If branches diverge, resolve manually while preserving local commits. Do
not use `git reset --hard` or force-push.

**Step 1: Validate PC-side checker before touching hardware.**

```powershell
python tools/clock_96m_validate.py --self-test
```

Expected: `SELF-TEST PASSED`.

**Step 2: Build first, flash only after SUCCESS.**

```powershell
python -m platformio run -e clock_96m_trim25
python -m platformio run -e clock_96m_trim25 --target upload
```

**Step 3: Capture 240 seconds.**

```powershell
python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 240
```

AFTER Python opens COM7, press hardware RESET **once**, then keep the board
completely stationary. Do not press KEY, attach motor power or run another
serial program.

**Step 4: Immediately reflash the original/default diagnostic firmware
after capture (do not leave the experimental profile deployed).**

```powershell
python -m platformio run -e genericSTM32F411CE --target upload
```

**Step 5: Validate captured data offline.**

```powershell
$run = Get-ChildItem .\clock_logs -Directory |
  Where-Object { Test-Path (Join-Path $_.FullName "raw_serial.log") } |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1

Select-String -Path "$($run.FullName)\raw_serial.log" -Pattern "CLOCK_PROFILE|CLOCK_TRIM|CLOCK_DIAG|CAL_DONE|CAL_CHECK"

python tools/clock_96m_validate.py --folder "$($run.FullName)"
```

Required boot markers:

```text
CLOCK_TRIM,mode=override requested=25 active=25 ...
CLOCK_PROFILE,mode=validate96,trim=25,plln=192,nominal_hz=96000000,motor_off=1
CLOCK_DIAG,fault=0 ...
```

Note: the older `CLOCK,sysclk=96000000 hsi_pll100=0` label is still
printed because it literally tests equality against *100MHz*. It is not
a failure. Check `CLOCK_PROFILE` for validation.

The checker issues **PASS (diagnostic only)** only when:
- Clock error magnitude ≤0.5%, estimated SYSCLK below **98 MHz**
- 150 or more regression samples, at least 160 host seconds
- No missing `CLK`, firmware TX drops, suspicious 60s slope/jitter
- Correct PLLN=192, HSITRIM=25, nominal SYSCLK=96MHz boot/frame readback
- No logged IMU I2C error/timeout/UART DMA drop
- 4000-sample, 8-block calibration and 120s drift check present
- No TIM11 gaps over 1500 nominal microseconds

The thresholds are **screening criteria for this one diagnostic run**,
not proof of a formally bounded ISR deadline or oscillator stability.
The tester may warn if I2C falls back to 100kHz, gyro freshness has stale
samples, or TIM11 interval jitter exceeds the nominal +/-50us threshold.

## After the measurement

Push the new `clock_logs/<session>` folder, including
`raw_serial.log` and `clock_summary.txt`. Review the
validator's output and compare effective timing with the prior 80MHz
autotune. If this passes, *next* verify several times under warmed board
and changing supply voltage. If that also passes, the team may evaluate
production clock configuration; it should NOT be made permanent now.

This procedure does not check actual PWM drive timing, encoder pulses,
motor torque or robot motion because **motors remain disabled**.
