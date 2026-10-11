# Automated HSI trim survey — STM32F411 Micromouse

This is a **diagnostic experiment**, not a production-clock calibration.
Run only with the **isolated motor-OFF MPU6050 main firmware** on branch
`feature/motor-debug-console`. Connect ST-Link and USART1 USB–TTL
(COM7 by default); **close PlatformIO Serial Monitor** first.

## Why the scan runs with a nominal 80 MHz PLL

The STM32F411 maximum SYSCLK is 100 MHz. A direct trim scan with a nominal
100 MHz PLL can accidentally exceed this limit before a PC measurement is
available. `CLOCK_AUTOTUNE=1` switches the otherwise-identical PLL configuration
to **PLLN=160**, making the nominal SYSCLK **80 MHz**, while production and
the original `clock_hsi_trim17..20` configurations remain at PLLN=200 / 100MHz.

TIM5 is still configured to advance one *nominal* microsecond per count and
TIM11 remains nominally 1 kHz. The PC measures the physical counter rate:
- observed TIM5 1,000,000 Hz means HSI is near nominal 16 MHz;
- SYSCLK during scanning ≈ **80 × TIM5_Hz / 1,000,000 MHz**;
- a later 100MHz PLL configuration, **not performed by this script**, must be
  validated separately before being considered for deployment.

This reduces risk but is not a mathematical guarantee across arbitrary
oscillator/voltage/temperature faults. The script stops its scan if an
80MHz-profile estimate reaches **90 MHz** or reports bad telemetry. It limits
the permitted survey to **HSITRIM 16..26** and never auto-scans 27..31.

## Windows instructions

First pull the latest branch. If local/remote commits have diverged, STOP
and merge deliberately; **do not reset --hard or force-push**.

```powershell
cd D:\Micromouse_STM32F411CEU6_BLE_Debug_Web
git switch feature/motor-debug-console
git status -sb
git fetch origin
git merge --ff-only origin/feature/motor-debug-console

python -m pip install pyserial matplotlib
python tools/hsi_autotune.py --self-test
```

The self-test does **not** access the board or upload firmware. It checks
the tagged CLK parser, generated PlatformIO environment, and calculation logic.

Start the fully automated experiment:

```powershell
python tools/hsi_autotune.py --port COM7 --start 20 --end 26 --seconds 60
```

The script:
1. Ensures the expected motor-OFF diagnostic source and 80MHz PLL guard exist.
2. Creates an **ignored temporary PlatformIO config**
   `.pio/hsi_autotune_project.ini` with trim set at build time
   (no more manually adding envs 21, 22, ... to tracked platformio.ini).
3. **Cleans, compiles and uploads** the diagnostic firmware using ST-Link.
4. Opens COM7 itself. **No RESET press needed** because each 1Hz
   `CLK` line contains `trim`, `fault`, and
   `sysclk_nominal` to authenticate this very firmware.
5. Collects a 60-second host/MCU slope, checks the first 12 timestamps for
   a dangerously high rate, and validates 30-second robust slope, missing
   messages, and USB timestamp jitter.
6. Moves to the next trim if valid, stopping at the first crossing of the
   1MHz TIM5 target or the configured maximum.
7. Saves a comparison and recommended *test candidate*. **The script does
   not promote any trim into production firmware.**
8. Reflashes the original `genericSTM32F411CE` environment in
   `finally`, including on normal finish, measurement failures and Ctrl+C.

If interrupted by a power failure or process kill, reflashing may not be
possible. Recover manually with:

```powershell
python -m platformio run -e genericSTM32F411CE --target upload
```

## Outputs

A new session appears in `clock_logs/auto_hsi_YYYYMMDD_HHMMSS/`:

```text
auto_hsi_YYYYMMDD_HHMMSS/
  trim20/
    clock_samples.csv
    raw_serial.log
    clock_summary.txt
    autotune_metrics.json
    01_clock_drift.png
    02_serial_timestamp_jitter.png
  trim21/
    ...
  comparison.csv
  summary.json
```

If a run fails validation, the directory for that trim still contains the
available CSV/raw serial logs. The next trim is not attempted. Review before
rerunning.

## Safety and interpretation

- **Never attach drive power/start motors while scanning.** The firmware is
  designed for motor-OFF diagnostics; run on a secured stationary robot.
- Serial COM7 and ST-Link must both remain connected.
- `SystemCoreClock` and `sysclk_nominal` are nominal values derived from PLL
  registers, **not physical measurements**.
- The PC's `perf_counter` provides a useful approximate reference, not a
  calibrated frequency counter. USB timestamp jitter and host power settings
  can influence the estimate.
- For the chosen trim, perform an independent **180–300-second verification
  with the normal PLLN=200 (nominal 100MHz) configuration** and guard against
  exceeding the 100MHz STM32F411 limit. Verify at different board temperatures
  and supply voltages before using it for timing-critical control.
- Do **not** automatically write the selected trim into default board
  initialization or alter PID / IMU time-step parameters based only on the
  survey results.
