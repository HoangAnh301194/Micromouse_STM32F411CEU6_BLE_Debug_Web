# JDY-33 USART2 PA3 RX — A/B/C hardware qualification

**Goal:** distinguish genuine MCU USART2 reception from merely successful
Windows WebBluetooth/GATT writes before considering BLE-based PID parameter
adjustment. Hardware has recently passed **TX-only** 16x16 map CRC32 twice.
Previous sessions returned some HELLO/PING replies while later sessions lost
all commands; this is **not sufficient** to prove permanent RX hardware
failure. This pipeline measures RX using an independent wired UART1 witness.

## Single command (Windows PowerShell)

Disconnect the Web BLE page and any Bluetooth serial apps from Nezumi_Chan;
connect STM32 ST-Link and independent PA9/PA10 USB-TTL on **COM7**, 115200.

```powershell
cd D:\Micromouse_STM32F411CEU6_BLE_Debug_Web
git switch giomuadongbac
git pull --rebase origin giomuadongbac
python tools/ble_rx_qualification.py --port COM7 --name Nezumi_Chan
```

A single invocation: install missing Python packages (`platformio`,
`pyserial`, `bleak`), flash isolated `ble_rx_diag_96m` using ST-Link,
connect JDY-33 FFE1, run A/B/C, generate all evidence, **set MODE B** and
restore motor-locked `flash_map_diag_96m` firmware by ST-Link on exit.
No manual firmware switches, Miniterm, Web BLE page or repeated commands
during the test. The Windows COM port must not be open in another app.
If the adapter advertises an unusual name, supply `--address <BLE ID>`.

**Safety:** The diagnostic locks motor outputs permanently until reboot,
and provides NO motor commands, PID gains, movement, Flash saves/erases or
`ARM` commands. Flash uploads still require an ST-Link configuration that
preserves existing data sectors 6 and 7. Review/backup important Flash data
and keep motor VM isolated before flashing. If automatic Flash restoration
fails, physically cut VM and reflash the locked image manually; no tool can
guarantee power safety after flashing fails. `--no-flash` only for an
already flashed RX diagnostic image; it does not attempt to restore.
`--no-restore` leaves the motor-locked RX diagnostic in safe MODE B.

## A/B/C test design (same firmware image)

| Phase | USART2 RE/RXNEIE | BLE TX | Incoming BLE command | Independent evidence |
|---|---|---|---|---|
| A | ON | Automatic `RHB` every 200 ms | None | COM7 `RXS` RX/error counters vs BLE beacon arrival |
| B | OFF | Same automatic `RHB` every 200 ms | None | COM7 `RXS` RX/error counters vs BLE beacon arrival |
| C | ON | Same `RHB` + nonce echoes | GATT `C,NNNN,HHHHHHHH\n` | MCU COM7 `RXACK`, BLE `RACK`, RX corruption/error counters |

A/B differs **only by the receiver enable bit and RXNE IRQ enable bit**.
The default baseline lasts 6 seconds for each mode. Mode C sends 20 nonce
commands at each frequency 2, 5, 10 and 20 Hz (16 bytes including LF,
well within default 20-byte ATT payload). The nonce for each serial number
is independently computed by the host and checked against the MCU's echoed
32-bit hex value. All mode changes use UART1, not BLE RX.

Accepted bytes are counted by existing USART2 RXNE ISR; errors include
overrun `ORE`, framing `FE`, noise `NE` and ring overflow. UART1
`RXACK,seq=...,value=...` proves the MCU decoded those bytes; BLE
`RACK` tests the return direction separately. A successful GATT write
alone proves **neither** of those. A/B incoming RX bytes from other
sources while no commands were intentionally sent may indicate unwanted
traffic/noise; it is not by itself evidence of a physical fault.

## How to interpret results

- `RX_FUNCTIONAL_IN_TESTED_RANGE`: **all** independent UART1 nonce
  confirmations match at all tested frequencies without write exceptions,
  malformed commands or UART RX errors. This rejects the hypothesis that
  PA3 is completely dead in the tested setup; it does not prove indefinite
  reliability or suitability for safety-critical motion control.
- `RX_INTERMITTENT_OR_RATE_LIMITED`: at least some commands arrive but
  some are missing/corrupt, or results vary by rate. Verify error counters,
  BLE ACKs and Windows GATT scheduling. Not safe for live PID tuning.
- `RX_NOT_VERIFIED_CHECK_GATT_AND_PA3`: GATT claims accepted writes but
  **zero** nonce ACK appears on COM7. Suspect downlink or USART2 path,
  BUT this does not localize a hardware break without checking JDY-33
  TXD/PA3 with an oscilloscope/logic analyzer.
- `NOT_TESTABLE_NO_GATT_WRITE`: no supported/writeable FFE1 transaction
  could be verified; not equivalent to a broken USART2 RX.
- If RX ON phase A shows TX beacons missing compared with B while
  UART1 errors increase, a floating/noisy or heavily interrupting RX
  is a possible cause. Correlation is not proof: compare repeated runs,
  physical UART signals and GATT traffic.

Each run writes `results/ble_rx_ab_YYYYMMDD_HHMMSS/` with:

- `ble_rx_qualification.json` — overall verdict, A/B baselines and all
  rate-specific C outcomes
- `ble_rx_rates.csv` — ACK vs GATT writes, RX error counts by frequency
- `ble_rx_uart1.log` and `ble_rx_uart1_events.json` — independent MCU
  UART1 ground truth
- `ble_rx_notifications.log`, `ble_rx_ble_events.json`,
  `ble_rx_gatt_notifications.json` and `ble_rx_raw.bin` — PC BLE capture
- `flash_ble_rx_diag_96m.log` and `flash_flash_map_diag_96m.log` —
  ST-Link firmware transition evidence

`*.bin` may be gitignored, so preserve/send `ble_rx_gatt_notifications.json`
if uploading results to GitHub.

Offline regression: `python tools/test_ble_rx_qualification.py`.
Firmware CI profile: `python -m platformio run -e ble_rx_diag_96m`.

## PID tuning follow-up: deliberately not implemented here

Even if RX passes, **do not directly feed unbounded Web BLE commands into
a 1kHz ISR or apply gains without limits**. For future PID tuning use a
separate motor-off/raised-wheel authorization, parameter whitelist, bounds,
CRC+sequence, explicit COMMIT/ROLLBACK, stale-link timeout, hardware
command watchdog and independent VM cutoff. The tuning control loop remains
local on STM32; BLE is only a slow supervisory channel (not an ISR).
First collect this A/B/C evidence, then design a guarded tuning protocol.
