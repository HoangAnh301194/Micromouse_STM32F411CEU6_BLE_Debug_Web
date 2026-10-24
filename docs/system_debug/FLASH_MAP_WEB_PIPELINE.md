# Flash map WebBLE + UART1 diagnostic pipeline

> **Restored on `giomuadongbac`:** `tools/flash_map_pipeline.py` is again
> available for the user-requested 5×5/16×16 Flash-map test. The BLE viewer
> remains in `web/ble_debug_console.html` and the native test is under
> `tests/regression/flash_map_protocol_test.py`. The complete restored debug tool suite is now also on the active
> `giomuadongbac` branch; the historical snapshot remains in the [archive branch](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/tree/archive/debug-tools-precleanup-20261024).
> Board-side Flash write / power-loss / JDY-33 acceptance is still pending.


**Branch:** `giomuadongbac`. **Firmware profile:** `flash_map_diag_96m`.

The experimental profile locks motors permanently until reset, does not start
TIM11, IR or MPU6050, and never erases Flash. It only writes one map journal slot
after an explicit `SAVE N CONFIRM` UART command. No physical-board acceptance
has been completed from this chat.

## Hardware and protocol

- Debug/results: **USART1 TX PA9 / RX PA10**, USB-UART, 115200 8N1. Emits `FM_BOOT`, `FM_INFO`, `FM_BEGIN`, `FM_SAVE`, `FM_VERIFY`, `FM_READ`, `FM_CFG`, `FM_ROW`, `FM_DONE`.
- Map display: **JDY-33 BLE USART2 TX PA2 only**, Web Bluetooth service FFE0, characteristic FFE1. STM32 USART2 receive is disabled in the optional Flash Map diagnostic. All debug control commands use UART1 PA10/COM7. Sends `MAPCFG`, one `MAPROW` per row, then `MAPEND`; web verifies complete rows and CRC32 before rendering.
- Two deterministic fake maps: **16×16** and **5×5**, byte header `MAP1`, width, height, two zero bytes, then N×N wall nibbles (N/E/S/W = 1/2/4/8), row zero at bottom. Flood fill is calculated by web; a wall nibble is not treated as an explored flag.
- Existing legacy `main` 5×5 map is readable as a fallback from sector 7, offset zero. A versioned 5×5 or 16×16 new map uses the journal's CRC32-protected record slots.

## PowerShell steps

1. Update the working branch (protect any uncommitted changes):

   `git fetch origin`
   `git switch giomuadongbac`
   `git pull --rebase origin giomuadongbac`

2. Launch the updated HTML from the repository:

   `python -m http.server 8000 --directory web`

   Open **http://localhost:8000/ble_debug_console.html** in Chrome or Edge,
   click **Connect** to JDY-33. The Web is receive-only; no GATT write
   requests exist. The STM32 automatically rebroadcasts the last diagnostic
   map about 12 seconds after each completed map transfer.
   Browser GATT access needs a secure context (localhost is supported).

3. Back up any important existing Flash data and check that ST-Link uploading
   does **not** mass-erase reserved sectors 6 and 7. Flash the opt-in firmware:

   `python -m platformio run -e flash_map_diag_96m -t upload`

   Confirm `FM_BOOT,profile=flash_map_diag_96m,locked=1,tim11=OFF`
   on PA9/PA10. This is not the ordinary motor-running firmware.

4. Run the **safe, no-write preview** (COM7 is an example only):

   `python tools/flash_map_pipeline.py --port COM7 --web-hold-seconds 6`

   This sends `INFO`, `PREVIEW 16`, `PREVIEW 5`, `READ`, `INFO`.
   Only the first two create maps, and they are RAM-only. Each preview
   is checked byte-for-byte and by CRC using the independent host generator.
   WebBLE should automatically switch 16×16 → 5×5 and display CRC PASS.

5. To append **one real test map** into sector 7 (explicitly consumes one
   previously erased 1 KiB slot; no auto-erase, no overwrite):

   `python tools/flash_map_pipeline.py --port COM7 --write-size 16 --allow-flash-write --backup-confirmed`

   Use `--write-size 5` to save a 5×5 map instead. The pipeline requires
   both explicit flags. Check `FM_SAVE,status=0`, `FM_VERIFY,status=PASS`,
   and a subsequent `FM_READ,status=0`. The firmware sends the read-back
   map to BLE with `src=FLASH`.

6. After MCU reset, the diagnostic firmware starts a fresh **RAM 16×16 test
   map** and rebroadcasts it periodically. This is not persistent Flash data.
   To display the actual persisted map, send `READ` over UART1/COM7;
   the last selected map then repeats automatically over BLE.

7. Export the BLE terminal log in the web page, then independently audit
   the exact MAPCFG/MAPROW/MAPEND frames:

   `python tools/flash_map_pipeline.py --ble-log "C:\path\to\nezumi_export.txt"`

   BLE report verification checks the received rows and CRC, not just
   a claimed UART success line.

Test artifacts go to `results/flash_map_diag/` as timestamped JSON and
UART .log files. Keep the web log export as complementary evidence.
The Python pipeline itself does not read the Web Bluetooth transport; the
browser verifies BLE frames and the optional `--ble-log` import audits them.

## Tests and safety limits

Offline tests:

- `python tests/regression/flash_map_protocol_test.py` — actual C map 5×5/16×16,
  CRC cross-check against Python zlib, and UART corrupted-row rejection.
- `node tests/web/test_ble_map_web.js` — web JS syntax, dynamic sizes,
  valid CRC render, bad CRC/missing-row rejection.
- `python tests/regression/flash_record_native_test.py` — Flash record CRC C regression.
- `python -m platformio run -e flash_map_diag_96m` — diagnostic firmware build.

The interactive test is not a bench-proven guarantee of JDY-33 throughput,
ST-Link sector preservation, or power-loss resilience. Record any real-board
failures with UART .log and exported BLE lines. Save commands are never sent
by default. The diagnostic does not implement navigation or map exploration.


## 2026-10-24 — RX-disabled BLE monitor (current mode)

### Hardware constraint

The current Micromouse may have an unreliable JDY-33-to-STM32 UART2 receive
path (FFE1 downlink). Prior short HELLO/PING diagnostics did succeed on
some sessions, so a permanent physical fault is **not conclusively proven**.
The optional Flash Map diagnostic now **does not depend on this path**.
`USART2->CR1` disables `RE` and `RXNEIE` in the Flash Map firmware only.
Other firmware profiles and physical pin mappings are unchanged.

| Path | Role | Direction |
|---|---|---|
| UART1 PA9/PA10 via USB-TTL COM7 | Debug commands and verification | PC ↔ STM32 |
| USART2 PA2 TX → JDY-33 RXD → Web notifications | Passive telemetry/map | STM32 → Web |
| USART2 PA3 RX ← JDY-33 TXD | Unused in this profile | Disabled |

### No BLE commands or Flash writes

No Web **Read saved map**, **Preview** or retry buttons are exposed.
Connecting to JDY-33 only subscribes to FFE1 notifications.
The firmware generates the deterministic **16×16 RAM test map at boot**,
broadcasts `MAPCFG`, 16 paced `MAPROW` records, and `MAPEND`, and
retransmits the last chosen map **12 seconds after each finished broadcast**
even if no Web browser is connected. With 120 ms per row, a full 16×16
broadcast lasts about 2 seconds; a later subscriber may need slightly
longer than 12 seconds to receive the next complete map.
`MAPREADY` heartbeat is sent every ~3.5 seconds and the browser
tracks MCU uptime resets without trying to send any requests.

To display a different map while Web is connected use COM7 UART1:

```powershell
python -m serial.tools.miniterm COM7 115200
```

Inside Miniterm (NOT PowerShell) enter `PREVIEW 5`, `PREVIEW 16` or
`READ`; `PREVIEW` uses RAM only, and `READ` reads saved Flash
(in the prior run, a legacy 5×5 map). Whichever map was most recently
selected is the one that will be rebroadcast periodically.
Exit Miniterm with `Ctrl+]`.

One-shot UART diagnostic (no Flash writes) remains available:

```powershell
python tools/flash_map_pipeline.py --port COM7 --web-hold-seconds 6
```

This changes the displayed diagnostic map through UART1, *not BLE RX*.
Run the browser before launching the script to observe all phases.

### Evidence and acceptance

A Web terminal screenshot showing only `Connected` is insufficient.
To pass the BLE uplink test require **all** `MAPCFG`, N `MAPROW`, and
`MAPEND` with `MAP VERIFY PASS` and CRC32 matching the UART output.
Use the browser **Export** button and `--ble-log` Python audit.

`FM_REPLAY`, `FM_CFG`, `FM_ROW`, `FM_DONE` printed on COM7
confirm firmware scheduling/USART1 mirror, but not radio delivery.
If Web sees `MAPREADY` with no complete map, check UART2 TX PA2 →
JDY-33 RXD, JDY module baudrate **115200**, GND, and BLE notify traffic.
Web must not silently retry GATT writes; receiver just awaits the
next autonomous broadcast.

Note: this isolated Flash Map profile keeps motors locked, and
**does not start TIM11/IR/MPU**. It cannot qualify production control
jitter or motor-on BLE throughput. Those are separate tests.
