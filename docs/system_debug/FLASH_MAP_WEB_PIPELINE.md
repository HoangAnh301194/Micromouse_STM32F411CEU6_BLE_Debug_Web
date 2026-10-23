# Flash map WebBLE + UART1 diagnostic pipeline

> **Restored on `giomuadongbac`:** `tools/flash_map_pipeline.py` is again
> available for the user-requested 5×5/16×16 Flash-map test. The BLE viewer
> remains in `web/ble_debug_console.html` and the native test is under
> `tests/regression/flash_map_protocol_test.py`. Other retired host debug tools
> remain in the [archive branch](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/tree/archive/debug-tools-precleanup-20261024).
> Board-side Flash write / power-loss / JDY-33 acceptance is still pending.


**Branch:** `giomuadongbac`. **Firmware profile:** `flash_map_diag_96m`.

The experimental profile locks motors permanently until reset, does not start
TIM11, IR or MPU6050, and never erases Flash. It only writes one map journal slot
after an explicit `SAVE N CONFIRM` UART command. No physical-board acceptance
has been completed from this chat.

## Hardware and protocol

- Debug/results: **USART1 TX PA9 / RX PA10**, USB-UART, 115200 8N1. Emits `FM_BOOT`, `FM_INFO`, `FM_BEGIN`, `FM_SAVE`, `FM_VERIFY`, `FM_READ`, `FM_CFG`, `FM_ROW`, `FM_DONE`.
- Map display: **JDY-33 BLE USART2 TX PA2 / RX PA3**, Web Bluetooth service FFE0, characteristic FFE1. Sends `MAPCFG`, one `MAPROW` per row, then `MAPEND`; web verifies complete rows and CRC32 before rendering.
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
   click **Connect** to JDY-33. The **Request map** button sends a one-byte
   `?` request; if FFE1 is not writable, use UART command `READ` instead.
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

6. After an MCU reset (the map must persist), rerun the safe preview,
   click **Request map** or send `READ` over UART1 to retransmit the
   stored 5×5/16×16 map.

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
