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


## 2026-10-24 — WebBLE map replay and verification

The initial 2026-10-24 Flash Map report `results/flash_map_diag/flash_map_20261024_082035.json`
proved **UART1** 16x16 RAM preview CRC `933A97B3` (16 rows), RAM
5x5 preview CRC `1323C17D` (5 rows), and read-back of the earlier
**legacy 5x5** Flash map CRC `22646C99`. It did **not** include a BLE
Web export, so these UART-only PASS verdicts cannot prove on-air delivery.

**Important:** `PREVIEW 16` and `PREVIEW 5` live in **RAM** and are
transmitted only at command time. Resetting the STM32 does not persist RAM
previews. The `READ` / Web **Read saved map** action accesses the persisted
Flash map (here it is the legacy 5x5), not the last 16x16 RAM preview.
Neither web preview button performs a Flash write.

Run the **new** `flash_map_diag_96m` firmware via ST-Link and the matching
`web/ble_debug_console.html` in a refreshed Chrome/Edge tab:
```powershell
git pull --rebase origin giomuadongbac
python -m platformio run -e flash_map_diag_96m -t upload
python -m http.server 8000 --directory web
```

Open `http://localhost:8000/ble_debug_console.html` (hard refresh
Ctrl+F5), connect `Nezumi_Chan`, and click:

- **Read saved map**: Web sends `?`, MCU logs `FM_BLE_REQ,cmd=READ`
  on UART1 and answers BLE `MAPACK,cmd=READ` + `MAPCFG`/`MAPROW`/`MAPEND`.
  Here the expected result is 5×5 legacy CRC `22646C99`.
- **Preview 16×16 (RAM)**: Web sends `V`, MCU logs
  `FM_BLE_REQ,cmd=PREVIEW16`, then streams 16x16 RAM map with
  expected CRC `933A97B3`, without programming Flash.
- **Preview 5×5 (RAM)**: Web sends `v`, expected CRC `1323C17D`.
  These two commands work only in the guarded Flash Map diagnostic build.

The UART1 log `FM_BLE_REQ` distinguishes a failed FFE1 downlink
(no MCU receipt) from a failed uplink (MCU ACK exists on serial but no
BLE MAPACK). The Web terminal shows `MAP TX`, `MAPACK`, the number
of valid `MAPROW` rows and either `MAP VERIFY PASS` or a specific error.
If incomplete, it retries the requested map up to three attempts.
`MAPREADY,ms=...` arrives about every 3.5s, detects an STM32 uptime
reset and can request a map again over the **existing** BLE connection.
There is no unbounded flood when the remote device is absent.

Keep the Web connected **before** sending UART `PREVIEW`; otherwise
the RAM preview's one-time BLE output can be missed. The request buttons
allow deliberate replay after the original UART event.
If a browser or Bluetooth terminal already holds GATT, disconnect it before
using the Python Bleak benchmark. Do not operate concurrent GATT clients.

If Web still says `waiting for data`, export the **Web BLE TXT** and
capture UART1 `FM_BLE_REQ`/ `FM_CFG`/ `FM_ROW`/`FM_DONE` from the
same instant. UART-only Flash-map PASS must not be presented as WebBLE PASS.


## 2026-10-24 follow-up — Web connects but never receives MAPACK or MAPREADY

**Symptoms:** Web shows `Connected to Nezumi_Chan`, `MAP TX ...` and
three retries, but ZERO `MAPACK`, `MAPREADY`, `MAPCFG` or `MAPROW`.
This is **not a map rendering/CRC verdict**: there is no map frame to
render. FFE1 GATT being connected and accepting writes is NOT proof that
the JDY-33 UART and STM32 USART2 are exchanging bytes.

An independent USART1/COM7 observer is now compiled into
`flash_map_diag_96m`. Flash the updated image and start serial capture
**before** pressing the Web map buttons:

```powershell
git pull --rebase origin giomuadongbac
python -m platformio run -e flash_map_diag_96m -t upload
python -m serial.tools.miniterm COM7 115200
```

Miniterm must show:

```text
FM_BOOT,profile=flash_map_diag_96m,...
FM_BLE_DIAG,ms=...,baud=115200,accepted=...,txdr=...,q=...,drop=...,rx=...,ore=...,fe=...,ne=...,overflow=...,cmd=...,unknown=...,lasthex=...
```

`FM_BLE_DIAG` repeats every two seconds. In Chrome/Edge open the **new**
`web/ble_debug_console.html` (hard refresh Ctrl+F5), connect the module,
click **Preview 5×5 (RAM)** exactly once and check these counters before
and after the click:

| Observation on COM7 | Conclusion / next investigation |
|---|---|
| No `FM_BOOT` and no `FM_BLE_DIAG` | Incorrect firmware, COM port, upload/reset or CPU stopped; the standalone JDY-33 can advertise/connect without STM32. |
| `FM_BLE_DIAG` `txdr` keeps increasing but Web has **zero** GATT notifications, even for `MAPREADY` | STM32 writes USART2 DR, but forward delivery to Web is unverified; check **PA2 TX → JDY RXD**, shared GND, UART baud 115200 on **both sides**, module transparent-UART configuration and PC BLE notifications. |
| `FM_BLE_DIAG` `rx` does not increase on a Web click | The outgoing GATT write did not reach **PA3 RX ← JDY TXD**; check GATT write permissions/mode, JDY UART wiring/configuration and RXNE interrupt. |
| `rx` increases, `cmd` increases and `FM_BLE_REQ,cmd=PREVIEW5` appears | Web → JDY → MCU command path **works**. Look for `MAPACK` and `MAPROW` notifications and validate BLE transmit direction/byte integrity. |
| `rx` increases but `unknown`, `fe`, `ore` or `ne` increases | Bytes reached USART2 but are invalid or corrupted; compare `lasthex` and error counters against the expected single-byte command `76` (ASCII `v`). |

`txdr` counts writes to USART2 TX data register, **not radio ACKs**.
`rx` counts accepted UART bytes at PA3, not GATT transaction success.
A clean map still requires the full `MAPCFG` + N `MAPROW` +
`MAPEND` CRC verification on Web.

Capture the COM7 `FM_BOOT` and two `FM_BLE_DIAG` lines before/after
the single Web button press, plus the exported Web BLE TXT. Do not run
`ble_full_pipeline.py`, a Bluetooth phone client or any other GATT
connection in parallel. This check runs with motors permanently locked;
neither Web preview command writes to Flash.
