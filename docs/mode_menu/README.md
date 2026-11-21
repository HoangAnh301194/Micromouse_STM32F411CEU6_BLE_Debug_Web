# Event-only 7-mode selector over Bluetooth Classic SPP

**Branch:** `giomuadongbacv2`  
**Firmware profile:** `mode_menu_96m` (default in `platformio.ini`)

This ports **menu selection only** from the old `main` branch. Choosing
a mode never starts calibration, exploration, A*, Speedrun, map deletion,
or a motor test. The choice is RAM-only and disappears on reboot.

## Build and flash

```powershell
git switch giomuadongbacv2
git pull --ff-only origin giomuadongbacv2
pio run -e mode_menu_96m
pio run -e mode_menu_96m -t upload
```

Do not flash `genericSTM32F411CE` if you want event-only menu logs:
that profile is the **legacy motion-capable runtime**, with continuous telemetry.
The menu profile locks motor outputs, drops incoming F/L/R/Z commands,
and uses SSD1306 on shared MPU I2C through `OLED_SharedService`.

## How to use

1. Open the **outgoing** JDY-33 COM port in the desktop app.
   The app sends exactly one safe `?` status probe at each COM connection.
   The STM32 replies **`SPP CONNECTED`** once for that probe.
   This is a verified round trip through the UART/SPP link, unlike
   an unsolicited boot-only READY message that can be missed by a PC.
2. The first physical **short press** on PA0 focuses **Mode 1: Calibration**.
   Subsequent short presses focus Modes 2 through 7, wrapping to 1.
3. Holding PA0 **at least 400 ms** confirms the focused mode and
   prints a single `Selected mode ...` line. There is no mode action.
4. Another short press browses the next mode; long press confirms it.
   A long press before the very first short press does nothing.

The OLED uses the same seven labels as the old `main` branch,
and displays `MODE SELECTED` / `NO ACTION YET` after confirmation.
Software debounce is 30 ms. A key held across reboot is ignored until
released.

## Exact event-only SPP output

These are examples of the output format, **not physical test logs**.

```text
SPP CONNECTED
Mode 1: Calibration
Mode 2: Run Slow
Mode 3: Run Faster
Selected mode 3: Run Faster
```

- **`SPP CONNECTED`**: one reply per desktop connection's `?` probe.
- **`Mode N: Name`**: one line per valid short press.
- **`Selected mode N: Name`**: one line per long press.
- Nothing is transmitted when the button is idle.
- `MENU,STATUS` every 2 seconds and `t=... state=...` every 100 ms
  have been **disabled in the menu profile**.
- The same periodic telemetry still exists in other explicitly selected
  build profiles, which are not modified by this menu setting.

| N | OLED name |
|---:|---|
| 1 | Calibration |
| 2 | Run Slow |
| 3 | Run Faster |
| 4 | A* Run |
| 5 | Speedrun |
| 6 | Reset Map |
| 7 | System Test |

The firmware **cannot infer SPP connection from UART initialization alone**:
USART2 is electrically active whether or not a Bluetooth PC is connected.
The one-byte request-and-reply handshake confirms actual host-to-device
and device-to-host delivery, without motor/Flash commands.

## Desktop app

```powershell
py tools/classic_spp_link_diag.py --list
py tools/classic_spp_dashboard.py --port COM16
```

Substitute the real **outgoing** COM port. Do not leave another SPP
terminal open on the same port. The dashboard transmits only a one-byte
status probe `?` on each connection; it never sends motor, Flash, or
mode-selection commands. Mode selection is strictly PA0.

## Tests

```powershell
py tests/regression/mode_menu_native_test.py
```

Requires a C compiler named `gcc` on PATH (or set environment variable
`CC`). Covers first short press, 7-item wraparound, long press,
debounce, boot-held button, long press before focus, and `millis()`
rollover.

The native test does not validate compiled STM32 firmware or the
physical JDY-33/SPI/I2C/PA0 hardware. Confirm the SPP handshake,
one-time events, OLED rendering, and motor lock on the real robot,
with wheels raised and an independent VM cutoff.

## Known behavior

The legacy diagnostic telemetry previously displayed blank `yaw=`,
`gz=`, `d=`, and `e=` due to a likely embedded floating-point
formatting configuration issue. Those diagnostic lines are suppressed
by this menu-only profile; the underlying sensor quality must be
evaluated separately in a suitable diagnostic firmware.
