# PA0 7-mode selection menu over Bluetooth Classic SPP

Branch: `giomuadongbacv2`. This feature ports **selection only** from `main`.
No calibration, exploration, A*, Speedrun, map erase, or system-test action
is dispatched. A selection is RAM-only until reboot.

## Firmware

The default PlatformIO environment is `mode_menu_96m`:

```powershell
git switch giomuadongbacv2
git pull --ff-only origin giomuadongbacv2
pio run -e mode_menu_96m
pio run -e mode_menu_96m -t upload
```

This profile enables `MODE_MENU_ONLY` and the shared-I2C OLED renderer.
It uses PA0 for input, SSD1306 on the existing MPU I2C bus, and JDY-33
USART2 PA2/PA3 at 115200 baud. PWM outputs are **latched locked** at boot.
Incoming Bluetooth F/L/R/Z motion commands are discarded; STOP over UART1
remains harmless. Other profiles/diagnostics are not changed by the menu
logic, and can still be built explicitly with `-e <environment>`.

## OLED and button behavior

- At power-on, the cursor points to `1.Calibration`.
- **Short press:** advances focus by one; after `7.System Test`, wraps
  back to `1.Calibration`.
- **Hold for at least 400 ms:** confirms focused mode. OLED displays
  `MODE SELECTED`, the label, and `NO ACTION YET`.
- After confirmation, a short press advances to the next item and returns
  to the `SELECT MODE` list; further long presses can select again.
- Software debounce: 30 ms; a key held while booting is ignored until
  released. Menu polling is main-context only.
- If OLED is absent, selection still logs over SPP; OLED rendering uses
  existing `OLED_SharedService` and can be deferred until gyro
  calibration finishes to protect the MPU I2C acquisition.

The seven labels match `main` exactly:

| ID | Code | OLED label |
|---:|---|---|
| 1 | CALIBRATION | 1.Calibration |
| 2 | RUN_SLOW | 2.Run Slow |
| 3 | RUN_FASTER | 3.Run Faster |
| 4 | ASTAR_RUN | 4.A* Run |
| 5 | SPEEDRUN | 5.Speedrun |
| 6 | RESET_MAP | 6.Reset Map |
| 7 | SYSTEM_TEST | 7.System Test |

## What SPP receives

The payload is plain ASCII with CRLF, over the existing JDY-33 channel.
Examples below illustrate the **format**, not measured hardware values.

```text
MENU,READY,count=7,short=NEXT,long=SELECT,action=NONE,locked=1,oled=1
MENU,FOCUS,id=1,code=CALIBRATION,label=1.Calibration
MENU,FOCUS,id=2,code=RUN_SLOW,label=2.Run Slow
MENU,SELECT,id=2,code=RUN_SLOW,label=2.Run Slow,action=NONE,locked=1
MENU,STATUS,focus=2,selected=2,code=RUN_SLOW,locked=1
```

`FOCUS` is emitted on each short press, and `SELECT` on each long press.
`STATUS` repeats every 2 seconds so a PC connecting late can recover
the selected mode. A zero `selected` means that no mode has been confirmed.
`action=NONE` means **no function ran** and no Flash was modified.
The existing `t=... state=... encL=...` telemetry continues at 10 Hz.

Use the **outgoing** Windows Bluetooth COM port (not incoming):

```powershell
py tools/classic_spp_link_diag.py --list
py tools/classic_spp_dashboard.py --port COM8
```

Replace `COM8` with the real outgoing port. The desktop dashboard is
receive-only; pressing the robot's physical PA0 button is required.

## Development checks

```powershell
py tests/regression/mode_menu_native_test.py
```

This native test requires `gcc` on PATH; it compiles the isolated menu
state machine and tests debounce, short/long press, wraparound, power-on
held button and the `millis()` rollover. It does **not** validate the
STM32 image. Build and test the real board to confirm OLED, MPU scheduling,
button electrical behavior and Bluetooth radio delivery.

## Scope and safety

- This change only adds selection UI/protocol and repairs a pre-existing
  conflict-corrupted `src/main.c`, restoring its 39-line main dispatcher.
- It does not copy `main`'s control algorithms; mode execution is deliberately
  deferred.
- Do not attach motor power solely based on code review. Verify
  `MENU,READY ... locked=1` and test motor lock with wheels raised
  before connecting the robot to a maze.
