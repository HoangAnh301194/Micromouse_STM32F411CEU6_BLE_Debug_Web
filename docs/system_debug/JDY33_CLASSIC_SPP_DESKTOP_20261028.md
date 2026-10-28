# JDY-33 Bluetooth Classic SPP desktop console (2026-10-28)

## Scope

This is a host-side-only Python dashboard. It **does not** add a second Bluetooth module, rewrite firmware, or reassign STM32 pins.

- STM32F411CEU6 USART2 TX PA2 -> JDY-33 RXD; RX PA3 <- JDY-33 TXD.
- Keep the JDY-33, its 115200 8N1 UART interface, and the current Web BLE console unchanged.
- Desktop transport is **Bluetooth Classic SPP** via a Windows virtual COM port, not Web Bluetooth GATT.
- This version is deliberately **receive-only**: no motion, Flash-write, or PID-tuning commands; avoids reliance on the previously unreliable JDY-33 downlink.
- JDY-33 dual-mode support does not prove *simultaneous* BLE and Classic connections. Test them separately first.

## Windows setup and COM37

1. Power the current JDY-33 on the robot. Disconnect Web BLE during initial SPP tests.
2. Windows Settings -> Bluetooth & devices -> Add device -> Bluetooth. Pair its **Classic/SPP** endpoint. The device's Classic name may differ from the BLE name.
3. Identify the **outgoing Bluetooth Serial Port (SPP)** in Windows Bluetooth COM settings or Device Manager -> Ports (COM & LPT). Use the **outgoing** port, not the incoming server port.
4. **COM37 is only a preferred high-numbered port**, not something Python can make Windows assign. To set it: Device Manager -> Bluetooth Serial Port (COMx) -> Properties -> Port Settings -> Advanced -> COM Port Number -> COM37, but **only if unused**. If occupied, select another free port (e.g. COM47), or retain the assigned COM. Do not change the USB-TTL COM7. Reconnect after reassignment.
5. Install Python 3.10+ (standard Windows Python includes Tkinter) and pyserial:

   PowerShell:

       python -m pip install pyserial
       python tools/classic_spp_dashboard.py --port COM37

6. Click Refresh; choose the **actual Windows outgoing SPP COM**, which may not be COM37; click Connect. The dashboard never auto-connects to an arbitrary port. Serial baud is 115200. Windows virtual SPP COM drivers may ignore baud settings, but the physical STM32 UART2 remains 115200.

Keep the COM port exclusive: do not open the same port in a second Python process or terminal. USB-TTL on USART1/COM7 is a different physical UART.

## Features

- **Terminal:** timestamped RX, text filter, bounded GUI history, optional complete log file.
- **Maze viewer:** CRC32-validated MAPCFG/MAPROW/MAPEND 2..16 square maps; legacy MAZE: format, MZ0..MZ3 wall chunks and CELL: updates. Y=0 is the bottom row, NESW wall bits are 1,2,4,8. A CRC error cannot overwrite the previous accepted map. Wall nibble != visited cell.
- **Telemetry & PID:** extracts key=value numeric fields; parses nine-value W, wall PID traces; displays latest numeric values and up to two selected time series on a shared axis. Unit-bearing field names are retained.
- **Replay log:** reads line-oriented logs saved by this app or exported elsewhere. No radio needed.
- The host tool never sends data to the robot, arms motors, or updates STM32 Flash.

## Validation

No hardware needed:

    python tests/regression/classic_spp_protocol_test.py

Physical sign-off requires: (1) outgoing SPP COM verified; (2) receive real UART2 data; (3) full map pass with matching CRC32; (4) motor-locked logging; (5) compare BLE and SPP throughput, latency, gaps, and drop counts under the same firmware/payload. These are **not** claimed PASS without physical tests.

STM32 USART2 at 115200 baud 8N1 limits payload throughput to approximately 11.52 kB/s *before* radio and frame overhead. Avoid streaming full 1-kHz raw control telemetry continuously when it could affect CPU or UART queues; prefer buffered capture and later replay.

## Troubleshooting

- **COM37 not listed:** Windows did not assign it. Select the outgoing SPP COM actually shown. Only manually reassign to COM37 when it is free.
- **Port busy/access denied:** close other programs holding the port.
- **Bluetooth paired but no SPP COM:** possible BLE-only pairing, no Classic SPP service advertisement, or incompatible settings/firmware. Pair the Classic endpoint; do not repurpose USART1.
- **Connected but no received lines:** verify JDY-33 Classic link, STM32 USART2 TX PA2, and that the flashed mode actually transmits. COM open is not proof of data delivery.
- **Map CRC failure:** missing/corrupt fragments or an incomplete stream; await a fresh MAPCFG sequence.
- **BLE disconnects while SPP runs:** confirm whether this particular JDY-33 firmware supports simultaneous links; until tested, treat these as alternate connections.

## Files

- tools/classic_spp_protocol.py: testable independent protocol parser
- tools/classic_spp_dashboard.py: Tkinter GUI and background serial receiver
- tests/regression/classic_spp_protocol_test.py: offline regression tests
- web/ble_debug_console.html: unchanged

No embedded firmware, PlatformIO flags, motor logic, or Web BLE files changed for this feature.
