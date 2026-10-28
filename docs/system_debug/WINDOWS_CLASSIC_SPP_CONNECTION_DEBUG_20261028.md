# Diagnose Windows Bluetooth Classic pairing / RFCOMM disconnect (JDY-33)

Date: 2026-10-28. Branch: giomuadongbac. Hardware is still the same
JDY-33 on STM32F411 USART2 PA2/PA3; no second Bluetooth module.

## Distinguish three different states

1. **Paired in Windows Settings, status "Not connected"**: normal when
   no program has opened the outgoing SPP COM. Pairing is a stored trust
   relationship, not an active serial data session.
2. **Outgoing SPP COM opens**: Windows accepted the serial handle; it
   does not prove the JDY-33 has supplied any serial data.
3. **SPP RX confirmed**: at least one received byte; strongest initial
   end-to-end proof from STM32 -> JDY-33 -> Windows -> program.
   A later Win32/pyserial error indicates the serial session failed.

Windows can create several generic "Standard Serial over Bluetooth" COM
ports. Use Bluetooth settings -> More Bluetooth settings -> **COM Ports**
and pick the **Outgoing** COM for Nezumi_Chan_SPP. Do not guess based
on COM number, nor try the incoming listener port.

## Step 1 - Live Windows-side Event Viewer + PnP monitor

Run in **Administrator PowerShell** from the repository root:

    git switch giomuadongbac
    git pull --ff-only origin giomuadongbac
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\windows_bluetooth_link_debug.ps1 -DurationSeconds 90

The monitor enumerates installed/active Bluetooth event channels
dynamically, queries the Windows System and Kernel-PnP channels when
available, and logs PnP changes, discovered COM mappings, provider names
and event IDs. Output is both real-time console and a timestamped JSONL
file under results\classic_spp\windows\.

This monitor is PASSIVE: it does not open a COM port or force a Bluetooth
connection. A PnP Status=OK is device-driver state, **not link state**.
Some Windows builds emit little/no user-visible pairing telemetry in
Event Viewer; a blank monitor is not proof of success.

## Step 2 - Open the actual outgoing SPP COM in a second PowerShell

First find ports:

    python -m serial.tools.list_ports -v

Then connect, replacing COM16 with the actual Outgoing SPP COM:

    python tools/classic_spp_link_diag.py --port COM16 --seconds 40

Do this while the Windows monitor in Step 1 is active, and compare
timestamps. This second command logs: COM opened, elapsed time,
bytes/chunks received, the first bytes (hex + UTF-8), any SerialException
or Windows error, and the COM list before and after disconnection.
Its JSONL logs are under results\classic_spp\.

Keep classic_spp_dashboard.py, miniterm and other COM readers CLOSED
during this test. No commands or AT queries are transmitted.

## Interpret results

| Result | Meaning / next diagnostic |
|---|---|
| Windows Settings "Not connected", SPP COM never opened | Normal idle pairing possible; start Step 2 |
| com_open + rx events | Actual SPP serial receive confirmed |
| com_open + alive with rx_bytes=0 | Port opened; cannot confirm radio data. Verify firmware emits USART2 TX, and Windows selected Outgoing COM |
| serial_error with "Access is denied" | Another program may own the COM; identify exclusive access / permission / driver |
| serial_error with GetOverlappedResult / device disconnected | Windows serial link failed; correlate Windows events and deeper ETW |
| PnP events but no serial RX | Device enumeration is not proof of active RFCOMM |
| No Bluetooth event logs | Expected on some builds. Use the Bluetooth stack ETW profile below |

## Step 3 - Deep Microsoft Bluetooth stack trace (ETW)

This is the official Microsoft Bluetooth diagnostics method documented at:
https://github.com/microsoft/busiotools/blob/master/bluetooth/tracing/readme.md

Run in **Administrator PowerShell** at the repository root:

    New-Item -ItemType Directory -Force .\results\classic_spp\windows | Out-Null
    Invoke-WebRequest 'https://github.com/Microsoft/busiotools/raw/master/bluetooth/tracing/BluetoothStack.wprp' -OutFile .\BluetoothStack.wprp
    wpr.exe -start BluetoothStack.wprp!BluetoothStack -filemode

Next, reproduce the problem: pair/reconnect JDY-33 and open its Outgoing
SPP COM with the Python diagnostic. Immediately stop capture:

    wpr.exe -stop .\results\classic_spp\windows\BthTracing.etl

Microsoft's WPR Bluetooth stack profile captures deeper ETW events that
ordinary Windows Event Viewer may not expose. The resulting ETL needs an
ETW/WPA-capable analyzer, and not every event can be interpreted as
plain text. This procedure collects logs only; it does not fix pairing.
Do not leave high-volume tracing enabled for extended periods.

**Privacy:** JSONL and ETL traces can contain local device names,
hardware IDs, Bluetooth addresses and other system identifiers.
Inspect/redact before sharing publicly. Do not upload the full trace
to a public repository.

## Module-side precautions

During these host-side tests, keep JDY-33's existing UART wiring,
115200 baud, AT+TYPE0, Classic name Nezumi_Chan_SPP and BLE name
Nezumi_Chan unchanged. Do not send AT commands while streaming SPP.
Do not assume Web BLE and SPP can stay connected simultaneously; first
test with Web BLE closed. No robot motor or Flash commands are needed.
