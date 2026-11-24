#!/usr/bin/env python3
"""Two-way JDY-33/USART2 isolation test. Requires spp_smoke_96m firmware.

Test matrix:
- periodic SPP_SMOKE,HB without any host command => MCU TX -> JDY -> PC
- a matching SPP_SMOKE,PONG after host '?' => host -> JDY -> MCU RX
The test sends NO motor, sensor reset, menu, Flash or AT commands.

Usage:
    py tools/classic_spp_smoke_probe.py --port COM16 --seconds 20
"""
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import time


def probe(port: str, seconds: float, out: Path | None = None) -> int:
    try:
        import serial
    except ImportError:
        print("Missing pyserial: py -m pip install pyserial")
        return 4

    start = time.monotonic()
    total_bytes = 0
    hb = 0
    pong = 0
    probes = 0
    lines = 0
    next_probe = start
    framer = bytearray()
    log = None
    try:
        if out:
            out.parent.mkdir(parents=True, exist_ok=True)
            log = out.open("w", encoding="utf-8", buffering=1)
        with serial.Serial(port, 115200, timeout=0.15, write_timeout=0.5) as link:
            print(f"OPEN {port} 115200 (TX '?' only, no motor commands)")
            print("Expect MCU PC13 LED to toggle every second, and SPP_SMOKE,HB frames.")
            while time.monotonic() - start < seconds:
                now = time.monotonic()
                if now >= next_probe:
                    link.write(b"?")
                    link.flush()
                    probes += 1
                    next_probe = now + 1.0
                chunk = link.read(link.in_waiting or 1)
                if not chunk:
                    continue
                total_bytes += len(chunk)
                framer.extend(chunk)
                if len(framer) > 2048:
                    print("WARNING: oversized frame buffer; discarding partial data")
                    framer.clear()
                while b"\n" in framer:
                    raw, _, remaining = framer.partition(b"\n")
                    framer = bytearray(remaining)
                    line = raw.rstrip(b"\r").decode("utf-8", "replace")
                    if not line:
                        continue
                    lines += 1
                    stamp = datetime.now().isoformat(timespec="milliseconds")
                    print(f"{stamp} {line}", flush=True)
                    if log:
                        log.write(f"{stamp} {line}\n")
                    if line.startswith("SPP_SMOKE,HB,"):
                        hb += 1
                    elif line.startswith("SPP_SMOKE,PONG,"):
                        pong += 1
    except (OSError, Exception) as err:
        # Keep a readable diagnosis for Windows RFCOMM, read/write and COM faults.
        print(f"SERIAL ERROR: {type(err).__name__}: {err}")
        return 4
    finally:
        if log:
            log.close()

    print(f"RESULT: probes={probes} RX_bytes={total_bytes} "
          f"lines={lines} heartbeats={hb} pongs={pong}")
    if hb and pong:
        print("PASS: TX heartbeat and two-way round-trip work with sensors isolated.")
        print("If mode_menu_96m still has RX 0, investigate full-firmware startup.")
        return 0
    if hb:
        print("PARTIAL: MCU->PC succeeds; host '?' not decoded by STM32.")
        print("Check USART2 PA3 RX, JDY TXD, grounds, UART framing errors.")
        return 2
    if total_bytes:
        print("PARTIAL: some bytes arrived, but no SPP_SMOKE heartbeat.")
        print("Inspect raw bytes/baud and verify spp_smoke_96m was flashed.")
        return 2
    print("NO BYTES: check PC13 LED blink. If blinking, suspect STM32 PA2 TX,")
    print("JDY RXD, common GND, JDY baud, COM Outgoing or concurrent BLE link.")
    print("If PC13 does not blink, verify MCU boot/clock and ST-Link flash.")
    return 3


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Windows Bluetooth SPP OUTGOING COM")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    if args.seconds < 3:
        parser.error("--seconds must be at least 3")
    return probe(args.port, args.seconds, args.log)


if __name__ == "__main__":
    raise SystemExit(main())
