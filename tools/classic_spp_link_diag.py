#!/usr/bin/env python3
"""Non-invasive Bluetooth Classic SPP disconnect diagnostic for JDY-33.

Usage:
    python tools/classic_spp_link_diag.py --port COM16 --seconds 30

Uses the existing JDY-33 / STM32 USART2 PA2/PA3 hardware. Does not transmit
AT commands, motor commands or modify device settings. Opening the outgoing
SPP COM is itself what requests a Classic RFCOMM session from Windows.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import sys
import time
import traceback


def iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


class Recorder:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = path.open("w", encoding="utf-8", buffering=1)
        self.path = path
        self.t0 = time.monotonic()

    def emit(self, event: str, **fields):
        record = {"time": iso_now(), "elapsed_s": round(time.monotonic() - self.t0, 3),
                  "event": event, **fields}
        self.handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        details = " ".join("%s=%r" % (key, val) for key, val in fields.items())
        print("[%8.3f] %-16s %s" % (record["elapsed_s"], event, details), flush=True)

    def close(self):
        self.handle.close()


def find_port(port_name: str):
    from serial.tools import list_ports
    ports = sorted(list_ports.comports())
    selected = None
    for p in ports:
        if p.device.lower() == port_name.lower():
            selected = p
    return selected, [
        {"port": p.device, "description": p.description, "hwid": p.hwid}
        for p in ports
    ]


def run(port_name: str, baud: int, seconds: float, sample_ms: int,
        recorder: Recorder, rts: bool, dtr: bool):
    import serial
    chosen, inventory = find_port(port_name)
    recorder.emit("inventory", ports=inventory, selected=port_name,
                  selected_info=vars(chosen) if chosen is not None else None)
    if chosen is None:
        recorder.emit("not_listed", explanation="Outgoing SPP port not listed by Windows; verify COM Ports tab")
        return 2
    hwid = (chosen.hwid or "").upper()
    if "BTHENUM" not in hwid:
        recorder.emit("warning", message="Selected COM is not identified as a Bluetooth serial port by Windows")
    recorder.emit("opening", port=port_name, baud=baud, dtr=dtr, rts=rts)
    start = time.monotonic()
    rx_bytes = 0
    chunks = 0
    last_bytes_at = None
    raw_preview_limit = 256
    preview = bytearray()
    try:
        # Do not drive modem control signals before the port opens by accident.
        link = serial.Serial(port=None, baudrate=baud, timeout=sample_ms / 1000.0,
                             write_timeout=1)
        link.dtr = dtr
        link.rts = rts
        link.port = port_name
        link.open()
        try:
            recorder.emit("com_open", open_ms=round((time.monotonic() - start) * 1000),
                          note="COM open does not prove radio data arrives")
            next_progress = time.monotonic() + 2.0
            while time.monotonic() - start < seconds:
                # read() is the critical operation that typically reveals a failed
                # RFCOMM link as SerialException/Windows ERROR_DEVICE_NOT_CONNECTED.
                chunk = link.read(link.in_waiting or 1)
                if chunk:
                    rx_bytes += len(chunk)
                    chunks += 1
                    last_bytes_at = time.monotonic()
                    if len(preview) < raw_preview_limit:
                        preview.extend(chunk[:raw_preview_limit - len(preview)])
                    recorder.emit("rx", count=len(chunk), total=rx_bytes,
                                  hex=chunk[:48].hex(" "),
                                  utf8=chunk[:96].decode("utf-8", "replace"))
                if time.monotonic() >= next_progress:
                    recorder.emit("alive", rx_bytes=rx_bytes, chunks=chunks,
                                  silent_s=(None if last_bytes_at is None else
                                            round(time.monotonic() - last_bytes_at, 2)),
                                  com_open=link.is_open)
                    next_progress = time.monotonic() + 2.0
            recorder.emit("completed", status="no_serial_error",
                          total_bytes=rx_bytes, chunks=chunks,
                          first_256_bytes_hex=preview.hex(" "))
            return 0
        finally:
            link.close()
    except (serial.SerialException, OSError) as exc:
        recorder.emit("serial_error", error_type=type(exc).__name__, error=str(exc),
                      after_open_s=round(time.monotonic() - start, 3),
                      rx_bytes=rx_bytes, chunks=chunks,
                      traceback="".join(traceback.format_exception_only(type(exc), exc)).strip())
        return 1
    finally:
        _, after = find_port(port_name)
        recorder.emit("inventory_after", ports=after)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Windows OUTGOING Bluetooth SPP COM, e.g. COM16")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--sample-ms", type=int, default=200)
    parser.add_argument("--dtr", action="store_true", help="Test DTR enabled (default disabled)")
    parser.add_argument("--rts", action="store_true", help="Test RTS enabled (default disabled)")
    parser.add_argument("--log", type=Path, help="JSON Lines diagnostic log destination")
    args = parser.parse_args()
    if args.seconds <= 0 or args.sample_ms < 50 or args.baud <= 0:
        parser.error("seconds, sample-ms and baud must be positive (sample-ms >= 50)")
    try:
        import serial
    except ImportError:
        print("Install pyserial first: python -m pip install pyserial", file=sys.stderr)
        return 2
    dest = args.log or (
        Path("results") / "classic_spp" /
        ("spp_link_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".jsonl")
    )
    recorder = Recorder(dest)
    try:
        recorder.emit("start", python=sys.version.split()[0], windows=platform.platform(),
                      pyserial=getattr(serial, "__version__", "unknown"),
                      port=args.port, baud=args.baud,
                      seconds=args.seconds, dtr=args.dtr, rts=args.rts)
        result = run(args.port, args.baud, args.seconds, args.sample_ms, recorder,
                     rts=args.rts, dtr=args.dtr)
    except KeyboardInterrupt:
        recorder.emit("interrupted", note="Ctrl+C")
        result = 130
    finally:
        recorder.close()
    print("DIAGNOSTIC LOG:", dest)
    return result


if __name__ == "__main__":
    sys.exit(main())
