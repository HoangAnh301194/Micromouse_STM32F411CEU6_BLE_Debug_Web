#!/usr/bin/env python3
"""USART1 PA9/PA10 flash-map diagnostic; WebBluetooth on PA2/PA3 runs separately.

Default is READ-ONLY/PREVIEW ONLY. Flash programming is never automatic.
Use --write-size N --allow-flash-write --backup-confirmed to opt in once.
The browser connects through JDY-33 and independently checks MAPCFG/ROW/END CRC.
"""
from __future__ import annotations
import argparse
import datetime as dt
import json
from pathlib import Path
import re
import subprocess
import sys
import time
import zlib

ROOT = Path(__file__).resolve().parent.parent
ENV = "flash_map_diag_96m"
HEADER = b"MAP1"
WALL_MASK = 0xF


def make_map(n: int) -> bytes:
    assert n in (5, 16)
    b = bytearray(b"MAP1" + bytes((n, n, 0, 0)))
    for y in range(n):
        for x in range(n):
            w = (
                (1 if y == n-1 or (3*x + 5*(y+1)) % 11 == 0 else 0)
                | (2 if x == n-1 or (7*(x+1) + 2*y) % 13 == 0 else 0)
                | (4 if y == 0 or (3*x + 5*y) % 11 == 0 else 0)
                | (8 if x == 0 or (7*x + 2*y) % 13 == 0 else 0)
            )
            b.append(w)
    return bytes(b)


def crc32(payload: bytes) -> int:
    return zlib.crc32(payload) & 0xFFFFFFFF


def fields(line: str) -> dict[str, str]:
    return {k: v for f in line.split(",")[1:] if "=" in f
            for k, v in (f.split("=", 1),)}


def verify_uart(lines: list[str], n: int, source: str) -> dict:
    cfg = next((fields(x) for x in lines if x.startswith("FM_CFG,")), None)
    assert cfg, "Missing FM_CFG"
    assert int(cfg["w"]) == n and int(cfg["h"]) == n
    assert cfg["src"] == source, (cfg["src"], source)
    expected = make_map(n)
    expected_crc = crc32(expected)
    assert int(cfg["crc"], 16) == expected_crc
    rows = {}
    for line in lines:
        if line.startswith("FM_ROW,"):
            d = fields(line)
            y = int(d["y"])
            assert y not in rows, "Duplicate FM_ROW"
            rows[y] = d["hex"]
    assert len(rows) == n, "UART row count wrong: "+str(len(rows))
    for y in range(n):
        got = rows[y]
        assert got == "".join("%X" % b for b in expected[8+y*n:8+(y+1)*n])
    done = next((fields(x) for x in lines if x.startswith("FM_DONE,")), None)
    assert done and int(done["rows"]) == n
    assert int(done["crc"], 16) == expected_crc
    return dict(size=n, source=source, bytes=len(expected),
                crc32="%08X" % expected_crc, uart_rows=n, status="PASS")


def verify_web_export(path: Path) -> dict:
    """Audit exact BLE frames exported from web, not just a PASS status line."""
    raw = path.read_text(encoding="utf-8")
    sessions = []
    active = None
    for text in raw.splitlines():
        match = re.search(r"(MAPCFG,.*|MAPROW,.*|MAPEND,.*)", text)
        if not match:
            continue
        line = match.group(1).strip()
        if line.startswith("MAPCFG,"):
            d = fields(line)
            n = int(d["w"])
            assert n in (5, 16) and int(d["h"]) == n
            active = dict(size=n, crc=d["crc"], src=d["src"], rows={})
        elif line.startswith("MAPROW,") and active is not None:
            d = fields(line)
            y = int(d["y"])
            assert 0 <= y < active["size"]
            assert re.fullmatch("[0-9A-Fa-f]{%d}" % active["size"], d["hex"])
            active["rows"][y] = d["hex"].upper()
        elif line.startswith("MAPEND,") and active is not None:
            d = fields(line)
            n = active["size"]
            assert len(active["rows"]) == n and int(d["rows"]) == n
            payload = (HEADER + bytes((n, n, 0, 0)) +
                       bytes(int(ch, 16) for y in range(n)
                             for ch in active["rows"][y]))
            actual_crc = "%08X" % crc32(payload)
            assert actual_crc == active["crc"].upper() == d["crc"].upper()
            expected = make_map(n)
            match_fake = payload == expected
            sessions.append(dict(size=n, src=active["src"], crc32=actual_crc,
                                 rows=n, fake_pattern_matches=match_fake, status="PASS"))
            active = None
    assert sessions, "No complete MAPCFG/ROW/END BLE capture found"
    return dict(ble_capture=str(path), frames=sessions, status="PASS")


def upload_diagnostic():
    command = [sys.executable, "-m", "platformio", "run", "-e", ENV, "-t", "upload"]
    print("Running ST-Link upload; verify programmer does not mass erase sector 6/7.")
    subprocess.run(command, cwd=ROOT, check=True)


def capture(ser, command: str, terminal: str, timeout: float,
            all_lines: list[str]) -> list[str]:
    print("UART TX:", command)
    ser.reset_input_buffer()
    ser.write((command + "\r\n").encode("ascii"))
    ser.flush()
    lines = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = ser.readline()
        if not data:
            continue
        line = data.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        print(line)
        lines.append(line)
        all_lines.append(line)
        if line.startswith(terminal):
            return lines
    raise TimeoutError("No " + terminal + " response for " + command)


def run_serial(args) -> dict:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial: python -m pip install pyserial") from exc

    all_lines = []
    report: dict = dict(port=args.port, mode=ENV, date=dt.datetime.now().isoformat(),
                        writes_requested=args.write_size, phases=[])
    with serial.Serial(args.port, 115200, timeout=0.15, write_timeout=3) as ser:
        time.sleep(0.4)
        before = capture(ser, "INFO", "FM_INFO,", 6, all_lines)
        assert "locked=1" in before[-1] and "ctrl=OFF" in before[-1], (
            "Flash profile must have motor outputs permanently locked")
        # Preview in RAM, which does not modify flash.
        for n in (16, 5):
            lines = capture(ser, "PREVIEW "+str(n), "FM_DONE,", 12, all_lines)
            assert any(x.startswith("FM_PREVIEW,size=%d,flash_written=0" % n) for x in lines)
            report["phases"].append(verify_uart(lines, n, "RAM"))

        if args.write_size:
            n = args.write_size
            print("EXPLICIT FLASH APPEND AUTHORIZED: consumes one erased slot (no erase).")
            lines = capture(ser, "SAVE %d CONFIRM" % n, "FM_DONE,", 15, all_lines)
            assert "status=0" in next(x for x in lines if x.startswith("FM_SAVE,"))
            assert "status=PASS" in next(x for x in lines if x.startswith("FM_VERIFY,"))
            report["phases"].append(verify_uart(lines, n, "FLASH"))
            lines = capture(ser, "READ", "FM_DONE,", 12, all_lines)
            assert "status=0" in next(x for x in lines if x.startswith("FM_READ,"))
            report["phases"].append(verify_uart(lines, n, "FLASH"))
        else:
            try:
                lines = capture(ser, "READ", "FM_DONE,", 12, all_lines)
                # Existing Flash may have legacy or unrelated map; report raw only.
                report["existing_flash"] = "READ completed; inspect FM_CFG/ROW data, not a known fake pattern"
            except TimeoutError:
                # On blank board READ returns FM_READ NOT_FOUND and no FM_DONE.
                report["existing_flash"] = "no complete map stream (may be blank)"
        capture(ser, "INFO", "FM_INFO,", 6, all_lines)
    report["uart_log"] = all_lines
    report["status"] = "PASS"
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", help="USART1 PA9/PA10 USB-UART e.g. COM7")
    p.add_argument("--flash", action="store_true",
                   help="Upload guarded firmware via ST-Link before serial checks")
    p.add_argument("--allow-upload", action="store_true",
                   help="Acknowledge flashing can affect existing Flash data")
    p.add_argument("--write-size", type=int, choices=(5,16),
                   help="Append exactly one 5x5 or 16x16 map record to Flash")
    p.add_argument("--allow-flash-write", action="store_true",
                   help="Explicitly permit a single on-board Flash program operation")
    p.add_argument("--backup-confirmed", action="store_true",
                   help="Confirm important prior calibration/map contents were backed up")
    p.add_argument("--ble-log", type=Path, help="Audit a separately exported web BLE log")
    p.add_argument("--output", type=Path, default=ROOT/"results"/"flash_map_diag",
                   help="Output folder for results JSON and raw UART log")
    a = p.parse_args()
    if a.flash and not a.allow_upload:
        p.error("--flash requires --allow-upload: ST-Link behavior must be checked")
    if a.write_size and not (a.allow_flash_write and a.backup_confirmed):
        p.error("--write-size requires BOTH --allow-flash-write and --backup-confirmed")
    if a.write_size and not a.port:
        p.error("--write-size requires --port")
    if not a.port and not a.ble_log and not a.flash:
        p.error("Supply --port, --ble-log or --flash")
    if a.flash:
        upload_diagnostic()
    report = {}
    if a.port:
        report = run_serial(a)
    if a.ble_log:
        report["web"] = verify_web_export(a.ble_log)
    a.output.mkdir(parents=True,exist_ok=True)
    tag = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    dest = a.output / ("flash_map_" + tag + ".json")
    dest.write_text(json.dumps(report,indent=2,ensure_ascii=False), encoding="utf-8")
    if "uart_log" in report:
        (a.output / ("flash_map_" + tag + ".log")).write_text(
            "\n".join(report["uart_log"])+"\n", encoding="utf-8")
    print("REPORT:", dest)
    print("DONE: browser must be connected to JDY-33 or request a retransmission.")


if __name__ == "__main__":
    main()
