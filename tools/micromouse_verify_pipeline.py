#!/usr/bin/env python3
"""Unified Micromouse verification pipeline (build, upload, UART capture, and acceptance).

Single-file tool for Windows/PowerShell when Micromouse is connected via ST-Link + USB-TTL:
1. Run offline regression tests (optional --skip-tests).
2. Build PlatformIO firmware (--env, default: integration_diag_motor_off_96m).
3. Upload firmware via ST-Link (--skip-upload to skip).
4. Capture UART1 debug telemetry log (auto-detect COM port or --port COMx).
5. Parse key health counters (TIM11, MPU/I2C, IR, UART drops/errors) and write
   raw log + summary.json under results/verify_<timestamp>/.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - runtime dependency check
    serial = None
    list_ports = None


ROOT_DIR = Path(__file__).resolve().parent.parent
RESULTS_ROOT = ROOT_DIR / "results"
DEFAULT_ENV = "integration_diag_motor_off_96m"


def run_cmd(cmd: list[str], cwd: Path, log_path: Path | None = None) -> tuple[int, str]:
    print(f"[RUN] {' '.join(cmd)}", flush=True)
    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    output = proc.stdout or ""
    if output:
        print(output, end="" if output.endswith("\n") else "\n", flush=True)
    if log_path is not None:
        log_path.write_text(output, encoding="utf-8")
    return proc.returncode, output


def detect_uart_port(preferred: str | None) -> str:
    if preferred:
        return preferred
    if list_ports is None:
        raise RuntimeError("pyserial is required for UART capture. Install with: pip install pyserial")
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No COM ports detected. Connect USB-TTL (UART1 PA9/PA10) and retry.")

    # Prefer USB-TTL adapters (CH340, CP210x, FTDI, USB Serial) over Bluetooth SPP ports.
    usb_keywords = ("ch340", "cp210", "ftdi", "usb", "uart", "stlink", "serial")
    candidates = []
    for p in ports:
        desc = f"{p.description} {p.manufacturer or ''} {p.hwid or ''}".lower()
        if "bluetooth" in desc:
            continue
        score = sum(1 for kw in usb_keywords if kw in desc)
        candidates.append((score, p.device, p.description))

    if not candidates:
        candidates = [(0, p.device, p.description) for p in ports]
    candidates.sort(key=lambda item: (-item[0], item[1]))
    chosen = candidates[0]
    print(f"[PORT] Auto-selected {chosen[1]} ({chosen[2]})", flush=True)
    return chosen[1]


def run_regression_tests(out_dir: Path) -> tuple[bool, list[dict[str, Any]]]:
    test_scripts = [
        ROOT_DIR / "tests" / "regression" / "ir_sensor_native_test.py",
        ROOT_DIR / "tests" / "regression" / "flash_map_protocol_test.py",
        ROOT_DIR / "tests" / "regression" / "ssd1306_native_test.py",
        ROOT_DIR / "tests" / "regression" / "spp_handshake_test.py",
        ROOT_DIR / "tests" / "regression" / "wheel_speed_pid_native_test.py",
        ROOT_DIR / "tests" / "regression" / "motor_driver_native_test.py",
        ROOT_DIR / "tests" / "regression" / "encoder_native_test.py",
        ROOT_DIR / "tests" / "regression" / "platformio_profile_cleanup_test.py",
    ]
    results: list[dict[str, Any]] = []
    all_ok = True
    for script in test_scripts:
        if not script.exists():
            continue
        log_file = out_dir / f"test_{script.stem}.log"
        rc, _ = run_cmd([sys.executable, str(script)], ROOT_DIR, log_file)
        ok = rc == 0
        all_ok = all_ok and ok
        results.append({"script": script.name, "passed": ok, "returncode": rc})
    return all_ok, results


def capture_uart(
    port: str,
    baud: int,
    duration_s: float,
    raw_log_path: Path,
    send_cmds: list[str],
) -> list[str]:
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

    lines: list[str] = []
    print(f"[UART] Opening {port} @ {baud} baud for {duration_s:.1f}s...", flush=True)
    with serial.Serial(port, baudrate=baud, timeout=0.2) as ser:
        time.sleep(0.25)
        ser.reset_input_buffer()
        for cmd in send_cmds:
            payload = (cmd.rstrip("\r\n") + "\r\n").encode("utf-8")
            ser.write(payload)
            ser.flush()
            print(f"[UART TX] {cmd}", flush=True)

        deadline = time.monotonic() + duration_s
        buf = bytearray()
        with raw_log_path.open("w", encoding="utf-8", errors="replace") as f:
            while time.monotonic() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                buf.extend(chunk)
                while b"\n" in buf:
                    raw_line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    text = raw_line.decode("utf-8", errors="replace").rstrip("\r")
                    f.write(text + "\n")
                    f.flush()
                    lines.append(text)
                    print(f"  {text}", flush=True)
            if buf:
                tail = buf.decode("utf-8", errors="replace").rstrip("\r")
                f.write(tail + "\n")
                lines.append(tail)
                print(f"  {tail}", flush=True)
    return lines


def parse_kv_tokens(line: str) -> dict[str, str]:
    kv: dict[str, str] = {}
    for m in re.finditer(r"([A-Za-z0-9_]+)=([^\s,]+)", line):
        kv[m.group(1)] = m.group(2)
    return kv


def analyze_uart_lines(lines: list[str]) -> dict[str, Any]:
    sys_t_count = 0
    sys_ir_count = 0
    sys_mpu_count = 0
    max_late = 0
    max_gap = 0
    max_i2c_err = 0
    max_i2c_to = 0
    max_uart_drop = 0
    max_uart_ore = 0
    ir_nonzero_scans = 0
    fault_lines: list[str] = []

    for line in lines:
        kv = parse_kv_tokens(line)
        upper = line.upper()
        if "HARDFAULT" in upper or "CLOCK_FAULT" in upper or "FLASH_ERR" in upper:
            fault_lines.append(line)

        if line.startswith("SYS_T"):
            sys_t_count += 1
            max_late = max(max_late, int(kv.get("late", "0")))
            max_gap = max(max_gap, int(kv.get("gap", "0")))
        elif line.startswith("SYS_IR"):
            sys_ir_count += 1
            if int(kv.get("scans", "0")) > 0 or int(kv.get("ok", "0")) > 0:
                ir_nonzero_scans += 1
        elif line.startswith("SYS_MPU") or line.startswith("SYS_I2C"):
            sys_mpu_count += 1
            max_i2c_err = max(max_i2c_err, int(kv.get("i2cerr", kv.get("err", "0"))))
            max_i2c_to = max(max_i2c_to, int(kv.get("to", kv.get("timeout", "0"))))
        elif line.startswith("SYS_UR"):
            max_uart_ore = max(max_uart_ore, int(kv.get("ore", "0")))
            max_uart_drop = max(max_uart_drop, int(kv.get("tx_drop", kv.get("drop", "0"))))

    checks = {
        "has_uart_output": len(lines) > 0,
        "no_fault_lines": len(fault_lines) == 0,
        "tim11_no_late_or_gap": max_late == 0 and max_gap == 0,
        "i2c_no_errors_or_timeouts": max_i2c_err == 0 and max_i2c_to == 0,
        "uart_no_ore_or_drop": max_uart_ore == 0 and max_uart_drop == 0,
    }
    overall_pass = all(checks.values())
    return {
        "line_count": len(lines),
        "sys_t_frames": sys_t_count,
        "sys_ir_frames": sys_ir_count,
        "sys_mpu_frames": sys_mpu_count,
        "ir_nonzero_scans": ir_nonzero_scans,
        "max_tim11_late": max_late,
        "max_tim11_gap": max_gap,
        "max_i2c_err": max_i2c_err,
        "max_i2c_timeout": max_i2c_to,
        "max_uart_ore": max_uart_ore,
        "max_uart_drop": max_uart_drop,
        "fault_lines": fault_lines[:10],
        "checks": checks,
        "verdict": "PASS" if overall_pass else "FAIL",
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build, upload, capture UART debug log, and verify Micromouse firmware."
    )
    parser.add_argument("--env", default=DEFAULT_ENV, help=f"PlatformIO environment (default: {DEFAULT_ENV})")
    parser.add_argument("--port", default=None, help="UART1 COM port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate (default: 115200)")
    parser.add_argument("--duration", type=float, default=15.0, help="UART capture duration in seconds (default: 15)")
    parser.add_argument("--skip-tests", action="store_true", help="Skip offline regression tests")
    parser.add_argument("--skip-build", action="store_true", help="Skip PlatformIO build")
    parser.add_argument("--skip-upload", action="store_true", help="Skip ST-Link upload")
    parser.add_argument("--skip-capture", action="store_true", help="Skip UART capture and online analysis")
    parser.add_argument("--send", action="append", default=[], help="Optional UART command to send after opening port")
    parser.add_argument("--restore-env", default=None, help="Optional PlatformIO env to upload after capture")
    args = parser.parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = RESULTS_ROOT / f"verify_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "timestamp": stamp,
        "env": args.env,
        "baud": args.baud,
        "duration_s": args.duration,
        "regression_tests": [],
        "build_ok": None,
        "upload_ok": None,
        "uart_analysis": None,
        "verdict": "INCOMPLETE",
    }

    if not args.skip_tests:
        tests_ok, test_rows = run_regression_tests(out_dir)
        summary["regression_tests"] = test_rows
        if not tests_ok:
            summary["verdict"] = "FAIL_TESTS"
            (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            print(f"[FAIL] Regression tests failed. See {out_dir}", flush=True)
            return 1

    if not args.skip_build:
        rc, _ = run_cmd(["pio", "run", "-e", args.env], ROOT_DIR, out_dir / "build.log")
        summary["build_ok"] = rc == 0
        if rc != 0:
            summary["verdict"] = "FAIL_BUILD"
            (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            print(f"[FAIL] PlatformIO build failed for {args.env}. See {out_dir / 'build.log'}", flush=True)
            return rc

    if not args.skip_upload:
        rc, _ = run_cmd(["pio", "run", "-e", args.env, "-t", "upload"], ROOT_DIR, out_dir / "upload.log")
        summary["upload_ok"] = rc == 0
        if rc != 0:
            summary["verdict"] = "FAIL_UPLOAD"
            (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            print(f"[FAIL] ST-Link upload failed. See {out_dir / 'upload.log'}", flush=True)
            return rc

    if args.skip_capture:
        summary["verdict"] = "PASS_OFFLINE"
        (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"\n[DONE] Verdict={summary['verdict']} (Offline test & build passed) | Results saved in: {out_dir}", flush=True)
        return 0

    port = detect_uart_port(args.port)
    summary["port"] = port
    raw_log = out_dir / "uart_raw.log"
    lines = capture_uart(port, args.baud, args.duration, raw_log, args.send)
    analysis = analyze_uart_lines(lines)
    summary["uart_analysis"] = analysis
    summary["verdict"] = analysis["verdict"]

    if args.restore_env:
        rc, _ = run_cmd(["pio", "run", "-e", args.restore_env, "-t", "upload"], ROOT_DIR, out_dir / "restore_upload.log")
        summary["restore_upload_ok"] = rc == 0

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\n[DONE] Verdict={summary['verdict']} | Results saved in: {out_dir}", flush=True)
    return 0 if summary["verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
