#!/usr/bin/env python3
"""Shared SPP Motion Control client, CSV logger, metrics analyzer, and plotter.

Used by both:
- tools/classic_spp_dashboard.py (GUI)
- tools/motion_test_runner.py (headless CLI automated runner)

Never opens two connections to the same COM port; protocol and parsing stay unified.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time
from typing import Any, Callable

from classic_spp_ports import classify_port
from classic_spp_protocol import (
    LineFramer,
    MOTION_STATE_NAMES,
    parse_ext_telemetry_line,
    parse_motion_telemetry_line,
    parse_spp_control_line,
)

# Robot physical constants (must match include/pinout.h)
WHEEL_DIAMETER_MM = 34.0
WHEEL_BASE_MM = 84.0
ENCODER_PPR = 1430.0
CELL_SIZE_MM = 165.0
MM_PER_PULSE = (math.pi * WHEEL_DIAMETER_MM) / ENCODER_PPR
PULSES_PER_MM = ENCODER_PPR / (math.pi * WHEEL_DIAMETER_MM)

DEFAULT_WHEEL_GAINS = {
    "kp": 0.090,
    "ki": 0.250,
    "ks": 80.0,
    "kv": 0.098,
    "ka": 0.0,
    "integral_limit": 3000.0,
    "pwm_limit": 1000.0,
}

DEFAULT_HEADING_GAINS = {
    "kp": 4.0,
    "ki": 0.0,
    "kd": 0.05,
    "max_w_dps": 120.0,
}

CSV_COLUMNS = [
    "host_timestamp",
    "mcu_timestamp_ms",
    "test_id",
    "sample_seq",
    "motion_state",
    "phase",
    "enc_l",
    "enc_r",
    "ref_l_pps",
    "meas_l_pps",
    "error_l_pps",
    "ref_r_pps",
    "meas_r_pps",
    "error_r_pps",
    "pwm_cmd_l_permille",
    "pwm_cmd_r_permille",
    "pwm_applied_l_permille",
    "pwm_applied_r_permille",
    "yaw_deg",
    "gyro_z_dps",
    "distance_mm",
    "angle_error_deg",
    "v_ref_mm_s",
    "v_meas_mm_s",
    "w_ref_dps",
    "w_encoder_dps",
    "integral_l",
    "integral_r",
    "sat_flags",
    "fault_flags",
    "telemetry_drop_counter",
]


def wheel_pps_to_chassis(left_pps: float, right_pps: float) -> tuple[float, float]:
    """Inverse differential kinematics: (left_pps, right_pps) -> (v_mm_s, w_dps)."""
    v_l = left_pps * MM_PER_PULSE
    v_r = right_pps * MM_PER_PULSE
    v_mm_s = 0.5 * (v_l + v_r)
    w_rad_s = (v_r - v_l) / WHEEL_BASE_MM
    return v_mm_s, math.degrees(w_rad_s)


def chassis_to_wheel_pps(v_mm_s: float, w_dps: float) -> tuple[float, float]:
    """Forward differential kinematics: (v_mm_s, w_dps) -> (left_pps, right_pps)."""
    w_rad = math.radians(w_dps)
    half_diff = 0.5 * WHEEL_BASE_MM * w_rad
    return (v_mm_s - half_diff) * PULSES_PER_MM, (v_mm_s + half_diff) * PULSES_PER_MM


def list_serial_ports() -> list[dict[str, str]]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    rows = []
    for p in list_ports.comports():
        direction, _ = classify_port(p.hwid or "")
        rows.append({
            "device": p.device,
            "description": p.description or "",
            "manufacturer": p.manufacturer or "",
            "hwid": p.hwid or "",
            "direction": direction,
        })
    return rows


def detect_spp_outgoing_port(preferred: str | None = None) -> str | None:
    if preferred:
        return preferred
    ports = list_serial_ports()
    named = []
    outgoing = []
    for p in ports:
        if p["direction"] == "incoming":
            continue
        desc = f"{p['description']} {p['manufacturer']}".lower()
        if "jdy-33" in desc or "nezumi_chan_spp" in desc:
            named.append(p["device"])
        if p["direction"] == "outgoing":
            outgoing.append(p["device"])
    if len(named) == 1:
        return named[0]
    if len(outgoing) == 1:
        return outgoing[0]
    return named[0] if named else (outgoing[0] if outgoing else None)


def get_git_commit_sha(repo_root: Path) -> str:
    try:
        res = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            timeout=3,
        )
        return res.stdout.strip() if res.returncode == 0 else "unknown"
    except Exception:
        return "unknown"


class MotionSppClient:
    """Thread-safe SPP serial client shared by Dashboard and Headless Runner."""

    def __init__(
        self,
        on_line: Callable[[str], None] | None = None,
        on_telemetry: Callable[[dict[str, Any]], None] | None = None,
        on_ext: Callable[[dict[str, Any]], None] | None = None,
        on_control: Callable[[dict[str, Any]], None] | None = None,
        on_status: Callable[[str], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ):
        self.on_line = on_line
        self.on_telemetry = on_telemetry
        self.on_ext = on_ext
        self.on_control = on_control
        self.on_status = on_status
        self.on_error = on_error

        self._stop_evt = threading.Event()
        self._tx_queue: queue.Queue[bytes] = queue.Queue(maxsize=64)
        self._rx_thread: threading.Thread | None = None
        self._hb_thread: threading.Thread | None = None
        self._hb_active = threading.Event()

        self._cmd_lock = threading.Lock()
        self._next_cmd_id = 1
        self._pending_acks: dict[int, tuple[threading.Event, dict[str, Any] | None]] = {}
        self._ack_results: dict[int, dict[str, Any]] = {}
        self._latest_gains: dict[str, dict[str, float]] = {}
        self._latest_hgains: dict[str, float] = {}
        self._done_event = threading.Event()
        self._last_done_payload: dict[str, Any] | None = None

        self.connected = threading.Event()
        self.port: str = ""
        self.baud: int = 115200
        self.stop_sent_at: float | None = None

    def is_connected(self) -> bool:
        return self.connected.is_set() and self._rx_thread is not None and self._rx_thread.is_alive()

    def connect(self, port: str, baud: int = 115200, timeout_s: float = 12.0) -> None:
        if self.is_connected():
            self.disconnect()
        self.port = port
        self.baud = baud
        self._stop_evt.clear()
        self.connected.clear()
        while not self._tx_queue.empty():
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                break
        self._rx_thread = threading.Thread(
            target=self._io_loop,
            args=(port, baud),
            daemon=True,
            name="spp-motion-io",
        )
        self._rx_thread.start()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.connected.is_set():
                return
            if not self._rx_thread.is_alive():
                break
            time.sleep(0.05)
        if not self.connected.is_set():
            self.disconnect()
            raise RuntimeError(f"Failed to open SPP port {port}")

    def disconnect(self) -> None:
        self.stop_heartbeat()
        self._stop_evt.set()
        self.connected.clear()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2.0)
        self._rx_thread = None

    def start_heartbeat(self, interval_s: float = 0.30) -> None:
        if self._hb_thread and self._hb_thread.is_alive():
            self._hb_active.set()
            return
        self._hb_active.set()
        self._hb_thread = threading.Thread(
            target=self._hb_loop,
            args=(interval_s,),
            daemon=True,
            name="spp-motion-hb",
        )
        self._hb_thread.start()

    def stop_heartbeat(self) -> None:
        self._hb_active.clear()

    def _hb_loop(self, interval_s: float) -> None:
        while not self._stop_evt.is_set():
            if self._hb_active.is_set() and self.connected.is_set():
                try:
                    self.send_raw_line_nowait("HB")
                except Exception:
                    pass
            time.sleep(interval_s)

    def _alloc_cmd_id(self) -> int:
        with self._cmd_lock:
            cid = self._next_cmd_id
            self._next_cmd_id += 1
            if self._next_cmd_id > 999999:
                self._next_cmd_id = 1
            return cid

    def send_raw_bytes(self, data: bytes) -> bool:
        if not self.is_connected():
            return False
        try:
            self._tx_queue.put_nowait(data)
            return True
        except queue.Full:
            return False

    def send_raw_line_nowait(self, verb: str, *args: Any) -> int:
        cid = self._alloc_cmd_id()
        parts = [f"${cid}", verb] + [str(a) for a in args]
        payload = (",".join(parts) + "\n").encode("ascii")
        self.send_raw_bytes(payload)
        return cid

    def send_emergency_stop(self) -> None:
        self.stop_heartbeat()
        self.stop_sent_at = time.monotonic()
        # Send both raw '!' and structured $0,STOP
        self.send_raw_bytes(b"!\n$0,STOP\n")

    def clear_done_event(self) -> None:
        self._done_event.clear()
        self._last_done_payload = None

    def wait_done_event(self, timeout_s: float) -> dict[str, Any] | None:
        if self._done_event.wait(timeout_s):
            return self._last_done_payload
        return None

    def send_command(
        self,
        verb: str,
        *args: Any,
        timeout_s: float = 2.0,
    ) -> dict[str, Any]:
        if not self.is_connected():
            raise RuntimeError("SPP not connected")
        cid = self._alloc_cmd_id()
        evt = threading.Event()
        with self._cmd_lock:
            self._pending_acks[cid] = (evt, None)
            self._ack_results.pop(cid, None)

        parts = [f"${cid}", verb] + [str(a) for a in args]
        payload = (",".join(parts) + "\n").encode("ascii")
        if not self.send_raw_bytes(payload):
            with self._cmd_lock:
                self._pending_acks.pop(cid, None)
            raise RuntimeError("SPP TX queue full")

        if not evt.wait(timeout_s):
            with self._cmd_lock:
                self._pending_acks.pop(cid, None)
            raise TimeoutError(f"Timeout waiting ACK for {verb} (id={cid})")

        with self._cmd_lock:
            self._pending_acks.pop(cid, None)
            res = self._ack_results.pop(cid, {})
        return res

    def read_gains(self, timeout_s: float = 2.0) -> dict[str, Any]:
        ack = self.send_command("GET_GAINS", timeout_s=timeout_s)
        return {
            "ack": ack,
            "left": dict(self._latest_gains.get("L", DEFAULT_WHEEL_GAINS)),
            "right": dict(self._latest_gains.get("R", DEFAULT_WHEEL_GAINS)),
            "heading": dict(self._latest_hgains or DEFAULT_HEADING_GAINS),
        }

    def _io_loop(self, port: str, baud: int) -> None:
        import serial

        framer = LineFramer()
        try:
            with serial.Serial(
                port,
                baud,
                timeout=0.05,
                write_timeout=2.0,
            ) as link:
                self.connected.set()
                if self.on_status:
                    self.on_status(f"Connected {port} @ {baud}")
                while not self._stop_evt.is_set():
                    try:
                        while True:
                            pkt = self._tx_queue.get_nowait()
                            try:
                                link.write(pkt)
                            except serial.SerialTimeoutException:
                                pass
                    except queue.Empty:
                        pass

                    waiting = link.in_waiting
                    chunk = link.read(waiting or 1)
                    if chunk:
                        for line in framer.feed(chunk):
                            self._dispatch_line(line)
        except Exception as exc:
            if self.on_error:
                self.on_error(str(exc))
        finally:
            self.connected.clear()
            if self.on_status:
                self.on_status("Disconnected")

    def _dispatch_line(self, line: str) -> None:
        if self.on_line:
            self.on_line(line)

        telem = parse_motion_telemetry_line(line)
        if telem is not None:
            if self.on_telemetry:
                self.on_telemetry(telem)
            return

        ext = parse_ext_telemetry_line(line)
        if ext is not None:
            if self.on_ext:
                self.on_ext(ext)
            return

        ctrl = parse_spp_control_line(line)
        if ctrl is not None:
            kind = ctrl["kind"]
            if kind == "GAINS":
                self._latest_gains[ctrl["side"]] = {
                    "kp": ctrl["kp"],
                    "ki": ctrl["ki"],
                    "ks": ctrl["ks"],
                    "kv": ctrl["kv"],
                    "ka": ctrl["ka"],
                    "integral_limit": ctrl["integral_limit"],
                    "pwm_limit": ctrl["pwm_limit"],
                }
            elif kind == "HGAINS":
                self._latest_hgains = {
                    "kp": ctrl["kp"],
                    "ki": ctrl["ki"],
                    "kd": ctrl["kd"],
                    "max_w_dps": ctrl["max_w_dps"],
                }
            elif kind in ("ACK", "NACK"):
                cid = ctrl["cmd_id"]
                with self._cmd_lock:
                    entry = self._pending_acks.get(cid)
                    if entry is not None:
                        self._ack_results[cid] = ctrl
                        entry[0].set()
            elif kind == "EVENT" and ctrl["event"] == "DONE":
                self._last_done_payload = ctrl
                self._done_event.set()

            if self.on_control:
                self.on_control(ctrl)


class MotionCsvLogger:
    """Per-run directory logger writing raw_serial.log, telemetry.csv, and parameters.json."""

    def __init__(self, base_dir: Path, test_name: str, test_id: str, metadata: dict[str, Any]):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(c if (c.isalnum() or c in "-_") else "_" for c in test_name).strip("_") or "motion"
        self.run_dir = base_dir / f"{stamp}_{safe_name}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.test_id = test_id
        self.metadata = dict(metadata)
        self.metadata["run_dir"] = str(self.run_dir)
        self.metadata["started_at"] = datetime.now().isoformat(timespec="seconds")

        self.raw_path = self.run_dir / "raw_serial.log"
        self.csv_path = self.run_dir / "telemetry.csv"
        self.params_path = self.run_dir / "parameters.json"

        self._raw_fp = self.raw_path.open("w", encoding="utf-8", errors="replace")
        self._csv_fp = self.csv_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._csv_fp, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()

        self._seq = 0
        self._last_ext: dict[str, Any] = {}
        self._last_mcu_t: int | None = None
        self._drop_est = 0
        self._fault_flags = 0
        self._lock = threading.Lock()
        self.save_parameters()

    def save_parameters(self) -> None:
        self.params_path.write_text(
            json.dumps(self.metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def log_raw(self, line: str) -> None:
        with self._lock:
            if self._raw_fp.closed:
                return
            self._raw_fp.write(f"{datetime.now().isoformat(timespec='milliseconds')} | {line}\n")

    def update_ext(self, ext: dict[str, Any]) -> None:
        with self._lock:
            self._last_ext = dict(ext)

    def log_telemetry(self, telem: dict[str, Any]) -> None:
        with self._lock:
            if self._csv_fp.closed:
                return
            mcu_t = int(telem["t_ms"])
            if self._last_mcu_t is not None:
                dt = mcu_t - self._last_mcu_t
                if dt > 35:
                    # 50Hz = 20ms period; dt > 35ms implies missed sample(s)
                    self._drop_est += max(1, round(dt / 20.0) - 1)
            self._last_mcu_t = mcu_t

            v_ref, w_ref = wheel_pps_to_chassis(telem["ref_l_pps"], telem["ref_r_pps"])
            v_meas, w_enc = wheel_pps_to_chassis(telem["meas_l_pps"], telem["meas_r_pps"])
            ext = self._last_ext

            row = {
                "host_timestamp": f"{time.time():.6f}",
                "mcu_timestamp_ms": mcu_t,
                "test_id": self.test_id,
                "sample_seq": self._seq,
                "motion_state": MOTION_STATE_NAMES.get(int(telem["state"]), str(telem["state"])),
                "phase": int(telem["phase"]),
                "enc_l": int(telem["enc_l"]),
                "enc_r": int(telem["enc_r"]),
                "ref_l_pps": f"{telem['ref_l_pps']:.1f}",
                "meas_l_pps": f"{telem['meas_l_pps']:.1f}",
                "error_l_pps": f"{telem['error_l_pps']:.1f}",
                "ref_r_pps": f"{telem['ref_r_pps']:.1f}",
                "meas_r_pps": f"{telem['meas_r_pps']:.1f}",
                "error_r_pps": f"{telem['error_r_pps']:.1f}",
                "pwm_cmd_l_permille": int(telem["pwm_cmd_l_pm"]),
                "pwm_cmd_r_permille": int(telem["pwm_cmd_r_pm"]),
                "pwm_applied_l_permille": int(telem["pwm_app_l_pm"]),
                "pwm_applied_r_permille": int(telem["pwm_app_r_pm"]),
                "yaw_deg": f"{telem['yaw_deg']:.2f}",
                "gyro_z_dps": f"{telem['gyro_z_dps']:.2f}",
                "distance_mm": f"{telem['distance_mm']:.1f}",
                "angle_error_deg": f"{telem['angle_error_deg']:.2f}",
                "v_ref_mm_s": f"{ext.get('v_ref_mm_s', v_ref):.2f}",
                "v_meas_mm_s": f"{ext.get('v_meas_mm_s', v_meas):.2f}",
                "w_ref_dps": f"{ext.get('w_ref_dps', w_ref):.2f}",
                "w_encoder_dps": f"{ext.get('w_meas_dps', w_enc):.2f}",
                "integral_l": f"{ext.get('integral_l', 0.0):.2f}",
                "integral_r": f"{ext.get('integral_r', 0.0):.2f}",
                "sat_flags": int(telem["sat_flags"]),
                "fault_flags": self._fault_flags,
                "telemetry_drop_counter": self._drop_est,
            }
            self._writer.writerow(row)
            self._seq += 1

    def close(self, final_metadata: dict[str, Any] | None = None) -> None:
        with self._lock:
            if final_metadata:
                self.metadata.update(final_metadata)
            self.metadata["finished_at"] = datetime.now().isoformat(timespec="seconds")
            self.metadata["sample_count"] = self._seq
            self.metadata["drop_estimate"] = self._drop_est
            self.save_parameters()
            if not self._csv_fp.closed:
                self._csv_fp.flush()
                self._csv_fp.close()
            if not self._raw_fp.closed:
                self._raw_fp.flush()
                self._raw_fp.close()


def _wheel_metrics(
    t_ms: list[float],
    refs: list[float],
    meas: list[float],
    pwm_cmd: list[float],
    pwm_max_pm: float,
) -> dict[str, Any]:
    n = len(t_ms)
    if n == 0:
        return {"samples": 0, "verdict": "NO_DATA"}

    active_idx = [i for i in range(n) if abs(refs[i]) > 1e-3]
    if not active_idx:
        # Idle or zero reference: only report mean meas and PWM check
        nonzero_pwm = sum(1 for p in pwm_cmd if abs(p) > 0)
        return {
            "samples": n,
            "mean_ref_pps": 0.0,
            "mean_meas_pps": round(sum(meas) / n, 2),
            "steady_state_error_pps": None,
            "steady_state_error_pct": "N/A (ref=0)",
            "mae_pps": round(sum(abs(m) for m in meas) / n, 2),
            "rmse_pps": round(math.sqrt(sum(m * m for m in meas) / n), 2),
            "max_abs_error_pps": round(max(abs(m) for m in meas), 2),
            "rise_time_10_90_ms": "N/A (ref=0)",
            "overshoot_pct": "N/A (ref=0)",
            "settling_time_5pct_ms": "N/A (ref=0)",
            "steady_std_pps": "N/A (ref=0)",
            "pwm_sat_pct": round(100.0 * sum(1 for p in pwm_cmd if abs(p) >= pwm_max_pm - 1.0) / n, 2),
            "nonzero_pwm_when_ref_zero": nonzero_pwm,
        }

    # Find longest contiguous segment with near-constant reference for step/steady analysis
    best_seg: list[int] = []
    cur_seg: list[int] = [active_idx[0]]
    for idx in active_idx[1:]:
        if idx == cur_seg[-1] + 1 and abs(refs[idx] - refs[cur_seg[-1]]) <= max(5.0, 0.02 * abs(refs[cur_seg[-1]])):
            cur_seg.append(idx)
        else:
            if len(cur_seg) > len(best_seg):
                best_seg = cur_seg
            cur_seg = [idx]
    if len(cur_seg) > len(best_seg):
        best_seg = cur_seg

    r_seg = [refs[i] for i in best_seg]
    m_seg = [meas[i] for i in best_seg]
    t_seg = [t_ms[i] for i in best_seg]
    p_seg = [pwm_cmd[i] for i in best_seg]

    mean_ref = sum(r_seg) / len(r_seg)
    errs = [r_seg[k] - m_seg[k] for k in range(len(best_seg))]
    abs_errs = [abs(e) for e in errs]
    mae = sum(abs_errs) / len(abs_errs)
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    max_abs_err = max(abs_errs)

    # Separate transient vs steady-state: last 45% of constant-ref segment (min 5 samples)
    steady_start = max(0, int(0.55 * len(best_seg)))
    steady_m = m_seg[steady_start:]
    steady_r = r_seg[steady_start:]
    mean_steady_ref = sum(steady_r) / len(steady_r)
    mean_steady_meas = sum(steady_m) / len(steady_m)
    ss_err_pps = mean_steady_ref - mean_steady_meas
    ss_err_pct: float | str = (
        round(100.0 * abs(ss_err_pps) / abs(mean_steady_ref), 2)
        if abs(mean_steady_ref) > 1e-3
        else "N/A (ref=0)"
    )
    steady_var = sum((x - mean_steady_meas) ** 2 for x in steady_m) / max(1, len(steady_m) - 1)
    steady_std = math.sqrt(steady_var)

    # Rise time (10% -> 90%) & Overshoot & Settling time (±5%) only for clean step (>=12 samples)
    rise_time_ms: float | str = "N/A (insufficient steady segment)"
    overshoot_pct: float | str = "N/A (insufficient steady segment)"
    settling_ms: float | str = "N/A (insufficient steady segment)"

    if len(best_seg) >= 12 and abs(mean_steady_ref) > 50.0:
        sign = 1.0 if mean_steady_ref > 0 else -1.0
        target_mag = abs(mean_steady_ref)
        m_signed = [sign * x for x in m_seg]
        t0 = t_seg[0]
        t10 = next((t_seg[k] for k in range(len(m_signed)) if m_signed[k] >= 0.1 * target_mag), None)
        t90 = next((t_seg[k] for k in range(len(m_signed)) if m_signed[k] >= 0.9 * target_mag), None)
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_time_ms = round(t90 - t10, 1)
        else:
            rise_time_ms = "N/A (did not reach 90%)"

        peak = max(m_signed)
        overshoot_pct = round(max(0.0, 100.0 * (peak - target_mag) / target_mag), 2)

        band = 0.05 * target_mag
        outside = [k for k in range(len(m_signed)) if abs(m_signed[k] - target_mag) > band]
        if not outside:
            settling_ms = 0.0
        elif outside[-1] < len(m_signed) - 3:
            settling_ms = round(t_seg[outside[-1] + 1] - t0, 1)
        else:
            settling_ms = "N/A (not settled within ±5%)"

    sat_count = sum(1 for p in p_seg if abs(p) >= pwm_max_pm - 1.0)
    nonzero_zero_ref = sum(1 for i in range(n) if abs(refs[i]) <= 1e-3 and abs(pwm_cmd[i]) > 0)

    return {
        "samples": len(best_seg),
        "mean_ref_pps": round(mean_ref, 2),
        "mean_meas_pps": round(mean_steady_meas, 2),
        "steady_state_error_pps": round(ss_err_pps, 2),
        "steady_state_error_pct": ss_err_pct,
        "mae_pps": round(mae, 2),
        "rmse_pps": round(rmse, 2),
        "max_abs_error_pps": round(max_abs_err, 2),
        "rise_time_10_90_ms": rise_time_ms,
        "overshoot_pct": overshoot_pct,
        "settling_time_5pct_ms": settling_ms,
        "steady_std_pps": round(steady_std, 2),
        "pwm_sat_pct": round(100.0 * sat_count / len(p_seg), 2),
        "nonzero_pwm_when_ref_zero": nonzero_zero_ref,
    }


def analyze_motion_csv(
    csv_path: Path,
    pwm_max_pm: float = 1000.0,
    stop_latency_ms: float | None = None,
    finish_result: str = "UNKNOWN",
) -> dict[str, Any]:
    t_ms: list[float] = []
    ref_l, meas_l, pwm_l = [], [], []
    ref_r, meas_r, pwm_r = [], [], []
    states: list[str] = []
    drops = 0
    faults = 0
    dist_final = 0.0
    yaw_final = 0.0

    with csv_path.open("r", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            t_ms.append(float(row["mcu_timestamp_ms"]))
            ref_l.append(float(row["ref_l_pps"]))
            meas_l.append(float(row["meas_l_pps"]))
            pwm_l.append(float(row["pwm_cmd_l_permille"]))
            ref_r.append(float(row["ref_r_pps"]))
            meas_r.append(float(row["meas_r_pps"]))
            pwm_r.append(float(row["pwm_cmd_r_permille"]))
            states.append(row["motion_state"])
            drops = max(drops, int(row["telemetry_drop_counter"]))
            faults += int(row["fault_flags"])
            dist_final = float(row["distance_mm"])
            yaw_final = float(row["yaw_deg"])

    n = len(t_ms)
    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0 if n >= 2 else 0.0
    hz = (n - 1) / duration_s if duration_s > 0.05 else 0.0
    expected_samples = max(1, int(round(duration_s * 50.0)) + 1) if duration_s > 0.05 else n
    coverage_pct = min(100.0, 100.0 * n / expected_samples) if expected_samples > 0 else 0.0

    # Active-only subset (exclude leading/trailing IDLE rows when computing metrics)
    act = [i for i in range(n) if states[i] != "IDLE"]
    if act:
        t_act = [t_ms[i] for i in act]
        rl_act = [ref_l[i] for i in act]
        ml_act = [meas_l[i] for i in act]
        pl_act = [pwm_l[i] for i in act]
        rr_act = [ref_r[i] for i in act]
        mr_act = [meas_r[i] for i in act]
        pr_act = [pwm_r[i] for i in act]
    else:
        t_act, rl_act, ml_act, pl_act = t_ms, ref_l, meas_l, pwm_l
        rr_act, mr_act, pr_act = ref_r, meas_r, pwm_r

    left_m = _wheel_metrics(t_act, rl_act, ml_act, pl_act, pwm_max_pm)
    right_m = _wheel_metrics(t_act, rr_act, mr_act, pr_act, pwm_max_pm)

    same_ref_idx = [i for i in range(len(t_act)) if abs(rl_act[i] - rr_act[i]) < 5.0 and abs(rl_act[i]) > 1e-3]
    lr_diff = (
        round(sum(abs(ml_act[i] - mr_act[i]) for i in same_ref_idx) / len(same_ref_idx), 2)
        if same_ref_idx
        else "N/A (different wheel references)"
    )

    return {
        "samples_total": n,
        "samples_active": len(act),
        "duration_s": round(duration_s, 3),
        "telemetry_hz": round(hz, 2),
        "sample_loss_est": drops,
        "coverage_pct": round(coverage_pct, 2),
        "fault_count": faults,
        "stop_latency_ms": round(stop_latency_ms, 1) if stop_latency_ms is not None else "N/A",
        "finish_result": finish_result,
        "final_distance_mm": round(dist_final, 1),
        "final_yaw_deg": round(yaw_final, 2),
        "lr_diff_mean_pps": lr_diff,
        "left": left_m,
        "right": right_m,
    }


def generate_motion_plots(csv_path: Path, out_dir: Path, title_prefix: str = "") -> list[Path]:
    """Generate left_motor.png, right_motor.png, and chassis.png (non-GUI Agg)."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    t_s, ref_l, meas_l, err_l, p_cmd_l, p_app_l = [], [], [], [], [], []
    ref_r, meas_r, err_r, p_cmd_r, p_app_r = [], [], [], [], []
    v_ref, v_meas, w_ref, w_meas, yaw, gz, dist, ang_err = [], [], [], [], [], [], [], []

    with csv_path.open("r", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            t_s.append(float(row["mcu_timestamp_ms"]) / 1000.0)
            ref_l.append(float(row["ref_l_pps"]))
            meas_l.append(float(row["meas_l_pps"]))
            err_l.append(float(row["error_l_pps"]))
            p_cmd_l.append(float(row["pwm_cmd_l_permille"]))
            p_app_l.append(float(row["pwm_applied_l_permille"]))
            ref_r.append(float(row["ref_r_pps"]))
            meas_r.append(float(row["meas_r_pps"]))
            err_r.append(float(row["error_r_pps"]))
            p_cmd_r.append(float(row["pwm_cmd_r_permille"]))
            p_app_r.append(float(row["pwm_applied_r_permille"]))
            v_ref.append(float(row["v_ref_mm_s"]))
            v_meas.append(float(row["v_meas_mm_s"]))
            w_ref.append(float(row["w_ref_dps"]))
            w_meas.append(float(row["w_encoder_dps"]))
            yaw.append(float(row["yaw_deg"]))
            gz.append(float(row["gyro_z_dps"]))
            dist.append(float(row["distance_mm"]))
            ang_err.append(float(row["angle_error_deg"]))

    if not t_s:
        return []
    t0 = t_s[0]
    t = [x - t0 for x in t_s]

    saved: list[Path] = []
    for fname, side, r, m, e, pc, pa in (
        ("left_motor.png", "Left Wheel", ref_l, meas_l, err_l, p_cmd_l, p_app_l),
        ("right_motor.png", "Right Wheel", ref_r, meas_r, err_r, p_cmd_r, p_app_r),
    ):
        fig = Figure(figsize=(9, 6.5), dpi=120)
        FigureCanvasAgg(fig)
        ax1, ax2, ax3 = fig.subplots(3, 1, sharex=True)
        ax1.plot(t, r, label="Ref (PPS)", linestyle="--", color="#1f77b4")
        ax1.plot(t, m, label="Measured (PPS)", color="#2ca02c")
        ax1.set_ylabel("Speed (PPS)")
        ax1.set_title(f"{title_prefix} {side}".strip())
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc="upper right", fontsize=8)

        ax2.plot(t, e, label="Error (Ref - Meas)", color="#d62728")
        ax2.axhline(0, color="#555", linewidth=0.8)
        ax2.set_ylabel("Error (PPS)")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="upper right", fontsize=8)

        ax3.plot(t, pc, label="PWM Cmd (‰)", color="#ff7f0e")
        ax3.plot(t, pa, label="PWM Applied (‰)", linestyle=":", color="#9467bd")
        ax3.set_ylabel("PWM (‰)")
        ax3.set_xlabel("Time (s)")
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc="upper right", fontsize=8)

        fig.tight_layout()
        out_p = out_dir / fname
        fig.savefig(out_p)
        saved.append(out_p)

    fig_c = Figure(figsize=(9, 7.5), dpi=120)
    FigureCanvasAgg(fig_c)
    c1, c2, c3 = fig_c.subplots(3, 1, sharex=True)
    c1.plot(t, v_ref, label="v_ref (mm/s)", linestyle="--", color="#1f77b4")
    c1.plot(t, v_meas, label="v_meas (mm/s)", color="#2ca02c")
    c1.set_ylabel("Linear (mm/s)")
    c1.set_title(f"{title_prefix} Chassis".strip())
    c1.grid(True, alpha=0.3)
    c1.legend(loc="upper right", fontsize=8)

    c2.plot(t, w_ref, label="w_ref (deg/s)", linestyle="--", color="#ff7f0e")
    c2.plot(t, w_meas, label="w_enc (deg/s)", color="#9467bd")
    c2.plot(t, gz, label="gyro_z (deg/s)", color="#8c564b", alpha=0.8)
    c2.set_ylabel("Angular (deg/s)")
    c2.grid(True, alpha=0.3)
    c2.legend(loc="upper right", fontsize=8)

    c3.plot(t, dist, label="Distance (mm)", color="#17becf")
    ax3_r = c3.twinx()
    ax3_r.plot(t, yaw, label="Yaw (deg)", color="#e377c2")
    ax3_r.plot(t, ang_err, label="Angle Err (deg)", linestyle=":", color="#d62728")
    c3.set_ylabel("Dist (mm)")
    ax3_r.set_ylabel("Angle (deg)")
    c3.set_xlabel("Time (s)")
    c3.grid(True, alpha=0.3)
    c3.legend(loc="upper left", fontsize=8)
    ax3_r.legend(loc="upper right", fontsize=8)

    fig_c.tight_layout()
    out_c = out_dir / "chassis.png"
    fig_c.savefig(out_c)
    saved.append(out_c)
    return saved


def write_summary_md(run_dir: Path, test_name: str, params: dict[str, Any], metrics: dict[str, Any]) -> Path:
    lines = [
        f"# Motion Test Summary: {test_name}",
        "",
        f"- **Timestamp:** `{params.get('started_at', '')}`",
        f"- **Git Commit:** `{params.get('git_commit', '')}`",
        f"- **Port:** `{params.get('port', '')}`",
        f"- **Finish Result:** `{metrics.get('finish_result', 'UNKNOWN')}`",
        f"- **Duration:** `{metrics.get('duration_s', 0)} s` | **Samples:** `{metrics.get('samples_total', 0)}` | **Rate:** `{metrics.get('telemetry_hz', 0)} Hz` | **Coverage:** `{metrics.get('coverage_pct', 0)}%`",
        f"- **L/R Mean Diff:** `{metrics.get('lr_diff_mean_pps', 'N/A')} PPS` | **Final Dist:** `{metrics.get('final_distance_mm', 0)} mm` | **Final Yaw:** `{metrics.get('final_yaw_deg', 0)} deg`",
        "",
        "## Wheel Metrics",
        "",
        "| Metric | Left Wheel | Right Wheel |",
        "|---|---|---|",
    ]
    lm = metrics.get("left", {})
    rm = metrics.get("right", {})
    keys = [
        ("mean_ref_pps", "Mean Ref (PPS)"),
        ("mean_meas_pps", "Mean Meas (PPS)"),
        ("steady_state_error_pps", "Steady Error (PPS)"),
        ("steady_state_error_pct", "Steady Error (%)"),
        ("mae_pps", "MAE (PPS)"),
        ("rmse_pps", "RMSE (PPS)"),
        ("max_abs_error_pps", "Max Abs Error (PPS)"),
        ("rise_time_10_90_ms", "Rise Time 10-90% (ms)"),
        ("overshoot_pct", "Overshoot (%)"),
        ("settling_time_5pct_ms", "Settling Time ±5% (ms)"),
        ("steady_std_pps", "Steady StdDev (PPS)"),
        ("pwm_sat_pct", "PWM Saturation (%)"),
    ]
    for k, label in keys:
        lines.append(f"| {label} | {lm.get(k, 'N/A')} | {rm.get(k, 'N/A')} |")
    out = run_dir / "summary.md"
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out
