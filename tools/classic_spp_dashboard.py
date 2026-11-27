#!/usr/bin/env python3
"""JDY-33 Classic SPP dashboard with safe menu Next/Select commands.

Guarded menu commands, one-box test and RAM-only wheel PID tuning.
Flash calibration confirmation (V) is user-gated. Web BLE remains untouched.
Run: python tools/classic_spp_dashboard.py --port COM37
"""
from __future__ import annotations

import argparse
import csv
from decimal import Decimal, InvalidOperation
from wheel_plot_data import WheelPlotData
from mpu_angle_view import AngleSample, draw_angle
from wheel_speed_metrics import summarize_wheels, save_wheel_summary
import os
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from classic_spp_protocol import LineFramer, MazeState, numeric_fields
from classic_spp_handshake import SPPHandshake
from classic_spp_ports import classify_port
from spp_dual_log import (DualSessionLog, classify_spp_line,
                          data_to_csv_row, csv_row_text, CSV_COLUMNS,
                          TEST_CSV_COLUMNS, sensor_test_csv)


BAUD = 115200
# Never assume a persistent Windows COM number; auto-pick only if unambiguous.
PREFERRED_PORT = ""
MAX_EVENTS = 4096
MAX_TERMINAL_LINES = 2500
PLOT_COLORS = ("#19b7a6", "#ffb65c")


def available_ports():
    try:
        from serial.tools import list_ports
        return list(list_ports.comports())
    except ImportError:
        return []


def suggested_port(ports):
    """Pick a known JDY-33 port, or the sole outgoing Bluetooth COM.

    A Windows Bluetooth Incoming COM can appear in the same device list;
    never infer pairing/MCU readiness from a COM number alone.
    """
    named = []
    outgoing = []
    for port in ports:
        descriptor = " ".join(
            (port.description or "", port.manufacturer or "")
        ).lower()
        direction, _ = classify_port(port.hwid or "")
        if direction == "incoming":
            continue
        if "jdy-33" in descriptor or "nezumi_chan_spp" in descriptor:
            named.append(port.device)
        if direction == "outgoing":
            outgoing.append(port.device)
    if len(named) == 1:
        return named[0]
    if len(named) > 1:
        return None
    return outgoing[0] if len(outgoing) == 1 else None


class SPPReader:
    """Thread owns serial I/O. Tkinter is only touched by the main/UI thread."""

    def __init__(self, events: queue.Queue):
        self.events = events
        self.stop = threading.Event()
        # Lease for opt-in 1 Box Forward only: MCU aborts if H stops for 350ms.
        self.motion_lease = threading.Event()
        self.commands: queue.Queue[bytes] = queue.Queue(maxsize=32)
        self.thread: threading.Thread | None = None

    def _emit(self, kind: str, value):
        try:
            self.events.put_nowait((kind, value))
        except queue.Full:
            # Never block a serial receive thread on a slow GUI.
            pass

    def start(self, port: str, baud: int):
        if self.thread and self.thread.is_alive():
            raise RuntimeError("SPP reader already running")
        self.stop.clear()
        self.motion_lease.clear()
        while not self.commands.empty():
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.thread = threading.Thread(target=self._run, args=(port, baud),
                                       daemon=True, name="jdy33-spp-reader")
        self.thread.start()

    def close(self):
        self.motion_lease.clear()
        self.stop.set()

    def send_menu(self, command: bytes) -> bool:
        if command not in (b"N", b"K", b"P", b"G", b"E", b"Y", b"I", b"V") or not self.thread or not self.thread.is_alive():
            return False
        return self._enqueue(command)

    def send_pid(self, values):
        limits = (10000, 20000, 1000) * 2
        if len(values) != 6 or any(not isinstance(v, int) or not 0 <= v <= m
                                   for v, m in zip(values, limits)):
            return False
        frame = ("p" + "".join(f"{v:05d}" for v in values) + ";").encode("ascii")
        return self._enqueue(frame)

    def _enqueue(self, command):
        if not self.thread or not self.thread.is_alive():
            return False
        try:
            self.commands.put_nowait(command)
            return True
        except queue.Full:
            return False

    def _run(self, port: str, baud: int):
        connected_at = None
        total_bytes = 0
        fault = None
        try:
            import serial
            with serial.Serial(port=port, baudrate=baud, timeout=0.15,
                               write_timeout=0.5) as link:
                connected_at = time.monotonic()
                self._emit("state", "%s COM open; verifying SPP firmware" % port)
                framer = LineFramer()
                handshake = SPPHandshake()
                last_motion_ping = 0.0
                while not self.stop.is_set():
                    # The first probe may be lost while Windows/RFCOMM or the
                    # MCU is becoming ready. Retry safely and rate-limit probes.
                    now = time.monotonic()
                    if handshake.should_probe(now):
                        link.write(b"?")
                        link.flush()  # Match the previously working two-pane SPP.
                        handshake.record_probe(now)
                        if handshake.attempts == handshake.fast_attempts:
                            self._emit("warning", (
                                "COM open, no SPP CONNECTED after %d probes; "
                                "%d RX bytes. Check firmware/MPU boot status "
                                "and COM Outgoing; use Reconnect COM if needed."
                                % (handshake.attempts, total_bytes)
                            ))
                    # All serial operations remain in this worker, never Tkinter.
                    for _ in range(4):
                        try:
                            command = self.commands.get_nowait()
                        except queue.Empty:
                            break
                        link.write(command)
                    if self.motion_lease.is_set() and (
                            time.monotonic() - last_motion_ping >= 0.1):
                        link.write(b"H")
                        last_motion_ping = time.monotonic()
                    chunk = link.read(link.in_waiting or 1)
                    if not chunk:
                        continue
                    total_bytes += len(chunk)
                    if total_bytes == len(chunk):
                        self._emit("state", "SPP RX data seen; waiting for firmware handshake")
                    if total_bytes <= len(chunk) or total_bytes % 512 < len(chunk):
                        self._emit("rx_bytes", total_bytes)
                    for line in framer.feed(chunk):
                        handshake.observe_line(line)
                        self._emit("line", line)
                    if framer.discarded:
                        self._emit("warning", "Line buffer overflow: %d" % framer.discarded)
                        framer.discarded = 0
        except Exception as exc:
            age = ("before COM opened" if connected_at is None else
                   "%.2fs after COM open" % (time.monotonic() - connected_at))
            fault = "%s: %s (%s; %d RX bytes)" % (type(exc).__name__,
                                                   exc, age, total_bytes)
            self._emit("warning", "SPP DISCONNECT: " + fault)
        finally:
            state = ("Disconnected after SPP error: " + fault if fault else
                     "Disconnected by user" if self.stop.is_set() else
                     "SPP port closed")
            self._emit("closed", state)


class Dashboard:
    def __init__(self, root: tk.Tk, default_port: str):
        self.root = root
        self.root.title("Nezumi Chan | Bluetooth Classic SPP")
        self.root.geometry("1160x760")
        self.root.minsize(880, 560)
        self.root.protocol("WM_DELETE_WINDOW", self.quit)

        self.events: queue.Queue = queue.Queue(maxsize=MAX_EVENTS)
        self.reader = SPPReader(self.events)
        self.maze = MazeState()
        self.wheel_data = WheelPlotData()
        self.angle_sample = AngleSample()
        self.angle_preview = False
        self.angle_last_rx = None
        self.angle_ready = False
        self.wheel_preview = False
        self.wheel_dirty = True
        self.metrics = {}
        self.series = defaultdict(lambda: deque(maxlen=400))
        self.samples = 0
        self.total_lines = 0
        self._replaying = False
        self._last_auto_log = None
        self._logging_disabled = False
        self.dual_log = DualSessionLog(Path(__file__).resolve().parents[1] / "logs" / "spp_runs")
        self.maze_dirty = True
        self.plot_dirty = True
        self.last_plot = 0.0
        self.filter_text = tk.StringVar(value="")
        self.data_filter = tk.StringVar(value="All")
        self.status = tk.StringVar(value="Disconnected · BLE web remains independent")
        self.port_value = tk.StringVar(value=default_port)
        self.baud_value = tk.StringVar(value=str(BAUD))
        self.map_status = tk.StringVar(value=self.maze.status)
        self.rx_count = tk.StringVar(value="0 lines")
        self.menu_ready = False
        self.onebox_capable = False
        self.mpu_boot_ok = True
        self.rx_byte_count = tk.StringVar(value="RX 0 bytes")
        self._reconnect_attempts = 0
        self.active_test_name = None
        self.test_stream_running = False
        self.onebox_start_pending = False
        self.active_data_stream = None
        self.cal_rows=deque(maxlen=1000)
        self.cal_live=(0,)*6
        self.cal_live_valid=False
        self.cal_monitoring=False
        self.cal_wall_state=[False]*6
        self.cal_thresholds={}
        self.cal_review=False
        self.cal_visible=False
        self.cal_note=tk.StringVar(value="Mode 1: chọn để bắt đầu")
        self.plot_a = tk.StringVar(value="l0")
        self.plot_b = tk.StringVar(value="r0")

        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Title.TLabel", font=("Segoe UI", 15, "bold"))
        style.configure("Muted.TLabel", font=("Segoe UI", 9))
        self._build()
        self.refresh_ports()
        self.root.after(40, self.poll)
        self.root.after(150, self.redraw)

    def _build(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Nezumi Chan · Classic SPP Console",
                  style="Title.TLabel").pack(anchor="w")
        ttk.Label(outer, text="JDY-33 · STM32 USART2 PA2/PA3 · '?' check, N=Next, K=Select. "
                  "Guarded motion; valid IR candidates are saved after 360° but not auto-applied.",
                  style="Muted.TLabel").pack(anchor="w", pady=(1, 8))
        bar = ttk.Frame(outer)
        bar.pack(fill="x", pady=(0, 8))
        ttk.Label(bar, text="SPP COM:").pack(side="left")
        self.ports_box = ttk.Combobox(bar, width=36, textvariable=self.port_value)
        self.ports_box.pack(side="left", padx=(5, 6))
        ttk.Button(bar, text="Refresh", command=self.refresh_ports).pack(side="left")
        ttk.Label(bar, text="Baud:").pack(side="left", padx=(12, 4))
        ttk.Combobox(bar, textvariable=self.baud_value, width=9,
                     values=("9600", "38400", "57600", "115200", "230400"),
                     state="readonly").pack(side="left")
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.connect)
        self.connect_btn.pack(side="left", padx=(10, 4))
        ttk.Button(bar, text="Disconnect", command=self.disconnect).pack(side="left")
        ttk.Button(bar, text="Reconnect COM", command=self.reconnect).pack(side="left", padx=(4, 0))
        ttk.Button(bar, text="Replay CSV/log", command=self.open_log).pack(side="right")
        ttk.Button(bar, text="Open logs folder", command=self.open_logs_folder).pack(
            side="right", padx=(4, 4))

        info = ttk.Frame(outer)
        info.pack(fill="x", pady=(0, 7))
        ttk.Label(info, textvariable=self.status).pack(side="left")
        ttk.Label(info, textvariable=self.rx_count).pack(side="right")
        ttk.Label(info, textvariable=self.rx_byte_count).pack(side="right", padx=(0, 12))

        self.tabs = ttk.Notebook(outer)
        self.tabs.pack(fill="both", expand=True)
        self.terminal_tab = ttk.Frame(self.tabs, padding=8)
        self.maze_tab = ttk.Frame(self.tabs, padding=8)
        self.telemetry_tab = ttk.Frame(self.tabs, padding=8)
        self.tabs.add(self.terminal_tab, text="Terminal")
        self.tabs.add(self.maze_tab, text="Maze viewer")
        self.tabs.add(self.telemetry_tab, text="Telemetry & PID")
        self._terminal_widgets()
        self._maze_widgets()
        self._telemetry_widgets()

    def _terminal_widgets(self):
        panes = ttk.PanedWindow(self.terminal_tab, orient="horizontal")
        panes.pack(fill="both", expand=True)
        left = ttk.LabelFrame(panes, text="Continuous data - MPU / IR / Encoder", padding=5)
        right = ttk.LabelFrame(panes, text="Micromouse status - modes / events", padding=5)
        panes.add(left, weight=3)
        panes.add(right, weight=2)

        data_bar = ttk.Frame(left)
        data_bar.pack(fill="x", pady=(0, 5))
        ttk.Label(data_bar, text="Stream:").pack(side="left")
        ttk.Combobox(data_bar, textvariable=self.data_filter, width=12,
                     values=("All", "CAL_IR", "MPU", "PIVOT_CSV", "IR", "ENC", "ONEBOX_CSV", "ALL", "ENC_CSV", "CSV", "DATA"),
                     state="readonly").pack(side="left", padx=5)
        ttk.Button(data_bar, text="Clear data", command=self.clear_data).pack(side="right")
        self.data_split = ttk.PanedWindow(left, orient="vertical")
        self.data_split.pack(fill="both", expand=True)
        self.log_pane = ttk.Frame(self.data_split)
        self.data_split.add(self.log_pane, weight=1)
        self.data_text = self._make_log_text(self.log_pane, "none")
        self.data_text.configure(height=6)
        self.cal_pane=ttk.LabelFrame(self.data_split,text="Mode 1 — IR calibration (6 mắt)")
        ttk.Label(self.cal_pane,textvariable=self.cal_note).pack(anchor="w")
        self.cal_canvas=tk.Canvas(self.cal_pane,bg="#101923",height=230,highlightthickness=0)
        self.cal_canvas.pack(fill="both",expand=True)
        self.cal_canvas.bind("<Configure>",lambda event:self._draw_calibration_panel())
        self.cal_save_btn=ttk.Button(self.cal_pane,text="Save Flash manually (V)",state="disabled",command=lambda:self.send_menu(b"V"))
        self.cal_save_btn.pack(anchor="e",pady=3)
        self.wheel_pane = ttk.Frame(self.data_split)
        self.wheel_pane.columnconfigure(0, weight=1, uniform="wheel")
        self.wheel_pane.columnconfigure(1, weight=1, uniform="wheel")
        self.wheel_pane.rowconfigure(1, weight=1)
        self.wheel_canvases = []
        self.wheel_checks = {}
        legend = ttk.Frame(self.wheel_pane)
        legend.grid(row=0, column=0, columnspan=2, sticky="ew")
        for key, label in (("ref", "REF"), ("speed", "SPEED"), ("error", "ERROR"), ("pwm", "PWM"), ("count", "ENC")):
            var = tk.BooleanVar(value=True)
            self.wheel_checks[key] = var
            ttk.Checkbutton(legend, text=label, variable=var,
                            command=self._mark_wheels).pack(side="left")
        for i, side in enumerate(("L", "R")):
            frame = ttk.LabelFrame(self.wheel_pane, text="Motor " + ("trái (L)" if i == 0 else "phải (R)"))
            frame.grid(row=1, column=i, sticky="nsew", padx=2)
            canvas = tk.Canvas(frame, bg="#101923", highlightthickness=0, width=230, height=180)
            canvas.pack(fill="both", expand=True)
            canvas.bind("<Configure>", lambda event: self._mark_wheels())
            self.wheel_canvases.append(canvas)
        self.angle_pane = ttk.LabelFrame(self.data_split, text="MPU — Yaw / góc quay Micromouse")
        self.angle_canvas = tk.Canvas(self.angle_pane, bg="#081722", highlightthickness=0, height=260)
        self.angle_canvas.pack(fill="both", expand=True)
        self.angle_canvas.bind("<Configure>", lambda event: self._draw_angle_panel())
        # Header is selected when the first frame identifies ENC/IR/MPU.

        controls = ttk.Frame(right)
        controls.pack(fill="x", pady=(0, 6))
        self.next_btn = ttk.Button(controls, text="Chuyển (KEY ngắn)",
                                   command=lambda: self.send_menu(b"N"), state="disabled")
        self.next_btn.pack(side="left", padx=(0, 6))
        self.select_btn = ttk.Button(controls, text="Chọn (KEY giữ)",
                                     command=lambda: self.send_menu(b"K"), state="disabled")
        self.select_btn.pack(side="left")
        stream_bar = ttk.Frame(right)
        stream_bar.pack(fill="x", pady=(0, 6))
        self.start_test_btn = ttk.Button(stream_bar, text="Start test",
                                         command=lambda: self.send_test_control(b"G"),
                                         state="disabled")
        self.start_test_btn.pack(side="left", padx=(0, 6))
        self.stop_test_btn = ttk.Button(stream_bar, text="Stop test",
                                        command=lambda: self.send_test_control(b"P"),
                                        state="disabled")
        self.stop_test_btn.pack(side="left")
        ttk.Label(stream_bar, text="Pause/resume selected sensor only",
                  style="Muted.TLabel").pack(side="left", padx=(8, 0))
        reset_bar = ttk.Frame(right)
        reset_bar.pack(fill="x", pady=(0, 6))
        self.reset_enc_btn = ttk.Button(reset_bar, text="Reset Encoder",
                                        command=lambda: self.send_sensor_reset(b"E", "Encoder Test"),
                                        state="disabled")
        self.reset_enc_btn.pack(side="left", padx=(0, 6))
        self.reset_yaw_btn = ttk.Button(reset_bar, text="Zero MPU yaw",
                                        command=lambda: self.send_sensor_reset(b"Y", "MPU Test"),
                                        state="disabled")
        self.reset_yaw_btn.pack(side="left", padx=(0, 6))
        self.restart_ir_btn = ttk.Button(reset_bar, text="Restart IR scan",
                                         command=lambda: self.send_sensor_reset(b"I", "IR Test"),
                                         state="disabled")
        self.restart_ir_btn.pack(side="left")
        self.pid_panel = ttk.LabelFrame(right, text="Wheel PID (RAM) — PPS / permille", padding=6)
        self.pid_values = [tk.StringVar(value=x) for x in ("0.09", "0.25", "0", "0.09", "0.25", "0")]
        self.pid_ready = False
        self.pid_pending = False
        self.pid_entries = []
        for col, side in enumerate(("L", "R")):
            ttk.Label(self.pid_panel, text="Motor " + ("trái" if col == 0 else "phải")).grid(row=0, column=col*2, columnspan=2, sticky="w")
            for row, gain in enumerate(("P", "I", "D"), 1):
                ttk.Label(self.pid_panel, text=f"{gain}_{side}").grid(row=row, column=col*2, padx=4, pady=2)
                entry = ttk.Entry(self.pid_panel, textvariable=self.pid_values[col*3+row-1], width=12)
                entry.grid(row=row, column=col*2+1, padx=4, pady=2, sticky="ew")
                self.pid_entries.append(entry)
        self.pid_apply_btn = ttk.Button(self.pid_panel, text="Áp dụng PID", command=self.apply_pid, state="disabled")
        self.pid_apply_btn.grid(row=4, column=0, columnspan=4, pady=4)
        self.pid_note = tk.StringVar(value="Chờ thông số từ STM32")
        ttk.Label(self.pid_panel, textvariable=self.pid_note, wraplength=360).grid(row=5, column=0, columnspan=4, sticky="w")
        status_bar = self.pid_status_bar = ttk.Frame(right)
        status_bar.pack(fill="x", pady=(0, 5))
        ttk.Label(status_bar, text="Filter:").pack(side="left")
        ttk.Entry(status_bar, textvariable=self.filter_text, width=15).pack(side="left", padx=5)
        ttk.Button(status_bar, text="Clear status", command=self.clear_status).pack(side="right")
        self.status_text = self._make_log_text(right, "word")

    def _data_header(self, stream=None):
        columns = TEST_CSV_COLUMNS.get(stream, CSV_COLUMNS)
        self.data_text.configure(state="normal")
        self.data_text.insert("end", ",".join(columns) + "\n")
        self.data_text.configure(state="disabled")

    @staticmethod
    def _make_log_text(parent, wrap):
        frame = ttk.Frame(parent)
        frame.pack(fill="both", expand=True)
        text = tk.Text(frame, wrap=wrap, state="disabled", bg="#101923",
                       fg="#d5e2ef", font=("Consolas", 10), relief="flat")
        scroll = ttk.Scrollbar(frame, orient="vertical", command=text.yview)
        text.configure(yscrollcommand=scroll.set)
        text.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        if wrap == "none":
            horizontal = ttk.Scrollbar(frame, orient="horizontal", command=text.xview)
            text.configure(xscrollcommand=horizontal.set)
            horizontal.grid(row=1, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        text.tag_configure("normal", foreground="#d5e2ef")
        text.tag_configure("error", foreground="#ff8686")
        text.tag_configure("map", foreground="#51d0be")
        return text

    def _maze_widgets(self):
        row = ttk.Frame(self.maze_tab)
        row.pack(fill="x")
        ttk.Label(row, textvariable=self.map_status).pack(side="left")
        ttk.Button(row, text="Reset display", command=self.reset_map).pack(side="right")
        self.map_canvas = tk.Canvas(self.maze_tab, bg="#080e16", highlightthickness=0)
        self.map_canvas.pack(fill="both", expand=True, pady=(8, 0))
        self.map_canvas.bind("<Configure>", lambda event: self._mark_maze())

    def _telemetry_widgets(self):
        controls = ttk.Frame(self.telemetry_tab)
        controls.pack(fill="x")
        ttk.Label(controls, text="Series A:").pack(side="left")
        self.a_box = ttk.Combobox(controls, textvariable=self.plot_a, width=22,
                                  state="readonly")
        self.a_box.pack(side="left", padx=(5, 15))
        ttk.Label(controls, text="Series B:").pack(side="left")
        self.b_box = ttk.Combobox(controls, textvariable=self.plot_b, width=22,
                                  state="readonly")
        self.b_box.pack(side="left", padx=5)
        ttk.Label(controls, text="Last 400 samples · shared Y axis",
                  style="Muted.TLabel").pack(side="right")
        self.plot_canvas = tk.Canvas(self.telemetry_tab, bg="#101923",
                                      highlightthickness=0, height=280)
        self.plot_canvas.pack(fill="both", expand=True, pady=(8, 8))
        self.plot_canvas.bind("<Configure>", lambda event: self._mark_plot())
        fields = ttk.Frame(self.telemetry_tab)
        fields.pack(fill="both", expand=True)
        ttk.Label(fields, text="Latest decoded numeric fields:").pack(anchor="w")
        self.metric_table = ttk.Treeview(fields, columns=("name", "value"),
                                         show="headings", height=9)
        self.metric_table.heading("name", text="Field (units unchanged)")
        self.metric_table.heading("value", text="Latest value")
        self.metric_table.column("name", width=230)
        self.metric_table.column("value", width=150)
        self.metric_table.pack(fill="both", expand=True)

    def refresh_ports(self):
        ports = available_ports()
        choices = [p.device for p in ports]
        self.ports_box.configure(values=choices)
        current = self.port_value.get().upper().strip()
        if not current:
            match = suggested_port(ports)
            if match:
                self.port_value.set(match)
        # Keep manually entered ports; never silently choose among multiple
        # outgoing Bluetooth devices or assume a hard-coded COM number.

    def apply_pid(self):
        if not self.menu_ready or self.active_test_name != "PID" or not self.pid_ready or self.pid_pending:
            return
        limits = (10000, 20000, 1000) * 2
        try:
            values = []
            for var, limit in zip(self.pid_values, limits):
                raw = Decimal(var.get().strip()) * 10000
                if not raw.is_finite() or raw != raw.to_integral_value() or not 0 <= raw <= limit:
                    raise ValueError
                values.append(int(raw))
        except (InvalidOperation, ValueError, OverflowError):
            self.pid_note.set("P: 0..1; I: 0..2; D: 0..0.1; tối đa 4 chữ số thập phân.")
            return
        if self.reader.send_pid(values):
            self.pid_pending = True
            self.pid_note.set("Đã gửi; chờ STM32 xác nhận RAM...")
            self._update_test_controls()
            self.root.after(2000, self._pid_ack_timeout)
        else:
            self.pid_note.set("Không gửi được PID; kiểm tra SPP.")

    def _pid_ack_timeout(self):
        if self.pid_pending:
            self.pid_pending = False
            self.pid_note.set("Chưa nhận ACK; chọn lại PID để đọc RAM hoặc gửi lại.")
            self._update_test_controls()

    def _update_test_controls(self):
        self._sync_cal_panel()
        self._sync_wheel_panel()
        self._sync_angle_panel()
        pid_visible = self.menu_ready and self.active_test_name == "PID"
        if pid_visible:
            if not self.pid_panel.winfo_manager():
                self.pid_panel.pack(fill="x", before=self.pid_status_bar, pady=(0, 6))
        else:
            self.pid_panel.pack_forget()
            self.pid_ready = self.pid_pending = False
        pid_editable = pid_visible and self.pid_ready and not self.pid_pending
        self.pid_apply_btn.configure(state="normal" if pid_editable else "disabled")
        for entry in self.pid_entries:
            entry.configure(state="normal" if pid_editable else "disabled")
        can_control = self.menu_ready and self.active_test_name not in (None, "PID", "Calibration")
        if (self.active_test_name == "1 Box Forward" or (self.active_test_name or "").startswith("Pivot ")):
            # This is an encoder + motor test, not an MPU test.
            can_control = can_control and self.onebox_capable
        self.start_test_btn.configure(
            state="normal" if can_control and not self.test_stream_running
                  and not self.onebox_start_pending else "disabled")
        self.stop_test_btn.configure(
            state="normal" if can_control and (self.test_stream_running or
                  self.onebox_start_pending) else "disabled")
        self.reset_enc_btn.configure(
            state="normal" if can_control and self.active_test_name == "Encoder Test" else "disabled")
        self.reset_yaw_btn.configure(
            state="normal" if can_control and self.active_test_name == "MPU Test"
                  and self.mpu_boot_ok else "disabled")
        self.restart_ir_btn.configure(
            state="normal" if can_control and self.active_test_name == "IR Test" else "disabled")

    def _set_menu_ready(self, ready: bool):
        self.menu_ready = ready
        state = "normal" if ready else "disabled"
        self.next_btn.configure(state=state)
        self.select_btn.configure(state=state)
        if not ready:
            self.wheel_preview = False
            self.angle_preview = False
            self.angle_sample.clear()
            self.angle_last_rx = None
            self.angle_ready = False
            self.wheel_data.clear()
            self.active_test_name = None
            self.test_stream_running = False
            self.onebox_start_pending = False
            self.reader.motion_lease.clear()
            self.onebox_capable = False
            self.mpu_boot_ok = True
            self.cal_review = self.cal_visible = False
            self.cal_rows.clear()
            self.cal_thresholds.clear()
        self._update_test_controls()

    def send_menu(self, command: bytes):
        if command in (b"N", b"K"):
            self.reader.motion_lease.clear()
        if not self.menu_ready or not self.reader.send_menu(command):
            self.status.set("SPP menu unavailable: wait for SPP CONNECTED")

    def send_test_control(self, command: bytes):
        # MCU acknowledgement is authoritative. For one-box, an in-flight
        # Start disables repeat Start clicks and still permits Stop.
        if not self.menu_ready or self.active_test_name in (None, "PID", "Calibration"):
            self.status.set("Select Log Receive, IR/MPU or 1 Box Forward before Start/Stop.")
            return
        if (self.active_test_name == "1 Box Forward" or (self.active_test_name or "").startswith("Pivot ")) and not self.onebox_capable:
            self.status.set("1 Box Forward requires updated unified mode_menu_96m.")
            return
        if (self.active_test_name == "1 Box Forward" or (self.active_test_name or "").startswith("Pivot ")):
            if command == b"G":
                self.reader.motion_lease.set()
            elif command == b"P":
                self.reader.motion_lease.clear()
        if not self.reader.send_menu(command):
            self.reader.motion_lease.clear()
            self.onebox_start_pending = False
            self.status.set("Could not send test Start/Stop; check the SPP connection.")
        elif (self.active_test_name == "1 Box Forward" or (self.active_test_name or "").startswith("Pivot ")):
            self.onebox_start_pending = command == b"G"
            self._update_test_controls()

    def send_sensor_reset(self, command: bytes, expected_test: str):
        if not self.menu_ready or self.active_test_name != expected_test:
            self.status.set("Reset requires the corresponding active sensor test.")
            return
        if command == b"Y" and not self.mpu_boot_ok:
            self.status.set("MPU failed boot: yaw reset is unavailable.")
            return
        if not self.reader.send_menu(command):
            self.status.set("Could not send sensor reset over SPP.")

    def reconnect(self):
        # User-initiated RF/COM cycling. Do not flap COM automatically when
        # MCU is faulted or multiple devices share Windows Bluetooth.
        self.reader.close()
        self._set_menu_ready(False)
        self._reconnect_attempts = 0
        self.status.set("Reopening Outgoing SPP COM...")
        self.root.after(200, self._reconnect_when_closed)

    def _reconnect_when_closed(self):
        if self.reader.thread and self.reader.thread.is_alive():
            self._reconnect_attempts += 1
            if self._reconnect_attempts >= 25:
                self.status.set("COM did not close; disconnect manually and retry.")
                return
            self.root.after(150, self._reconnect_when_closed)
            return
        # Process the closed event before creating a new log session, so an
        # old session close cannot close the new recording.
        self.poll_pending_events()
        self.connect()

    def poll_pending_events(self):
        self._drain_events()

    def connect(self):
        if self.reader.thread and self.reader.thread.is_alive():
            self.status.set("SPP reader active. Disconnect, wait for it to close, then reconnect.")
            return
        self._set_menu_ready(False)
        self.rx_byte_count.set("RX 0 bytes")
        self._last_auto_log = None
        self._logging_disabled = False
        port = self.port_value.get().strip()
        if not port:
            messagebox.showerror("COM port", "Select the Windows SPP outgoing COM port.")
            return
        try:
            baud = int(self.baud_value.get())
            if not 1200 <= baud <= 921600:
                raise ValueError("invalid baud")
        except ValueError:
            messagebox.showerror("Baud", "Invalid baudrate.")
            return
        candidates = [p for p in available_ports() if p.device.lower() == port.lower()]
        if candidates and classify_port(candidates[0].hwid or "")[0] == "incoming":
            messagebox.showerror(
                "Incorrect Bluetooth COM",
                "%s is probably an INCOMING Windows Bluetooth serial port.\n"
                "It cannot initiate SPP to the JDY-33.\n\n"
                "Use Windows > More Bluetooth settings > COM Ports > "
                "Outgoing for Nezumi_Chan_SPP." % port
            )
            return
        if not candidates:
            if not messagebox.askyesno(
                "Unlisted COM", port + " is not in Windows serial port list.\n"
                "Try opening anyway? (COM37 is only a preference)"
            ):
                return
        try:
            self.dual_log.open_session(port)
        except OSError as exc:
            messagebox.showerror("SPP log files", str(exc))
            return
        try:
            self.reader.start(port, baud)
        except Exception:
            self.dual_log.close()
            raise
        self.status.set("Opening " + port + " ...")

    def disconnect(self):
        self._set_menu_ready(False)
        self.reader.close()
        self.status.set("Disconnect requested")

    def _append(self, line, ts, category=None, host_time=None):
        category = category or classify_spp_line(line)
        sensor_record = None
        if category == "data":
            selected = self.data_filter.get()
            if selected != "All" and not line.startswith(selected + ","):
                return
            widget = self.data_text
            sensor_record = sensor_test_csv(line, host_time=host_time)
            stream = sensor_record[0] if sensor_record else "LEGACY"
            if stream != self.active_data_stream:
                # Each test starts a clean CSV table: never mix columns
                # from another sensor into the active view.
                self._clear_log(self.data_text)
                self.active_data_stream = stream
                self._data_header(stream)
        else:
            if self.filter_text.get().lower() not in line.lower():
                return
            widget = self.status_text
        tag = "error" if "FAIL" in line or "ERROR" in line else "normal"
        widget.configure(state="normal")
        display = (",".join(sensor_record[2]) if sensor_record
                   else csv_row_text(data_to_csv_row(line)) if category == "data"
                   else "[%s] %s" % (ts, line))
        widget.insert("end", display + "\n", tag)
        if int(widget.index("end-1c").split(".")[0]) > MAX_TERMINAL_LINES:
            widget.delete("1.0", "251.0")
        widget.see("end")
        widget.configure(state="disabled")

    def handle_line(self, line: str):
        # Unlogged post-calibration dashboard frame. Ignore when another
        # mode owns the screen; never append to a finished calibration CSV.
        if line.startswith("CAL_LIVE,"):
            if self.active_test_name == "Calibration" and self.cal_visible:
                try:
                    fields=dict(part.split("=",1) for part in line.split(",")[1:] if "=" in part)
                    values=tuple(int(fields[k]) for k in ("L0","L45","L90","R0","R45","R90"))
                    if all(0<=v<=4095 for v in values):
                        self.cal_live=values
                        self.cal_live_valid=fields.get("valid")=="1"
                        self.cal_monitoring=True
                except (KeyError,ValueError):
                    self.cal_live_valid=False
            return
        # Retried '?' probes can produce multiple acknowledgements if the
        # firmware was briefly busy. Keep only the first status log entry.
        if line.strip() == "SPP CONNECTED" and self.menu_ready:
            return
        host_time = datetime.now().isoformat(timespec="milliseconds")
        ts = host_time[11:]
        self.total_lines += 1
        self.rx_count.set("%d lines" % self.total_lines)
        category = classify_spp_line(line)
        # An Encoder frame is displayable only while Log Receive was
        # explicitly selected. Ignore any queued frames from old tests.
        if category == "data" and line.startswith("CAL_IR,") and self.active_test_name != "Calibration":
            return
        if category == "data" and line.startswith("PIVOT_CSV,") and (not (self.active_test_name or "").startswith("Pivot ") or not self.test_stream_running):
            return
        if category == "data" and line.startswith("MPU,") and (
                self.active_test_name != "MPU Test" or not self.test_stream_running):
            return
        if category == "data" and line.startswith("ENC,") and (
                self.active_test_name != "Encoder Test" or
                not self.test_stream_running):
            return
        if category == "data" and line.startswith("ONEBOX_CSV,") and (
                self.active_test_name != "1 Box Forward" or
                not self.test_stream_running):
            return
        if not self._replaying and not self._logging_disabled:
            try:
                data_path = self.dual_log.write(line, category, host_time=host_time)
                if data_path is not None and data_path != self._last_auto_log:
                    self._last_auto_log = data_path
                    # UI-only status: the status terminal is NOT saved to disk.
                    self.status.set("Auto-saving Continuous data: " + str(data_path.parent))
            except OSError as exc:
                # Disk permissions/full disk must never break SPP event processing.
                self._logging_disabled = True
                self.status.set("Auto-save failed: %s (reconnect to retry)" % exc)
        # Encoder parent is navigation only; never turn on logging here.
        if line == "Selected mode 1: Calibration":
            self.active_test_name="Calibration"
            self.cal_visible=True
            self.cal_review=False
            self.cal_rows.clear()
            self.cal_thresholds.clear()
            self.cal_wall_state=[False]*6
            self.cal_monitoring=False
            self.cal_live_valid=False
            self.cal_note.set("ARMED — nhả KEY để quay 360° chậm")
            self.test_stream_running=False
            self.data_filter.set("CAL_IR")
            self._clear_log(self.data_text)
            self.active_data_stream=None
            self._update_test_controls()
        elif line.startswith("CAL,START,"):
            self.active_test_name="Calibration"
            self.cal_visible=True
            self.cal_review=False
            self.cal_rows.clear()
            self.cal_thresholds.clear()
            self.cal_wall_state=[False]*6
            self.cal_monitoring=False
            self.cal_note.set("Đang ghi IR · 4 × 90° chậm")
            self.test_stream_running=True
            self.reader.motion_lease.set()
            self._update_test_controls()
        elif line.startswith("CAL,PIVOT_START,"):
            self.cal_note.set("Đang quay: pivot "+line.split("segment=")[-1].split(",")[0]+"/4")
        elif line.startswith("CAL,PIVOT_DONE,"):
            self.cal_note.set("Pivot "+line.split("segment=")[-1].split(",")[0]+"/4 xong, chờ gyro")
        elif line.startswith("CAL,THRESH,"):
            try:
                f=dict(item.split("=",1) for item in line.split(",")[2:] if "=" in item)
                i=int(f["index"])
                if 0<=i<6:
                    self.cal_thresholds[i]=(int(f["off"]),int(f["on"]))
                if len(self.cal_thresholds)==6 and self.dual_log.last_saved_path:
                    from ir_calibration_report import write_ir_report
                    report=write_ir_report(self.dual_log.last_saved_path,self.cal_thresholds)
                    self.status.set("Calibration plots and summary: "+str(report))
            except (KeyError,ValueError,OSError) as exc:
                self.status.set("Calibration report error: "+str(exc))
        elif line.startswith("CAL,REVIEW,"):
            self.reader.motion_lease.clear()
            self.test_stream_running=False
            self.cal_monitoring=True
            self.cal_review="quality=CANDIDATE_ONLY" in line
            self.cal_note.set(line)
            self._update_test_controls()
        elif line.startswith(("CAL,FAULT,","CAL,SAVED,","CAL,SAVE_REJECT,")):
            self.reader.motion_lease.clear()
            self.test_stream_running=False
            self.cal_monitoring=True
            if line.startswith(("CAL,FAULT,","CAL,SAVED,")):self.cal_review=False
            self.cal_note.set(line)
            self._update_test_controls()
        if line.startswith("CAL_IR,") and self.active_test_name=="Calibration":
            rec=sensor_test_csv(line,host_time=host_time)
            if rec:
                self.cal_rows.append(tuple(map(int,rec[2])))
                self.cal_live=tuple(map(int,rec[2][1:]))
                self.cal_live_valid=True
        if line.startswith("Selected mode ") and not line.startswith("Selected mode 1:"):
            self.cal_visible=False
            self.cal_review=False
            self.cal_monitoring=False
            if self.active_test_name=="Calibration":
                self.active_test_name=None
                self._update_test_controls()
        is_encoder_log = line == "Selected encoder option 1: Log Receive"
        is_onebox = line.startswith("Selected encoder option 2: 1 Box Forward")
        is_ir_or_mpu = (line.startswith("Selected test ") and
                         line.split(": ", 1)[-1].strip() in
                         ("IR Test", "MPU Test", "All Sensors Test"))
        is_pivot = line.startswith("Selected mpu option ") and line.partition(": ")[2].startswith("Pivot ")
        if is_pivot:
            self.angle_preview=False
            self.wheel_preview=False
            self.active_test_name=line.partition(": ")[2]
            self.test_stream_running=False
            self.onebox_start_pending=False
            self.reader.motion_lease.clear()
            self.angle_ready=False
            self.data_filter.set("All")
            self.clear_data()
            self._update_test_controls()
        elif line == "Selected encoder option 8: PID":
            self.angle_preview = False
            self.wheel_preview = False
            self.active_test_name = "PID"
            self.test_stream_running = False
            self.onebox_start_pending = False
            self.reader.motion_lease.clear()
            self.pid_ready = self.pid_pending = False
            self.pid_note.set("Chờ đọc hệ số đang lưu trong RAM...")
            self._update_test_controls()
        elif line.startswith(("PID,VALUES,", "PID,APPLIED,")) and self.active_test_name == "PID":
            try:
                fields = line.split(",")
                vals = [int(x) for x in fields[2:]]
                if len(vals) != 6 or any(not 0 <= v <= m for v, m in zip(vals, (10000,20000,1000)*2)):
                    raise ValueError
                for var, value in zip(self.pid_values, vals):
                    var.set(f"{value / 10000:.4f}")
                self.pid_ready = True
                self.pid_pending = False
                self.pid_note.set("Đã áp dụng vào RAM STM32" if fields[1] == "APPLIED" else "Đã đọc PID từ RAM STM32")
                self._update_test_controls()
            except ValueError:
                self.pid_note.set("Bản tin PID không hợp lệ.")
        elif line.startswith("PID,REJECT,") and self.active_test_name == "PID":
            self.pid_pending = False
            self.pid_note.set("STM32 từ chối PID: kiểm tra mode, giới hạn hoặc trạng thái motor.")
            self._update_test_controls()
        elif is_encoder_log or is_onebox or is_ir_or_mpu:
            self.angle_preview = False
            self.angle_sample.clear()
            self.angle_last_rx = None
            self.angle_ready = False
            self.wheel_preview = False
            self.wheel_data.clear()
            self.wheel_dirty = True
            self.active_test_name = ("Encoder Test" if is_encoder_log else
                                     "1 Box Forward" if is_onebox else
                                     line.split(": ", 1)[-1].strip())
            # Log Receive and 1 Box Forward arm IDLE: no data/motion yet.
            self.test_stream_running = not (is_encoder_log or is_onebox or self.active_test_name == "MPU Test")
            self.onebox_start_pending = False
            self.reader.motion_lease.clear()
            self.data_filter.set("All")
            self._clear_log(self.data_text)
            self.active_data_stream = None
            self.metrics.clear()
            self.series.clear()
            self.plot_dirty = True
            self._update_test_controls()
        elif (line.startswith("MPU Test:") or line.startswith("MPU Test option ") or
              line.startswith("Encoder Test") or
              line.startswith("Selected encoder option ") or
              line.startswith("Selected test 2: Encoder Test") or
              line.startswith("System Test ") or
              line.startswith("Selected mode 7: System Test")):
            # Browsing or selecting a movement placeholder stops Encoder
            # streaming. Neither status messages nor motion placeholders
            # send any motor commands or create a CSV.
            self.angle_preview = False
            self.active_test_name = None
            self.test_stream_running = False
            self.onebox_start_pending = False
            self.reader.motion_lease.clear()
            self._update_test_controls()
            self.clear_data()
            self.metrics.clear()
            self.series.clear()
            self.plot_dirty = True
        elif line == "ONEBOX,PROFILE,ENABLED":
            self.onebox_capable = True
            self._update_test_controls()
            self.status.set("1 Box Forward firmware enabled: supervised motion available.")
        elif line == "ONEBOX,PROFILE,MOTOR_LOCKED":
            self.onebox_capable = False
            self.reader.motion_lease.clear()
            self._update_test_controls()
            self.status.set("Legacy motor-locked firmware detected; update unified mode_menu_96m.")
        elif line.startswith(("ONEBOX,START,","PIVOT,START,")):
            if (self.active_test_name == "1 Box Forward" or (self.active_test_name or "").startswith("Pivot ")):
                self.wheel_data.clear()
                self.wheel_dirty = True
                self.clear_data()
                self.onebox_start_pending = False
                self.test_stream_running = True
                self.reader.motion_lease.set()
                self._update_test_controls()
                self.status.set(self.active_test_name + " running (fixed speed PID, max 20% PWM).")
        elif line.startswith(("ONEBOX,RESULT,", "ONEBOX,REJECT,", "PIVOT,RESULT,", "PIVOT,REJECT,")) and not (self.active_test_name == "Calibration" and line.startswith(("PIVOT,RESULT,", "PIVOT,REJECT,"))):
            self.reader.motion_lease.clear()
            self.onebox_start_pending = False
            # Driver is fail-locked on severe faults until STM32 reset.
            if line.startswith(("ONEBOX,RESULT,","PIVOT,RESULT,")) and not (
                    "reason=DONE" in line or "reason=STOP" in line):
                self.onebox_capable = False
            if (self.active_test_name == "1 Box Forward" or (self.active_test_name or "").startswith("Pivot ")):
                self.test_stream_running = False
                self._update_test_controls()
            saved = self.dual_log.last_saved_path
            if line.startswith("ONEBOX,RESULT,") and (self.active_test_name == "1 Box Forward" or (self.active_test_name or "").startswith("Pivot ")):
                metrics = summarize_wheels(self.wheel_data.rows, 200)
                self._append("WHEEL METRICS (applied PWM, sampled): " + str(metrics), ts, "status")
                if saved and saved.parent.name == "mode 7 - encoder test - 1 box forward":
                    try:
                        save_wheel_summary(saved, metrics, line)
                    except OSError as exc:
                        self._append("METRICS SAVE ERROR: " + str(exc), ts, "status")
            note = (" | CSV: " + str(saved)
                    if line.startswith("ONEBOX,RESULT,") and saved and
                    saved.parent.name == "mode 7 - encoder test - 1 box forward"
                    else "")
            if line.startswith("PIVOT,RESULT,") and saved:
                note = " | CSV: " + str(saved)
                if self.angle_sample.valid:
                    note += f" | Relative yaw: {self.angle_sample.yaw:+.2f} deg"
            self.status.set(
                line + note + (" — reset STM32 before another motor test"
                               if line.startswith(("ONEBOX,RESULT,","PIVOT,RESULT,")) and
                               not self.onebox_capable else ""))
        elif line.startswith("TEST STREAM STOPPED: "):
            if line.partition(": ")[2].strip() == self.active_test_name:
                self.test_stream_running = False
                self._update_test_controls()
                if self.active_test_name == "Encoder Test":
                    saved = self.dual_log.last_saved_path
                    self.status.set("Encoder log saved: " + str(saved) if saved else
                                    "Encoder log stopped (no file opened).")
                else:
                    self.status.set("Test paused: " + self.active_test_name)
        elif line.startswith("TEST STREAM STARTED: "):
            if line.partition(": ")[2].strip() == self.active_test_name:
                self.test_stream_running = True
                if self.active_test_name == "MPU Test":
                    self.angle_preview = False
                    self.clear_data()
                self._update_test_controls()
                if self.active_test_name == "Encoder Test":
                    # A Start creates a new take, never appends to the last CSV.
                    self.clear_data()
                    self.metrics.clear()
                    self.series.clear()
                    self.plot_dirty = True
                    self.status.set("Encoder logging started: new timestamped CSV.")
                else:
                    self.status.set("Test resumed: " + self.active_test_name)
        elif line.startswith("BOOT,DEGRADED,"):
            self.mpu_boot_ok = False
            self._update_test_controls()
            self.status.set("SPP OK; STM32 MPU boot degraded. Encoder/IR tests remain available.")
        elif line.startswith("BOOT,READY,"):
            self.mpu_boot_ok = True
            self._update_test_controls()
        elif line.startswith("TEST,MPU,READY,"):
            self.angle_ready = True
        elif line.startswith(("TEST,MPU,CALIB_RESTART", "TEST,MPU,CAL,")):
            self.angle_ready = False
        elif line == "MPU YAW RESET OK":
            self.angle_sample.clear()  # wait for the next measured sample
            self.angle_last_rx = None
            self.status.set(line)
        elif (line.startswith("ENCODER RESET OK") or
              line.startswith("MPU YAW RESET") or
              line.startswith("IR SCAN RESTARTED")):
            self.status.set(line)
        if category == "data" and (self.active_test_name == "MPU Test" or (self.active_test_name or "").startswith("Pivot ")) and self.angle_sample.consume(line):
            self.angle_last_rx = time.monotonic()
        if category == "data" and self.wheel_data.add(line, host_time):
            self.wheel_dirty = True
        self._append(line, ts, category, host_time=host_time)
        changed = self.maze.apply(line)
        if changed:
            self.map_status.set(changed)
            self.maze_dirty = True
        values = numeric_fields(line)
        sensor_record = sensor_test_csv(line, host_time=host_time)
        if sensor_record and sensor_record[0] == "ENC":
            values = {"ENC_L": float(sensor_record[2][1]),
                      "ENC_R": float(sensor_record[2][2])}
        if line.startswith("W,"):
            items = line.split(",")
            if len(items) == 10:
                try:
                    nums = [int(x) for x in items[1:]]
                    names = ("ms", "distance_0p1mm", "l0", "r0",
                             "wall_error_norm_1000", "speed_mm_s",
                             "setpoint_mm_s", "pwmL", "pwmR")
                    values = dict(zip(names, map(float, nums)))
                except ValueError:
                    pass
        if values:
            self.metrics.update(values)
            for name, value in values.items():
                self.series[name].append(value)
            self.samples += 1
            self.plot_dirty = True

    def _drain_events(self):
        for _ in range(160):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                self.handle_line(value)
                if value.strip() == "SPP CONNECTED":
                    self._set_menu_ready(True)
                    self.status.set("SPP CONNECTED: " + self.port_value.get().strip())
            else:
                if kind == "rx_bytes":
                    self.rx_byte_count.set("RX %d bytes" % value)
                    continue
                self.status.set(value)
                if kind == "closed":
                    self._set_menu_ready(False)
                    self.dual_log.close()
                if kind == "warning":
                    self._append("WARNING: " + value, datetime.now().strftime("%H:%M:%S"), "status")

    def poll(self):
        self._drain_events()
        self.root.after(40, self.poll)

    def redraw(self):
        now = time.monotonic()
        if self.maze_dirty:
            self.draw_maze()
            self.maze_dirty = False
        if self.plot_dirty and now - self.last_plot > 0.14:
            self.draw_plot()
            self.last_plot = now
            self.plot_dirty = False
        if self.wheel_dirty:
            self.draw_wheels()
            self.wheel_dirty = False
        self._draw_angle_panel()
        if self.cal_visible:self._draw_calibration_panel()
        self.root.after(150, self.redraw)

    def _sync_cal_panel(self):
        show=self.cal_visible and self.menu_ready and self.active_test_name=="Calibration"
        present=str(self.cal_pane) in self.data_split.panes()
        if show and not present:self.data_split.add(self.cal_pane,weight=1)
        elif present and not show:self.data_split.forget(self.cal_pane)
        self.cal_save_btn.configure(state="normal" if show and self.cal_review else "disabled")

    def _draw_calibration_panel(self):
        c=self.cal_canvas
        c.delete("all")
        w=max(c.winfo_width(),340)
        h=max(c.winfo_height(),230)
        names=("L0","L45","L90","R0","R45","R90")
        current=self.cal_live
        if not self.cal_monitoring and self.cal_rows:
            current=self.cal_rows[-1][1:]
        invalid=self.cal_monitoring and not self.cal_live_valid
        top=47
        bottom=max(top+80,h-48)
        span=bottom-top
        step=(w-38)/6
        status=("LIVE · MOTOR STOPPED" if self.cal_monitoring else "CAPTURING 360°" if self.cal_rows else "WAITING")
        c.create_text(12,8,anchor="nw",text="6 IR CHANNELS · LIVE ADC (0–4095)",
                      fill="#d9e9f6",font=("Segoe UI",10,"bold"))
        c.create_text(w-12,9,anchor="ne",text=status,
                      fill="#4ee2ad" if self.cal_monitoring else "#f2c66d",
                      font=("Segoe UI",9,"bold"))
        c.create_text(12,26,anchor="nw",
                      text="BLUE = WALL  |  GREY = NO WALL  |  YELLOW = ON threshold",
                      fill="#9babc1",font=("Segoe UI",8))
        for value in (0,1024,2048,3072,4095):
            y=bottom-(value/4095)*span
            c.create_line(30,y,w-8,y,fill="#29384b",dash=(2,5))
            c.create_text(27,y,anchor="e",text=str(value),fill="#8397aa",
                          font=("Consolas",7))
        for i,name in enumerate(names):
            x=34+i*step
            value=int(current[i])
            threshold=self.cal_thresholds.get(i)
            if threshold and not invalid:
                off,on=threshold
                if value>=on:self.cal_wall_state[i]=True
                elif value<=off:self.cal_wall_state[i]=False
            if invalid:self.cal_wall_state[i]=False
            detected=bool(threshold and self.cal_wall_state[i] and not invalid)
            color="#18c7f0" if detected else "#788696"
            if invalid:
                color="#465364"
            bar_width=max(13,step-14)
            bar_height=value/4095*span
            c.create_rectangle(x+5,bottom-bar_height,x+5+bar_width,bottom,
                               fill=color,outline="")
            if threshold:
                off,on=threshold
                y=bottom-(on/4095)*span
                c.create_line(x+3,y,x+7+bar_width,y,fill="#f6c847",width=2)
            c.create_text(x+5+bar_width/2,bottom+11,text=name,fill="#e9f2ff",
                          font=("Consolas",9,"bold"))
            c.create_text(x+5+bar_width/2,bottom+29,text=str(value),fill=color,
                          font=("Consolas",9))
        if invalid:
            c.create_text(w/2,round((top+bottom)/2),text="IR INVALID · CHECK SCAN",
                          fill="#ffb86c",font=("Segoe UI",12,"bold"))

    def _mark_wheels(self):
        self.wheel_dirty = True

    def _sync_wheel_panel(self):
        visible = not self.angle_preview and (self.wheel_preview or (self.menu_ready and
                   self.active_test_name in ("Encoder Test", "1 Box Forward")))
        shown = str(self.wheel_pane) in self.data_split.panes()
        if visible and not shown:
            self.data_split.add(self.wheel_pane, weight=1)
            self.root.after_idle(self._balance_wheel_panes)
        elif shown and not visible:
            self.data_split.forget(self.wheel_pane)
        self.wheel_dirty = True

    def _draw_angle_panel(self):
        now = time.monotonic()
        if str(self.angle_pane) in self.data_split.panes():
            if self.angle_preview:
                state = "REPLAY"
            elif not self.test_stream_running:
                state = "PAUSED" if self.angle_sample.valid else ("READY — START" if self.angle_ready else "WAIT FOR MPU")
            elif self.angle_last_rx is None or now - self.angle_last_rx > .5:
                state = "NO FRESH TELEMETRY"
                self.angle_sample.clear()
            else:
                state = "LIVE" if self.angle_sample.valid else "INVALID SAMPLE"
            draw_angle(self.angle_canvas, self.angle_sample, state)

    def _sync_angle_panel(self):
        visible = self.angle_preview or (not self.wheel_preview and self.menu_ready and (self.active_test_name == "MPU Test" or (self.active_test_name or "").startswith("Pivot ")))
        shown = str(self.angle_pane) in self.data_split.panes()
        if visible and not shown:
            self.data_split.add(self.angle_pane, weight=1)
            self.root.after_idle(self._balance_angle_panes)
            self.root.after_idle(self._draw_angle_panel)
        elif shown and not visible:
            self.data_split.forget(self.angle_pane)

    def _balance_angle_panes(self):
        if str(self.angle_pane) in self.data_split.panes():
            self.data_split.sashpos(0, self.data_split.winfo_height() // 2)

    def _balance_wheel_panes(self):
        if str(self.wheel_pane) in self.data_split.panes():
            self.data_split.sashpos(0, self.data_split.winfo_height() // 2)

    def draw_wheels(self):
        rows = list(self.wheel_data.rows)
        colors = {"ref":"#ffcf65", "speed":"#58c9ff", "error":"#ff8181",
                  "pwm":"#71dfa1", "count":"#c5a1ff"}
        keys = [k for k, var in self.wheel_checks.items() if var.get()]
        for c, side in zip(self.wheel_canvases, ("L", "R")):
            c.delete("all")
            w, h = c.winfo_width(), c.winfo_height()
            if w < 100 or h < 90:
                continue
            if not rows:
                c.create_text(w/2,h/2,text="Waiting for encoder data",fill="#91a3b5")
                continue
            labels = {"ref": "REF_"+side+"_PPS", "speed":"SPEED_"+side+"_PPS",
                      "error":"ERR_"+side+"_PPS", "pwm":"PWM_"+side,
                      "count":"ENC_"+side}
            groups = [(unit, [k for k in ks if k in keys and
                        any(row.get(k+"_"+side) is not None for row in rows)])
                      for unit, ks in (("PPS", ("ref","speed","error")),
                                       ("PWM ‰", ("pwm",)), ("count", ("count",)))]
            groups = [(unit,ks) for unit,ks in groups if ks]
            if not groups:
                c.create_text(w/2,h/2,text="No selected signals available",fill="#91a3b5")
                continue
            t0, t1 = rows[0]["t"], rows[-1]["t"]
            span = max(0.05, t1-t0)
            x0, x1 = 46, w-8
            band = (h-24)/len(groups)
            for index,(unit,ks) in enumerate(groups):
                top, bottom = index*band+24, (index+1)*band-4
                vals = [row[k+"_"+s] for row in rows for k in ks for s in ("L","R")
                        if row.get(k+"_"+s) is not None]
                lo, hi = min(0,min(vals)), max(0,max(vals))
                if hi==lo: hi=lo+1
                if unit=="PWM ‰": hi=max(200,hi)
                if unit=="PPS": hi=max(1100,hi)
                c.create_text(3,top-15,text=unit,anchor="nw",fill="#abc0d0",font=("Consolas",8))
                c.create_rectangle(x0,top,x1,bottom,outline="#405063")
                for frac in (0,0.5,1):
                    y=bottom-frac*(bottom-top)
                    c.create_line(x0,y,x1,y,fill="#273747",dash=(2,4))
                    c.create_text(x0-3,y,text="%.0f"%(lo+frac*(hi-lo)),anchor="e",fill="#abc0d0",font=("Consolas",8))
                for k in ks:
                    coords=[]
                    last=None
                    for row in rows:
                        v=row.get(k+"_"+side)
                        if v is None:
                            if len(coords)>=4: c.create_line(*coords,fill=colors[k],width=2)
                            coords=[]
                            continue
                        x=x0+(row["t"]-t0)/span*(x1-x0)
                        y=bottom-(v-lo)/(hi-lo)*(bottom-top)
                        coords.extend((x,y))
                        last=v
                    if len(coords)>=4: c.create_line(*coords,fill=colors[k],width=2,dash=(4,3) if k=="ref" else ())
                    elif len(coords)==2: c.create_oval(coords[0]-2,coords[1]-2,coords[0]+2,coords[1]+2,fill=colors[k],outline="")
                # Compact color legend, with full signal names available in the CSV.
                c.create_text(x0,top-3,text=" / ".join(k.upper() for k in ks),anchor="sw",fill="#c6d6e4",font=("Consolas",8))
            c.create_text(x0,h-8,text=f"{t0:.2f}s",anchor="w",fill="#abc0d0",font=("Consolas",8))
            c.create_text(x1,h-8,text=f"{t1:.2f}s",anchor="e",fill="#abc0d0",font=("Consolas",8))

    def _mark_maze(self):
        self.maze_dirty = True

    def _mark_plot(self):
        self.plot_dirty = True

    def reset_map(self):
        self.maze = MazeState()
        self.map_status.set(self.maze.status)
        self.maze_dirty = True

    def draw_maze(self):
        c, m = self.map_canvas, self.maze
        c.delete("all")
        width, height = c.winfo_width(), c.winfo_height()
        side = min(width - 40, height - 40)
        if side <= 20:
            return
        step = side / m.n
        sx = (width - side) / 2
        sy = (height - side) / 2
        goal = {m.n // 2 - 1, m.n // 2} if m.n % 2 == 0 else {m.n // 2}
        for y in range(m.n):
            for x in range(m.n):
                left = sx + x * step
                top = sy + (m.n - y - 1) * step
                fill = ("#24462e" if x in goal and y in goal
                        else "#1c3a43" if (x, y) in m.visited else "#151e2a")
                c.create_rectangle(left, top, left + step, top + step,
                                   fill=fill, outline="#2b3542", width=1)
                w = m.walls[y][x]
                line = {"fill": "#ff6767", "width": max(2, int(step / 8))}
                if w & 1:
                    c.create_line(left, top, left + step, top, **line)
                if w & 2:
                    c.create_line(left + step, top, left + step, top + step, **line)
                if w & 4:
                    c.create_line(left, top + step, left + step, top + step, **line)
                if w & 8:
                    c.create_line(left, top, left, top + step, **line)
        x, y = sx + (m.x + 0.5) * step, sy + (m.n - m.y - 0.5) * step
        r = max(4, step * 0.27)
        c.create_oval(x - r, y - r, x + r, y + r,
                      fill="#45a5ff", outline="#bbdfff", width=2)
        dx, dy = ((0, -1), (1, 0), (0, 1), (-1, 0))[m.direction]
        c.create_line(x, y, x + dx * r * 1.3, y + dy * r * 1.3,
                      fill="white", width=2, arrow="last")
        c.create_text(8, 7, text="(%d,%d) %s · %d×%d" %
                      (m.x, m.y, "NESW"[m.direction], m.n, m.n),
                      anchor="nw", fill="#c6d6e4", font=("Consolas", 10))

    def draw_plot(self):
        keys = sorted(self.series)
        options = keys or ["l0", "r0"]
        self.a_box.configure(values=options)
        self.b_box.configure(values=options)
        c = self.plot_canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 100 or h < 80:
            return
        pad_left, pad_right, pad_top, pad_bottom = 75, 30, 20, 35
        c.create_rectangle(pad_left, pad_top, w - pad_right, h - pad_bottom,
                           outline="#506173", width=1)
        selected = [self.plot_a.get(), self.plot_b.get()]
        streams = [list(self.series.get(key, ())) for key in selected]
        all_values = [v for stream in streams for v in stream]
        if not all_values:
            c.create_text(w / 2, h / 2, text="Awaiting numeric telemetry",
                          fill="#91a3b5")
            return
        lo, hi = min(all_values), max(all_values)
        if hi == lo:
            lo -= 1.0
            hi += 1.0
        for i in range(5):
            yy = pad_top + (h - pad_top - pad_bottom) * i / 4
            c.create_line(pad_left, yy, w - pad_right, yy,
                          fill="#273747", dash=(2, 5))
            c.create_text(pad_left - 9, yy, text="%.2f" % (hi - (hi-lo)*i/4),
                          anchor="e", fill="#abc0d0", font=("Consolas", 9))
        for i, (key, stream) in enumerate(zip(selected, streams)):
            if not stream:
                continue
            coords = []
            for j, value in enumerate(stream):
                xx = pad_left + (w - pad_left - pad_right) * (
                    j / max(1, len(stream) - 1))
                yy = pad_top + (h - pad_top - pad_bottom) * (hi - value) / (hi - lo)
                coords.extend((xx, yy))
            if len(coords) > 2:
                c.create_line(*coords, fill=PLOT_COLORS[i], width=2)
            c.create_text(pad_left + i * 230, h - 12,
                          text="%s = %.3f" % (key, stream[-1]), anchor="w",
                          fill=PLOT_COLORS[i], font=("Consolas", 10))

        for item in self.metric_table.get_children():
            self.metric_table.delete(item)
        for name in sorted(self.metrics):
            self.metric_table.insert("", "end", values=(name, "%.5g" % self.metrics[name]))

    @staticmethod
    def _clear_log(widget):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.configure(state="disabled")

    def clear_data(self):
        self.angle_sample.clear()
        self.angle_last_rx = None
        self.wheel_data.clear()
        self.wheel_dirty = True
        self._clear_log(self.data_text)
        self.active_data_stream = None

    def clear_status(self):
        self._clear_log(self.status_text)

    def clear_view(self):
        self.clear_data()
        self.clear_status()

    def open_logs_folder(self):
        """Show the per-test auto-save directory (no duplicate raw log)."""
        root = self.dual_log.root
        root.mkdir(parents=True, exist_ok=True)
        try:
            if hasattr(os, "startfile"):
                os.startfile(str(root))
            else:
                self.status.set("Log directory: " + str(root))
        except OSError as exc:
            self.status.set("Cannot open log directory: " + str(exc))

    def open_log(self):
        """Replay new sensor CSV or older line-oriented logs to the left view.

        Playback never feeds the automatic logger and never duplicates files.
        """
        path = filedialog.askopenfilename(
            title="Replay Continuous data CSV or historical SPP log",
            filetypes=[("Sensor CSV", "*.csv"), ("Historical log", "*.log *.txt"),
                       ("All files", "*.*")])
        if not path:
            return
        if self.test_stream_running or self.onebox_start_pending:
            messagebox.showwarning("Replay", "Stop the live test before replaying CSV.")
            return
        self.wheel_preview = False
        self.angle_preview = False
        self.clear_data()
        count = 0
        try:
            if Path(path).suffix.lower() == ".csv":
                with open(path, "r", encoding="utf-8-sig", newline="") as file:
                    reader = csv.DictReader(file)
                    columns = set(reader.fieldnames or [])
                    if {"angle_deg","target_deg","yaw_deg","ENC_L","ENC_R"} <= columns:
                        mode = "PIVOT"
                    elif {"timestamp", "t_ms", "ENC_L", "ENC_R",
                            "PWM_L", "PWM_R"} <= columns:
                        mode = "ONEBOX"
                    elif {"timestamp", "ENC_L", "ENC_R", "ir_valid",
                            "mpu_valid", "L0", "R0"} <= columns:
                        mode = "ALL"
                    elif {"timestamp", "ENC_L", "ENC_R"} <= columns:
                        mode = "ENC"
                    elif {"valid", "L0", "L45", "L90", "R0", "R45", "R90"} <= columns:
                        mode = "IR"
                    elif {"valid", "yaw_deg"} <= columns or {"valid", "yaw_cd", "gz_cd"} <= columns:
                        mode = "MPU"
                    else:
                        messagebox.showerror(
                            "Replay CSV", "Not a recognized System Test CSV.")
                        return
                    self.data_filter.set("All")
                    self.clear_data()
                    for row in reader:
                        try:
                            if mode == "PIVOT":
                                keys=("mcu_ms","valid","age_us","target_deg","ENC_L","ENC_R","PWM_L","PWM_R","REF_L_PPS","REF_R_PPS","SPEED_L_PPS","SPEED_R_PPS")
                                names=("ms","valid","age_us","target","left","right","pwm_l","pwm_r","ref_l","ref_r","speed_l","speed_r")
                                line="PIVOT_CSV,"+",".join(f"{k}={int(row[col])}" for k,col in zip(names,keys))
                                for k,col in (("yaw_cd","yaw_deg"),("angle_cd","angle_deg"),("gz_cd","gz_dps")):
                                    line+=f",{k}={round(float(row[col])*100) if row.get(col) else 0}"
                            elif mode == "ONEBOX":
                                line = ("ONEBOX_CSV,ms=%d,left=%d,right=%d,"
                                        "pwm_l=%d,pwm_r=%d" %
                                        (int(row["t_ms"]), int(row["ENC_L"]),
                                         int(row["ENC_R"]), int(row["PWM_L"]),
                                         int(row["PWM_R"])))
                                speed_cols = ("REF_L_PPS", "REF_R_PPS", "SPEED_L_PPS", "SPEED_R_PPS")
                                if all(row.get(key) not in (None, "") for key in speed_cols):
                                    line += "," + ",".join(f"{key}={int(row[col])}" for key, col in zip(("ref_l", "ref_r", "speed_l", "speed_r"), speed_cols))
                            elif mode == "ALL":
                                keys = ("ir_valid", "L0", "L45", "L90",
                                        "R0", "R45", "R90", "mpu_valid",
                                        "yaw_cd", "gz_cd")
                                line = ("ALL,enc_l=%d,enc_r=%d," %
                                        (int(row["ENC_L"]), int(row["ENC_R"])))
                                line += ",".join(f"{key}={int(row[key])}" for key in keys)
                            elif mode == "ENC":
                                line = f"ENC,{int(row['ENC_L'])},{int(row['ENC_R'])}"
                            elif mode == "IR":
                                keys = ("valid", "L0", "L45", "L90", "R0", "R45", "R90")
                                line = "IR," + ",".join(f"{key}={int(row[key])}" for key in keys)
                            else:
                                line = f"MPU,valid={int(row['valid'])}"
                                for key, col in (("ms", "mcu_ms"), ("age_us", "age_us")):
                                    if row.get(col):
                                        line += f",{key}={int(row[col])}"
                                if row["valid"] == "1":
                                    if "yaw_deg" in row:
                                        line += f",yaw_cd={round(float(row['yaw_deg'])*100)}"
                                    else:
                                        line += (f",yaw_cd={int(row['yaw_cd'])},"
                                                 f"gz_cd={int(row['gz_cd'])}")
                            timestamp = row.get("timestamp") or datetime.now().isoformat(
                                timespec="milliseconds")
                            if mode in ("MPU","PIVOT") and self.angle_sample.consume(line):
                                self.angle_preview = True
                            if self.wheel_data.add(line, timestamp):
                                self.wheel_preview = True
                                self.wheel_dirty = True
                            self._append(line, timestamp[11:23], "data", host_time=timestamp)
                            count += 1
                        except (ValueError, TypeError, KeyError):
                            continue
            else:
                with open(path, "r", encoding="utf-8", errors="replace") as file:
                    for raw in file:
                        text = raw.strip()
                        if len(text) > 24 and text[4:5] == "-" and text[10:11] == "T":
                            text = text.split(" ", 1)[-1]
                        if text:
                            self._append(text, datetime.now().strftime("%H:%M:%S.%f")[:-3])
                            count += 1
        except OSError as exc:
            messagebox.showerror("Replay", str(exc))
            return
        self._sync_wheel_panel()
        self._sync_angle_panel()
        self.status.set("Replayed %d rows from %s (not saved)" %
                        (count, Path(path).name))

    def quit(self):
        self.reader.close()
        self.dual_log.close()
        self.root.destroy()


def main():
    arg = argparse.ArgumentParser(description=__doc__)
    arg.add_argument("--port", default=PREFERRED_PORT,
                     help="Windows outgoing SPP COM; auto-select only when unambiguous")
    args = arg.parse_args()
    root = tk.Tk()
    Dashboard(root, args.port)
    root.mainloop()


if __name__ == "__main__":
    main()
