#!/usr/bin/env python3
"""JDY-33 Classic SPP dashboard with safe menu Next/Select commands.

Only '?' link probe and 'N'/'K' menu navigation can be sent.
No motor/PID/Flash commands. Web BLE remains untouched.
Run: python tools/classic_spp_dashboard.py --port COM37
"""
from __future__ import annotations

import argparse
import csv
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
        if command not in (b"N", b"K", b"P", b"G", b"E", b"Y", b"I") or not self.thread or not self.thread.is_alive():
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
                  "No motor or Flash commands.",
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
                     values=("All", "MPU", "IR", "ENC", "ENC_CSV", "CSV", "DATA"),
                     state="readonly").pack(side="left", padx=5)
        ttk.Button(data_bar, text="Clear data", command=self.clear_data).pack(side="right")
        self.data_text = self._make_log_text(left, "none")
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
        status_bar = ttk.Frame(right)
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

    def _update_test_controls(self):
        can_control = self.menu_ready and self.active_test_name is not None
        if self.active_test_name == "1 Box Forward":
            can_control = can_control and self.onebox_capable and self.mpu_boot_ok
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
            self.active_test_name = None
            self.test_stream_running = False
            self.onebox_start_pending = False
            self.reader.motion_lease.clear()
            self.onebox_capable = False
            self.mpu_boot_ok = True
        self._update_test_controls()

    def send_menu(self, command: bytes):
        if command in (b"N", b"K"):
            self.reader.motion_lease.clear()
        if not self.menu_ready or not self.reader.send_menu(command):
            self.status.set("SPP menu unavailable: wait for SPP CONNECTED")

    def send_test_control(self, command: bytes):
        # MCU acknowledgement is authoritative. For one-box, an in-flight
        # Start disables repeat Start clicks and still permits Stop.
        if not self.menu_ready or not self.active_test_name:
            self.status.set("Select Log Receive, IR/MPU or 1 Box Forward before Start/Stop.")
            return
        if self.active_test_name == "1 Box Forward" and (
                not self.onebox_capable or not self.mpu_boot_ok):
            self.status.set("1 Box Forward needs motion-enabled firmware and ready MPU.")
            return
        if self.active_test_name == "1 Box Forward":
            if command == b"G":
                self.reader.motion_lease.set()
            elif command == b"P":
                self.reader.motion_lease.clear()
        if not self.reader.send_menu(command):
            self.reader.motion_lease.clear()
            self.onebox_start_pending = False
            self.status.set("Could not send test Start/Stop; check the SPP connection.")
        elif self.active_test_name == "1 Box Forward":
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
        if category == "data" and line.startswith("ENC,") and (
                self.active_test_name != "Encoder Test" or
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
        is_encoder_log = line == "Selected encoder option 1: Log Receive"
        is_onebox = line.startswith("Selected encoder option 2: 1 Box Forward")
        is_ir_or_mpu = (line.startswith("Selected test ") and
                         line.split(": ", 1)[-1].strip() in ("IR Test", "MPU Test"))
        if is_encoder_log or is_onebox or is_ir_or_mpu:
            self.active_test_name = ("Encoder Test" if is_encoder_log else
                                     "1 Box Forward" if is_onebox else
                                     line.split(": ", 1)[-1].strip())
            # Log Receive and 1 Box Forward arm IDLE: no data/motion yet.
            self.test_stream_running = not (is_encoder_log or is_onebox)
            self.onebox_start_pending = False
            self.reader.motion_lease.clear()
            self.data_filter.set("All")
            self._clear_log(self.data_text)
            self.active_data_stream = None
            self.metrics.clear()
            self.series.clear()
            self.plot_dirty = True
            self._update_test_controls()
        elif (line.startswith("Encoder Test") or
              line.startswith("Selected encoder option ") or
              line.startswith("Selected test 2: Encoder Test") or
              line.startswith("System Test ") or
              line.startswith("Selected mode 7: System Test") or
              line.startswith("Selected mode 9: System Test") or
              line.startswith("System Test:") or
              line.endswith(": submenu opened") or
              " option " in line and not line.startswith("Selected ")):
            # Browsing or selecting a movement placeholder stops Encoder
            # streaming. Neither status messages nor motion placeholders
            # send any motor commands or create a CSV.
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
            self.status.set("Motor-locked firmware: use the unified micromouse_96m build.")
        elif line.startswith("ONEBOX,START,"):
            if self.active_test_name == "1 Box Forward":
                self.onebox_start_pending = False
                self.test_stream_running = True
                self.reader.motion_lease.set()
                self._update_test_controls()
                self.status.set("1 Box Forward running (one-shot, max 26% PWM).")
        elif line.startswith(("ONEBOX,RESULT,", "ONEBOX,REJECT,")):
            self.reader.motion_lease.clear()
            self.onebox_start_pending = False
            # Driver is fail-locked on severe faults until STM32 reset.
            if line.startswith("ONEBOX,RESULT,") and not (
                    "reason=DONE" in line or "reason=STOP" in line):
                self.onebox_capable = False
            if self.active_test_name == "1 Box Forward":
                self.test_stream_running = False
                self._update_test_controls()
            self.status.set(
                line + (" — reset STM32 before another motor test"
                        if line.startswith("ONEBOX,RESULT,") and
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
        elif (line.startswith("ENCODER RESET OK") or
              line.startswith("MPU YAW RESET") or
              line.startswith("IR SCAN RESTARTED")):
            self.status.set(line)
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
        self.root.after(150, self.redraw)

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
        count = 0
        try:
            if Path(path).suffix.lower() == ".csv":
                with open(path, "r", encoding="utf-8-sig", newline="") as file:
                    reader = csv.DictReader(file)
                    columns = set(reader.fieldnames or [])
                    if {"timestamp", "ENC_L", "ENC_R"} <= columns:
                        mode = "ENC"
                    elif {"valid", "L0", "L45", "L90", "R0", "R45", "R90"} <= columns:
                        mode = "IR"
                    elif {"valid", "yaw_cd", "gz_cd"} <= columns:
                        mode = "MPU"
                    else:
                        messagebox.showerror(
                            "Replay CSV", "Not a recognized System Test CSV.")
                        return
                    self.data_filter.set("All")
                    self.clear_data()
                    for row in reader:
                        try:
                            if mode == "ENC":
                                line = f"ENC,{int(row['ENC_L'])},{int(row['ENC_R'])}"
                            elif mode == "IR":
                                keys = ("valid", "L0", "L45", "L90", "R0", "R45", "R90")
                                line = "IR," + ",".join(f"{key}={int(row[key])}" for key in keys)
                            else:
                                line = f"MPU,valid={int(row['valid'])}"
                                if row["valid"] == "1":
                                    line += (f",yaw_cd={int(row['yaw_cd'])},"
                                             f"gz_cd={int(row['gz_cd'])}")
                            timestamp = row.get("timestamp") or datetime.now().isoformat(
                                timespec="milliseconds")
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
