#!/usr/bin/env python3
"""JDY-33 Bluetooth Classic SPP desktop dashboard (receive-only by design).

Same physical JDY-33 on STM32 USART2 PA2/PA3; Web BLE remains untouched.
Run: python tools/classic_spp_dashboard.py --port COM37
"""
from __future__ import annotations

import argparse
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


BAUD = 115200
PREFERRED_PORT = "COM37"
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
    """Prefer a clearly identified JDY/SPP COM port; never infer USB-TTL."""
    for port in ports:
        descriptor = " ".join((port.description or "", port.manufacturer or "")).lower()
        if "jdy-33" in descriptor or "nezumi_chan_spp" in descriptor:
            return port.device
    # Multiple "Standard Serial over Bluetooth" ports often include INCOMING
    # virtual ports, which cannot initiate a connection to the robot.
    # Never silently pick the first generic Bluetooth COM.
    if any(p.device.upper() == PREFERRED_PORT for p in ports):
        return PREFERRED_PORT
    return None


class SPPReader:
    """Thread owns serial I/O. Tkinter is only touched by the main/UI thread."""

    def __init__(self, events: queue.Queue):
        self.events = events
        self.stop = threading.Event()
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
        self.thread = threading.Thread(target=self._run, args=(port, baud),
                                       daemon=True, name="jdy33-spp-reader")
        self.thread.start()

    def close(self):
        self.stop.set()

    def _run(self, port: str, baud: int):
        connected_at = None
        total_bytes = 0
        fault = None
        try:
            import serial
            with serial.Serial(port=port, baudrate=baud, timeout=0.15,
                               write_timeout=0.5) as link:
                connected_at = time.monotonic()
                self._emit("state", "%s COM open; waiting for SPP RX bytes" % port)
                framer = LineFramer()
                while not self.stop.is_set():
                    chunk = link.read(link.in_waiting or 1)
                    if not chunk:
                        continue
                    total_bytes += len(chunk)
                    if total_bytes == len(chunk):
                        self._emit("state", "SPP RX CONFIRMED: %s (%d bytes)" % (port, len(chunk)))
                    for line in framer.feed(chunk):
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
        self.logging_file = None
        self.log_path = ""
        self.maze_dirty = True
        self.plot_dirty = True
        self.last_plot = 0.0
        self.filter_text = tk.StringVar(value="")
        self.status = tk.StringVar(value="Disconnected · BLE web remains independent")
        self.port_value = tk.StringVar(value=default_port)
        self.baud_value = tk.StringVar(value=str(BAUD))
        self.map_status = tk.StringVar(value=self.maze.status)
        self.rx_count = tk.StringVar(value="0 lines")
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
        ttk.Label(outer, text="JDY-33 · STM32 USART2 PA2/PA3 · receive-only. "
                  "No firmware modification, no motor or Flash commands.",
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
        ttk.Button(bar, text="Replay log", command=self.open_log).pack(side="right")
        self.log_btn = ttk.Button(bar, text="Start log", command=self.toggle_log)
        self.log_btn.pack(side="right", padx=(4, 4))

        info = ttk.Frame(outer)
        info.pack(fill="x", pady=(0, 7))
        ttk.Label(info, textvariable=self.status).pack(side="left")
        ttk.Label(info, textvariable=self.rx_count).pack(side="right")

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
        bar = ttk.Frame(self.terminal_tab)
        bar.pack(fill="x", pady=(0, 5))
        ttk.Label(bar, text="Filter:").pack(side="left")
        ttk.Entry(bar, textvariable=self.filter_text, width=32).pack(side="left", padx=5)
        ttk.Button(bar, text="Clear view", command=self.clear_view).pack(side="left")
        ttk.Label(bar, text="All unfiltered RX lines are retained in the file log",
                  style="Muted.TLabel").pack(side="right")
        frame = ttk.Frame(self.terminal_tab)
        frame.pack(fill="both", expand=True)
        self.terminal = tk.Text(frame, wrap="none", state="disabled",
                                bg="#101923", fg="#d5e2ef", insertbackground="white",
                                font=("Consolas", 10), relief="flat")
        v = ttk.Scrollbar(frame, orient="vertical", command=self.terminal.yview)
        h = ttk.Scrollbar(frame, orient="horizontal", command=self.terminal.xview)
        self.terminal.configure(yscrollcommand=v.set, xscrollcommand=h.set)
        self.terminal.grid(row=0, column=0, sticky="nsew")
        v.grid(row=0, column=1, sticky="ns")
        h.grid(row=1, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        self.terminal.tag_configure("error", foreground="#ff8686")
        self.terminal.tag_configure("map", foreground="#51d0be")
        self.terminal.tag_configure("normal", foreground="#d5e2ef")

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
        if current in ("", PREFERRED_PORT) and current not in (
            p.device.upper() for p in ports
        ):
            match = suggested_port(ports)
            if match:
                self.port_value.set(match)
        # Never connect to or claim the availability of an unlisted COM.

    def connect(self):
        if self.reader.thread and self.reader.thread.is_alive():
            return
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
        if not any(p.device.lower() == port.lower() for p in available_ports()):
            if not messagebox.askyesno(
                "Unlisted COM", port + " is not in Windows serial port list.\n"
                "Try opening anyway? (COM37 is only a preference)"
            ):
                return
        self.reader.start(port, baud)
        self.status.set("Opening " + port + " ...")

    def disconnect(self):
        self.reader.close()
        self.status.set("Disconnect requested")

    def _append(self, line, ts):
        key = "error" if "FAIL" in line or "ERROR" in line else (
            "map" if line.startswith(("MAP", "MZ", "MAZE:", "CELL:")) else "normal")
        if self.filter_text.get().lower() not in line.lower():
            return
        self.terminal.configure(state="normal")
        self.terminal.insert("end", "[%s] %s\n" % (ts, line), key)
        if int(self.terminal.index("end-1c").split(".")[0]) > MAX_TERMINAL_LINES:
            self.terminal.delete("1.0", "251.0")
        self.terminal.see("end")
        self.terminal.configure(state="disabled")

    def handle_line(self, line: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.total_lines += 1
        self.rx_count.set("%d lines" % self.total_lines)
        if self.logging_file:
            self.logging_file.write("%s %s\n" % (datetime.now().isoformat(timespec="milliseconds"), line))
            self.logging_file.flush()
        self._append(line, ts)
        changed = self.maze.apply(line)
        if changed:
            self.map_status.set(changed)
            self.maze_dirty = True
        values = numeric_fields(line)
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

    def poll(self):
        for _ in range(160):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                self.handle_line(value)
            else:
                self.status.set(value)
                if kind == "warning":
                    self._append("WARNING: " + value, datetime.now().strftime("%H:%M:%S"))
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

    def clear_view(self):
        self.terminal.configure(state="normal")
        self.terminal.delete("1.0", "end")
        self.terminal.configure(state="disabled")

    def toggle_log(self):
        if self.logging_file:
            self.logging_file.close()
            self.logging_file = None
            self.log_btn.configure(text="Start log")
            self.status.set("Log saved: " + self.log_path)
            return
        default = "jdy33_spp_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".log"
        path = filedialog.asksaveasfilename(title="Record raw SPP telemetry",
                                            defaultextension=".log",
                                            initialfile=default,
                                            filetypes=[("Text log", "*.log"),
                                                       ("All files", "*.*")])
        if path:
            try:
                self.logging_file = open(path, "a", encoding="utf-8", buffering=1)
            except OSError as exc:
                messagebox.showerror("Log file", str(exc))
                return
            self.log_path = path
            self.log_btn.configure(text="Stop log")
            self.status.set("Recording to " + path)

    def open_log(self):
        path = filedialog.askopenfilename(title="Replay line-oriented BLE/SPP log",
                                          filetypes=[("Logs", "*.log *.txt"),
                                                     ("All files", "*.*")])
        if not path:
            return
        count = 0
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as file:
                for raw in file:
                    text = raw.strip()
                    # Log saved by this app begins with ISO datetime.
                    if len(text) > 24 and text[4:5] == "-" and text[10:11] == "T":
                        text = text.split(" ", 1)[-1]
                    if text:
                        self.handle_line(text)
                        count += 1
        except OSError as exc:
            messagebox.showerror("Replay", str(exc))
            return
        self.status.set("Replayed %d lines from %s" % (count, Path(path).name))

    def quit(self):
        self.reader.close()
        if self.logging_file:
            self.logging_file.close()
        self.root.destroy()


def main():
    arg = argparse.ArgumentParser(description=__doc__)
    arg.add_argument("--port", default=PREFERRED_PORT,
                     help="Windows outgoing SPP serial port, default COM37 preference")
    args = arg.parse_args()
    root = tk.Tk()
    Dashboard(root, args.port)
    root.mainloop()


if __name__ == "__main__":
    main()
