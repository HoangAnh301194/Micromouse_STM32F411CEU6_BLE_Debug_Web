"""Manual MPU6050 yaw logger, restored from the v2 System Test."""
from __future__ import annotations

import csv
from datetime import datetime
import math
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from typing import Callable

LOG_DIR = Path(__file__).resolve().parents[1] / "logs" / "spp_runs" / "mpu test - manual yaw"


class MPUTestTab(ttk.Frame):
    """Read-only MPU yaw + gyro-Z display, one CSV per MCU-confirmed Start."""

    def __init__(self, parent: tk.Widget, send_cmd_fn: Callable[[str], bool],
                 get_port_fn: Callable[[], str]):
        super().__init__(parent, padding=10)
        self.send_cmd_fn = send_cmd_fn
        self.get_port_fn = get_port_fn
        self._id = 600000
        self.pending = False
        self.streaming = False
        self._handle = None
        self._writer = None
        self._samples = 0
        self._yaw = 0.0

        self.status = tk.StringVar(value="Idle. Hold still until MPU calibration completes.")
        self.yaw_value = tk.StringVar(value="Yaw: -- deg")
        self.gz_value = tk.StringVar(value="Gyro Z: -- deg/s")
        self.freshness = tk.StringVar(value="Sample: --")
        self.file_value = tk.StringVar(value="CSV: not recording")

        head = ttk.Frame(self)
        head.pack(fill="x")
        ttk.Label(head, text="MPU6050 / Manual Yaw Test",
                  font=("TkDefaultFont", 12, "bold")).pack(side="left")
        ttk.Button(head, text="Start MPU Log", command=self.start).pack(side="right", padx=4)
        ttk.Button(head, text="Stop", command=self.stop).pack(side="right", padx=4)
        ttk.Button(head, text="Zero yaw", command=self.zero).pack(side="right", padx=4)
        ttk.Label(self, text="Rotate the robot by hand. Test Start does not move motors.").pack(
            anchor="w", pady=(8, 2))
        ttk.Label(self, textvariable=self.status).pack(anchor="w")
        ttk.Label(self, textvariable=self.file_value, wraplength=1000).pack(
            anchor="w", pady=(4, 8))

        content = ttk.Frame(self)
        content.pack(fill="both", expand=True)
        left = ttk.LabelFrame(content, text="2D yaw", padding=8)
        left.pack(side="left", fill="both", expand=True, padx=(0, 8))
        self.canvas = tk.Canvas(left, height=320, width=350,
                                bg="#101923", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self._draw(self._yaw))
        right = ttk.LabelFrame(content, text="MPU data (20 Hz)", padding=8)
        right.pack(side="left", fill="both", expand=True)
        for variable in (self.yaw_value, self.gz_value, self.freshness):
            ttk.Label(right, textvariable=variable, font=("Consolas", 11)).pack(
                anchor="w", pady=2)
        self.terminal = tk.Text(right, wrap="none", font=("Consolas", 9),
                                bg="#101923", fg="#d5e2ef")
        self.terminal.pack(fill="both", expand=True, pady=(8, 0))
        self._draw(0.0)

    def _send(self, verb: str) -> bool:
        current = self._id
        self._id += 1
        if self._id >= 700000:
            self._id = 600000
        # Leading '$' uses the existing framed SPP command parser.
        command = "$" + str(current) + "," + verb + "\n"
        ok = self.send_cmd_fn(command)
        if not ok:
            self.status.set("SPP not connected or transmit queue is full.")
        return ok

    def start(self) -> None:
        if self.streaming or self.pending:
            self.status.set("Already logging or waiting for an MCU acknowledgment.")
            return
        self.pending = self._send("MPU_TEST_START")
        if self.pending:
            self.status.set("Waiting for MCU MPU_TEST_START acknowledgment...")

    def stop(self) -> None:
        if self._send("MPU_TEST_STOP"):
            self.status.set("Stopping MPU Log...")

    def zero(self) -> None:
        if self._send("MPU_TEST_ZERO"):
            self.status.set("Zeroing MPU yaw...")

    def _open_csv(self) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        port = "".join(c for c in self.get_port_fn() if c.isalnum() or c in "-_")
        name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = LOG_DIR / (name + "_" + (port or "COM") + ".csv")
        self._handle = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(("ms", "valid", "age_us", "yaw_deg", "gz_dps"))
        self._samples = 0
        self.terminal.delete("1.0", "end")
        self.file_value.set("CSV: " + str(path))

    def _close_csv(self) -> None:
        if self._handle is not None:
            self._handle.close()
        self._handle = None
        self._writer = None

    def on_disconnect(self) -> None:
        if self.streaming or self.pending:
            self.status.set("SPP disconnected; MPU CSV closed.")
        self.streaming = False
        self.pending = False
        self._close_csv()

    def _draw(self, yaw: float) -> None:
        c = self.canvas
        c.delete("all")
        w, h = max(c.winfo_width(), 300), max(c.winfo_height(), 260)
        cx, cy = w / 2.0, h / 2.0
        r = min(w, h) * 0.34
        c.create_oval(cx-r, cy-r, cx+r, cy+r, outline="#537187", width=2)
        c.create_line(cx-r-12, cy, cx+r+12, cy, fill="#426174")
        c.create_line(cx, cy-r-12, cx, cy+r+12, fill="#426174")
        c.create_text(cx, cy-r-20, text="Forward / 0 deg", fill="#d5e2ef")
        a = math.radians(yaw)
        x = cx + r * 0.85 * math.sin(a)
        y = cy - r * 0.85 * math.cos(a)
        c.create_line(cx, cy, x, y, fill="#4ed7b5", width=5, arrow="last")
        c.create_oval(cx-4, cy-4, cx+4, cy+4, fill="#d5e2ef")
        c.create_text(cx, cy+r+22, text=f"Yaw = {yaw:+.2f} deg", fill="#d5e2ef")

    def on_spp_line(self, line: str) -> None:
        if line.startswith(("ACK,", "NACK,")):
            parts = line.split(",")
            if len(parts) < 4:
                return
            kind, verb, status = parts[0], parts[2], parts[3]
            if verb == "MPU_TEST_START":
                self.pending = False
                if kind == "ACK" and status == "OK":
                    try:
                        self._open_csv()
                    except OSError as exc:
                        self.status.set("CSV open error: " + str(exc))
                        self._send("MPU_TEST_STOP")
                        return
                    self.streaming = True
                    self.status.set("MPU Test running; motion commands blocked.")
                else:
                    self.status.set("Start rejected: " + status)
                return
            if verb == "MPU_TEST_STOP" and kind == "ACK":
                self.streaming = False
                self.pending = False
                self._close_csv()
                self.status.set(f"MPU Test stopped. Saved {self._samples} samples.")
                return
            if verb == "MPU_TEST_ZERO":
                self.status.set("Yaw reset." if kind == "ACK"
                                else "Yaw reset rejected: " + status)
                return
            if verb == "STOP" and kind == "ACK" and self.streaming:
                self.streaming = False
                self._close_csv()
                self.status.set("Emergency STOP; CSV closed.")
                return

        if line.startswith("MPU_TEST,STOP,EMERGENCY"):
            self.streaming = False
            self.pending = False
            self._close_csv()
            self.status.set("Emergency STOP; CSV closed.")
            return
        if not self.streaming or not line.startswith("MPU,"):
            return

        fields = {}
        for token in line.split(",")[1:]:
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        try:
            ms = int(fields["ms"])
            valid = int(fields["valid"])
            age_us = int(fields["age_us"])
            yaw = int(fields["yaw_cd"]) / 100.0 if valid else None
            gz = int(fields["gz_cd"]) / 100.0 if valid else None
        except (KeyError, ValueError):
            return
        if self._writer is not None:
            self._writer.writerow((ms, valid, age_us,
                                   f"{yaw:.2f}" if yaw is not None else "",
                                   f"{gz:.2f}" if gz is not None else ""))
        self._samples += 1
        self.freshness.set(f"t={ms} ms | valid={valid} | age={age_us} us")
        if yaw is not None and gz is not None:
            self._yaw = yaw
            self.yaw_value.set(f"Yaw: {yaw:+.2f} deg")
            self.gz_value.set(f"Gyro Z: {gz:+.2f} deg/s")
            self._draw(yaw)
        self.terminal.insert("end", line + "\n")
        if self._samples > 300:
            self.terminal.delete("1.0", "2.0")
        self.terminal.see("end")
