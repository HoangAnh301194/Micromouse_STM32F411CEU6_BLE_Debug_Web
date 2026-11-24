#!/usr/bin/env python3
"""Motion Control Test tab for tools/classic_spp_dashboard.py.

Provides:
- Left/Right independent Wheel Speed PI+FF configuration (Read/Apply/Restore/Reset)
- 5 Motion Test modes: Wheel Speed, Straight Profile (165mm cell), Turn Profile,
  Velocity Control (with kinematics preview), and Quick Step (LOW/MEDIUM/HIGH)
- Per-run CSV logging via MotionCsvLogger (shared with headless runner)
- Non-blocking live plots (10-15 FPS) with separate PPS & PWM axes and chassis view
"""
from __future__ import annotations

from collections import deque
from datetime import datetime
import os
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any, Callable

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from classic_spp_protocol import (
    MOTION_STATE_NAMES,
    parse_ext_telemetry_line,
    parse_motion_telemetry_line,
    parse_spp_control_line,
)
from motion_experiment import (
    CELL_SIZE_MM,
    DEFAULT_HEADING_GAINS,
    DEFAULT_WHEEL_GAINS,
    MotionCsvLogger,
    analyze_motion_csv,
    chassis_to_wheel_pps,
    generate_motion_plots,
    get_git_commit_sha,
    write_summary_md,
)

ROOT_DIR = Path(__file__).resolve().parent.parent
RESULTS_BASE = ROOT_DIR / "results" / "motion_test"


class MotionControlTab(ttk.Frame):
    """Embedded Tkinter tab inside Classic SPP Dashboard."""

    def __init__(
        self,
        parent: tk.Widget,
        send_cmd_fn: Callable[[str], bool],
        send_stop_fn: Callable[[], None],
        set_hb_fn: Callable[[bool], None],
        get_port_fn: Callable[[], str],
    ):
        super().__init__(parent, padding=6)
        self.send_cmd_fn = send_cmd_fn
        self.send_stop_fn = send_stop_fn
        self.set_hb_fn = set_hb_fn
        self.get_port_fn = get_port_fn

        self._cmd_id = 1
        self._logger: MotionCsvLogger | None = None
        self._last_params: dict[str, Any] = {}
        self._confirmed_gains: dict[str, dict[str, float]] = {
            "L": dict(DEFAULT_WHEEL_GAINS),
            "R": dict(DEFAULT_WHEEL_GAINS),
        }
        self._confirmed_hgains: dict[str, float] = dict(DEFAULT_HEADING_GAINS)

        # Bounded buffers for 50Hz RX -> 12 FPS plot redraw
        maxlen = 400
        self.t_buf: deque[float] = deque(maxlen=maxlen)
        self.ref_l_buf: deque[float] = deque(maxlen=maxlen)
        self.meas_l_buf: deque[float] = deque(maxlen=maxlen)
        self.err_l_buf: deque[float] = deque(maxlen=maxlen)
        self.pcmd_l_buf: deque[float] = deque(maxlen=maxlen)
        self.papp_l_buf: deque[float] = deque(maxlen=maxlen)

        self.ref_r_buf: deque[float] = deque(maxlen=maxlen)
        self.meas_r_buf: deque[float] = deque(maxlen=maxlen)
        self.err_r_buf: deque[float] = deque(maxlen=maxlen)
        self.pcmd_r_buf: deque[float] = deque(maxlen=maxlen)
        self.papp_r_buf: deque[float] = deque(maxlen=maxlen)

        self.vref_buf: deque[float] = deque(maxlen=maxlen)
        self.vmeas_buf: deque[float] = deque(maxlen=maxlen)
        self.wref_buf: deque[float] = deque(maxlen=maxlen)
        self.wmeas_buf: deque[float] = deque(maxlen=maxlen)
        self.yaw_buf: deque[float] = deque(maxlen=maxlen)
        self.gz_buf: deque[float] = deque(maxlen=maxlen)
        self.dist_buf: deque[float] = deque(maxlen=maxlen)
        self.aerr_buf: deque[float] = deque(maxlen=maxlen)

        self._t0_ms: int | None = None
        self._plot_dirty = False
        self._last_plot_ts = 0.0

        self.fw_status_var = tk.StringVar(value="MCU: Unknown")
        self.motion_state_var = tk.StringVar(value="State: IDLE")
        self.last_result_var = tk.StringVar(value="Result: NONE")
        self.confirmed_label_var = tk.StringVar(value="Confirmed L/R: (click Read Gains)")
        self.vel_preview_var = tk.StringVar(value="Wheels: L=0 PPS | R=0 PPS")

        self._build_ui()
        self.after(90, self._periodic_redraw)

    def _next_id(self) -> int:
        cid = self._cmd_id
        self._cmd_id = (self._cmd_id % 999999) + 1
        return cid

    def _send(self, verb: str, *args: Any) -> bool:
        cid = self._next_id()
        line = ",".join([f"${cid}", verb] + [str(a) for a in args]) + "\n"
        ok = self.send_cmd_fn(line)
        self._log_term(f"TX> {line.strip()}")
        return ok

    def _build_ui(self) -> None:
        # 1. Top Status & Action Bar
        top = ttk.Frame(self)
        top.pack(fill="x", pady=(0, 4))
        ttk.Label(top, textvariable=self.fw_status_var, width=28).pack(side="left", padx=4)
        ttk.Label(top, textvariable=self.motion_state_var, width=18, font=("TkDefaultFont", 9, "bold")).pack(side="left", padx=4)
        ttk.Label(top, textvariable=self.last_result_var, width=18).pack(side="left", padx=4)

        btn_stop = tk.Button(
            top, text="STOP TEST", bg="#c92a2a", fg="white",
            font=("TkDefaultFont", 9, "bold"), padx=10, command=self._on_stop,
        )
        btn_stop.pack(side="right", padx=4)
        ttk.Button(top, text="Start Test", command=self._on_start).pack(side="right", padx=3)
        ttk.Button(top, text="Read Params", command=self._on_read_params).pack(side="right", padx=3)
        ttk.Button(top, text="Apply Params", command=self._on_apply_params).pack(side="right", padx=3)
        ttk.Button(top, text="Reset Enc", command=lambda: self._send("RESET_ENC")).pack(side="right", padx=2)
        ttk.Button(top, text="Reset Yaw", command=lambda: self._send("RESET_YAW")).pack(side="right", padx=2)
        ttk.Button(top, text="Save CSV", command=self._on_save_csv).pack(side="right", padx=2)
        ttk.Button(top, text="Open Results", command=self._on_open_results).pack(side="right", padx=2)

        # 2. Config Area: Left = Wheel Speed Controller (L/R), Right = Motion Test Modes
        cfg_pane = ttk.Panedwindow(self, orient="horizontal")
        cfg_pane.pack(fill="x", pady=4)

        # 2A. Wheel Speed Controller
        ws_frame = ttk.LabelFrame(cfg_pane, text="Wheel Speed Controller (PI + FF)", padding=6)
        cfg_pane.add(ws_frame, weight=1)

        self.gain_vars: dict[str, dict[str, tk.StringVar]] = {"L": {}, "R": {}}
        fields = [
            ("kp", "Kp", "0.090"),
            ("ki", "Ki", "0.250"),
            ("ks", "Ks (‰)", "80.0"),
            ("kv", "Kv (‰/PPS)", "0.098"),
            ("ka", "Ka", "0.0"),
            ("integral_limit", "I-Limit", "3000.0"),
            ("pwm_limit", "PWM Limit", "1000.0"),
        ]
        ttk.Label(ws_frame, text="Param").grid(row=0, column=0, sticky="w")
        ttk.Label(ws_frame, text="LEFT MOTOR", font=("TkDefaultFont", 8, "bold")).grid(row=0, column=1, padx=4)
        ttk.Label(ws_frame, text="RIGHT MOTOR", font=("TkDefaultFont", 8, "bold")).grid(row=0, column=2, padx=4)
        for idx, (key, label, default) in enumerate(fields, start=1):
            ttk.Label(ws_frame, text=label).grid(row=idx, column=0, sticky="w", pady=1)
            vl = tk.StringVar(value=default)
            vr = tk.StringVar(value=default)
            self.gain_vars["L"][key] = vl
            self.gain_vars["R"][key] = vr
            ttk.Entry(ws_frame, textvariable=vl, width=9).grid(row=idx, column=1, padx=3, pady=1)
            ttk.Entry(ws_frame, textvariable=vr, width=9).grid(row=idx, column=2, padx=3, pady=1)

        ws_btns = ttk.Frame(ws_frame)
        ws_btns.grid(row=len(fields) + 1, column=0, columnspan=3, pady=(4, 0), sticky="ew")
        ttk.Button(ws_btns, text="Read Gains", command=self._on_read_params).pack(side="left", padx=2)
        ttk.Button(ws_btns, text="Apply Gains", command=self._on_apply_gains).pack(side="left", padx=2)
        ttk.Button(ws_btns, text="Restore Defaults", command=lambda: self._send("RESTORE_GAINS")).pack(side="left", padx=2)
        ttk.Button(ws_btns, text="Reset Integrator", command=lambda: self._send("RESET_INT")).pack(side="left", padx=2)

        ttk.Label(ws_frame, textvariable=self.confirmed_label_var, foreground="#19b7a6").grid(
            row=len(fields) + 2, column=0, columnspan=3, sticky="w", pady=(3, 0)
        )

        # 2B. Motion Mode Tabs
        mode_frame = ttk.LabelFrame(cfg_pane, text="Motion Test Mode", padding=4)
        cfg_pane.add(mode_frame, weight=2)
        self.mode_tabs = ttk.Notebook(mode_frame)
        self.mode_tabs.pack(fill="both", expand=True)

        self._build_wheel_test_subtab()
        self._build_straight_subtab()
        self._build_turn_subtab()
        self._build_vel_subtab()
        self._build_qstep_subtab()

        # 3. Telemetry & Plots Area (Top = Terminal + toggles, Bottom = L/R/Chassis plots)
        data_pane = ttk.Panedwindow(self, orient="vertical")
        data_pane.pack(fill="both", expand=True, pady=(4, 0))

        # 3A. Terminal + Series Toggles
        term_frame = ttk.Frame(data_pane)
        data_pane.add(term_frame, weight=1)

        toggle_bar = ttk.Frame(term_frame)
        toggle_bar.pack(fill="x")
        self.show_ref = tk.BooleanVar(value=True)
        self.show_meas = tk.BooleanVar(value=True)
        self.show_err = tk.BooleanVar(value=True)
        self.show_pcmd = tk.BooleanVar(value=True)
        self.show_papp = tk.BooleanVar(value=True)
        ttk.Label(toggle_bar, text="Plot Series:").pack(side="left", padx=4)
        for text, var in (
            ("Ref", self.show_ref),
            ("Meas", self.show_meas),
            ("Error", self.show_err),
            ("PWM Cmd", self.show_pcmd),
            ("PWM Applied", self.show_papp),
        ):
            ttk.Checkbutton(toggle_bar, text=text, variable=var, command=self._force_redraw).pack(side="left", padx=4)
        ttk.Button(toggle_bar, text="Clear Plots", command=self._clear_plots).pack(side="right", padx=4)

        self.term_text = tk.Text(term_frame, height=6, bg="#10151d", fg="#dbe5f0", font=("Consolas", 8))
        self.term_text.pack(fill="both", expand=True)

        # 3B. Plots Notebook (Left/Right split view + Chassis view)
        plot_nb = ttk.Notebook(data_pane)
        data_pane.add(plot_nb, weight=3)

        lr_tab = ttk.Frame(plot_nb)
        ch_tab = ttk.Frame(plot_nb)
        plot_nb.add(lr_tab, text="Left & Right Wheels")
        plot_nb.add(ch_tab, text="Chassis & Yaw")

        lr_pane = ttk.Panedwindow(lr_tab, orient="horizontal")
        lr_pane.pack(fill="both", expand=True)

        self.fig_l = Figure(figsize=(4.5, 3.6), dpi=90)
        self.ax_l_spd = self.fig_l.add_subplot(211)
        self.ax_l_pwm = self.fig_l.add_subplot(212, sharex=self.ax_l_spd)
        self.canvas_l = FigureCanvasTkAgg(self.fig_l, master=lr_pane)
        lr_pane.add(self.canvas_l.get_tk_widget(), weight=1)

        self.fig_r = Figure(figsize=(4.5, 3.6), dpi=90)
        self.ax_r_spd = self.fig_r.add_subplot(211)
        self.ax_r_pwm = self.fig_r.add_subplot(212, sharex=self.ax_r_spd)
        self.canvas_r = FigureCanvasTkAgg(self.fig_r, master=lr_pane)
        lr_pane.add(self.canvas_r.get_tk_widget(), weight=1)

        self.fig_c = Figure(figsize=(9.0, 3.6), dpi=90)
        self.ax_c_lin = self.fig_c.add_subplot(311)
        self.ax_c_ang = self.fig_c.add_subplot(312, sharex=self.ax_c_lin)
        self.ax_c_yaw = self.fig_c.add_subplot(313, sharex=self.ax_c_lin)
        self.canvas_c = FigureCanvasTkAgg(self.fig_c, master=ch_tab)
        self.canvas_c.get_tk_widget().pack(fill="both", expand=True)

    def _build_wheel_test_subtab(self) -> None:
        f = ttk.Frame(self.mode_tabs, padding=6)
        self.mode_tabs.add(f, text="Wheel Speed")
        self.wt_preset = tk.StringVar(value="Left Only")
        self.wt_l_pps = tk.StringVar(value="500")
        self.wt_r_pps = tk.StringVar(value="0")
        self.wt_dur = tk.StringVar(value="1.5")
        self.wt_pwm = tk.StringVar(value="300")
        self.wt_mode = tk.StringVar(value="Step (0)")

        presets = ["Left Only", "Right Only", "Both Equal", "Diff Speeds", "Step", "Ramp", "Low-Speed Reverse"]
        ttk.Label(f, text="Preset:").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(f, textvariable=self.wt_preset, values=presets, state="readonly", width=18)
        cb.grid(row=0, column=1, padx=4, pady=2)
        cb.bind("<<ComboboxSelected>>", self._on_wt_preset)

        for r, (lbl, var) in enumerate([
            ("Left Ref (PPS):", self.wt_l_pps),
            ("Right Ref (PPS):", self.wt_r_pps),
            ("Duration (s):", self.wt_dur),
            ("PWM Limit (‰):", self.wt_pwm),
        ], start=1):
            ttk.Label(f, text=lbl).grid(row=r, column=0, sticky="w")
            ttk.Entry(f, textvariable=var, width=12).grid(row=r, column=1, padx=4, pady=2, sticky="w")

        ttk.Label(f, text="Profile:").grid(row=1, column=2, sticky="w", padx=(12, 2))
        ttk.Combobox(f, textvariable=self.wt_mode, values=["Step (0)", "Ramp (1)"], state="readonly", width=12).grid(
            row=1, column=3, padx=4
        )

    def _on_wt_preset(self, _evt=None) -> None:
        p = self.wt_preset.get()
        if p == "Left Only":
            self.wt_l_pps.set("500"); self.wt_r_pps.set("0"); self.wt_mode.set("Step (0)")
        elif p == "Right Only":
            self.wt_l_pps.set("0"); self.wt_r_pps.set("500"); self.wt_mode.set("Step (0)")
        elif p == "Both Equal":
            self.wt_l_pps.set("1000"); self.wt_r_pps.set("1000"); self.wt_mode.set("Step (0)")
        elif p == "Diff Speeds":
            self.wt_l_pps.set("800"); self.wt_r_pps.set("1600"); self.wt_mode.set("Step (0)")
        elif p == "Ramp":
            self.wt_l_pps.set("1200"); self.wt_r_pps.set("1200"); self.wt_mode.set("Ramp (1)")
        elif p == "Low-Speed Reverse":
            self.wt_l_pps.set("-500"); self.wt_r_pps.set("-500"); self.wt_mode.set("Step (0)")

    def _build_straight_subtab(self) -> None:
        f = ttk.Frame(self.mode_tabs, padding=6)
        self.mode_tabs.add(f, text="Straight Profile")
        self.st_preset = tk.StringVar(value="1 Cell (165 mm)")
        self.st_dist = tk.StringVar(value=str(int(CELL_SIZE_MM)))
        self.st_maxv = tk.StringVar(value="100.0")
        self.st_minv = tk.StringVar(value="60.0")
        self.st_endv = tk.StringVar(value="0.0")
        self.st_acc = tk.StringVar(value="300.0")
        self.st_dec = tk.StringVar(value="300.0")
        self.st_hkp = tk.StringVar(value="4.0")
        self.st_hki = tk.StringVar(value="0.0")
        self.st_hkd = tk.StringVar(value="0.05")
        self.st_maxw = tk.StringVar(value="120.0")
        self.st_hold = tk.BooleanVar(value=False)

        presets = ["1 Cell (165 mm)", "2 Cells (330 mm)", "3 Cells (495 mm)", "5 Cells (825 mm)", "Custom"]
        ttk.Label(f, text="Preset:").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(f, textvariable=self.st_preset, values=presets, state="readonly", width=18)
        cb.grid(row=0, column=1, padx=4, pady=2)
        cb.bind("<<ComboboxSelected>>", self._on_st_preset)
        ttk.Checkbutton(f, text="Enable Heading Hold", variable=self.st_hold).grid(row=0, column=2, columnspan=2, sticky="w", padx=6)

        items = [
            ("Distance (mm):", self.st_dist), ("Max V (mm/s):", self.st_maxv),
            ("Min V (mm/s):", self.st_minv), ("End V (mm/s):", self.st_endv),
            ("Accel (mm/s²):", self.st_acc), ("Decel (mm/s²):", self.st_dec),
            ("Heading Kp:", self.st_hkp), ("Heading Ki:", self.st_hki),
            ("Heading Kd:", self.st_hkd), ("Max W (dps):", self.st_maxw),
        ]
        for idx, (lbl, var) in enumerate(items):
            r = 1 + (idx // 2)
            c = (idx % 2) * 2
            ttk.Label(f, text=lbl).grid(row=r, column=c, sticky="w", padx=(4, 2), pady=1)
            ttk.Entry(f, textvariable=var, width=10).grid(row=r, column=c + 1, padx=4, pady=1, sticky="w")

    def _on_st_preset(self, _evt=None) -> None:
        m = {
            "1 Cell (165 mm)": "165",
            "2 Cells (330 mm)": "330",
            "3 Cells (495 mm)": "495",
            "5 Cells (825 mm)": "825",
        }
        val = m.get(self.st_preset.get())
        if val:
            self.st_dist.set(val)

    def _build_turn_subtab(self) -> None:
        f = ttk.Frame(self.mode_tabs, padding=6)
        self.mode_tabs.add(f, text="Turn Profile")
        self.tn_preset = tk.StringVar(value="Left 90°")
        self.tn_ang = tk.StringVar(value="90.0")
        self.tn_maxw = tk.StringVar(value="360.0")
        self.tn_minw = tk.StringVar(value="40.0")
        self.tn_acc = tk.StringVar(value="1200.0")
        self.tn_dec = tk.StringVar(value="1200.0")
        self.tn_akp = tk.StringVar(value="6.0")
        self.tn_rkp = tk.StringVar(value="0.35")
        self.tn_atol = tk.StringVar(value="3.0")
        self.tn_rtol = tk.StringVar(value="10.0")
        self.tn_tout = tk.StringVar(value="1.5")

        presets = ["Left 90°", "Right 90°", "Left 180°", "Right 180°", "Custom"]
        ttk.Label(f, text="Preset:").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(f, textvariable=self.tn_preset, values=presets, state="readonly", width=15)
        cb.grid(row=0, column=1, padx=4, pady=2)
        cb.bind("<<ComboboxSelected>>", self._on_tn_preset)

        items = [
            ("Angle (deg):", self.tn_ang), ("Max W (dps):", self.tn_maxw),
            ("Min W (dps):", self.tn_minw), ("Accel (dps²):", self.tn_acc),
            ("Decel (dps²):", self.tn_dec), ("Angle Kp:", self.tn_akp),
            ("Rate Kp:", self.tn_rkp), ("Ang Tol (deg):", self.tn_atol),
            ("Rate Tol (dps):", self.tn_rtol), ("Timeout (s):", self.tn_tout),
        ]
        for idx, (lbl, var) in enumerate(items):
            r = 1 + (idx // 2)
            c = (idx % 2) * 2
            ttk.Label(f, text=lbl).grid(row=r, column=c, sticky="w", padx=(4, 2), pady=1)
            ttk.Entry(f, textvariable=var, width=10).grid(row=r, column=c + 1, padx=4, pady=1, sticky="w")

    def _on_tn_preset(self, _evt=None) -> None:
        m = {"Left 90°": "90.0", "Right 90°": "-90.0", "Left 180°": "180.0", "Right 180°": "-180.0"}
        val = m.get(self.tn_preset.get())
        if val:
            self.tn_ang.set(val)

    def _build_vel_subtab(self) -> None:
        f = ttk.Frame(self.mode_tabs, padding=6)
        self.mode_tabs.add(f, text="Velocity Control")
        self.vl_preset = tk.StringVar(value="Forward")
        self.vl_v = tk.StringVar(value="120.0")
        self.vl_w = tk.StringVar(value="0.0")
        self.vl_hold = tk.BooleanVar(value=False)
        self.vl_yaw = tk.StringVar(value="0.0")
        self.vl_dur = tk.StringVar(value="1.5")

        for v in (self.vl_v, self.vl_w):
            v.trace_add("write", lambda *_: self._update_vel_preview())

        presets = ["Forward", "Reverse", "Rotate Left", "Rotate Right", "Arc Left", "Arc Right", "Stop"]
        ttk.Label(f, text="Preset:").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(f, textvariable=self.vl_preset, values=presets, state="readonly", width=15)
        cb.grid(row=0, column=1, padx=4, pady=2)
        cb.bind("<<ComboboxSelected>>", self._on_vl_preset)
        ttk.Checkbutton(f, text="Hold Heading", variable=self.vl_hold).grid(row=0, column=2, columnspan=2, sticky="w")

        items = [
            ("v (mm/s):", self.vl_v), ("ω (deg/s):", self.vl_w),
            ("Target Yaw:", self.vl_yaw), ("Duration (s):", self.vl_dur),
        ]
        for idx, (lbl, var) in enumerate(items):
            r = 1 + (idx // 2)
            c = (idx % 2) * 2
            ttk.Label(f, text=lbl).grid(row=r, column=c, sticky="w", padx=4, pady=2)
            ttk.Entry(f, textvariable=var, width=10).grid(row=r, column=c + 1, padx=4, pady=2, sticky="w")

        ttk.Label(f, textvariable=self.vel_preview_var, foreground="#ffb65c").grid(
            row=4, column=0, columnspan=4, sticky="w", pady=(6, 0)
        )
        self._update_vel_preview()

    def _on_vl_preset(self, _evt=None) -> None:
        m = {
            "Forward": ("120.0", "0.0"),
            "Reverse": ("-120.0", "0.0"),
            "Rotate Left": ("0.0", "120.0"),
            "Rotate Right": ("0.0", "-120.0"),
            "Arc Left": ("120.0", "90.0"),
            "Arc Right": ("120.0", "-90.0"),
            "Stop": ("0.0", "0.0"),
        }
        v, w = m.get(self.vl_preset.get(), ("0.0", "0.0"))
        self.vl_v.set(v)
        self.vl_w.set(w)

    def _update_vel_preview(self) -> None:
        try:
            v = float(self.vl_v.get())
            w = float(self.vl_w.get())
            lp, rp = chassis_to_wheel_pps(v, w)
            self.vel_preview_var.set(f"Differential Preview: Left={lp:.0f} PPS | Right={rp:.0f} PPS")
        except ValueError:
            self.vel_preview_var.set("Differential Preview: invalid number")

    def _build_qstep_subtab(self) -> None:
        f = ttk.Frame(self.mode_tabs, padding=6)
        self.mode_tabs.add(f, text="Quick Step")
        self.qs_level = tk.StringVar(value="LOW (0)")
        ttk.Label(f, text="Level:").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(
            f, textvariable=self.qs_level, values=["LOW (0)", "MEDIUM (1)", "HIGH (2)"],
            state="readonly", width=14,
        )
        cb.grid(row=0, column=1, padx=4, sticky="w")
        cb.bind("<<ComboboxSelected>>", self._on_qs_preview)

        self.qs_preview = tk.Text(f, height=6, width=54, font=("Consolas", 8))
        self.qs_preview.grid(row=1, column=0, columnspan=3, pady=(4, 0), sticky="ew")
        self._on_qs_preview()

    def _on_qs_preview(self, _evt=None) -> None:
        tables = {
            "LOW (0)": "P0(0.8s): v=180 w=0 | P1: v=160 w=+120 | P2: v=160 w=-120\nP3: v=0 w=+180 | P4: v=0 w=-180 | P5: v=120 w=0",
            "MEDIUM (1)": "P0(0.8s): v=300 w=0 | P1: v=250 w=+240 | P2: v=250 w=-240\nP3: v=0 w=+360 | P4: v=0 w=-360 | P5: v=200 w=0",
            "HIGH (2)": "P0(0.8s): v=500 w=0 | P1: v=420 w=+420 | P2: v=420 w=-420\nP3: v=0 w=+720 | P4: v=0 w=-720 | P5: v=420 w=0",
        }
        self.qs_preview.config(state="normal")
        self.qs_preview.delete("1.0", "end")
        self.qs_preview.insert("1.0", tables.get(self.qs_level.get(), ""))
        self.qs_preview.config(state="disabled")

    def _log_term(self, text: str) -> None:
        self.term_text.insert("end", f"[{datetime.now().strftime('%H:%M:%S')}] {text}\n")
        lines = int(self.term_text.index("end-1c").split(".")[0])
        if lines > 500:
            self.term_text.delete("1.0", f"{lines - 500}.0")
        self.term_text.see("end")

    def _on_read_params(self) -> None:
        self._send("PING")
        self._send("GET_GAINS")

    def _on_apply_gains(self) -> None:
        for side in ("L", "R"):
            gv = self.gain_vars[side]
            try:
                vals = [float(gv[k].get()) for k in ("kp", "ki", "ks", "kv", "ka", "integral_limit", "pwm_limit")]
            except ValueError:
                messagebox.showerror("Invalid Gain", f"Check numeric values for {side} motor.")
                return
            self._send("SET_GAINS", side, *vals)

    def _on_apply_params(self) -> None:
        self._on_apply_gains()
        try:
            self._send(
                "SET_HGAINS",
                float(self.st_hkp.get()),
                float(self.st_hki.get()),
                float(self.st_hkd.get()),
                float(self.st_maxw.get()),
            )
        except ValueError:
            pass

    def _start_run_logger(self, test_name: str, params: dict[str, Any]) -> None:
        self._close_run_logger("SUPERSEDED")
        meta = {
            "test_name": test_name,
            "git_commit": get_git_commit_sha(ROOT_DIR),
            "port": self.get_port_fn(),
            "confirmed_gains": self._confirmed_gains,
            "confirmed_hgains": self._confirmed_hgains,
            "test_params": params,
        }
        self._last_params = meta
        self._logger = MotionCsvLogger(RESULTS_BASE, test_name, test_name, meta)
        self._clear_plots()

    def _close_run_logger(self, finish_result: str) -> None:
        if self._logger is None:
            return
        logger = self._logger
        self._logger = None
        logger.close({"finish_result": finish_result})
        if logger._seq >= 2:
            try:
                pwm_max = float(self._last_params.get("test_params", {}).get("pwm_limit", 1000.0))
                metrics = analyze_motion_csv(logger.csv_path, pwm_max_pm=pwm_max, finish_result=finish_result)
                (logger.run_dir / "metrics.json").write_text(
                    json.dumps(metrics, indent=2), encoding="utf-8"
                )
                generate_motion_plots(logger.csv_path, logger.run_dir, title_prefix=logger.test_id)
                write_summary_md(logger.run_dir, logger.test_id, self._last_params, metrics)
                self._log_term(f"Saved run -> {logger.run_dir.name} ({logger._seq} samples)")
            except Exception as exc:
                self._log_term(f"Metrics/Plot warning: {exc}")

    def _on_start(self) -> None:
        tab_idx = self.mode_tabs.index(self.mode_tabs.select())
        try:
            if tab_idx == 0:  # Wheel Speed
                l_pps = float(self.wt_l_pps.get())
                r_pps = float(self.wt_r_pps.get())
                dur = float(self.wt_dur.get())
                pwm = float(self.wt_pwm.get())
                mode = 1 if "1" in self.wt_mode.get() else 0
                self._start_run_logger("gui_wheel_test", {"l_pps": l_pps, "r_pps": r_pps, "dur": dur, "pwm_limit": pwm, "mode": mode})
                self.set_hb_fn(True)
                self._send("WTEST", l_pps, r_pps, dur, pwm, mode)
            elif tab_idx == 1:  # Straight
                dist = float(self.st_dist.get())
                maxv = float(self.st_maxv.get())
                minv = float(self.st_minv.get())
                endv = float(self.st_endv.get())
                acc = float(self.st_acc.get())
                dec = float(self.st_dec.get())
                maxw = float(self.st_maxw.get())
                hold = 1 if self.st_hold.get() else 0
                self._start_run_logger(f"gui_straight_{int(dist)}mm", {"dist": dist, "maxv": maxv, "hold": hold})
                self.set_hb_fn(True)
                self._send("STRAIGHT", dist, maxv, minv, endv, acc, dec, maxw, hold)
            elif tab_idx == 2:  # Turn
                ang = float(self.tn_ang.get())
                args = (
                    ang, float(self.tn_maxw.get()), float(self.tn_minw.get()),
                    float(self.tn_acc.get()), float(self.tn_dec.get()),
                    float(self.tn_akp.get()), float(self.tn_rkp.get()),
                    float(self.tn_atol.get()), float(self.tn_rtol.get()), float(self.tn_tout.get()),
                )
                self._start_run_logger(f"gui_turn_{int(ang)}deg", {"angle": ang})
                self.set_hb_fn(True)
                self._send("TURN", *args)
            elif tab_idx == 3:  # Velocity
                v = float(self.vl_v.get())
                w = float(self.vl_w.get())
                hold = 1 if self.vl_hold.get() else 0
                tyaw = float(self.vl_yaw.get())
                dur = float(self.vl_dur.get())
                self._start_run_logger("gui_velocity", {"v": v, "w": w, "dur": dur})
                self.set_hb_fn(True)
                self._send("VEL", v, w, hold, tyaw, dur)
            elif tab_idx == 4:  # Quick Step
                lvl = 2 if "2" in self.qs_level.get() else (1 if "1" in self.qs_level.get() else 0)
                if lvl == 2 and not messagebox.askyesno("Confirm HIGH Quick Step", "Run HIGH-speed Quick Step Test?"):
                    return
                self._start_run_logger(f"gui_qstep_lvl{lvl}", {"level": lvl})
                self.set_hb_fn(True)
                self._send("QSTEP", lvl)
        except ValueError as exc:
            messagebox.showerror("Invalid Parameter", str(exc))

    def _on_stop(self) -> None:
        self.set_hb_fn(False)
        self.send_stop_fn()
        self._log_term("TX> EMERGENCY STOP (! / $0,STOP)")
        self._close_run_logger("ABORTED_BY_USER")

    def _on_save_csv(self) -> None:
        if self._logger is not None:
            self._close_run_logger("MANUAL_SAVE")
        else:
            messagebox.showinfo("CSV Logger", f"Runs are auto-saved per test under:\n{RESULTS_BASE}")

    def _on_open_results(self) -> None:
        RESULTS_BASE.mkdir(parents=True, exist_ok=True)
        try:
            os.startfile(str(RESULTS_BASE))
        except Exception as exc:
            messagebox.showerror("Open Results", str(exc))

    def on_spp_line(self, line: str) -> None:
        """Called on Tkinter main thread by Dashboard.handle_line()."""
        if self._logger is not None:
            self._logger.log_raw(line)

        telem = parse_motion_telemetry_line(line)
        if telem is not None:
            if self._logger is not None:
                self._logger.log_telemetry(telem)
            st_name = MOTION_STATE_NAMES.get(int(telem["state"]), str(telem["state"]))
            self.motion_state_var.set(f"State: {st_name} (ph {telem['phase']})")

            if self._t0_ms is None:
                self._t0_ms = int(telem["t_ms"])
            ts = (int(telem["t_ms"]) - self._t0_ms) / 1000.0
            self.t_buf.append(ts)
            self.ref_l_buf.append(float(telem["ref_l_pps"]))
            self.meas_l_buf.append(float(telem["meas_l_pps"]))
            self.err_l_buf.append(float(telem["error_l_pps"]))
            self.pcmd_l_buf.append(float(telem["pwm_cmd_l_pm"]))
            self.papp_l_buf.append(float(telem["pwm_app_l_pm"]))

            self.ref_r_buf.append(float(telem["ref_r_pps"]))
            self.meas_r_buf.append(float(telem["meas_r_pps"]))
            self.err_r_buf.append(float(telem["error_r_pps"]))
            self.pcmd_r_buf.append(float(telem["pwm_cmd_r_pm"]))
            self.papp_r_buf.append(float(telem["pwm_app_r_pm"]))

            self.yaw_buf.append(float(telem["yaw_deg"]))
            self.gz_buf.append(float(telem["gyro_z_dps"]))
            self.dist_buf.append(float(telem["distance_mm"]))
            self.aerr_buf.append(float(telem["angle_error_deg"]))

            from motion_experiment import wheel_pps_to_chassis
            vr, wr = wheel_pps_to_chassis(float(telem["ref_l_pps"]), float(telem["ref_r_pps"]))
            vm, wm = wheel_pps_to_chassis(float(telem["meas_l_pps"]), float(telem["meas_r_pps"]))
            self.vref_buf.append(vr)
            self.wref_buf.append(wr)
            self.vmeas_buf.append(vm)
            self.wmeas_buf.append(wm)

            self._plot_dirty = True
            return

        ext = parse_ext_telemetry_line(line)
        if ext is not None:
            if self._logger is not None:
                self._logger.update_ext(ext)
            return

        ctrl = parse_spp_control_line(line)
        if ctrl is not None:
            kind = ctrl["kind"]
            if kind in ("ACK", "NACK"):
                self._log_term(ctrl["raw"])
                if ctrl["verb"] == "PING":
                    f = ctrl["fields"]
                    self.fw_status_var.set(
                        f"MCU: RDY={f.get('READY','?')} CAL={f.get('CAL','?')} LOCK={f.get('LOCK','?')}"
                    )
            elif kind == "GAINS":
                side = ctrl["side"]
                if side in self.gain_vars:
                    self._confirmed_gains[side] = {k: ctrl[k] for k in DEFAULT_WHEEL_GAINS}
                    for k, val in self._confirmed_gains[side].items():
                        self.gain_vars[side][k].set(f"{val:.4g}" if k in ("kp", "ki", "kv", "ka") else f"{val:.1f}")
                    lg = self._confirmed_gains["L"]
                    rg = self._confirmed_gains["R"]
                    self.confirmed_label_var.set(
                        f"Confirmed: L(kp={lg['kp']:.3f},ki={lg['ki']:.3f}) | R(kp={rg['kp']:.3f},ki={rg['ki']:.3f})"
                    )
            elif kind == "HGAINS":
                self._confirmed_hgains = {
                    "kp": ctrl["kp"], "ki": ctrl["ki"], "kd": ctrl["kd"], "max_w_dps": ctrl["max_w_dps"]
                }
                self.st_hkp.set(f"{ctrl['kp']:.2f}")
                self.st_hki.set(f"{ctrl['ki']:.2f}")
                self.st_hkd.set(f"{ctrl['kd']:.3f}")
                self.st_maxw.set(f"{ctrl['max_w_dps']:.1f}")
            elif kind == "EVENT" and ctrl["event"] == "DONE":
                self.set_hb_fn(False)
                res = ctrl["fields"].get("RES", "DONE")
                self.last_result_var.set(f"Result: {res}")
                self._log_term(ctrl["raw"])
                self._close_run_logger(res)

    def _clear_plots(self) -> None:
        self._t0_ms = None
        for dq in (
            self.t_buf, self.ref_l_buf, self.meas_l_buf, self.err_l_buf, self.pcmd_l_buf, self.papp_l_buf,
            self.ref_r_buf, self.meas_r_buf, self.err_r_buf, self.pcmd_r_buf, self.papp_r_buf,
            self.vref_buf, self.vmeas_buf, self.wref_buf, self.wmeas_buf,
            self.yaw_buf, self.gz_buf, self.dist_buf, self.aerr_buf,
        ):
            dq.clear()
        self._force_redraw()

    def _force_redraw(self) -> None:
        self._plot_dirty = True
        self._draw_all_plots()

    def _periodic_redraw(self) -> None:
        now = time.monotonic()
        if self._plot_dirty and (now - self._last_plot_ts) >= 0.08:  # ~12.5 FPS max
            self._draw_all_plots()
            self._last_plot_ts = now
            self._plot_dirty = False
        self.after(80, self._periodic_redraw)

    def _draw_all_plots(self) -> None:
        t = list(self.t_buf)
        if not t:
            return
        for ax_spd, ax_pwm, canvas, title, r_b, m_b, e_b, pc_b, pa_b in (
            (self.ax_l_spd, self.ax_l_pwm, self.canvas_l, "Left Wheel",
             self.ref_l_buf, self.meas_l_buf, self.err_l_buf, self.pcmd_l_buf, self.papp_l_buf),
            (self.ax_r_spd, self.ax_r_pwm, self.canvas_r, "Right Wheel",
             self.ref_r_buf, self.meas_r_buf, self.err_r_buf, self.pcmd_r_buf, self.papp_r_buf),
        ):
            ax_spd.clear()
            if self.show_ref.get():
                ax_spd.plot(t, list(r_b), "--", label="Ref", color="#1f77b4")
            if self.show_meas.get():
                ax_spd.plot(t, list(m_b), label="Meas", color="#2ca02c")
            if self.show_err.get():
                ax_spd.plot(t, list(e_b), label="Err", color="#d62728", alpha=0.8)
            ax_spd.set_title(title, fontsize=9)
            ax_spd.set_ylabel("PPS", fontsize=8)
            ax_spd.grid(True, alpha=0.3)
            ax_spd.legend(loc="upper left", fontsize=7, ncol=3)

            ax_pwm.clear()
            if self.show_pcmd.get():
                ax_pwm.plot(t, list(pc_b), label="PWM Cmd", color="#ff7f0e")
            if self.show_papp.get():
                ax_pwm.plot(t, list(pa_b), ":", label="PWM App", color="#9467bd")
            ax_pwm.set_ylabel("PWM ‰", fontsize=8)
            ax_pwm.set_xlabel("Time (s)", fontsize=8)
            ax_pwm.grid(True, alpha=0.3)
            ax_pwm.legend(loc="upper left", fontsize=7, ncol=2)
            canvas.draw_idle()

        self.ax_c_lin.clear()
        self.ax_c_lin.plot(t, list(self.vref_buf), "--", label="v_ref")
        self.ax_c_lin.plot(t, list(self.vmeas_buf), label="v_meas")
        self.ax_c_lin.set_ylabel("mm/s", fontsize=8)
        self.ax_c_lin.grid(True, alpha=0.3)
        self.ax_c_lin.legend(loc="upper left", fontsize=7, ncol=2)

        self.ax_c_ang.clear()
        self.ax_c_ang.plot(t, list(self.wref_buf), "--", label="w_ref")
        self.ax_c_ang.plot(t, list(self.wmeas_buf), label="w_enc")
        self.ax_c_ang.plot(t, list(self.gz_buf), label="gyro_z", alpha=0.8)
        self.ax_c_ang.set_ylabel("dps", fontsize=8)
        self.ax_c_ang.grid(True, alpha=0.3)
        self.ax_c_ang.legend(loc="upper left", fontsize=7, ncol=3)

        self.ax_c_yaw.clear()
        self.ax_c_yaw.plot(t, list(self.yaw_buf), label="Yaw (deg)")
        self.ax_c_yaw.plot(t, list(self.aerr_buf), label="AngErr (deg)")
        self.ax_c_yaw.plot(t, list(self.dist_buf), "--", label="Dist (mm)")
        self.ax_c_yaw.set_xlabel("Time (s)", fontsize=8)
        self.ax_c_yaw.grid(True, alpha=0.3)
        self.ax_c_yaw.legend(loc="upper left", fontsize=7, ncol=3)
        self.canvas_c.draw_idle()
