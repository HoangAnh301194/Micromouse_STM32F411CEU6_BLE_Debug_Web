#!/usr/bin/env python3
"""Capture STM32F411 MPU6050 debug lines from a serial port and plot CSV data.

Requirements:
    python -m pip install pyserial matplotlib

Examples:
    python imu_capture_plot.py --port COM7 --baud 115200 --seconds 180
    python imu_capture_plot.py --port COM7 --seconds 0 --live
    python imu_capture_plot.py --input 'existing_terminal_log.txt'

The recorder opens COM7 ITSELF; close PlatformIO's device monitor first.
This parses existing text diagnostics (IMU/FRESH/NOISE/TIMING/CAL etc.).
It cannot recover 1000-Hz raw samples that the firmware never transmits.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import re
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import TextIO


KINDS = {
    "IMU": ["read_hz", "good", "err", "busy", "timeout", "age_us",
            "i2c_last_us", "i2c_max_us", "update_max_us", "cal", "cal_n",
            "bias_mdps", "gz_mdps", "yaw_mdeg", "ctrl_hz", "last_i2c_error",
            "uart_drop", "uart_dma_err"],
    "FRESH": ["status", "ready", "not_ready", "fresh", "stale",
              "yaw_deadzone_mdeg", "yaw_lpf_no_deadzone_mdeg",
              "yaw_raw_no_deadzone_mdeg"],
    "NOISE": ["n", "raw_mean_mdps", "raw_std_mdps", "raw_min_mdps",
              "raw_max_mdps", "corrected_mean_mdps", "same_code",
              "sample_min_us", "sample_max_us"],
    "TIMING": ["period_min_us", "period_max_us", "late_50us",
               "gap_over_1500us", "observed_isr_max_us", "max_cycles",
               "core_hz"],
    "CAL": ["state", "retries", "fail_reason", "warmup_elapsed_ms",
            "accepted", "valid_blocks", "last_std_mdps", "last_span_mdps",
            "block_mean_span_mdps", "bias_mdps"],
    "CAL_DONE": ["n", "blocks", "retries", "bias_mdps", "previous_bias_mdps",
                 "delta_bias_mdps", "max_block_std_mdps", "block_mean_span_mdps"],
    "CAL_RESTART": ["old_bias_mdps"],
    "CAL_CHECK": ["elapsed_ms", "raw_drift_mdeg", "lpf_no_deadzone_mdeg",
                  "raw_mdeg_per_min", "current_bias_mdps"],
    "DRIFT_START": ["bias_mdps", "dlpf"],
    "CLOCK": ["sysclk", "hsi_pll100", "apb1", "apb2", "tim5_us_clock"],
    "CLOCK_DIAG": ["fault", "rcc_cr", "cfgr", "pllcfgr", "pwr_cr", "pwr_csr", "flash_acr"],
    "I2C_PROBE": ["400_a", "400_w", "400_id", "300_a", "300_w", "300_id",
                  "200_a", "200_w", "200_id", "100_a", "100_w", "100_id",
                  "selected", "selected_scl"],
    "I2C_LINES": ["scl_pb8", "sda_pb9", "idr"],
    "BOOT_FAIL": ["clock_fault", "imu_status", "init_step", "reg", "expect",
                  "got", "i2c_status", "scl_nominal"],
    "BOOT_FAIL_ALIVE": ["clock_fault", "imu_step", "i2c_status"],
    "D": ["mcu_time_us", "raw_corrected_mdps", "lpf_mdps"],
}

META = ["host_time_iso", "elapsed_s", "line_number", "session_id"]


def parse_line(text: str) -> tuple[str, dict[str, str]] | None:
    """Accept comma + key=value firmware records; skip malformed tokens safely."""
    text = text.strip()
    if not text or "," not in text:
        return None
    kind, remainder = text.split(",", 1)
    kind = kind.strip()
    if kind not in KINDS:
        return None
    if kind == "D":
        parts = remainder.split(",")
        if len(parts) < 3:
            return None
        data = dict(zip(KINDS["D"], [p.strip() for p in parts[:3]]))
    else:
        data = {}
        for token in remainder.replace(",", " ").split():
            if "=" not in token or "\ufffd" in token:
                continue
            key, value = token.split("=", 1)
            key = key.strip(",")
            value = value.strip(",")
            if key in KINDS[kind] and value:
                data[key] = value
    return (kind, data) if data else None


def numeric(value: object) -> float | None:
    """Convert numeric strings, including 0x-prefixed register values."""
    if value is None or value == "":
        return None
    try:
        if isinstance(value, str) and value.lower().startswith("0x"):
            return float(int(value, 16))
        if isinstance(value, str) and "/" in value:
            # CAL prints accepted=500/4000 and valid_blocks=1/8.
            return float(value.split("/", 1)[0])
        return float(value)
    except (TypeError, ValueError, OverflowError):
        return None


def plot_series(rows: list[dict[str, str]], pairs: list[tuple[str, float, str]],
                output: Path, title: str, ylabel: str,
                xlabel: str = "Elapsed time (s)") -> bool:
    """Create one chart per figure; scales convert mdps->dps or mdeg->deg."""
    if not rows:
        return False
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10.5, 5.3))
    plotted = False
    for field, scale, label in pairs:
        xs, ys = [], []
        last_session = None
        for r in rows:
            x = numeric(r.get("elapsed_s"))
            y = numeric(r.get(field))
            session = r.get("session_id", "0")
            if x is not None and y is not None:
                if last_session is not None and session != last_session:
                    # Never draw an artificial line across a yaw reset.
                    xs.append(float("nan"))
                    ys.append(float("nan"))
                xs.append(x)
                ys.append(y * scale)
                last_session = session
        if xs:
            ax.plot(xs, ys, label=label, linewidth=1.5)
            plotted = True
    if not plotted:
        plt.close(fig)
        return False
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return True


class Recorder:
    def __init__(self, directory: Path):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        self.handles: dict[str, TextIO] = {}
        self.csvwriters: dict[str, csv.DictWriter] = {}
        self.records: dict[str, list[dict[str, str]]] = defaultdict(list)
        self.raw_file = (directory / "raw_serial.log").open("w", encoding="utf-8")
        self.start_monotonic = time.monotonic()
        self.line_no = 0
        self.offline_time_s = -1.0
        self.session_id = 0
        self.count_unknown = 0
        self.live_view = None
        self.next_draw_time = 0.0

    def close(self):
        for f in self.handles.values():
            f.close()
        self.raw_file.close()

    def receive(self, line: str, offline: bool = False, original_host_time: str | None = None, original_elapsed: float | None = None):
        self.line_no += 1
        line = line.rstrip("\r\n")
        host_time = dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")
        if offline:
            # Existing terminal text usually contains no timestamps. Its IMU
            # records mark ~1 second boundaries; same-second FRESH/NOISE join it.
            if line.startswith("IMU,"):
                self.offline_time_s += 1.0
            elapsed_s = max(0.0, self.offline_time_s)
        else:
            elapsed_s = time.monotonic() - self.start_monotonic
        if original_host_time is not None:
            host_time = original_host_time
        if original_elapsed is not None:
            elapsed_s = original_elapsed
        self.raw_file.write(f"{host_time}\t{elapsed_s:.6f}\t{line}\n")

        parsed = parse_line(line)
        if not parsed:
            self.count_unknown += 1
            return
        kind, values = parsed
        if kind == "DRIFT_START":
            self.session_id += 1
        row = {"host_time_iso": host_time, "elapsed_s": f"{elapsed_s:.6f}",
               "line_number": str(self.line_no), "session_id": str(self.session_id)}
        row.update(values)
        if kind not in self.csvwriters:
            handle = (self.directory / f"{kind.lower()}.csv").open(
                "w", newline="", encoding="utf-8-sig")
            self.handles[kind] = handle
            writer = csv.DictWriter(handle, fieldnames=META + KINDS[kind], extrasaction="ignore")
            writer.writeheader()
            self.csvwriters[kind] = writer
        self.csvwriters[kind].writerow(row)
        self.records[kind].append(row)
        # Flush regularly so data survives an accidental power outage.
        if self.line_no % 25 == 0:
            for handle in self.handles.values():
                handle.flush()
            self.raw_file.flush()

    def init_live(self):
        import matplotlib.pyplot as plt
        try:
            plt.ion()
            fig, ax = plt.subplots(figsize=(9, 5))
            fig.canvas.manager.set_window_title("MPU6050 live yaw")
            self.live_view = (plt, fig, ax)
            plt.show(block=False)
        except Exception as exc:
            print(f"[WARN] Live graphics not available: {exc}", file=sys.stderr)
            self.live_view = None

    def update_live(self):
        if self.live_view is None or time.monotonic() < self.next_draw_time:
            return
        self.next_draw_time = time.monotonic() + 1.5
        rows = self.records.get("FRESH", [])
        if self.records.get("DRIFT_START"):
            rows = [r for r in rows
                    if r.get("session_id") == str(self.session_id)]
        elif self.records.get("CAL_RESTART"):
            # A new calibration is in progress and there is no valid yaw.
            return
        if not rows:
            return
        plt, fig, ax = self.live_view
        try:
            ax.clear()
            for field, label in [("yaw_raw_no_deadzone_mdeg", "raw yaw"),
                                 ("yaw_lpf_no_deadzone_mdeg", "LPF yaw"),
                                 ("yaw_deadzone_mdeg", "deadzone yaw")]:
                xy = [(numeric(r.get("elapsed_s")), numeric(r.get(field))) for r in rows]
                x = [a for a, b in xy if a is not None and b is not None]
                y = [b / 1000.0 for a, b in xy if a is not None and b is not None]
                if x:
                    ax.plot(x, y, label=label)
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("Elapsed time (s)")
            ax.set_ylabel("Yaw (deg)")
            ax.set_title("MPU6050 live drift (relative yaw)")
            ax.legend()
            fig.canvas.draw_idle()
            plt.pause(0.001)
        except Exception as exc:
            print(f"[WARN] Live view stopped: {exc}", file=sys.stderr)
            self.live_view = None

    def make_plots(self) -> list[str]:
        plots: list[str] = []
        candidates = [
            ("01_yaw_drift.png", "FRESH",
             [("yaw_deadzone_mdeg", 0.001, "Yaw with deadzone"),
              ("yaw_lpf_no_deadzone_mdeg", 0.001, "LPF yaw, no deadzone"),
              ("yaw_raw_no_deadzone_mdeg", 0.001, "Raw yaw, no deadzone")],
             "Yaw drift and deadzone comparison", "Yaw (deg)"),
            ("02_raw_gyro_noise.png", "NOISE",
             [("raw_std_mdps", 0.001, "Gyro standard deviation"),
              ], "Raw gyro noise by one-second window", "Std (deg/s)"),
            ("03_gyro_bias_residual.png", "NOISE",
             [("corrected_mean_mdps", 0.001, "Bias-corrected mean")],
             "Residual gyro bias (no software deadzone)", "Angular rate (deg/s)"),
            ("04_fresh_sample_rate.png", "FRESH",
             [("fresh", 1.0, "DATA_RDY-confirmed gyro samples/window"),
              ("not_ready", 1.0, "Not-ready status polls/window")],
             "Sample freshness per report window", "Samples per report window"),
            ("05_i2c_throughput.png", "IMU",
             [("read_hz", 1.0, "Gyro I2C reads per second"),
              ("ctrl_hz", 1.0, "TIM11 ticks per second")],
             "IMU throughput and controller diagnostic timer", "Rate (Hz)"),
            ("06_tim11_period.png", "TIMING",
             [("period_min_us", 1.0, "Shortest observed interval"),
              ("period_max_us", 1.0, "Longest observed interval")],
             "TIM11 ISR entry interval", "Interval (us)"),
            ("07_calibration_progress.png", "CAL",
             [("accepted", 1.0, "Samples accepted")],
             "Bias-calibration progress (retries reset progress)", "Samples"),
            ("08_dynamic_gyro_response.png", "D",
             [("raw_corrected_mdps", 0.001, "Raw bias-corrected gyro"),
              ("lpf_mdps", 0.001, "Software-LPF gyro")],
             "Gyro angular velocity during rotation", "Angular rate (deg/s)"),
        ]
        for filename, kind, pairs, title, ylabel in candidates:
            try:
                rows = self.records.get(kind, [])
                if kind == "FRESH" and self.records.get("DRIFT_START"):
                    # Do not connect old yaw to the zero value during
                    # calibration. Display only completed calibration sessions.
                    rows = [r for r in rows if r.get("session_id", "0") != "0"]
                if kind == "NOISE":
                    # While CAL runs, firmware intentionally reports n=0.
                    # That is missing noise data, not zero-variance data.
                    rows = [r for r in rows if (numeric(r.get("n")) or 0) > 0]
                if kind == "CAL" and self.records.get("CAL_RESTART"):
                    first_restart = numeric(self.records["CAL_RESTART"][0].get("elapsed_s"))
                    if first_restart is not None:
                        rows = [r for r in rows
                                if (numeric(r.get("elapsed_s")) or 0) >= first_restart]
                if kind == "D":
                    rates = [abs(numeric(r.get("raw_corrected_mdps")) or 0)
                             for r in rows]
                    if not rates or max(rates) < 1000.0:
                        # In 2026-10 branch firmware, the old signed
                        # deadline erroneously logged D after micros()>2^31.
                        print("[INFO] Suppress dynamic-motion chart: D values "
                              "never exceed 1 deg/s; possible old timer bug.")
                        continue
                path = self.directory / filename
                if plot_series(rows, pairs, path, title, ylabel):
                    plots.append(filename)
            except ImportError as exc:
                print(f"[WARN] Could not plot; install matplotlib: {exc}", file=sys.stderr)
                break
        return plots

    def write_summary(self):
        data = self.records
        out = []
        out.append("MPU6050 telemetry summary")
        out.append(f"Input lines: {self.line_no}; unrecognized/non-CSV lines: {self.count_unknown}")
        out.append("NOTE: IMU/FRESH/NOISE/TIMING are 1Hz summaries, NOT individual 1kHz samples.")
        for kind in sorted(data):
            out.append(f"{kind}: {len(data[kind])} records")
        for kind, field, scale, label in [
            ("IMU", "read_hz", 1.0, "Mean gyro read rate (Hz)"),
            ("NOISE", "raw_std_mdps", 0.001, "Mean gyro raw std (deg/s)"),
            ("NOISE", "corrected_mean_mdps", 0.001, "Mean residual bias (deg/s)"),
            ("TIMING", "period_max_us", 1.0, "Maximum observed TIM11 period (us)"),
        ]:
            source_rows = data.get(kind, [])
            if kind == "NOISE":
                source_rows = [r for r in source_rows
                               if (numeric(r.get("n")) or 0) > 0]
            vals = [numeric(r.get(field)) for r in source_rows]
            vals = [v for v in vals if v is not None]
            if vals:
                value = max(vals) if label.startswith("Maximum") else statistics.mean(vals)
                out.append(f"{label}: {value * scale:.6f}")
        # Only use the most recent calibration session for an endpoint slope.
        fresh_rows = data.get("FRESH", [])
        if fresh_rows:
            last_session = fresh_rows[-1].get("session_id", "0")
            fresh_rows = [r for r in fresh_rows if r.get("session_id", "0") == last_session]
        yaw = [(numeric(r.get("elapsed_s")), numeric(r.get("yaw_raw_no_deadzone_mdeg")))
               for r in fresh_rows]
        yaw = [(t, y) for t, y in yaw if t is not None and y is not None]
        if len(yaw) >= 2 and yaw[-1][0] > yaw[0][0]:
            drift = (yaw[-1][1] - yaw[0][1]) / 1000.0 * 60.0 / (yaw[-1][0] - yaw[0][0])
            out.append(f"Raw yaw endpoint slope (deg/min): {drift:.6f}")
            out.append("WARNING: Endpoint slope assumes hardware stayed stationary within this session.")
        (self.directory / "summary.txt").write_text("\n".join(out) + "\n", encoding="utf-8")
        return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Serial port, e.g. COM7 or /dev/ttyUSB0")
    source.add_argument("--input", type=Path, help="Existing text log captured earlier (offline analysis)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=180.0,
                        help="Capture seconds; 0 means until Ctrl+C (default 180)")
    parser.add_argument("--out", type=Path, help="Directory for CSV, raw log and plots")
    parser.add_argument("--live", action="store_true", help="Optional live yaw plot during capture")
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error("--seconds must be >= 0")
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = args.out or Path("imu_logs") / stamp
    rec = Recorder(out)
    print(f"Output: {out.resolve()}")
    try:
        if args.input is not None:
            with args.input.open("r", encoding="utf-8", errors="replace") as source_file:
                for line in source_file:
                    fields = line.rstrip("\r\n").split("\t", 2)
                    if len(fields) == 3:
                        try:
                            recorded_elapsed = float(fields[1])
                        except ValueError:
                            recorded_elapsed = None
                        if recorded_elapsed is not None:
                            rec.receive(fields[2], offline=True,
                                        original_host_time=fields[0],
                                        original_elapsed=recorded_elapsed)
                            continue
                    rec.receive(line, offline=True)
            print(f"Imported {rec.line_no} lines from {args.input}")
        else:
            try:
                import serial
            except ImportError as exc:
                raise SystemExit("Missing pyserial. Run: python -m pip install pyserial matplotlib") from exc
            try:
                ser = serial.Serial(args.port, args.baud, timeout=0.25)
            except serial.SerialException as exc:
                raise SystemExit(f"Cannot open {args.port}: {exc}. Close PlatformIO Serial Monitor first.") from exc
            print(f"Capturing {args.port} at {args.baud} baud. Ctrl+C ends capture and plots.")
            if args.live:
                rec.init_live()
            start = time.monotonic()
            try:
                with ser:
                    while args.seconds == 0 or time.monotonic() - start < args.seconds:
                        byte_line = ser.readline()
                        if byte_line:
                            line = byte_line.decode("utf-8", errors="replace").rstrip("\r\n")
                            rec.receive(line)
                            print(line)
                        if args.live:
                            rec.update_live()
            except KeyboardInterrupt:
                print("\nCapture interrupted by user; saving acquired data.")
    finally:
        rec.close()
    result = rec.write_summary()
    plot_files = rec.make_plots()
    print("\n".join(result))
    print(f"Saved {len(plot_files)} plots: {', '.join(plot_files) if plot_files else '(none)'}")
    print(f"All files: {out.resolve()}")


if __name__ == "__main__":
    main()