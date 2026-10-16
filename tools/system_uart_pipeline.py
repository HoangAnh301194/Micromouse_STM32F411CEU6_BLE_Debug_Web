#!/usr/bin/env python3
"""Automated USART1 PA9/PA10 96MHz whole-system debug via pyserial.

Motor-OFF is default. To run physical motors, the operator must explicitly
provide --motor-on (and independently secure lifted wheels + motor supply).
Never energizes motors on serial connect alone. No hardware KEY required.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
MOTOR_OFF_ENV = "integration_diag_motor_off_96m"
MOTOR_ON_ENV = "integration_diag_motor_on_96m"
PHASE_SECONDS = 18.0
CAL_TIMEOUT = 50.0


def parse_line(s: str):
    fields = s.strip().split(",")
    if len(fields) < 2:
        return None
    out = {"type": fields[0]}
    for field in fields[1:]:
        if "=" in field:
            k, v = field.split("=", 1)
            if k:
                try: out[k] = int(v)
                except ValueError: out[k] = v
    return out



def inspect_encoder_drive(logs, mode):
    """Response consistency on the 250ms DRIVE portion only (state=COAST).
    
    SYS_STEP state=2 is emitted at the end of the 250ms DRIVE interval.
    Logs at state=1 refer to the preceding COAST and must NOT be interpreted
    as response to the new stage. Thresholds are relative diagnostic flags,
    not electrical/current or actual PWM measurement.
    """
    points = {}
    for rec in logs:
        if rec.get("type") != "SYS_STEP" or rec.get("state") != 2:
            continue
        if rec.get("mode") != (1 if mode == "ON" else 0):
            continue
        group = rec.get("group")
        step = rec.get("step")
        if group not in (0, 1, 2) or not isinstance(step, int) or not 0 <= step <= 10:
            continue
        if not isinstance(rec.get("dl"), int) or not isinstance(rec.get("dr"), int):
            continue
        points[(group, step)] = (rec["dl"], rec["dr"])
    suspicious = []
    pairs = []
    if mode == "ON":
        for group, axis in ((0, 0), (1, 1), (2, 0), (2, 1)):
            baseline = [abs(points[(group, i)][axis]) for i in (6, 7)
                        if (group, i) in points]
            if len(baseline) != 2 or min(baseline) < 20:
                suspicious.append({"group": group, "wheel": "L" if axis == 0 else "R",
                                   "reason": "baseline_at_60_70pct_unreliable"})
                continue
            threshold = max(5.0, 0.15 * (sum(baseline) / 2.0))
            for step in (8, 9, 10):
                pair = points.get((group, step))
                if pair is None:
                    suspicious.append({"group": group, "step": step,
                                       "wheel": "L" if axis == 0 else "R",
                                       "reason": "missing_drive_window"})
                elif abs(pair[axis]) < threshold:
                    suspicious.append({"group": group, "step": step,
                                       "duty_pct": step * 10,
                                       "wheel": "L" if axis == 0 else "R",
                                       "delta_counts": pair[axis],
                                       "threshold_counts": round(threshold, 1),
                                       "reason": "high_pwm_encoder_flatline"})
        # Direction of encoder counts is a separate calibration concern; do
        # not reverse signs automatically without checking wheel geometry.
        for group, axis in ((0, 0), (1, 1), (2, 0), (2, 1)):
            x = points.get((group, 6))
            y = points.get((group, 7))
            if x and y:
                pairs.append({"group": group, "wheel": "L" if axis == 0 else "R",
                              "sign_60pct": -1 if x[axis] < 0 else 1,
                              "sign_70pct": -1 if y[axis] < 0 else 1})
    return {"drive_windows": len(points), "high_pwm_anomalies": suspicious,
            "encoder_direction_observations": pairs,
            "qualified": len(points) == 33 and not suspicious}

def evaluate(logs, mode):
    steps = sorted(set(int(x["id"]) for x in logs
                       if x.get("type") == "SYS_STEP" and x.get("state") == 1
                       and x.get("mode") == (1 if mode == "ON" else 0)))
    completed = any(x.get("type") == "SYS_STEP" and x.get("state") == 3
                    and x.get("mode") == (1 if mode == "ON" else 0) for x in logs)
    last = lambda kind: next((x for x in reversed(logs) if x.get("type") == kind), {})
    timer, mpu, uart = last("SYS_T"), last("SYS_M"), last("SYS_U")
    profile = last("CLOCK_PROFILE")
    faults = [x for x in logs if x.get("type") == "SYS_STEP" and x.get("state") == 4]
    encoder_review = inspect_encoder_drive(logs, mode)
    checks = {
        "33_unique_pwm_stages": steps == list(range(33)),
        "finished_and_locked": completed and mpu.get("locked") == 1,
        "clock_trim25": uart.get("trim") == 25 and uart.get("clkfault") == 0,
        "i2c_400k_nominal": uart.get("iclk") == 400000,
        "uart_no_rx_error": uart.get("rx_err", 0) == 0 and uart.get("rx_ov", 0) == 0,
        "uart_no_tx_drop": uart.get("tx_drop", 0) == 0,
        "uart_dma_no_error": uart.get("dma_err", 0) == 0,
        "mpu_no_i2c_error": mpu.get("i2cerr", 0) == 0 and mpu.get("to", 0) == 0,
        "no_tim11_gaps": timer.get("gap", 0) == 0,
        "no_tim11_excess_duration": timer.get("maxcy", 96001) < 96000,
        "no_motor_watchdog": mpu.get("wd", 1) == 0,
        "no_sweep_fault": not faults,
        "encoder_drive_response_consistent": encoder_review["qualified"] if mode == "ON"
            else encoder_review["drive_windows"] == 33,
    }
    return {
        "mode": mode, "checks": checks,
        "software_pass": all(checks.values()),
        "stages": len(steps), "timer": timer, "mpu": mpu, "uart": uart,
        "clock_profile": profile, "faults": faults, "encoder_review": encoder_review,
        "limits": "RCC nominal frequency only; no oscilloscope or current measurement",
    }


class PhaseSession:
    def __init__(self, port, baudrate, name, outdir):
        import serial
        self.ser = serial.Serial(port=port, baudrate=baudrate,
                                 timeout=0.1, write_timeout=0.4)
        self.name = name
        self.start = time.monotonic()
        self.last_ping = 0.0
        self.rows = []
        self.raw = (outdir / (name.lower() + "_uart.log")).open("w", encoding="utf-8")

    def send(self, command):
        self.ser.write((command + "\n").encode("ascii"))
        self.ser.flush()

    def poll(self, heartbeat=False):
        if heartbeat and time.monotonic() - self.last_ping >= 0.15:
            self.send("PING")
            self.last_ping = time.monotonic()
        raw = self.ser.readline().decode("utf-8", errors="replace").strip()
        if not raw: return None
        self.raw.write(f"{time.monotonic()-self.start:.4f}\t{raw}\n")
        self.raw.flush()
        parsed = parse_line(raw)
        if parsed:
            parsed["host_elapsed_s"] = round(time.monotonic() - self.start, 4)
            self.rows.append(parsed)
        return parsed

    def wait_response(self, expected, timeout=6.0):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            item = self.poll()
            if item and item.get("type") == expected: return item
        raise RuntimeError(f"UART {self.name}: missing {expected}")

    def calibrate(self):
        self.send("HELP")
        deadline = time.monotonic() + CAL_TIMEOUT
        next_status = 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_status:
                self.send("STATUS")
                next_status = time.monotonic() + 0.8
            item = self.poll()
            if item and item.get("type") in ("CMD_STATUS", "SYS_M"):
                state = item.get("cal")
                if state == 3: return
                if state == 4: raise RuntimeError("MPU calibration permanently failed")
        raise RuntimeError("MPU not calibrated; no motor command was sent")

    def run_phase(self):
        self.calibrate()
        self.send(f"MODE {self.name}")
        mode_ack = self.wait_response("CMD_MODE")
        if mode_ack.get("accepted") != 1:
            raise RuntimeError("MODE rejected by firmware: " + repr(mode_ack))
        self.send("START")
        start_ack = self.wait_response("CMD_START")
        if start_ack.get("accepted") != 1:
            raise RuntimeError("START rejected by firmware: " + repr(start_ack))
        print(f"{self.name}: started 33 steps; capturing all sensors and timers.")
        deadline = time.monotonic() + PHASE_SECONDS
        while time.monotonic() < deadline:
            data = self.poll(heartbeat=True)
            if data and data.get("type") == "SYS_STEP":
                if data.get("state") == 4:
                    raise RuntimeError(f"{self.name} firmware abort: {data}")
                if data.get("state") == 3:
                    # Read the following 1Hz health packet before evaluation.
                    extra_until = time.monotonic() + 1.15
                    while time.monotonic() < extra_until:
                        self.poll()
                    return
        raise TimeoutError(f"{self.name} exceeded host 18s test deadline")

    def close(self, emergency=False):
        if emergency:
            try:
                self.send("!")
                self.send("STOP")
            except Exception: pass
        try: self.ser.close()
        finally: self.raw.close()


def flash(env):
    print("Building/flashing explicitly selected", env)
    subprocess.run([sys.executable, "-m", "platformio", "run",
                    "-e", env, "-t", "upload"], cwd=ROOT,
                   check=True, timeout=240)


def write_output(outdir, results, rows):
    (outdir/"summary.json").write_text(
        json.dumps({"results": results, "physically_qualified": False},
                   indent=2, ensure_ascii=False), encoding="utf-8")
    fields = ("type", "host_elapsed_s", "ms", "ticks", "late", "gap",
              "maxcy", "encmaxcy", "motmaxcy", "motticks", "valid", "stale",
              "ir", "cal", "seq", "age", "i2cok", "i2cerr", "to", "busy",
              "mode", "state", "id", "group", "step", "duty", "dl", "dr")
    with (outdir/"measurements.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=("phase",)+fields, extrasaction="ignore")
        w.writeheader()
        for mode, data in rows.items():
            for item in data:
                w.writerow({"phase": mode, **item})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True, help="STM32 USART1 adapter, e.g. COM6")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--flash", action="store_true",
                   help="Automatically upload each diagnostic firmware via ST-Link")
    p.add_argument("--motor-on", action="store_true",
                   help="EXPLICIT consent to physical 0..100%% motor-on phase")
    p.add_argument("--mode", choices=["off", "on"], default="off",
                   help="Used when --flash is omitted; selected firmware must already be on MCU")
    p.add_argument("--outdir", type=Path)
    args=p.parse_args()
    if args.mode == "on" and not args.motor_on:
        p.error("--mode on requires --motor-on")
    if args.motor_on:
        print("LIVE MOTOR TEST REQUESTED: wheels must be secured off ground, "
              "motor supply independently switchable, no obstructions.")
    run_modes = ["OFF", "ON"] if args.motor_on and args.flash \
                else [args.mode.upper()] if not args.flash else ["OFF"]
    stamp=dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir=args.outdir or ROOT/"results"/("system_uart_"+stamp)
    outdir.mkdir(parents=True,exist_ok=True)
    rows, results = {}, []
    try:
        for mode in run_modes:
            if args.flash:
                flash(MOTOR_ON_ENV if mode == "ON" else MOTOR_OFF_ENV)
            session=None
            try:
                session=PhaseSession(args.port,args.baud,mode,outdir)
                session.run_phase()
                score=evaluate(session.rows,mode)
                rows[mode]=session.rows
                results.append(score)
                print(mode, "PASS" if score["software_pass"] else "UNQUALIFIED",
                      "steps",score["stages"],
                      "failed_checks",[k for k,v in score["checks"].items() if not v])
                session.close()
            except BaseException:
                if session: 
                    rows[mode]=session.rows
                    session.close(emergency=True)
                raise
    finally:
        write_output(outdir,results,rows)
        print("Saved logs and analysis:",outdir)
    if len(results)==2:
        off,on=results
        print("Paired OFF/ON TIM11 late deltas:",
              off["timer"].get("late"), "->",on["timer"].get("late"))
        print("Paired OFF/ON gyro stale:",
              off["timer"].get("stale"), "->",on["timer"].get("stale"))
    return 0 if results and all(r["software_pass"] for r in results) else 1


if __name__=="__main__":
    raise SystemExit(main())
