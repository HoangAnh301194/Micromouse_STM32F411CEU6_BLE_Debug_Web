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
PHASE_SECONDS = 175.0   # firmware expected ~145s; host deadline <180s MCU hard limit
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
    """Encoder response consistency on each actual DRIVE interval (state=COAST).

    SYS_STEP state=2 is emitted at the end of the current DRIVE interval.
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


def planned_stage_times_ms(stage):
    pwm_step = stage % 11
    drive = (750 if pwm_step == 10 else
             1000 if pwm_step == 9 else
             3500 if pwm_step in (3, 6, 8) else 2000)
    coast = 3000 if pwm_step >= 9 else 2000
    return drive, coast


def inspect_extended_schedule(logs, mode, limited=False):
    """Validate drive/coast timing and capture residual inertia 500ms traces.

    SYS_STEP state=2 is the END of the current DRIVE interval.
    SYS_STEP state=1 for id>0 is the END of the PREVIOUS COAST.
    SYS_STEP state=3 is the end of stage 32's final COAST.
    """
    mode_num = 1 if mode == "ON" else 0
    steps = [r for r in logs if r.get("type") == "SYS_STEP"
             and r.get("mode") == mode_num]
    problems, summary = [], []
    coasts = [r for r in logs if r.get("type") == "SYS_COAST"]
    total=12 if limited and mode=="ON" else 33
    per_group=4 if limited and mode=="ON" else 11
    for stage in range(total):
        drive = next((r for r in steps if r.get("id") == stage
                      and r.get("state") == 2), None)
        coast = next((r for r in steps
                      if r.get("id") == stage + 1
                      and r.get("state") == (3 if stage == total-1 else 1)), None)
        planned_drive, planned_coast = planned_stage_times_ms(
            (stage // per_group) * 11 + stage % per_group)
        if not drive or not coast:
            problems.append({"stage": stage, "reason": "missing_interval"})
            continue
        observed_drive, observed_coast = drive.get("dt"), coast.get("dt")
        if not isinstance(observed_drive, int) or not isinstance(observed_coast, int):
            problems.append({"stage": stage, "reason": "missing_interval_timestamps"})
        else:
            if abs(observed_drive - planned_drive) > 250:
                problems.append({"stage": stage, "reason": "drive_timing",
                                 "planned": planned_drive, "observed": observed_drive})
            if abs(observed_coast - planned_coast) > 250:
                problems.append({"stage": stage, "reason": "coast_timing",
                                 "planned": planned_coast, "observed": observed_coast})
        series = [r for r in coasts if r.get("id") == stage
                  and isinstance(r.get("dt"), int)]
        if len(series) < 3:
            problems.append({"stage": stage, "reason": "missing_500ms_coast_samples",
                             "samples": len(series)})
        summary.append({
            "stage": stage, "wheel_group": stage // per_group,
            "pwm_percent": (stage % per_group) * 10,
            "drive_ms": observed_drive, "coast_ms": observed_coast,
            "drive_left_counts": drive.get("dl"),
            "drive_right_counts": drive.get("dr"),
            "coast_left_counts": coast.get("dl"),
            "coast_right_counts": coast.get("dr"),
            "coast_end_left_pps": coast.get("vL"),
            "coast_end_right_pps": coast.get("vR"),
            "coast_trace": [{"dt_ms": r.get("dt"),
                             "left_pps": r.get("ppsL"), "right_pps": r.get("ppsR"),
                             "left_counts": r.get("dl"), "right_counts": r.get("dr")}
                            for r in series],
        })
    # Positive forward encoder sign applies to the NEW integrated firmware only.
    if mode == "ON":
        for group, axis in ((0,"dl"),(1,"dr"),(2,"dl"),(2,"dr")):
            for step in ((2,3) if limited else (6,7)):
                stage = group * per_group + step
                candidate = next((x for x in summary if x["stage"] == stage),None)
                if candidate and candidate.get("drive_"+("left" if axis=="dl" else "right")+"_counts",0) < 0:
                    problems.append({"stage": stage, "reason": "forward_encoder_sign_negative",
                                     "wheel": axis})
    return {"endurance_pass": len(problems) == 0, "problems": problems,
            "stages": summary,
            "coast_intervals": len(coasts)}


def inspect_limited_drive(logs):
    """Strict 12-stage motor-on response for PWM<=30%, three wheel groups."""
    points={x.get("id"):x for x in logs
            if x.get("type")=="SYS_STEP" and x.get("mode")==1 and x.get("state")==2}
    anomalies=[]
    for group in range(3):
        for step in (2,3):
            row=points.get(group*4+step)
            for wheel in ((0,) if group==0 else (1,) if group==1 else (0,1)):
                field="dl" if wheel==0 else "dr"
                if row is None or row.get(field,0)<20:
                    anomalies.append({"group":group,"step":step,"wheel":wheel,
                        "reason":"missing_or_weak_encoder_response"})
    return {"drive_windows":len(points),"high_pwm_anomalies":anomalies,
            "qualified":len(points)==12 and not anomalies}

def evaluate(logs, mode, limited=False):
    steps = sorted(set(int(x["id"]) for x in logs
                       if x.get("type") == "SYS_STEP" and x.get("state") == 1
                       and x.get("mode") == (1 if mode == "ON" else 0)))
    completed = any(x.get("type") == "SYS_STEP" and x.get("state") == 3
                    and x.get("mode") == (1 if mode == "ON" else 0) for x in logs)
    last = lambda kind: next((x for x in reversed(logs) if x.get("type") == kind), {})
    timer, mpu, uart = last("SYS_T"), last("SYS_M"), last("SYS_U")
    uart_profile = last("SYS_UQ")
    uart_rx_detail = last("SYS_UR")
    shadow_queue = last("SYS_AQ")
    shadow_trace_present = any(
        x.get("type") in ("SYS_AB","SYS_AM") for x in logs
    )
    profile = last("CLOCK_PROFILE")
    faults = [x for x in logs if x.get("type") == "SYS_STEP" and x.get("state") == 4]
    limited_on = limited and mode=="ON"
    expected=12 if limited_on else 33
    encoder_review = (inspect_limited_drive(logs) if limited_on
                      else inspect_encoder_drive(logs, mode))
    extended=inspect_extended_schedule(logs, mode, limited=limited_on)
    checks = {
        "expected_unique_pwm_stages": steps == list(range(expected)),
        "33_unique_pwm_stages": steps == list(range(33)) if not limited_on else True,
        "limited_30pct_no_excess": (not limited_on or
            not any(x.get("type")=="SYS_STEP" and x.get("mode")==1 and
                    x.get("duty",0)>300 for x in logs)),
        "finished_and_locked": completed and mpu.get("locked") == 1,
        "clock_trim25": uart.get("trim") == 25 and uart.get("clkfault") == 0,
        "i2c_400k_nominal": uart.get("iclk") == 400000,
        "uart_no_rx_error": uart.get("rx_err", 0) == 0 and uart.get("rx_ov", 0) == 0,
        "uart_no_tx_drop": uart.get("tx_drop", 0) == 0,
        "uart_dma_no_error": uart.get("dma_err", 0) == 0,
        "shadow_spsc_no_loss": (not shadow_trace_present or
                               (bool(shadow_queue) and
                                shadow_queue.get("lost",1)==0 and
                                shadow_queue.get("edge",1)==0)),
        "mpu_no_i2c_error": mpu.get("i2cerr", 0) == 0 and mpu.get("to", 0) == 0,
        "no_tim11_gaps": timer.get("gap", 0) == 0,
        "tim11_irq_jitter_under_50us": timer.get("late", 1) == 0,
        "no_tim11_excess_duration": timer.get("maxcy", 96001) < 96000,
        "motor_service_every_tick": timer.get("motticks", -1) == timer.get("ticks", 0),
        "ir_scans_progress": timer.get("ir", 0) >= int(timer.get("ticks", 0) * .95),
        "gyro_fresh_after_calibration": mpu.get("age", 99999) <= 3000
                                      and timer.get("stale", 1) == 0
                                      and timer.get("valid", 0) > 10000,
        "no_motor_watchdog": mpu.get("wd", 1) == 0,
        "no_sweep_fault": not faults,
        "encoder_drive_response_consistent": encoder_review["qualified"] if mode == "ON"
            else encoder_review["drive_windows"] == 33,
        "extended_drive_coast_inertia": extended["endurance_pass"],
    }
    return {
        "mode": mode, "checks": checks,
        "software_pass": all(checks.values()),
        "stages": len(steps), "expected_stages":expected,
        "max_pwm":30 if limited_on else 100,
        "timer": timer, "mpu": mpu, "uart": uart,
        "clock_profile": profile, "faults": faults, "encoder_review": encoder_review,
        "uart_dma_cpu_profile": uart_profile,
        "uart_rx_error_detail": uart_rx_detail,
        "encoder_shadow_queue": shadow_queue,
        "uart_cpu_fraction_of_nominal_sysclk": (round(
           100 * (uart_profile.get("prncy",0)+uart_profile.get("dmacy",0)+
                  uart_profile.get("rxcy",0)) /
           (96000 * max(1,uart_profile.get("ms",1))),3)
           if uart_profile else None),
        "extended": extended,
        "limits": "RCC nominal frequency only; no oscilloscope or current measurement",
    }


class PhaseSession:
    def __init__(self, port, baudrate, name, outdir, limited=False):
        import serial
        self.ser = serial.Serial(port=port, baudrate=baudrate,
                                 timeout=0.1, write_timeout=0.4)
        self.name = name
        self.limited = limited and name == "ON"
        self.expected_stages = 12 if self.limited else 33
        self.start = time.monotonic()
        self.last_ping = 0.0
        self.ping_sent_count = 0
        self.max_ping_gap_ms = 0.0
        self.max_ping_write_ms = 0.0
        self.rows = []
        self.raw = (outdir / (name.lower() + "_uart.log")).open("w", encoding="utf-8")

    def send(self, command):
        self.ser.write((command + "\n").encode("ascii"))
        self.ser.flush()

    def poll(self, heartbeat=False):
        if heartbeat and time.monotonic() - self.last_ping >= 0.15:
            before = time.monotonic()
            if self.ping_sent_count:
                self.max_ping_gap_ms = max(
                    self.max_ping_gap_ms, (before-self.last_ping)*1000)
            self.send("PING")
            after = time.monotonic()
            self.max_ping_write_ms = max(
                self.max_ping_write_ms, (after-before)*1000)
            self.last_ping = after
            self.ping_sent_count += 1
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
        self.send("STATUS")
        status=self.wait_response("CMD_STATUS")
        if self.name=="ON":
            expected_pwm=30 if self.limited else 100
            if status.get("max_pwm")!=expected_pwm or status.get("total")!=self.expected_stages:
                raise RuntimeError("Wrong motor-on firmware safety profile: "+repr(status))
        self.send(f"MODE {self.name}")
        mode_ack = self.wait_response("CMD_MODE")
        if mode_ack.get("accepted") != 1:
            raise RuntimeError("MODE rejected by firmware: " + repr(mode_ack))
        self.send("START")
        start_ack = self.wait_response("CMD_START")
        if start_ack.get("accepted") != 1:
            raise RuntimeError("START rejected by firmware: " + repr(start_ack))
        print(f"{self.name}: started {self.expected_stages} steps; capturing all sensors and timers.")
        deadline = time.monotonic() + PHASE_SECONDS
        while time.monotonic() < deadline:
            data = self.poll(heartbeat=True)
            if data and data.get("type") == "SYS_STEP":
                group = ("LEFT","RIGHT","BOTH")
                selection = data.get("group",-1)
                wheel = group[selection] if selection in (0,1,2) else "UNKNOWN"
                state = data.get("state",-1)
                step_desc = ("DRIVE" if state == 1 else
                             "COAST" if state == 2 else
                             "DONE" if state == 3 else "FAULT")
                display_stage=(self.expected_stages if state==3 else data.get("id",-1)+1)
                if self.limited and (data.get("duty",0)>300 or data.get("id",0)>12):
                    raise RuntimeError("LIMITED firmware exceeded PWM/stage safety limit: "+repr(data))
                print(f"{self.name} stage {display_stage}/{self.expected_stages} | "
                      f"{wheel} | PWM={data.get('duty',0)/10:.0f}% | "
                      f"{step_desc}",flush=True)
                if data.get("state") == 4:
                    raise RuntimeError(f"{self.name} firmware abort: {data}")
                if data.get("state") == 3:
                    # Read the following 1Hz health packet before evaluation.
                    extra_until = time.monotonic() + 1.15
                    while time.monotonic() < extra_until:
                        self.poll()
                    return
        raise TimeoutError(f"{self.name} exceeded host {PHASE_SECONDS:.0f}s test deadline")

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
              "dt", "vL", "vR", "appL", "appR", "ppsL", "ppsR", "l", "r",
              "q", "peak", "accb", "doneb", "prncy", "dmacy", "rxcy", "rxb",
              "mode", "state", "id", "group", "step", "duty", "dl", "dr")
    with (outdir/"measurements.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=("phase",)+fields, extrasaction="ignore")
        w.writeheader()
        for mode, data in rows.items():
            for item in data:
                w.writerow({"phase": mode, **item})
    # Generic measurements.csv intentionally has a fixed schema and cannot
    # hold SYS_AB/SYS_AM alpha fields. Preserve complete parsed diagnostic
    # records in separate CSVs, so gitignored raw .log files are not the only
    # possible way to recover real alpha A/B data on the remote repository.
    for phase, data in rows.items():
        captured=[item for item in data
                  if item.get("type") in ("SYS_AB","SYS_AM","SYS_IPB","SYS_IB","SYS_UR","SYS_AQ")]
        if not captured:
            continue
        keys=("type","host_elapsed_s","ms","id","state","w","m","n",
              "x0","x1","x2","x3","x4","x5","x6","x7","x8","x9",
              "a05","a08","a10","a12","a15","a20","a30",
              "a40","a60","a80","b5","b10","b20",
              "idx","mode","a0","w0","id0","a1","w1","id1",
              "a2","w2","id2","a3","w3","id3","rec","sr1",
              "sr2","pin","after","err","ore","fe","ne","ov",
              "in","out","lost","edge","peak","q")
        with (outdir/("encoder_shadow_"+phase.lower()+".csv")).open(
                "w",newline="",encoding="utf-8") as handle:
            writer=csv.DictWriter(handle,fieldnames=keys,extrasaction="ignore")
            writer.writeheader()
            for item in captured:
                writer.writerow(item)



def review_saved_measurements(csv_file):
    """Analyze existing motor logs safely, without opening serial or flashing."""
    rows = {"OFF": [], "ON": []}
    numeric = {"ms","ticks","late","gap","maxcy","encmaxcy","motmaxcy",
               "motticks","valid","stale","ir","cal","seq","age",
               "i2cok","i2cerr","to","busy","state","id","group",
               "step","duty","dl","dr","locked","wd","dt","vL","vR",
               "appL","appR","ppsL","ppsR","q","peak","accb","doneb",
               "prncy","dmacy","rxcy","rxb","rx_err","rx_ov",
               "tx_drop","dma_err","trim","clkfault","iclk","mode","l","r"}
    with csv_file.open(newline="", encoding="utf-8-sig") as file:
        for row in csv.DictReader(file):
            phase = row.get("phase")
            if phase not in rows: continue
            data = {"type": row.get("type", "")}
            for key,value in row.items():
                if not value: continue
                if key in numeric:
                    try: data[key] = int(value)
                    except ValueError: data[key] = value
                else: data[key] = value
            rows[phase].append(data)
    results = [{"mode": phase,
                "encoder_review": inspect_encoder_drive(rows[phase], phase),
                "timer_last": next((r for r in reversed(rows[phase])
                                    if r.get("type") == "SYS_T"), {}),
                "mpu_last": next((r for r in reversed(rows[phase])
                                  if r.get("type") == "SYS_M"), {})}
               for phase in ("OFF", "ON") if rows[phase]]
    report = {"source": str(csv_file), "results": results,
              "note": "Offline CSV omits UART/motor lock counters. Interpret encoder and TIM11 fields only; no physical qualification."}
    output = csv_file.parent / "encoder_review_v2.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    needs_review = False
    for result in results:
        review = result["encoder_review"]
        if not review["qualified"]: needs_review = True
        print(result["mode"], "ENCODER_REVIEW_OK" if review["qualified"] else "ENCODER_NEEDS_REVIEW",
              "drive_windows", review["drive_windows"],
              "high_pwm_anomalies", len(review["high_pwm_anomalies"]))
        for issue in review["high_pwm_anomalies"]: print("  ", issue)
    print("Saved", output)
    return 2 if needs_review else 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", help="STM32 USART1 adapter, e.g. COM6")
    p.add_argument("--review-csv", type=Path,
                   help="Re-evaluate saved measurements.csv; no flashing, UART, or motor actions")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--flash", action="store_true",
                   help="Automatically upload each diagnostic firmware via ST-Link")
    p.add_argument("--encoder-telemetry", action="store_true",
                   help="Use encoder-specific OFF/ON builds with ~50Hz SYS_EN logs")
    p.add_argument("--motor-on", action="store_true",
                   help="EXPLICIT consent to physical 0..100%% motor-on phase")
    p.add_argument("--off-encmaxcy-limit", type=int, default=0,
                   help="Reject the upcoming ON phase if preceding OFF Encoder IRQ peak exceeds this many CPU cycles")
    p.add_argument("--limited-30", action="store_true",
                   help="Compile-time 30pct cap, 12 ON stages; requires --flash --motor-on --encoder-telemetry")
    p.add_argument("--mode", choices=["off", "on"], default="off",
                   help="Used when --flash is omitted; selected firmware must already be on MCU")
    p.add_argument("--outdir", type=Path)
    p.add_argument("--uart-only", action="store_true",
                   help="Profile motor-locked PA9/PA10 throughput only, no PWM sweep")
    p.add_argument("--with-uart", action="store_true",
                   help="Add PA9/PA10 throughput profile BEFORE the motor OFF/ON tests; requires --flash")
    args=p.parse_args()
    if args.review_csv is not None:
        return review_saved_measurements(args.review_csv)
    if not args.port:
        p.error("--port is required unless --review-csv is supplied")
    if args.uart_only and (args.motor_on or args.with_uart):
        p.error("--uart-only is mutually exclusive with --motor-on/--with-uart")
    if args.with_uart and not args.flash:
        p.error("--with-uart needs --flash to switch diagnostic firmware profiles")
    if args.encoder_telemetry and args.uart_only:
        p.error("--encoder-telemetry cannot be used with UART-only profile")
    if args.mode == "on" and not args.motor_on:
        p.error("--mode on requires --motor-on")
    if args.limited_30 and not (args.flash and args.motor_on and args.encoder_telemetry):
        p.error("--limited-30 requires --flash --motor-on --encoder-telemetry")
    if args.motor_on:
        print("LIVE MOTOR TEST REQUESTED: wheels must be secured off ground, "
              "motor supply independently switchable, no obstructions.")
    run_modes = ["OFF", "ON"] if args.motor_on and args.flash \
                else [args.mode.upper()] if not args.flash else ["OFF"]
    stamp=dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir=args.outdir or ROOT/"results"/("system_uart_"+stamp)
    outdir.mkdir(parents=True,exist_ok=True)
    if args.uart_only or args.with_uart:
        # Bench mode is always motor-output-locked; host explicitly flashes
        # the separate diagnostic image before any motion-capable firmware.
        from uart_rate_benchmark import run as run_uart_profile
        uart_report = run_uart_profile(args.port, outdir / "uart",
                                       upload=args.flash)
        if args.uart_only:
            print("Max tested stable UART rate:",
                  uart_report["chosen_tested_sustainable_hz"], "Hz")
            return 0 if uart_report["chosen_tested_sustainable_hz"] else 2
        if not uart_report["chosen_tested_sustainable_hz"]:
            print("UART throughput profile did not pass any tested rate; "
                  "motor-on phase is not launched.")
            return 2
    rows, results = {}, []
    try:
        for mode in run_modes:
            if args.flash:
                firmware = (("integration_diag_encoder_motor_on_limited_30_96m" if mode == "ON" and args.limited_30
                             else "integration_diag_encoder_motor_on_96m" if mode == "ON"
                             else "integration_diag_encoder_motor_off_96m")
                            if args.encoder_telemetry
                            else (MOTOR_ON_ENV if mode == "ON" else MOTOR_OFF_ENV))
                flash(firmware)
            session=None
            try:
                session=PhaseSession(args.port,args.baud,mode,outdir,limited=args.limited_30)
                session.run_phase()
                score=evaluate(session.rows,mode,limited=args.limited_30)
                score["host_heartbeat"]={
                    "ping_sent":session.ping_sent_count,
                    "ping_ack_received":sum(
                        row.get("type")=="CMD_PING" for row in session.rows),
                    "max_inter_ping_gap_ms":round(session.max_ping_gap_ms,2),
                    "max_ping_write_ms":round(session.max_ping_write_ms,2),
                    "note":"Measured by Python monotonic clock; an ACK logged "
                           "does not guarantee MCU processed every ping."}
                rows[mode]=session.rows
                results.append(score)
                print(mode, "PASS" if score["software_pass"] else "UNQUALIFIED",
                      "steps",score["stages"],
                      "failed_checks",[k for k,v in score["checks"].items() if not v])
                session.close()
                if mode=="OFF" and args.motor_on and (
                    not score["software_pass"] or
                    (args.off_encmaxcy_limit > 0 and
                     score.get("timer",{}).get("encmaxcy",999999) >= args.off_encmaxcy_limit)
                ):
                    session=None
                    raise RuntimeError(
                        "OFF prerequisite or Encoder IRQ timing FAILED; refusing MOTOR-ON: "+
                        repr({"software_pass":score["software_pass"],
                              "encmaxcy":score.get("timer",{}).get("encmaxcy"),
                              "limit":args.off_encmaxcy_limit}))
            except BaseException as exc:
                if session:
                    rows[mode]=session.rows
                    # Do not lose all ON health/fault evidence just because
                    # the firmware deliberately stopped before stage 33.
                    # evaluate() will mark the truncated sweep NOT qualified.
                    try:
                        aborted=evaluate(session.rows, mode,limited=args.limited_30)
                        aborted["host_heartbeat"]={
                            "ping_sent":session.ping_sent_count,
                            "ping_ack_received":sum(
                                row.get("type")=="CMD_PING" for row in session.rows),
                            "max_inter_ping_gap_ms":round(session.max_ping_gap_ms,2),
                            "max_ping_write_ms":round(session.max_ping_write_ms,2),
                            "note":"Host-local timing only; link-fault cause "
                                   "still requires command/ACK correlation."}
                        aborted["host_abort"]={
                            "exception":str(exc),
                            "last_firmware_fault":next(
                                (item for item in reversed(session.rows)
                                 if item.get("type")=="SYS_STEP" and
                                    item.get("state")==4),None),
                        }
                        aborted["software_pass"]=False
                        results.append(aborted)
                    except Exception as diagnostic_error:
                        results.append({
                            "mode":mode,"software_pass":False,
                            "stages":0,"faults":[],
                            "host_abort":{"exception":str(exc),
                                          "diagnostic_error":str(diagnostic_error)}
                        })
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
