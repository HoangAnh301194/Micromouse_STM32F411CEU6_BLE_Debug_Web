#!/usr/bin/env python3
"""Micromouse 180s MPU/I2C yaw-drift and full-system benchmark.
No hardware action without --flash. Default: TWO MOTOR-LOCKED profiles;
--motor-live additionally runs a separately gated, low-PWM wheels-lifted
session and must NEVER be used with wheels contacting a surface.
Review saved results with --review RESULTS.csv without touching hardware.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime
import json
from collections import deque
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent.parent
PROFILES=[("STILL","integration_diag_imu_soak_96m"),
          ("SHADOW","integration_diag_full_shadow_96m")]
LIVE=("LIVE","integration_diag_full_live_96m")
LIVE_HIGH=("LIVE","integration_diag_full_live_high_pwm_96m")
DURATION=180.0
BAUD=115200
CAL_TIMEOUT=60.0
BOOT_SYS_M_TIMEOUT=15.0  # MCU must stream SYS_M at 1Hz during calibration
RUN_TIMEOUT=190.0

def parse_line(line):
    parts=line.strip().split(",")
    if not parts or not(parts[0].startswith(("SOAK_","I2CP","IM6","IM1","IMN","IMIR","IMPID","IMDRV","SYS_","CMD_"))):
        return None
    rec={"type":parts[0]}
    for p in parts[1:]:
        if "=" not in p: continue
        k,v=p.split("=",1)
        if not k: continue
        try: rec[k]=int(v)
        except ValueError: rec[k]=v
    return rec

def summarise(items, mode):
    samples=[x for x in items if x.get("type")=="IM6"]
    secs=[x for x in items if x.get("type")=="IM1"]
    noise=[x for x in items if x.get("type")=="IMN"]
    pid=[x for x in items if x.get("type")=="IMPID"]
    ir=[x for x in items if x.get("type")=="IMIR"]
    motors=[x for x in items if x.get("type")=="IMDRV"]
    end=next((x for x in reversed(items) if x.get("type")=="SOAK_DONE"),{})
    begin=next((x for x in items if x.get("type")=="SOAK_BEGIN"),{})
    if len(samples)>=2:
        duration=(samples[-1].get("t",0)-samples[0].get("t",0))/1000.0
    else:duration=0.0
    changes={}
    for key,name in (("y","yaw_filtered"),("n","yaw_no_deadzone"),("r","yaw_raw_integrated")):
        if len(samples)>=2 and key in samples[0] and key in samples[-1]:
            drift=(samples[-1][key]-samples[0][key])/1000
            changes[name]={"start_deg":samples[0][key]/1000,
                           "end_deg":samples[-1][key]/1000,
                           "drift_deg":round(drift,6),
                           "drift_deg_per_min":round(drift/max(0.01,duration)*60,6)}
    gaps=[samples[i]["t"]-samples[i-1]["t"] for i in range(1,len(samples))
          if isinstance(samples[i].get("t"),int) and isinstance(samples[i-1].get("t"),int)]
    seqs=[samples[i]["s"]-samples[i-1]["s"] for i in range(1,len(samples))
          if isinstance(samples[i].get("s"),int) and isinstance(samples[i-1].get("s"),int)]
    def latest(a,k,default=None):return a[-1].get(k,default) if a else default
    temps=[x for x in noise if x.get("tvalid")==1]
    begin_is_high=begin.get("high")==1
    temp_mc=[x.get("tc") for x in temps if isinstance(x.get("tc"),int)]
    def correlation(x,y):
        if len(x)<3:return None
        ax=statistics.mean(x);ay=statistics.mean(y)
        sx=sum((v-ax)**2 for v in x);sy=sum((v-ay)**2 for v in y)
        if not sx or not sy:return None
        return round(sum((a-ax)*(b-ay) for a,b in zip(x,y))/(sx*sy)**.5,4)
    temp_corr=correlation([r["tc"]/1000 for r in temps],
                          [r["mean"]/1000 for r in temps])
    motor_requested=any(r.get("setL",0)>0 or r.get("setR",0)>0 for r in motors)
    motor_encoders_active=bool(motors) and (
       max((r.get("encL",0) for r in motors),default=0)-
       min((r.get("encL",0) for r in motors),default=0)>=500 or
       max((r.get("encR",0) for r in motors),default=0)-
       min((r.get("encR",0) for r in motors),default=0)>=500)
    high_levels={v for v in (800,900,1000) if any(
       (r.get("setL")==v or r.get("setR")==v) and
       (r.get("appL")==v or r.get("appR")==v) for r in motors)}
    checks={
       "completed_180s":end.get("ok")==1 and end.get("ms",0)>=180000,
       "samples_60hz_at_least_95pct":len(samples)>=int(180*60*.95),
       "no_telemetry_gaps_over_100ms":bool(gaps) and max(gaps)<=100,
       "fresh_gyro_at_least_900hz":latest(secs,"good",0)>=int(180*900),
       "no_gyro_stale":latest(secs,"stale",1)==0,
       "no_i2c_errors":latest(secs,"err",1)==0 and latest(secs,"to",1)==0,
       "no_tim11_gaps":latest(secs,"gap",1)==0 and latest(secs,"late",1)==0,
       "ir_adc_scans_present":len(ir)>160 and
           latest(ir,"c",0)>ir[0].get("c",0)+100000,
       "no_uart_tx_drop_or_rx_error":latest(secs,"drop",1)==0
           and latest(secs,"rxerr",1)==0,
       "pid_load_executed": mode=="STILL" or
           latest(pid,"n",0)>=int(180*1000*.95),
       "motor_control_requested_in_live": mode!="LIVE" or any(
           x.get("cmdL",0)>0 or x.get("cmdR",0)>0 for x in pid),
       "live_encoder_pulses_observed": mode!="LIVE" or (
           bool(pid) and (
               max((r.get("encL",0) for r in pid),default=0) -
               min((r.get("encL",0) for r in pid),default=0) >= 500 or
               max((r.get("encR",0) for r in pid),default=0) -
               min((r.get("encR",0) for r in pid),default=0) >= 500)),
       "gyro_sequence_monotonic":bool(seqs) and all(z>0 for z in seqs),
       "no_60hz_generator_stalls_over_40ms":bool(gaps) and max(gaps)<=40,
    }
    probes=[{k:v for k,v in x.items() if k not in ("host_elapsed_s","type")}
            for x in items if x.get("type")=="I2CP"]
    return {
        "mode":mode, "passed_telemetry_checks":all(checks.values()),
        "checks":checks,"captured_60hz_samples":len(samples),
        "captured_1hz_health":len(secs),"captured_1hz_noise":len(noise),
        "observed_window_s":round(duration,3),
        "mean_telemetry_gap_ms":round(statistics.mean(gaps),3) if gaps else None,
        "worst_telemetry_gap_ms":max(gaps,default=None),
        "seq_increment_nonpositive":sum(x<=0 for x in seqs),
        "telemetry_gaps_over_25ms":sum(x>25 for x in gaps),
        "sample_age_max_us":max((x.get("a",0) for x in samples),default=None),
        "yaw_drift":changes,
        "gyro_noise_mean_corrected_dps_mean":round(
           statistics.mean(x.get("mean",0) for x in noise)/1000,5) if noise else None,
        "gyro_noise_std_dps_max":max((x.get("std",0) for x in noise),default=0)/1000,
        "raw_sample_gap_max_us":max((x.get("gmax",0) for x in noise),default=None),
        "raw_sample_gap_min_us":min((x.get("gmin",0) for x in noise if x.get("n",0)>1),default=None),
        "raw_equal_pairs":sum(x.get("eq",0) for x in noise),
        "last_health":secs[-1] if secs else {},
        "last_pid":pid[-1] if pid else {},
        "initial_i2c_probe":probes,
        "i2c_nominal_hz":begin.get("iclk"),
        "completion":end,
        "limitations":[
          "Yaw drift is an IMU relative-heading observation, not external angular truth.",
          "In LIVE mode vibration and physical yaw may contribute; do not label it stationary bias.",
          "60Hz UART does not transport all 1000Hz IMU samples; on-device counters capture intervening faults.",
          "No external voltage, current, temperature, mechanical displacement or PWM waveform measurements."
        ],
    }

def flash(env):
    print("Building/flashing",env,flush=True)
    subprocess.run([sys.executable,"-m","platformio","run","-e",env,"-t","upload"],
                   cwd=ROOT,check=True,timeout=240)

class Session:
    def __init__(self,port,mode,outdir):
        import serial
        self.ser=serial.Serial(port,BAUD,timeout=.07,write_timeout=.5)
        self.mode=mode
        self.start=time.monotonic()
        self.last_ping=0
        self.records=[]
        self.raw=(outdir/(mode.lower()+"_uart.log")).open("w",encoding="utf-8")
        self.last_flush=time.monotonic()
        self.recent_raw=deque(maxlen=8)
        self.last_progress_s=-10
        print(f"[{mode}] Opened {port} at 115200 8N1; waiting for MPU calibration",
              flush=True)

    def send(self,s):
        self.ser.write((s+"\n").encode("ascii"))
        self.ser.flush()

    def poll(self,ping=False):
        if ping and time.monotonic()-self.last_ping>=.15:
            self.send("PING")
            self.last_ping=time.monotonic()
        raw=self.ser.readline().decode("utf-8","replace").strip()
        if not raw:return None
        self.raw.write(f"{time.monotonic()-self.start:.4f}\t{raw}\n")
        self.recent_raw.append(raw)
        if time.monotonic()-self.last_flush>=1.0:
            self.raw.flush()
            self.last_flush=time.monotonic()
        row=parse_line(raw)
        if row:
            row["host_elapsed_s"]=round(time.monotonic()-self.start,4)
            self.records.append(row)
        return row

    def until(self,kind,seconds,accepted=None,ping=False):
        limit=time.monotonic()+seconds
        while time.monotonic()<limit:
            r=self.poll(ping=ping)
            if not r:continue
            if r.get("type")=="SOAK_BOOT_FAIL":
                raise RuntimeError(f"{self.mode}: MCU boot failure: {r}")
            if kind=="SOAK_DONE" and r.get("type")=="IM1":
                secs=int(r.get("t",0))//1000
                if secs>=self.last_progress_s+10:
                    self.last_progress_s=secs
                    print(f"[{self.mode}] {secs:3d}/180s"
                          f" gyro_good={r.get('good','?')} I2Cerr={r.get('err','?')}"
                          f" I2Cto={r.get('to','?')} TIM11gap={r.get('gap','?')}"
                          f" UARTdrop={r.get('drop','?')}",flush=True)
            if r.get("type")!=kind:continue
            if accepted is not None and r.get("ok")!=accepted:
                raise RuntimeError(f"{self.mode} rejected: {r}")
            return r
        raise TimeoutError(f"{self.mode}: missing {kind} within {seconds}s;"
                           f" recent UART={list(self.recent_raw)}")

    def run(self):
        self.send("HELP")
        # SYS_M must emit once per second even while MPU is warming up.
        # Fail fast on silent boot, but allow calibration retries until 60s.
        deadline=time.monotonic()+CAL_TIMEOUT
        first_sysm_deadline=time.monotonic()+BOOT_SYS_M_TIMEOUT
        last_cal_state=None
        while time.monotonic()<deadline:
            r=self.poll()
            if r:
                if r.get("type")=="SOAK_BOOT_FAIL":
                    raise RuntimeError(f"{self.mode}: MCU boot error: {r}")
                if r.get("type")=="SYS_CAL":
                    print(f"[{self.mode}] calibration transition:"
                          f" state={r.get('state')} retries={r.get('retries')}"
                          f" reason={r.get('reason')}",flush=True)
                if r.get("type")=="SYS_M":
                    first_sysm_deadline=time.monotonic()+BOOT_SYS_M_TIMEOUT
                    cal=r.get("cal")
                    if cal!=last_cal_state:
                        print(f"[{self.mode}] cal_state={cal}",flush=True)
                        last_cal_state=cal
                    print(f"[{self.mode}] calibration: state={cal}"
                          f" n={r.get('n')} I2Cok={r.get('i2cok')}"
                          f" I2Cerr={r.get('i2cerr')}"
                          f" timeout={r.get('to')} seq={r.get('seq')}",
                          flush=True)
                    if cal==4:
                        raise RuntimeError(
                            f"{self.mode}: MPU calibration FAILED; "
                            f"status={r}; recent UART={list(self.recent_raw)}")
                    if cal==3:
                        print(f"[{self.mode}] MPU calibrated; preparing 180s soak",
                              flush=True)
                        break
            if time.monotonic()>first_sysm_deadline:
                raise TimeoutError(
                    f"{self.mode}: no SYS_M in {BOOT_SYS_M_TIMEOUT:.0f}s; "
                    "check COM port, PA9->USB RX, common GND, and MPU boot."
                    f" Recent UART={list(self.recent_raw)}")
        else:
            raise TimeoutError(
                f"{self.mode}: MPU not calibrated within {CAL_TIMEOUT:.0f}s;"
                f" recent UART={list(self.recent_raw)}")
        if self.mode=="LIVE":
            self.send("SOAK ARM LIVE")
            self.until("SOAK_ARM",5,accepted=1)
        self.send("SOAK START")
        self.until("SOAK_ACK",5,accepted=1)
        print(f"[{self.mode}] SOAK START accepted, measurement started",flush=True)
        done=self.until("SOAK_DONE",RUN_TIMEOUT,ping=True)
        if done.get("ok")!=1:
            raise RuntimeError(f"{self.mode}: session aborted by MCU: {done}")
        end=time.monotonic()+1.0
        while time.monotonic()<end:self.poll()
        report=summarise(self.records,self.mode)
        print(self.mode,"PASS" if report["passed_telemetry_checks"] else "REVIEW",
              "samples",report["captured_60hz_samples"],
              "drift",report["yaw_drift"].get("yaw_filtered"),
              "failed_checks",[k for k,v in report["checks"].items() if not v],
              flush=True)
        return report

    def close(self,emergency=False):
        try:
            if emergency:self.send("!")
        except Exception:pass
        try:self.ser.close()
        finally:self.raw.close()

def write_results(outdir,reports,logs):
    outdir.mkdir(exist_ok=True,parents=True)
    (outdir/"summary.json").write_text(json.dumps({
        "reports":reports,"hardware_physical_qualified":False,
        "note":"LIVE means bounded low-PWM actual motor commands. STATIC and SHADOW keep hardware outputs locked."
    },indent=2),encoding="utf-8")
    allrows=[{"phase":phase,**r} for phase,data in logs.items() for r in data]
    fields=["phase","type","host_elapsed_s"]+sorted(
        set().union(*(set(x) for x in allrows))-
        {"phase","type","host_elapsed_s"}) if allrows else ["phase","type","host_elapsed_s"]
    with (outdir/"measurements.csv").open("w",encoding="utf-8",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction="ignore")
        writer.writeheader();writer.writerows(allrows)

def review(path):
    by={}
    with path.open(encoding="utf-8-sig",newline="") as f:
        for r in csv.DictReader(f):
            mode=r.get("phase")
            if mode not in ("STILL","SHADOW","LIVE"):continue
            one={"type":r.get("type","")}
            for k,v in r.items():
                if not v or k in ("phase","type"):continue
                try:one[k]=int(v)
                except ValueError:
                    try:one[k]=float(v)
                    except ValueError:one[k]=v
            by.setdefault(mode,[]).append(one)
    reports=[summarise(rows,mode) for mode,rows in by.items()]
    out=path.with_name("soak_review.json")
    out.write_text(json.dumps(reports,indent=2),encoding="utf-8")
    print("Saved",out)
    for r in reports:print(r["mode"],r["passed_telemetry_checks"],r["yaw_drift"])
    return 0 if reports and all(x["passed_telemetry_checks"] for x in reports) else 2

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("--port",help="USART1 PA9/PA10 USB adapter, e.g. COM7")
    a.add_argument("--flash",action="store_true",help="Flash each explicitly selected firmware")
    a.add_argument("--motor-live",action="store_true",help="EXPLICIT wheels-lifted motor VM consent for third 180s benchmark")
    a.add_argument("--outdir",type=Path)
    a.add_argument("--review",type=Path,help="Offline CSV review; never connect or flash")
    args=a.parse_args()
    if args.review:return review(args.review)
    if not args.port or not args.flash:
        a.error("hardware capture requires --port and --flash; offline analysis uses --review")
    if args.motor_live:
        print("LIVE MOTOR: secure lifted wheels, use independent motor VM switch, supervise all 180s; max PWM 35%, coast >=3s.",flush=True)
    runs=PROFILES+([LIVE] if args.motor_live else [])
    name="imu_soak_"+datetime.now().strftime("%Y%m%d_%H%M%S")
    out=args.outdir or ROOT/"results"/name
    out.mkdir(parents=True,exist_ok=True)
    logs={};reports=[]
    try:
        for mode,env in runs:
            session=None
            flash(env)
            try:
                session=Session(args.port,mode,out)
                report=session.run()
                logs[mode]=session.records
                reports.append(report)
                session.close()
            except BaseException:
                if session:
                    logs[mode]=session.records
                    session.close(emergency=True)
                raise
    finally:
        write_results(out,reports,logs)
        print("Saved",out,flush=True)
    return 0 if all(x["passed_telemetry_checks"] for x in reports) else 2

if __name__=="__main__":
    raise SystemExit(main())
