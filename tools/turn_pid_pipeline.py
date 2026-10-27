#!/usr/bin/env python3
"""PA0-only 90/180-degree turn PID capture, offline metrics and PNG plots.

Serial NEVER starts a motor. It may only select idle angle/profile and STOP.
No numpy required. Requires pyserial for live capture; matplotlib is optional.
Typical:
  python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile base --runs 3
  python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile fast --runs 3
  python tools/turn_pid_pipeline.py --analyze results/turn_pid_YYYYmmdd_HHMMSS
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import sys
import time
import zipfile

FIELD_NAMES = (
    "run_id", "t_ms", "target_deg", "profile", "phase",
    "yaw_deg", "error_deg", "gyro_dps", "ref_dps",
    "pid_pm", "ff_pm", "cmd_pm", "applied_l_pm", "applied_r_pm",
    "enc_l", "enc_r", "encoder_l_pps", "encoder_r_pps",
    "imu_age_us", "motor_stage"
)
REPORT_FIELDS = (
    "run_id", "target_deg", "profile", "reason",
    "t10_ms", "t90_ms", "rise_10_90_ms", "t_first_target_ms",
    "stop_ms", "settling_ms", "peak_yaw_deg", "overshoot_deg",
    "overshoot_percent", "max_abs_gyro_dps", "final_yaw_deg",
    "final_abs_error_deg", "coast_drift_deg", "trace_samples",
    "sampling_gap_max_ms", "ring_drops", "uart_drops",
    "meets_trace_thresholds", "imu_effective_hz", "imu_intervals_max_us",
    "imu_intervals_mean_us", "imu_gap_gt1500", "imu_gap_gt2000",
    "imu_observer_missed", "imu_status_notready", "imu_i2c_errors",
    "imu_age_mean_us", "imu_age_max_us",
    "max_requested_pwm_pm", "max_applied_pwm_pm", "encoder_imbalance_pct",
    "motor_sweep_steps", "terminal_80_to_stop_ms",
    "kick_samples", "smooth_approach_samples", "reverse_pwm_samples",
    "rapid_accel_samples", "rapid_brake_samples", "rapid_near_samples",
    "rapid_coast_samples", "peak_gyro_1khz_dps",
    "flow_accel_samples","flow_brake_samples",
    "flow_coast_samples","flow_approach_samples","flow_finish_samples",
    "rapid_or_flow_brake_reaccelerations",
    "glide_accel_samples","glide_brake_samples",
    "glide_coast_samples","glide_approach_samples",
    "glide_finish_samples","brake_first_yaw_deg",
    "max_cmd_delta_20ms","trace_first_ms",
    "trace_terminal_disagreement_deg",
    "sprint_accel_samples","sprint_brake_samples",
    "sprint_coast_samples","sprint_approach_samples",
    "sprint_finish_samples","terminal_last10_to_stop_ms",
    "track_accel_samples","track_brake_samples",
    "track_near_samples","track_settle_samples",
    "midturn_low_speed_samples","midturn_reverse_samples",
    "min_midturn_gyro_dps",
    "guard_forward_samples","guard_brake_samples",
    "guard_near_samples","guard_stop_samples",
    "guard_active_band_brake_samples",
    "in_target_band_high_rate_samples","first_band_rate_dps",
    "pace_drive_samples","pace_brake_samples",
    "pace_near_samples","pace_stop_samples","pace_band_brake_samples",
    "coast_drive_samples","coast_neutral_samples",
    "coast_near_samples","coast_stop_samples",
    "coast_band_neutral_samples","coast_negative_command_samples",
    "command_polarity_flips","applied_pwm_max_step_20ms",
    "neutral_actual_high_gyro_samples"
)

IMU_FIELDS = (
    "run_id", "t_ms", "window_ms", "new_samples", "effective_hz",
    "interval_mean_us", "interval_min_us", "interval_max_us",
    "gap_gt1500", "gap_gt2000", "observer_missed", "status_notready",
    "i2c_errors", "last_age_us", "last_seq", "is_final"
)
def parse_imu(line: str) -> dict | None:
    """Decode one MCU timing window; no inference from UART arrival gaps."""
    if not line.startswith("TP,IMU,"):
        return None
    data=parse_kv(line,"TP,IMU,")
    keys=("id","t","win","new","hz10","dtavg","dtmin","dtmax",
          "g1500","g2000","miss","nr","err","age","seq","last")
    try:
        v={k:int(data[k]) for k in keys}
    except (KeyError,ValueError):
        return None
    if v["id"]<=0 or v["win"]<=0 or v["new"]<0 or v["last"] not in (0,1):
        return None
    return dict(run_id=v["id"],t_ms=v["t"],window_ms=v["win"],
                new_samples=v["new"],effective_hz=v["hz10"]/10.0,
                interval_mean_us=v["dtavg"],interval_min_us=v["dtmin"],
                interval_max_us=v["dtmax"],gap_gt1500=v["g1500"],
                gap_gt2000=v["g2000"],observer_missed=v["miss"],
                status_notready=v["nr"],i2c_errors=v["err"],
                last_age_us=v["age"],last_seq=v["seq"],
                is_final=v["last"])




def parse_sample(line: str) -> dict | None:
    """One producer-timestamped gyro/PWM sample; rejects partial UART frames."""
    parts = line.strip().split(",")
    if len(parts) not in (17, 20) or parts[:2] != ["TP", "D"]:
        return None
    try:
        raw = [int(i) for i in parts[2:]]
    except ValueError:
        return None
    if len(raw)==15:  # v1/v2 historical trace
        (run_id,t_ms,target,profile,phase,yaw_cd,err_cd,gyro_tenth,
         ref_tenth,pid_pm,ff_pm,cmd_pm,left,right,age_us)=raw
        applied_l=applied_r=motor_stage=None
    else:  # v3: applied motor commands + sweep stage
        (run_id,t_ms,target,profile,phase,yaw_cd,err_cd,gyro_tenth,
         ref_tenth,pid_pm,ff_pm,cmd_pm,applied_l,applied_r,
         left,right,age_us,motor_stage)=raw
    if run_id <= 0 or not 0 <= t_ms <= 100_000 or target not in (0,90,180):
        return None
    if profile not in (0,1,2,3,4,5,6,7,8,9,10,11) or phase not in (1,2):
        return None
    if abs(cmd_pm) > (890 if profile==11 and target==90 else (940 if profile==11 else (850 if profile==10 and target==90 else (920 if profile==10 else (750 if profile==9 and target==90 else (850 if profile==9 else (1000 if profile in (7,8) else (800 if profile in (4,5,6) else 400)))))))) or age_us < 0:
        return None
    if applied_l is not None and max(abs(applied_l),abs(applied_r))>1000:
        return None
    if motor_stage is not None:
        if not 0 <= motor_stage <= 46:
            return None
        if profile == 11:
            if motor_stage not in (0,42,43,44,45,46):return None
            if cmd_pm < 0:return None
        elif profile == 10:
            if motor_stage not in (0,37,38,39,40,41):return None
        elif profile == 9:
            if motor_stage not in (0,32,33,34,35,36):return None
        elif profile == 8:
            if motor_stage not in (0,28,29,30,31):return None
        elif profile == 7:
            if motor_stage not in (0,23,24,25,26,27):return None
        elif profile == 6:
            if motor_stage not in (0,18,19,20,21,22):return None
        elif profile == 5:
            if motor_stage not in (0,13,14,15,16,17):return None
        elif profile == 4:
            if motor_stage > 12:return None
        elif motor_stage > 8:
            return None
    return dict(run_id=run_id,t_ms=t_ms,target_deg=target,
                profile=profile,phase=phase,yaw_deg=yaw_cd/100,
                error_deg=err_cd/100,gyro_dps=gyro_tenth/10,
                ref_dps=ref_tenth/10,pid_pm=pid_pm,ff_pm=ff_pm,
                cmd_pm=cmd_pm,applied_l_pm=applied_l,
                applied_r_pm=applied_r,motor_stage=motor_stage,
                enc_l=left,enc_r=right,imu_age_us=age_us)


def parse_kv(line: str, prefix: str) -> dict:
    if not line.startswith(prefix):
        return {}
    result = {}
    for item in line[len(prefix):].strip().split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value
    return result


def interpolate_crossing(samples: list[dict], threshold: float) -> float | None:
    """First upward threshold crossing, interpolated between 50Hz points."""
    previous = None
    for item in sorted(samples, key=lambda x: x["t_ms"]):
        if previous is not None:
            a, b = previous["yaw_deg"], item["yaw_deg"]
            if a < threshold <= b and b > a:
                return previous["t_ms"] + (item["t_ms"] - previous["t_ms"]) * (
                    threshold - a) / (b - a)
        elif item["yaw_deg"] >= threshold:
            # Initial part of the trace is missing: the crossing time
            # cannot be reconstructed from the first UART sample.
            return None
        previous = item
    return None



MOTOR_SWEEP_PWM = (70, 95, 120, 155, 200, 260)
MOTOR_ON_MS=100
MOTOR_SEG_MS=250
MOTOR_FIELDS=("stage","pwm_requested_pm","trace_points","peak_gyro_dps",
              "mean_gyro_dps","max_applied_l_pm","max_applied_r_pm",
              "mean_applied_l_pm","mean_applied_r_pm",
              "mean_left_pps","mean_right_pps",
              "left_encoder_delta","right_encoder_delta",
              "signed_encoder_imbalance_pct","start_yaw_deg","end_yaw_deg")

def motor_sweep_steps(samples: list[dict]) -> list[dict]:
    """Bounded open-loop pulse response, not a fitted motor model."""
    steps=[]
    for stage,pwm in enumerate(MOTOR_SWEEP_PWM):
        start=stage*MOTOR_SEG_MS
        active=[s for s in samples
                if s["phase"]==1 and start<=s["t_ms"]<start+MOTOR_ON_MS
                and s.get("motor_stage")==stage]
        if not active:
            continue
        left=active[-1]["enc_l"]-active[0]["enc_l"]
        right=active[-1]["enc_r"]-active[0]["enc_r"]
        denom=abs(left)+abs(right)
        step={"stage":stage,"pwm_requested_pm":pwm,
              "trace_points":len(active),
              "peak_gyro_dps":round(max(abs(s["gyro_dps"]) for s in active),2),
              "mean_gyro_dps":round(statistics.mean(s["gyro_dps"] for s in active),2),
              "max_applied_l_pm":max(abs(s.get("applied_l_pm") or 0) for s in active),
              "max_applied_r_pm":max(abs(s.get("applied_r_pm") or 0) for s in active),
              "mean_applied_l_pm":round(statistics.mean(abs(s.get("applied_l_pm") or 0) for s in active),2),
              "mean_applied_r_pm":round(statistics.mean(abs(s.get("applied_r_pm") or 0) for s in active),2),
              "mean_left_pps":round(statistics.mean(s.get("encoder_l_pps") or 0 for s in active),2),
              "mean_right_pps":round(statistics.mean(s.get("encoder_r_pps") or 0 for s in active),2),
              "left_encoder_delta":left,"right_encoder_delta":right,
              "signed_encoder_imbalance_pct":round(100*abs(left+right)/denom,2) if denom else None,
              "start_yaw_deg":active[0]["yaw_deg"],
              "end_yaw_deg":active[-1]["yaw_deg"]}
        steps.append(step)
    return steps

def analyze(samples: list[dict], result: dict,
            imu_records: list[dict] | None = None) -> dict:
    if not samples:
        return {"error": "No valid TP,D frames", **result}
    rows = sorted((s for s in samples if s["run_id"] == samples[0]["run_id"]),
                  key=lambda s: s["t_ms"])
    target = int(rows[0]["target_deg"])
    applied_pwm=[abs(x) for s in rows for x in (s.get("applied_l_pm"),s.get("applied_r_pm")) if x is not None]
    final_l,final_r=rows[-1]["enc_l"],rows[-1]["enc_r"]
    enc_den=abs(final_l)+abs(final_r)
    encoder_imbalance=100*abs(final_l+final_r)/enc_den if enc_den else None
    if target == 0:
        steps=motor_sweep_steps(rows)
        return dict(run_id=rows[0]["run_id"],target_deg=0,profile="SWEEP",
                    reason=result.get("reason","PARTIAL"),
                    stop_ms=int(result.get("stop_ms",0)),
                    trace_samples=len(rows),motor_sweep_steps=len(steps),
                    max_abs_gyro_dps=round(max(abs(s["gyro_dps"]) for s in rows),3),
                    final_yaw_deg=rows[-1]["yaw_deg"],
                    max_requested_pwm_pm=max(abs(s["cmd_pm"]) for s in rows),
                    max_applied_pwm_pm=max(applied_pwm) if applied_pwm else None,
                    encoder_imbalance_pct=round(encoder_imbalance,3) if encoder_imbalance is not None else None,
                    ring_drops=int(result.get("qdrop",0)),
                    uart_drops=int(result.get("uartdrop",0)),
                    meets_trace_thresholds=bool(result.get("reason")=="DONE" and
                      len(steps)==6 and len(rows)>=50 and int(result.get("qdrop",0))==0
                      and int(result.get("uartdrop",0))==0))
    t10 = interpolate_crossing(rows, target * 0.10)
    t90 = interpolate_crossing(rows, target * 0.90)
    tfull = interpolate_crossing(rows, float(target))
    peak = max(s["yaw_deg"] for s in rows)
    if result.get("peak_cd") is not None:
        peak=max(peak,int(result["peak_cd"])/100.0)
    max_rate = max(abs(s["gyro_dps"]) for s in rows)
    # MCU RESULT is captured at 1kHz and can follow the last 50Hz
    # telemetry sample. It is authoritative for terminal yaw.
    final_yaw = (int(result["final_cd"])/100.0
                 if result.get("final_cd") is not None
                 else rows[-1]["yaw_deg"])
    terminal_disagreement = abs(final_yaw-rows[-1]["yaw_deg"])
    overshoot = max(0.0, peak - target)
    stop_ms = int(result["stop_ms"]) if result.get("stop_ms") else None
    reach_80 = next((v["t_ms"] for v in rows if v["phase"]==1 and v["yaw_deg"]>=80.0),None)
    reach_final_10 = next((v["t_ms"] for v in rows if v["phase"]==1
                           and v["yaw_deg"]>=target-10.0),None)
    at_stop = min(rows, key=lambda s: abs(s["t_ms"] - stop_ms)) if stop_ms is not None else None
    coast_drift = round(final_yaw - at_stop["yaw_deg"], 3) if at_stop else None
    gaps = [b["t_ms"]-a["t_ms"] for a,b in zip(rows,rows[1:])]
    ordered=[r for r in rows if r["phase"]==1]
    braking_stages=(10,14,19,24)
    first_brake=next((r for r in ordered if r.get("motor_stage") in braking_stages),None)
    cmd_deltas=[abs(y["cmd_pm"]-x["cmd_pm"])
                for x,y in zip(ordered,ordered[1:])
                if 0<y["t_ms"]-x["t_ms"]<=20]
    signs=[(1 if v["cmd_pm"]>0 else -1) for v in ordered
           if v["cmd_pm"]!=0]
    polarity_flips=sum(a!=b for a,b in zip(signs,signs[1:]))
    motor_steps=[abs(y["applied_r_pm"]-x["applied_r_pm"])
                 for x,y in zip(ordered,ordered[1:])
                 if x["applied_r_pm"] is not None and
                    y["applied_r_pm"] is not None and
                    0<y["t_ms"]-x["t_ms"]<=20]
    high_gyro_neutral=sum(1 for v in ordered
                   if abs(v["gyro_dps"])>100.0 and
                      v["applied_l_pm"]==0 and v["applied_r_pm"]==0)
    midturn=[r for r in ordered
             if r["yaw_deg"]>target*0.30 and
                r["yaw_deg"]<target-10.0]
    slow_mid=[r for r in midturn if abs(r["gyro_dps"])<15.0]
    reverse_mid=[r for r in midturn if r["gyro_dps"]<-3.0]
    in_band=[r for r in ordered if abs(r["error_deg"])<=2.0]
    fast_in_band=[r for r in in_band if abs(r["gyro_dps"])>12.0]
    imu_records = imu_records or []
    valid_imu = [w for w in imu_records if w["run_id"]==rows[0]["run_id"]]
    duration_ms=sum(w["window_ms"] for w in valid_imu)
    new_samples=sum(w["new_samples"] for w in valid_imu)
    mean_gyro_dt=(sum(w["interval_mean_us"]*w["new_samples"] for w in valid_imu)
                  /new_samples) if new_samples else None
    sample_ages=[row["imu_age_us"] for row in rows]
    settling = None
    # "settled" means ALL remaining samples stay within +-2 degrees
    # for at least the final 150 ms. An isolated one-sample crossing
    # is not a genuine settling event.
    if rows[-1]["t_ms"] - rows[0]["t_ms"] >= 150:
        for i,s in enumerate(rows):
            if rows[-1]["t_ms"]-s["t_ms"] < 150:
                break
            if all(abs(t["yaw_deg"]-target) <= 2.0 for t in rows[i:]):
                settling = float(s["t_ms"])
                break
    def rnd(x):
        return None if x is None else round(float(x), 3)
    return {
        "run_id": rows[0]["run_id"],
        "target_deg": target,
        "profile": {0:"BASE",1:"FAST",2:"TUNED",3:"SMOOTH",4:"RAPID",5:"FLOW",6:"GLIDE",7:"SPRINT",8:"TRACK",9:"GUARD",10:"PACE",11:"COAST"}[rows[0]["profile"]],
        "reason": result.get("reason", "PARTIAL"),
        "t10_ms": rnd(t10),
        "t90_ms": rnd(t90),
        "rise_10_90_ms": rnd(t90-t10) if t10 is not None and t90 is not None else None,
        "t_first_target_ms": rnd(tfull),
        "stop_ms": stop_ms,
        "settling_ms": rnd(settling),
        "peak_yaw_deg": rnd(peak),
        "overshoot_deg": rnd(overshoot),
        "overshoot_percent": rnd(overshoot*100.0/target),
        "max_abs_gyro_dps": rnd(max_rate),
        "final_yaw_deg": rnd(final_yaw),
        "final_abs_error_deg": rnd(abs(target-final_yaw)),
        "coast_drift_deg": coast_drift,
        "terminal_80_to_stop_ms": (stop_ms-reach_80) if stop_ms is not None and reach_80 is not None else None,
        "terminal_last10_to_stop_ms": (stop_ms-reach_final_10)
            if stop_ms is not None and reach_final_10 is not None else None,
        "kick_samples": sum(v.get("motor_stage")==6 for v in rows if v["phase"]==1),
        "smooth_approach_samples": sum(v.get("motor_stage")==7 for v in rows if v["phase"]==1),
        "reverse_pwm_samples": sum(v["cmd_pm"]<0 for v in rows if v["phase"]==1),
        "rapid_accel_samples": sum(v.get("motor_stage")==9 for v in rows if v["phase"]==1),
        "rapid_brake_samples": sum(v.get("motor_stage")==10 for v in rows if v["phase"]==1),
        "rapid_near_samples": sum(v.get("motor_stage")==11 for v in rows if v["phase"]==1),
        "rapid_coast_samples": sum(v.get("motor_stage")==12 for v in rows if v["phase"]==1),
        "flow_accel_samples": sum(v.get("motor_stage")==13 for v in rows if v["phase"]==1),
        "flow_brake_samples": sum(v.get("motor_stage")==14 for v in rows if v["phase"]==1),
        "flow_coast_samples": sum(v.get("motor_stage")==15 for v in rows if v["phase"]==1),
        "flow_approach_samples": sum(v.get("motor_stage")==16 for v in rows if v["phase"]==1),
        "flow_finish_samples": sum(v.get("motor_stage")==17 for v in rows if v["phase"]==1),
        "glide_accel_samples": sum(v.get("motor_stage")==18 for v in ordered),
        "glide_brake_samples": sum(v.get("motor_stage")==19 for v in ordered),
        "glide_coast_samples": sum(v.get("motor_stage")==20 for v in ordered),
        "glide_approach_samples": sum(v.get("motor_stage")==21 for v in ordered),
        "glide_finish_samples": sum(v.get("motor_stage")==22 for v in ordered),
        "sprint_accel_samples": sum(v.get("motor_stage")==23 for v in ordered),
        "sprint_brake_samples": sum(v.get("motor_stage")==24 for v in ordered),
        "sprint_coast_samples": sum(v.get("motor_stage")==25 for v in ordered),
        "sprint_approach_samples": sum(v.get("motor_stage")==26 for v in ordered),
        "sprint_finish_samples": sum(v.get("motor_stage")==27 for v in ordered),
        "track_accel_samples": sum(v.get("motor_stage")==28 for v in ordered),
        "track_brake_samples": sum(v.get("motor_stage")==29 for v in ordered),
        "track_near_samples": sum(v.get("motor_stage")==30 for v in ordered),
        "track_settle_samples": sum(v.get("motor_stage")==31 for v in ordered),
        "guard_forward_samples": sum(v.get("motor_stage")==32 for v in ordered),
        "guard_brake_samples": sum(v.get("motor_stage")==33 for v in ordered),
        "guard_near_samples": sum(v.get("motor_stage")==34 for v in ordered),
        "guard_stop_samples": sum(v.get("motor_stage")==35 for v in ordered),
        "guard_active_band_brake_samples": sum(v.get("motor_stage")==36 for v in ordered),
        "pace_drive_samples": sum(v.get("motor_stage")==37 for v in ordered),
        "pace_brake_samples": sum(v.get("motor_stage")==38 for v in ordered),
        "pace_near_samples": sum(v.get("motor_stage")==39 for v in ordered),
        "pace_stop_samples": sum(v.get("motor_stage")==40 for v in ordered),
        "pace_band_brake_samples": sum(v.get("motor_stage")==41 for v in ordered),
        "coast_drive_samples": sum(v.get("motor_stage")==42 for v in ordered),
        "coast_neutral_samples": sum(v.get("motor_stage")==43 for v in ordered),
        "coast_near_samples": sum(v.get("motor_stage")==44 for v in ordered),
        "coast_stop_samples": sum(v.get("motor_stage")==45 for v in ordered),
        "coast_band_neutral_samples": sum(v.get("motor_stage")==46 for v in ordered),
        "coast_negative_command_samples": sum(v["cmd_pm"]<0 for v in ordered),
        "in_target_band_high_rate_samples": len(fast_in_band),
        "first_band_rate_dps": rnd(in_band[0]["gyro_dps"]) if in_band else None,
        "midturn_low_speed_samples": len(slow_mid),
        "midturn_reverse_samples": len(reverse_mid),
        "min_midturn_gyro_dps": rnd(min((r["gyro_dps"] for r in midturn),
                                       default=None)),
        "brake_first_yaw_deg": rnd(first_brake["yaw_deg"]) if first_brake else None,
        "max_cmd_delta_20ms": max(cmd_deltas,default=None),
        "command_polarity_flips": polarity_flips,
        "applied_pwm_max_step_20ms": max(motor_steps,default=None),
        "neutral_actual_high_gyro_samples": high_gyro_neutral,
        "rapid_or_flow_brake_reaccelerations": sum(
            v.get("motor_stage") in (9,13,18,23) and any(
                z.get("motor_stage") in (10,14,19,24) for z in rows[:i])
            for i,v in enumerate(rows) if v["phase"]==1),
        "peak_gyro_1khz_dps": (round(int(result["peak_dps10"])/10.0,3)
                                if result.get("peak_dps10") else None),
        "max_requested_pwm_pm": max(abs(s["cmd_pm"]) for s in rows),
        "max_applied_pwm_pm": max(applied_pwm) if applied_pwm else None,
        "encoder_imbalance_pct": rnd(encoder_imbalance),
        "motor_sweep_steps": None,
        "imu_effective_hz": rnd(new_samples*1000.0/duration_ms) if duration_ms else None,
        "imu_intervals_max_us": max((w["interval_max_us"] for w in valid_imu),default=None),
        "imu_intervals_mean_us": rnd(mean_gyro_dt),
        "imu_gap_gt1500": sum(w["gap_gt1500"] for w in valid_imu) if valid_imu else None,
        "imu_gap_gt2000": sum(w["gap_gt2000"] for w in valid_imu) if valid_imu else None,
        "imu_observer_missed": sum(w["observer_missed"] for w in valid_imu) if valid_imu else None,
        "imu_status_notready": sum(w["status_notready"] for w in valid_imu) if valid_imu else None,
        "imu_i2c_errors": sum(w["i2c_errors"] for w in valid_imu) if valid_imu else None,
        "imu_age_mean_us": rnd(statistics.mean(sample_ages)),
        "imu_age_max_us": max(sample_ages),
        "trace_samples": len(rows),
        "sampling_gap_max_ms": max(gaps) if gaps else 0,
        "trace_first_ms": rows[0]["t_ms"],
        "trace_terminal_disagreement_deg": rnd(terminal_disagreement),
        "ring_drops": int(result.get("qdrop", 0)),
        "uart_drops": int(result.get("uartdrop", 0)),
        # Software trace quality only, NOT physical floor-motion acceptance.
        "meets_trace_thresholds": bool(
            result.get("reason") == "DONE" and overshoot <= 3.0 and
            abs(target-final_yaw) <= 2.0 and settling is not None and
            stop_ms is not None and rows[-1]["t_ms"]>=stop_ms+650 and
            max(gaps, default=0) <= 60 and len(rows) >= 12 and
            rows[0]["t_ms"] <= 60 and
            terminal_disagreement <= 2.0 and
            int(result.get("qdrop", 0)) == 0 and
            int(result.get("uartdrop", 0)) == 0),
    }


def save_csv(path: Path, samples: list[dict]) -> None:
    # Derive signed wheel speed from raw tick deltas using the MCU timeline.
    # No filtering; allows reconstructing motor asymmetry and lag offline.
    prev = None
    for sample in sorted(samples,key=lambda z:z["t_ms"]):
        dt=sample["t_ms"]-prev["t_ms"] if prev else 0
        sample["encoder_l_pps"]=(1000*(sample["enc_l"]-prev["enc_l"])/dt) if dt>0 else None
        sample["encoder_r_pps"]=(1000*(sample["enc_r"]-prev["enc_r"])/dt) if dt>0 else None
        prev=sample
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELD_NAMES)
        writer.writeheader()
        for sample in samples:
            writer.writerow({k: sample.get(k) for k in FIELD_NAMES})


def plot_run(path: Path, samples: list[dict], title: str) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    if not samples:
        return False
    t=[s["t_ms"]/1000 for s in samples]
    target=samples[0]["target_deg"]
    # One four-panel engineering trace, same timestamps on all channels.
    fig, axes=plt.subplots(4,1,figsize=(11,10),sharex=True)
    axes[0].plot(t,[s["yaw_deg"] for s in samples],label="yaw IMU")
    axes[0].axhline(target,linestyle="--",label=f"target {target} deg")
    axes[0].axhline(target+2,linestyle=":",alpha=0.5)
    axes[0].axhline(target-2,linestyle=":",alpha=0.5)
    axes[0].set_ylabel("Angle [deg]")
    axes[0].legend(loc="best")
    axes[1].plot(t,[s["error_deg"] for s in samples],label="error")
    axes[1].axhline(0,linestyle="--")
    axes[1].set_ylabel("Error [deg]")
    axes[2].plot(t,[s["gyro_dps"] for s in samples],label="gyro rate")
    axes[2].plot(t,[s["ref_dps"] for s in samples],label="rate reference")
    axes[2].set_ylabel("Rate [deg/s]")
    axes[2].legend(loc="best")
    axes[3].plot(t,[s["cmd_pm"] for s in samples],label="commanded PWM")
    axes[3].plot(t,[s["pid_pm"] for s in samples],label="rate PID",alpha=0.65)
    axes[3].plot(t,[s["ff_pm"] for s in samples],label="feedforward",alpha=0.65)
    if any(s.get("applied_l_pm") is not None for s in samples):
        axes[3].plot(t,[s.get("applied_l_pm") for s in samples],label="actual L",alpha=0.55)
        axes[3].plot(t,[s.get("applied_r_pm") for s in samples],label="actual R",alpha=0.55)
    axes[3].set_ylabel("Motor permille")
    axes[3].set_xlabel("TIM11 relative time [s]")
    axes[3].legend(loc="best")
    for ax in axes:
        ax.grid(True,alpha=0.25)
        coast = next((s["t_ms"]/1000 for s in samples if s["phase"]==2),None)
        if coast is not None:
            ax.axvline(coast,linestyle="--",alpha=0.5)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path,dpi=130)
    plt.close(fig)
    return True


def write_report(root: Path, runs: dict[int,list[dict]],
                 results: dict[int,dict],
                 imu_windows: dict[int,list[dict]] | None = None) -> list[dict]:
    imu_windows = imu_windows or {}
    root.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for run_id in sorted(runs):
        samples=runs[run_id]
        if not samples:
            continue
        folder=root/f"run_{run_id:03d}"
        folder.mkdir(exist_ok=True)
        save_csv(folder/"samples.csv",samples)
        if samples[0]["target_deg"]==0:
            steps=motor_sweep_steps(samples)
            with (folder/"motor_steps.csv").open("w",newline="",encoding="utf-8") as f:
                w=csv.DictWriter(f,fieldnames=MOTOR_FIELDS)
                w.writeheader()
                for item in steps:w.writerow(item)
            (folder/"motor_steps.json").write_text(
                json.dumps(steps,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
        subset=imu_windows.get(run_id,[])
        if subset:
            with (folder/"imu_windows.csv").open("w",newline="",encoding="utf-8") as f:
                w=csv.DictWriter(f,fieldnames=IMU_FIELDS)
                w.writeheader()
                for item in subset:w.writerow({k:item.get(k) for k in IMU_FIELDS})
        summary=analyze(samples,results.get(run_id,{}),subset)
        (folder/"summary.json").write_text(
            json.dumps(summary,indent=2,ensure_ascii=False)+"\n",
            encoding="utf-8")
        plot_run(folder/"trace.png",samples,
                 f"Run {run_id} / {summary.get('target_deg')} deg / "
                 f"{summary.get('profile')} / {summary.get('reason')}")
        summaries.append(summary)
    with (root/"summary.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=REPORT_FIELDS,extrasaction="ignore")
        writer.writeheader()
        for item in summaries:
            writer.writerow({k:item.get(k) for k in REPORT_FIELDS})
    (root/"summary.json").write_text(
        json.dumps(summaries,indent=2,ensure_ascii=False)+"\n",
        encoding="utf-8")
    stats={}
    groups={}
    for item in summaries:
        # Exclude aborted/PA0_STOP and the motor sweep from turn-timing means.
        if "error" in item or item.get("reason")!="DONE" or item.get("target_deg")==0:
            continue
        key=f"{item['target_deg']}deg_{item['profile']}"
        groups.setdefault(key,[]).append(item)
    for key,items in groups.items():
        entry={"runs":len(items),"trace_pass":sum(bool(z.get("meets_trace_thresholds")) for z in items)}
        for field in ("rise_10_90_ms","stop_ms","settling_ms",
                      "overshoot_deg","max_abs_gyro_dps",
                      "final_abs_error_deg","coast_drift_deg",
                      "imu_effective_hz","imu_intervals_max_us",
                      "imu_intervals_mean_us","imu_age_mean_us",
                      "imu_age_max_us","max_requested_pwm_pm",
                      "max_applied_pwm_pm","encoder_imbalance_pct",
                      "terminal_80_to_stop_ms","kick_samples",
                      "smooth_approach_samples","reverse_pwm_samples",
                      "rapid_accel_samples","rapid_brake_samples",
                      "rapid_near_samples","rapid_coast_samples",
                      "peak_gyro_1khz_dps","trace_first_ms",
                      "trace_terminal_disagreement_deg",
                      "flow_accel_samples","flow_brake_samples",
                      "flow_coast_samples","flow_approach_samples",
                      "flow_finish_samples","rapid_or_flow_brake_reaccelerations",
                      "glide_accel_samples","glide_brake_samples",
                      "glide_coast_samples","glide_approach_samples",
                      "glide_finish_samples","brake_first_yaw_deg",
                      "max_cmd_delta_20ms","terminal_last10_to_stop_ms",
                      "sprint_accel_samples","sprint_brake_samples",
                      "sprint_coast_samples","sprint_approach_samples",
                      "sprint_finish_samples",
                      "track_accel_samples","track_brake_samples",
                      "track_near_samples","track_settle_samples",
                      "midturn_low_speed_samples","midturn_reverse_samples",
                      "min_midturn_gyro_dps",
                      "guard_forward_samples","guard_brake_samples",
                      "guard_near_samples","guard_stop_samples",
                      "guard_active_band_brake_samples",
                      "in_target_band_high_rate_samples","first_band_rate_dps",
                      "pace_drive_samples","pace_brake_samples",
                      "pace_near_samples","pace_stop_samples","pace_band_brake_samples",
                      "coast_drive_samples","coast_neutral_samples",
                      "coast_near_samples","coast_stop_samples",
                      "coast_band_neutral_samples","coast_negative_command_samples",
                      "command_polarity_flips","applied_pwm_max_step_20ms",
                      "neutral_actual_high_gyro_samples"):
            numbers=[float(z[field]) for z in items if z.get(field) is not None]
            if numbers:
                entry[field]={"mean":round(statistics.mean(numbers),3),
                              "stdev":round(statistics.stdev(numbers),3) if len(numbers)>1 else None}
            else:
                entry[field]={"mean":None,"stdev":None}
        stats[key]=entry
    (root/"group_stats.json").write_text(
        json.dumps(stats,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    return summaries


def zip_session(root: Path) -> Path:
    destination=root/"turn_pid_bundle.zip"
    with zipfile.ZipFile(destination,"w",compression=zipfile.ZIP_DEFLATED) as zf:
        for source in root.rglob("*"):
            if source.is_file() and source!=destination:
                zf.write(source,arcname=str(source.relative_to(root)))
    return destination


def capture(args: argparse.Namespace) -> int:
    try:
        import serial
    except ImportError:
        print("Missing pyserial. Install: python -m pip install pyserial",
              file=sys.stderr)
        return 2
    root=Path(args.output) if args.output else Path("results")/(
        "turn_pid_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    root.mkdir(parents=True,exist_ok=True)
    runs: dict[int,list[dict]]={}
    results: dict[int,dict]={}
    imu_windows: dict[int,list[dict]]={}
    completed=set()
    mode_byte={"90":b"1","180":b"2","motor":b"m"}[args.mode]
    profile_byte={"base":b"b","fast":b"f","tuned":b"t","smooth":b"v","rapid":b"h","flow":b"w","glide":b"g","sprint":b"p","track":b"k","guard":b"z","pace":b"j","coast":b"c"}[args.profile]
    pending=bytearray()
    last_serial_activity=time.monotonic()
    last_config=0.0
    configured=False
    print(f"Results directory: {root.resolve()}")
    print(f"Requested: {args.mode} degrees / {args.profile.upper()} / "
          f"{args.runs} run(s)")
    print("IMPORTANT: UART cannot start motors. Wait for TP,CAL_OK / "
          "TP,CONFIG then physically PRESS and RELEASE PA0.")
    print("A second PA0 press during motion stops the motors; VM power "
          "cutoff must remain available.")
    try:
        ser=serial.Serial(args.port,args.baud,timeout=0.15,
                          write_timeout=1.0)
    except Exception as exc:
        print(f"Cannot open {args.port}: {exc}",file=sys.stderr)
        return 2
    began=time.monotonic()
    try:
        with ser, (root/"raw_uart.log").open("w",encoding="utf-8") as raw:
            def configure():
                nonlocal last_config
                # Idle-only configuration; no start command exists.
                ser.write(mode_byte+profile_byte+b"?")
                ser.flush()
                last_config=time.monotonic()
                print(f"[CONFIG] sent angle={args.mode}, profile={args.profile}; "
                      "PA0 is the ONLY start.")
            configure()
            while len(completed)<args.runs:
                if args.duration and time.monotonic()-began>args.duration:
                    print("[STOP] Total session time limit reached; commanding UART stop.")
                    try:ser.write(b"S")
                    except Exception:pass
                    break
                try:
                    data=ser.readline()
                except (serial.SerialException,OSError) as exc:
                    print(f"[ERROR] COM read failed: {exc}",file=sys.stderr)
                    break
                if not data:
                    continue
                line=data.decode("utf-8",errors="replace").strip()
                raw.write(line+"\n")
                raw.flush()
                last_serial_activity=time.monotonic()
                if line.startswith("TP,D,"):
                    sample=parse_sample(line)
                    if sample:
                        runs.setdefault(sample["run_id"],[]).append(sample)
                    else:
                        print(f"[WARN] Invalid trace frame: {line}")
                elif line.startswith("TP,IMU,"):
                    w=parse_imu(line)
                    if w:
                        imu_windows.setdefault(w["run_id"],[]).append(w)
                        print(f"[IMU] run={w['run_id']} published={w['effective_hz']:.1f} Hz "
                              f"gap_avg={w['interval_mean_us']}us "
                              f"gap_max={w['interval_max_us']}us "
                              f"notready={w['status_notready']} "
                              f"i2c_err={w['i2c_errors']}")
                    else:
                        print(f"[WARN] Invalid IMU timing record: {line}")
                elif line.startswith("TP,RESULT,"):
                    kv=parse_kv(line,"TP,RESULT,")
                    try:rid=int(kv["id"])
                    except (KeyError,ValueError):
                        print(f"[WARN] Invalid result: {line}")
                        continue
                    results[rid]=kv
                    completed.add(rid)
                    print(f"[RESULT] {line}")
                    summaries=write_report(root,runs,results,imu_windows)
                    if summaries: print("[METRICS] "+json.dumps(summaries[-1],
                                                     ensure_ascii=False))
                elif line.startswith("TP,HELLO,") or line.startswith("TP,BOOT,"):
                    print("[BOOT] "+line)
                    if time.monotonic()-last_config>1.0:
                        configure()
                elif line.startswith("TP,CAL,"):
                    print("[CAL] "+line)
                elif line.startswith("TP,CAL_OK,"):
                    print("[CAL OK] "+line)
                elif line.startswith("TP,READY,"):
                    print("[READY] "+line)
                    if not configured:
                        configure()
                        configured=True
                elif line.startswith("TP,START,"):
                    print("[START] "+line)
                elif line.startswith("TP,CONFIG,"):
                    print("[CONFIG OK] "+line)
                elif line.startswith("TP,") and not line.startswith("TP,IDLE,"):
                    print("[MCU] "+line)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Sending serial STOP as a precaution.")
        try:ser.write(b"S")
        except Exception:pass
    finally:
        summaries=write_report(root,runs,results,imu_windows)
        (root/"session.json").write_text(
            json.dumps({"port":args.port,"baud":args.baud,"requested_mode":args.mode,
                        "requested_profile":args.profile,"requested_runs":args.runs,
                        "completed_runs":len(completed),"valid_samples":sum(map(len,runs.values()))},
                       indent=2)+"\n",encoding="utf-8")
        print(f"[SAVED] {root.resolve()}")
        if args.zip:
            archive=zip_session(root)
            print(f"[ZIP CREATED] {archive.resolve()}")
        else:
            print("[GIT] Raw UART, CSV, JSON, and charts saved; no ZIP by default.")
        print(f"[COUNT] {len(completed)} complete result(s); "
              f"{len(summaries)} run(s) with trace data.")
    return 0 if len(completed)>=args.runs else 1


def offline(folder: Path, make_zip: bool = False) -> int:
    path=folder/"raw_uart.log"
    if not path.exists():
        print(f"Missing {path}",file=sys.stderr)
        return 2
    runs={}
    results={}
    imu_windows={}
    for line in path.read_text(encoding="utf-8",errors="replace").splitlines():
        sample=parse_sample(line)
        if sample:
            runs.setdefault(sample["run_id"],[]).append(sample)
        elif line.startswith("TP,IMU,"):
            w=parse_imu(line)
            if w:imu_windows.setdefault(w["run_id"],[]).append(w)
        elif line.startswith("TP,RESULT,"):
            kv=parse_kv(line,"TP,RESULT,")
            try:results[int(kv["id"])]=kv
            except (KeyError,ValueError):pass
    summaries=write_report(folder,runs,results,imu_windows)
    for item in summaries:
        print(json.dumps(item,ensure_ascii=False))
    print(f"[SAVED] {folder.resolve()}")
    if make_zip:
        archive=zip_session(folder)
        print(f"[ZIP CREATED] {archive.resolve()}")
    else:
        print("[GIT] Reprocessed raw files directly; no ZIP by default.")
    return 0 if summaries else 1


def main(argv=None) -> int:
    parser=argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port",default="COM7",help="USB-TTL USART1 port")
    parser.add_argument("--baud",type=int,default=115200)
    parser.add_argument("--mode",choices=["90","180","motor"],default="90",
                        help="90/180 degree pivot or bounded phase-1 motor PWM sweep")
    parser.add_argument("--profile",choices=["base","fast","tuned","smooth","rapid","flow","glide","sprint","track","guard","pace","coast"],default="base")
    parser.add_argument("--runs",type=int,default=3)
    parser.add_argument("--output",type=str,help="destination results folder")
    parser.add_argument("--duration",type=float,default=360,
                        help="maximum capture session seconds, 0 means unlimited")
    parser.add_argument("--analyze",type=Path,help="offline reprocess previous raw UART folder")
    parser.add_argument("--zip",action="store_true",
                        help="optionally bundle files for manual upload; not needed for GitHub")
    args=parser.parse_args(argv)
    if args.runs<1:parser.error("--runs must be >=1")
    if not args.analyze and args.profile in ("rapid","flow","glide") and args.mode!="90":
        parser.error("--profile rapid/flow/glide is restricted to 90-degree test only")
    if not args.analyze and args.profile in ("sprint","track","guard","pace","coast") and args.mode=="motor":
        parser.error("--profile sprint/track/guard/pace/coast supports only 90 and 180 degrees")
    return offline(args.analyze,make_zip=args.zip) if args.analyze else capture(args)


if __name__=="__main__":
    raise SystemExit(main())
