#!/usr/bin/env python3
"""Encoder + gyro heading fusion replay; optional L0/R0 optical residual.

Requires separately measured wheel track (mm). Does not create a calibrated
IR position fix, never changes STM32 runtime, motor PID or map geometry.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys

MM_PER_TICK=math.pi*34.0/1430.0
FIELDS=("ms","distance_mm","yaw_deg","valid","enc_l","enc_r","L0","R0")

def angle_wrap_deg(x:float)->float:
    return (x+180.0)%360.0-180.0

def load_capture(path:Path)->list[dict]:
    with path.open("r",encoding="utf-8-sig",newline="") as f:
        reader=csv.DictReader(f)
        if not reader.fieldnames or not set(FIELDS).issubset(reader.fieldnames):
            raise ValueError("Requires MC trace.csv with encoder, yaw, L0/R0 and validity")
        rows=[]
        for i,r in enumerate(reader,2):
            try:
                z={k:float(r[k]) for k in FIELDS}
                if any(not math.isfinite(v) for v in z.values()):
                    raise ValueError("nonfinite")
                if z["valid"] not in (0,1):
                    raise ValueError("valid=0/1")
                if rows and z["ms"]<=rows[-1]["ms"]:
                    raise ValueError("nonincreasing time")
                rows.append(z)
            except (ValueError,KeyError) as exc:
                raise ValueError(f"invalid row {i}: {exc}") from exc
    if len(rows)<10:raise ValueError(">=10 samples needed")
    return rows

def replay(rows:list[dict],track_mm:float,imu_weight:float=.98)->tuple[list[dict],dict]:
    if not 20.0<=track_mm<=200.0 or not .0<=imu_weight<=1.0:
        raise ValueError("Measure wheel track 20..200mm; imu weight in [0,1]")
    origin=next((r for r in rows if r["valid"]),None)
    if origin is None:raise ValueError("No valid IMU/IR frame")
    fused=odom=0.0
    x=y=0.0
    last=rows[0]
    imu_origin=origin["yaw_deg"]
    prev_imu=imu_origin
    unwrap_imu=0.0
    out=[]
    deltas=[]
    reference_ir=(origin["L0"],origin["R0"])
    for i,r in enumerate(rows):
        dl=(r["enc_l"]-last["enc_l"])*MM_PER_TICK
        dr=(r["enc_r"]-last["enc_r"])*MM_PER_TICK
        ds=(dl+dr)*.5
        dyaw_enc=math.degrees((dr-dl)/track_mm)
        odom+=dyaw_enc
        predicted=fused+dyaw_enc
        if r["valid"]:
            if i==0:prev_imu=r["yaw_deg"]
            unwrap_imu+=angle_wrap_deg(r["yaw_deg"]-prev_imu)
            prev_imu=r["yaw_deg"]
            innov=angle_wrap_deg(unwrap_imu-predicted)
            fused=predicted+imu_weight*innov
            quality="VALID"
            deltas.append(abs(angle_wrap_deg(unwrap_imu-odom)))
        else:
            innov=None
            fused=predicted
            quality="PREDICTED_ONLY"
        yaw_rad=math.radians(fused-.5*dyaw_enc)
        x+=ds*math.cos(yaw_rad)
        y+=ds*math.sin(yaw_rad)
        l0,r0=reference_ir
        norm_l=(r["L0"]-l0)/(r["L0"]+l0+32)
        norm_r=(r["R0"]-r0)/(r["R0"]+r0+32)
        out.append({
            "ms":r["ms"],"distance_mm":r["distance_mm"],
            "ds_mm":round(ds,4),
            "x_mm":round(x,4),"y_mm":round(y,4),
            "odom_yaw_deg":round(odom,5),
            "imu_yaw_rel_deg":round(unwrap_imu,5),
            "fused_yaw_deg":round(fused,5),
            "imu_minus_encoder_deg":round(angle_wrap_deg(unwrap_imu-odom),5)
                                     if quality=="VALID" else None,
            "ir_side_error_norm":round(norm_l-norm_r,5),
            "status":quality,
        })
        last=r
    report={
        "frames":len(rows),"valid_frames":sum(r["valid"]==1 for r in rows),
        "wheel_diameter_mm":34,"encoder_cpr":1430,
        "wheel_track_mm_operator_supplied":track_mm,
        "imu_weight":imu_weight,
        "encoder_only_final_yaw_deg":out[-1]["odom_yaw_deg"],
        "gyro_final_relative_yaw_deg":out[-1]["imu_yaw_rel_deg"],
        "fused_final_yaw_deg":out[-1]["fused_yaw_deg"],
        "p95_abs_gyro_encoder_yaw_diff_deg":sorted(deltas)[math.ceil(len(deltas)*.95)-1] if deltas else None,
        "final_lateral_dead_reckoned_mm":out[-1]["y_mm"],
        "physical_ground_truth":False,
        "IR_used_as_control_or_position_update":False,
        "note":("Encoder/gyro complementary orientation replay only. "
                "L0/R0 normalized optical residual is separate, not converted "
                "to mm or injected into pose without geometry/LUT validation."),
    }
    return out,report

def main(argv=None)->int:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--trace",type=Path,required=True)
    p.add_argument("--wheel-track-mm",type=float,required=True)
    p.add_argument("--imu-weight",type=float,default=.98)
    p.add_argument("--output",type=Path,default=Path("results/maze_pose_fusion"))
    args=p.parse_args(argv)
    try:
        rows=load_capture(args.trace)
        data,report=replay(rows,args.wheel_track_mm,args.imu_weight)
        args.output.mkdir(parents=True,exist_ok=True)
        with (args.output/"pose_fusion.csv").open("w",newline="",encoding="utf-8") as f:
            w=csv.DictWriter(f,fieldnames=data[0]);w.writeheader();w.writerows(data)
        (args.output/"pose_fusion_report.json").write_text(
            json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        print("POSE FUSION REPLAY:",len(data),"frames; yaw disagreement p95",
              report["p95_abs_gyro_encoder_yaw_diff_deg"],"|",args.output)
        return 0
    except (ValueError,OSError,ZeroDivisionError) as exc:
        print("ERROR:",exc,file=sys.stderr);return 2
if __name__=="__main__":
    raise SystemExit(main())
