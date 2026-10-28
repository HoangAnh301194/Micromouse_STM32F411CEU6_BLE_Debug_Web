#!/usr/bin/env python3
"""One-command offline analysis for PA0 six-IR BLE captures.

Decode -> QC -> interactive 6-IR dashboard -> edge candidates ->
optional nonlinear calibration -> optional 3-state wall fusion ->
optional encoder/gyro fusion. Never touches hardware.
"""
from __future__ import annotations
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import sys

TOOLS=Path(__file__).resolve().parent
def load(name):
    path=TOOLS/(name+".py")
    spec=importlib.util.spec_from_file_location(name,path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Unable to load tool: "+str(path))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def run(ble:Path,out:Path,manifest:Path|None=None,
        model:Path|None=None,geometry:Path|None=None,
        truth:Path|None=None,track_mm:float|None=None)->dict:
    cap=load("maze_sensor_capture_report")
    cal=load("ir_calibration_lab")
    obs=load("maze_wall_observer")
    pose=load("maze_pose_fusion_lab")
    out.mkdir(parents=True,exist_ok=True)
    rows,qc=cap.decode_text(ble.read_text(encoding="utf-8",errors="replace"))
    (out/"capture").mkdir(parents=True,exist_ok=True)
    (out/"capture"/"capture_qc.json").write_text(json.dumps(qc,indent=2)+"\n")
    if not qc["complete"]:
        raise ValueError("BLE log not complete; checksum/frame mismatch: "+", ".join(qc["faults"]))
    cols=cap.FIELDS+("distance_mm","yaw_deg","gyro_dps","cell_index")
    trace=out/"capture"/"trace.csv"
    with trace.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=cols);w.writeheader();w.writerows(rows)
    (out/"capture"/"viewer.html").write_text(cap.build_html(rows,qc),encoding="utf-8")
    edge=obs.run_edges(trace,out/"edges",truth,4,.08,10,20)
    result={
        "mode":"OFFLINE_SENSOR_ALL_IN_ONE",
        "BLE_complete":True,
        "motor_started_by_tool":False,"firmware_flashing":False,
        "capture_rows":len(rows),"edge_candidates":edge["candidates"],
        "edge_scoring":edge["scoring"],
        "capture_dashboard":"capture/viewer.html",
        "maze_solve_hardware_accepted":False,
        "missing":[],
    }
    if manifest is not None:
        calibration=cal.analyze(manifest,out/"calibration")
        result["calibration"]={
            "channel_groups":len(calibration["channel_groups"]),
            "offline_gate_pass":{k:v["offline_calibration_gate_passed"]
                                 for k,v in calibration["channel_groups"].items()},
            "dashboard":"calibration/calibration_viewer.html",
        }
    else:result["missing"].append("calibration_manifest")
    if model is not None:
        fused=obs.run_fusion(trace,model,out/"wall_fusion",3,geometry)
        result["wall_fusion"]={
            "unknown_frames":fused["unknown_frames"],
            "geometry_confirmed":fused["geometry_confirmed"],
            "next_cell_candidates":fused["next_cell_candidates"],
            "dashboard":"wall_fusion/wall_state_viewer.html",
        }
    else:result["missing"].append("labelled_wall_model")
    if track_mm is not None:
        records=pose.load_capture(trace)
        fused,report=pose.replay(records,track_mm)
        (out/"pose").mkdir(parents=True,exist_ok=True)
        with (out/"pose"/"pose_fusion.csv").open("w",newline="",encoding="utf-8") as f:
            w=csv.DictWriter(f,fieldnames=fused[0]);w.writeheader();w.writerows(fused)
        (out/"pose"/"pose_fusion_report.json").write_text(json.dumps(report,indent=2)+"\n")
        result["pose_fusion"]={
            "yaw_disagreement_p95_deg":report["p95_abs_gyro_encoder_yaw_diff_deg"],
            "IR_applied_as_pose_correction":False,
        }
    else:result["missing"].append("measured_wheel_track")
    result["message"]=("This is a sensor QA report, NOT maze-solve acceptance. "
                       "Measure physical track/geometry and repeat on unseen maze layouts.")
    (out/"all_summary.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n",
                                        encoding="utf-8")
    return result

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ble",type=Path,required=True)
    p.add_argument("--out",type=Path,default=Path("results/maze_sensor_suite"))
    p.add_argument("--manifest",type=Path)
    p.add_argument("--model",type=Path)
    p.add_argument("--geometry",type=Path)
    p.add_argument("--edge-truth",type=Path)
    p.add_argument("--wheel-track-mm",type=float)
    args=p.parse_args(argv)
    try:
        if args.geometry and not args.model:
            raise ValueError("--geometry requires --model")
        report=run(args.ble,args.out,args.manifest,args.model,args.geometry,
                   args.edge_truth,args.wheel_track_mm)
        print("SENSOR SUITE:",report["capture_rows"],"verified frames;",
              report["edge_candidates"],"edge candidates;",
              "missing:",", ".join(report["missing"]) or "none")
        print("VIEWER:",args.out/"capture"/"viewer.html")
        print("OUTPUT:",args.out/"all_summary.json")
        return 0
    except (OSError,ValueError,RuntimeError) as exc:
        print("ERROR:",exc,file=sys.stderr)
        return 2
if __name__=="__main__":raise SystemExit(main())
