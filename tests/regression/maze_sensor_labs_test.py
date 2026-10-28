#!/usr/bin/env python3
"""Synthetic verification only. No BLE radio, physical wall labels, motor or flash."""
from __future__ import annotations
import csv
import importlib.util
import json
from pathlib import Path
import tempfile

ROOT=Path(__file__).resolve().parents[2]

def load(name):
    path=ROOT/"tools"/(name+".py")
    spec=importlib.util.spec_from_file_location(name,path)
    assert spec and spec.loader
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod

capture=load("maze_sensor_capture_report")
lut=load("ir_calibration_lab")
observer=load("maze_wall_observer")
pose=load("maze_pose_fusion_lab")
suite=load("maze_sensor_suite")

def check_capture(root:Path):
    records=[]
    s="MC,ARMED,motor_locked=1\nMC,BEGIN,n=64,hz=200,locked=1\n"
    h=capture.HASH_INIT
    for n in range(64):
        vals=[int(120 if n<21 or n>=44 else 1500) for _ in capture.IR]
        ints=[n,n*5,100+n,int(n*35),int(n*35),n*3,0,500,1,*vals]
        row="MC,"+",".join(map(str,ints))+"\n"
        h=capture.fnv32(h,row.encode("ascii"))
        s+=row
        records.append(ints)
    s+=f"MC,END,n=64,fnv32={h:08x},locked=1\n"
    rows,qc=capture.decode_text(s)
    assert qc["complete"] and qc["received_count"]==64 and qc["valid_frames"]==64,qc
    assert abs(rows[10]["distance_mm"]-10*35*capture.MM_PER_TICK)<.001
    assert rows[1]["cell_index"]==0
    assert "<canvas" in capture.build_html(rows,qc)
    root.joinpath("ble.txt").write_text(s,encoding="utf8")
    assert capture.main(["--input",str(root/"ble.txt"),"--output",str(root/"decoded")])==0
    assert (root/"decoded"/"trace.csv").is_file()
    assert (root/"decoded"/"viewer.html").is_file()
    tampered=s.replace("MC,2,10,102","MC,2,11,102",1)
    _,bad=capture.decode_text(tampered)
    assert not bad["complete"] and "FNV32_MISMATCH" in bad["faults"]
    missing=s.replace("MC,30,150,130","BAD,30,150,130",1)
    _,bad=capture.decode_text(missing)
    assert not bad["complete"] and any("COUNT_MISMATCH" in x for x in bad["faults"])
    (root/"ble_bad.txt").write_text(missing,encoding="utf8")
    assert capture.main(["--input",str(root/"ble_bad.txt"),
                         "--output",str(root/"bad_output")])==2
    return root/"decoded"/"trace.csv"

def write_trace(path:Path,channel:str,signal:int):
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=("ms","scan","us","channel","raw","median3","ema10","ema25","hybrid"))
        w.writeheader()
        for i in range(20):
            value=signal+(i%3-1)
            w.writerow({"ms":i*25,"scan":i,"us":i*25000,"channel":channel,
                        "raw":value,"median3":value,"ema10":value,
                        "ema25":value,"hybrid":value})

def check_calibrate(root:Path):
    m=root/"manifest.csv"
    rows=[]
    for channel,scale in (("L0",1.0),("R0",1.3)):
        for dist in (20,30,40,50,70):
            for repeat in range(2):
                path=root/f"{channel}_{dist}_{repeat}.csv"
                write_trace(path,channel,round(scale*45000/(dist**1.5)))
                rows.append({"trace_path":str(path),"channel":channel,
                             "distance_mm":dist,"trial_id":str(repeat),
                             "surface":"white_matte","light":"room","split":"train"})
        for dist in (35,55):
            path=root/f"{channel}_{dist}_holdout.csv"
            write_trace(path,channel,round(scale*45000/(dist**1.5)))
            rows.append({"trace_path":str(path),"channel":channel,
                         "distance_mm":dist,"trial_id":"holdout_"+str(dist),
                         "surface":"white_matte","light":"room","split":"test"})
    with m.open("w",newline="",encoding="utf8") as f:
        w=csv.DictWriter(f,fieldnames=lut.FIELDS);w.writeheader();w.writerows(rows)
    report=lut.analyze(m,root/"lut")
    assert len(report["channel_groups"])==2
    for res in report["channel_groups"].values():
        assert res["unique_distances"]==5 and res["holdout_n"]==2,res
        assert res["holdout_p95_abs_mm"]<5.0,res
        assert res["offline_calibration_gate_passed"]
        assert res["hardware_acceptance"]=="NOT_CERTIFIED_BY_OFFLINE_TOOL"
        assert lut.predict_mm(res["knots"],4095) is None
        assert lut.predict_mm(res["knots"],0) is None
    assert "<canvas" in (root/"lut"/"calibration_viewer.html").read_text()
    small=root/"small_manifest.csv"
    with small.open("w",newline="",encoding="utf8") as f:
        w=csv.DictWriter(f,fieldnames=lut.FIELDS);w.writeheader();w.writerows(rows[:4])
    report=lut.analyze(small,root/"small")
    assert not next(iter(report["channel_groups"].values()))["offline_calibration_gate_passed"]
    # Real setup.json auto-discovery must not infer L45/R45 geometry.
    session=root/"session"/"runs"/"run1"
    (session/"data").mkdir(parents=True)
    (session/"setup.json").write_text(json.dumps({"trial":1,"distance_left_mm":60,
        "distance_right_mm":70,"surface":"white_matte","light":"room"}))
    (session/"data"/"ir_filter_trace.csv").write_text(
        (root/"L0_20_0.csv").read_text())
    generated=lut.generate_manifest(root/"session",root/"generated_manifest.csv")
    assert len(generated)==2 and {r["channel"] for r in generated}=={"L0","R0"}

def check_observer(root:Path,trace:Path):
    readings=observer.read_trace(trace)
    assert len(readings)==64
    ev=observer.detect_edges(readings,window=3,min_relative=.08,refractory_mm=0)
    assert any(e["type"]=="WALL_APPEAR_CANDIDATE" for e in ev),ev
    assert any(e["type"]=="WALL_LOST_CANDIDATE" for e in ev),ev
    assert all(0<=e["distance_mm"]<180 for e in ev)
    sample=ev[0]
    ground=[{"channel":sample["channel"],
             "type":sample["type"].replace("_CANDIDATE",""),
             "distance_mm":sample["distance_mm"]+1.0}]
    scored=observer.score_edges(ev,ground,5)
    assert scored["matched"]==1 and scored["mae_mm"]==1.0
    report=observer.run_edges(trace,root/"edges",None,3,.08,0,20)
    assert report["candidates"]>0
    model={ch:{"threshold":750,"hysteresis_adc":100} for ch in observer.CHANNELS}
    predictions,states=observer.classify_trace(readings,model,debounce=3)
    assert not predictions
    assert states["summary"]["geometry_confirmed"] is False
    assert any(row["front"]=="WALL" for row in states["trace"])
    assert any(row["front"]=="OPEN" for row in states["trace"])
    assert states["summary"]["wall_state_conflicts"]["left"]==0
    # Disagreement between diagonal and lateral beams becomes UNKNOWN.
    alter=[dict(r) for r in readings]
    for r in alter:r["L45"]=4095-r["L0"]
    _,mix=observer.classify_trace(alter,model,debounce=3)
    assert any(r["left"]=="UNKNOWN" and r["left_conflict"]
               for r in mix["trace"])
    geometry={"measured_and_reviewed":True,"forward_projection_mm":{
        ch:100.0 for ch in observer.CHANNELS}}
    geo=root/"geometry.json";geo.write_text(json.dumps(geometry))
    model_file=root/"model.json";model_file.write_text(json.dumps({"model":model}))
    fused=observer.run_fusion(trace,model_file,root/"fusion",3,geo)
    assert fused["geometry_confirmed"]
    assert fused["cell_observations"]>0
    assert any(v>0 for v in fused["next_cell_candidates"].values())
    assert (root/"fusion"/"cell_observations.csv").is_file()
    assert "<canvas" in (root/"fusion"/"wall_state_viewer.html").read_text(encoding="utf-8")
    # Refuse a fabricated geometry file with no independent review flag.
    geo.write_text(json.dumps({"forward_projection_mm":geometry["forward_projection_mm"]}))
    try:observer.run_fusion(trace,model_file,root/"unsafe",3,geo)
    except ValueError as e:assert "measured_and_reviewed" in str(e)
    else:raise AssertionError("Unreviewed geometry should be rejected")
    assert not fused.get("maze_wall_detection_qualified",False)

def check_pose(root:Path,trace:Path):
    samples=pose.load_capture(trace)
    assert len(samples)==64
    track_mm=80.0  # test input only, not measured robot wheelbase
    states,report=pose.replay(samples,track_mm,imu_weight=.98)
    assert len(states)==len(samples)
    assert report["IR_used_as_control_or_position_update"] is False
    assert report["physical_ground_truth"] is False
    assert all(r["status"]=="VALID" for r in states)
    assert states[-1]["x_mm"]>0
    assert states[-1]["y_mm"]>0
    assert all(r["ir_side_error_norm"]==0 for r in states)
    assert pose.main(["--trace",str(trace),"--wheel-track-mm","80",
                      "--output",str(root/"pose")])==0
    assert (root/"pose"/"pose_fusion.csv").exists()
    assert (root/"pose"/"pose_fusion_report.json").exists()
    try:pose.replay(samples,0.0)
    except ValueError:pass
    else:raise AssertionError("Unmeasured/invalid wheel track must be rejected")
    corrupt=[dict(x) for x in samples]
    corrupt[4]["valid"]=0
    result,report=pose.replay(corrupt,track_mm)
    assert result[4]["status"]=="PREDICTED_ONLY"
    assert report["valid_frames"]==len(samples)-1


def check_all_in_one(root:Path):
    # Exercises entire workflow with independently fabricated inputs.
    result=suite.run(root/"ble.txt",root/"suite",
                     manifest=root/"manifest.csv",
                     model=root/"model.json",track_mm=80.0)
    assert result["BLE_complete"] and result["capture_rows"]==64
    assert result["calibration"]["channel_groups"]==2
    assert result["wall_fusion"]["geometry_confirmed"] is False
    assert result["maze_solve_hardware_accepted"] is False
    assert (root/"suite"/"capture"/"viewer.html").is_file()
    assert (root/"suite"/"calibration"/"calibration_viewer.html").is_file()
    assert (root/"suite"/"wall_fusion"/"wall_state_viewer.html").is_file()
    assert (root/"suite"/"pose"/"pose_fusion_report.json").is_file()
    assert suite.main(["--ble",str(root/"ble.txt"),
                       "--out",str(root/"suite_cli")])==0
    assert (root/"suite_cli"/"all_summary.json").is_file()
    try:suite.run(root/"ble_bad.txt",root/"suite_bad")
    except ValueError as exc:assert "not complete" in str(exc)
    else:raise AssertionError("Must reject incomplete BLE capture")


def main():
    with tempfile.TemporaryDirectory(prefix="maze_sensor_lab_") as d:
        root=Path(d)
        trace=check_capture(root)
        check_calibrate(root)
        check_observer(root,trace)
        check_pose(root,trace)
        check_all_in_one(root)
    print("MAZE_SENSOR_LABS_SYNTHETIC_PASS")

if __name__=="__main__":main()
