#!/usr/bin/env python3
"""Decode and visualize motor-running 200Hz six-IR Wall PID telemetry (MW).

Works on the original mixed W + MW BLE terminal export. Validates sample
number, FNV32 checksum, missed frames, ring overflow and motor lock.
No hardware commands, firmware flashing or automatic maze claims.
"""
from __future__ import annotations
import argparse
import csv
import html
import importlib.util
import json
import math
from pathlib import Path
import sys

FIELDS=("seq","ms","scan","enc_l","enc_r","yaw_cdeg","rate_cdps",
        "ir_age_us","valid","L90","L45","L0","R0","R45","R90",
        "wall_error_norm_1000","speed_mm_s","target_mm_s",
        "pwm_l_permille","pwm_r_permille","controller_state")
IR=("L90","L45","L0","R0","R45","R90")
MM_PER_COUNT=math.pi*34.0/1430.0
HASH_OFFSET=2166136261

def fnv32(hash_value:int, text:str)->int:
    for b in text.encode("ascii",errors="replace"):
        hash_value=((hash_value^b)*16777619)&0xffffffff
    return hash_value

def parse_pairs(parts):
    out={}
    for part in parts:
        key, sep, value=part.partition("=")
        if sep and key:
            if key in out:raise ValueError("duplicated metadata key "+key)
            out[key]=value
    return out

def parse_motor_lines(lines:list[str]):
    rows=[]
    start=end=None
    faults=[]
    checksum=HASH_OFFSET
    first_run=None
    for ix,line in enumerate(lines,1):
        pos=line.find("MW,")
        if pos<0:continue
        wire=line[pos:].strip()
        bits=wire.split(",")
        if len(bits)>1 and bits[1]=="BEGIN":
            if start is not None:
                faults.append("DUPLICATE_BEGIN");continue
            try:start=parse_pairs(bits[2:])
            except ValueError:faults.append("BAD_BEGIN")
        elif len(bits)>1 and bits[1]=="END":
            if end is not None:
                faults.append("DUPLICATE_END");continue
            try:end=parse_pairs(bits[2:])
            except ValueError:faults.append("BAD_END")
        elif len(bits)>1 and bits[1].isdigit():
            if start is None or end is not None:
                faults.append("ROW_OUTSIDE_FRAME:"+str(ix));continue
            if len(bits)!=len(FIELDS)+1:
                faults.append("COLUMN_COUNT:"+str(ix));continue
            try:
                vals=[int(x) for x in bits[1:]]
            except ValueError:
                faults.append("NON_INTEGER:"+str(ix));continue
            record=dict(zip(FIELDS,vals))
            if record["seq"]!=len(rows):
                faults.append("SEQUENCE_GAP:"+str(ix))
            if rows and (record["ms"]<=rows[-1]["ms"] or
                         record["scan"]<rows[-1]["scan"]):
                faults.append("NONMONOTONIC_TIME_OR_SCAN:"+str(ix))
            if (record["valid"] not in (0,1) or
                any(not 0<=record[k]<=4095 for k in IR) or
                not 0<=record["ir_age_us"]<=65535):
                faults.append("BAD_SENSOR_VALUE:"+str(ix))
            if (any(not 0<=record[k]<=800 for k in ("pwm_l_permille","pwm_r_permille")) or
                abs(record["wall_error_norm_1000"])>1000):
                faults.append("COMMAND_BOUND:"+str(ix))
            if record["ms"]%5:
                faults.append("NON_5MS_SAMPLE:"+str(ix))
            checksum=fnv32(checksum,wire+"\n")
            record["distance_mm"]=round((record["enc_l"]+record["enc_r"])*.5*MM_PER_COUNT,3)
            record["yaw_deg"]=round(record["yaw_cdeg"]*.01,3)
            record["gyro_dps"]=round(record["rate_cdps"]*.01,3)
            record["wall_error_norm"]=round(record["wall_error_norm_1000"]*.001,5)
            record["cell_index"]=int(math.floor(record["distance_mm"]/180.0))
            rows.append(record)
        else:
            faults.append("UNKNOWN_MW_LINE:"+str(ix))
    if start is None:faults.append("MISSING_BEGIN")
    if end is None:faults.append("MISSING_END")
    if start is not None:
        try:
            if int(start["n"])!=len(rows):faults.append("BEGIN_COUNT_MISMATCH")
            if int(start["locked"])!=1:faults.append("BEGIN_NOT_LOCKED")
            if int(start["motor_run"])!=1:faults.append("NOT_MOTOR_RUN")
            if int(start["dropped"])!=0:faults.append("CAPTURE_OVERFLOW")
            if int(start["hz"])!=200:faults.append("UNEXPECTED_SAMPLE_HZ")
        except (ValueError,KeyError):faults.append("INVALID_BEGIN")
    if end is not None:
        try:
            if int(end["n"])!=len(rows):faults.append("END_COUNT_MISMATCH")
            if int(end["locked"])!=1:faults.append("END_NOT_LOCKED")
            if int(end["dropped"])!=0:faults.append("CAPTURE_OVERFLOW")
            if int(end["fnv32"],16)!=checksum:faults.append("FNV32_MISMATCH")
        except (ValueError,KeyError):faults.append("INVALID_END")
    gaps=[b["ms"]-a["ms"] for a,b in zip(rows,rows[1:])]
    if gaps and any(g!=5 for g in gaps):faults.append("SAMPLE_GAP_NOT_5MS")
    if rows and rows[0]["ms"]!=5:faults.append("START_SAMPLE_MISSING")
    if not rows:faults.append("NO_SENSOR_SAMPLES")
    unique=list(dict.fromkeys(faults))
    return rows,{
        "protocol":"MW_V1_MOTOR_ON_200HZ",
        "complete":not unique,
        "faults":unique,
        "begin":start,"end":end,
        "frames":len(rows),
        "valid_ir_frames":sum(r["valid"] for r in rows),
        "last_distance_mm":rows[-1]["distance_mm"] if rows else None,
        "last_sample_ms":rows[-1]["ms"] if rows else None,
        "max_sample_gap_ms":max(gaps) if gaps else None,
        "max_ir_age_us":max((r["ir_age_us"] for r in rows),default=None),
        "max_speed_mm_s":max((r["speed_mm_s"] for r in rows),default=None),
        "max_pwm_permille":max((max(r["pwm_l_permille"],r["pwm_r_permille"])
                              for r in rows),default=None),
        "fnv_computed":f"{checksum:08x}",
        "physical_ground_truth":False,
        "ir_distance_calibrated":False,
        "note":"ADC/EMA and wall_error_norm are NOT physical distance. Motion is motor-driven; physical results require operator verification."
    }

def build_dashboard(rows:list[dict],qc:dict)->str:
    # Uses the existing interactive six-channel dashboard, then adds
    # synchronized PWM/speed/error plots with the same cursor and X axis.
    path=Path(__file__).resolve().parent/"maze_sensor_capture_report.py"
    spec=importlib.util.spec_from_file_location("maze_sensor_capture_report",path)
    if spec is None or spec.loader is None:raise RuntimeError("viewer dependency missing")
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    adapted=[]
    for r in rows:
        adapted.append({**r,
            "gyro_dps":r["gyro_dps"],
            "yaw_deg":r["yaw_deg"],
            "ir_age_us":r["ir_age_us"],
            "distance_mm":r["distance_mm"]})
    base=module.build_html(adapted,qc)
    extra="""<section><h2>Encoder speed & target (mm/s)</h2><canvas id="speed"></canvas></section>
<section><h2>PWM left / right (permille)</h2><canvas id="pwm"></canvas></section>
<section><h2>Wall PID optical error (unitless) / control state</h2><canvas id="error"></canvas>
<small>Controller state: 1=RUNNING, 2=DONE, 3=FAULT. Six IR plotted above are EMA0.10, not millimetres.</small></section>
<script>
function extras(){
 if(!rows.length)return;
 chart('speed',[{key:'speed_mm_s',label:'Actual mm/s',color:'#58d5c0'},
                {key:'target_mm_s',label:'Target mm/s',color:'#ffc17d'}],true);
 chart('pwm',[{key:'pwm_l_permille',label:'PWM L',color:'#58d5c0'},
              {key:'pwm_r_permille',label:'PWM R',color:'#eb87ab'}],true);
 chart('error',[{key:'wall_error_norm',label:'Norm error',color:'#f5c773'}],true);
}
axis.addEventListener('change',extras);
cursor.addEventListener('input',extras);
showInvalid.addEventListener('change',extras);
window.addEventListener('resize',extras);
extras();
</script></html>"""
    return base.replace("</script></html>","</script>"+extra)

def create_report(input_path:Path,out:Path):
    text=input_path.read_text(encoding="utf-8",errors="replace")
    rows,qc=parse_motor_lines(text.splitlines())
    # Interpret old W summary without treating MW as 20Hz W samples.
    spec=importlib.util.spec_from_file_location("wall_pid_ble_report",
        Path(__file__).resolve().parent/"wall_pid_ble_report.py")
    if spec is None or spec.loader is None:raise RuntimeError("summary dependency unavailable")
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    wrows,wsum=module.parse_lines(text.splitlines())
    qc["legacy_wall_trace_complete"]=wsum["data_complete"]
    qc["legacy_wall_summary"]=wsum
    qc["combined_complete"]=bool(qc["complete"] and wsum["data_complete"])
    out.mkdir(parents=True,exist_ok=True)
    columns=FIELDS+("distance_mm","yaw_deg","gyro_dps","wall_error_norm","cell_index")
    with (out/"full_trace_200hz.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=columns)
        writer.writeheader();writer.writerows(rows)
    (out/"full_trace_qc.json").write_text(json.dumps(qc,ensure_ascii=False,indent=2)+"\n",
                                          encoding="utf-8")
    (out/"viewer.html").write_text(build_dashboard(rows,qc),encoding="utf-8")
    return qc

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input",type=Path,required=True)
    ap.add_argument("--output",type=Path)
    args=ap.parse_args(argv)
    try:
        out=args.output or args.input.parent/(args.input.stem+"_wall_6ir")
        qc=create_report(args.input,out)
        print("FULL WALL PID 6-IR:", "COMPLETE" if qc["combined_complete"] else "INCOMPLETE",
              "| frames",qc["frames"],"|",out)
        if qc["faults"]:print("ERRORS:",", ".join(qc["faults"]))
        if not qc["legacy_wall_trace_complete"]:
            print("WARNING: W legacy trace incomplete; check BLE terminal export")
        return 0 if qc["combined_complete"] else 2
    except (OSError,ValueError,RuntimeError) as e:
        print("ERROR:",e,file=sys.stderr);return 2

if __name__=="__main__":
    raise SystemExit(main())
