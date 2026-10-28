#!/usr/bin/env python3
"""Decode reassembled W, lines copied from the JDY-33 BLE terminal.
Never confuses receiving UART bytes with confirmed BLE packet integrity.
Example: python tools/wall_pid_ble_report.py --input my_terminal.txt
"""
import argparse
import csv
import json
import re
from pathlib import Path
import statistics

FIELDS=("ms","distance_0p1mm","L0_ema_adc","R0_ema_adc",
        "wall_error_norm_1000","speed_mm_s","setpoint_mm_s",
        "pwm_l_permille","pwm_r_permille")
LEGACY_FIELDS=("ms","distance_0p1mm","L0_ema_adc","R0_ema_adc",
        "wall_error_0p01mm","speed_mm_s","setpoint_mm_s",
        "pwm_l_permille","pwm_r_permille")

def parse_lines(lines):
    rows=[]
    active_fields=LEGACY_FIELDS
    result=None
    start=None
    post=None
    fault=None
    end_seen=False
    invalid=[]
    for ix,line in enumerate(lines,1):
        # MW is a separate six-IR capture protocol. A substring search
        # accidentally treated "MW,<seq>" as legacy "W,<seq>" rows.
        match=re.search(r"(?<![A-Za-z0-9_])W,",line)
        if not match:continue
        pos=match.start()
        record=line[pos:].strip()
        fields=record.split(",")
        if len(fields)>=3 and fields[1]=="END":
            if result is not None:invalid.append(f"duplicate END at line {ix}")
            result={"reason":fields[2]}
            for f in fields[3:]:
                if "=" in f:
                    k,v=f.split("=",1)
                    try:result[k]=int(v)
                    except ValueError:result[k]=v
        elif len(fields)>=3 and fields[1]=="FAULT":
            fault={}
            for f in fields[2:]:
                if "=" in f:
                    k,v=f.split("=",1)
                    try:fault[k]=int(v)
                    except ValueError:fault[k]=v
        elif len(fields)>=3 and fields[1]=="START":
            start={"raw":record}
            active_fields=FIELDS if "error=REL_NORM" in record else LEGACY_FIELDS
        elif len(fields)>=3 and fields[1]=="POST":
            post={}
            for f in fields[2:]:
                if "=" in f:
                    k,v=f.split("=",1)
                    try:post[k]=int(v)
                    except ValueError:post[k]=v
        elif fields[:2]==["W","TRACE_END"]:
            end_seen=True
        elif len(fields)>2 and fields[1].isdigit():
            if len(fields)!=len(active_fields)+1:
                invalid.append(f"bad trace columns at line {ix}")
                continue
            try: values=[int(x) for x in fields[1:]]
            except ValueError:
                invalid.append(f"bad trace values at line {ix}")
                continue
            rows.append(dict(zip(active_fields,values)))
    if not start:invalid.append("missing START")
    if not result:invalid.append("missing END")
    if not end_seen:invalid.append("missing TRACE_END")
    if not post:invalid.append("missing POST")
    if result is not None:
        expected=result.get("trace")
        if isinstance(expected,int) and expected!=len(rows):
            invalid.append(f"missing BLE lines: expected={expected},got={len(rows)}")
        if result.get("locked")!=1:
            invalid.append("motor not reported locked on END")
    if rows:
        times=[row["ms"] for row in rows]
        if any(b<=a for a,b in zip(times,times[1:])):
            invalid.append("time not strictly increasing")
        if len(set(times))!=len(times): invalid.append("duplicated timestamps")
        if any(b-a>55 for a,b in zip(times,times[1:])):
            invalid.append("missing trace intervals >55ms")
    normalized=active_fields==FIELDS
    error_key="wall_error_norm_1000" if normalized else "wall_error_0p01mm"
    errors=[abs(row[error_key])*(0.001 if normalized else 0.01) for row in rows]
    p95=(sorted(errors)[min(len(errors)-1,int(.95*(len(errors)-1)))]
         if errors else None)
    metrics={
        "data_complete":len(invalid)==0,
        "invalid_reasons":invalid,
        "reason":result.get("reason") if result else None,
        "n_trace_rows":len(rows),
        "error_units":"dimensionless" if normalized else "legacy_uncalibrated_mm_proxy",
        "trace_fields":list(active_fields),
        "mae_relative_wall_mm":statistics.mean(errors) if errors and not normalized else None,
        "p95_abs_relative_wall_error_mm":p95 if not normalized else None,
        "max_abs_relative_wall_error_mm":max(errors) if errors and not normalized else None,
        "mae_wall_error_norm":statistics.mean(errors) if errors and normalized else None,
        "p95_abs_wall_error_norm":p95 if normalized else None,
        "max_abs_wall_error_norm":max(errors) if errors and normalized else None,
        "max_speed_mm_s":max((r["speed_mm_s"] for r in rows),default=None),
        "peak_pwm_permille":max((max(r["pwm_l_permille"],r["pwm_r_permille"])
                                for r in rows),default=None),
        "end":result,"post":post,"fault":fault,
        "fault_side":(
            "L0" if fault and fault.get("why")=="IR_EXCURSION" and
            abs(fault.get("l",0)-fault.get("refL",0))>165 and
            abs(fault.get("r",0)-fault.get("refR",0))<=510 else
            "R0" if fault and fault.get("why")=="IR_EXCURSION" and
            abs(fault.get("r",0)-fault.get("refR",0))>510 and
            abs(fault.get("l",0)-fault.get("refL",0))<=165 else
            "BOTH" if fault and fault.get("why")=="IR_EXCURSION" and
            abs(fault.get("r",0)-fault.get("refR",0))>510 and
            abs(fault.get("l",0)-fault.get("refL",0))>165 else None
        ),
        "stop_distance_mm":result["d10"]*.1 if result and "d10" in result else None,
        "poststop_distance_mm":post["dist10"]*.1 if post and "dist10" in post else None,
        "drift_mm":post["drift10"]*.1 if post and "drift10" in post else None,
        "physical_pid_qualified":False,
        "note":"Software telemetry only; assess floor distance and wall-clearance with independent measurements.",
    }
    return rows,metrics

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--input",type=Path,required=True,
                    help="ASCII BLE terminal lines, notifications already reassembled")
    ap.add_argument("--output",type=Path,default=None)
    args=ap.parse_args()
    source=args.input.read_text(encoding="utf-8",errors="replace")
    rows,report=parse_lines(source.splitlines())
    output=args.output or args.input.parent/(args.input.stem+"_report")
    output.mkdir(parents=True,exist_ok=True)
    with (output/"trace.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=report["trace_fields"])
        writer.writeheader()
        writer.writerows(rows)
    (output/"summary.json").write_text(
        json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,indent=2))
    print("OUTPUT:",output)
    return 0 if report["data_complete"] else 2

if __name__=="__main__":raise SystemExit(main())
