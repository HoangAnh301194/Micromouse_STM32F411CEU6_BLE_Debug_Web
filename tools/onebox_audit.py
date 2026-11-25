"""Read-only OneBox speed PID and heading-shadow audit.

Usage: py tools/onebox_audit.py path/to/onebox.csv --wheelbase-mm 74
Wheelbase is REQUIRED for heading estimate and must be measured on the robot.
The script never sends motor commands or changes controller gains.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path

def _value(row, field):
    try:
        raw = row.get(field, "")
        return float(raw) if raw not in ("", None) else None
    except (ValueError, TypeError):
        return None

def audit(rows, *, wheelbase_mm=None, wheel_diameter_mm=34.0,
          counts_per_rev=1430.0, pwm_limit=200.0):
    if wheelbase_mm is not None and wheelbase_mm <= 0:
        raise ValueError("wheelbase_mm must be positive")
    if wheel_diameter_mm <= 0 or counts_per_rev <= 0 or pwm_limit <= 0:
        raise ValueError("Invalid geometry or PWM limit")
    rows=list(rows)
    timestamps=[_value(r, "t_ms") for r in rows]
    intervals=[b-a for a,b in zip(timestamps,timestamps[1:]) if a is not None and b is not None and b>a]
    gaps=sum(dt>30 for dt in intervals)
    wheels={}
    for side in ("L","R"):
        refs=[(_value(r,f"REF_{side}_PPS"),_value(r,f"SPEED_{side}_PPS")) for r in rows]
        errors=[a-b for a,b in refs if a is not None and b is not None and a>0]
        pwm=[abs(v) for r in rows if (v:=_value(r,f"PWM_{side}")) is not None]
        wheels[side]={
            "valid_samples":len(errors),
            "mae_pps":round(sum(abs(e) for e in errors)/len(errors),3) if errors else None,
            "rmse_pps":round(math.sqrt(sum(e*e for e in errors)/len(errors)),3) if errors else None,
            "saturation_percent":round(100*sum(p>=pwm_limit-0.5 for p in pwm)/len(pwm),2) if pwm else None,
            "max_applied_pwm_permille":max(pwm) if pwm else None}
    latest=next((r for r in reversed(rows) if _value(r,"REL_L") is not None and _value(r,"REL_R") is not None),None)
    heading=None
    if latest and wheelbase_mm is not None:
        l=_value(latest,"REL_L")*math.pi*wheel_diameter_mm/counts_per_rev
        r=_value(latest,"REL_R")*math.pi*wheel_diameter_mm/counts_per_rev
        heading={"distance_l_mm":round(l,3),"distance_r_mm":round(r,3),
                 "distance_center_mm":round((l+r)/2,3),
                 "yaw_proxy_deg":round(math.degrees((r-l)/wheelbase_mm),3)}
    return {"samples":len(rows),"nominal_log_rate_hz":50,
            "observed_intervals":len(intervals),
            "interval_median_ms":sorted(intervals)[len(intervals)//2] if intervals else None,
            "gaps_over_30ms":gaps,"wheel_pid":wheels,"heading_shadow":heading,
            "limitations":["50 Hz telemetry does not prove 1 kHz control timing",
                           "encoder yaw proxy is not gyroscope yaw, and depends on wheel slip and geometry",
                           "offline heading shadow never commands motor PWM"]}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file",type=Path)
    p.add_argument("--wheelbase-mm",type=float,help="measured effective wheel separation")
    p.add_argument("--wheel-diameter-mm",type=float,default=34.0)
    p.add_argument("--counts-per-rev",type=float,default=1430.0)
    p.add_argument("--pwm-limit",type=float,default=200.0)
    p.add_argument("--out",type=Path)
    a=p.parse_args()
    with a.csv_file.open(newline="",encoding="utf-8-sig") as f:
        data=audit(csv.DictReader(f),wheelbase_mm=a.wheelbase_mm,
                   wheel_diameter_mm=a.wheel_diameter_mm,
                   counts_per_rev=a.counts_per_rev,pwm_limit=a.pwm_limit)
    output=json.dumps(data,indent=2,ensure_ascii=False)+"\n"
    if a.out:
        a.out.write_text(output,encoding="utf-8")
        print(a.out)
    else:
        print(output,end="")

if __name__=="__main__":
    main()
