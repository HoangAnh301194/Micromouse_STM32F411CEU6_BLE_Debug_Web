"""Offline/GUI Mode-1 report: plot time-series and candidate thresholds.
No calibration values are ever transmitted to motor control by this module.
"""
import csv
import json
from math import sqrt
from pathlib import Path
from statistics import mean
from xml.sax.saxutils import escape
CHANNELS=("L0","L45","L90","R0","R45","R90")
COLORS=("#10b981","#3b82f6","#f59e0b","#ef4444","#a78bfa","#64748b")
EXPECTED=("t_ms",)+CHANNELS
def write_ir_report(csv_path, thresholds=None):
    path=Path(csv_path)
    with path.open(newline="",encoding="utf-8") as f:
        reader=csv.DictReader(f)
        if tuple(reader.fieldnames or ())!=EXPECTED:
            raise ValueError("Unexpected Mode 1 CSV schema")
        rows=[]
        for line in reader:
            try:
                t=int(line["t_ms"])
                adc=[int(line[ch]) for ch in CHANNELS]
            except (ValueError,TypeError,KeyError):
                continue
            if t<0 or any(v<0 or v>4095 for v in adc): continue
            rows.append((t,*adc))
    if not rows: raise ValueError("Empty calibration capture")
    rows.sort(key=lambda r:r[0])
    thresholds=thresholds or {}
    stats={}
    for i,ch in enumerate(CHANNELS):
        v=[r[i+1] for r in rows]
        vals=sorted(v)
        data={"min":min(v),"max":max(v),"mean":round(mean(v),2),
              "std":round(sqrt(sum((x-mean(v))**2 for x in v)/len(v)),2),
              "p10":vals[int(.1*(len(vals)-1))],
              "p90":vals[int(.9*(len(vals)-1))]}
        if i in thresholds:
            data["wall_off"],data["wall_on"]=thresholds[i]
        stats[ch]=data
    summary={"source":path.name,"samples":len(rows),"duration_ms":rows[-1][0]-rows[0][0],
             "calibration_quality":"CANDIDATE_ONLY_NOT_GEOMETRY_VALIDATED",
             "auto_apply":False,"channels":stats}
    summary_path=path.with_name(path.stem+"_summary.json")
    summary_path.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    W,H=960,470
    x0,x1,y0,y1=65,927,45,410
    t0,t1=rows[0][0],max(rows[-1][0],rows[0][0]+1)
    x=lambda t:x0+(t-t0)/(t1-t0)*(x1-x0)
    y=lambda adc:y1-adc/4095*(y1-y0)
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}">',
           '<rect width="100%" height="100%" fill="#0f172a"/>',
           '<text x="65" y="25" font-family="sans-serif" font-size="17" fill="#f8fafc">Mode 1 IR 360° | ADC versus MCU time</text>']
    for tick in (0,1024,2048,3072,4095):
        yy=y(tick)
        parts.append(f'<line x1="{x0}" x2="{x1}" y1="{yy:.1f}" y2="{yy:.1f}" stroke="#334155"/>')
        parts.append(f'<text x="12" y="{yy+4:.1f}" font-size="12" fill="#cbd5e1">{tick}</text>')
    for i,ch in enumerate(CHANNELS):
        points=" ".join(f'{x(r[0]):.1f},{y(r[i+1]):.1f}' for r in rows)
        parts.append(f'<polyline fill="none" stroke="{COLORS[i]}" stroke-width="1.6" points="{points}"/>')
        parts.append(f'<text x="{x0+i*143}" y="445" font-family="sans-serif" font-size="13" fill="{COLORS[i]}">{escape(ch)}</text>')
    parts.append('</svg>')
    image_path=path.with_name(path.stem+"_ir_plot.svg")
    image_path.write_text("\n".join(parts),encoding="utf-8")
    return summary_path
if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument("csv_path")
    args=ap.parse_args()
    print(write_ir_report(args.csv_path))
