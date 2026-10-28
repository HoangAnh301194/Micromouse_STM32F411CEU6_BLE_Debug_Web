#!/usr/bin/env python3
"""Per-channel nonlinear ADC->mm IR calibration from physical labelled captures.

Does NOT assume same gain for L/R or transfer side distances to front beams.
Never extrapolates beyond measured distance knots. Does NOT write MCU Flash.
Each record in the manifest is an independently labelled physical placement.
"""
from __future__ import annotations
import argparse
from collections import defaultdict
import csv
import html
import json
import math
from pathlib import Path
import statistics
import sys

CHANNELS=("L90","L45","L0","R0","R45","R90")
FIELDS=("trace_path","channel","distance_mm","trial_id","surface","light","split")
MIN_POINTS=5
MIN_REPEATS=2

def open_trace(path:Path, channel:str)->float:
    if not path.is_file():
        raise ValueError("Missing trace: "+str(path))
    values=[]
    with path.open("r",encoding="utf-8-sig",newline="") as f:
        reader=csv.DictReader(f)
        if not reader.fieldnames or not {"channel","ema10"}.issubset(reader.fieldnames):
            raise ValueError("Expected ir_filter_trace.csv: channel,ema10")
        for r in reader:
            if r["channel"]!=channel:continue
            x=float(r["ema10"])
            if not math.isfinite(x) or x<0 or x>4095:
                raise ValueError("Invalid EMA10 ADC in "+str(path))
            values.append(x)
    if len(values)<10:
        raise ValueError(f"{path}: too few samples for {channel}: {len(values)}")
    return statistics.median(values)

def generate_manifest(session:Path, out:Path)->list[dict]:
    rows=[]
    for setup in sorted(session.rglob("setup.json")):
        data=setup.parent/"data"/"ir_filter_trace.csv"
        if not data.is_file():continue
        try:cfg=json.loads(setup.read_text(encoding="utf-8"))
        except ValueError:continue
        for c,key in (("L0","distance_left_mm"),("R0","distance_right_mm")):
            dist=cfg.get(key)
            if dist is not None:
                rows.append({"trace_path":str(data.resolve()),"channel":c,
                             "distance_mm":dist,
                             "trial_id":str(cfg.get("trial",setup.parent.name)),
                             "surface":cfg.get("surface","unknown"),
                             "light":cfg.get("light","unknown"),
                             "split":"train"})
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader();w.writerows(rows)
    return rows

def read_manifest(path:Path)->list[dict]:
    with path.open("r",encoding="utf-8-sig",newline="") as f:
        reader=csv.DictReader(f)
        if not reader.fieldnames or not set(FIELDS).issubset(reader.fieldnames):
            raise ValueError("Manifest columns required: "+",".join(FIELDS))
        rows=[]
        for i,r in enumerate(reader,2):
            c=r["channel"]
            if c not in CHANNELS:raise ValueError(f"row {i} invalid channel {c}")
            split=r["split"].strip().lower()
            if split not in ("train","test"):raise ValueError(f"row {i}: split=train/test")
            dist=float(r["distance_mm"])
            if not 0<dist<500 or not math.isfinite(dist):
                raise ValueError(f"row {i}: invalid physical distance")
            path_val=Path(r["trace_path"])
            if not path_val.is_absolute():path_val=(path.parent/path_val).resolve()
            rows.append({"channel":c,"split":split,"distance_mm":dist,
                         "trial_id":r["trial_id"].strip(),
                         "surface":r["surface"].strip(),"light":r["light"].strip(),
                         "trace_path":str(path_val),
                         "adc":open_trace(path_val,c)})
    if len({(x["trace_path"],x["channel"],x["split"]) for x in rows})!=len(rows):
        raise ValueError("Duplicated trace/channel/split; independent trials required")
    return rows

def pava_nonincreasing(ys:list[float],weights:list[int])->list[float]:
    # Weighted isotonic regression (pool-adjacent violators).
    blocks=[]
    for i,(y,w) in enumerate(zip(ys,weights)):
        blocks.append([i,i,y*w,w])
        while len(blocks)>1 and blocks[-2][2]/blocks[-2][3] < blocks[-1][2]/blocks[-1][3]:
            b=blocks.pop();blocks[-1][1]=b[1]
            blocks[-1][2]+=b[2];blocks[-1][3]+=b[3]
    result=[0.0]*len(ys)
    for left,right,total,count in blocks:
        for i in range(left,right+1):result[i]=total/count
    return result

def predict_mm(knots:list[dict],adc:float)->float|None:
    # Lookup only interpolates in observed amplitude/distance support.
    if len(knots)<2:return None
    if adc>knots[0]["adc"] or adc<knots[-1]["adc"]:return None
    for a,b in zip(knots,knots[1:]):
        if a["adc"]>=adc>=b["adc"]:
            if a["adc"]-b["adc"]<1e-5:return None # flat/ambiguous interval
            t=(a["adc"]-adc)/(a["adc"]-b["adc"])
            return a["distance_mm"]+(b["distance_mm"]-a["distance_mm"])*t
    return None

def p95(values):
    if not values:return None
    values=sorted(values)
    return values[math.ceil(len(values)*.95)-1]

def analyze(manifest:Path,out:Path)->dict:
    samples=read_manifest(manifest)
    out.mkdir(parents=True,exist_ok=True)
    groups=defaultdict(list)
    for row in samples:groups[(row["channel"],row["surface"],row["light"])].append(row)
    reports={}
    all_knots=[]
    for (channel,surface,light),sample in sorted(groups.items()):
        key=f"{channel}::{surface}::{light}"
        train=[x for x in sample if x["split"]=="train"]
        test=[x for x in sample if x["split"]=="test"]
        by_dist=defaultdict(list)
        for p in train:by_dist[p["distance_mm"]].append(p)
        distances=sorted(by_dist)
        repeats={str(d):len(by_dist[d]) for d in distances}
        med=[statistics.median(x["adc"] for x in by_dist[d]) for d in distances]
        iso=pava_nonincreasing(med,[len(by_dist[d]) for d in distances])
        knots=[{"distance_mm":d,"adc":round(a,4),"raw_adc_median":round(m,4),
                "train_repeats":len(by_dist[d])} for d,a,m in zip(distances,iso,med)]
        residual=[]
        rejected=0
        for p in test:
            mm=predict_mm(knots,p["adc"])
            if mm is None:rejected+=1
            else:residual.append(mm-p["distance_mm"])
        p95abs=p95([abs(x) for x in residual])
        sufficient=len(distances)>=MIN_POINTS and all(len(by_dist[d])>=MIN_REPEATS for d in distances)
        # No pass claim without unseen physical placements at known distances.
        held_out=len(test)>=2 and rejected==0 and len(residual)>=2
        gate=sufficient and held_out and p95abs is not None and p95abs<=5.0
        reports[key]={
            "channel":channel,"surface":surface,"light":light,
            "train_placements":len(train),"test_placements":len(test),
            "unique_distances":len(distances),"repeats_per_distance":repeats,
            "knots":knots,"usable_mm_range":[min(distances),max(distances)] if distances else None,
            "holdout_n":len(residual),"holdout_unusable":rejected,
            "holdout_mae_mm":round(statistics.mean(abs(x) for x in residual),4) if residual else None,
            "holdout_p95_abs_mm":round(p95abs,4) if p95abs is not None else None,
            "offline_calibration_gate_passed":bool(gate),
            "hardware_acceptance":"NOT_CERTIFIED_BY_OFFLINE_TOOL",
            "warnings":(
                (["Need >=5 independent distances and >=2 placements at every distance."] if not sufficient else [])+
                (["Need >=2 held-out physical placements with no extrapolation."] if not held_out else [])+
                (["No LUT beyond observed ADC/mm range."] if distances else ["No training data."])
            ),
        }
        all_knots.extend({"channel":channel,"surface":surface,"light":light,**k} for k in knots)
    report={"manifest":str(manifest),"samples":len(samples),
            "physical_distance_reference":"operator_supplied_independent_measurement",
            "channel_groups":reports,"global_accepted":False,
            "note":"Offline LUT candidates only. Side and diagonal/front sensors require separate geometry and angular calibration."}
    (out/"calibration_report.json").write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    with (out/"lut_candidates.csv").open("w",newline="",encoding="utf-8") as f:
        cols=("channel","surface","light","distance_mm","adc","raw_adc_median","train_repeats")
        w=csv.DictWriter(f,fieldnames=cols);w.writeheader();w.writerows(all_knots)
    (out/"calibration_viewer.html").write_text(build_html(reports),encoding="utf-8")
    return report

def build_html(groups:dict)->str:
    payload=json.dumps(groups,ensure_ascii=True).replace("<","\\u003c")
    return """<!doctype html><html lang="vi"><meta charset="utf-8"><title>IR calibration</title>
<style>body{background:#101827;color:#edf1f8;font:14px system-ui;padding:24px;max-width:900px}
#chart{width:100%;height:420px}select{padding:6px}pre{background:#1c293b;padding:16px;white-space:pre-wrap}
a{color:#82cce4}</style><h1>IR ADC → distance calibration</h1>
<p>Separate curves by sensor / material / light. Measured knots only; no extrapolation.</p>
<select id="group"></select><canvas id="chart"></canvas><pre id="summary"></pre>
<script>const data="""+payload+""",select=document.getElementById('group');
Object.keys(data).forEach(k=>{let o=document.createElement('option');o.value=k;o.textContent=k;select.appendChild(o)});
function draw(){let r=data[select.value];if(!r)return;document.getElementById('summary').textContent=JSON.stringify({
gate:r.offline_calibration_gate_passed,range:r.usable_mm_range,repeats:r.repeats_per_distance,
p95:r.holdout_p95_abs_mm,warnings:r.warnings},null,2);
let c=document.getElementById('chart'),w=c.clientWidth||800,h=420;c.width=w*devicePixelRatio;c.height=h*devicePixelRatio;
let q=c.getContext('2d');q.scale(devicePixelRatio,devicePixelRatio);q.fillStyle='#1a2537';q.fillRect(0,0,w,h);
let L=55,R=w-25,T=20,B=h-50,k=r.knots;if(k.length<1)return;
const xx=k.map(z=>z.distance_mm),yy=k.map(z=>z.adc);
let xmin=Math.min(...xx),xmax=Math.max(...xx),ymin=Math.min(...yy)-20,ymax=Math.max(...yy)+20;
const x=v=>L+(v-xmin)/(xmax-xmin||1)*(R-L),y=v=>B-(v-ymin)/(ymax-ymin||1)*(B-T);
q.strokeStyle='#42627f';q.fillStyle='#b1c4d9';q.font='12px system-ui';
for(let i=0;i<=4;i++){let v=i/4;let py=B-v*(B-T);q.beginPath();q.moveTo(L,py);q.lineTo(R,py);q.stroke();q.fillText((ymin+v*(ymax-ymin)).toFixed(0),4,py+4);}
q.strokeStyle='#55d4c0';q.lineWidth=2;q.beginPath();k.forEach((v,i)=>i?q.lineTo(x(v.distance_mm),y(v.adc)):q.moveTo(x(v.distance_mm),y(v.adc)));q.stroke();
q.fillStyle='#55d4c0';for(let v of k){q.beginPath();q.arc(x(v.distance_mm),y(v.adc),4,0,2*Math.PI);q.fill();q.fillText(v.distance_mm+'mm',x(v.distance_mm)+6,y(v.adc)-7);}
q.fillStyle='#b1c4d9';q.fillText('Distance (mm)',w/2-30,h-13);q.fillText('ADC EMA 0.10',L+5,15);}
select.onchange=draw;draw();</script></html>"""

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    sub=ap.add_subparsers(dest="cmd",required=True)
    a=sub.add_parser("manifest",help="Generate measured L0/R0 manifest from previous qualification session")
    a.add_argument("--session",type=Path,required=True)
    a.add_argument("--output",type=Path,required=True)
    b=sub.add_parser("fit",help="Build per-channel monotone piecewise LUT from labelled trial CSV")
    b.add_argument("--manifest",type=Path,required=True)
    b.add_argument("--output",type=Path,default=Path("results/ir_lut_lab"))
    args=ap.parse_args(argv)
    try:
        if args.cmd=="manifest":
            items=generate_manifest(args.session,args.output)
            print("MANIFEST:",len(items),"rows. Set independent test split before approval.")
        else:
            report=analyze(args.manifest,args.output)
            print("CALIBRATION:",len(report["channel_groups"]),"sensor/surface/light groups;",
                  "offline candidate, no production LUT auto-deployed")
        return 0
    except (OSError,ValueError) as exc:
        print("ERROR:",exc,file=sys.stderr);return 2
if __name__=="__main__":raise SystemExit(main())
