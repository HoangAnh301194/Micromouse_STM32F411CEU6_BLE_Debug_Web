#!/usr/bin/env python3
"""Offline IR wall loss / next-cell observation / cautious six-sensor fusion.

Uses MC trace.csv (six IR @200Hz + encoder/IMU) and labels/model supplied
by real measurements. UNKNOWN is a first-class state; no motor control,
no arbitrary Wall-PID cutoffs and no unlabelled physical acceptance.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys

CHANNELS=("L90","L45","L0","R0","R45","R90")
PAIR={"left":("L0","L45"),"right":("R0","R45"),"front":("L90","R90")}
STATES=("UNKNOWN","OPEN","WALL")

def read_trace(path:Path)->list[dict]:
    with path.open("r",encoding="utf-8-sig",newline="") as f:
        reader=csv.DictReader(f)
        required=set(("ms","distance_mm","yaw_deg","valid",*CHANNELS))
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("trace needs ms,distance_mm,yaw_deg,valid and all six IR")
        rows=[]
        for i,r in enumerate(reader,2):
            try:
                item={key:float(r[key]) for key in ("ms","distance_mm","yaw_deg")}
                item["valid"]=int(r["valid"])
                if item["valid"] not in (0,1):
                    raise ValueError("valid must be 0 or 1")
                for ch in CHANNELS:
                    item[ch]=float(r[ch])
                    if not 0<=item[ch]<=4095:raise ValueError("ADC out of 0..4095")
                if any(not math.isfinite(v) for k,v in item.items() if k!="valid"):
                    raise ValueError("nonfinite number")
                if rows and item["ms"]<=rows[-1]["ms"]:
                    raise ValueError("time not increasing")
                rows.append(item)
            except (ValueError,KeyError) as exc:
                raise ValueError(f"{path}:{i}: {exc}") from exc
    if len(rows)<5:raise ValueError("need >=5 samples for event/fusion analysis")
    return rows

def save(out:Path,filename:str,data:dict|list):
    out.mkdir(parents=True,exist_ok=True)
    (out/filename).write_text(json.dumps(data,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")

def write_csv(out:Path,name:str,rows:list[dict],fields):
    out.mkdir(parents=True,exist_ok=True)
    with (out/name).open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction="ignore")
        writer.writeheader();writer.writerows(rows)

def detect_edges(rows:list[dict],window:int=4,min_relative:float=.08,
                 refractory_mm:float=10.0)->list[dict]:
    if window<1 or not (0<min_relative<1) or refractory_mm<0:
        raise ValueError("window>=1, 0<min_relative<1, refractory>=0")
    found=[]
    for ch in CHANNELS:
        last_mm=float("-inf")
        for i in range(window,len(rows)):
            p,q=rows[i-window],rows[i]
            if not q["valid"] or not p["valid"]:continue
            # Signed contrast is bounded even for 0->3500 nonlinear jumps.
            delta=(q[ch]-p[ch])/(q[ch]+p[ch]+32)
            if abs(delta)<min_relative:continue
            # Detect the PEAK excursion in this window, not every
            # derivative step in a long wall gradient.
            prev=(rows[i-1][ch]-rows[max(0,i-window-1)][ch])/(
                  rows[i-1][ch]+rows[max(0,i-window-1)][ch]+32)
            if abs(delta)<abs(prev):continue
            direction="WALL_APPEAR_CANDIDATE" if delta>0 else "WALL_LOST_CANDIDATE"
            x=q["distance_mm"]
            if x-last_mm<refractory_mm:continue
            found.append({"channel":ch,"type":direction,
                          "ms":q["ms"],"distance_mm":round(x,3),
                          "relative_change":round(delta,5),
                          "cell_index":int(x//180) if x>=0 else -1})
            last_mm=x
    return sorted(found,key=lambda x:(x["ms"],x["channel"]))

def read_edge_truth(path:Path)->list[dict]:
    with path.open("r",encoding="utf-8-sig",newline="") as f:
        reader=csv.DictReader(f)
        if not reader.fieldnames or not {"channel","type","distance_mm"}.issubset(reader.fieldnames):
            raise ValueError("ground truth columns: channel,type,distance_mm")
        gt=[]
        for row in reader:
            c=row["channel"]
            t=row["type"].upper().strip()
            if c not in CHANNELS or t not in ("WALL_APPEAR","WALL_LOST"):
                raise ValueError("Invalid ground-truth channel/type")
            d=float(row["distance_mm"])
            if not math.isfinite(d):raise ValueError("Invalid truth distance")
            gt.append({"channel":c,"type":t,"distance_mm":d})
    return gt

def score_edges(events:list[dict],truth:list[dict],max_error_mm:float)->dict:
    matched=set()
    offsets=[]
    missed=[]
    for gt in truth:
        opts=[(i,e["distance_mm"]-gt["distance_mm"]) for i,e in enumerate(events)
              if i not in matched and e["channel"]==gt["channel"] and
              e["type"]==gt["type"]+"_CANDIDATE" and
              abs(e["distance_mm"]-gt["distance_mm"])<=max_error_mm]
        if not opts:missed.append(gt)
        else:
            i,err=min(opts,key=lambda x:abs(x[1]))
            matched.add(i)
            offsets.append({"channel":gt["channel"],"type":gt["type"],
                            "offset_mm":round(err,3),
                            "reference_mm":gt["distance_mm"]})
    false=[e for i,e in enumerate(events) if i not in matched]
    return {"truth_events":len(truth),"matched":len(matched),
            "missed":missed,"unmatched_candidates":false,
            "offsets":offsets,
            "mae_mm":round(statistics.mean(abs(x["offset_mm"]) for x in offsets),3) if offsets else None,
            "classification":"OFFLINE_CANDIDATES_ONLY_NOT_HARDWARE_QUALIFIED"}

def load_model(path:Path)->dict:
    obj=json.loads(path.read_text(encoding="utf-8"))
    model=obj.get("model")
    if not isinstance(model,dict) or any(ch not in model for ch in CHANNELS):
        raise ValueError("model must be wall_detection_report.json from maze_readiness_suite.py")
    for ch in CHANNELS:
        if any(k not in model[ch] for k in ("threshold","hysteresis_adc")):
            raise ValueError("Model missing per-channel threshold/hysteresis for "+ch)
    return model

def load_geometry(path:Path|None)->dict|None:
    if path is None:return None
    g=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(g,dict) or g.get("measured_and_reviewed") is not True:
        raise ValueError("Geometry must include measured_and_reviewed=true")
    offsets=g.get("forward_projection_mm")
    if not isinstance(offsets,dict) or any(ch not in offsets for ch in CHANNELS):
        raise ValueError("Provide forward_projection_mm for six channels")
    for ch in CHANNELS:
        if not math.isfinite(float(offsets[ch])) or abs(float(offsets[ch]))>180:
            raise ValueError("Invalid sensor projection distance")
    return g

def classify_trace(rows:list[dict],model:dict,debounce:int=3,
                   geometry:dict|None=None)->tuple[list[dict],dict]:
    if debounce<1:raise ValueError("debounce must be >=1")
    previous={ch:"UNKNOWN" for ch in CHANNELS}
    pending={ch:None for ch in CHANNELS}
    streak={ch:0 for ch in CHANNELS}
    combined=[]
    observations=[]
    for r in rows:
        line={"ms":r["ms"],"distance_mm":r["distance_mm"],"yaw_deg":r["yaw_deg"]}
        for ch in CHANNELS:
            if not r["valid"]:
                previous[ch]="UNKNOWN";pending[ch]=None;streak[ch]=0
                line[ch]="UNKNOWN"
                continue
            th=float(model[ch]["threshold"])
            margin=float(model[ch]["hysteresis_adc"])*.5
            value=r[ch]
            desired="WALL" if value>=th+margin else "OPEN" if value<=th-margin else "UNKNOWN"
            if desired=="UNKNOWN":
                # Keep UNKNOWN explicitly rather than silently carrying
                # stale, weak evidence over a physical wall edge.
                previous[ch]="UNKNOWN";pending[ch]=None;streak[ch]=0
            elif desired!=previous[ch]:
                if pending[ch]!=desired:pending[ch]=desired;streak[ch]=1
                else:streak[ch]+=1
                if streak[ch]>=debounce:
                    previous[ch]=desired;pending[ch]=None;streak[ch]=0
            line[ch]=previous[ch]
        for side,(a,b) in PAIR.items():
            # Diagonal sensors may look into *different* cells. This
            # pair vote is provisional and never commits to a map without
            # explicit physical geometry and matching projection cell.
            aa,bb=line[a],line[b]
            agree=aa==bb and aa!="UNKNOWN"
            line[side]=aa if agree else "UNKNOWN"
            line[side+"_conflict"]=aa!=bb and aa!="UNKNOWN" and bb!="UNKNOWN"
        combined.append(line)
        if geometry is not None:
            offsets=geometry["forward_projection_mm"]
            for side,(a,b) in PAIR.items():
                a_cell=math.floor((r["distance_mm"]+float(offsets[a]))/180)
                b_cell=math.floor((r["distance_mm"]+float(offsets[b]))/180)
                # Need agreement, same cell and credible heading for a
                # candidate map observation; this is NOT sensor-ray tracing.
                if (line[side]!="UNKNOWN" and a_cell==b_cell and
                    r["valid"] and abs(r["yaw_deg"]-rows[0]["yaw_deg"])<=10):
                    target=a_cell
                    current=math.floor(r["distance_mm"]/180)
                    observations.append({"side":side,"target_cell":target,
                        "observer_cell":current,"ms":r["ms"],
                        "distance_mm":r["distance_mm"],
                        "lookahead_mm":round((target*180)-r["distance_mm"],3),
                        "state":line[side],"source":a+"+"+b})
    conflicts={side:sum(int(x[side+"_conflict"]) for x in combined) for side in PAIR}
    report={"frames":len(rows),"debounce_frames":debounce,
            "wall_state_conflicts":conflicts,
            "unknown_frames":{side:sum(x[side]=="UNKNOWN" for x in combined)
                              for side in PAIR},
            "geometry_confirmed":geometry is not None,
            "cell_observations":len(observations),
            "status":"OFFLINE_REPLAY_NOT_MAZE_ACCEPTED",
            "warning":"Projection-offset cell association is preliminary; validate beam-wall geometry and separate truth labels."}
    return observations,{"trace":combined,"summary":report}

def build_wall_dashboard(states:list[dict],observations:list[dict])->str:
    # Offline self-contained 9-lane state timeline. No network/CDN.
    rows=json.dumps(states,separators=(",",":"),ensure_ascii=True).replace("<","\\u003c")
    events=json.dumps(observations,separators=(",",":"),ensure_ascii=True).replace("<","\\u003c")
    return """<!doctype html><html lang="vi"><meta charset="utf-8">
<title>Micromouse Wall States</title>
<style>body{font:14px system-ui;color:#eef2fb;background:#101827;padding:22px;max-width:1200px}
section{border:1px solid #40516a;background:#1b293c;border-radius:10px;padding:15px;margin:10px 0}
canvas{width:100%;height:390px}pre{white-space:pre-wrap}select{padding:6px}
small{color:#a7bdd3}</style>
<h1>6 IR + FUSION — WALL / OPEN / UNKNOWN</h1>
<section><p>WALL <b style="color:#42ccaa">■</b> OPEN <b style="color:#669dde">■</b>
UNKNOWN <b style="color:#9a9da6">■</b> CONFLICT <b style="color:#f56d80">■</b></p>
<label>Trục X <select id="axis"><option value="distance_mm">Encoder mm</option>
<option value="ms">Time ms</option></select></label><canvas id="vis"></canvas>
<small>180mm/cell; biểu diễn ứng viên offline. UNKNOWN khác OPEN.
Góc chiếu L45/R45 có thể quan sát cell khác L0/R0.</small></section>
<section>Frame <input id="index" type="range" min="0" value="0" style="width:90%">
<pre id="frame"></pre></section><section>Next-cell observations <pre id="next"></pre></section>
<script>
const rows="""+rows+""",obs="""+events+""";
const names=['L90','L45','L0','R0','R45','R90','left','right','front'];
const palette={WALL:'#42ccaa',OPEN:'#669dde',UNKNOWN:'#838e9e'};
const slider=document.getElementById('index'),axis=document.getElementById('axis');
slider.max=Math.max(0,rows.length-1);
function draw(){let c=document.getElementById('vis'),w=Math.max(450,c.clientWidth),h=390;
c.width=w*devicePixelRatio;c.height=h*devicePixelRatio;let g=c.getContext('2d');
g.scale(devicePixelRatio,devicePixelRatio);g.fillStyle='#132033';g.fillRect(0,0,w,h);
if(!rows.length)return;const L=86,R=w-25,T=15,xkey=axis.value;
let xmin=rows[0][xkey],xmax=rows[rows.length-1][xkey];
const pos=x=>L+(x-xmin)/(xmax-xmin||1)*(R-L);
if(xkey==='distance_mm'){g.setLineDash([3,4]);g.strokeStyle='#66798e';
for(let d=Math.ceil(xmin/180)*180;d<=xmax;d+=180){let px=pos(d);g.beginPath();
g.moveTo(px,15);g.lineTo(px,350);g.stroke();g.fillStyle='#b9cadc';g.fillText(d/180+'c',px+2,12);}
g.setLineDash([]);}
names.forEach((name,j)=>{const Y=28+j*35;g.fillStyle='#e1eaf6';g.fillText(name,8,Y+15);
rows.forEach((r,i)=>{let px=pos(r[xkey]),right=pos(i<rows.length-1?rows[i+1][xkey]:xmax);
g.fillStyle=palette[r[name]]||palette.UNKNOWN;g.fillRect(px,Y,Math.max(1,right-px),26);
if(j>=6 && r[name+'_conflict']){g.fillStyle='#f56d80';g.fillRect(px,Y,3,26);}});});
const r=rows[Number(slider.value)];if(r){g.strokeStyle='#fff';let px=pos(r[xkey]);
g.beginPath();g.moveTo(px,16);g.lineTo(px,355);g.stroke();}
document.getElementById('frame').textContent=JSON.stringify(r,null,2);
document.getElementById('next').textContent=JSON.stringify(obs.slice(0,70),null,2)+
(obs.length>70?' ...':'');}
slider.oninput=draw;axis.onchange=draw;window.onresize=draw;draw();
</script></html>"""


def run_edges(trace:Path,out:Path,truth:Path|None,window:int,
              relative:float,refractory:float,max_error:float)->dict:
    rows=read_trace(trace)
    events=detect_edges(rows,window,relative,refractory)
    report={"trace":str(trace),"frames":len(rows),"candidates":len(events),
            "parameters":{"window":window,"min_relative_change":relative,
                          "refractory_mm":refractory},
            "hardware_accepted":False,
            "candidates_not_motion_cutoffs":True,
            "ground_truth":str(truth) if truth else None,
            "scoring":score_edges(events,read_edge_truth(truth),max_error) if truth else None}
    write_csv(out,"edge_candidates.csv",events,
              ("channel","type","ms","distance_mm","relative_change","cell_index"))
    save(out,"edge_report.json",report)
    return report

def run_fusion(trace:Path,model_file:Path,out:Path,
               debounce:int,geometry_file:Path|None)->dict:
    rows=read_trace(trace)
    model=load_model(model_file)
    geometry=load_geometry(geometry_file)
    observations,data=classify_trace(rows,model,debounce,geometry)
    states=data["trace"]
    columns=("ms","distance_mm","yaw_deg",*CHANNELS,
             "left","right","front",
             "left_conflict","right_conflict","front_conflict")
    write_csv(out,"wall_state_timeline.csv",states,columns)
    write_csv(out,"cell_observations.csv",observations,
              ("side","target_cell","observer_cell","ms","distance_mm",
               "lookahead_mm","state","source"))
    report=data["summary"]
    report["model"]=str(model_file)
    report["geometry"]=str(geometry_file) if geometry_file else None
    report["next_cell_candidates"]={
        side:sum(o["target_cell"]>o["observer_cell"] for o in observations
                 if o["side"]==side) for side in PAIR}
    save(out,"fusion_report.json",report)
    save(out,"timeline.json",states)
    (out/"wall_state_viewer.html").write_text(
        build_wall_dashboard(states,observations),encoding="utf-8")
    return report

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    sub=ap.add_subparsers(dest="cmd",required=True)
    p=sub.add_parser("edges")
    p.add_argument("--trace",type=Path,required=True)
    p.add_argument("--truth",type=Path)
    p.add_argument("--out",type=Path,default=Path("results/maze_edge_lab"))
    p.add_argument("--window",type=int,default=4)
    p.add_argument("--relative-change",type=float,default=.08)
    p.add_argument("--refractory-mm",type=float,default=10.0)
    p.add_argument("--max-error-mm",type=float,default=20.0)
    q=sub.add_parser("fusion")
    q.add_argument("--trace",type=Path,required=True)
    q.add_argument("--model",type=Path,required=True)
    q.add_argument("--geometry",type=Path)
    q.add_argument("--debounce",type=int,default=3)
    q.add_argument("--out",type=Path,default=Path("results/maze_fusion_lab"))
    args=ap.parse_args(argv)
    try:
        if args.cmd=="edges":
            result=run_edges(args.trace,args.out,args.truth,args.window,
                             args.relative_change,args.refractory_mm,args.max_error_mm)
            print("EDGE CANDIDATES:",result["candidates"],"|",args.out)
        else:
            result=run_fusion(args.trace,args.model,args.out,args.debounce,args.geometry)
            print("FUSION:",result["frames"],"frames, UNKNOWN:",
                  result["unknown_frames"],"|",args.out)
        return 0
    except (OSError,ValueError,ZeroDivisionError) as exc:
        print("ERROR:",exc,file=sys.stderr);return 2

if __name__=="__main__":
    raise SystemExit(main())
