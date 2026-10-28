#!/usr/bin/env python3
"""Parse 200Hz MC, BLE log from motor-locked STM32 collector.
Emit validated trace.csv + JSON + self-contained interactive six-IR dashboard.
No BLE connection, UART writes, MCU flashing or motor commands.
"""
from __future__ import annotations
import argparse
import csv
import html
import json
from pathlib import Path
import statistics
import sys

FIELDS=("seq","ms","scan","enc_l","enc_r","yaw_cdeg","rate_cdps",
        "ir_age_us","valid","L90","L45","L0","R0","R45","R90")
IR=("L90","L45","L0","R0","R45","R90")
MM_PER_TICK=3.141592653589793*34.0/1430.0
HASH_INIT=2166136261

def fnv32(hash_value: int, raw: bytes)->int:
    for b in raw:
        hash_value=((hash_value^b)*16777619)&0xffffffff
    return hash_value

def decode_text(text: str)->tuple[list[dict],dict]:
    begin=None
    end=None
    rows=[]
    hash_value=HASH_INIT
    failures=[]
    for lineno,line in enumerate(text.splitlines(keepends=True),1):
        pos=line.find("MC,")
        if pos<0:continue
        wire=line[pos:].rstrip("\r\n")
        parts=wire.split(",")
        if len(parts)>1 and parts[1]=="BEGIN":
            if begin is not None:
                failures.append("MULTIPLE_BEGIN")
                continue
            begin={}
            for item in parts[2:]:
                k,sep,v=item.partition("=")
                if sep:begin[k]=v
        elif len(parts)>1 and parts[1]=="END":
            if end is not None:
                failures.append("MULTIPLE_END")
            else:
                end={}
                for item in parts[2:]:
                    k,sep,v=item.partition("=")
                    if sep:end[k]=v
        elif len(parts)>1 and parts[1].isdigit():
            if begin is None or end is not None:
                failures.append(f"ROW_OUTSIDE_FRAME:{lineno}")
                continue
            if len(parts)!=len(FIELDS)+1:
                failures.append(f"BAD_COLUMNS:{lineno}")
                continue
            try:
                values=list(map(int,parts[1:]))
            except ValueError:
                failures.append(f"BAD_NUMBER:{lineno}")
                continue
            row=dict(zip(FIELDS,values))
            if row["valid"] not in (0,1) or row["ir_age_us"]<0:
                failures.append(f"BAD_VALIDITY:{lineno}")
            if any(not 0<=row[ch]<=4095 for ch in IR):
                failures.append(f"ADC_RANGE:{lineno}")
            if row["seq"]!=len(rows):
                failures.append(f"SEQ_GAP:{lineno}")
            if rows and (row["ms"]<=rows[-1]["ms"] or row["scan"]<rows[-1]["scan"]):
                failures.append(f"NON_MONOTONIC:{lineno}")
            hash_value=fnv32(hash_value,(wire+"\n").encode("ascii",errors="replace"))
            row["distance_mm"]=round((row["enc_l"]+row["enc_r"])*.5*MM_PER_TICK,3)
            row["yaw_deg"]=row["yaw_cdeg"]*.01
            row["gyro_dps"]=row["rate_cdps"]*.01
            row["cell_index"]=int(row["distance_mm"]//180) if row["distance_mm"]>=0 else -1
            rows.append(row)
    if begin is None:failures.append("NO_BEGIN")
    if end is None:failures.append("NO_END")
    if begin is not None:
        try:
            if int(begin["n"])!=len(rows):failures.append("BEGIN_COUNT_MISMATCH")
            if int(begin["locked"])!=1:failures.append("MOTOR_NOT_LOCKED")
        except (KeyError,ValueError):failures.append("INVALID_BEGIN")
    if end is not None:
        try:
            if int(end["n"])!=len(rows):failures.append("END_COUNT_MISMATCH")
            if int(end["locked"])!=1:failures.append("END_MOTOR_NOT_LOCKED")
            if int(end["fnv32"],16)!=hash_value:failures.append("FNV32_MISMATCH")
        except (KeyError,ValueError):failures.append("INVALID_END")
    good=[r for r in rows if r["valid"]==1]
    intervals=[b["ms"]-a["ms"] for a,b in zip(rows,rows[1:])]
    stats={
        "format":"MC_BLE_SIX_IR_V1",
        "complete":not failures,
        "faults":list(dict.fromkeys(failures)),
        "expected_count":int(begin.get("n",0)) if begin and str(begin.get("n","")).isdigit() else None,
        "received_count":len(rows),
        "valid_frames":len(good),
        "invalid_frames":len(rows)-len(good),
        "hash_computed":f"{hash_value:08x}",
        "begin":begin,"end":end,
        "duration_ms":rows[-1]["ms"]-rows[0]["ms"] if len(rows)>1 else None,
        "sample_gap_p95_ms":sorted(intervals)[int(.95*(len(intervals)-1))] if intervals else None,
        "sample_gap_max_ms":max(intervals,default=None),
        "distance_end_mm":rows[-1]["distance_mm"] if rows else None,
        "yaw_span_deg":max(r["yaw_deg"] for r in good)-min(r["yaw_deg"] for r in good) if good else None,
        "experimental_hardware_qualified":False,
        "note":"BLE TX only; MCU UART acceptance is not radio delivery. FNV validates end-to-end text if all lines present.",
    }
    return rows,stats

def build_html(rows:list[dict],stats:dict)->str:
    # Entire file is portable/offline: no CDN, fonts or network.
    payload=json.dumps(rows,separators=(",",":"),ensure_ascii=True).replace("<","\\u003c")
    meta=html.escape(json.dumps({k:v for k,v in stats.items() if k not in ("begin","end")},
                                ensure_ascii=False,indent=2))
    return """<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Micromouse 6-IR Diagnostic</title>
<style>
body{font:14px system-ui,Arial;background:#101827;color:#edf1f8;margin:0;padding:22px;max-width:1300px}
h1{font-size:25px}section{background:#1a2537;border:1px solid #34445d;border-radius:12px;padding:14px;margin:15px 0}
.controls{display:flex;gap:14px;flex-wrap:wrap;align-items:center}label{white-space:nowrap}
select,input{accent-color:#5bd5bd}canvas{width:100%;height:300px;display:block}
#timeline{height:350px}pre{white-space:pre-wrap;color:#afc0d2}
details{margin-top:10px}small{color:#9fb1c6}code{color:#8fe8d1}
</style><h1>Micromouse — 6 IR / encoder / gyro</h1>
<section><div class="controls"><label>X-axis <select id="axis"><option value="ms">Time (ms)</option>
<option value="distance_mm">Distance (mm)</option></select></label>
<label>Cell = 180 mm</label><label>Slide cursor <input id="cursor" type="range" min="0" max="1" value="0"></label>
<label><input type="checkbox" id="invalid" checked>Show invalid samples</label></div>
<p id="status"></p><small>RAW ADC is not distance. L90/R90 are rear-mounted, looking forward. 200Hz nominal, real sampling gaps reported below.</small></section>
<section><h2>All 6 IR — EMA 0.10, ADC</h2><canvas id="ir"></canvas></section>
<section><h2>Encoder distance / integrated gyro</h2><canvas id="pose"></canvas></section>
<section><h2>Gyro rate / IR sample age</h2><canvas id="health"></canvas></section>
<section><h2>Selected sample</h2><pre id="sample"></pre></section>
<details><summary>Capture QC / warnings</summary><pre>""" + meta + """</pre></details>
<script>
const rows="""+payload+""";
const sensors=['L90','L45','L0','R0','R45','R90'];
const colors=['#ffba66','#b68afa','#57d2c7','#5caaf2','#ff7d9a','#f0de6d'];
const axis=document.getElementById('axis'),cursor=document.getElementById('cursor');
const showInvalid=document.getElementById('invalid');
cursor.max=Math.max(0,rows.length-1);
function chart(id,series,cellMarks){
 const c=document.getElementById(id),w=Math.max(380,c.clientWidth),h=c.clientHeight||300;
 c.width=w*devicePixelRatio;c.height=h*devicePixelRatio;
 const q=c.getContext('2d');q.scale(devicePixelRatio,devicePixelRatio);
 q.fillStyle='#142033';q.fillRect(0,0,w,h);const L=62,T=20,R=w-20,B=h-37;
 const valid=rows.filter(r=>showInvalid.checked||r.valid);
 if(!valid.length)return;
 const xs=valid.map(r=>r[axis.value]),mn=Math.min(...xs),mx=Math.max(...xs);
 const values=valid.flatMap(r=>series.map(s=>r[s.key]));
 let ymin=Math.min(...values),ymax=Math.max(...values);if(ymin===ymax){ymin-=1;ymax+=1;}
 const x=r=>L+(r[axis.value]-mn)/(mx-mn||1)*(R-L);
 const y=(v)=>B-(v-ymin)/(ymax-ymin)*(B-T);
 q.lineWidth=1;q.strokeStyle='#304563';q.fillStyle='#adc1d6';q.font='12px system-ui';
 for(let k=0;k<=4;k++){const yy=T+(B-T)*k/4;q.beginPath();q.moveTo(L,yy);q.lineTo(R,yy);q.stroke();q.fillText((ymax-(ymax-ymin)*k/4).toFixed(1),3,yy+4);}
 if(cellMarks&&axis.value==='distance_mm'){
  q.setLineDash([4,5]);q.strokeStyle='#536c86';
  for(let d=Math.ceil(mn/180)*180;d<=mx;d+=180){const xx=L+(d-mn)/(mx-mn||1)*(R-L);
   q.beginPath();q.moveTo(xx,T);q.lineTo(xx,B);q.stroke();q.fillText((d/180).toFixed(0)+'c',xx+2,T+12);}
  q.setLineDash([]);
 }
 for(let s of series){q.beginPath();q.strokeStyle=s.color;q.lineWidth=1.5;let started=false;
 for(let r of valid){const xx=x(r),yy=y(r[s.key]);
   if(!started){q.moveTo(xx,yy);started=true;}else q.lineTo(xx,yy);}
 q.stroke();}
 const selected=rows[Number(cursor.value)]||rows[0];q.setLineDash([3,4]);q.strokeStyle='#ffffff';
 const mark=x(selected);q.beginPath();q.moveTo(mark,T);q.lineTo(mark,B);q.stroke();q.setLineDash([]);
 q.fillStyle='#bdcede';q.fillText(mn.toFixed(1),L,B+19);q.fillText(mx.toFixed(1),R-55,B+19);
 let pos=L;for(let s of series){q.fillStyle=s.color;q.fillRect(pos,T+4,9,9);q.fillText(s.label,pos+13,T+13);pos+=Math.min(135,95+(s.label.length>10?25:0));}
}
function draw(){
 if(!rows.length){document.getElementById('status').textContent='No frames';return;}
 const n=Number(cursor.value),s=rows[n]||rows[0];
 document.getElementById('status').textContent=
 'frames='+rows.length+'  sample='+n+'  cell≈'+s.cell_index+
 '  t='+s.ms+'ms  x='+s.distance_mm+'mm  scan='+s.scan+'  valid='+s.valid;
 document.getElementById('sample').textContent=JSON.stringify(s,null,2);
 chart('ir',sensors.map((key,i)=>({key,label:key,color:colors[i]})),true);
 chart('pose',[{key:'distance_mm',label:'Distance mm',color:'#57d2c7'},
               {key:'yaw_deg',label:'Yaw deg',color:'#ffba66'}],true);
 chart('health',[{key:'gyro_dps',label:'Rate dps',color:'#ff7d9a'},
                 {key:'ir_age_us',label:'IR age us',color:'#5caaf2'}],true);
}
axis.onchange=draw;cursor.oninput=draw;showInvalid.onchange=draw;window.onresize=draw;draw();
</script></html>"""

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input",type=Path,required=True,help="Exported BLE text (full MC,BEGIN..END)")
    ap.add_argument("--output",type=Path,default=None)
    args=ap.parse_args(argv)
    try:
        rows,qc=decode_text(args.input.read_text(encoding="utf-8",errors="replace"))
        out=args.output or args.input.parent/(args.input.stem+"_maze_capture")
        out.mkdir(parents=True,exist_ok=True)
        with (out/"trace.csv").open("w",newline="",encoding="utf-8") as f:
            cols=FIELDS+("distance_mm","yaw_deg","gyro_dps","cell_index")
            w=csv.DictWriter(f,fieldnames=cols);w.writeheader();w.writerows(rows)
        (out/"capture_qc.json").write_text(json.dumps(qc,indent=2)+"\n",encoding="utf-8")
        (out/"viewer.html").write_text(build_html(rows,qc),encoding="utf-8")
        print("MC capture", "COMPLETE" if qc["complete"] else "INVALID",
              "| frames",len(rows),"|",out)
        if qc["faults"]:print("FAULTS:",", ".join(qc["faults"]))
        return 0 if qc["complete"] else 2
    except (ValueError,OSError) as exc:
        print("ERROR",exc,file=sys.stderr);return 2

if __name__=="__main__":
    raise SystemExit(main())
