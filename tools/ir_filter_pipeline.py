#!/usr/bin/env python3
"""Capture six IR filter candidates with ON/OFF ground truth and repeatable labels.

Reads PA9 UART at 115200 (motor-locked IR_FILTER_DIAG firmware).
CSV records stay aligned to their own scan number. No motor commands.
Optional matplotlib plots are generated per sensor; no dependency for capture.
"""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import json
from pathlib import Path
import statistics as stats
import sys
import time

CHANNELS=("L90","L45","L0","R0","R45","R90")
KEYS=("d","med","e10","e25","hy")
ROOT=Path(__file__).resolve().parent.parent
FIELDS=["ms","n","us"]
CSV_FIELDS=("ms","scan","us","channel","raw","median3","ema10","ema25","hybrid")
RAW_FIELDS=("ms","scan","channel","off","on","difference")
FILTER_MAP={"raw":"raw","median3":"median3","ema10":"ema10","ema25":"ema25","hybrid":"hybrid"}

def parse(line: str) -> dict | None:
    parts=line.strip().split(",")
    if not parts or parts[0] not in ("IRF","IRB","IRH"):
        return None
    out={"type":parts[0]}
    for part in parts[1:]:
        key,sep,value=part.partition("=")
        if not sep: return None
        if key in ("d","med","e10","e25","hy","off","on"):
            try: values=[int(v) for v in value.split("|")]
            except ValueError:return None
            if len(values)!=6 or any(x<0 or x>4095 for x in values):
                return None
            out[key]=values
        else:
            try:out[key]=int(value)
            except ValueError:return None
    required={"IRF":(*FIELDS,*KEYS),"IRB":("ms","n","off","on"),
              "IRH":("ms","n","fail","dma_err","dma_to","ovr","drop","locked","age")}
    if not all(k in out for k in required[out["type"]]): return None
    return out

def make_rows(record:dict)->list[dict]:
    if record["type"]=="IRF":
        return [{"ms":record["ms"],"scan":record["n"],"us":record["us"],
                 "channel":CHANNELS[i],"raw":record["d"][i],
                 "median3":record["med"][i],
                 "ema10":record["e10"][i],"ema25":record["e25"][i],
                 "hybrid":record["hy"][i]} for i in range(6)]
    if record["type"]=="IRB":
        return [{"ms":record["ms"],"scan":record["n"],"channel":CHANNELS[i],
                 "off":record["off"][i],"on":record["on"][i],
                 "difference":max(0,record["on"][i]-record["off"][i])}
                for i in range(6)]
    return []

def metrics(rows:list[dict],health:list[dict]|None=None)->dict:
    summary={"samples_per_channel":{},"channels":{}}
    health=health or []
    for ch in CHANNELS:
        subset=[r for r in rows if r["channel"]==ch]
        summary["samples_per_channel"][ch]=len(subset)
        if not subset: continue
        out={}
        for src in FILTER_MAP.values():
            vals=[float(r[src]) for r in subset]
            mean=stats.fmean(vals)
            std=stats.stdev(vals) if len(vals)>1 else 0.0
            med=stats.median(vals)
            mad=stats.median(abs(v-med) for v in vals)
            out[src]={"mean_adc":round(mean,3),"std_adc":round(std,3),
                      "median_adc":round(med,3),"mad_adc":round(mad,3),
                      "range_adc":round(max(vals)-min(vals),3)}
        summary["channels"][ch]=out
    if rows:
        times=sorted(set(int(r["ms"]) for r in rows))
        scans=sorted(set(int(r["scan"]) for r in rows))
        summary["log_rows"]=len(times)
        summary["duration_ms"]=times[-1]-times[0]
        summary["scan_first"]=scans[0]
        summary["scan_last"]=scans[-1]
        summary["observed_log_hz"]=(round((len(times)-1)*1000/(times[-1]-times[0]),2)
                                   if len(times)>1 and times[-1]>times[0] else 0)
    if health:
        h=health[-1]
        summary["health_final"]=h
        summary["valid_motor_locked"]=all(x["locked"]==1 for x in health)
        summary["telemetry_drops"]=h.get("drop",0)-health[0].get("drop",0)
        summary["dma_error_delta"]=h.get("dma_err",0)-health[0].get("dma_err",0)
        summary["scan_failed_delta"]=h.get("fail",0)-health[0].get("fail",0)
    return summary

def step_response(rows:list[dict])->dict:
    """Rough SINGLE step edge timing on the 40Hz exported stream.
    Threshold crossing is quantized to 25ms and is not a 1kHz delay
    measurement. Use one wall insertion/removal per capture.
    """
    out={}
    for ch in CHANNELS:
        part=sorted((r for r in rows if r["channel"]==ch),
                    key=lambda v:int(v["ms"]))
        if len(part)<15:continue
        n=max(4,len(part)//6)
        before=stats.median(float(r["raw"]) for r in part[:n])
        after=stats.median(float(r["raw"]) for r in part[-n:])
        span=after-before
        if abs(span)<max(15.0,0.10*max(before,1.0)):
            out[ch]={"status":"insufficient_change","delta_adc":round(span,2)}
            continue
        direction=1.0 if span>0 else -1.0
        t0=int(part[0]["ms"])
        channels={}
        for src in FILTER_MAP.values():
            low=stats.median(float(r[src]) for r in part[:n])
            hi=stats.median(float(r[src]) for r in part[-n:])
            if abs(hi-low)<10.0:
                channels[src]={"status":"no_response"};continue
            crossing={}
            for pct in (0.1,0.5,0.9):
                threshold=low+pct*(hi-low)
                t=next((int(r["ms"])-t0 for r in part
                        if (float(r[src])-threshold)*direction>=0),None)
                crossing[f"t{round(pct*100)}_ms"]=t
            if crossing["t10_ms"] is not None and crossing["t90_ms"] is not None:
                crossing["rise_10_90_ms"]=max(0,crossing["t90_ms"]-crossing["t10_ms"])
            channels[src]=crossing
        if channels.get("raw",{}).get("t50_ms") is not None:
            raw_mid=channels["raw"]["t50_ms"]
            for stage in channels.values():
                if "t50_ms" in stage and stage["t50_ms"] is not None:
                    stage["lag_vs_raw_t50_ms"]=stage["t50_ms"]-raw_mid
        out[ch]={"status":"single_step_estimate","delta_adc":round(span,2),
                 "direction":"rising" if span>0 else "falling",
                 "filters":channels}
    return out

def plot_six(rows:list[dict],dest:Path)->int:
    try:import matplotlib.pyplot as plt
    except ImportError:
        print("Plot skipped: install matplotlib for PNG output.",file=sys.stderr)
        return 0
    for ch in CHANNELS:
        r=[v for v in rows if v["channel"]==ch]
        if not r:continue
        t=[(int(x["ms"])-int(r[0]["ms"]))/1000.0 for x in r]
        fig,ax=plt.subplots(figsize=(10,4))
        for name,label in (("raw","ON-OFF raw"),("median3","Median3"),
                           ("ema10","EMA 0.10"),("ema25","EMA 0.25"),
                           ("hybrid","Median3 + EMA 0.25")):
            ax.plot(t,[x[name] for x in r],label=label,linewidth=1.2)
        ax.set(title=f"IR {ch} — sampled output (not 1kHz)",xlabel="Time (s)",
               ylabel="ADC difference (counts)")
        ax.legend(loc="best",fontsize=8)
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(dest/f"filter_{ch}.png",dpi=150)
        plt.close(fig)
    return 6

def write_csv(path:Path,rows:list[dict],headers)->None:
    with path.open("w",newline="",encoding="utf-8") as stream:
        wr=csv.DictWriter(stream,fieldnames=headers)
        wr.writeheader()
        wr.writerows(rows)

def main(argv=None)->int:
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument("--port",help="USART1 PA9 COM port, e.g. COM7")
    group.add_argument("--input",type=Path,help="Replay raw_uart.log")
    p.add_argument("--seconds",type=float,default=20)
    p.add_argument("--distance-mm",type=float,default=None,
                   help="Measured target distance; omit for dynamic/step")
    p.add_argument("--surface",default="unknown")
    p.add_argument("--light",default="room")
    p.add_argument("--scenario",default="static",
                   choices=["static","step","ambient","crosstalk","no-wall"])
    p.add_argument("--trial",type=int,default=1)
    p.add_argument("--notes",default="")
    p.add_argument("--plot",action="store_true")
    p.add_argument("--output",type=Path,default=None)
    args=p.parse_args(argv)
    if args.seconds<=0 or args.trial<1:p.error("seconds and trial must be positive")
    stamp=dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    dest=args.output or ROOT/"results"/f"ir_filter_{stamp}"
    if dest.exists():p.error(f"Output directory exists: {dest}")
    dest.mkdir(parents=True)
    raw_lines=[]
    frames=[]; off_on=[];health=[];malformed=0;last_print=0.0
    ser=None
    try:
        if args.port:
            try:import serial
            except ImportError:raise RuntimeError("pyserial required: pip install pyserial")
            ser=serial.Serial(args.port,115200,timeout=0.2)
            started=time.monotonic()
            print(f"Receiving {args.port}; motor output MUST be locked. CTRL+C ends capture.")
            while time.monotonic()-started<args.seconds:
                b=ser.readline()
                if not b:continue
                line=b.decode("ascii",errors="replace").strip()
                raw_lines.append(line)
                parsed=parse(line)
                if parsed is None:
                    if line.startswith(("IRF,","IRB,","IRH,")):malformed+=1
                    continue
                if parsed["type"]=="IRF":frames.extend(make_rows(parsed))
                elif parsed["type"]=="IRB":off_on.extend(make_rows(parsed))
                else:health.append(parsed)
                now=time.monotonic()
                if now-last_print>=1.0 and frames:
                    latest=frames[-6:]
                    preview="  ".join(f"{v['channel']}={v['raw']}/{v['ema25']}" for v in latest)
                    print(f"{latest[0]['ms']}ms {preview} (raw/EMA25)")
                    last_print=now
        else:
            for line in args.input.read_text(encoding="utf-8",errors="replace").splitlines():
                raw_lines.append(line)
                parsed=parse(line)
                if parsed is None:
                    if line.startswith(("IRF,","IRB,","IRH,")):malformed+=1
                    continue
                if parsed["type"]=="IRF":frames.extend(make_rows(parsed))
                elif parsed["type"]=="IRB":off_on.extend(make_rows(parsed))
                else:health.append(parsed)
    except KeyboardInterrupt:
        print("Capture interrupted: saving complete frames.")
    except (OSError,RuntimeError) as exc:
        print(f"ERROR: {exc}",file=sys.stderr)
        return 2
    finally:
        if ser:ser.close()
    (dest/"raw_uart.log").write_text("\n".join(raw_lines)+"\n",encoding="utf-8")
    write_csv(dest/"ir_filter_trace.csv",frames,CSV_FIELDS)
    write_csv(dest/"ir_on_off.csv",off_on,RAW_FIELDS)
    summary=metrics(frames,health)
    summary["metadata"]={key:getattr(args,key) for key in
        ("distance_mm","surface","light","scenario","trial","notes")}
    summary["malformed_ir_frames"]=malformed
    summary["raw_on_off_sample_count"]=len(off_on)//6
    if args.scenario=="step":
        summary["step_response_40hz_estimate"]=step_response(frames)
    # Explicit reject on drops, errors and missing coverage.
    summary["valid_capture"]=(len(frames)>=60 and len(health)>=2
         and not malformed and summary.get("valid_motor_locked",False)
         and summary.get("telemetry_drops",999)==0
         and summary.get("dma_error_delta",999)==0
         and summary.get("scan_failed_delta",999)==0)
    summary["output_directory"]=str(dest)
    if args.plot:summary["plots_generated"]=plot_six(frames,dest)
    (dest/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",
                                    encoding="utf-8")
    print(f"Captured IRF={len(frames)//6} IRB={len(off_on)//6} "
          f"IRH={len(health)} malformed={malformed}, valid={summary['valid_capture']}")
    for ch in CHANNELS:
        m=summary["channels"].get(ch)
        if m:
            print(f"{ch:>3}: raw std={m['raw']['std_adc']:7.2f} "
                  f"EMA10={m['ema10']['std_adc']:7.2f} "
                  f"EMA25={m['ema25']['std_adc']:7.2f} "
                  f"hybrid={m['hybrid']['std_adc']:7.2f}")
    print(f"Saved {dest}")
    return 0 if summary["valid_capture"] else 1

if __name__=="__main__":
    raise SystemExit(main())
