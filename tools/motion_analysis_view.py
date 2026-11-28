"""Motion analysis: aligned sampled telemetry, estimated relative pose (not ground truth)."""
import math
import csv
from pathlib import Path
import tkinter as tk
from tkinter import ttk

def save_odometry(path, rows):
    path=Path(path).with_suffix(".odom.csv")
    cols=("t","x_mm","y_mm","angle","drift_mm","odom_gap")
    with path.open("w",encoding="utf-8",newline="") as f:
        w=csv.DictWriter(f,fieldnames=cols,extrasaction="ignore")
        w.writeheader()
        w.writerows(r for r in rows if "x_mm" in r)

def metrics(rows):
    out=[]
    for side in ("L","R"):
        active=[r for r in rows if r.get("ref_"+side,0)!=0 and "speed_"+side in r]
        if active:
            rmse=math.sqrt(sum((r["ref_"+side]-r["speed_"+side])**2 for r in active)/len(active))
            sat=100*sum(abs(r.get("pwm_"+side,0))>=200 for r in active)/len(active)
            out.append(f"{side}: RMSE {rmse:.1f} PPS | saturation {sat:.1f}%")
    sync=[abs(r["sync"]) for r in rows if "sync" in r]
    if sync: out.append(f"Sync max {max(sync):.0f} counts")
    pose=[r for r in rows if "drift_mm" in r]
    if pose:
        r=pose[-1]
        out.append(f"Estimated drift {r['drift_mm']:.2f} mm" + (" (sample gap)" if r.get("odom_gap") else ""))
    if rows and "angle" in rows[-1] and "target" in rows[-1]:
        out.append(f"Angle error {rows[-1]['target']-rows[-1]['angle']:+.2f} deg")
    return " | ".join(out) or "Awaiting motion telemetry"

class MotionAnalysisView:
    def __init__(self,parent,data):
        self.data=data; self.frozen=None
        bar=ttk.Frame(parent); bar.pack(fill="x")
        self.pause=ttk.Button(bar,text="Pause view",command=self.toggle);self.pause.pack(side="left")
        self.window=tk.StringVar(value="All")
        ttk.Label(bar,text="Time window:").pack(side="left",padx=5)
        ttk.Combobox(bar,textvariable=self.window,values=("All","Last 1 s","Last 2 s","Last 5 s"),width=12,state="readonly").pack(side="left")
        self.note=tk.StringVar(value="Awaiting motion telemetry")
        ttk.Label(parent,textvariable=self.note,wraplength=1050).pack(fill="x",pady=4)
        self.cursor=tk.StringVar(value="Move pointer over plot to inspect time")
        ttk.Label(parent,textvariable=self.cursor).pack(fill="x")
        grid=ttk.Frame(parent);grid.pack(fill="both",expand=True)
        for i in range(2): grid.columnconfigure(i,weight=1,uniform="plot")
        for i in range(3): grid.rowconfigure(i,weight=1,uniform="plot")
        self.canvases=[]
        for i,title in enumerate(("Left motor — PPS","Right motor — PPS","PWM — permille","Encoder synchronization — counts","Angle — degrees","Estimated XY — mm")):
            f=ttk.LabelFrame(grid,text=title);f.grid(row=i//2,column=i%2,sticky="nsew",padx=3,pady=3)
            c=tk.Canvas(f,bg="#101923",highlightthickness=0,width=280,height=135)
            c.pack(fill="both",expand=True);self.canvases.append(c)
            if i<5: c.bind("<Motion>",lambda e,idx=i:self.inspect(e,idx))
        self.shown=[]
    def reset(self):
        self.frozen=None;self.pause.configure(text="Pause view")
    def toggle(self):
        self.frozen=list(self.data.rows) if self.frozen is None else None
        self.pause.configure(text="Resume view" if self.frozen is not None else "Pause view")
    def inspect(self,event,idx):
        if not self.shown: return
        c=self.canvases[idx]
        frac=max(0,min(1,(event.x-48)/max(1,c.winfo_width()-58)))
        t=self.shown[0]["t"]+frac*(self.shown[-1]["t"]-self.shown[0]["t"])
        r=min(self.shown,key=lambda r:abs(r["t"]-t))
        keys=("ref_L","speed_L","ref_R","speed_R","pwm_L","pwm_R","sync","angle","gyro","phase")
        self.cursor.set(f"t={r['t']:.3f}s | "+" | ".join(f"{k}={r[k]:.1f}" for k in keys if k in r))
    def draw(self):
        rows=self.frozen if self.frozen is not None else list(self.data.rows)
        self.note.set(metrics(rows))
        shown=rows
        if rows and self.window.get()!="All":
            span=float(self.window.get().split()[1])
            shown=[r for r in rows if r["t"]>=rows[-1]["t"]-span]
        self.shown=shown
        groups=(("ref_L","speed_L"),("ref_R","speed_R"),("pwm_L","pwm_R"),("sync",),("target","angle"))
        colors=("#ffcf65","#58c9ff","#71dfa1")
        for c,keys in zip(self.canvases,groups):
            c.delete("all");w,h=c.winfo_width(),c.winfo_height()
            if w<100 or h<60: continue
            vals=[r[k] for r in shown for k in keys if k in r]
            if not vals:
                c.create_text(w/2,h/2,text="No data for this mode",fill="#91a3b5");continue
            lo,hi=min(0,min(vals)),max(0,max(vals))
            if hi-lo<1: hi=lo+1
            t0,t1=shown[0]["t"],shown[-1]["t"];span=max(.001,t1-t0)
            x=lambda t:48+(w-58)*(t-t0)/span
            y=lambda v:h-23-(h-42)*(v-lo)/(hi-lo)
            for v in (lo,(lo+hi)/2,hi):
                c.create_line(48,y(v),w-10,y(v),fill="#405063")
                c.create_text(44,y(v),anchor="e",text=f"{v:.1f}",fill="#abc0d0")
            last_phase=None
            for r in shown:
                phase=r.get("phase")
                if phase is not None and phase!=last_phase:
                    c.create_line(x(r["t"]),19,x(r["t"]),h-23,fill="#506173",dash=(2,4))
                    c.create_text(x(r["t"])+2,9,anchor="w",text={1:"ACC",2:"CRUISE",3:"DEC",4:"SETTLE",5:"END"}.get(phase,""),fill="#abc0d0",font=("Consolas",8))
                last_phase=phase
            for i,k in enumerate(keys):
                coords=[]
                for r in shown:
                    if k in r: coords.extend((x(r["t"]),y(r[k])))
                    else:
                        if len(coords)>=4:c.create_line(*coords,fill=colors[i],width=2)
                        coords=[]
                if len(coords)>=4:c.create_line(*coords,fill=colors[i],width=2)
                c.create_text(50+i*105,h-9,anchor="w",text=k,fill=colors[i])
        c=self.canvases[-1];c.delete("all");w,h=c.winfo_width(),c.winfo_height()
        poses=[r for r in rows if "x_mm" in r]
        if not poses or w<100 or h<60:
            c.create_text(w/2,h/2,text="MPU + encoder required",fill="#91a3b5");return
        extent=max(2.0,max(max(abs(r["x_mm"]),abs(r["y_mm"])) for r in poses))
        scale=min(w-35,h-35)/(2*extent)
        px=lambda r:w/2+r["x_mm"]*scale
        py=lambda r:h/2-r["y_mm"]*scale
        c.create_line(10,h/2,w-10,h/2,fill="#405063")
        c.create_line(w/2,10,w/2,h-10,fill="#405063")
        coords=[v for r in poses for v in (px(r),py(r))]
        if len(coords)>=4:c.create_line(*coords,fill="#58c9ff",width=2)
        r=poses[-1];c.create_oval(px(r)-3,py(r)-3,px(r)+3,py(r)+3,fill="#ffcf65")
        c.create_text(8,8,anchor="nw",text=f"±{extent:.1f} mm | estimate, not measured",fill="#abc0d0")
