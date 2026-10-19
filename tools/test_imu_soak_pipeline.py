#!/usr/bin/env python3
"""Deterministic host checks; never opens a COM port or flashes firmware."""
from pathlib import Path
import importlib.util
import subprocess
import sys
base=Path(__file__).resolve().parent
p=subprocess.run([sys.executable,str(base/"imu_soak_pipeline.py"),"--help"],
                 capture_output=True,text=True,timeout=8,check=True)
assert "--motor-live" in p.stdout and "--review" in p.stdout
spec=importlib.util.spec_from_file_location("soak",base/"imu_soak_pipeline.py")
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
assert mod.parse_line("IM6,t=100,s=42,y=-1234,n=-567,r=-890,z=23,a=500")["y"]==-1234
assert mod.parse_line("SOAK_DONE,ok=1,reason=0,ms=180000")["ok"]==1
for mode in ("STILL","SHADOW","LIVE"):
    records=[{"type":"SOAK_BEGIN","t":0,"iclk":400000}]
    for hz in range(60*180):
        records.append({"type":"IM6","t":round(1000*hz/60),
                        "s":hz*16+1,"y":hz//6,"n":2*(hz//6),
                        "r":3*(hz//6),"z":0,"a":400})
    for second in range(1,181):
        ms=second*1000
        records.append({"type":"IM1","t":ms,"good":second*1000,
                        "stale":0,"err":0,"to":0,"late":0,"gap":0,
                        "drop":0,"rxerr":0})
        records.append({"type":"IMN","t":ms,"n":1000,"std":50,
                        "mean":1,"gmin":910,"gmax":1200,"eq":0})
        records.append({"type":"IMIR","t":ms,"c":second*1000})
        records.append({"type":"IMPID","t":ms,"n":second*1000 if mode!="STILL" else 0,
                        "cmdL":200 if mode=="LIVE" else 0,"cmdR":200 if mode=="LIVE" else 0,
                        "encL":second*75 if mode=="LIVE" else 0,
                        "encR":second*74 if mode=="LIVE" else 0})
    records.append({"type":"SOAK_DONE","ok":1,"reason":0,"ms":180000})
    out=mod.summarise(records,mode)
    assert out["passed_telemetry_checks"],(mode,out["checks"])
    assert out["captured_60hz_samples"]==10800
    if mode=="LIVE":
        noencoder=[dict(r) for r in records]
        for x in noencoder:
            if x.get("type")=="IMPID":x["encL"]=x["encR"]=0
        assert not mod.summarise(noencoder,"LIVE")["checks"]["live_encoder_pulses_observed"]

    damaged=[dict(r) for r in records]
    damaged[-1]["ok"]=0
    assert not mod.summarise(damaged,mode)["checks"]["completed_180s"]
    damaged=[r for r in records if r.get("type")!="IM6" or r.get("t",0)<2000]
    assert not mod.summarise(damaged,mode)["checks"]["samples_60hz_at_least_95pct"]
    err=[dict(r) for r in records]
    for r in err:
        if r.get("type")=="IM1" and r.get("t")==180000:r["err"]=1
    assert not mod.summarise(err,mode)["checks"]["no_i2c_errors"]
print("PASS CLI, 60Hz 180s drift summaries, shadow/live gates and negative fault cases; hardware not tested")
