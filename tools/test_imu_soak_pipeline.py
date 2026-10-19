#!/usr/bin/env python3
"""Deterministic host checks; never opens a COM port or flashes firmware."""
from pathlib import Path
import importlib.util
import subprocess
import sys
base=Path(__file__).resolve().parent
p=subprocess.run([sys.executable,str(base/"imu_soak_pipeline.py"),"--help"],
                 capture_output=True,text=True,timeout=8,check=True)
assert "--motor-live" in p.stdout and "--high-pwm" in p.stdout and "--review" in p.stdout
assert mod.parse_line("IMDRV,t=200,stage=1,setL=800,appL=800,encL=123")["appL"]==800
spec=importlib.util.spec_from_file_location("soak",base/"imu_soak_pipeline.py")
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
assert mod.parse_line("IM6,t=100,s=42,y=-1234,n=-567,r=-890,z=23,a=500")["y"]==-1234
assert mod.parse_line("SOAK_DONE,ok=1,reason=0,ms=180000")["ok"]==1
for mode in ("STILL","SHADOW","LIVE"):
    records=[{"type":"SOAK_BEGIN","t":0,"iclk":400000,"high":0}]
    for hz in range(60*180):
        records.append({"type":"IM6","t":round(1000*hz/60),
                        "s":hz*16+1,"y":hz//6,"n":2*(hz//6),
                        "r":3*(hz//6),"z":0,"a":400})
    for second in range(1,181):
        ms=second*1000
        records.append({"type":"IM1","t":ms,"good":second*1000,
                        "stale":0,"err":0,"to":0,"late":0,"gap":0,
                        "drop":0,"rxerr":0,"qp":480})
        records.append({"type":"IMN","t":ms,"n":1000,"std":50,
                        "mean":1,"gmin":910,"gmax":1200,"eq":0,
                        "tc":28000+second*5,"tvalid":1,"tok":second+10,
                        "terr":0,"tto":0,"tage":100})
        records.append({"type":"IMIR","t":ms,"c":second*1000})
        records.append({"type":"IMPID","t":ms,"n":second*1000 if mode!="STILL" else 0,
                        "cmdL":200 if mode=="LIVE" else 0,"cmdR":200 if mode=="LIVE" else 0,
                        "encL":second*75 if mode=="LIVE" else 0,
                        "encR":second*74 if mode=="LIVE" else 0})
    if mode=="LIVE":
        for seq in range(1,180*4):
            records.append({"type":"IMDRV","t":seq*250,
                            "setL":200,"setR":200,"appL":200,"appR":200,
                            "encL":seq*22,"encR":seq*23})
    records.append({"type":"SOAK_DONE","ok":1,"reason":0,"ms":180000})
    out=mod.summarise(records,mode)
    assert out["passed_telemetry_checks"],(mode,out["checks"])
    assert out["captured_60hz_samples"]==10800
    assert out["temperature_c"]["n"]==180
    if mode=="LIVE":
        noencoder=[dict(r) for r in records]
        for x in noencoder:
            if x.get("type")=="IMDRV":x["encL"]=x["encR"]=0
        assert not mod.summarise(noencoder,"LIVE")["checks"]["live_encoder_pulses_observed"]


    no_temp=[dict(r) for r in records]
    for r in no_temp:
        if r.get("type")=="IMN":r["tvalid"]=0
    assert not mod.summarise(no_temp,mode)["checks"]["temperature_1hz_fresh_and_error_free"]
    high_q=[dict(r) for r in records]
    for r in high_q:
        if r.get("type")=="IM1" and r.get("t")==180000:r["qp"]=900
    assert not mod.summarise(high_q,mode)["checks"]["tx_queue_peak_below_75pct"]
    if mode=="LIVE":
        high=[dict(r) for r in records]
        high[0]["high"]=1
        high.extend([
          {"type":"IMDRV","t":10100,"setL":800,"setR":0,"appL":800,"appR":0,"encL":2000,"encR":300},
          {"type":"IMDRV","t":20100,"setL":900,"setR":900,"appL":900,"appR":900,"encL":3000,"encR":400},
          {"type":"IMDRV","t":25100,"setL":1000,"setR":1000,"appL":1000,"appR":1000,"encL":4000,"encR":500},
        ])
        assert mod.summarise(high,mode)["checks"]["high_pwm_80_90_100_applied"]
        high[-1]["appL"]=high[-1]["appR"]=0
        assert not mod.summarise(high,mode)["checks"]["high_pwm_80_90_100_applied"]
    damaged=[dict(r) for r in records]
    damaged[-1]["ok"]=0
    assert not mod.summarise(damaged,mode)["checks"]["completed_180s"]
    damaged=[r for r in records if r.get("type")!="IM6" or r.get("t",0)<2000]
    assert not mod.summarise(damaged,mode)["checks"]["samples_60hz_at_least_95pct"]
    err=[dict(r) for r in records]
    for r in err:
        if r.get("type")=="IM1" and r.get("t")==180000:r["err"]=1
    assert not mod.summarise(err,mode)["checks"]["no_i2c_errors"]
# Regression: do not silently wait for 60s on UART silence, and abort as
# soon as the MPU explicitly reports calibration failed. No hardware needed.
from collections import deque
import time
original_boot_timeout=mod.BOOT_SYS_M_TIMEOUT
try:
    mod.BOOT_SYS_M_TIMEOUT=.12
    no_uart=mod.Session.__new__(mod.Session)
    no_uart.mode="STILL"
    no_uart.recent_raw=deque(maxlen=8)
    no_uart.send=lambda msg:None
    no_uart.poll=lambda ping=False:(time.sleep(.005) or None)
    try:
        no_uart.run()
        raise AssertionError("missing SYS_M did not fail")
    except TimeoutError as exc:
        assert "no SYS_M" in str(exc)

    calibration_failed=mod.Session.__new__(mod.Session)
    calibration_failed.mode="STILL"
    calibration_failed.recent_raw=deque(maxlen=8)
    calibration_failed.send=lambda msg:None
    calibration_failed.poll=lambda ping=False:{"type":"SYS_M","cal":4,"n":0}
    try:
        calibration_failed.run()
        raise AssertionError("cal=4 did not fail")
    except RuntimeError as exc:
        assert "calibration FAILED" in str(exc)
finally:
    mod.BOOT_SYS_M_TIMEOUT=original_boot_timeout

print("PASS CLI, 60Hz 180s drift summaries, shadow/live gates and negative fault cases; hardware not tested")
