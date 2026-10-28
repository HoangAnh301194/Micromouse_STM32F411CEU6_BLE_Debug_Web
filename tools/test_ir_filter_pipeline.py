#!/usr/bin/env python3
"""Synthetic standalone tests for labeled IR filter stream and metrics."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tools"))
from ir_filter_pipeline import parse,make_rows,metrics,step_response,isolated_spikes

def check():
    diag=(ROOT/"src/ir_filter_diag.c").read_text(encoding="utf-8")
    assert diag.count("LogHello();")==2
    p="|".join(["800","801","802","803","804","805"])
    f=f"IRF,ms=1000,n=123,us=1000000,d={p},med={p},e10={p},e25={p},hy={p}"
    item=parse(f)
    assert item and item["type"]=="IRF"
    assert len(make_rows(item))==6
    assert parse(f.replace("e25="+p,"e25=abc")) is None
    assert parse(f.replace("e25="+p,"e25=800|801")) is None
    h=parse("IRH,ms=1000,n=1,scans=1000,fail=0,dma_err=0,dma_to=0,ovr=0,drop=0,locked=1,age=0")
    assert h and h["locked"]==1
    assert parse("IRH,ms=1000,n=1,fail=0,dma_err=0,dma_to=0,ovr=0,drop=0,locked=1,age=0") is None
    b=parse("IRB,ms=1000,n=1,off=10|10|10|10|10|10,on=20|20|20|20|20|20")
    assert b and make_rows(b)[0]["difference"]==10
    with tempfile.TemporaryDirectory(prefix="irfilter-") as td:
        root=Path(td)
        src=root/"input.log"
        out=root/"out"
        lines=["IR_FILTER,HELLO,ir_filter_v1_motor_locked,channels=L90|L45|L0|R0|R45|R90,log_hz=40,unit=adc"]
        for t in range(0,3100,25):
            lines.append(f.replace("ms=1000",f"ms={t}").replace("n=123",f"n={t+1}"))
            if t%200==0:lines.append(
                "IRB,ms="+str(t)+",n="+str(t+1)+
                ",off=10|10|10|10|10|10,on=20|20|20|20|20|20")
            if t%1000==0:
                lines.append(
                    "IR_FILTER,HELLO,ir_filter_v1_motor_locked,"
                    "channels=L90|L45|L0|R0|R45|R90,log_hz=40,unit=adc")
                lines.append(
                    f"IRH,ms={t},n={t+1},scans={t+1},fail=0,"
                    "dma_err=0,dma_to=0,ovr=0,drop=0,locked=1,age=0")
        src.write_text("\n".join(lines)+"\n",encoding="utf-8")
        subprocess.run([sys.executable,str(ROOT/"tools/ir_filter_pipeline.py"),
                        "--input",str(src),"--distance-mm","20",
                        "--surface","white_matte","--trial","2",
                        "--output",str(out)],
                       cwd=ROOT,check=True,timeout=30)
        summary=json.loads((out/"summary.json").read_text())
        assert summary["valid_capture"]
        assert summary["samples_per_channel"]["L90"]==124
        assert summary["metadata"]["distance_mm"]==20.0
        assert summary["metadata"]["trial"]==2
        assert summary["channels"]["L90"]["raw"]["std_adc"]==0.0
        assert summary["channels"]["L90"]["raw"]["noise_reduction_pct"] is None
        assert summary["irf_coverage_pct"]>=95.0
        assert summary["irb_coverage_pct"]>=95.0
        assert summary["scan_rate_hz"]==1000.0
        with (out/"ir_filter_trace.csv").open(newline="") as fh:
            assert len(list(csv.DictReader(fh)))==124*6
        with (out/"ir_on_off.csv").open(newline="") as fh:
            assert len(list(csv.DictReader(fh)))>=15*6
        bad_src=root/"missing_hello.log"
        bad_src.write_text("\n".join(
            line for line in lines if not line.startswith("IR_FILTER,HELLO,")
        )+"\n",encoding="utf-8")
        bad_out=root/"bad"
        result=subprocess.run(
            [sys.executable,str(ROOT/"tools/ir_filter_pipeline.py"),
             "--input",str(bad_src),"--output",str(bad_out)],
            cwd=ROOT,check=False,timeout=30)
        assert result.returncode==1
        assert not json.loads((bad_out/"summary.json").read_text())["valid_capture"]
    # One known 40Hz rising edge; EMA trails raw. Do not claim ms-
    # resolution below the 25ms transmit decimation interval.
    step=[]
    for k in range(50):
        raw=100 if k<20 else 600
        ema=100 if k<20 else round(600-500*(0.5**(k-19)))
        for ch in ("L90","L45","L0","R0","R45","R90"):
            step.append({"ms":k*25,"scan":k*25+1,"channel":ch,
                         "raw":raw,"median3":raw,"ema10":ema,
                         "ema25":ema,"hybrid":ema})
    sr=step_response(step)
    assert sr["L90"]["status"]=="single_step_estimate"
    assert sr["L90"]["filters"]["raw"]["t50_ms"]==500
    assert sr["L90"]["filters"]["ema25"]["lag_vs_raw_t50_ms"]>=0
    assert isolated_spikes([100,100,100,900,100,100,100])==1
    print("IR_FILTER_PIPELINE_SYNTHETIC_PASS")
if __name__=="__main__":check()
