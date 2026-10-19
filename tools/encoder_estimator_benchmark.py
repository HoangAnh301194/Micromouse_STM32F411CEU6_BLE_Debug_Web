#!/usr/bin/env python3
"""Deterministic host A/B comparison of speed estimators; NOT real sensor noise."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import statistics

COUNT_PER_REV=1430
DT=0.001
WINDOWS=(1,5,10,20)
DURATION_TICKS=30000

def reference_pps(tick):
    if tick < 3000: rpm=0
    elif tick < 12000: rpm=80
    elif tick < 20000: rpm=360
    else: rpm=120
    return rpm*COUNT_PER_REV/60

def test_model():
    assert reference_pps(0)==0
    assert reference_pps(3000)>0
    assert len(WINDOWS)==4
    assert round(1000/COUNT_PER_REV*60,2)==41.96

def evaluate():
    exact=0.0
    previous=0
    window_bins={n:[] for n in WINDOWS}
    last={n:0.0 for n in WINDOWS}
    ema_1ms=0.0
    errs={("EMA 1ms",1):[]}
    for window in WINDOWS:
        errs[("boxcar",window)]=[]
    examples=[]
    for tick in range(DURATION_TICKS):
        true=reference_pps(tick)
        exact+=true*DT
        total=int(exact)
        raw=total-previous
        previous=total
        ema_1ms+=0.3*(raw*1000-ema_1ms)
        if tick>3000 and tick<12000-250 or tick>12000+250 and tick<20000-250 or tick>20000+250:
            errs[("EMA 1ms",1)].append(ema_1ms-true)
        for window in WINDOWS:
            buf=window_bins[window]
            buf.append(raw)
            if len(buf)==window:
                last[window]=sum(buf)*1000/window
                buf.clear()
            if tick>3000+250 and tick<12000-250 or tick>12000+250 and tick<20000-250 or tick>20000+250:
                errs[("boxcar",window)].append(last[window]-true)
        if tick%100==0:
            examples.append({"ms":tick,"reference_pps":round(true,2),
                "ema_1ms":round(ema_1ms,2),
                "window_5ms":round(last[5],2),
                "window_10ms":round(last[10],2),
                "window_20ms":round(last[20],2)})
    metrics=[]
    for (method,window),errors in errs.items():
        mae=statistics.mean(map(abs,errors))
        rmse=(statistics.mean(x*x for x in errors))**0.5
        metrics.append({"method":method,"window_ms":window,
                        "steady_mae_pps":round(mae,3),
                        "steady_rmse_pps":round(rmse,3),
                        "measurement_window_delay_bound_ms":window})
    return metrics,examples

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir",type=Path)
    parser.add_argument("--self-test",action="store_true")
    args=parser.parse_args()
    test_model()
    if args.self_test:
        print("PASS model validity and unit conventions")
        return 0
    metrics,examples=evaluate()
    if args.outdir:
        args.outdir.mkdir(parents=True,exist_ok=True)
        for name,rows in [("estimator_metrics.csv",metrics),("estimator_samples.csv",examples)]:
            with (args.outdir/name).open("x",newline="",encoding="utf-8") as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        (args.outdir/"estimator_report.json").write_text(json.dumps({
            "kind":"SYNTHETIC_ONLY",
            "not_real_hardware_acceptance":True,
            "counts_per_rev_assumed":COUNT_PER_REV,
            "metrics":metrics,
            "caveat":"Step RPM profile, ideal encoder pulse quantization. Does not model EMI, missed edges, motor plant, counter IRQ jitter, latency or PID stability."
        },indent=2)+"\n",encoding="utf-8")
    for x in metrics:
        print(x)
    return 0

if __name__=="__main__":
    raise SystemExit(main())
