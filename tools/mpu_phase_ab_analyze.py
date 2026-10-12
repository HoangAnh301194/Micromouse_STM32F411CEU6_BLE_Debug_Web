#!/usr/bin/env python3
"""Paired MPU6050 phase trace: I2C IRQ complete vs published snapshot at TIM11.

Both arms are measured on the SAME TIM11 ticks. This prevents confounding
by different UART/ISR loads; separate historical RC1 data remain only context.
NO motor actions, no new sampling/control algorithm.
"""
from __future__ import annotations
import argparse
import datetime as dt
import json
from pathlib import Path
import time
import mpu_integration_regression as integration


def record(line):
    return integration.decode(line)


def analyze(lines, min_phases=30, min_valid_mint=150):
    old = integration.analyze(lines, min_samples=min_valid_mint)
    issues = list(old.get("failures", []))
    warnings = list(old.get("warnings", []))
    b = [i for i, line in enumerate(lines) if "[MINT_BOOT]" in line]
    if len(b) != 1:
        issues.append("Exactly one MINT_BOOT required, including pre-reset bytes")
    rows = [record(s) for s in lines[b[0]:]] if b else []
    def tag(k):
        return [v for name,v in rows if name == k]

    headers = tag("PHASE_PROFILE")
    if len(headers) != 1 or any(headers[0].get(k) != v for k,v in {
        "mode":"paired_irq_vs_publish","version":"1","motor_off":"1",
        "policy":"B","diag":"0","period_us":"1000",
        "trace_interval_s":"5","uart_in_isr":"0"}.items()):
        issues.append("PHASE_PROFILE missing or unexpected (wrong firmware)")
    req = tag("PHASE_REQ")
    latency = tag("PHASE_LAT")
    pair = tag("PHASE_PAIR")
    if min(map(len,(req,latency,pair)),default=0) < min_phases:
        issues.append(f"Insufficient complete phase reports (<{min_phases})")
    req_by, lat_by, pair_by = {}, {}, {}
    def numeric(v,k):
        try: return int(v[k])
        except (ValueError, KeyError, TypeError):
            issues.append("Missing/bad integer " + k)
            return 0
    for source,dst,name in ((req,req_by,"REQ"),(latency,lat_by,"LAT"),(pair,pair_by,"PAIR")):
        for r in source:
            n=numeric(r,"seq")
            if n in dst: issues.append("Duplicate "+name+" seq "+str(n))
            dst[n]=r
    same=set(req_by)&set(lat_by)&set(pair_by)
    if len(same)!=len(req_by) or len(same)!=len(lat_by) or len(same)!=len(pair_by):
        issues.append("Unmatched phase report groups")
    phase_seqs=sorted(same)
    if any(y-x!=5 for x,y in zip(phase_seqs,phase_seqs[1:])):
        issues.append("Missing / out-of-order 5-second phase records")
    if len(phase_seqs)<2:
        return {"status":"INCOMPLETE","failures":sorted(set(issues)),
                "warnings":warnings,"integration_gate":old.get("status")}
    first,last=phase_seqs[0],phase_seqs[-1]
    r0,r1=req_by[first],req_by[last]
    l0,l1=lat_by[first],lat_by[last]
    p0,p1=pair_by[first],pair_by[last]
    for d0,d1,fields in (
        (r0,r1,("n","intervals","gt1050","gt1500")),
        (l0,l1,("n",)),
        (p0,p1,("n","irq_reuse","pub_reuse","irq_ahead","contention"))
    ):
        for k in fields:
            if numeric(d1,k)<numeric(d0,k):
                issues.append("Counter decreased: "+k)
    comparable=numeric(p1,"n")-numeric(p0,"n")
    ir=numeric(p1,"irq_reuse")-numeric(p0,"irq_reuse")
    pub=numeric(p1,"pub_reuse")-numeric(p0,"pub_reuse")
    ahead=numeric(p1,"irq_ahead")-numeric(p0,"irq_ahead")
    contention=numeric(p1,"contention")-numeric(p0,"contention")
    req_delta=numeric(r1,"n")-numeric(r0,"n")
    request_intervals=numeric(r1,"intervals")-numeric(r0,"intervals")
    over1050=numeric(r1,"gt1050")-numeric(r0,"gt1050")
    over1500=numeric(r1,"gt1500")-numeric(r0,"gt1500")
    processed=numeric(l1,"n")-numeric(l0,"n")
    if comparable<30000 or processed<30000:
        issues.append("Insufficient postcal IRQ/publication paired ticks")
    if ir>pub:
        warnings.append("IRQ reuse exceeds published reuse; examine timestamp/trace windows")
    if contention:
        warnings.append("IRQ trace lock was preempted by TIM11 on some ticks")
    if over1050:
        warnings.append("Some request start intervals exceeded 1050us")
    if numeric(l1,"irq2main_max")>=1000:
        warnings.append("At least one gyro completion waited >=1ms before main observation")
    if numeric(l1,"main2pub_max")>=1000:
        warnings.append("At least one main processing->publish delay >=1ms")

    # The three timestamps are from the SAME accepted sample (last emitted
    # completed sample). Must be ordered without wrap ambiguity at capture
    # end; 32-bit TIM5 wraps at ~71 minutes, 240s is safely below wrap.
    req_t=numeric(r1,"t_request")
    irq_t=numeric(r1,"t_complete")
    seen_t=numeric(r1,"t_main")
    pub_t=numeric(r1,"t_publish")
    last_timestamps=[req_t,irq_t,seen_t,pub_t]
    if last_timestamps!=sorted(last_timestamps):
        issues.append("Last-sample timestamps out of request/IRQ/main/publish order")

    metrics={
        "paired_tim11_ticks":comparable,
        "arm_A_irq_completion_reused":ir,
        "arm_B_snapshot_published_reused":pub,
        "arm_A_irq_completion_reuse_pct":round(100*ir/max(comparable,1),5),
        "arm_B_publication_reuse_pct":round(100*pub/max(comparable,1),5),
        "extra_publication_reuse_ticks":pub-ir,
        "irq_complete_but_not_published_ticks":ahead,
        "seqlock_contention_ticks":contention,
        "sampling_requests":req_delta,
        "request_intervals":request_intervals,
        "request_gap_gt1050_count":over1050,
        "request_gap_gt1500_count":over1500,
        "request_interval_avg_us_reported":numeric(r1,"avg"),
        "request_interval_max_us_observed":numeric(r1,"max"),
        "request_to_irq_avg_us":numeric(l1,"r2irq_avg"),
        "request_to_irq_max_us":numeric(l1,"r2irq_max"),
        "irq_to_main_avg_us":numeric(l1,"irq2main_avg"),
        "irq_to_main_max_us":numeric(l1,"irq2main_max"),
        "main_to_publish_avg_us":numeric(l1,"main2pub_avg"),
        "main_to_publish_max_us":numeric(l1,"main2pub_max"),
        "irq_to_publish_avg_us":numeric(l1,"irq2pub_avg"),
        "irq_to_publish_max_us":numeric(l1,"irq2pub_max"),
        "last_sample_timestamps_us":{
            "request":req_t,"irq_complete":irq_t,
            "main_observed":seen_t,"snapshot_publish":pub_t
        },
        "window_phase_records":len(phase_seqs)
    }
    if pub-ir>0 and ir>=0:
        interpretation=("Publication makes additional TIM11 ticks reuse a gyro "
            "sample whose I2C IRQ had already completed; measure main wake/filter path.")
    elif pub==ir:
        interpretation=("Both markers reuse equally in matched TIM11 ticks; "
            "investigate request schedule/actual I2C completion phase.")
    else:
        interpretation=("Unexpected counter relationship; inspect traces and "
            "timestamp alignment before attributing root cause.")
    # A paired timing trace is a DIAGNOSTIC result, not a claim that
    # freshness regression or real-world motor performance is resolved.
    status=("FAIL_TRACE" if issues else "PASS_PAIRED_TRACE_FOR_DIAGNOSIS")
    return {"status":status,"failures":sorted(set(issues)),
            "warnings":sorted(set(warnings)),
            "integration_gate":old.get("status"),
            "paired_interpretation":interpretation,
            "metrics":metrics,
            "scope":"motor-OFF, no sampling policy changes, two marker arms on identical ISR ticks"}


def self_test():
    def pack(t,**v): return t+","+",".join(f"{k}={x}" for k,x in v.items())
    lines=["[MINT_BOOT] MPU production sampling Policy B, motors disabled",
        pack("PHASE_PROFILE",mode="paired_irq_vs_publish",version=1,
             motor_off=1,policy="B",diag=0,period_us=1000,
             trace_interval_s=5,uart_in_isr=0),
        pack("MINT_PROFILE",**integration.REQUIRED_PROFILE),
        pack("MINT_I2C",whoami="0x68",selected=0,scl_nominal=400000,init_status=0),
        pack("MINT_CAL_DONE",n=4000,retries=0,blocks=8,bias_mdps=0)]
    for i in range(200):
        lines.append(pack("MINT",seq=i+1,cal=1,valid=1,ticks=1000*i,
            reused=i,aged1500=i,age_max=800,missing=0,
            sample_seq=999*i+1,good=1000*i+1,fresh=1000*i+1,
            stale=0,notready=0,i2c_err=0,timeout=0,busy=0,
            timing_late=0,timing_gap=0,timing_max_cycles=500,tx_drop=0))
        if (i+1)%5==0:
            n=i+1
            lines.append(pack("PHASE_REQ",seq=n,n=1000*n,intervals=1000*n-1,
                avg=1000,min=999,max=1001,gt1050=0,gt1500=0,
                t_request=1000*n,t_complete=1000*n+220,
                t_main=1000*n+235,t_publish=1000*n+254))
            lines.append(pack("PHASE_LAT",seq=n,n=1000*n,
                r2irq_avg=220,r2irq_max=280,
                irq2main_avg=15,irq2main_max=30,
                main2pub_avg=19,main2pub_max=26,
                irq2pub_avg=34,irq2pub_max=45))
            lines.append(pack("PHASE_PAIR",seq=n,n=1000*n,
                irq_reuse=n//5,pub_reuse=n//5+2*n//5,
                irq_ahead=n//5,contention=0,irq_age_max=810,
                main_age_max=790,irq_seq=9000+1000*n,
                pub_seq=1000*n))
    ok=analyze(lines)
    assert ok["status"]=="PASS_PAIRED_TRACE_FOR_DIAGNOSIS",ok
    assert ok["metrics"]["extra_publication_reuse_ticks"]>0
    assert "Publication makes" in ok["paired_interpretation"]
    bad=[x for x in lines if not x.startswith("PHASE_PAIR,seq=100")]
    assert analyze(bad)["status"]=="FAIL_TRACE"
    bad=[x.replace("motor_off=1,policy=B","motor_off=0,policy=B") for x in lines]
    assert analyze(bad)["status"]=="FAIL_TRACE"
    bad=[x.replace("t_publish=200254","t_publish=200100") for x in lines]
    assert analyze(bad)["status"]=="FAIL_TRACE"
    bad=lines+[lines[0]]
    assert analyze(bad)["status"]=="FAIL_TRACE"
    print("PHASE A/B SELF-TEST PASSED: paired event trace, completeness, safety profile and malformed rejects.")
    print("Synthetic host data only; no firmware compilation or STM32 execution.")


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    modes=ap.add_mutually_exclusive_group(required=True)
    modes.add_argument("--self-test",action="store_true")
    modes.add_argument("--folder",type=Path)
    modes.add_argument("--port",help="USB TTL serial port, e.g. COM6")
    ap.add_argument("--baud",type=int,default=115200)
    ap.add_argument("--seconds",type=float,default=240.0)
    args=ap.parse_args()
    if args.self_test:
        self_test()
        return
    if args.port:
        if args.seconds<180: ap.error("Collect at least 180s for a paired phase trace")
        try: import serial
        except ImportError: ap.error("Install pyserial: python -m pip install pyserial")
        folder=Path("mpu_phase_ab_logs")/("session_"+dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
        folder.mkdir(parents=True,exist_ok=True)
        raw=folder/"raw_serial.log"
        print("Open",args.port,"at",args.baud,"baud. Press STM32 RESET once after opening.")
        print("Motors DISCONNECTED. Capture",args.seconds,"seconds into",folder)
        try:
            with serial.Serial(args.port,args.baud,timeout=0.25) as ser:
                ser.reset_input_buffer()
                t0=time.perf_counter()
                with raw.open("w",encoding="utf-8") as f:
                    while time.perf_counter()-t0<args.seconds:
                        val=ser.readline()
                        if not val: continue
                        line=val.decode("utf-8",errors="replace").strip()
                        f.write(f"{time.perf_counter()-t0:.9f}\t{line}\n")
                        if line.startswith(("PHASE_PROFILE","MINT_BOOT_FAIL","MINT_CAL_FAIL")):
                            print(line)
        except KeyboardInterrupt: print("Interrupted; analyzing partial capture.")
        except serial.SerialException as exc: raise SystemExit("Serial error: "+str(exc))
    else:
        folder=args.folder
        raw=folder/"raw_serial.log"
    if not raw.is_file(): raise SystemExit("Missing "+str(raw))
    report=analyze(raw.read_text(encoding="utf-8",errors="replace").splitlines())
    (folder/"phase_ab_report.json").write_text(
        json.dumps(report,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(report,indent=2))
    print("Results",folder)
    if report["status"]!="PASS_PAIRED_TRACE_FOR_DIAGNOSIS": raise SystemExit(1)


if __name__=="__main__": main()
