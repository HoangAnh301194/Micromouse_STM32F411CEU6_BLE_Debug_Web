#!/usr/bin/env python3
"""Supervised, repeated *real closed-loop* Encoder EMA comparison: 0.08/0.10/0.30.

Only the estimator alpha differs across builds. PID Kp=0.050, Ki=0.080,
Kd=0 and targets 800/1600/2400 encoder PPS are fixed. All trials are
motor-on capped at 30% in the actual compiled firmware. Never selects
or deploys production settings without reproducible physical results.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

from system_uart_pipeline import PhaseSession, evaluate, MOTOR_OFF_ENV

ROOT = Path(__file__).resolve().parent.parent
ENV = {
    8: "speed_pid_alpha_008_96m",
    10: "speed_pid_alpha_010_96m",
    30: "speed_pid_alpha_030_96m",
}
# Counterbalanced against heating, battery drift and trial order.
ORDER = ((30, 8, 10), (10, 30, 8), (8, 10, 30))
REQUIRED_ACTIVE = [(group*4+step, wheel)
                   for group in range(3) for step in (1, 2, 3)
                   for wheel in ((0,) if group == 0 else
                                 (1,) if group == 1 else (0, 1))]


def run_cmd(cmd, logfile):
    with logfile.open("w", encoding="utf-8") as f:
        result = subprocess.run([str(v) for v in cmd], cwd=ROOT,
                                stdout=f, stderr=subprocess.STDOUT,
                                check=False)
    if result.returncode:
        tail=logfile.read_text(encoding="utf-8",errors="replace").splitlines()[-45:]
        print("\n".join(tail),file=sys.stderr)
        raise RuntimeError("Command failed with rc=%d: %s (log %s)" %
                           (result.returncode, " ".join(map(str, cmd)), logfile))


def qgate(phase, stages):
    failures = [k for k, v in (phase.get("checks") or {}).items() if not v]
    if not phase.get("software_pass") or phase.get("stages") != stages or failures:
        raise RuntimeError("Hardware phase unqualified: stages=%s, checks=%s, faults=%s" %
                           (phase.get("stages"), failures, phase.get("faults")))
    if phase.get("faults") or phase.get("host_abort"):
        raise RuntimeError("Firmware safety abort: " + repr(phase.get("host_abort")))
    hb = phase.get("host_heartbeat") or {}
    if (hb.get("ping_sent", 0) < 150 or
        hb.get("ping_ack_received", 0) < hb.get("ping_sent", 0)-3 or
        hb.get("max_inter_ping_gap_ms", 99999) >= 500):
        raise RuntimeError("Host heartbeat unstable: "+repr(hb))
    if (phase.get("uart", {}).get("rx_err", -1) != 0 or
        phase.get("uart", {}).get("tx_drop", -1) != 0 or
        phase.get("uart", {}).get("dma_err", -1) != 0 or
        phase.get("timer", {}).get("late", -1) != 0 or
        phase.get("timer", {}).get("gap", -1) != 0):
        raise RuntimeError("UART / TIM11 health failure")
    return phase


def analyze_trial(rows):
    """Score real wheel-count windows, *not* the EMA loop-feedback signal.

    A 50-ms independent count delta is still from the same encoder,
    so it cannot prove true shaft RPM accuracy, slip or ground truth.
    """
    by=defaultdict(list)
    for row in rows:
        if row.get("type") != "SYS_PID":
            continue
        stage, group, step = (row.get("id"), row.get("group"), row.get("step"))
        if (not isinstance(stage, int) or not isinstance(group, int) or
            not isinstance(step, int) or group > 2 or step not in (1,2,3) or
            stage != group*4+step or row.get("target") != step*800):
            continue
        dt=row.get("dt", 0)
        if not isinstance(dt, int) or dt < 40 or dt > 110:
            continue
        for wheel in ((0,) if group == 0 else (1,) if group == 1 else (0,1)):
            field="L" if wheel == 0 else "R"
            actual=row.get("ref"+field)
            pwm=row.get("pwm"+field)
            if not isinstance(actual, int) or not isinstance(pwm, int):
                continue
            if pwm < 0 or pwm > 300:
                raise RuntimeError("Actual commanded PWM exceeded 30pct")
            by[(stage,wheel)].append((row["ms"], actual, pwm,
                                       row.get("ema"+field)))
    stats=[]
    if sorted(by) != sorted(REQUIRED_ACTIVE):
        raise RuntimeError("Missing active wheel/stage telemetry: "+
                           repr(sorted(set(REQUIRED_ACTIVE)-set(by))))
    for (stage,wheel),samples in sorted(by.items()):
        samples.sort()
        if len(samples) < 28:
            raise RuntimeError("Insufficient 20Hz samples on stage/wheel: "+
                               repr((stage,wheel,len(samples))))
        t0=samples[0][0]
        points=[((t-t0),v,p) for t,v,p,_ in samples]
        # Discard first 600 ms to avoid PWM ramp and 50-ms interval overlap.
        steady=[v for t,v,p in points if t >= 600]
        if len(steady)<16:
            raise RuntimeError("Insufficient steady tracking window")
        target=(stage%4)*800
        errors=[abs(v-target) for v in steady]
        overshoot=max(0.0, max(v for t,v,p in points if t>=100)/target-1.0)
        # Settling: all following windows remain within +/-10% of target.
        settling=None
        for i,(t,v,_) in enumerate(points):
            if t<150: continue
            future=[z for tt,z,pp in points if tt>=t]
            if len(future)>=8 and all(abs(z-target)<=.10*target for z in future):
                settling=t
                break
        rise=next((t for t,v,p in points if t>=50 and v>=.9*target),None)
        variation=(sum(abs(steady[i]-steady[i-1])
                       for i in range(1,len(steady)))/
                   (len(steady)-1)) if len(steady)>1 else 0.0
        clipped=sum(p>=299 for t,v,p in points if t>=600)
        stats.append({
            "stage":stage, "wheel":"L" if wheel==0 else "R",
            "target_pps":target,"sample_count":len(samples),
            "mean_actual_pps":round(statistics.mean(steady),2),
            "mean_absolute_error_pps":round(statistics.mean(errors),2),
            "normalized_mae":round(statistics.mean(errors)/target,5),
            "overshoot_fraction":round(overshoot,5),
            "settling_ms":settling,"rise_ms":rise,
            "normalized_variation":round(variation/target,5),
            "pwm_saturation_fraction":round(clipped/max(1,len(steady)),5),
            "closed_loop":True,
            "reference":"50ms encoder count delta, not external RPM reference",
        })
    return stats


def summarize(trials):
    """No promotion based solely on one run or on small score differences."""
    if len(trials) != 9: raise RuntimeError("Nine complete runs required")
    weights={"normalized_mae":.55, "overshoot_fraction":.15,
             "normalized_variation":.15, "settling_ms":.15}
    per_alpha={}
    for alpha in (8,10,30):
        t=[x for x in trials if x["alpha_pct"]==alpha]
        if len(t)!=3 or any(len(x["metrics"])!=len(REQUIRED_ACTIVE) for x in t):
            raise RuntimeError("Three complete per-alpha repeats required")
        records=[m for x in t for m in x["metrics"]]
        means={
            "normalized_mae":statistics.mean(m["normalized_mae"] for m in records),
            "overshoot_fraction":statistics.mean(m["overshoot_fraction"] for m in records),
            "normalized_variation":statistics.mean(m["normalized_variation"] for m in records),
            "settling_ms":statistics.mean(
                (m["settling_ms"] if m["settling_ms"] is not None else 3500)
                for m in records),
        }
        # A fixed, stated score; no post hoc adjustment of metric weights.
        score=(weights["normalized_mae"]*means["normalized_mae"]
               +weights["overshoot_fraction"]*means["overshoot_fraction"]
               +weights["normalized_variation"]*means["normalized_variation"]
               +weights["settling_ms"]*means["settling_ms"]/3500)
        per_alpha[str(alpha)]={"score":round(score,5),
                              "metrics":{k:round(v,5) for k,v in means.items()},
                              "repeat_scores": []}
        for t0 in t:
            m=t0["metrics"]
            sc=(.55*statistics.mean(z["normalized_mae"] for z in m)
                +.15*statistics.mean(z["overshoot_fraction"] for z in m)
                +.15*statistics.mean(z["normalized_variation"] for z in m)
                +.15*statistics.mean((z["settling_ms"] if z["settling_ms"] is not None else 3500)/3500 for z in m))
            per_alpha[str(alpha)]["repeat_scores"].append(round(sc,5))
    ranked=sorted(({"alpha":int(k)/100,**v} for k,v in per_alpha.items()),
                  key=lambda x:x["score"])
    best,runner=ranked[:2]
    # 5% minimum advantage + similar winner each repetition.
    winning_margin=(runner["score"]-best["score"])/max(runner["score"],1e-9)
    by_trial={}
    for repetition in range(3):
        per={int(k):v["repeat_scores"][repetition] for k,v in per_alpha.items()}
        by_trial[repetition]=min(per,key=per.get)
    consistent=sum(x==round(best["alpha"]*100) for x in by_trial.values())
    chosen = best["alpha"] if winning_margin>=.05 and consistent>=2 else None
    return {
        "status": "CLOSED_LOOP_CANDIDATE" if chosen is not None else "INCONCLUSIVE",
        "ranked":ranked,"margin_to_runner_up":round(winning_margin,4),
        "repeat_winners_pct":by_trial,
        "candidate_alpha":chosen,
        "production_alpha_unchanged":0.30,
        "pid_gains_unchanged":{"kp":.05,"ki":.08,"kd":0},
        "conditions":"At least 5% score advantage and winner in >=2/3 repeats",
        "limits":"Same encoder 50ms count reference, no external RPM/torque/slip; "
                 "candidate only, not automatic production deployment",
    }


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port",default="COM7")
    p.add_argument("--outdir",type=Path)
    p.add_argument("--offline-only",action="store_true")
    p.add_argument("--fixture-ready",action="store_true")
    p.add_argument("--independent-vm-cutoff",action="store_true")
    p.add_argument("--allow-30-percent",action="store_true")
    args=p.parse_args(argv)
    if args.offline_only and (args.fixture_ready or args.allow_30_percent
                               or args.independent_vm_cutoff):
        p.error("--offline-only forbids motor-on flags")
    if not args.offline_only:
        branch=subprocess.run(["git","branch","--show-current"],cwd=ROOT,
                              capture_output=True,text=True,check=False)
        tracked=subprocess.run(["git","status","--porcelain","--untracked-files=no"],
                               cwd=ROOT,capture_output=True,text=True,check=False)
        if (branch.returncode or branch.stdout.strip()!="giomuadongbac" or
            tracked.returncode or tracked.stdout.strip()):
            p.error("Physical Motor-ON blocked: must use clean tracked "
                    "giomuadongbac checkout; commit/reconcile local changes first")
    if not args.offline_only and not (args.fixture_ready and
                       args.independent_vm_cutoff and args.allow_30_percent):
        p.error("Live run requires --fixture-ready --independent-vm-cutoff "
                "--allow-30-percent")
    out=args.outdir or ROOT/"results"/("speed_pid_final_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True,exist_ok=False)
    record={"status":"STARTED","trials":[],"production_alpha_modified":False,
            "p0":"NOT_RUN","restored_motor_off":False}
    fault=False
    try:
        # Compile and test EVERY required image before connecting motor power.
        run_cmd([sys.executable,"tools/test_speed_pid_final.py"],out/"host_tests.log")
        for alpha,env in ENV.items():
            run_cmd([sys.executable,"-m","platformio","run","-e",env],
                    out/f"build_alpha_{alpha:03d}.log")
        if args.offline_only:
            record["status"]="OFFLINE_TESTS_PASS"
            return 0
        # Real motor-OFF prerequisite at this firmware revision.
        locked=out/"motor_off"
        run_cmd([sys.executable,"tools/system_uart_pipeline.py",
                 "--port",args.port,"--flash","--encoder-telemetry",
                 "--outdir",locked],out/"motor_off.log")
        offdata=json.loads((locked/"summary.json").read_text(encoding="utf-8"))
        off=qgate(next(x for x in offdata["results"] if x["mode"]=="OFF"),33)
        if (off["timer"].get("encmaxcy",99999)>=2000 or
            (off.get("encoder_shadow_queue") or {}).get("lost",1)!=0):
            raise RuntimeError("P0 motor-off encoder timing/FIFO gate failed")
        record["p0"]="PASS"
        # The explicit CLI flags are one-time authorization for the
        # complete experiment. NO interactive input between repeats.
        # Operator must remain present at the independent VM cutoff.
        print("LIVE RUN: 9 motor-enabled repeats, each capped at 30% PWM. "
              "Next run starts automatically after firmware profile check. "
              "Keep wheels elevated and stay at the physical VM cutoff.",
              flush=True)
        for repetition,block in enumerate(ORDER,1):
            for alpha in block:
                directory=out/f"repeat_{repetition}_alpha_{alpha:03d}"
                directory.mkdir()
                print(f"RUN {len(record['trials'])+1}/9: "
                      f"alpha={alpha/100:.2f}, repetition={repetition}; "
                      "flashing dedicated guarded diagnostic firmware",
                      flush=True)
                run_cmd([sys.executable,"-m","platformio","run",
                         "-e",ENV[alpha],"-t","upload"],directory/"flash.log")
                # PhaseSession verifies max PWM, stage count, exact alpha
                # and fixed PID gains BEFORE sending MODE ON or START.
                session=None
                try:
                    session=PhaseSession(args.port,115200,"ON",directory,
                                         limited=True, expected_alpha_pct=alpha)
                    session.run_phase()
                    score=evaluate(session.rows,"ON",limited=True)
                    score["host_heartbeat"]={
                        "ping_sent":session.ping_sent_count,
                        "ping_ack_received":sum(x["type"]=="CMD_PING" for x in session.rows),
                        "max_inter_ping_gap_ms":round(session.max_ping_gap_ms,2),
                        "max_ping_write_ms":round(session.max_ping_write_ms,2),
                    }
                    qgate(score,12)
                    metrics=analyze_trial(session.rows)
                    trial={"alpha_pct":alpha,"repeat":repetition,
                           "qualification":"PASS","metrics":metrics,
                           "health":score}
                    record["trials"].append(trial)
                    (directory/"metrics.json").write_text(
                        json.dumps(trial,indent=2)+"\n",encoding="utf-8")
                    session.close()
                    session=None
                except BaseException as exc:
                    if session is not None:
                        partial={"alpha_pct":alpha,"repeat":repetition,
                                 "reason":str(exc),
                                 "received_frames":len(session.rows),
                                 "host_ping_sent":session.ping_sent_count,
                                 "host_ping_ack":sum(
                                    row.get("type")=="CMD_PING" for row in session.rows),
                                 "last_fault":next((
                                    row for row in reversed(session.rows)
                                    if row.get("type")=="SYS_STEP" and
                                       row.get("state")==4),None)}
                        (directory/"fault_evidence.json").write_text(
                            json.dumps(partial,indent=2)+"\n",encoding="utf-8")
                    raise
                finally:
                    if session is not None:
                        session.close(emergency=True)
                # Firmware latches motor outputs OFF after DONE. The
                # operator is still responsible for independent VM cutoff,
                # particularly during reflashing/MCU resets.
                print(f"PASS trial: alpha={alpha/100:.2f}, "
                      f"repetition={repetition}; next run is automatic.",
                      flush=True)
                (out/"partial_summary.json").write_text(
                    json.dumps(record,indent=2)+"\n",encoding="utf-8")
        record["decision"]=summarize(record["trials"])
        record["status"]=record["decision"]["status"]
    except (RuntimeError, OSError, ValueError, KeyboardInterrupt) as exc:
        fault=True
        record["status"]="ABORTED_UNQUALIFIED"
        record["error"]=str(exc)
        print("HALT; independently disconnect VM:",exc,file=sys.stderr)
    finally:
        if not args.offline_only:
            try:
                # Restoration attempt, even after interrupted motor trials.
                # Requires the physically accessible independent VM cutoff.
                run_cmd([sys.executable,"-m","platformio","run","-e",MOTOR_OFF_ENV,
                         "-t","upload"],out/"restore_motor_off.log")
                record["restored_motor_off"]=True
            except (RuntimeError,OSError) as exc:
                fault=True
                record["status"]="RESTORE_FAILED_VM_MUST_STAY_OFF"
                record["restore_error"]=str(exc)
        (out/"summary.json").write_text(
            json.dumps(record,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
        print("Results: ",out,flush=True)
    return 2 if fault or record["status"]=="INCONCLUSIVE" else 0


if __name__=="__main__":
    raise SystemExit(main())
