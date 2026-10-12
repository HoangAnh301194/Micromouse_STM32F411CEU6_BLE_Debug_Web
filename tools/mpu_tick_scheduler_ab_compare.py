#!/usr/bin/env python3
"""Compare two matched, motor-OFF phase A/B runs: elapsed vs TIM11 tick.

Run each firmware profile for >=240s, reset ONCE after serial open.
Accepted comparison requires two independently attested raw traces.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import mpu_phase_ab_analyze as phase


def validate_input(report, expected):
    errors = []
    if report.get("status") != "PASS_PAIRED_TRACE_FOR_DIAGNOSIS":
        errors.append(f"{expected}: incomplete or invalid paired trace: {report.get('status')}")
    if report.get("scheduler") != expected:
        errors.append(f"{expected}: expected scheduler tag {expected}, got {report.get('scheduler')}")
    metrics = report.get("metrics", {})
    if metrics.get("paired_tim11_ticks", 0) < 120000:
        errors.append(f"{expected}: insufficient >=120000 postcal paired ticks")
    if report.get("integration_gate") == "FAIL":
        errors.append(f"{expected}: sensor/clock/scheduler integrity errors")
    if metrics.get("seqlock_contention_ticks", 0) > 0:
        errors.append(f"{expected}: I2C completion trace contention (investigate)")
    return errors


def compare(legacy, tick):
    errors = validate_input(legacy, "legacy_elapsed") + validate_input(tick, "tim11_tick")
    if errors:
        return {"status":"INCOMPLETE_OR_INVALID","failures":errors}
    a, b = legacy["metrics"], tick["metrics"]
    def ratio(m, n, d):
        return 100.0 * float(m.get(n, 0)) / max(1, int(m.get(d, 0)))
    legacy_reuse = ratio(a, "arm_B_snapshot_published_reused", "paired_tim11_ticks")
    tick_reuse = ratio(b, "arm_B_snapshot_published_reused", "paired_tim11_ticks")
    legacy_irq = ratio(a,"arm_A_irq_completion_reused","paired_tim11_ticks")
    tick_irq = ratio(b,"arm_A_irq_completion_reused","paired_tim11_ticks")
    legacy_long = ratio(a, "request_gap_gt1500_count", "request_intervals")
    tick_long = ratio(b, "request_gap_gt1500_count", "request_intervals")
    legacy_1050 = ratio(a, "request_gap_gt1050_count", "request_intervals")
    tick_1050 = ratio(b, "request_gap_gt1050_count", "request_intervals")
    differences = {
        "gyro_snapshot_reuse_pct": {"legacy":round(legacy_reuse,5),
                                     "tick":round(tick_reuse,5),
                                     "delta_pp":round(tick_reuse-legacy_reuse,5)},
        "gyro_irq_reuse_pct": {"legacy":round(legacy_irq,5),
                               "tick":round(tick_irq,5),
                               "delta_pp":round(tick_irq-legacy_irq,5)},
        "request_gap_gt1500_pct": {"legacy":round(legacy_long,5),
                                    "tick":round(tick_long,5),
                                    "delta_pp":round(tick_long-legacy_long,5)},
        "request_gap_gt1050_pct": {"legacy":round(legacy_1050,5),
                                    "tick":round(tick_1050,5),
                                    "delta_pp":round(tick_1050-legacy_1050,5)},
        "request_interval_max_us": {
            "legacy":a.get("request_interval_max_us_observed"),
            "tick":b.get("request_interval_max_us_observed")},
        "irq_to_publish_avg_us": {
            "legacy":a.get("irq_to_publish_avg_us"),
            "tick":b.get("irq_to_publish_avg_us")},
        "paired_ticks": {"legacy":a.get("paired_tim11_ticks"),
                         "tick":b.get("paired_tim11_ticks")}
    }
    # The 0.15% absolute figure was a conservative INVESTIGATION threshold,
    # not a validated universal bound. A single clean run only suggests trend.
    if tick_reuse > legacy_reuse*1.05 or tick_long > legacy_long*1.05:
        state="REGRESSED_INVESTIGATE"
    elif tick_reuse < legacy_reuse*0.85 and tick_long < legacy_long*0.85:
        state="IMPROVED_REQUIRES_REPLICATION"
    else:
        state="NO_CONCLUSIVE_IMPROVEMENT"
    return {
        "status":state,
        "old_acceptance_threshold_0_15pct_met": tick_reuse <= 0.15,
        "baseline_rc1_gyro_reuse_pct": 0.0441,
        "baseline_rc1_stage0_gyro_reuse_pct": 0.10508,
        "paired_trace_comparison":differences,
        "caveats":[
            "Motor OFF; no physical-motion/EMI/fault-injection qualification.",
            "Two separate firmware runs may have different initial MPU/TIM11 phase.",
            "Paired IRQ-vs-publish uses the same TIM11 ticks WITHIN each run.",
            "A second matched pair is needed to confirm a causal improvement.",
            "Do not change DLPF, PWM, I2C speed or Policy B during this A/B."
        ],
        "failures":[]
    }


def run_file(folder:Path):
    p=folder/"raw_serial.log"
    if not p.is_file(): raise SystemExit("Missing "+str(p))
    return phase.analyze(p.read_text(encoding="utf-8",errors="replace").splitlines())


def self_test():
    example={
        "status":"PASS_PAIRED_TRACE_FOR_DIAGNOSIS",
        "integration_gate":"REVIEW_FRESHNESS_REGRESSION",
        "scheduler":"legacy_elapsed",
        "metrics":{"paired_tim11_ticks":225000,
                   "arm_B_snapshot_published_reused":450,
                   "arm_A_irq_completion_reused":449,
                   "request_intervals":224770,
                   "request_gap_gt1500_count":228,
                   "request_gap_gt1050_count":453,
                   "request_interval_max_us_observed":2000,
                   "irq_to_publish_avg_us":15,
                   "seqlock_contention_ticks":0}}
    old=example
    new=json.loads(json.dumps(example))
    new["scheduler"]="tim11_tick"
    new["metrics"].update({
        "arm_B_snapshot_published_reused":60,
        "arm_A_irq_completion_reused":60,
        "request_gap_gt1500_count":2,
        "request_gap_gt1050_count":3,
        "request_interval_max_us_observed":1501})
    assert compare(old,new)["status"]=="IMPROVED_REQUIRES_REPLICATION"
    assert compare(old,old)["status"]=="INCOMPLETE_OR_INVALID"
    bad=json.loads(json.dumps(new))
    bad["metrics"]["arm_B_snapshot_published_reused"]=500
    assert compare(old,bad)["status"]=="REGRESSED_INVESTIGATE"
    bad=json.loads(json.dumps(new))
    bad["metrics"]["paired_tim11_ticks"]=2
    assert compare(old,bad)["status"]=="INCOMPLETE_OR_INVALID"
    print("SELF-TEST PASSED: independently attested modes, denominator, improvement/regression and safe rejection")
    print("This is a synthetic host-only test, not ARM compilation or hardware acceptance.")


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--legacy",type=Path)
    ap.add_argument("--tick",type=Path)
    ap.add_argument("--self-test",action="store_true")
    ap.add_argument("--output",type=Path)
    args=ap.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.legacy or not args.tick:
        ap.error("Provide BOTH --legacy and --tick folders")
    if args.legacy.resolve()==args.tick.resolve():
        ap.error("Use two distinct session folders")
    result=compare(run_file(args.legacy),run_file(args.tick))
    print(json.dumps(result,indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
        print("Saved:",args.output)
    if result["status"]=="INCOMPLETE_OR_INVALID": raise SystemExit(1)


if __name__=="__main__": main()
