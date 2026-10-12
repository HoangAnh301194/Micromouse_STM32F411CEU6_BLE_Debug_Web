#!/usr/bin/env python3
"""Analyze LOAD telemetry for opt-in 3-stage motor-off control ISR benchmark."""
import argparse
import json
import pathlib
import re
import sys

def fields(line):
    return dict(re.findall(r"([a-zA-Z_][a-zA-Z0-9_]*)=([^,\s]+)", line))

def analyze(lines):
    boots = [i for i, line in enumerate(lines) if "[BOOT] Isolated MPU6050/I2C test" in line]
    if not boots:
        return {"status": "INCOMPLETE", "failures": ["No verified reset/boot marker"]}
    lines = lines[boots[-1]:]
    profile = [fields(x) for x in lines if x.startswith("LOAD_PROFILE,")]
    cal = [fields(x) for x in lines if x.startswith("CAL_DONE,")]
    # Stage 0 counters reset at CAL_DONE; omit pre-calibration snapshots.
    cal_pos = max((i for i, x in enumerate(lines) if x.startswith("CAL_DONE,")), default=-1)
    postcal = lines[cal_pos + 1:] if cal_pos >= 0 else []
    loads = [fields(x) for x in postcal if x.startswith("LOAD,")]
    ctrl_by_stage = {"0": [], "1": [], "2": []}
    stage = "0"
    for line in postcal:
        if line.startswith("LOAD_SWITCH,"):
            stage = fields(line).get("stage", stage)
        elif line.startswith("SYNC_CTRL,") and stage in ctrl_by_stage:
            ctrl_by_stage[stage].append(fields(line))
    warnings = []

    policies = [fields(x) for x in lines if x.startswith("SYNC_POLICY,")]
    failures = []
    if not profile or profile[-1].get("motor_off") != "1" or profile[-1].get("pwm_initialized") != "0":
        failures.append("Missing motor-off LOAD_PROFILE signature")
    if not policies or policies[-1].get("policy") != "B" or policies[-1].get("motor_off") != "1":
        failures.append("Policy B motor-off signature missing")
    if not cal or cal[-1].get("n") != "4000" or cal[-1].get("retries") != "0":
        failures.append("Invalid or absent calibration")
    if any("BOOT_FAIL" in line for line in lines):
        failures.append("Firmware BOOT_FAIL")
    records = {}
    for stage in range(3):
        items = [x for x in loads if x.get("stage") == str(stage)]
        if len(items) < 30:
            failures.append(f"Stage {stage}: fewer than 30 summary frames")
            continue
        try:
            ticks = [int(x["ticks"]) for x in items]
            maxima = [int(x["max_cycles"]) for x in items]
            over800 = [int(x["over800"]) for x in items]
            over1000 = [int(x["over1000"]) for x in items]
        except (KeyError, ValueError):
            failures.append(f"Stage {stage}: invalid counters")
            continue
        if any(b < a for a,b in zip(ticks, ticks[1:])):
            failures.append(f"Stage {stage}: ticks decreased")
        # SYNC_CTRL is cumulative; use first-to-last deltas within each stage.
        ctrl_items = ctrl_by_stage[str(stage)]
        sync_stats = {}
        if len(ctrl_items) >= 2:
            try:
                counts = {key: int(ctrl_items[-1][key]) - int(ctrl_items[0][key])
                          for key in ("ticks", "repeat", "age_gt1500")}
                if counts["ticks"] > 0 and counts["repeat"] >= 0 and counts["age_gt1500"] >= 0:
                    sync_stats = {
                        "sync_ticks_compared": counts["ticks"],
                        "gyro_reused_ticks": counts["repeat"],
                        "gyro_reused_pct": round(100 * counts["repeat"] / counts["ticks"], 5),
                        "gyro_age_gt1500_ticks": counts["age_gt1500"],
                    }
                else:
                    failures.append(f"Stage {stage}: invalid SYNC_CTRL deltas")
            except (KeyError, ValueError):
                failures.append(f"Stage {stage}: malformed SYNC_CTRL counters")
        else:
            warnings.append(f"Stage {stage}: insufficient SYNC_CTRL for per-stage freshness")
        records[str(stage)] = {
            "frames":len(items),
            "ticks":ticks[-1],
            "max_cycles":max(maxima),
            "max_us_nominal_96mhz":round(max(maxima)/96.0,3),
            "over800":over800[-1],
            "over1000":over1000[-1],
            **sync_stats,
        }
        if over1000[-1] or over800[-1]:
            failures.append(f"Stage {stage}: ISR budget violations")
    return {"status":"PASS_LOAD_DIAGNOSTIC" if not failures else "FAIL_OR_INCOMPLETE",
            "motor_off_only": True, "stages": records, "failures":failures,
            "warnings": warnings,
            "note":"Cycles/96MHz is nominal; physical timing uncertainty applies. Observed maxima are not formal WCET."}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--folder",required=True,type=pathlib.Path)
    args=parser.parse_args()
    raw=args.folder/"raw_serial.log"
    if not raw.is_file():
        parser.error("Missing raw_serial.log")
    lines=[x.strip().split("\t",1)[-1] for x in raw.read_text(encoding="utf-8",errors="replace").splitlines()]
    result=analyze(lines)
    target=args.folder/"load_stage_report.json"
    target.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2))
    return 0 if result["status"]=="PASS_LOAD_DIAGNOSTIC" else 1

if __name__=="__main__":
    sys.exit(main())
