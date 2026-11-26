"""Motor-locked IR calibration bench tool for v4. No actuator commands."""
import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

CHANNELS = ("L0", "L45", "L90", "R0", "R45", "R90")
FRONT = ("L0", "R0")  # pinout and runtime front threshold mapping

def signal(s):
    s = float(s)
    if not math.isfinite(s) or not 1 < s < 4090:
        raise ValueError("invalid or saturated IR")
    return s

def relative(s, ref):
    return 100 * math.log(signal(ref)) / math.log(signal(s))

def fit(d1, s1, d2, s2):
    s1, s2 = signal(s1), signal(s2)
    if not all(math.isfinite(d) and d > 0 for d in (d1, d2)) or (d1-d2)*(s1-s2) >= 0:
        raise ValueError("invalid distance points or nonmonotonic signal")
    a = (d1-d2)/(1/math.log(s1)-1/math.log(s2))
    b = a/math.log(s1)-d1
    return {"a": a, "b": b, "low": min(s1,s2), "high": max(s1,s2),
            "dmin": min(d1,d2), "dmax": max(d1,d2)}

def distance(s, model):
    s = signal(s)
    if not model["low"] <= s <= model["high"]:
        raise ValueError("outside calibrated range")
    return model["a"]/math.log(s)-model["b"]

def parse(line):
    if not line.startswith("IR,"):
        return None
    try:
        fields = dict(part.split("=",1) for part in line.strip().split(",")[1:])
        if int(fields["valid"]) != 1 or int(fields["age_us"]) > 5000:
            return None
        row = {k: int(fields[k]) for k in CHANNELS}
        if not all(0 <= v <= 4095 for v in row.values()):
            return None
        return {"scan": int(fields["scan"]), "ms": int(fields["ms"]), **row}
    except (KeyError, ValueError):
        return None

def capture(ser, seconds, file):
    ser.reset_input_buffer()
    rows, last = [], None
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        row = parse(ser.readline().decode("ascii", errors="ignore"))
        if row and row["scan"] != last:
            rows.append(row)
            last = row["scan"]
    if len(rows) < 10:
        raise ValueError("insufficient fresh IR frames; select IR Test and enable stream")
    with file.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["scan","ms",*CHANNELS])
        writer.writeheader()
        writer.writerows(rows)
    return rows

def stats(rows):
    result = {}
    for k in CHANNELS:
        v = sorted(r[k] for r in rows)
        result[k] = {"mean": statistics.mean(v), "std": statistics.stdev(v),
                     "p10": v[int(.1*(len(v)-1))], "p90": v[int(.9*(len(v)-1))],
                     "min": v[0], "max": v[-1], "n": len(v)}
    return result

def self_test():
    assert abs(relative(2000,2000)-100)<1e-9
    m = fit(30,2000,70,500)
    assert abs(distance(2000,m)-30)<1e-8
    assert abs(distance(500,m)-70)<1e-8
    for s in (0,1,4095,float("nan"),400):
        try:
            distance(s,m)
        except ValueError:
            continue
        raise AssertionError("invalid signal accepted")
    assert parse("IR,valid=0") is None
    print("SELF TEST PASS (mathematics only)")

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port")
    p.add_argument("--self-test",action="store_true")
    p.add_argument("--seconds",type=float,default=4)
    p.add_argument("--root",type=Path,default=Path("log_car_ir/hybrid_calibration"))
    args = p.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.port or args.seconds < 2:
        p.error("provide --port and --seconds >= 2")
    import serial
    folder = args.root / time.strftime("%Y%m%d_%H%M%S")
    folder.mkdir(parents=True,exist_ok=False)
    with serial.Serial(args.port,115200,timeout=.15) as ser:
        print("Motor VM OFF. Select Mode 7 > IR Test with physical KEY. Close other COM apps.")
        input("Place robot at reference with all six sensors facing appropriate walls; Enter: ")
        ref = stats(capture(ser,args.seconds,folder/"reference.csv"))
        input("Rotate robot manually through 360 degrees during capture; Enter: ")
        spin = stats(capture(ser,max(12,args.seconds),folder/"spin.csv"))
        models = {}
        points = []
        for i in (1,2):
            d = float(input(f"Measured front distance point {i} (mm): "))
            input("Place square to wall, motor VM OFF; Enter: ")
            points.append((d,stats(capture(ser,args.seconds,folder/f"point{i}.csv"))))
        for k in FRONT:
            models[k] = fit(points[0][0],points[0][1][k]["mean"],
                            points[1][0],points[1][1][k]["mean"])
        profile = {"reference":ref,"spin":spin,"front_models":models,
                   "status":"candidate_only","motor_enabled":False}
        (folder/"calibration.json").write_text(json.dumps(profile,indent=2))
        d = float(input("Independent validation distance (mm), between calibration points: "))
        input("Place robot at validation distance; Enter: ")
        rows = capture(ser,args.seconds,folder/"validation.csv")
        errors = []
        for row in rows:
            try:
                errors.append(sum(distance(row[k],models[k]) for k in FRONT)/2-d)
            except ValueError:
                pass
        report = {"valid":len(errors),"total":len(rows),"ground_truth_mm":d,
                  "mae_mm":statistics.mean(map(abs,errors)) if errors else None,
                  "rmse_mm":math.sqrt(statistics.mean(e*e for e in errors)) if errors else None}
        (folder/"validation.json").write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
        target = float(input("Shadow distance setpoint (mm): "))
        if not all(m["dmin"]<=target<=m["dmax"] for m in models.values()):
            raise ValueError("setpoint outside model")
        input("Move robot manually with VM OFF; Enter to capture shadow test: ")
        rows = capture(ser,max(10,args.seconds),folder/"shadow_raw.csv")
        with (folder/"shadow_control.csv").open("w",newline="") as f:
            writer=csv.writer(f)
            writer.writerow(["ms","target_mm","left_mm","right_mm","front_mm","error_mm","alignment_mm","request","motor_enabled"])
            for r in rows:
                try:
                    l,q=(distance(r[k],models[k]) for k in FRONT)
                    e=(l+q)/2-target
                    writer.writerow([r["ms"],target,l,q,(l+q)/2,e,q-l,"FORWARD" if e>2 else "REVERSE" if e< -2 else "HOLD",0])
                except ValueError:
                    writer.writerow([r["ms"],target,"","","","","","STOP_INVALID",0])
        print(folder)

if __name__ == "__main__":
    main()
