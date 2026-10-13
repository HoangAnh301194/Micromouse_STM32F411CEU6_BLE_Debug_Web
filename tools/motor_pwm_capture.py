#!/usr/bin/env python3
"""Serial capture + completeness validation for opt-in KEY-gated PWM bench.

This tool measures telemetry completeness, not motor electrical safety.
Requires external verification of motor rating and TB6612 logic VCC.
"""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import json
from pathlib import Path
import time

MAX_COMMAND = 120
EXPECTED_STAGES = 5
FIELDS = ("stage", "ms", "elapsed", "applied_l", "applied_r",
          "enc_l", "enc_r", "ccr_l", "ccr_r", "tim11_late",
          "tim11_max_cycles", "tim11_gaps", "tx_drop")
EXPECTED_COMMANDS = [(80,0),(0,80),(120,0),(0,120),(120,120)]


def parse_payload(line: str):
    payload = line.split("\t", 1)[-1].strip()
    tag, sep, rest = payload.partition(",")
    if not sep:
        return tag, {}
    data = {}
    for part in rest.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            data[k] = v
    return tag, data


def analyze(raw_lines):
    boot = [i for i, line in enumerate(raw_lines)
            if "[BOOT] KEY-armed motor PWM bench" in line]
    if not boot:
        return {"status": "INCOMPLETE", "errors": ["MOTOR BOOT not captured"]}
    lines = raw_lines[boot[-1]:]
    profile = [v for x in lines if (tag := parse_payload(x)[0]) == "MOTOR_PROFILE"
               for v in [parse_payload(x)[1]]]
    starts, ends, samples = [], [], []
    finished = False
    errors = []
    for line in lines:
        tag, value = parse_payload(line)
        if tag == "MOTOR_START":
            starts.append(value)
        elif tag == "MOTOR_END":
            ends.append(value)
        elif tag == "MOTOR_SAMPLE":
            samples.append(value)
        elif tag == "MOTOR_DONE":
            finished = True
        elif tag in ("MOTOR_BOOT_FAIL", "MOTOR_FAULT"):
            errors.append("Firmware reported " + tag)
    if not profile or any(profile[-1].get(k) != v for k, v in (
        ("version","2"), ("nominal_hz","96000000"),
        ("pwm_hz","30000"), ("hard_cap_permille", "120"),
        ("pulse_ms", "200"), ("key_arm_ms", "1500"),
        ("one_shot","1"), ("motor_off_at_boot", "1")
    )):
        errors.append("Missing/mismatched motor-off/KEY startup profile")
    if len(starts) != EXPECTED_STAGES or len(ends) != EXPECTED_STAGES or not finished:
        errors.append("Missing one or more of 5 manually requested motor pulses")
    sequence = [int(x.get("stage", "-1")) for x in starts]
    if sequence != list(range(EXPECTED_STAGES)):
        errors.append("Unexpected pulse order")
    if any(max(abs(int(x.get("l", "9999"))), abs(int(x.get("r", "9999"))))
           > MAX_COMMAND for x in starts):
        errors.append("Command outside experiment 12% duty ceiling")
    if [(int(x.get("l","-999")),int(x.get("r","-999"))) for x in starts] != EXPECTED_COMMANDS:
        errors.append("Nonmatching commanded PWM pulse matrix")
    if [int(x.get("stage","-1")) for x in ends] != list(range(EXPECTED_STAGES)):
        errors.append("Unexpected end sequence")
    for x in ends:
        if not (195 <= int(x.get("elapsed", "-1")) <= 205) or x.get("reason") != "0":
            errors.append("Pulse did not complete the intended 200ms timeout")
        if int(x.get("tx_drop", "1")):
            errors.append("UART queue drop during motor pulse")
    for x in samples:
        if any(abs(int(x.get(k,"99999"))) > MAX_COMMAND for k in ("applied_l","applied_r")):
            errors.append("Applied command exceeded 12% cap")
        if int(x.get("ccr_l","999999")) > 400 or int(x.get("ccr_r","999999")) > 400:
            errors.append("Unexpected TIM2 CCR: >12.5% of 3200 nominal")
        if int(x.get("tim11_gaps", "1")):
            errors.append("TIM11 1.5ms gap observed")
        if int(x.get("tx_drop", "1")):
            errors.append("UART telemetry drop observed")
    errors = sorted(set(errors))
    per_stage = []
    for stage in range(EXPECTED_STAGES):
        rows = [x for x in samples if x.get("stage") == str(stage)]
        if len(rows) < 3:
            errors.append(f"Insufficient motor telemetry stage {stage}")
        per_stage.append({
            "stage":stage, "records":len(rows),
            "last_left_count":int(rows[-1]["enc_l"]) if rows else None,
            "last_right_count":int(rows[-1]["enc_r"]) if rows else None,
            "max_applied_l_permille":max((abs(int(x["applied_l"])) for x in rows),default=0),
            "max_applied_r_permille":max((abs(int(x["applied_r"])) for x in rows),default=0),
            "last_ccr_left":int(rows[-1]["ccr_l"]) if rows else None,
            "last_ccr_right":int(rows[-1]["ccr_r"]) if rows else None,
            "max_tim11_late_ticks":max((int(x.get("tim11_late","0")) for x in rows),default=0),
            "any_encoder_count_changed":any(int(x["enc_l"]) != 0 or int(x["enc_r"]) != 0 for x in rows),
        })
    for stage, rows in enumerate([[x for x in samples if x.get("stage")==str(i)]
                                   for i in range(EXPECTED_STAGES)]):
        if rows and not any(abs(int(x["applied_l"])) + abs(int(x["applied_r"])) > 0 for x in rows):
            errors.append(f"No motor PWM applied in stage {stage}; check dedicated TIM11 path")
    return {
        "status":"TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL" if not errors else "INCOMPLETE",
        "pulses_started":len(starts), "pulses_ended":len(ends),
        "stages":per_stage, "errors":sorted(set(errors)),
        "motor_response_observed": any(x["any_encoder_count_changed"] for x in per_stage),
        "safety_note":"Commanded PWM, TIM2 CCR and encoder only. No current sensing, scope measurements, guaranteed logic-high margin or stall-current approval."
    }


def write_report(folder: Path):
    raw = folder / "raw_serial.log"
    if not raw.is_file():
        raise SystemExit("Missing raw_serial.log in " + str(folder))
    lines = raw.read_text(encoding="utf-8", errors="replace").splitlines()
    report = analyze(lines)
    (folder / "motor_pwm_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    samples = [v for x in lines if (tag := parse_payload(x)[0]) == "MOTOR_SAMPLE"
               for v in [parse_payload(x)[1]]]
    with (folder / "motor_samples.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(samples)
    print(json.dumps(report, indent=2))
    print("Output:", folder.resolve())
    return 0 if report["status"].startswith("TRACE_COMPLETE") else 1


def self_test():
    lines = ["[BOOT] KEY-armed motor PWM bench; MOTOR OFF",
             "MOTOR_PROFILE,version=2,nominal_hz=96000000,pwm_hz=30000,hard_cap_permille=120,pulse_ms=200,key_arm_ms=1500,one_shot=1,motor_off_at_boot=1"]
    for stage, (l,r) in enumerate(EXPECTED_COMMANDS):
        lines.append(f"MOTOR_START,stage={stage},l={l},r={r},pulse_ms=200")
        for ms in [20,40,60,80,100,120,140,160,180,200]:
            lines.append(f"MOTOR_SAMPLE,stage={stage},ms={ms},elapsed={ms},applied_l={l},applied_r={r},enc_l={ms if l else 0},enc_r={ms if r else 0},ccr_l={l*3},ccr_r={r*3},tim11_late=0,tim11_max_cycles=1200,tim11_gaps=0,tx_drop=0")
        lines.append(f"MOTOR_END,stage={stage},reason=0,elapsed=200,tx_drop=0")
    lines.append("MOTOR_DONE,all_5_pulses_complete,pwm=0")
    assert analyze(lines)["status"] == "TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL"
    assert analyze(lines[1:])["status"] == "INCOMPLETE"
    faulty = lines[:]
    faulty[2] = "MOTOR_START,stage=0,l=1000,r=0,pulse_ms=200"
    assert analyze(faulty)["status"] == "INCOMPLETE"
    faulted = lines + ["MOTOR_FAULT,TIM11_GAP,LATCHED"]
    assert analyze(faulted)["status"] == "INCOMPLETE"
    stopped = [x.replace("reason=0,elapsed=200","reason=1,elapsed=100") for x in lines]
    assert analyze(stopped)["status"] == "INCOMPLETE"
    print("SELF-TEST PASSED: complete pulse capture, wrong profile, duty escape, latched fault, incomplete pulse rejected.")
    print("Synthetic host test; NOT evidence of motor current, PWM signal, or physical operation.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--port")
    group.add_argument("--folder", type=Path)
    group.add_argument("--self-test", action="store_true")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=90)
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    if args.folder:
        raise SystemExit(write_report(args.folder))
    if not args.port:
        ap.error("Use --port COMx or --folder PATH, or --self-test")
    if args.seconds < 20:
        ap.error("Capture should last at least 20s")
    try:
        import serial
    except ImportError:
        ap.error("pip install pyserial")
    folder = Path("motor_logs") / ("pwm_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    folder.mkdir(parents=True, exist_ok=True)
    print("Open", args.port, "- after opening, press STM32 RESET once.")
    print("Motor does NOT start until deliberate KEY arm + fresh press.")
    print("Lift wheels and provide direct physical VM disconnect. 3.3V MCU GPIO HIGH is outside the guaranteed TB6612 HIGH threshold at VCC=5V.")
    print("This test does NOT measure current, heat, motor rated duty or switching transients.")
    print("Capture duration", args.seconds, "seconds, folder", folder)
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            ser.reset_input_buffer()
            t0 = time.perf_counter()
            with (folder / "raw_serial.log").open("w", encoding="utf-8") as f:
                while time.perf_counter() - t0 < args.seconds:
                    rx = ser.readline()
                    if not rx:
                        continue
                    message = rx.decode("utf-8", errors="replace").strip()
                    f.write(f"{time.perf_counter()-t0:.9f}\t{message}\n")
                    if any(x in message for x in
                           ("MOTOR_ARMED", "MOTOR_START", "MOTOR_END", "MOTOR_DONE", "MOTOR_FAULT")):
                        print(message)
    except KeyboardInterrupt:
        print("Interrupted: use physical motor supply disconnect if needed.")
    except serial.SerialException as exc:
        raise SystemExit("Cannot open serial port: " + str(exc)) from exc
    raise SystemExit(write_report(folder))


if __name__ == "__main__":
    main()
