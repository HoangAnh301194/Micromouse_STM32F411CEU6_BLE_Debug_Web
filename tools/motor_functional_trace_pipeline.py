#!/usr/bin/env python3
"""STM32 USART1 PA9/PA10 read-only 1-kHz Motor Driver register trace.

Modes:
  --self-test                  synthetic parser + source safety regressions only
  --analyze <raw_serial.log>   analyze already recorded UART text
  --port COM6 --confirm-vm-disconnected
                               ask an ALREADY-FLASHED v14/16/17/18 board for
                               a no-drive 512-tick trace (no upload/START).

MCU register snapshots are NOT measured PWM output edges, H-bridge current,
electrical coast, stopping time or physical safety qualification.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
N = 512
EXPECTED_FIELDS = {"i", "us", "dt", "cl", "cr", "al", "ar",
                   "tl", "tr", "el", "er", "p", "s", "r"}


class TraceError(ValueError):
    pass


def fields(record: str, prefix: str) -> dict[str, int | str]:
    if not record.startswith(prefix + ","):
        raise TraceError(f"Expected {prefix} line")
    data = {}
    for field in record[len(prefix) + 1:].split(","):
        if "=" not in field:
            raise TraceError(f"Non-key/value entry in {prefix}")
        k, v = field.split("=", 1)
        if k in data:
            raise TraceError(f"Duplicate {k} in {prefix}")
        if re.fullmatch(r"-?[0-9]+", v):
            data[k] = int(v)
        elif k in ("rx_mode", "rx", "source") and re.fullmatch(r"[A-Za-z0-9_]+", v):
            data[k] = v
        else:
            raise TraceError(f"Invalid {k} in {prefix}")
    return data


def analyze_text(content: str) -> dict:
    lines = [s.strip() for s in content.splitlines() if s.strip()]
    starts = [idx for idx, s in enumerate(lines) if s.startswith("MFD_BEGIN,")]
    ends = [idx for idx, s in enumerate(lines) if s.startswith("MFD_END,")]
    if len(starts) != 1 or len(ends) != 1 or ends[0] <= starts[0]:
        raise TraceError("Exactly one complete MFD_BEGIN -> MFD_END session required")
    a, z = starts[0], ends[0]
    begin = fields(lines[a], "MFD_BEGIN")
    end = fields(lines[z], "MFD_END")
    if (begin.get("v") != 1 or begin.get("n") != N or
        begin.get("p0_locked") != 1 or
        end.get("n") != N):
        raise TraceError("Wrong protocol version, trace length or missing firmware P0 lock")
    arr = begin.get("arr", 0)
    if not 500 <= arr <= 65534:
        raise TraceError("Implausible TIM2 ARR")
    if end.get("tx_drop", -1) != 0:
        raise TraceError("UART dropped messages: trace cannot be trusted")
    records = [s for s in lines[a + 1:z] if s.startswith("MFD,")]
    if len(records) != N:
        raise TraceError(f"Missing/extra trace records: expected {N}, got {len(records)}")
    parsed = [fields(line, "MFD") for line in records]
    if any(not EXPECTED_FIELDS.issubset(x.keys()) for x in parsed):
        raise TraceError("Missing mandatory per-tick trace fields")
    bad = []
    gaps = []
    for i, v in enumerate(parsed):
        if v["i"] != i:
            bad.append(f"sample index out of sequence: {i} != {v['i']}")
        if not 0 <= v["p"] < 16:
            bad.append(f"impossible direction GPIO bitfield: i={i}")
        if not 0 <= v["cl"] <= arr + 1 or not 0 <= v["cr"] <= arr + 1:
            bad.append(f"CCR outside [0,ARR+1]: i={i}")
        if not 0 <= v["s"] <= 3 or not 0 <= v["r"] <= 3:
            bad.append(f"unexpected test state/reason: i={i}")
        if i:
            delta = (v["us"] - parsed[i - 1]["us"]) & 0xFFFFFFFF
            gaps.append(delta)
            if delta > 1500 or delta < 500:
                bad.append(f"TIM11 timestamp gap {delta} us at index={i}")
            if v["dt"] != min(65535, delta):
                # ISR timestamp and interval_us are sampled from same TIM5.
                bad.append(f"TIM11 dt mismatch {v['dt']} vs {delta} at index={i}")
    if bad:
        raise TraceError("; ".join(bad[:20]))
    # The current MCU P0 lock forbids powered START entirely. A nonzero
    # target/applied, direction or active Motor Test state must fail closed.
    nonneutral = [
        i for i, v in enumerate(parsed)
        if v["p"] != 0 or v["cl"] != arr + 1 or v["cr"] != arr + 1 or
           v["al"] != 0 or v["ar"] != 0 or v["tl"] != 0 or v["tr"] != 0 or v["s"] != 0
    ]
    if nonneutral:
        raise TraceError(f"P0-locked idle trace contains non-neutral command/register at indices {nonneutral[:10]}")
    outliers = sum(not (950 <= d <= 1050) for d in gaps)
    left_displacement = parsed[-1]["el"] - parsed[0]["el"]
    right_displacement = parsed[-1]["er"] - parsed[0]["er"]
    return {
        "status": "PASS_IDLE_REGISTERS" if not outliers else "REVIEW_TIM11_JITTER",
        "qualified_scope": "idle CPU-visible CCR/GPIO registers and ISR/encoder counters only",
        "sample_count": N,
        "nominal_duration_ms": N,
        "observed_span_us": (parsed[-1]["us"] - parsed[0]["us"]) & 0xFFFFFFFF,
        "arr": arr,
        "ccr_idle": arr + 1,
        "interval_min_us": min(gaps),
        "interval_max_us": max(gaps),
        "tick_jitter_over_50us": outliers,
        "encoder_delta_left": left_displacement,
        "encoder_delta_right": right_displacement,
        "motor_was_started": False,
        "uart_dropped_messages": end["tx_drop"],
        "motor_functional_response_qualified": False,
        "physical_pwm_or_brake_current_qualified": False,
        "evidence_source": "UART SRAM-register trace, not an oscilloscope",
    }


def source_regression() -> None:
    bench = (ROOT / "src/motor_auto_bench.c").read_text(encoding="utf-8")
    control = (ROOT / "src/control.c").read_text(encoding="utf-8")
    trace = (ROOT / "src/motor_functional_trace.c").read_text(encoding="utf-8")
    header = (ROOT / "include/motor_functional_trace.h").read_text(encoding="utf-8")
    assert "#define MOTOR_AUTO_P0_ARM_LOCKED 1U" in bench
    assert 'strcmp(line,"MOTOR TRACE")' in bench
    assert "state == AUTO_WAIT && MotorFunctionalTrace_Begin()" in bench
    assert "MotorFunctionalTrace_Poll(now);" in bench
    assert "#define MOTOR_FUNCTIONAL_TRACE_SAMPLES 512U" in header
    assert "MotorFunctionalTrace_Record1ms(entry_us, interval_us);" in control
    update = control.split("#if defined(MOTOR_PWM_DIAG) || defined(MOTOR_AUTO_BENCH)", 1)[1]
    before = update.split("Control_TimingFinish(start_cycles);", 1)[0]
    assert before.index("Encoder_Update(CONTROL_DT_S);") < before.index("Motor_Update1ms();")
    assert before.index("Motor_Update1ms();") < before.index("MotorTest_Tick1ms();")
    assert before.index("MotorTest_Tick1ms();") < before.index("MotorFunctionalTrace_Record1ms")
    record = trace.split("void MotorFunctionalTrace_Record1ms", 1)[1].split("void MotorFunctionalTrace_Poll", 1)[0]
    assert "UART_Printf" not in record and "UART_Send" not in record
    assert "Motor_SetPair" not in record and "Motor_Brake(" not in record
    assert "Motor_Stop(" not in record
    # Ensure the source never silently begins MotorTest during the read-only command.
    trace_command = bench.split('strcmp(line,"MOTOR TRACE")', 1)[1].split('strcmp(line,"MOTOR KEEP")', 1)[0]
    # Strip explanatory C comments before checking executable actions.
    trace_command = re.sub(r"/\*.*?\*/", "", trace_command, flags=re.DOTALL)
    assert "MotorTest_Start" not in trace_command and "Motor_SetPair" not in trace_command


def synthetic_records() -> str:
    items = ["MFD_ARMED,n=512,motor_start=0",
             "MFD_BEGIN,v=1,n=512,arr=3199,p0_locked=1,source=registers"]
    for i in range(N):
        items.append(
            f"MFD,i={i},us={1000000 + 1000*i},dt={1000 if i else 0},"
            "cl=3200,cr=3200,al=0,ar=0,tl=0,tr=0,"
            "el=0,er=0,p=0,s=0,r=0"
        )
    items.append("MFD_END,n=512,tx_drop=0")
    return "\n".join(items) + "\n"


def self_test() -> None:
    source_regression()
    good = synthetic_records()
    p = analyze_text(good)
    assert p["status"] == "PASS_IDLE_REGISTERS" and p["sample_count"] == N
    cases = [
        good.replace("MFD,i=257,", "MFD,i=258,", 1),
        good.replace("MFD,i=257,", "MFD_X,i=257,", 1),
        good.replace("MFD_END,n=512,tx_drop=0", "MFD_END,n=512,tx_drop=1", 1),
        good.replace("cl=3200,cr=3200", "cl=0,cr=3200", 1),
        good.replace("p=0,s=0,r=0", "p=1,s=0,r=0", 1),
        good.replace("al=0,ar=0", "al=20,ar=0", 1),
        good.replace("MFD,i=257,us=1257000,dt=1000",
                     "MFD,i=257,us=1259000,dt=3000", 1),
        good.replace("p0_locked=1", "p0_locked=0", 1),
    ]
    for idx, candidate in enumerate(cases):
        try:
            analyze_text(candidate)
        except TraceError:
            pass
        else:
            raise AssertionError(f"Negative trace case {idx} incorrectly accepted")
    print("PASS: 512-tick idle trace, UART/CCR/GPIO/timing integrity and 8 fail-closed cases.")
    print("SOURCE REGRESSION PASS; synthetic signals are NOT real motor measurements.")


def capture_device(port: str, timeout: int) -> str:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Install pyserial using: python -m pip install pyserial") from exc
    out: list[str] = []
    with serial.Serial(port=port, baudrate=115200, timeout=0.2, write_timeout=1) as ser:
        ser.reset_input_buffer()
        start = time.monotonic()
        ready = False
        profile = False
        while time.monotonic() - start < min(timeout, 90):
            ser.write(b"MOTOR PING\n")
            for _ in range(8):
                line = ser.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                out.append(line)
                if line.startswith("MOTOR_AUTO_PROFILE,"):
                    try:
                        info = fields(line, "MOTOR_AUTO_PROFILE")
                    except TraceError:
                        raise RuntimeError("Malformed firmware PROFILE")
                    if info.get("version") not in (14, 16, 17, 18) or info.get("p0_locked") != 1:
                        raise RuntimeError("Old/missing P0-locked firmware. No trace request issued.")
                    profile = True
                if line.startswith("MOTOR_AUTO_READY,"):
                    state = fields(line, "MOTOR_AUTO_READY")
                    if (not profile or state.get("p0_locked") != 1 or
                        state.get("state") != 0):
                        raise RuntimeError("Firmware not P0-locked/idle; refusing trace request")
                    ready = True
                    break
            if ready:
                break
        if not ready:
            raise RuntimeError("No locked-and-idle READY received; no TRACE or START sent")
        # Only read-only trace request; never flashing or activating the motor.
        ser.write(b"MOTOR TRACE\n")
        deadline = time.monotonic() + max(25, timeout)
        while time.monotonic() < deadline:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            out.append(line)
            if line.startswith("MFD_REJECT,"):
                raise RuntimeError(line)
            if line.startswith("MFD_END,"):
                break
        else:
            raise RuntimeError("Timed out before MFD_END; capture incomplete")
    return "\n".join(out) + "\n"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    actions = p.add_mutually_exclusive_group(required=True)
    actions.add_argument("--self-test", action="store_true")
    actions.add_argument("--analyze", type=Path)
    actions.add_argument("--port")
    p.add_argument("--confirm-vm-disconnected", action="store_true",
                   help="Required for live capture. Motor power VM MUST be physically disconnected.")
    p.add_argument("--timeout", type=int, default=65,
                   help="Seconds allocated for UART diagnostic dump after READY")
    p.add_argument("--output", type=Path,
                   help="Existing or new output directory (default motor_logs/functional_trace_UTC)")
    args = p.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.port and not args.confirm_vm_disconnected:
        p.error("For read-only UART capture, disconnect VM and pass --confirm-vm-disconnected.")
    if args.port:
        try:
            data = capture_device(args.port, args.timeout)
        except (RuntimeError, OSError) as exc:
            p.error(str(exc))
    else:
        data = args.analyze.read_text(encoding="utf-8", errors="replace")
    output = args.output or ROOT / "motor_logs" / (
        "functional_trace_" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    )
    output.mkdir(parents=True, exist_ok=True)
    (output / "raw_serial.log").write_text(data, encoding="utf-8")
    try:
        result = analyze_text(data)
    except TraceError as exc:
        result = {"status": "FAIL_CAPTURE", "reason": str(exc),
                  "motor_functional_response_qualified": False,
                  "physical_pwm_or_brake_current_qualified": False}
    (output / "functional_trace_report.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    summary = (
        "MOTOR FUNCTIONAL TRACE — REGISTER-LEVEL OBSERVATION ONLY\n"
        f"Status: {result['status']}\n"
        f"Samples: {result.get('sample_count', 'INVALID')}\n"
        f"TIM11 tick jitter >50us: {result.get('tick_jitter_over_50us', 'N/A')}\n"
        f"Encoder displacement L/R: {result.get('encoder_delta_left', 'N/A')} / "
        f"{result.get('encoder_delta_right', 'N/A')}\n"
        f"Error: {result.get('reason','none')}\n"
        "NO motor powered/started by script. No physical electrical qualification.\n"
    )
    (output / "functional_trace_summary.txt").write_text(summary, encoding="utf-8")
    print(summary)
    print("Output:", output.resolve())
    return 1 if result["status"] == "FAIL_CAPTURE" else 0


if __name__ == "__main__":
    raise SystemExit(main())
