#!/usr/bin/env python3
"""No-hardware Motor Driver transplant gate for the giomuadongbac firmware.

Does NOT power motors, validate current, flash firmware, or run a production
safety certification. Only the donor Motor Driver subsystem is transplanted.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent.parent
SOURCE_COMMIT = "4ed2816f8c9b2dbc0f4adbebbe88dcd1b5dde41c"
SOURCE_DRIVER_BLOBS = {
    "src/motor.c": "57fc04072cd9d206e22e54e987a9b63b450739f0",
    "include/motor.h": "9ed969ef6e46d682dbfd7a0607bff629be2330fc",
    "src/motor_test.c": "fdd1be9cb4ae353e76b289f765b206424a62cb74",
    "include/motor_test.h": "1cf62c21e2e611770bc18d719693b71dde165b95",
    "src/motor_startup.c": "2b6438ee93305a7d6d6e2e5dba1d83b65288a25c",
    "include/motor_startup.h": "12638ac6a5749609c7dc283c89af0a0ddefa8e79",
}
class IntegrationError(ValueError): pass

def git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()

def audit_sources(files: dict[str,bytes]) -> dict:
    for path, target in SOURCE_DRIVER_BLOBS.items():
        if path not in files:
            raise IntegrationError("Donor Motor Driver file missing: " + path)
        raw = files[path]
        if target not in (git_blob(raw), git_blob(raw.replace(b"\r\n", b"\n"))):
            raise IntegrationError("Donor Motor Driver baseline modified: " + path)
    def contents(path: str) -> str:
        if path not in files: raise IntegrationError("Missing integration file: " + path)
        return files[path].decode("utf-8")
    control, motion, pin, board = map(contents,
         ("src/control.c","src/motion.c","include/pinout.h","src/board.c"))
    hdr = contents("include/control.h")
    if "#define CONTROL_FREQUENCY_HZ  1000UL" not in hdr:
        raise IntegrationError("TIM11 scheduling frequency changed")
    if not re.search(r"(?m)^#define\s+MOTOR_PWM_FREQUENCY\s+30000U\b",pin):
        raise IntegrationError("TIM2 must generate 30kHz PWM")
    if "MOTOR_PWM_PERIOD" in pin or "MOTOR_PWM_PRESCALER" in pin:
        raise IntegrationError("Legacy fixed TIM2 period symbols remain")
    for macro,n in (("MOTOR_R_PWM_PIN",15),("MOTOR_L_PWM_PIN",3),
                    ("MOTOR_R_IN1_PIN",15),("MOTOR_R_IN2_PIN",14),
                    ("MOTOR_L_IN1_PIN",13),("MOTOR_L_IN2_PIN",12)):
        if not re.search(rf"(?m)^#define\s+{macro}\s+{n}\b",pin):
            raise IntegrationError("Motor pin mapping drift: "+macro)
    if '#include "motor.h"' not in control:
        raise IntegrationError("Control does not include Motor Driver API")
    # Count only actual call statements, not declarations or comments.
    if len(re.findall(r"(?m)^\s*Motor_Update1ms\(\);",control)) != 1:
        raise IntegrationError("Driver must be serviced exactly once in TIM11 path")
    # MPU RC2 moves I2C acquisition to main: TIM11 only stamps a sample
    # opportunity. Preserve driver service before MPU tick notification.
    if "MPU6050_Update(control_mpu);" in control:
        if control.index("Motor_Update1ms();") > control.index("MPU6050_Update(control_mpu);"):
            raise IntegrationError("TIM11 must service driver before MPU")
    elif "MPU6050_NotifyControlTick(control_mpu, micros());" in control:
        if control.index("Motor_Update1ms();") > control.index("MPU6050_NotifyControlTick(control_mpu, micros());"):
            raise IntegrationError("TIM11 must service driver before MPU tick notify")
    else:
        raise IntegrationError("Missing MPU scheduling handoff: unsafe integration")
    if control.index("Encoder_Update(CONTROL_DT_S);") > control.index("Motor_Update1ms();"):
        raise IntegrationError("Encoder update must precede Motor service")
    stop=control.split("void Control_Stop(void)",1)[-1].split("uint8_t Control_IsRunning(void)",1)[0]
    if not (stop.index("Motion_Stop();") < stop.index("TIM11->CR1 &= ~TIM_CR1_CEN;")):
        raise IntegrationError("Control_Stop must neutralize before disabling TIM11")
    if "Motor_KeepAlive(" in control or "Motor_KeepAlive(" in motion:
        raise IntegrationError("Motion/Control keepalive would conceal stalled controller")
    for name,body in (("Motion",motion),("Control",control)):
        for direct in ("TIM2->", "GPIOB->BSRR"):
            if direct in body: raise IntegrationError(name+" illegally owns "+direct)
    if "Motor_SetPair(ClampPwm(left_pwm), ClampPwm(right_pwm));" not in motion or \
       "Motor_SetPair(ClampPwm(-output), ClampPwm(output));" not in motion:
        raise IntegrationError("Motion no longer routes through Driver API")
    if "TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);" not in board or \
       "GPIOB->BSRR = (1UL << (MOTOR_L_IN1_PIN + 16U))" not in board:
        raise IntegrationError("HardFault GPIO direction and output-disable guard missing")
    return {"status":"PASS_MOTOR_DRIVER_GIOMUADONGBAC_INTEGRATION",
            "donor_source_commit":SOURCE_COMMIT,
            "donor_code_files":len(SOURCE_DRIVER_BLOBS),
            "timer11_frequency_hz":1000,
            "pwm_frequency_hz":30000,
            "motor_lease_ms":300,
            "encoder_rpm_or_battery_condition": "NOT_REQUIRED",
            "physical_mcu_integration": "NOT_VERIFIED",
            "powered_electrical_safety": "NOT_QUALIFIED"}

def gather(root: Path=ROOT) -> dict[str,bytes]:
    names=tuple(SOURCE_DRIVER_BLOBS)+("src/control.c","src/motion.c",
                                     "include/pinout.h","src/board.c","include/control.h")
    return {p:(root/p).read_bytes() for p in names}

def self_test() -> None:
    inputs=gather()
    audit_sources(inputs)
    mutants=[
       ("src/control.c",b"    Motor_Update1ms();",b"    /* no update */", "exactly once"),
       ("src/control.c",b"    Motor_Update1ms();",b"    Motor_KeepAlive();\n    Motor_Update1ms();", "keepalive"),
       ("src/motion.c",b"Motor_SetPair(ClampPwm(-output), ClampPwm(output));",
        b"TIM2->CCR1 = 0;", "illegally"),
       ("include/pinout.h",b"30000U",b"20000U", "30kHz"),
       ("src/board.c",b"TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);",
        b"/* removed */", "HardFault"),
       ("src/motor.c",b"#include \"board.h\"",b"#include \"motion.h\"", "baseline modified"),
    ]
    for path,old,repl,expected in mutants:
        if old not in inputs[path]: raise AssertionError(f"Bad test fixture {path}")
        modified=dict(inputs)
        modified[path]=inputs[path].replace(old,repl,1)
        try: audit_sources(modified)
        except IntegrationError as e:
            if expected not in str(e): raise AssertionError((expected,str(e)))
        else: raise AssertionError(f"Undetected mutation: {path}, {expected}")

def main() -> int:
    p=argparse.ArgumentParser(description=__doc__)
    flags=p.add_mutually_exclusive_group(required=True)
    flags.add_argument("--self-test",action="store_true")
    flags.add_argument("--audit",action="store_true")
    a=p.parse_args()
    try:
        if a.self_test:
            self_test(); print("PASS: integration gate rejects six incompatible source mutations")
        else: print(json.dumps(audit_sources(gather()),indent=2))
    except (OSError,IntegrationError,AssertionError,UnicodeError) as e:
        print("FAIL_MOTOR_DRIVER_TRANSPLANT: "+str(e),file=sys.stderr)
        return 1
    return 0

if __name__=="__main__": raise SystemExit(main())
