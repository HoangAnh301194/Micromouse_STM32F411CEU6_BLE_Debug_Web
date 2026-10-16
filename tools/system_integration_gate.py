#!/usr/bin/env python3
"""Audit STM32 96 MHz integration debug profiles (software-only checks)."""
from pathlib import Path
import argparse
import configparser
import sys

ROOT=Path(__file__).resolve().parent.parent
REQUIRED={"CLOCK_VALIDATE_96M=1","CLOCK_HSI_TRIM=25",
          "MPU_SAMPLING_POLICY_B=1","MPU_TIM11_TICK_SCHED=1"}
class AuditError(Exception): pass
def audit(files=None):
    def get(path):
        return files[path] if files is not None else (ROOT/path).read_text(encoding="utf-8")
    cfg=configparser.ConfigParser(interpolation=None)
    cfg.read_string(get("platformio.ini"))
    for env in ("genericSTM32F411CE","integration_diag_motor_off_96m",
                "integration_diag_motor_on_96m"):
        flags=set(x.removeprefix("-D") for x in cfg.get(f"env:{env}","build_flags").split())
        if not REQUIRED.issubset(flags):raise AuditError(f"{env}: bad clock/MPU flags")
    off=cfg.get("env:integration_diag_motor_off_96m","build_flags")
    on=cfg.get("env:integration_diag_motor_on_96m","build_flags")
    if not all(s in off for s in ("INTEGRATION_DIAG=1","INTEGRATION_DIAG_MOTOR_OFF=1")):
        raise AuditError("OFF profile missing flags")
    if not all(s in on for s in ("INTEGRATION_DIAG=1","INTEGRATION_DIAG_MOTOR_ON=1")):
        raise AuditError("ON profile missing flags")
    if "INTEGRATION_DIAG" in cfg.get("env:genericSTM32F411CE","build_flags"):
        raise AuditError("Production firmware must NOT auto-run motor diagnostic")
    motor=get("src/integration_sweep.c")
    if not all(s in motor for s in
            ("SWEEP_STEP_DRIVE_MS 250U","SWEEP_STEP_COAST_MS 150U",
             "SWEEP_STEP_TOTAL 33U","SWEEP_HARD_LIMIT_MS 18000U",
             "SWEEP_LINK_TIMEOUT_MS 750U","Motor_LockOutputs();",
             "SWEEP_GYRO_MAX_AGE_US 3000U","Command(0U)")):
        raise AuditError("Sweep timing or fault-latch contract lost")
    control=get("src/control.c")
    if control.count("Motor_Update1ms();") != 1 or "DWT->CYCCNT" not in control:
        raise AuditError("TIM11 motor and cycle diagnostics mismatch")
    if "Sweep_ImuFaultFromISR()" not in control:
        raise AuditError("Gyro stale cannot abort live sweep")
    main=get("src/main.c")
    if "Sweep_Update(now, &mpu)" not in main or "IntegrationSerial_Poll(&mpu, now)" not in main:
        raise AuditError("Serial and motor timing loop not integrated")
    if "Motor_LockOutputs();" not in main:
        raise AuditError("Motor-off compile-time lock missing")
    serial=get("src/integration_serial.c")
    for token in ('"MODE OFF"','"MODE ON"','"START"','"PING"','"STOP"',
                  'UART_ReadByte(&value)','UART_RX_BUDGET'):
        if token not in serial:raise AuditError("Serial protocol token absent: "+token)
    return "PASS 96MHz system integration static checks; physical motor tests NOT run"
def main():
    p=argparse.ArgumentParser(); group=p.add_mutually_exclusive_group(required=True)
    group.add_argument("--audit",action="store_true")
    group.add_argument("--self-test",action="store_true")
    group.add_argument("--native-test",action="store_true")
    args=p.parse_args()
    try:
        if args.native_test:
            # Use actual C motor native host regression, not fake sweep results.
            import subprocess
            subprocess.run([sys.executable,str(ROOT/"tools/motor_driver_native_test.py")],
                           cwd=ROOT,check=True)
            print("PASS native Motor Driver; sweep C state machine NOT native qualified")
        elif args.self_test:
            import json
            names=("platformio.ini","src/integration_sweep.c","src/control.c","src/main.c",
                   "src/integration_serial.c")
            files={name:(ROOT/name).read_text(encoding="utf-8") for name in names}
            audit(files)
            mutations=[
                ("platformio.ini","-DCLOCK_HSI_TRIM=25","-DCLOCK_HSI_TRIM=20"),
                ("src/integration_sweep.c","SWEEP_LINK_TIMEOUT_MS 750U","SWEEP_LINK_TIMEOUT_MS 9999U"),
                ("src/control.c","Motor_Update1ms();","/* removed */"),
                ("src/main.c","IntegrationSerial_Poll(&mpu, now)","/* removed */"),
                ("src/integration_serial.c","UART_ReadByte(&value)","0")
            ]
            for name,old,new in mutations:
                if old not in files[name]: raise AuditError("fixture missing "+old)
                copy=dict(files);copy[name]=copy[name].replace(old,new,1)
                try:audit(copy)
                except AuditError:pass
                else:raise AuditError("unsafe mutation accepted: "+name)
            print("PASS 5 negative mutations rejected")
        else:print(audit())
    except (AuditError,Exception) as e:
        print("FAIL",e,file=sys.stderr);return 1
    return 0
if __name__=="__main__":raise SystemExit(main())
