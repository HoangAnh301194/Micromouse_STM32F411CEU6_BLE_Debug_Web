#!/usr/bin/env python3
"""Offline acceptance: selective MPU/I2C RC2 + USART1 integration with Motor Rev3.

This checks exact donor source blobs and the target production wiring.
It neither opens serial ports nor flashes/drives motors; STM32 firmware builds
and actual MCU motor-on qualification are independent gates.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent.parent
MOTOR_BASELINE = "b40228e85382e54f2487d8145fa3b0558802065b"
MPU_SOURCE = "5219e04d3fe7b8fdb5a092100154e6d9f0317e09"
UART_SOURCE = "4ed2816f8c9b2dbc0f4adbebbe88dcd1b5dde41c"
FROZEN = {
 "src/motor.c": "57fc04072cd9d206e22e54e987a9b63b450739f0",
 "include/motor.h": "9ed969ef6e46d682dbfd7a0607bff629be2330fc",
 "src/motor_test.c": "fdd1be9cb4ae353e76b289f765b206424a62cb74",
 "include/motor_test.h": "1cf62c21e2e611770bc18d719693b71dde165b95",
 "src/i2c.c": "25735b2dcf2b7389a20f0c4e0ada4ee6e2a84a2b",
 "include/i2c.h": "e2351fb08a608372ccab5833442692abd999bdac",
 "src/mpu6050.c": "6c1a4530251975cfa2de84a92f33a738e25be942",
 "include/mpu6050.h": "5ff45b667126b3ca49db72c265eb103be39b4f37",
 "include/mpu_policy.h": "b7430e15b7c82af8fc71abf3a90d25426034ab99",
 "src/system_timer.c": "29be7bc48279c113f695f246f6937743edcc6575",
 "include/system_timer.h": "4079c7d48c45ee49515a1d3d085caa10146ade7c",
}
MUST = tuple(FROZEN) + (
 "src/main.c", "src/control.c", "src/uart.c", "include/uart.h",
 "src/motion.c", "include/control.h", "platformio.ini",
 "include/pinout.h", "src/bt_debug.c",
)
class GateError(ValueError): pass
def git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
def check(s: dict[str,bytes]) -> dict:
    for path,expected in FROZEN.items():
        data=s.get(path)
        if data is None:
            raise GateError("Missing frozen source: "+path)
        if expected not in (git_blob(data), git_blob(data.replace(b"\r\n",b"\n"))):
            raise GateError("Frozen donor source drift: "+path)
    def source(path: str) -> str:
        if path not in s: raise GateError("Missing target file: "+path)
        return s[path].decode("utf-8")
    main,ctrl,config,uart,header,mpu = map(source,(
       "src/main.c","src/control.c","platformio.ini",
       "src/uart.c","include/uart.h","src/mpu6050.c"))
    timer=source("src/system_timer.c")
    i2c=source("src/i2c.c")
    bt=source("src/bt_debug.c")
    pin=source("include/pinout.h")
    if not all(f"-D{key}=1" in config for key in (
            "MPU_SAMPLING_POLICY_B", "MPU_TIM11_TICK_SCHED", "UART_DEBUG_RX")):
        raise GateError("RC2 policy/tick and UART RX not enabled in default firmware")
    if not re.search(r"\[env:genericSTM32F411CE\][\s\S]*?-DMPU_TIM11_TICK_SCHED=1",config):
        raise GateError("RC2 flags missing from default build profile")
    if 'USART1->BRR' not in uart or 'DMA2_Stream7_IRQHandler' not in uart or \
       'USART1_IRQHandler' not in uart or 'UART_TryReadByte' not in uart or \
       'defined(UART_DEBUG_RX)' not in uart or 'defined(UART_DEBUG_RX)' not in header:
        raise GateError("USART1 PA9/PA10 DMA-TX + IRQ-RX implementation incomplete")
    if 'UART_Init();' not in main or 'UART_TryReadByte(&ch)' not in main:
        raise GateError("Production main lacks UART1 init/RX consumption")
    if 'HandleBleCommand(ch)' in main or "if (ch == 'F')" in main:
        raise GateError("USB-UART debug must be read-only, not a motor controller")
    if "if (!ImuReadyForCommand(&yaw)) return;" not in main or \
       "if (c == 's' || c == 'S') { Motion_Stop(); return; }" not in main:
        raise GateError("Motion commands are not gated by MPU readiness, STOP priority")
    if 'MPU6050_GetSnapshot(&mpu, &snapshot)' not in main or \
       '(uint32_t)(micros() - snapshot.completed_us) > 3000U' not in main:
        raise GateError("No valid age-bounded MPU guard in BLE motion command")
    if '(void)MPU6050_Update(&mpu);' not in main:
        raise GateError("MPU nonblocking state machine missing from main loop")
    if 'MPU6050_Calibrate(&mpu)' not in main or \
       main.index('Control_Start();') > main.index('    while (1) {',main.index('Control_Start();')):
        raise GateError("Startup calibration/scheduler order invalid")
    if 'I2C_DMA_Init(I2C_1)' in main:
        raise GateError("MPU RC2 uses IRQ, do not activate legacy RX DMA")
    if 'MPU6050_Update(control_mpu)' in ctrl or 'I2C_Read' in ctrl:
        raise GateError("Blocking/I2C state machine must not run in TIM11")
    if ctrl.count('Motor_Update1ms();') != 1:
        raise GateError("Exactly one Motor_Update1ms per TIM11 interrupt")
    if ctrl.count('MPU6050_NotifyControlTick(control_mpu, micros());') != 1:
        raise GateError("Exactly one TIM11 RC2 tick notification required")
    if ctrl.count('MPU6050_GetSnapshot(control_mpu, &imu)') != 1 or \
       '(uint32_t)(micros() - imu.completed_us) <= 3000U' not in ctrl or \
       'Motion_Stop();' not in ctrl:
        raise GateError("ISR missing fail-closed 3ms snapshot consumer")
    if not (ctrl.index('Encoder_Update(CONTROL_DT_S);') <
            ctrl.index('Motor_Update1ms();') <
            ctrl.index('MPU6050_GetSnapshot(control_mpu, &imu)') <
            ctrl.index('Motion_Update(CONTROL_DT_S,')):
        raise GateError("Motor/MPU/Motion TIM11 ordering changed")
    if 'SystemTimer_ArmWakeAfterUs' not in timer or \
       'TIM5_IRQHandler' not in timer or 'I2C1_EV_IRQHandler' not in i2c or \
       'I2C1_ER_IRQHandler' not in i2c:
        raise GateError("TIM5 bounded retry / nonblocking I2C IRQ missing")
    if 'USART2_IRQHandler' not in bt or 'USART1_IRQHandler' in bt:
        raise GateError("USART1/USART2 handler collision")
    for alias,pin_no in (
      ("DEBUG_TX_PIN",9),("DEBUG_RX_PIN",10),("BT_TX_PIN",2),("BT_RX_PIN",3),
      ("I2C_SCL_PIN",8),("I2C_SDA_PIN",9)):
        if not re.search(rf"(?m)^#define\s+{alias}\s+{pin_no}\b",pin):
            raise GateError("Board UART/I2C pin mapping drift: "+alias)
    return {"status":"PASS_SELECTIVE_UART1_MPU_RC2_INTEGRATION",
       "rc2_tested_source":MPU_SOURCE,
       "motor_frozen_baseline":MOTOR_BASELINE,
       "uart_donor_head":UART_SOURCE,
       "pinned_files":len(FROZEN),
       "motor_pwm_scheduler":"TIM11_1KHZ",
       "i2c_transfer":"IRQ_NONBLOCKING_MAIN_CONTEXT",
       "gyro_sampling":"POLICY_B_TICK_RC2",
       "uart1":"PA9_TX_DMA_PA10_RX_IRQ_READ_ONLY",
       "uart2_ble":"UNCHANGED",
       "motor_on_hardware_test":"NOT_PERFORMED",
       "electrical_safety":"NOT_QUALIFIED"}

def gather(root: Path=ROOT) -> dict[str,bytes]:
    return {p:(root/p).read_bytes() for p in MUST}
def self_test() -> None:
    source=gather()
    check(source)
    cases=[
      ("src/control.c",b"    Motor_Update1ms();",b"    /* removed */", "Exactly one"),
      ("src/control.c",b"MPU6050_NotifyControlTick(control_mpu, micros());",
       b"/* removed scheduler notify */", "tick notification"),
      ("src/control.c",b"(uint32_t)(micros() - imu.completed_us) <= 3000U",
       b"1U", "3ms snapshot"),
      ("src/main.c",b"(void)MPU6050_Update(&mpu);",b"/* removed */", "main loop"),
      ("src/main.c",b"if (!ImuReadyForCommand(&yaw)) return;",
       b"/* bypass safety */", "not gated"),
      ("src/main.c",b"UART_TryReadByte(&ch)",b"HandleBleCommand(ch)", "UART1 init/RX"),
      ("platformio.ini",b"-DMPU_TIM11_TICK_SCHED=1",
       b"-DMPU_TIM11_TICK_SCHED=0","not enabled"),
      ("src/mpu6050.c",b"#include \"mpu6050.h\"",
       b"#include \"other.h\"", "source drift"),
    ]
    for path,a,b,expected in cases:
        if a not in source[path]: raise AssertionError("Bad fixture: "+path)
        m=dict(source);m[path]=source[path].replace(a,b,1)
        try: check(m)
        except GateError as e:
            if expected not in str(e): raise AssertionError((path,expected,str(e)))
        else: raise AssertionError("Mutation not detected: "+path)

def main() -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    group=ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test",action="store_true")
    group.add_argument("--audit",action="store_true")
    opt=ap.parse_args()
    try:
        if opt.self_test:
            self_test();print("PASS: 8 negative mutation tests, no hardware access")
        else: print(json.dumps(check(gather()),indent=2))
    except (GateError,AssertionError,OSError,UnicodeError) as e:
        print("FAIL_SELECTIVE_UART1_MPU_RC2_INTEGRATION: "+str(e),file=sys.stderr)
        return 1
    return 0
if __name__=="__main__":
    raise SystemExit(main())
