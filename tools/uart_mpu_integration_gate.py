#!/usr/bin/env python3
"""Offline regression: verify RC2 main-loop acquisition + frozen Motor boundary."""
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def read(p):return (R/p).read_text(encoding="utf-8")
c=read("src/control.c");m=read("src/main.c");u=read("src/uart.c");ini=read("platformio.ini")
assert c.count("Motor_Update1ms();")==1
assert "MPU6050_Update(control_mpu);" not in c
assert "MPU6050_NotifyControlTick(control_mpu, micros());" in c
assert c.index("Motor_Update1ms();") < c.index("MPU6050_NotifyControlTick(")
assert "MPU6050_GetSnapshot(control_mpu, &gyro)" in c
assert "Motion_Stop(); /* Invalid, uncalibrated or stale gyro. */" in c
assert "MPU6050_Update(&mpu);" in m
assert "UART_Init();" in m and "UART_Printf(" in m
assert "if (ready && Motion_IsDone()) Motion_CommandStraight" in m
assert "-DMPU_TIM11_TICK_SCHED=1" in ini and "-DMPU_SAMPLING_POLICY_B=1" in ini
assert "DMA2_Stream7" in u and "USART1->BRR" in u
assert "USART1_IRQHandler" not in u and "USART_CR1_RE" not in u
print("PASS: MPU RC2 IRQ acquisition outside TIM11, fail-closed Motion; USART1 TX DMA (RX not yet implemented)")
