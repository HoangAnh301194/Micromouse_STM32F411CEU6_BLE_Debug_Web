# P0 Motor STOP / Coast / Brake — source truth-table audit (13 Oct 2026)

**Status: BLOCKED for physical STOP/Coast qualification.** Motor v7 sensor integration was already **PASS_BENCH** at <=20% PWM on elevated wheels, at user-measured VM 8.0 V. This audit does not revoke that motor/sensor telemetry result; it identifies a separate software-state contract problem in the low-level H-bridge STOP API.

## 1. Manufacturer truth table and current source

Verified against the **Toshiba TB6612FNG datasheet**, p.4, H-SW Control Function (STBY=HIGH). No active electrical test has been performed.

| IN1 | IN2 | PWM | Manufacturer's bridge state |
| --- | --- | --- | --- |
| LOW | LOW | HIGH | Stop: both motor outputs OFF / high impedance |
| HIGH | HIGH | LOW or HIGH | Short Brake: both outputs LOW |
| HIGH | LOW | HIGH | CW |
| LOW | HIGH | HIGH | CCW |
| HIGH | LOW | LOW | Short Brake |
| LOW | HIGH | LOW | Short Brake |
| **LOW** | **LOW** | **LOW** | **Not specified by this manufacturer's input truth table** |
| any | any | any, STBY=LOW | Standby: both outputs OFF / high impedance |

Current **`src/motor.c: Motor_Stop()`** uses:
- `AddDirection(..., 0)` on both motors, yielding IN1=IN2=LOW;
- `TIM2->CCR1 = TIM2->CCR2 = 0`, with TIM2 configured for PWM Mode 1, non-inverted;
- Thus the **software's settled command state is LOW/LOW/LOW**, which is **not guaranteed to be high-impedance coast** by the datasheet. This does NOT prove short braking either. Actual OUT1/OUT2 were not measured.

`Motor_Brake()` sets both direction inputs HIGH and CCR=ARR+1 (100% PWM), matching the documented short-brake *command* state. Actual driver current/output behavior and transistor/power-stage limits remain unknown.

The source also uses TIM2 CCR preload OC1PE/OC2PE and writes GPIO BSRR immediately: without pin/terminal measurements, brief PWM-cycle transitional states cannot be fully qualified. For brake release it commands a neutral interval via `brake_release_ticks=2` and a 30ms reversal gap; these are **source-inferred**, not instrumented.

## 2. Static source and protocol audit changes in this branch

A new **no-motion** `tools/motor_stop_truth_table_audit.py`:
- Models only documented inputs; classifies LOW/LOW/LOW as `UNSPECIFIED_BY_DATASHEET`, never as COAST or BRAKE;
- Extracts the real functions `Motor_Stop`, `Motor_Brake`, `Motor_SetPairPermille`, `Motor_Update1ms` to inspect CCR, GPIO directions, preload, and release/reverse gap;
- Self-tests the documented table, the actual current source state and non-motion safety blockers;
- Integrates into `tools/motor_full_qualification.py --audit`, so the current P0-STOP correctly becomes `BLOCKED` rather than `REVIEW`; other gates keep their independent statuses.

Only inaccurate code comments were corrected: physical wiring labels for right motor PB15/PB14, and current 30kHz PWM in source/header (previous comments said 20kHz). **No runtime motor/PWM command behavior or pin mapping has been altered.**

Run without any flashing or motor power:

```powershell
python tools/motor_stop_truth_table_audit.py --self-test
python tools/motor_stop_truth_table_audit.py
python tools/motor_full_qualification.py --audit
```

## 3. Electrical evidence needed before changing physical STOP or energizing brake

1. With VM disconnected, confirm actual TB6612 logic VCC and MCU GPIO HIGH levels; at chip VCC=5 V its required VIH(min) = 3.5 V and STM32 nominal 3.3 V is out of guaranteed HIGH range. Confirm common ground and actual motor direction wiring PB15/PB14 (right), PB13/PB12 (left), PWM PA15/PB3 without rewiring an unchangeable PCB.
2. Identify exact N20 motor and gearbox, supply voltage rating, no-load current and stall current from genuine documentation where possible. Do not deliberately lock a powered wheel to 'measure stall' with an unprotected DMM.
3. For physical electrical tests use appropriately rated current-limited equipment plus oscilloscope / isolated or differential-rated probes where required. Capture TB6612 **input PWM, IN1, IN2** and OUT1/OUT2 across a safe command-to-neutral transition, including CCR preload latency and any current surge. A hand voltmeter alone cannot validate 30kHz PWM timing or millisecond transients.
4. Validate short-brake transient **only after** current/thermal and external VM cutoff conditions are satisfied; no automatic `Motor_Brake()` pulse, reversal or 80–100% PWM has been added here.
5. Consider a separately reviewed firmware state-machine revision of `Motor_Stop()` toward **datasheet-defined LOW/LOW/HIGH** with safe update sequencing. That changes the prior `CCR=0` protocol assumptions, the register neutral snapshot and PWM-host sample invariants; do not patch it blindly into running v4/v6/v7/v8 without refactoring and re-testing the whole pipeline.

## 4. Status after source audit

- **P1 sensor concurrency at <=20%**: PASS_BENCH, no need to rerun as part of this static audit.
- **P0 STOP semantic correctness**: BLOCKED (LOW/LOW/LOW is not guaranteed Coast).
- **P0 current, heat, output voltage/waveforms**: BLOCKED until externally measured.
- **P1 8–9% startup order/encoder repeatability**: REVIEW; still a separate controlled characterization question.
- **P2 sustained/loaded, >60% duty, live brake/reverse**: BLOCKED, no automated powered tests authorised by this audit.

Datasheet: https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG
