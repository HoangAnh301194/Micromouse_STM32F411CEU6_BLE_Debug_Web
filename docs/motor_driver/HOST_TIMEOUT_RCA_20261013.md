# RCA — Auto motor debug v3 HOST_TIMEOUT, 2026-10-13

## Evidence (actual hardware run)

Session: `motor_logs/auto_20261013_095735/` (commit `846d8c1538908f52a3b300cbf84e5d22b444d086`).

- `MOTOR_AUTO_PROFILE,version=3` and `MOTOR_AUTO_READY` successful.
- `MOTOR_AUTO_BEGIN` successful (host START command decoded).
- Round 0, stage 0, left 8% 200ms: 63 encoder counts, PWM TIM2 CCR=256; post-neutral +21 counts during 400ms.
- Round 0, stage 1, right 8% 200ms: only 1 right encoder count; PWM TIM2 CCR=256. Post-neutral right delta=-1 count.
- At **host timestamp 2.450210s**, firmware reported
  `MOTOR_AUTO_FAULT,reason=HOST_TIMEOUT,round=0,stage=1,completed=2,ccr_l=0,ccr_r=0`.
- TIM11 late/gaps and UART **TX** drop counters=0 in 57 received samples; 2 START, 2 END, 1 POST_END. Remaining 61 stages **were not measured**.
- v3 firmware did **not** count accepted `MOTOR KEEP` or USART1 RX overruns/framing, and host did not persist its number of transmissions: exact source of missing accepted heartbeat is **unproven**. A successful START proves RX initially worked; it does not prove reliable receipt of ongoing KEEP under telemetry load.

## Root-cause hypothesis / mechanism

The previous v3 `UART_TryReadByte` polled `USART1->SR/DR` from the main loop. UART TX telemetry (printf + DMA queue) and motor state machine execute in the same main context. At 115200 baud (~87us/byte) the main loop can fail to read an incoming byte in time and the STM32 UART can set ORE; v3 silently dropped any `ORE/FE/NE` byte and did not increment counters. Dropping one byte from the exact-match newline command `MOTOR KEEP\n` causes that whole heartbeat to be rejected. Multiple lost KEEP messages can lead to the existing 1600ms HOST_TIMEOUT. This is **plausible, not conclusively demonstrated by v3 raw logs**.

Another possibility is irregular host scheduling, serial driver buffering or external UART RX noise; the patch instruments both ends so the next capture can separate them.

## Code fixes — opt-in motor benchmark ONLY

1. `src/uart.c`: enable USART1 RXNE interrupt with preempt priority 6 and 256-byte single-producer/consumer ring buffer for `MOTOR_AUTO_BENCH`. Reading of DR occurs inside ISR even while the main loop formats TX telemetry. Monitor `rx_bytes`, `rx_ore`, `rx_fe`, `rx_noise`, `rx_ring_drop`.
2. `src/motor_auto_bench.c`: instrument *decoded* `MOTOR KEEP` count, `host_age_ms` and UART RX errors; emit `MOTOR_AUTO_LINK` after each completed stage, and add heartbeat diagnostics to the fault record.
3. `tools/motor_auto_pipeline.py`: transmit KEEP every 250ms (was 350ms), record host `host_keep_sent_count`, compare with latest MCU `keep_ok`; surface RX counters in JSON/text reports; reject RX errors.
4. Analyzer no longer interprets 61 unexecuted stages as `0 counts`. Stages not executed become `NOT_TESTED`, executed once become `PARTIAL` (not 3/3). This prevents false deadband/motor conclusions.
5. Serial protocol bumped **v3 → v4** and new `rx_mode=irq256` field required by handshake and report profile validation. Old firmware cannot be silently accepted by `--skip-flash`.

**Safety retained:** no PWM on reset, explicit UART START, independently bounded motor pulses (200ms each at <=20% single and <=16% simultaneous), MCU heartbeat fail-stop 1600ms still active, TIM11 gap fault-stop, 400ms post-stop observe, STBY fixed at HIGH. No electrical current limiting, hardware interlock or tested electrical brake.

## Confirm the fix

After updating to the new commit, execute **one command**:

```powershell
git fetch origin
git switch feature/motor-driver-optimization
git pull --ff-only origin feature/motor-driver-optimization

python tools/motor_auto_pipeline.py --run --port COM6 --vm-state powered --meter-vm-volts 8.2
```

Expected MCU handshake `version=4`, `rx_mode=irq256`; 63 START/END/POST_END and 63 `MOTOR_AUTO_LINK` records; `keep_ok` monotonically increasing and >0, `rx_ore=0`, `rx_fe=0`, `rx_ring_drop=0`. An RX-related fault or host transmission gap should now be diagnosable from `MOTOR_AUTO_FAULT` + `session_meta.json`.

Do **not** use `--skip-flash` for the first v4 run. **Keep both wheels elevated and unobstructed, with a physical motor supply disconnect ready.** After the session, push its `motor_logs/auto_...` directory for independent assessment. This patch was compiled and host-self-tested by GitHub Actions, **not yet hardware-verified**.
