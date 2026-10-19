# Encoder unified pipeline report

Firmware changes: none by this pipeline.
Safety: MOTOR_ON only by explicit multi-flag opt-in; includes 100% PWM.
Git branch: giomuadongbac
Git commit: 8aab5e5d2c189776e3b2a452f616a002c8b9426e

## Software gates
- integration_audit: PASS
- integration_negative: PASS
- encoder_probe_parser: PASS
- encoder_native: PASS
- pipeline_regression: PASS
- estimator_selftest: PASS
- motor_driver_native: PASS
- sweep_native: PASS
- ir_native: PASS
- ir_parser: PASS
- estimator_study: PASS
- build_locked: PASS

## Static code audit
- No explicit TIM3/TIM4 IC1F/IC2F hardware glitch filter configuration.
- Raw PPS from one-millisecond delta: quantization 1000 counts/s.
- EMA alpha 0.30 at 1 kHz is not equivalent to encoder electrical filtering.
- 1 Hz SYS_E cannot detect high-rate glitches or independently prove zero missed edges.

## Hardware / sweep
{
  "status": "ABORTED_OR_INCOMPLETE",
  "details": "Motor sweep failed; check raw logs and STOP state"
}
{
  "status": "REVIEWED",
  "source": "D:\\Micromouse_STM32F411CEU6_BLE_Debug_Web\\results\\encoder_qual_20261020_014805\\system_uart_hardware\\summary.json",
  "stage_count": 66,
  "motor_on_stages": 33,
  "both_wheels_mismatch_mean_pct": 0.65,
  "qualified_electrically": false,
  "note": "Stage-average counts only; not 1ms dynamics or ground-truth RPM."
}

## Remaining limitations
- Input capture filter IC1F/IC2F and 10-ms speed estimator remain candidates, NOT auto-applied.
- In encoder motor-off profile SYS_EN is ~50 Hz; remaining high-frequency glitches/missed edges require per-tick samples and independent reference.
- estimator_metrics.csv and estimator_samples.csv (if present) are SYNTHETIC, NOT measured MCU response.
- 1430 counts per wheel revolution is retained as the user's current engineering approximation.
- Historical stage response is not a calibration of closed-loop speed PID.
