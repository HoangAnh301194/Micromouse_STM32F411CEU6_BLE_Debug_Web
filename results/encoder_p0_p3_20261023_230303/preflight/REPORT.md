# Encoder unified pipeline report

Firmware changes: none by this pipeline.
Safety: MOTOR_ON only by explicit multi-flag opt-in; includes 100% PWM.
Git branch: giomuadongbac
Git commit: 925846ccf4d2cf97229f4b7e7803544ce7156a5b

## Software gates
- integration_audit: PASS
- integration_negative: PASS
- encoder_probe_parser: PASS
- shadow_ab_regression: PASS
- encoder_native: PASS
- pipeline_regression: PASS
- estimator_selftest: PASS
- motor_driver_native: PASS
- sweep_native: PASS
- ir_native: PASS
- ir_parser: PASS
- estimator_study: PASS
- build_locked: PASS
- build_encoder_on: PASS

## Static code audit
- No explicit TIM3/TIM4 IC1F/IC2F hardware glitch filter configuration.
- Raw PPS from one-millisecond delta: quantization 1000 counts/s.
- EMA alpha 0.30 at 1 kHz is not equivalent to encoder electrical filtering.
- 50 Hz SYS_EN during motor OFF/ON resolves short-term PPS trends but cannot prove zero missed edges.

## Hardware / sweep
{
  "status": "NOT_RUN"
}
{
  "status": "REVIEWED",
  "source": "D:\\Micromouse_STM32F411CEU6_BLE_Debug_Web\\results\\system_uart_20261019_104529\\summary.json",
  "stage_count": 66,
  "motor_on_stages": 33,
  "both_wheels_mismatch_mean_pct": 0.7,
  "qualified_electrically": false,
  "note": "Stage-average counts only; not 1ms dynamics or ground-truth RPM."
}
## Encoder OFF/ON high-rate telemetry
null
## Alpha A/B estimator comparison (1kHz MCU metrics)
null
## Qualification warnings (kept separate from Encoder measurements)
None

## Remaining limitations
- Input capture filter IC1F/IC2F and 10-ms speed estimator remain candidates, NOT auto-applied.
- Encoder diagnostic OFF/ON builds emit SYS_EN ~50 Hz; missing edges require independent reference; synthetic estimator study is not board qualification.
- estimator_metrics.csv and estimator_samples.csv (if present) are SYNTHETIC, NOT measured MCU response.
- 1430 counts per wheel revolution is retained as the user's current engineering approximation.
- Historical stage response is not a calibration of closed-loop speed PID.
