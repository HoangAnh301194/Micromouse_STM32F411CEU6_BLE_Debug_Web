# Encoder unified pipeline report

Firmware changes: none by this pipeline.
Safety: MOTOR_ON only by explicit multi-flag opt-in; includes 100% PWM.
Git branch: giomuadongbac
Git commit: 426f1715935165af0734c44327a80938912c144b

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
  "status": "ABORTED_OR_INCOMPLETE",
  "details": "Encoder motor telemetry incomplete or no response: {'OFF': 'FAIL_HEALTH', 'ON': 'NO_ENCODER_RESPONSE'}"
}
{
  "status": "REVIEWED",
  "source": "D:\\Micromouse_STM32F411CEU6_BLE_Debug_Web\\results\\encoder_qual_20261020_085212\\system_uart_hardware\\summary.json",
  "stage_count": 33,
  "motor_on_stages": 0,
  "both_wheels_mismatch_mean_pct": null,
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
