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
  "high_rate_encoder": {
    "status": "OBSERVED",
    "samples": 5998,
    "median_interval_ms": 20,
    "max_interval_ms": 20,
    "intervals_over_40ms": 0,
    "pps_comparison": {
      "L": {
        "mean_abs_ema_vs_window_pps": 0.0,
        "max_abs_ema_vs_window_pps": 0.0
      },
      "R": {
        "mean_abs_ema_vs_window_pps": 0.03,
        "max_abs_ema_vs_window_pps": 160.0
      }
    },
    "limitations": "50Hz is decimated; these differences are not solely measurement noise or proof of missed pulses."
  },
  "frame_counts": {
    "SYS_EN": 5998,
    "SYS_T": 120,
    "SYS_IR": 120,
    "SYS_IRD": 120,
    "SYS_IRC": 120,
    "SYS_M": 120,
    "SYS_U": 120,
    "SYS_UQ": 120,
    "SYS_E": 120,
    "SYS_CAL": 7,
    "SYS_IRV": 120
  },
  "seconds_between_encoder_frames": 119.0,
  "wheels": {
    "L": {
      "net_counts": 0,
      "nonzero_1s_windows": 0,
      "max_abs_1s_delta": 0,
      "last_filtered_pps": 0
    },
    "R": {
      "net_counts": 0,
      "nonzero_1s_windows": 2,
      "max_abs_1s_delta": 1,
      "last_filtered_pps": 0
    }
  },
  "tim11_late_delta": 0,
  "tim11_gap_delta": 0,
  "mpu_i2c_error_delta": 0,
  "mpu_timeout_delta": 0,
  "uart_tx_drop_delta": 0,
  "motor_lock_frames": 120,
  "all_reported_locked": true,
  "note": "1Hz snapshots do not qualify high-rate PPS jitter or missed edges."
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

## Remaining limitations
- Input capture filter IC1F/IC2F and 10-ms speed estimator remain candidates, NOT auto-applied.
- In encoder motor-off profile SYS_EN is ~50 Hz; remaining high-frequency glitches/missed edges require per-tick samples and independent reference.
- estimator_metrics.csv and estimator_samples.csv (if present) are SYNTHETIC, NOT measured MCU response.
- 1430 counts per wheel revolution is retained as the user's current engineering approximation.
- Historical stage response is not a calibration of closed-loop speed PID.
