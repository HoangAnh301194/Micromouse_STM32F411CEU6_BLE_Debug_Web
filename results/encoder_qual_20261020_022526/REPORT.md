# Encoder unified pipeline report

Firmware changes: none by this pipeline.
Safety: MOTOR_ON only by explicit multi-flag opt-in; includes 100% PWM.
Git branch: giomuadongbac
Git commit: bcf2803493529aeea62720e362c3233e657f5a1b

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
- build_encoder_on: PASS

## Static code audit
- No explicit TIM3/TIM4 IC1F/IC2F hardware glitch filter configuration.
- Raw PPS from one-millisecond delta: quantization 1000 counts/s.
- EMA alpha 0.30 at 1 kHz is not equivalent to encoder electrical filtering.
- 50 Hz SYS_EN during motor OFF/ON resolves short-term PPS trends but cannot prove zero missed edges.

## Hardware / sweep
{
  "status": "SWEEP_COMPLETE_I2C_RATE_UNQUALIFIED",
  "execution": {
    "pass": false,
    "returncode": 1,
    "log": "motor_on_automated.log",
    "diagnostic_excerpt": "integration_diag_encoder_motor_on_96m  SUCCESS   00:00:06.458\n========================= 1 succeeded in 00:00:06.458 =========================\nOFF PASS steps 33 failed_checks []\nBuilding/flashing explicitly selected integration_diag_encoder_motor_on_96m\nON: started 33 steps; capturing all sensors and timers.\nON stage 27/33 | BOTH | PWM=40% | COAST\nON stage 28/33 | BOTH | PWM=50% | DRIVE\nON stage 28/33 | BOTH | PWM=50% | COAST\nON stage 29/33 | BOTH | PWM=60% | DRIVE\nON stage 29/33 | BOTH | PWM=60% | COAST\nON stage 30/33 | BOTH | PWM=70% | DRIVE\nON stage 30/33 | BOTH | PWM=70% | COAST\nON stage 31/33 | BOTH | PWM=80% | DRIVE\nON stage 31/33 | BOTH | PWM=80% | COAST\nON stage 32/33 | BOTH | PWM=90% | DRIVE\nON stage 32/33 | BOTH | PWM=90% | COAST\nON stage 33/33 | BOTH | PWM=100% | DRIVE\nON stage 33/33 | BOTH | PWM=100% | COAST\nON stage 34/33 | BOTH | PWM=100% | DONE\nON UNQUALIFIED steps 33 failed_checks ['i2c_400k_nominal']\nSaved logs and analysis: D:\\Micromouse_STM32F411CEU6_BLE_Debug_Web\\results\\encoder_qual_20261020_022526\\system_uart_hardware\nPaired OFF/ON TIM11 late deltas: 0 -> 0\nPaired OFF/ON gyro stale: 0 -> 0"
  },
  "i2c_rate_failed_phases": {
    "OFF": [],
    "ON": [
      "i2c_400k_nominal"
    ]
  }
}
{
  "status": "REVIEWED",
  "source": "D:\\Micromouse_STM32F411CEU6_BLE_Debug_Web\\results\\encoder_qual_20261020_022526\\system_uart_hardware\\summary.json",
  "stage_count": 66,
  "motor_on_stages": 33,
  "both_wheels_mismatch_mean_pct": 0.44,
  "qualified_electrically": false,
  "note": "Stage-average counts only; not 1ms dynamics or ground-truth RPM."
}
## Encoder OFF/ON high-rate telemetry
{
  "OFF": {
    "status": "PASS",
    "samples": 7779,
    "steady_drive_samples": 2632,
    "wheels": [
      {
        "wheel": "L",
        "steady_drive_samples": 2632,
        "mean_20ms_pps": 0.0,
        "std_20ms_pps": 0.0,
        "mae_pps_vs_1ms_ema": 0.0
      },
      {
        "wheel": "R",
        "steady_drive_samples": 2632,
        "mean_20ms_pps": 0.0,
        "std_20ms_pps": 0.0,
        "mae_pps_vs_1ms_ema": 0.0
      }
    ],
    "i2c_startup_probe": {
      "selected_speed_index": 0,
      "selected_speed_mode": 1,
      "configured_scl_hz": 400000,
      "attempt_400k": {
        "address_status": 0,
        "who_read_status": 0,
        "who_am_i": 104
      },
      "attempt_300k": {
        "address_status": 255,
        "who_read_status": 255,
        "who_am_i": 255
      },
      "fallback_occurred": false,
      "explanation": "Status 255 means unattempted. Probe status 0 indicates driver success; WHO_AM_I must equal 0x68. The configured clock is nominal, not an SCL oscilloscope measurement."
    },
    "health": {
      "tim11_late_delta": 0,
      "tim11_gap_delta": 0,
      "uart_tx_drop_delta": 0,
      "uart_dma_error_delta": 0,
      "mpu_i2c_error_delta": 0,
      "mpu_timeout_delta": 0
    },
    "note": "PWM-to-count response, not an independent proof of encoder accuracy/no missed edges."
  },
  "ON": {
    "status": "PASS",
    "samples": 7781,
    "steady_drive_samples": 2632,
    "wheels": [
      {
        "wheel": "L",
        "steady_drive_samples": 2632,
        "mean_20ms_pps": 3074.89,
        "std_20ms_pps": 2692.07,
        "mae_pps_vs_1ms_ema": 47.75
      },
      {
        "wheel": "R",
        "steady_drive_samples": 2632,
        "mean_20ms_pps": 3091.09,
        "std_20ms_pps": 2728.67,
        "mae_pps_vs_1ms_ema": 49.45
      }
    ],
    "i2c_startup_probe": {
      "selected_speed_index": 1,
      "selected_speed_mode": 2,
      "configured_scl_hz": 296296,
      "attempt_400k": {
        "address_status": 2,
        "who_read_status": 255,
        "who_am_i": 255
      },
      "attempt_300k": {
        "address_status": 0,
        "who_read_status": 0,
        "who_am_i": 104
      },
      "fallback_occurred": true,
      "explanation": "Status 255 means unattempted. Probe status 0 indicates driver success; WHO_AM_I must equal 0x68. The configured clock is nominal, not an SCL oscilloscope measurement."
    },
    "health": {
      "tim11_late_delta": 0,
      "tim11_gap_delta": 0,
      "uart_tx_drop_delta": 0,
      "uart_dma_error_delta": 0,
      "mpu_i2c_error_delta": 0,
      "mpu_timeout_delta": 0
    },
    "note": "PWM-to-count response, not an independent proof of encoder accuracy/no missed edges."
  }
}
## Qualification warnings (kept separate from Encoder measurements)
All 33 OFF/ON motor stages completed, but at least one boot chose an I2C rate other than 400 kHz. Encoder 50-Hz diagnostics and raw logs remain valid for analysis; whole-system 400-kHz acceptance is not met.

## Remaining limitations
- Input capture filter IC1F/IC2F and 10-ms speed estimator remain candidates, NOT auto-applied.
- Encoder diagnostic OFF/ON builds emit SYS_EN ~50 Hz; missing edges require independent reference; synthetic estimator study is not board qualification.
- estimator_metrics.csv and estimator_samples.csv (if present) are SYNTHETIC, NOT measured MCU response.
- 1430 counts per wheel revolution is retained as the user's current engineering approximation.
- Historical stage response is not a calibration of closed-loop speed PID.
