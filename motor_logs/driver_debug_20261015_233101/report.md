# Motor Driver debug checklist

UTC: 2026-10-15T16:31:59.809429+00:00
Overall: **REVIEW_PENDING_EVIDENCE**

| Gate | Status | Detail |
| --- | --- | --- |
| native_driver_watchdog | PASS | native_driver_watchdog.log |
| probe_parser | PASS | probe_parser.log |
| metrics_pipeline_tests | PASS | metrics_pipeline_tests.log |
| offline_source_model | PASS | 9/9 offline gates |
| pin_waveforms | NOT_MEASURED |  |
| build_motor_pwm_key_bench_96m_trim25 | PASS | build_motor_pwm_key_bench_96m_trim25.log |
| build_motor_auto_pipeline_96m_trim25 | PASS | build_motor_auto_pipeline_96m_trim25.log |
| build_motor_auto_high_96m_trim25 | PASS | build_motor_auto_high_96m_trim25.log |
| build_motor_auto_sensor_96m_trim25 | PASS | build_motor_auto_sensor_96m_trim25.log |
| build_motor_auto_sensor_high_96m_trim25 | PASS | build_motor_auto_sensor_high_96m_trim25.log |
| locked_idle_uart | PASS | {'status': 'PASS_LOCKED_IDLE', 'mean_cycles': 108.03125, 'error': None, 'snapshot_count': 12} |
| tim11_raw_trace | NOT_MEASURED | Live capture requires physically disconnected VM confirmation |
| motor_log_integrity | PASS | 2 saved captures; image identity unverified |
| startup_deadband | MEASURED_SAVED_LOG | Startup heuristics only; no automatic deadband assignment |
| holding_deadband | NOT_MEASURED | Existing pulse protocol has no descending-duty hold sweep |
| repeatability | LOW_REPEATS | 52 groups; minimum 10 distinct trials per group |
| steady_speed | INSUFFICIENT_PLATEAU | {'INSUFFICIENT_PLATEAU': 156} |
| rise_slew | MEASURED_SAVED_LOG |  |
| fall_slew | NOT_MEASURED | Fall-rate alternatives are not advertised in current pulse protocol |
| software_stop_tail | MEASURED_SAVED_LOG | Neutral register checks, net and absolute observed stop-tail counts |
| live_watchdog | NOT_MEASURED | Actual C watchdog tested on host; powered heartbeat-loss trace not captured |
| single_dual_wheel | MEASURED_SAVED_LOG |  |
| energized_brake_reverse_stall | NOT_AUTOMATED |  |
| pwm_frequency_comparison | NOT_MEASURED |  |

Encoder polarity: owner confirmed; not retested.
Logic-level compatibility: excluded by owner; not a gate.
Current, temperature and supply droop: NOT_MEASURED.
No upload, START, brake, reverse, or P0-unlock action is implemented.
Saved motor logs do not verify the currently flashed image or electrical safety.
Missing evidence is review, not proof that the driver is fully optimized.
