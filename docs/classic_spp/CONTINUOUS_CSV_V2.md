# Continuous SPP telemetry in CSV (giomuadongbacv2)

Left Data pane now displays comma-separated rows with one common header:

`host_time,type,ms,valid,age_us,yaw_cd,gz_cd,L0,L45,L90,R0,R45,R90,left,right,pps_l,pps_r,raw`

Automatic file: `logs/spp_runs/connections/*_data.csv` (Excel/pandas compatible).
The right Status pane remains human-readable; status is saved as `*_status.log`.
Missing sensor columns are intentionally empty. `raw` preserves the full UART frame,
including unknown fields; embedded commas are CSV-escaped. UI filters do not filter disk logs.

Supported encoder format: `ENC,ms=1100,left=12,right=13,pps_l=120,pps_r=125` or
`ENC_CSV,1100,12,13,120,125`. The current v2 menu firmware does not yet send encoder streams.

Firmware, KEY, and OLED are unchanged.

Test: `py tests/regression/spp_dual_log_test.py`.
Syntax: `py -m py_compile tools/classic_spp_dashboard.py tools/spp_dual_log.py`.
