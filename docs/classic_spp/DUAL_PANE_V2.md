# Classic SPP two-pane dashboard (giomuadongbacv2)

Run `py tools/classic_spp_dashboard.py --port COM16`.
The left pane shows MPU / IR / ENC / ENC_CSV / CSV / DATA streams
and historical numeric W traces. The right pane shows SPP status,
Mode 1-7 navigation, confirmation events, and errors.

Per-connection files are created automatically under
`logs/spp_runs/connections/` with `_status.log` and `_data.log` suffixes.
Existing optional manual Start log remains available for mixed logs.
Replay only changes the displayed panes, not live file recordings.

Firmware is unchanged. Mode 7 System Test does NOT yet implement
an encoder sub-mode or real-time encoder CSV streaming.
Future firmware ENC / ENC_CSV messages are routed left automatically.

Run regression: `py tests/regression/spp_dual_log_test.py`
Syntax check: `py -m py_compile tools/classic_spp_dashboard.py tools/spp_dual_log.py`
