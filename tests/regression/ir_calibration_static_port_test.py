"""Source-level regression for optional v2 static calibration port.

This does not replace a PlatformIO compile or a physical measurement.
"""
from pathlib import Path
root=Path(__file__).resolve().parents[2]
runtime=(root/"src/main_runtime.c").read_text(encoding="utf-8")
firmware=(root/"src/ir_calibration_static.c").read_text(encoding="utf-8")
dashboard=(root/"tools/classic_spp_dashboard.py").read_text(encoding="utf-8")
assert 'command == \'C\'' in runtime
assert 'IRStatic_Start();' in runtime
assert 'IRStatic_Capture(now);' in runtime
assert 'IRStatic_Poll(now);' in runtime
assert 'if(IRStatic_Active())' in runtime
assert 'else if (!IRC_Save(&ir_calibration))' in runtime
assert 'SC_N 200U' in firmware
assert 'SC_MIN_SPAN 50U' in firmware
assert 'if(!active || !review || busy || !Motor_OutputsLocked()' in firmware
assert 'quality=STATIC_CANDIDATE' in firmware
assert 'b"V", b"C"' in dashboard
assert 'self.static_capture_btn' in dashboard
print("ir_calibration_static_port_test PASS (source-level checks)")
