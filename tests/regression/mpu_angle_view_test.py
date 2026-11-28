from pathlib import Path
import sys
import math
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from mpu_angle_view import AngleSample, heading_point

for a, expected in [(0,(0,-1)),(90,(-1,0)),(-90,(1,0)),(180,(0,1)),(360,(0,-1)),(450,(-1,0))]:
    assert all(math.isclose(x,y,abs_tol=1e-9) for x,y in zip(heading_point(a),expected))
s=AngleSample()
assert s.consume("MPU,ms=50,valid=1,age_us=100,yaw_cd=-9025,gz_cd=0")
assert s.valid and s.yaw == -90.25 and s.age == 100
assert not s.consume("ENC,1,2") and s.yaw == -90.25
assert s.consume("MPU,ms=100,valid=0,age_us=0") and not s.valid and s.yaw is None
assert s.consume("MPU,ms=150,valid=1,age_us=4000,yaw_cd=9000") and not s.valid
s.clear()
assert s.yaw is None and not s.valid
print("mpu_angle_view_test: PASS")

# Ensure the radar is actually rendered by the recurring Tk redraw callback.
# This catches the previous placement in __init__ rather than redraw().
import ast
module=ast.parse((Path(__file__).resolve().parents[2]/"tools"/"classic_spp_dashboard.py").read_text(encoding="utf-8"))
redraw=next(n for n in ast.walk(module) if isinstance(n,ast.FunctionDef) and n.name=="redraw")
assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=="_draw_angle_panel" for n in ast.walk(redraw))
render=next(n for n in ast.walk(module) if isinstance(n,ast.FunctionDef) and n.name=="_draw_angle_panel")
assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=="draw_angle" for n in ast.walk(render))

# Pivot radar uses relative angle, not accumulated MPU yaw.
s = AngleSample()
assert s.consume("PIVOT_CSV,ms=100,valid=1,age_us=850,yaw_cd=12500,angle_cd=9000,gz_cd=0,target=90,left=-985,right=991,pwm_l=0,pwm_r=0,ref_l=0,ref_r=0,speed_l=0,speed_r=0")
assert s.valid and s.yaw == 90.0

from wheel_plot_data import WheelPlotData
w=WheelPlotData()
frame="PIVOT_CSV,ms=100,valid=1,age_us=850,yaw_cd=12500,angle_cd=9000,gz_cd=0,target=90,left=-985,right=991,pwm_l=-100,pwm_r=100,ref_l=-1100,ref_r=1100,speed_l=-1050,speed_r=1000"
assert w.add(frame, "2026-11-28T12:00:00")
assert w.rows[-1]["ref_L"]==-1100 and w.rows[-1]["error_L"]==-50
assert w.add(frame.replace("ms=100,","ms=120,").replace("left=-985","left=-995"), "2026-11-28T12:00:00")
assert abs(w.rows[-1]["t"]-.02)<1e-6 and w.rows[-1]["count_L"]==-10
assert w.add_live(dict(left=1,right=2,speed_l=-4,speed_r=5,pwm_l=0,pwm_r=0), 10.)
assert w.kind=="LIVE" and w.rows[-1]["speed_L"]==-4
runtime=(Path(__file__).resolve().parents[2]/"src"/"encoder_onebox.c").read_text()
assert "pivot_angle += RelativeYaw(snap.yaw_deg, pivot_previous_yaw)" in runtime
assert "RelativeYaw(snap.yaw_deg,pivot_origin)" not in runtime

# Pure pivot: no IR capability and no unrelated dashboard panel.
resources=(Path(__file__).resolve().parents[2]/"src"/"mode_test_resources.c").read_text()
main=(Path(__file__).resolve().parents[2]/"src"/"main_runtime.c").read_text()
assert "if (ModeMenu_PivotTarget(menu)) return TEST_RES_MPU | TEST_RES_ENCODER | TEST_RES_MOTOR;" in resources
assert "test_resource_mask == (TEST_RES_MPU|TEST_RES_ENCODER|TEST_RES_MOTOR)" in main
dashboard=(Path(__file__).resolve().parents[2]/"tools"/"classic_spp_dashboard.py").read_text()
assert '("MPU","ENC") if name.startswith("Pivot ") else ()' in dashboard
assert "self.sensor_dashboard.pack_forget()" in dashboard
