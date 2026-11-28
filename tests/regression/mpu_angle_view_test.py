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
assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=="_draw_sensor_dashboard" for n in ast.walk(redraw))
render=next(n for n in ast.walk(module) if isinstance(n,ast.FunctionDef) and n.name=="_draw_sensor_dashboard")
assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=="draw_angle" for n in ast.walk(render))

# Pivot radar uses relative angle, not accumulated MPU yaw.
s = AngleSample()
assert s.consume("PIVOT_CSV,ms=100,valid=1,age_us=850,yaw_cd=12500,angle_cd=9000,gz_cd=0,target=90,left=-985,right=991,pwm_l=0,pwm_r=0,ref_l=0,ref_r=0,speed_l=0,speed_r=0")
assert s.valid and s.yaw == 90.0
