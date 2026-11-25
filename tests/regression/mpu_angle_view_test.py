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
