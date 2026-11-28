from pathlib import Path
import sys,math
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from wheel_plot_data import WheelPlotData
from spp_dual_log import sensor_test_csv
w=WheelPlotData()
f="ONEBOX_CSV,ms=0,left=0,right=0,pwm_l=0,pwm_r=0,ref_l=0,ref_r=0,speed_l=0,speed_r=0,dl=0,dr=0,sync=0,remain=2414,phase=1,heading=1,valid=1,age_us=100,angle_cd=0,gz_cd=0"
assert sensor_test_csv(f)[0]=="HEADING"
assert w.add(f,"2026-11-28T12:00:00")
assert w.add(f.replace("ms=0,","ms=30,").replace("left=0,right=0","left=100,right=100"),"2026-11-28T12:00:00")
assert math.isclose(w.rows[-1]["x_mm"],100*165/2414)
assert w.rows[-1]["y_mm"]==0
w.clear()
p="PIVOT_CSV,ms=0,valid=1,age_us=100,yaw_cd=0,angle_cd=0,gz_cd=0,target=90,left=0,right=0,pwm_l=0,pwm_r=0,ref_l=0,ref_r=0,speed_l=0,speed_r=0"
assert w.add(p,"2026-11-28T12:00:00")
assert w.add(p.replace("ms=0,","ms=30,").replace("left=0,right=0","left=-100,right=100").replace("angle_cd=0","angle_cd=9000"),"2026-11-28T12:00:00")
assert w.rows[-1]["drift_mm"]==0
print("motion_analysis_test: PASS")
