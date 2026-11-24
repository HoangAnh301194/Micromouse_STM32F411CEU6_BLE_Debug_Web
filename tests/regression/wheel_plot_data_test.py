from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from wheel_plot_data import WheelPlotData
d=WheelPlotData()
t="2026-11-24T16:00:00"
assert d.add("ONEBOX_CSV,ms=50,left=10,right=12,pwm_l=145,pwm_r=146,ref_l=1100,ref_r=1100,speed_l=900,speed_r=1000",t)
assert d.rows[-1]["t"]==0.05 and d.rows[-1]["error_L"]==200
assert d.rows[-1]["pwm_R"]==146
assert d.add("ONEBOX_CSV,ms=100,left=20,right=22,pwm_l=140,pwm_r=140",t)
assert "ref_L" not in d.rows[-1] and d.rows[-1]["count_L"]==10
assert d.add("ENC,100,200",t)
assert len(d.rows)==1 and "pwm_L" not in d.rows[-1]
assert d.add("ENC,101,203","2026-11-24T16:00:00.050")
assert abs(d.rows[-1]["t"]-0.05)<0.001 and d.rows[-1]["count_R"]==3
assert not d.add("IR,valid=0",t)
print("wheel_plot_data_test: PASS")
