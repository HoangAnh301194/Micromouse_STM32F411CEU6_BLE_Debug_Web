from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from wheel_speed_metrics import summarize_wheels
rows=[{"t":i*.02,"ref_L":1100,"ref_R":1100,"speed_L":1050,"speed_R":1100,"pwm_L":200,"pwm_R":180} for i in range(20)]
m=summarize_wheels(rows)
assert m["L"]["mae_pps"]==50 and m["R"]["rmse_pps"]==0
assert m["L"]["pwm_sat_pct"]==100 and m["lr_mean_abs_diff_pps"]==50
assert m["telemetry_hz"]==50
assert summarize_wheels([])["L"]["verdict"]=="NO_DATA"
print("wheel_speed_metrics_test: PASS")
