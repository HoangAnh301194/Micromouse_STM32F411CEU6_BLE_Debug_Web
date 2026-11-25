"""No-motor OneBox audit regression."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from onebox_audit import audit

def main():
    rows=[
      dict(t_ms="0",REF_L_PPS="1100",REF_R_PPS="1100",SPEED_L_PPS="1000",SPEED_R_PPS="900",PWM_L="200",PWM_R="180",REL_L="0",REL_R="0"),
      dict(t_ms="20",REF_L_PPS="1100",REF_R_PPS="1100",SPEED_L_PPS="1050",SPEED_R_PPS="1000",PWM_L="190",PWM_R="200",REL_L="100",REL_R="98"),
      dict(t_ms="60",REF_L_PPS="0",REF_R_PPS="0",SPEED_L_PPS="0",SPEED_R_PPS="0",PWM_L="0",PWM_R="0",REL_L="200",REL_R="195"),
    ]
    s=audit(rows,wheelbase_mm=74)
    assert s["samples"]==3 and s["gaps_over_30ms"]==1
    assert s["wheel_pid"]["L"]["valid_samples"]==2
    assert s["wheel_pid"]["L"]["mae_pps"]==75
    assert s["wheel_pid"]["L"]["saturation_percent"]==round(100/3,2)
    assert s["heading_shadow"]["yaw_proxy_deg"]<0
    assert audit(rows)["heading_shadow"] is None
    assert audit([dict(t_ms="0")])["wheel_pid"]["L"]["mae_pps"] is None
    try:
        audit(rows,wheelbase_mm=0)
        raise AssertionError("invalid wheelbase accepted")
    except ValueError:
        pass
    print("test_onebox_audit: PASS")

if __name__=="__main__":
    main()
