#!/usr/bin/env python3
"""Offline fabricated BLE terminal capture tests; no radio/hardware claims."""
import importlib.util
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
p=ROOT/"tools/wall_pid_ble_report.py"
s=importlib.util.spec_from_file_location("wall_ble",p)
m=importlib.util.module_from_spec(s)
s.loader.exec_module(m)

def main():
    good=[
        "12:00 W,START,mm=720,diam=34,cpr=1430,pwmcap=800,l=460,r=664",
        "12:01 W,END,DONE,ms=100,d10=7202,avgerr100=30,peakerr100=60,vpk=900,pkpwm=550,locked=1,trace=2",
        "12:02 W,50,100,460,664,20,800,900,400,430",
        "12:03 W,POST,dist10=7211,drift10=9,locked=1",
        "12:04 W,100,7202,462,665,-25,790,20,200,210",
        "12:05 W,TRACE_END",
    ]
    rows,rep=m.parse_lines(good)
    assert rep["data_complete"],rep
    assert rep["n_trace_rows"]==2
    assert rep["poststop_distance_mm"]==721.1
    assert rep["drift_mm"]==0.9
    assert abs(rep["mae_relative_wall_mm"]-0.225)<0.0001
    assert rows[0]["L0_ema_adc"]==460
    _,rep=m.parse_lines(good[:-1])
    assert not rep["data_complete"]
    assert "missing TRACE_END" in rep["invalid_reasons"]
    _,rep=m.parse_lines(good[:3]+good[3:5]+[good[-1]])
    assert rep["data_complete"]
    bad=good[:]
    bad.pop(4)
    _,rep=m.parse_lines(bad)
    assert not rep["data_complete"]
    assert any("expected=2" in x for x in rep["invalid_reasons"])
    bad=good[:]
    bad[1]=bad[1].replace("locked=1","locked=0")
    _,rep=m.parse_lines(bad)
    assert not rep["data_complete"]
    fault_line="W,FAULT,why=IR_EXCURSION,ms=440,l=835,r=750,rawL=1050,rawR=740,refL=669,refR=750,age=400,scan=10022"
    _,fr=m.parse_lines(good+[fault_line])
    assert fr["fault"]["l"]==835 and fr["fault"]["rawL"]==1050
    assert fr["fault_side"]=="L0"
    right_fault=fault_line.replace("l=835,r=750","l=669,r=1261").replace("rawL=1050","rawL=650")
    _,fr=m.parse_lines(good+[right_fault])
    assert fr["fault_side"]=="R0"
    print("WALL_PID_BLE_REPORT_TEST_PASS")

if __name__=="__main__":main()
