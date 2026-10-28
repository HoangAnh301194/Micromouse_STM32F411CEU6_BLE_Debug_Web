#!/usr/bin/env python3
"""Regression for the *real C logger* and Python MW BLE viewer.

Mocks BT TX and motor lock only, not logger implementation. Test records
all 5s/1000 samples, checksum, sequence gaps, ADC 0..3500 and old W parser.
"""
from __future__ import annotations
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]

def load(name):
    path=ROOT/"tools"/(name+".py")
    spec=importlib.util.spec_from_file_location(name,path)
    assert spec and spec.loader
    obj=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj

mw=load("wall_pid_6ir_report")
legacy=load("wall_pid_ble_report")

C_HARNESS=r"""
#include "wall_pid_trace.h"
#include "motor.h"
#include "bt_debug.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
static unsigned char bytes[250000];
static size_t total;
uint8_t Motor_OutputsLocked(void){return 1U;}
uint8_t BT_TrySendData(const uint8_t *data,uint16_t len){
    assert(total+len<sizeof(bytes));
    memcpy(bytes+total,data,len);total+=len;
    return 1U;
}
int main(void){
    WP_Input i={0};
    WP_Controller c={0};
    unsigned long tick;
    FILE *f;
    WPT_Reset();
    assert(WPT_Record_Bench_Not_Used==0); /* replaced by Python */
    for(tick=5;tick<=5000;tick+=5){
        i.left_count=(int32_t)(tick*2);
        i.right_count=(int32_t)(tick*2);
        i.yaw_deg=2.0f;i.gyro_dps=1.25f;
        i.ir.status=IR_WALL_VALID;
        i.ir.age_us=200U;i.ir.scan_count=(uint32_t)(100+tick);
        for(unsigned j=0;j<6;++j)
            i.ir.ema10_adc[j]=(uint16_t)((tick/5)%2 ? 3500 : 0);
        c.error_norm=.2f;c.speed_mm_s=550;c.target_speed_mm_s=900;
        c.pwm_l=650;c.pwm_r=550;c.state=WP_RUNNING;
        WPT_Append((uint32_t)tick,&i,&c);
    }
    assert(WPT_Count()==1000U);
    assert(WPT_Dropped()==0U);
    WPT_BeginReplay();
    for(tick=125;tick<900000&&!WPT_ReplayComplete();tick+=125)
        (void)WPT_ServiceReplay((uint32_t)tick);
    assert(WPT_ReplayComplete());
    f=fopen("capture.txt","wb");assert(f);
    assert(fwrite(bytes,1,total,f)==total);fclose(f);
    return 0;
}
""".replace("assert(WPT_Record_Bench_Not_Used==0); /* replaced by Python */","")

def build_real_c(root:Path)->str:
    gcc=shutil.which("gcc")
    assert gcc,"gcc unavailable: C logger must be compiled in native regression"
    (root/"bt_debug.h").write_text(
        '#include <stdint.h>\nuint8_t BT_TrySendData(const uint8_t*,uint16_t);\n',
        encoding="utf-8")
    (root/"motor.h").write_text(
        '#include <stdint.h>\nuint8_t Motor_OutputsLocked(void);\n',
        encoding="utf-8")
    (root/"test.c").write_text(C_HARNESS,encoding="utf8")
    exe=root/"out"
    subprocess.run([gcc,"-std=c11","-O2","-Wall","-Wextra","-Werror",
        "-DWALL_PID_TEST_MODE=1","-I",str(root),"-I",str(ROOT/"include"),
        str(ROOT/"src/wall_pid_trace.c"),str(root/"test.c"),"-o",str(exe)],
        check=True,cwd=root)
    subprocess.run([str(exe)],check=True,cwd=root)
    return (root/"capture.txt").read_text(encoding="ascii")

def main():
    with tempfile.TemporaryDirectory(prefix="wall_full6_") as td:
        root=Path(td)
        raw=build_real_c(root)
        rows,qc=mw.parse_motor_lines(raw.splitlines())
        assert qc["complete"],qc
        assert len(rows)==1000, len(rows)
        assert rows[0]["ms"]==5 and rows[-1]["ms"]==5000
        assert rows[-1]["L0"] in (0,3500)
        assert rows[-1]["pwm_l_permille"]==650
        assert rows[-1]["pwm_r_permille"]==550
        assert qc["max_speed_mm_s"]==550
        assert qc["valid_ir_frames"]==1000
        assert qc["last_distance_mm"]>700
        assert "<canvas" in mw.build_dashboard(rows,qc)
        # Match old W trace valid with real motor-on MW trace in same export.
        old=[
          "W,START,mm=720,diam=34,cpr=1430,pwmcap=800,error=REL_NORM,l=500,r=600",
          "W,END,DONE,ms=5000,d10=7400,avgen1000=200,peakn1000=200,vpk=550,pkpwm=650,locked=1,trace=2",
          "W,50,70,500,600,100,100,130,250,240",
          "W,POST,dist10=7415,drift10=15,locked=1",
          "W,100,140,600,400,200,200,300,400,390",
          "W,TRACE_END",
        ]
        mixed="\n".join(old)+"\n"+raw
        _,oldqc=legacy.parse_lines(mixed.splitlines())
        assert oldqc["data_complete"],oldqc
        assert oldqc["n_trace_rows"]==2
        mixedfile=root/"mixed.txt";mixedfile.write_text(mixed,encoding="utf8")
        output=root/"report"
        report=mw.create_report(mixedfile,output)
        assert report["combined_complete"],report
        assert (output/"full_trace_200hz.csv").is_file()
        assert (output/"viewer.html").is_file()
        assert "speed_mm_s" in (output/"viewer.html").read_text()
        # Dropped one data line corrupts BEGIN/END counts and FNV checksum.
        lines=raw.splitlines()
        damaged="\n".join(lines[:31]+lines[32:])+"\n"
        _,bad=mw.parse_motor_lines(damaged.splitlines())
        assert not bad["complete"]
        assert any("COUNT_MISMATCH" in reason for reason in bad["faults"])
        assert "FNV32_MISMATCH" in bad["faults"]
        altered=raw.replace(",3500,3500,3500,3500,3500,3500,",
                            ",3500,3500,3500,3501,3500,3500,",1)
        _,tamper=mw.parse_motor_lines(altered.splitlines())
        assert not tamper["complete"]
        assert "FNV32_MISMATCH" in tamper["faults"]
        no_end="\n".join(lines[:-1])
        _,missing=mw.parse_motor_lines(no_end.splitlines())
        assert "MISSING_END" in missing["faults"]
        # Stale IR not clipped by ADC; invalid samples captured with valid=0.
        strange=raw.replace(",200,1,3500",",200,0,3500",1)
        _,changed=mw.parse_motor_lines(strange.splitlines())
        assert "FNV32_MISMATCH" in changed["faults"]
    print("WALL_PID_6IR_FULLRATE_C_AND_BLE_PASS")

if __name__=="__main__":main()
