#!/usr/bin/env python3
"""Run real portable four-cell Wall PID C controller with simulated sensor data.
Not a claim of hardware PID acceptance; exercises limits and fault paths.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
HARNESS=r"""
#include "wall_pid_test_core.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

static WP_Input input(void)
{
    WP_Input i;
    memset(&i,0,sizeof(i));
    i.ir.status=IR_WALL_VALID;i.ir.age_us=150U;
    i.ir.ema10_adc[IR_WALL_L0]=460U;
    i.ir.ema10_adc[IR_WALL_R0]=664U;
    return i;
}
static void tick(WP_Controller *p,WP_Input *i)
{
    uint8_t up=WP_Step(p,i);
    if(up) {
        assert(p->state==WP_RUNNING);
        assert(p->pwm_l>=0&&p->pwm_l<=WP_PWM_CAP_PM);
        assert(p->pwm_r>=0&&p->pwm_r<=WP_PWM_CAP_PM);
        assert(p->pwm_l-p->pwm_r<=2*WP_STEER_CAP_PM);
    }
}
int main(void)
{
    WP_Controller p;
    WP_Input i=input();
    assert(WP_CELLS==4);
    assert(WP_CELL_MM==180.0f);
    assert(WP_TARGET_MM==720.0f);
    assert(WP_WHEEL_DIAMETER_MM==34.0f);
    assert(WP_ENCODER_CPR==1430.0f);
    assert(fabsf(WP_MM_PER_COUNT-0.074696f)<0.0001f);
    WP_Init(&p);assert(WP_Start(&p,&i));
    assert(p.state==WP_RUNNING && p.ref_l_adc==460U && p.ref_r_adc==664U);
    i.ir.ema10_adc[IR_WALL_L0]=500U;
    i.ir.ema10_adc[IR_WALL_R0]=544U;
    for(int n=0;n<5;n++)tick(&p,&i);
    assert(p.error_norm>0.05f);
    assert(p.pwm_l>p.pwm_r); /* closer LEFT -> steer RIGHT */
    WP_Stop(&p,WP_PA0_STOP);
    assert(p.state==WP_FAULT&&p.reason==WP_PA0_STOP);
    assert(p.pwm_l==0&&p.pwm_r==0);
    assert(!WP_Start(&p,&i)); /* no rearm on terminal state */

    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    i.ir.ema10_adc[IR_WALL_L0]=420U;
    i.ir.ema10_adc[IR_WALL_R0]=784U;
    for(int n=0;n<5;n++)tick(&p,&i);
    assert(p.error_norm<-0.05f);
    assert(p.pwm_l<p.pwm_r); /* closer RIGHT -> steer LEFT */

    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    i.ir.status=IR_WALL_STALE;
    tick(&p,&i);assert(p.reason==WP_IR_STALE&&p.state==WP_FAULT);

    /* 0..3500 ADC is a legal optical signal range. No intensity
       threshold may trigger IR_EXCURSION or WALL_LOST abort. */
    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    i.ir.ema10_adc[IR_WALL_L0]=3500;
    i.ir.ema10_adc[IR_WALL_R0]=0;
    for(int n=0;n<20;n++)tick(&p,&i);
    assert(p.state==WP_RUNNING);
    assert(p.error_norm>0.5f&&p.error_norm<=1.0f);
    assert(p.pwm_l<=WP_PWM_CAP_PM && p.pwm_r<=WP_PWM_CAP_PM);

    i.ir.ema10_adc[IR_WALL_L0]=0;
    i.ir.ema10_adc[IR_WALL_R0]=3500;
    for(int n=0;n<20;n++)tick(&p,&i);
    assert(p.state==WP_RUNNING);
    assert(p.error_norm< -0.5f&&p.error_norm>=-1.0f);

    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    i.yaw_deg=16.0f;
    tick(&p,&i);assert(p.reason==WP_HEADING);

    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    i.left_count=-40;
    tick(&p,&i);assert(p.reason==WP_ENCODER);

    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    for(int n=0;n<90;n++)tick(&p,&i);
    assert(p.target_speed_mm_s>0.0f);
    assert(p.pid_updates>0);
    /* Regression: the old controller artificially capped speed base
       at 600‰ even with zero wall error because it ALWAYS reserved 200‰
       for steering. New cap is 800-abs(actual_steer). */
    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    for(int n=0;n<430;n++)tick(&p,&i);
    assert(p.state==WP_RUNNING);
    assert(p.pwm_l>600 && p.pwm_r>600);
    assert(p.pwm_l==p.pwm_r); /* error=0 => no yaw correction */
    /* Confirm ISR input snapshot available in the POST-STOP BLE FAULT. */
    i.ir.ema10_adc[IR_WALL_L0]=830;
    i.ir.raw_adc[IR_WALL_L0]=900;
    i.ir.ema10_adc[IR_WALL_R0]=666;
    i.ir.raw_adc[IR_WALL_R0]=671;
    i.ir.age_us=250;
    i.ir.scan_count=12345;
    tick(&p,&i);
    assert(p.state==WP_RUNNING && p.reason==WP_NONE);
    assert(p.last_l0_adc==830 && p.last_r0_adc==666);
    assert(p.last_l0_raw==900 && p.last_r0_raw==671);
    assert(p.last_ir_age_us==250 && p.last_ir_scan==12345);

    WP_Init(&p);i=input();assert(WP_Start(&p,&i));
    for(int n=0;n<90;n++)tick(&p,&i);
    assert(p.target_speed_mm_s>0.0f);
    assert(p.pid_updates>0);
    i.left_count=9640;i.right_count=9640;
    tick(&p,&i);
    assert(p.state==WP_DONE&&p.reason==WP_FINISHED);
    assert(p.distance_mm>=719.0f&&p.distance_mm<=721.0f);
    assert(p.pwm_l==0&&p.pwm_r==0);

    WP_Init(&p);i=input();
    i.ir.ema10_adc[IR_WALL_L0]=12U;
    assert(WP_Start(&p,&i)); /* no arbitrary start ADC gate */
    WP_Init(&p);i=input();
    i.ir.ema10_adc[IR_WALL_L0]=0U;
    i.ir.ema10_adc[IR_WALL_R0]=3500U;
    assert(WP_Start(&p,&i));
    tick(&p,&i);
    assert(p.state==WP_RUNNING);
    i=input();i.ir.status=IR_WALL_WARMUP;
    assert(!WP_Start(&p,&i));
    puts("WALL_PID_4CELL_NO_ADC_THRESHOLD_NATIVE_PASS");
    return 0;
}
"""
def main():
    compiler=shutil.which(os.environ.get("CC","gcc"))
    if not compiler: raise RuntimeError("GCC required")
    with tempfile.TemporaryDirectory(prefix="wall-pid-four-") as td:
        p=Path(td)
        (p/"test.c").write_text(HARNESS,encoding="utf-8")
        out=p/("a.exe" if os.name=="nt" else "a")
        subprocess.run([compiler,"-std=c11","-O2","-Wall","-Wextra",
                        "-Werror","-I",str(ROOT/"include"),
                        str(ROOT/"src/wall_pid_test_core.c"),
                        str(p/"test.c"),"-lm","-o",str(out)],check=True,cwd=ROOT)
        subprocess.run([str(out)],check=True,cwd=ROOT)
if __name__=="__main__":main()
