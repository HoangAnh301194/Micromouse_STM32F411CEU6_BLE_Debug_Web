"""Native host C regression: one-cell count controller and independent safety cutoffs."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CC = os.environ.get("CC", "gcc")
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include "encoder_onebox_core.h"

static void CheckTargets(void) {
    OneBoxCore s;
    OneBoxOutput o;
    int i;
    OneBoxCore_Start(&s, 123, 456);
    assert(s.running && !s.ticks);
    for (i=0; i<100; ++i) {
        if (i%40==0) OneBoxCore_HostPing(&s);
        o=OneBoxCore_Step(&s,123 + i*24,456+i*24);
        assert(o.reason == ONEBOX_REASON_NONE);
        assert(o.pwm_l>=0 && o.pwm_l<=ONEBOX_MAX_PWM);
        assert(o.pwm_r>=0 && o.pwm_r<=ONEBOX_MAX_PWM);
    }
    o=OneBoxCore_Step(&s,123+ONEBOX_TARGET_L,
                       456+ONEBOX_TARGET_R);
    assert(o.reason==ONEBOX_REASON_DONE);
    assert(!s.running);
    assert(s.delta_l==ONEBOX_TARGET_L && s.delta_r==ONEBOX_TARGET_R);
    assert(o.pwm_l==0 && o.pwm_r==0);
}
static void CheckHostLoss(void) {
    OneBoxCore s; OneBoxOutput o={0,0,0}; uint32_t i;
    OneBoxCore_Start(&s,0,0);
    for (i=0;i<ONEBOX_LEASE_TICKS;i++) {
        o=OneBoxCore_Step(&s,i/2,i/2);
    }
    assert(o.reason==ONEBOX_REASON_HOST_LOST);
    assert(!s.running);
}
static void CheckReverse(void) {
    OneBoxCore s; OneBoxOutput o;
    OneBoxCore_Start(&s,200,300);
    OneBoxCore_HostPing(&s);
    o=OneBoxCore_Step(&s,150,301);
    assert(o.reason==ONEBOX_REASON_ENCODER);
}
static void CheckDivergence(void) {
    OneBoxCore s; OneBoxOutput o;
    OneBoxCore_Start(&s,0,0);
    o=OneBoxCore_Step(&s,800,400);
    assert(o.reason==ONEBOX_REASON_ENCODER);
}
static void CheckStall(void) {
    OneBoxCore s; OneBoxOutput o={0,0,0}; uint32_t i;
    OneBoxCore_Start(&s,0,0);
    for (i=0;i<ONEBOX_STALL_TICKS;i++) {
        if (i%80==0) OneBoxCore_HostPing(&s);
        o=OneBoxCore_Step(&s,0,0);
    }
    assert(o.reason==ONEBOX_REASON_STALL);
}
static void CheckTimeout(void) {
    OneBoxCore s; OneBoxOutput o={0,0,0}; uint32_t i;
    OneBoxCore_Start(&s,0,0);
    for (i=0;i<ONEBOX_MAX_TICKS;i++) {
        if (i%80U==0U) OneBoxCore_HostPing(&s);
        /* Move slowly enough to remain below target, but no stall. */
        o=OneBoxCore_Step(&s,(int32_t)(i/4U),(int32_t)(i/4U));
    }
    assert(o.reason==ONEBOX_REASON_TIMEOUT);
    assert(!s.running);
}
static void CheckExternalStop(void) {
    OneBoxCore s;
    OneBoxCore_Start(&s,0,0);
    OneBoxCore_Stop(&s,ONEBOX_REASON_USER_STOP);
    assert(!s.running && s.reason==ONEBOX_REASON_USER_STOP);
}
static void CheckSpeedPid(void) {
    OneBoxGains l={500,800,0},r={200,0,0},bad={10001,0,0};
    OneBoxSpeedPid a,b;
    OneBoxSpeedPid_Reset(&a);OneBoxSpeedPid_Reset(&b);
    assert(OneBoxGains_Valid(&l) && !OneBoxGains_Valid(&bad));
    assert(OneBoxSpeedPid_Step(&a,&l,1100,1000)>OneBoxSpeedPid_Step(&b,&r,1100,1000));
    assert(OneBoxSpeedPid_Step(&a,&l,1100,0)==ONEBOX_MAX_PWM);
    OneBoxSpeedPid_Reset(&a);
    assert(OneBoxSpeedPid_Step(&a,&l,1100,1100)==188);
    assert(ONEBOX_DEFAULT_P==900U && ONEBOX_DEFAULT_I==2500U);
    assert(ONEBOX_PID_DT_S==0.001f);
    assert(OneBoxSpeedPid_Step(&a,&l,0,1000)==0 && !a.initialized);
}
static void CheckSharedArrival(void) {
    OneBoxCore s; OneBoxOutput o; float l,r;
    OneBoxCore_Start(&s,0,0);
    o=OneBoxCore_Step(&s,ONEBOX_TARGET_L,ONEBOX_TARGET_R-50);
    assert(o.reason==ONEBOX_REASON_NONE && o.pwm_l>0 && o.pwm_r>0);
    o=OneBoxCore_Step(&s,ONEBOX_TARGET_L+25,ONEBOX_TARGET_R-25);
    assert(o.reason==ONEBOX_REASON_DONE && o.pwm_l==0 && o.pwm_r==0);
    MotionConfig good={850,220,2200,450,300,220,4000,200};
    assert(MotionConfig_Valid(&good));
    good.cruise=1200; assert(!MotionConfig_Valid(&good));
    Motion_Sync(600,100,&l,&r); assert(l<r && r==600 && l>0);
    Motion_Sync(600,-100,&l,&r); assert(r<l && l==600 && r>0);
    Motion_SyncPD(600,0,100,0,&l,&r); assert(l<r && r==600);
    Motion_SyncPD(600,0,0,-100,&l,&r); assert(r<l && l==600);
    OneBoxSpeedPid a,b; OneBoxGains g={0,0,0};
    OneBoxSpeedPid_Reset(&a); OneBoxSpeedPid_Reset(&b);
    assert(OneBoxSpeedPid_StepFF(&a,&g,&motion_ff_l,850,850)>
           OneBoxSpeedPid_StepFF(&b,&g,&motion_ff_r,850,850));
    assert(Motion_Base(0,850,850)>0 && Motion_Base(0,850,850)<10);
    assert(Motion_DistanceDesired(10)<Motion_DistanceDesired(450));
    assert(Motion_AngleDesired(5,80)<Motion_AngleDesired(40,0));
}
int main(void) {
    CheckSharedArrival();
    CheckSpeedPid();
    assert(ONEBOX_TARGET_L==2414L && ONEBOX_TARGET_R==2414L);
    CheckTargets();CheckHostLoss();CheckReverse();
    CheckDivergence();CheckStall();CheckTimeout();CheckExternalStop();
    puts("encoder_onebox_native_test: PASS");
    return 0;
}
'''

def main():
    if not shutil.which(CC):
        raise SystemExit(f"No native C compiler: {CC}")
    with tempfile.TemporaryDirectory(prefix="onebox-core-") as d:
        src = Path(d) / "test.c"
        exe = Path(d) / ("test.exe" if os.name=="nt" else "test")
        src.write_text(HARNESS,encoding="utf-8")
        subprocess.run([CC, "-std=c99", "-Wall", "-Wextra", "-Werror",
                        "-I", str(ROOT / "include"),
                        str(ROOT / "src/encoder_onebox_core.c"),
                        str(src), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

if __name__ == "__main__":
    main()
