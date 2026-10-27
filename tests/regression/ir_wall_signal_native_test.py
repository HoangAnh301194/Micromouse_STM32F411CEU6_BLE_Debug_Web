#!/usr/bin/env python3
"""Compile the production EMA10 handover adapter, test it against the
already-validated experimental IR_FilterStep EMA10 bank, and probe faults.
This is synthetic verification, never physical Wall-PID acceptance.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HARNESS = r"""
#include "ir_wall_signal.h"
#include "ir_filter.h"
#include <assert.h>
#include <stdio.h>

static void test_nominal(void)
{
    IR_WallSignal wall;
    IR_WallSnapshot s;
    IR_Filter_t reference;
    IR_FilterFrame_t frame;
    uint16_t raw[6] = {20, 180, 500, 750, 150, 25};
    uint32_t count = 1U, us = 1000U;
    IR_WallSignal_Init(&wall);
    IR_FilterReset(&reference);
    assert(IR_WallSignal_Read(&wall, 1100U, &s) == IR_WALL_NO_SAMPLE);
    for (unsigned step=0U; step < 120U; ++step) {
        raw[2]=(uint16_t)(500U+step*2U);
        raw[3]=(uint16_t)(750U-step*3U);
        if (step==54U) raw[2]=3000U; /* spike */
        if (step==55U) raw[3]=4095U; /* valid ADC rail */
        assert(IR_WallSignal_Update(&wall,raw,count,us));
        IR_FilterStep(&reference,raw,&frame);
        assert(IR_WallSignal_Read(&wall,us+250U,&s)==
               (step<31U ? IR_WALL_WARMUP : IR_WALL_VALID));
        for (unsigned i=0U; i<6U; ++i) {
            assert(s.raw_adc[i]==raw[i]);
            assert(s.ema10_adc[i]==frame.ema10[i]);
        }
        assert(s.scan_count==count && s.completed_us==us);
        assert(s.age_us==250U);
        count++;us+=826U;
    }
    assert(s.warmup_count==IR_WALL_WARMUP_SAMPLES);
    assert(s.skipped_scans==0U && s.reset_count==0U);
    assert(IR_WallSignal_Read(&wall,wall.completed_us+3001U,&s)==IR_WALL_STALE);
    assert(IR_WallSignal_Read(&wall,wall.completed_us+3000U,&s)==IR_WALL_VALID);
}

static void test_invalid_and_recovery(void)
{
    IR_WallSignal wall;
    IR_WallSnapshot s;
    uint16_t raw[6] = {100,200,300,400,500,600};
    IR_WallSignal_Init(&wall);
    assert(!IR_WallSignal_Update(&wall,raw,0U,1000U));
    assert(wall.rejected_frames==1U);
    assert(IR_WallSignal_Update(&wall,raw,1U,1000U));
    raw[4]=6000U;
    assert(!IR_WallSignal_Update(&wall,raw,2U,1830U));
    assert(wall.scan_count==1U && wall.raw[4]==500U);
    raw[4]=500U;
    assert(!IR_WallSignal_Update(&wall,raw,1U,1800U)); /* duplicate */
    assert(IR_WallSignal_Update(&wall,raw,3U,2660U));
    assert(wall.reset_count==1U && wall.skipped_scans==1U);
    assert(wall.warmup_count==1U);
    assert(IR_WallSignal_Read(&wall,2700U,&s)==IR_WALL_WARMUP);
    assert(s.rejected_frames==3U);
    assert(IR_WallSignal_Update(&wall,raw,4U,12000U)); /* gap */
    assert(wall.reset_count==2U && wall.warmup_count==1U);
    assert(wall.skipped_scans==1U);
    assert(!IR_WallSignal_Update(&wall,raw,5U,11999U)); /* backwards time */
}

static void test_scan_wrap_and_no_sample(void)
{
    IR_WallSignal wall;
    IR_WallSnapshot s;
    uint16_t raw[6] = {0,0,0,0,0,0};
    IR_WallSignal_Init(&wall);
    assert(IR_WallSignal_Update(&wall,raw,0xFFFFFFFFUL,0xFFFFFF00UL));
    assert(IR_WallSignal_Update(&wall,raw,0U,0x0000023AUL));
    assert(wall.skipped_scans==0U && wall.reset_count==0U);
    assert(IR_WallSignal_Read(&wall,0x00000300UL,&s)==IR_WALL_WARMUP);
    assert(s.scan_count==0U);
    assert(IR_WallSignal_Read(&wall,0x00000300UL,0)==IR_WALL_NO_SAMPLE);
}

int main(void)
{
    test_nominal();
    test_invalid_and_recovery();
    test_scan_wrap_and_no_sample();
    puts("IR_WALL_EMA010_NATIVE_PASS");
    return 0;
}
"""

def main():
    compiler = shutil.which(os.environ.get("CC", "gcc"))
    if not compiler:
        raise SystemExit("GCC missing")
    with tempfile.TemporaryDirectory(prefix="ir-wall-ema10-") as td:
        p = Path(td)
        (p/"test.c").write_text(HARNESS,encoding="utf8")
        binary=p/("test.exe" if os.name=="nt" else "test")
        cmd=[compiler,"-std=c11","-O2","-Wall","-Wextra","-Werror",
             "-I",str(ROOT/"include"),
             str(ROOT/"src/ir_wall_signal.c"),
             str(ROOT/"src/ir_filter.c"),
             str(p/"test.c"),"-o",str(binary)]
        subprocess.run(cmd,cwd=ROOT,check=True,timeout=30)
        subprocess.run([str(binary)],cwd=ROOT,check=True,timeout=30)

if __name__=="__main__":
    main()
