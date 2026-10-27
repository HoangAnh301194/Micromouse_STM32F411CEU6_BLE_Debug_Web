#!/usr/bin/env python3
"""Host-only regression of src/motion_primitives.c (not physical qualification)."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

HARNESS = r"""
#include "motion_primitives.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void update_n(MotionPrimitiveController *c, MotionPrimitiveOutput *o,
                     unsigned n, float yaw, float rate) {
    for(unsigned i=0;i<n;i++)
        MotionPrimitive_Update(c,0.001f,yaw,rate,o);
}

static void test_selection_and_direction(void) {
    MotionPrimitiveController c;
    MotionPrimitiveOutput o;
    MotionPrimitive_Init(&c);
    assert(c.status==MOTION_PRIMITIVE_IDLE);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_SLALOM_90,MOTION_DIR_LEFT,0)==
           MOTION_PRIMITIVE_UNSUPPORTED);
    assert(c.status==MOTION_PRIMITIVE_IDLE);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_90,MOTION_DIR_LEFT,0)==
           MOTION_PRIMITIVE_OK);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_180,MOTION_DIR_LEFT,0)==
           MOTION_PRIMITIVE_BUSY);
    update_n(&c,&o,5,0,0);
    assert(o.status==MOTION_PRIMITIVE_RUNNING);
    assert(o.right_pm>0 && o.left_pm<0);
    assert(o.right_pm<=900 && o.immediate_neutral==0);
    MotionPrimitive_Abort(&c,MOTION_PRIMITIVE_STOP_REQUEST);
    update_n(&c,&o,1,0,0);
    assert(o.status==MOTION_PRIMITIVE_FAULT && o.immediate_neutral==1);
    assert(o.left_pm==0 && o.right_pm==0);

    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_90,MOTION_DIR_RIGHT,0)==
           MOTION_PRIMITIVE_OK);
    update_n(&c,&o,5,0,0);
    assert(o.right_pm<0 && o.left_pm>0); /* both wheels reverse */
    assert(o.left_pm<=900);
}

static void test_angle_wrap_and_stopping(void) {
    MotionPrimitiveController c;
    MotionPrimitiveOutput o;
    MotionPrimitive_Init(&c);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_180,
                                 MOTION_DIR_LEFT,170)==MOTION_PRIMITIVE_OK);
    update_n(&c,&o,1,-179,100);
    assert(fabsf(c.traveled_deg-11.0f)<0.001f); /* crossed +180 wrap */
    update_n(&c,&o,1,169,100);
    assert(fabsf(c.traveled_deg+1.0f)<0.001f);
    MotionPrimitive_Abort(&c,MOTION_PRIMITIVE_STOP_REQUEST);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_90,
                                 MOTION_DIR_RIGHT,0)==MOTION_PRIMITIVE_OK);
    update_n(&c,&o,5,-89.5f,-100.0f);
    assert(o.status==MOTION_PRIMITIVE_RUNNING); /* angle is good but gyro high */
    assert(o.immediate_neutral==0); /* acceleration phase may still be active */
    update_n(&c,&o,1,-94.0f,-100.0f);
    assert(o.status==MOTION_PRIMITIVE_FAULT);
    assert(o.result==MOTION_PRIMITIVE_OVERSHOOT);

    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_90,
                                 MOTION_DIR_LEFT,0)==MOTION_PRIMITIVE_OK);
    update_n(&c,&o,5,89,0);
    assert(o.phase==MOTION_PIVOT_SETTLE);
    assert(o.immediate_neutral==1);
    update_n(&c,&o,160,89,0);
    assert(o.status==MOTION_PRIMITIVE_DONE);
    assert(o.left_pm==0 && o.right_pm==0);
}

static void test_brake_release_180_only(void) {
    MotionPrimitiveController c;
    MotionPrimitiveOutput o;
    MotionPrimitive_Init(&c);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_180,
                                 MOTION_DIR_LEFT,0)==MOTION_PRIMITIVE_OK);
    update_n(&c,&o,5,0,0);
    update_n(&c,&o,5,110,830); /* predicted stopping distance starts braking */
    assert(c.phase==MOTION_PIVOT_BRAKE);
    update_n(&c,&o,5,140,500);
    assert(c.phase==MOTION_PIVOT_BRAKE);
    update_n(&c,&o,5,151,325);
    assert(c.phase==MOTION_PIVOT_COAST);
    assert(o.right_pm==0 && o.immediate_neutral==1);
    update_n(&c,&o,35,163,110);
    assert(c.phase==MOTION_PIVOT_APPROACH);
    assert(c.status==MOTION_PRIMITIVE_RUNNING);
    assert(o.right_pm>=0 && o.right_pm<=155); /* never high-power restart */
    assert(o.left_pm<=0);
}

static void test_faults_and_future_ids(void) {
    MotionPrimitiveController c;
    MotionPrimitiveOutput o;
    MotionPrimitive_Init(&c);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_STRAIGHT,
                                 MOTION_DIR_LEFT,0)==MOTION_PRIMITIVE_UNSUPPORTED);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_180,
                                 (MotionPrimitiveDirection)7,0)==MOTION_PRIMITIVE_BAD_ARGUMENT);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_180,
                                 MOTION_DIR_RIGHT,0)==MOTION_PRIMITIVE_OK);
    update_n(&c,&o,1,0,NAN);
    assert(o.status==MOTION_PRIMITIVE_FAULT);
    assert(o.result==MOTION_PRIMITIVE_SENSOR_FAULT);
    assert(o.immediate_neutral==1);
    assert(MotionPrimitive_Start(&c,MOTION_PRIMITIVE_PIVOT_90,
                                 MOTION_DIR_LEFT,0)==MOTION_PRIMITIVE_OK);
    update_n(&c,&o,80,0,0);
    update_n(&c,&o,1,0,-900);
    assert(o.status==MOTION_PRIMITIVE_FAULT);
    assert(o.result==MOTION_PRIMITIVE_RATE_LIMIT);
}

int main(void) {
    test_selection_and_direction();
    test_angle_wrap_and_stopping();
    test_brake_release_180_only();
    test_faults_and_future_ids();
    puts("MOTION_PRIMITIVES_NATIVE_PASS (not on-robot qualification)");
    return 0;
}
"""


def main() -> int:
    compiler = shutil.which(os.environ.get("CC", "gcc"))
    if compiler is None:
        raise SystemExit("A native C compiler is required (CC or gcc).")
    with tempfile.TemporaryDirectory(prefix="motion-primitives-") as folder:
        test_c = Path(folder) / "test_motion_primitives.c"
        test_c.write_text(HARNESS, encoding="utf-8")
        executable = Path(folder) / ("pivot_test.exe" if os.name == "nt" else "pivot_test")
        subprocess.run([
            compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "include"),
            str(ROOT / "src/motion_primitives.c"), str(test_c),
            "-lm", "-o", str(executable),
        ], cwd=ROOT, check=True, timeout=30)
        subprocess.run([str(executable)], cwd=ROOT, check=True, timeout=30)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
