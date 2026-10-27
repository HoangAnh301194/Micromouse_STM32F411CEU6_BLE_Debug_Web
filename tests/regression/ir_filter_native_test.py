#!/usr/bin/env python3
"""Compile and exercise actual portable six-channel IR candidate filters."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
HARNESS=r"""
#include "ir_filter.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>

int main(void) {
    IR_Filter_t f;
    IR_FilterFrame_t o;
    uint16_t x[6]={400,400,400,400,400,400};
    IR_FilterReset(&f);
    IR_FilterStep(&f,x,&o);
    for(int i=0;i<6;i++){
        assert(o.raw[i]==400 && o.median3[i]==400);
        assert(o.ema10[i]==400 && o.ema25[i]==400);
        assert(o.median_ema25[i]==400);
    }
    IR_FilterStep(&f,x,&o);
    IR_FilterStep(&f,x,&o);
    x[0]=4000; /* single-sample spike, median3 rejects it */
    IR_FilterStep(&f,x,&o);
    assert(o.raw[0]==4000);
    assert(o.median3[0]==400);
    assert(o.ema10[0]>400);
    assert(o.ema25[0]>o.ema10[0]);
    assert(o.median_ema25[0]==400);
    x[0]=400;
    IR_FilterStep(&f,x,&o);
    assert(o.median3[0]==400);
    for(int k=0;k<30;k++)IR_FilterStep(&f,x,&o);
    assert(o.ema25[0]==400);
    assert(o.ema10[0]<=416);
    /* A real step MUST eventually pass median and every EMA. */
    x[2]=1100;
    IR_FilterStep(&f,x,&o);
    assert(o.median3[2]==400); /* 2 old samples remain */
    IR_FilterStep(&f,x,&o);
    assert(o.median3[2]==1100);
    for(int k=0;k<150;k++)IR_FilterStep(&f,x,&o);
    assert(o.ema10[2]==1100 && o.ema25[2]==1100);
    assert(o.median_ema25[2]==1100);
    x[5]=6000; /* out-of-range safety clamp */
    IR_FilterStep(&f,x,&o);
    assert(o.raw[5]==4095);
    IR_FilterReset(&f);
    IR_FilterStep(&f,x,&o);
    assert(o.ema10[5]==4095);
    puts("IR_FILTER_NATIVE_PASS");
    return 0;
}
"""

def main():
    compiler=shutil.which(os.environ.get("CC","gcc"))
    if not compiler:raise SystemExit("GCC missing")
    with tempfile.TemporaryDirectory(prefix="ir-filter-test-") as td:
        p=Path(td)
        (p/"test.c").write_text(HARNESS,encoding="utf8")
        exe=p/("ir.exe" if os.name=="nt" else "ir")
        subprocess.run([compiler,"-std=c11","-O2","-Wall","-Wextra","-Werror",
                        "-I",str(ROOT/"include"),str(ROOT/"src/ir_filter.c"),
                        str(p/"test.c"),"-o",str(exe)],
                       cwd=ROOT,check=True,timeout=30)
        subprocess.run([str(exe)],cwd=ROOT,check=True,timeout=30)
if __name__=="__main__":main()
