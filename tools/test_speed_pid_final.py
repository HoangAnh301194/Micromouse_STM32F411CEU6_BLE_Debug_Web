#!/usr/bin/env python3
"""Strict host-only safety and native speed PID regression. NO COM/motor use."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
from speed_pid_final_pipeline import (
    analyze_trial, summarize, qgate, REQUIRED_ACTIVE
)


def native_test():
    cc=shutil.which(os.environ.get("CC","gcc"))
    if not cc:
        raise RuntimeError("gcc needed for actual speed PID source native test")
    with tempfile.TemporaryDirectory(prefix="pid-native-") as tmp:
        exe=Path(tmp)/("speed_pid.exe" if os.name=="nt" else "speed_pid")
        cmd=[cc,"-std=c11","-O2","-Wall","-Wextra","-Werror",
             "-DENCODER_SPEED_PID_DIAG=1",
             "-I",str(ROOT/"tests"/"speed_pid"/"stubs"),
             "-I",str(ROOT/"include"),
             str(ROOT/"src"/"pid.c"),str(ROOT/"src"/"speed_pid_diag.c"),
             str(ROOT/"tests"/"speed_pid"/"test_speed_pid.c"),
             "-o",str(exe)]
        subprocess.run(cmd,cwd=ROOT,check=True,timeout=60)
        subprocess.run([str(exe)],cwd=ROOT,check=True,timeout=60)


def synth_trial(alpha,repetition,delta=0.0):
    frames=[]
    metrics=[]
    for stage,wheel in REQUIRED_ACTIVE:
        step=stage%4
        target=step*800
        for i in range(40):
            t=i*50
            actual=target*(1.0+delta if i>=12 else 0.40+0.05*i)
            row={"type":"SYS_PID","ms":stage*10000+t,"id":stage,
                 "group":stage//4,"step":step,"target":target,"dt":50,
                 "refL":int(actual) if wheel==0 else 0,
                 "refR":int(actual) if wheel==1 else 0,
                 "pwmL":max(0,min(300,step*100-10)) if wheel==0 else 0,
                 "pwmR":max(0,min(300,step*100-10)) if wheel==1 else 0,
                 "emaL":int(actual),"emaR":int(actual)}
            frames.append(row)
    # Deduplicate shared BOTH records by timestamps.
    unique={}
    for f in frames:
        key=(f["id"],f["ms"])
        if key in unique:
            unique[key]["refL"]=f["refL"]
            unique[key]["refR"]=f["refR"]
            unique[key]["pwmL"]=f["pwmL"]
            unique[key]["pwmR"]=f["pwmR"]
        else: unique[key]=dict(f)
    metrics=analyze_trial(list(unique.values()))
    return {"alpha_pct":alpha,"repeat":repetition,"metrics":metrics}


class AnalysisTests(unittest.TestCase):
    def test_reference_and_repeats(self):
        data=[synth_trial(a,n,d) for n in range(1,4)
              for a,d in [(8,.01),(10,.10),(30,.19)]]
        for t in data:self.assertEqual(len(t["metrics"]),len(REQUIRED_ACTIVE))
        result=summarize(data)
        self.assertEqual(result["status"],"CLOSED_LOOP_CANDIDATE")
        self.assertEqual(result["candidate_alpha"],.08)
        self.assertEqual(result["production_alpha_unchanged"],.30)
        with self.assertRaises(RuntimeError):summarize(data[:8])
    def test_inconclusive(self):
        data=[synth_trial(a,n,.015) for n in range(1,4) for a in (8,10,30)]
        self.assertEqual(summarize(data)["status"],"INCONCLUSIVE")
    def test_missing_data_is_not_pass(self):
        frames=synth_trial(8,1)["metrics"]
        self.assertEqual(len(frames),len(REQUIRED_ACTIVE))
        with self.assertRaises(RuntimeError):analyze_trial([])
        with self.assertRaises(RuntimeError):
            analyze_trial([{"type":"SYS_PID","id":1,"group":0,"step":1,
                            "ms":100,"dt":50,"target":800,"refL":800,"pwmL":450}])
    def test_qgate_fault_heartbeat_and_uart(self):
        phase={"mode":"ON","software_pass":True,"stages":12,
               "faults":[],"checks":{"expected_unique_pwm_stages":True},
               "host_heartbeat":{"ping_sent":300,"ping_ack_received":300,
                                 "max_inter_ping_gap_ms":170},
               "uart":{"rx_err":0,"tx_drop":0,"dma_err":0},
               "timer":{"late":0,"gap":0}}
        self.assertTrue(qgate(phase,12))
        phase["host_heartbeat"]["max_inter_ping_gap_ms"]=950
        with self.assertRaises(RuntimeError):qgate(phase,12)
        phase["host_heartbeat"]["max_inter_ping_gap_ms"]=170
        phase["uart"]["rx_err"]=1
        with self.assertRaises(RuntimeError):qgate(phase,12)


if __name__=="__main__":
    native_test()
    unittest.main()
