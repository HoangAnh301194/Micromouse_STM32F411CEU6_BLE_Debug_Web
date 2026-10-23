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
import time
import types
from unittest.mock import patch
from system_uart_pipeline import PhaseSession

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
             "-DINTEGRATION_DIAG=1",
             "-DINTEGRATION_DIAG_MOTOR_ON=1",
             "-DINTEGRATION_DIAG_LIMITED_PWM_30=1",
             "-DENCODER_PID_ALPHA_PCT=8",
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
            # A BOTH-stage has two independent wheel measurements at
            # the same timestamp; do not erase wheel L with wheel R zeros.
            if f["refL"] or f["pwmL"]:
                unique[key]["refL"]=f["refL"]
                unique[key]["pwmL"]=f["pwmL"]
            if f["refR"] or f["pwmR"]:
                unique[key]["refR"]=f["refR"]
                unique[key]["pwmR"]=f["pwmR"]
        else: unique[key]=dict(f)
    metrics=analyze_trial(list(unique.values()))
    return {"alpha_pct":alpha,"repeat":repetition,"metrics":metrics}


class ProductionAlphaSelectionTests(unittest.TestCase):
    def test_production_encoder_gain_and_diagnostic_baselines(self):
        firmware=(ROOT/"src"/"encoder.c").read_text(encoding="utf-8")
        # Guard only the non-diagnostic branch; the historical alpha=0.30
        # diagnostic baseline remains compiled for A/B comparisons.
        self.assertIn(
            "#else\\n/* Production encoder speed estimator, 1-kHz TIM11 update. */\\n"
            "#define SPEED_LPF_ALPHA 0.10f\\n#endif", firmware)
        self.assertIn("#if ENCODER_PID_ALPHA_PCT == 8\\n"
                      "#define SPEED_LPF_ALPHA 0.08f", firmware)
        self.assertIn("#elif ENCODER_PID_ALPHA_PCT == 10\\n"
                      "#define SPEED_LPF_ALPHA 0.10f", firmware)
        self.assertIn("#elif ENCODER_PID_ALPHA_PCT == 30\\n"
                      "#define SPEED_LPF_ALPHA 0.30f", firmware)


class AnalysisTests(unittest.TestCase):
    def test_reference_and_repeats(self):
        data=[synth_trial(a,n,d) for n in range(1,4)
              for a,d in [(8,.01),(10,.10),(30,.19)]]
        for t in data:self.assertEqual(len(t["metrics"]),len(REQUIRED_ACTIVE))
        result=summarize(data)
        self.assertEqual(result["status"],"CLOSED_LOOP_CANDIDATE")
        self.assertEqual(result["candidate_alpha"],.08)
        self.assertEqual(result["production_alpha_unchanged"],.10)
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


class HeartbeatIndependenceTests(unittest.TestCase):
    def test_ping_continues_during_blocked_main_serial_read(self):
        class FakeSerial:
            def __init__(self):
                self.writes=[]
                self.closed=False
            def write(self, payload):
                self.writes.append((time.monotonic(),payload))
                return len(payload)
            def flush(self):
                pass
            def readline(self):
                # Reproduces a host read/log stall near the historical 750ms
                # MCU heartbeat lease. Main thread alone cannot emit PING.
                time.sleep(.62)
                return b""
            def close(self):
                self.closed=True

        fake=FakeSerial()
        serial_module=types.SimpleNamespace(Serial=lambda **kwargs: fake)
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(sys.modules,{"serial":serial_module}):
                session=PhaseSession("FAKE",115200,"ON",Path(folder),
                                     limited=True,expected_alpha_pct=30)
                try:
                    session.start_heartbeat()
                    session.poll()
                    pings=[(t,b) for t,b in fake.writes if b==b"PING\n"]
                    self.assertGreaterEqual(len(pings),3)
                    self.assertLess(session.max_ping_gap_ms,500.0)
                    self.assertIsNone(session._hb_error)
                finally:
                    session.close()
        self.assertTrue(fake.closed)
        self.assertFalse(session._hb_thread.is_alive())

    def test_plain_pipeline_does_not_spawn_heartbeat(self):
        class FakeSerial:
            def write(self,payload): return len(payload)
            def flush(self): pass
            def readline(self): return b""
            def close(self): pass
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(sys.modules,{"serial":types.SimpleNamespace(
                    Serial=lambda **kwargs:FakeSerial())}):
                session=PhaseSession("FAKE",115200,"OFF",Path(folder))
                try:
                    session.start_heartbeat()
                    self.assertIsNone(session._hb_thread)
                finally:
                    session.close()


if __name__=="__main__":
    native_test()
    unittest.main()
