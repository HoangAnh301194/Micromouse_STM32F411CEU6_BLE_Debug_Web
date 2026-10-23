#!/usr/bin/env python3
"""Offline UART profiler CLI, byte counts, CPU budget and fault negatives."""
import importlib.util
from pathlib import Path
import subprocess
import sys

tool=Path(__file__).resolve().parent/"uart_rate_benchmark.py"
spec=importlib.util.spec_from_file_location("profile",tool)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
p=subprocess.run([sys.executable,str(tool),"--help"],
                 capture_output=True,text=True,check=True,timeout=15)
assert "--port" in p.stdout and "--rates" in p.stdout
assert m.BAUD//10==11520
assert m.TX_RING_BYTES==1024
assert m.MAX_RING_PEAK==768
assert all(5<=r<=400 for r in m.RATES)

t=3.0
start={"accb":1000,"doneb":1000,"drops":0,"fmtcy":100,
       "dmacy":20,"rxcy":10,"dmaerr":0}
end={"accb":11000,"doneb":10900,"drops":0,"fmtcy":1_200_100,
     "dmacy":45_020,"rxcy":20_010,"peak":450,"skip":0,"dmaerr":0}
sys_start={"late":0,"gap":0,"stale":0,"i2cerr":0,"to":0,
           "rxerr":0,"rxov":0,"rxb":0}
sys_end=dict(sys_start)
probes=[{"type":"U_PROBE","seq":i} for i in range(120)]
good=m.assess(40,120,probes,start,end,sys_start,sys_end,t,10300)
assert good["stable"],good
assert good["observed_probe_hz"]==40.0
assert good["host_probe_sequence_missing"]==0
assert good["cpu_uart_pct_nominal_96m"]<3.0

cases=[
  ("host_missed_frame",dict(end),dict(sys_end),probes[:-1],120,40),
  ("ring_almost_full",{**end,"peak":900},dict(sys_end),probes,120,40),
  ("TX_drop",{**end,"drops":1},dict(sys_end),probes,120,40),
  ("UART_CPU_high",{**end,"fmtcy":13_000_100},dict(sys_end),probes,120,40),
  ("TIM11_late",dict(end),{**sys_end,"late":1},probes,120,40),
  ("gyro_stale",dict(end),{**sys_end,"stale":1},probes,120,40),
  ("i2c_error",dict(end),{**sys_end,"i2cerr":1},probes,120,40),
  ("rx_overflow",dict(end),{**sys_end,"rxov":1},probes,120,40),
  ("generator_skipped",{**end,"skip":1},dict(sys_end),probes,120,40),
  ("DMA_error",{**end,"dmaerr":1},dict(sys_end),probes,120,40),
  ("oversubscribed_rate",dict(end),dict(sys_end),probes,120,100),
]
for name,tx,sy,packets,produced,rate in cases:
    verdict=m.assess(rate,produced,packets,start,tx,sys_start,sy,t,10300)
    assert not verdict["stable"],(name,verdict)
assert not m.assess(40,120,probes,start,end,sys_start,
                    {**sys_end,"rxb":100},t,10300,20,19)["stable"]
assert m.assess(40,120,probes,start,end,sys_start,
                {**sys_end,"rxb":100},t,10300,20,20)["stable"]
assert m.parse_packet("U_TX,ms=1000,hz=100,seq=120,drop=0")=={
    "type":"U_TX","ms":1000,"hz":100,"seq":120,"drop":0}
assert m.elapsed32(0xFFFFFFF0,0x10)==0x20
print("PASS UART CLI + 115200 8N1 math + 11 fault injections + PA10 ACK/bytes + IRQ/CPU/frequency budgets; no MCU tested")
