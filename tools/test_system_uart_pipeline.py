#!/usr/bin/env python3
"""Synthetic host parser/regression tests. Never touches UART or motor."""
import importlib.util
from pathlib import Path
root=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("system_uart_pipeline",root/"system_uart_pipeline.py")
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)

assert mod.parse_line("SYS_STEP,ms=120,state=1,mode=0,id=0,duty=0,dl=0,dr=0") == {
    "type":"SYS_STEP","ms":120,"state":1,"mode":0,"id":0,
    "duty":0,"dl":0,"dr":0}
assert mod.parse_line("CMD_MODE,requested=ON,accepted=1")["accepted"] == 1
assert mod.parse_line("CLOCK_PROFILE,nominal_hz=96000000,trim=25")["trim"] == 25
for mode in ("OFF","ON"):
    code=1 if mode=="ON" else 0
    s=[{"type":"SYS_STEP","id":i,"state":1,"mode":code} for i in range(33)]
    s.append({"type":"SYS_STEP","id":33,"state":3,"mode":code})
    s.extend([
        {"type":"SYS_T","gap":0,"late":0,"maxcy":8000,"stale":0},
        {"type":"SYS_M","locked":1,"i2cerr":0,"to":0,"wd":0},
        {"type":"SYS_U","trim":25,"clkfault":0,"iclk":400000,
         "rx_err":0,"rx_ov":0,"tx_drop":0,"dma_err":0}])
    good=mod.evaluate(s,mode)
    assert good["software_pass"],good
    assert good["stages"]==33
    bad=[x for x in s if x.get("id")!=25]
    assert not mod.evaluate(bad,mode)["checks"]["33_unique_pwm_stages"]
    faulty=s+[{"type":"SYS_STEP","state":4,"mode":code,"fault":9}]
    assert not mod.evaluate(faulty,mode)["checks"]["no_sweep_fault"]
print("PASS host log parser and positive/negative OFF/ON stage evaluation; no hardware tested")
