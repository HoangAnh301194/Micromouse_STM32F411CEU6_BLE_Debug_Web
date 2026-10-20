#!/usr/bin/env python3
"""Synthetic host parser/regression tests. Never touches UART or motor."""
import importlib.util
from pathlib import Path
root=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("system_uart_pipeline",root/"system_uart_pipeline.py")
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)

import subprocess
import sys

# Parse --help in a fresh Python process to catch argparse percent escaping
# and Python 3.14 help formatter compatibility without a COM port or MCU.
cli = subprocess.run(
    [sys.executable, str(root/"system_uart_pipeline.py"), "--help"],
    capture_output=True, text=True, check=True, timeout=10
)
assert "--motor-on" in cli.stdout and "100% motor-on" in cli.stdout

assert mod.parse_line("SYS_STEP,ms=120,state=1,mode=0,id=0,duty=0,dl=0,dr=0") == {
    "type":"SYS_STEP","ms":120,"state":1,"mode":0,"id":0,
    "duty":0,"dl":0,"dr":0}
assert mod.parse_line("CMD_MODE,requested=ON,accepted=1")["accepted"] == 1
assert mod.parse_line("CLOCK_PROFILE,nominal_hz=96000000,trim=25")["trim"] == 25
for mode in ("OFF","ON"):
    code=1 if mode=="ON" else 0
    s=[{"type":"SYS_STEP","id":0,"state":1,"mode":code,"dt":0}]
    for i in range(33):
        g, step = divmod(i, 11)
        drive_ms, coast_ms = mod.planned_stage_times_ms(i)
        dl = dr = 0
        if mode == "ON" and step > 0:
            if g in (0, 2): dl = 40 * step
            if g in (1, 2): dr = 40 * step
        s.append({"type":"SYS_STEP","id":i,"state":2,"mode":code,
                  "group":g,"step":step,"dl":dl,"dr":dr,"dt":drive_ms})
        for ms in range(500,coast_ms,500):
            s.append({"type":"SYS_COAST","id":i,"mode":code,"dt":ms,
                      "ppsL":0,"ppsR":0,"dl":0,"dr":0})
        s.append({"type":"SYS_STEP","id":i+1,
                  "state":3 if i==32 else 1,
                  "mode":code,"dt":coast_ms,"vL":0,"vR":0,"dl":0,"dr":0})
    s.extend([
        {"type":"SYS_T","gap":0,"late":0,"maxcy":8000,"stale":0,
         "ticks":160000,"motticks":160000,"ir":160000,"valid":148000},
        {"type":"SYS_M","locked":1,"i2cerr":0,"to":0,"wd":0,"age":990},
        {"type":"SYS_U","trim":25,"clkfault":0,"iclk":400000,
         "rx_err":0,"rx_ov":0,"tx_drop":0,"dma_err":0}])
    good=mod.evaluate(s,mode)
    assert good["software_pass"],good
    assert good["stages"]==33
    # Diagnostic SPSC acceptance: any dropped/misaligned encoder 1-ms
    # sample must fail; old non-Encoder profiles remain compatible.
    shadow=[*s, {"type":"SYS_AB","ms":123},
            {"type":"SYS_AQ","ms":150000,"in":150000,"out":150000,
             "lost":0,"edge":0,"q":0,"peak":7}]
    assert mod.evaluate(shadow,mode)["checks"]["shadow_spsc_no_loss"]
    assert not mod.evaluate(shadow[:-1],mode)["checks"]["shadow_spsc_no_loss"]
    lost=[dict(x) for x in shadow]
    lost[-1]["lost"]=1
    assert not mod.evaluate(lost,mode)["checks"]["shadow_spsc_no_loss"]
    misaligned=[dict(x) for x in shadow]
    misaligned[-1]["edge"]=1
    assert not mod.evaluate(misaligned,mode)["checks"]["shadow_spsc_no_loss"]

    bad=[x for x in s if x.get("id")!=25]
    assert not mod.evaluate(bad,mode)["checks"]["33_unique_pwm_stages"]
    faulty=s+[{"type":"SYS_STEP","state":4,"mode":code,"fault":9}]
    assert not mod.evaluate(faulty,mode)["checks"]["no_sweep_fault"]
    if mode == "ON":
        flat=[dict(x) for x in s]
        for x in flat:
            if x.get("type") == "SYS_STEP" and x.get("state") == 2 and x.get("group") == 2 and x.get("step") in (8,9,10):
                x["dl"] = x["dr"] = -2
        outcome = mod.evaluate(flat, mode)
        assert not outcome["checks"]["encoder_drive_response_consistent"]
        assert len(outcome["encoder_review"]["high_pwm_anomalies"]) == 6
        wrong=[dict(x) for x in s]
        for x in wrong:
            if x.get("type") == "SYS_STEP" and x.get("state") == 2 and x.get("group") == 2 and x.get("step") in (6,7):
                x["dl"] = x["dr"] = -300
        assert not mod.evaluate(wrong,"ON")["checks"]["extended_drive_coast_inertia"]
        assert any(x.get("reason") == "forward_encoder_sign_negative"
                   for x in mod.evaluate(wrong,"ON")["extended"]["problems"])
        truncated=[x for x in s if not(x.get("type")=="SYS_COAST" and x.get("id")==3)]
        assert not mod.evaluate(truncated,"ON")["checks"]["extended_drive_coast_inertia"]
print("PASS CLI boot + synthetic UART data + high-duty encoder flatline regression; no hardware tested")
