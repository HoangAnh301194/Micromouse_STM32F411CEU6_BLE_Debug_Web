#!/usr/bin/env python3
"""One invocation: diagnose JDY-33 FFE1 -> USART2 RX PA3 without motor actuation.

A: receiver enabled + autonomous 5Hz TX beacon.
B: receiver disabled + identical autonomous 5Hz TX beacon (baseline).
C: receiver enabled + PC BLE challenge, verified via independent UART1 COM7
   and optional BLE echo; rates 2/5/10/20 Hz, one 16-byte nonce command each.

This test NEVER tunes PID, sends motor commands, or writes Flash map sectors.
By default reflash known motor-locked TX-only Flash Map firmware afterwards.
Close Web BLE and other Bluetooth clients before starting!
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
DIAG_ENV = "ble_rx_diag_96m"
RESTORE_ENV = "flash_map_diag_96m"
RATES = (2, 5, 10, 20)


def parse_line(text: str) -> dict:
    parts = text.strip().split(",")
    d = {"type": parts[0], "text": text.strip()}
    for item in parts[1:]:
        if "=" not in item:
            continue
        k, val = item.split("=", 1)
        d[k] = int(val) if k != "value" and val.isdecimal() else val
    return d


def nonce_value(seq: int) -> int:
    return (0xA5C30000 ^ ((seq * 2654435761) & 0xFFFFFFFF)) & 0xFFFFFFFF


def build_nonce(seq: int) -> bytes:
    if not 0 <= seq <= 9999:
        raise ValueError("nonce sequence is 0..9999")
    cmd = f"C,{seq:04d},{nonce_value(seq):08X}\n".encode("ascii")
    assert len(cmd) == 16
    return cmd


def percentile(values: list[float], pct: int):
    if not values:
        return None
    a = sorted(values)
    return round(a[min(len(a)-1, max(0, (len(a)*pct + 99)//100-1))], 3)


def counter_delta(before: dict, after: dict, key: str):
    x, y = before.get(key), after.get(key)
    if isinstance(x, int) and isinstance(y, int):
        return (y-x) & 0xFFFFFFFF
    return None


def assess_challenges(expected: dict[int, int], sent: set[int],
                      uart_events: list[dict], ble_events: list[dict],
                      write_errors: list[str], before: dict, after: dict) -> dict:
    uart_hits, ble_hits = set(), set()
    uart_bad, ble_bad = [], []
    for events, hits, bad in ((uart_events, uart_hits, uart_bad),
                              (ble_events, ble_hits, ble_bad)):
        for e in events:
            seq, val = e.get("seq"), e.get("value")
            if not isinstance(seq, int):
                continue
            if seq not in expected:
                continue
            if isinstance(val, str):
                try:
                    actual = int(val, 16)
                except ValueError:
                    bad.append(seq)
                    continue
            else:
                bad.append(seq)
                continue
            if actual == expected[seq]:
                hits.add(seq)
            else:
                bad.append(seq)
    total = len(expected)
    uart_recv = len(uart_hits)
    ble_recv = len(ble_hits)
    uart_rx_errors = sum((counter_delta(before,after,k) or 0)
                         for k in ("ore","fe","ne","overflow"))
    result = {
        "expected": total,
        "gatt_writes_without_exception": len(sent),
        "gatt_write_errors": write_errors,
        "independent_uart_acks": uart_recv,
        "ble_echo_acks": ble_recv,
        "uart_missing_seq": sorted(set(expected)-uart_hits),
        "ble_missing_seq": sorted(set(expected)-ble_hits),
        "uart_corrupted_seq": sorted(set(uart_bad)),
        "ble_corrupted_seq": sorted(set(ble_bad)),
        "mcu_uart_rx_byte_delta": counter_delta(before,after,"rx"),
        "mcu_rx_error_delta": uart_rx_errors,
        "mcu_rx_ok_delta": counter_delta(before,after,"ok"),
        "mcu_rx_bad_delta": counter_delta(before,after,"bad"),
    }
    if uart_recv == total and not write_errors and not uart_bad and uart_rx_errors == 0:
        result["verdict"] = "RX_FUNCTIONAL_IN_THIS_TEST"
    elif uart_recv:
        result["verdict"] = "INTERMITTENT_OR_INCOMPLETE_RX"
    elif len(sent) == 0:
        result["verdict"] = "GATT_WRITE_NOT_TESTABLE"
    else:
        result["verdict"] = "DOWNLINK_NOT_VERIFIED"
    return result


def install_missing():
    for name in ("bleak","serial","platformio"):
        module = "serial" if name == "serial" else name
        if importlib.util.find_spec(module) is None:
            pkg = "pyserial" if name == "serial" else name
            print(f"Installing dependency: {pkg}", flush=True)
            subprocess.run([sys.executable,"-m","pip","install",pkg],check=True)


def flash(env: str, out: Path):
    cmd = [sys.executable,"-m","platformio","run","-e",env,"-t","upload"]
    log_file = out / ("flash_"+env+".log")
    print(f"FLASH {env}", flush=True)
    with log_file.open("w",encoding="utf8") as f:
        r = subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
    if r.returncode:
        tail = "\n".join(log_file.read_text(encoding="utf8",errors="replace").splitlines()[-35:])
        raise RuntimeError(f"ST-Link flash failed ({env}), log={log_file}\n{tail}")


async def discover(name: str, address: str | None):
    from bleak import BleakScanner
    seen = await BleakScanner.discover(timeout=12, return_adv=True)
    devs = [pair[0] for pair in seen.values()]
    if address:
        matches = [d for d in devs if d.address.lower() == address.lower()]
    else:
        matches = [d for d,a in seen.values()
                   if name.lower() in (a.local_name or d.name or "").lower()]
    if len(matches) != 1:
        listings = ", ".join(f"{d.name or a.local_name or '?'} ({d.address})"
                             for d,a in seen.values())
        raise RuntimeError(f"Expected exactly one JDY-33, found {len(matches)}"
                           f" matches for {address or name}: {listings}. "
                           "Disconnect Web BLE and verify the advertised address.")
    return matches[0]


class Capture:
    def __init__(self, port: str, out: Path):
        import serial
        self.serial = serial.Serial(port,baudrate=115200,timeout=0,write_timeout=2)
        self.out=out
        self.uart_events: list[dict] = []
        self.ble_events: list[dict] = []
        self.uart_pending=bytearray()
        self.ble_pending=bytearray()
        self.ble_chunks=[]
        self.logs=(out/"ble_rx_uart1.log").open("w",encoding="utf8")
        self.ble_logs=(out/"ble_rx_notifications.log").open("w",encoding="utf8")
        self.ble_raw=(out/"ble_rx_raw.bin").open("wb")
        self.client=None
        self.characteristic=None
        self.with_response=None
        self.rx_error=None
        self.task=None

    @staticmethod
    def _parse(data: bytes, pending: bytearray,
               events: list[dict], name: str, dest):
        pending.extend(data)
        while True:
            end=pending.find(b"\n")
            if end<0:
                if len(pending)>1024:
                    del pending[:-128]
                return
            raw=bytes(pending[:end]).rstrip(b"\r")
            del pending[:end+1]
            if not raw:
                continue
            line=raw.decode("ascii",errors="replace")
            item=parse_line(line)
            item["host_ns"]=time.perf_counter_ns()
            events.append(item)
            dest.write(f"{datetime.now(timezone.utc).isoformat()} [{name}] {line}\n")
            dest.flush()

    async def uart_pump(self):
        try:
            while True:
                waiting=self.serial.in_waiting
                if waiting:
                    chunk=self.serial.read(min(waiting,4096))
                    self._parse(chunk,self.uart_pending,self.uart_events,
                                "UART1",self.logs)
                await asyncio.sleep(0.008)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.rx_error=exc

    def on_notify(self, _handle, value: bytearray):
        try:
            raw=bytes(value)
            self.ble_raw.write(raw)
            self.ble_chunks.append({
                "host_ns":time.perf_counter_ns(),"data_hex":raw.hex()
            })
            self._parse(raw,self.ble_pending,self.ble_events,"BLE",self.ble_logs)
        except Exception as exc:
            self.rx_error=exc

    async def connect(self, name: str, address: str | None):
        from bleak import BleakClient
        dev=await discover(name,address)
        self.client=BleakClient(dev,timeout=20)
        await self.client.connect()
        char=self.client.services.get_characteristic(UUID)
        if not char or "notify" not in char.properties:
            raise RuntimeError("JDY-33 FFE1 notify is not available")
        self.characteristic=char
        if "write-without-response" in char.properties:
            self.with_response=False
        elif "write" in char.properties:
            self.with_response=True
        else:
            self.with_response=None
        await self.client.start_notify(char,self.on_notify)
        print("BLE connected",dev.address,
              "write_permission",self.with_response,flush=True)

    async def write(self, data: bytes):
        if self.with_response is None:
            raise RuntimeError("GATT FFE1 does not advertise a write property")
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE device is not connected")
        await asyncio.wait_for(
            self.client.write_gatt_char(self.characteristic,data,
                                        response=self.with_response),timeout=3)

    async def send_uart(self, command: str):
        self.serial.write((command+"\n").encode("ascii"))
        self.serial.flush()

    async def event(self, typ: str, after: int, timeout: float = 5):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if self.rx_error:
                raise RuntimeError(f"Capture failed: {self.rx_error}")
            for e in self.uart_events[after:]:
                if e["type"]==typ:
                    return e
            await asyncio.sleep(0.025)
        raise TimeoutError(f"No UART1 {typ} response; check correct flashed RXBOOT image/COM port")

    async def change_mode(self, mode: str):
        mark=len(self.uart_events)
        await self.send_uart("MODE "+mode)
        e=await self.event("RXMODE",mark,6)
        if e.get("mode")!=mode or e.get("locked") != 1:
            raise RuntimeError("Unsafe or incorrect mode/locked state: "+str(e))
        return e

    async def snapshot(self):
        mark=len(self.uart_events)
        await self.send_uart("STATS")
        return await self.event("RXS",mark,5)

    async def close(self):
        try:
            if self.serial.is_open:
                await self.send_uart("MODE B")
                await asyncio.sleep(0.10)
        except Exception:
            pass
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
            except Exception:
                pass
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        self.serial.close()
        for f in (self.logs,self.ble_logs,self.ble_raw):
            f.close()
        (self.out/"ble_rx_gatt_notifications.json").write_text(
            json.dumps(self.ble_chunks,indent=2),encoding="utf8")


async def baseline(capture: Capture, mode: str, duration: float):
    await capture.change_mode(mode)
    before=await capture.snapshot()
    n=len(capture.ble_events)
    await asyncio.sleep(duration)
    after=await capture.snapshot()
    beacons=[x for x in capture.ble_events[n:]
             if x["type"]=="RHB" and x.get("mode")==mode]
    seqs={x.get("seq") for x in beacons if isinstance(x.get("seq"),int)}
    result={
        "mode":mode,"duration_s":duration,
        "expected_approx_5hz":round(duration*5),
        "ble_beacons_received":len(seqs),
        "ble_duplicates":len(beacons)-len(seqs),
        "ble_beacon_ratio":round(len(seqs)/(duration*5),3),
        "mcu_rx_bytes_delta":counter_delta(before,after,"rx"),
        "rx_errors_delta":sum((counter_delta(before,after,k) or 0)
                              for k in ("ore","fe","ne","overflow")),
        "txdrop_delta":counter_delta(before,after,"drop"),
        "motor_locked":after.get("locked")==1,
    }
    result["pass"]=result["motor_locked"] and result["ble_beacon_ratio"]>=0.75
    if mode=="B" and result["mcu_rx_bytes_delta"]:
        result["pass"]=False
    print(f"PHASE {mode}: {len(seqs)}/{duration*5:.0f} received TX beacons "
          f"rxbytes={result['mcu_rx_bytes_delta']} "
          f"rxerrors={result['rx_errors_delta']} "
          f"{'PASS' if result['pass'] else 'LIMIT'}",flush=True)
    return result


async def challenge_rate(capture: Capture, rate: int, samples: int, offset: int):
    before=await capture.snapshot()
    u0=len(capture.uart_events)
    b0=len(capture.ble_events)
    expected={}
    sent=set()
    errors=[]
    started=time.monotonic()
    for i in range(samples):
        seq=offset+i
        expected[seq]=nonce_value(seq)
        try:
            await capture.write(build_nonce(seq))
            sent.add(seq)
        except Exception as exc:
            errors.append(f"seq={seq} {type(exc).__name__}: {exc}")
            if capture.client is None or not capture.client.is_connected:
                break
        due=started+(i+1)/rate
        await asyncio.sleep(max(0.0,due-time.monotonic()))
    await asyncio.sleep(1.0) # collect serial DMA and delayed BLE notifications
    after=await capture.snapshot()
    result=assess_challenges(expected,sent,capture.uart_events[u0:],
                             capture.ble_events[b0:],errors,before,after)
    result.update({"rate_hz":rate,"sample_count":samples,
                   "real_send_duration_s":round(time.monotonic()-started,2),
                   "motor_locked":after.get("locked")==1})
    print(f" C/{rate:>2}Hz: UART verified {result['independent_uart_acks']}/{samples}; "
          f"BLE echo {result['ble_echo_acks']}/{samples}; "
          f"RX errors={result['mcu_rx_error_delta']} => {result['verdict']}",
          flush=True)
    return result


def save(out: Path, results: dict, capture: Capture | None):
    if capture:
        (out/"ble_rx_uart1_events.json").write_text(
            json.dumps(capture.uart_events,indent=2),encoding="utf8")
        (out/"ble_rx_ble_events.json").write_text(
            json.dumps(capture.ble_events,indent=2),encoding="utf8")
    (out/"ble_rx_qualification.json").write_text(
        json.dumps(results,indent=2),encoding="utf8")
    rows=results.get("challenge_results",[])
    if rows:
        keys=["rate_hz","sample_count","gatt_writes_without_exception",
              "independent_uart_acks","ble_echo_acks","mcu_uart_rx_byte_delta",
              "mcu_rx_error_delta","mcu_rx_ok_delta","mcu_rx_bad_delta","verdict"]
        with (out/"ble_rx_rates.csv").open("w",newline="",encoding="utf8") as f:
            writer=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)


def decide(results):
    a,b=results.get("A"),results.get("B")
    tests=results.get("challenge_results") or []
    if not a or not b or not tests:
        return "INCOMPLETE"
    if all(x["verdict"]=="RX_FUNCTIONAL_IN_THIS_TEST" for x in tests):
        return "RX_FUNCTIONAL_IN_TESTED_RANGE"
    if all(x["verdict"]=="GATT_WRITE_NOT_TESTABLE" for x in tests):
        return "NOT_TESTABLE_NO_GATT_WRITE"
    if all(x["independent_uart_acks"]==0 for x in tests):
        return "RX_NOT_VERIFIED_CHECK_GATT_AND_PA3"
    return "RX_INTERMITTENT_OR_RATE_LIMITED"


async def run(args,out):
    cap=None
    result={
        "timestamp_utc":datetime.now(timezone.utc).isoformat(),
        "hardware_verified":False,
        "port":args.port,"ble_name":args.name,"ble_address":args.address,
        "diagnostic_env":DIAG_ENV,"restored_env":None,
        "A":None,"B":None,"challenge_results":[],
        "verdict":"INCOMPLETE",
        "limitations":[
            "BLE GATT write completion alone is NOT proof MCU PA3 received a byte.",
            "UART1 RXACK with exact nonce independently proves selected MCU RX path.",
            "No single test can establish permanent physical failure of JDY-33 RX.",
            "A and B vary ONLY USART2 RE/RXNEIE under identical 5Hz TX beacon.",
            "A/B firmware has motors hard locked, no TIM11/IR/MPU and no Flash writes.",
            "For PID tuning, use independently authenticated, bounded, fail-safe "
            "parameter updates in a DIFFERENT firmware only after RX qualification.",
        ],
    }
    try:
        if not args.no_flash:
            flash(DIAG_ENV,out)
        cap=Capture(args.port,out)
        cap.task=asyncio.create_task(cap.uart_pump())
        # Require diagnostic firmware, motor locked; refuse unrelated firmware.
        seen=len(cap.uart_events)
        await cap.send_uart("HELP")
        await cap.event("RXHELP",seen,6)
        status=await cap.snapshot()
        if status.get("locked") != 1:
            raise RuntimeError("MOTOR NOT LOCKED; aborting test")
        await cap.connect(args.name,args.address)
        result["gatt_write_supported"]=cap.with_response is not None
        result["A"]=await baseline(cap,"A",args.seconds)
        result["B"]=await baseline(cap,"B",args.seconds)
        await cap.change_mode("C")
        for index,rate in enumerate(args.rates):
            if cap.with_response is None:
                result["challenge_results"].append({
                    "rate_hz":rate,"sample_count":0,"independent_uart_acks":0,
                    "ble_echo_acks":0,"verdict":"GATT_WRITE_NOT_TESTABLE"
                })
                break
            result["challenge_results"].append(
                await challenge_rate(cap,rate,args.samples,index*1000+1))
        result["verdict"]=decide(result)
    except Exception as exc:
        result["error"]=f"{type(exc).__name__}: {exc}"
        print("QUALIFICATION ABORTED:",result["error"],file=sys.stderr,flush=True)
    finally:
        if cap is not None:
            await cap.close()
            save(out,result,cap)
        if not args.no_flash and not args.no_restore:
            try:
                flash(RESTORE_ENV,out)
                result["restored_env"]=RESTORE_ENV
            except Exception as exc:
                result["restore_error"]=f"{type(exc).__name__}: {exc}"
                print("CRITICAL: motor OFF restore failed. "
                      "Cut motor VM power and restore flash_map_diag_96m manually.",
                      file=sys.stderr,flush=True)
        save(out,result,cap)
    print("FINAL",result["verdict"],"results",out,flush=True)
    return result


def arguments():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port",default="COM7",help="Independent UART1 USB-TTL")
    p.add_argument("--name",default="Nezumi_Chan")
    p.add_argument("--address",help="Exact BLE device adapter address")
    p.add_argument("--seconds",type=float,default=6.0,
                   help="A and B comparison duration (4..30 seconds)")
    p.add_argument("--samples",type=int,default=20,
                   help="Challenge count at each frequency (10..100)")
    p.add_argument("--rates",type=int,nargs="+",default=list(RATES))
    p.add_argument("--no-flash",action="store_true",
                   help="Use already flashed ble_rx_diag_96m only; no restore")
    p.add_argument("--no-restore",action="store_true",
                   help="Keep diagnostic firmware in motor-locked MODE B")
    p.add_argument("--output",type=Path)
    return p.parse_args()


def main():
    args=arguments()
    if not 4<=args.seconds<=30 or not 10<=args.samples<=100:
        raise SystemExit("--seconds must be 4..30 and --samples 10..100")
    if not args.rates or any(r<1 or r>20 for r in args.rates):
        raise SystemExit("--rates may contain values 1..20 Hz; this is NOT PID tuning")
    if len(args.rates)*1000+args.samples>9999:
        raise SystemExit("Too many rates for four-digit nonce sequence")
    install_missing()
    out=args.output or ROOT/"results"/(
        "ble_rx_ab_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True,exist_ok=True)
    report=asyncio.run(run(args,out))
    if report.get("error") or report.get("restore_error") or report["verdict"]=="INCOMPLETE":
        raise SystemExit(2)


if __name__=="__main__":
    main()
