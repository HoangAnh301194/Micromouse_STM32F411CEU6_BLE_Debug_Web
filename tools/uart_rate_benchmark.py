#!/usr/bin/env python3
"""Measure safe PA9 TX DMA / PA10 RX IRQ log frequency with all sensors active.

Opt-in, motor outputs hard-locked at boot. No KEY and no motor command.
Source: firmware [env:integration_diag_uart_96m] and TXBENCH <hz>, TXSTAT.
Measures confirmed host frames and MCU counters; never claims 115200 baud
means 115200 bytes/sec, nor confuses queued bytes with bytes transmitted.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime
from collections import deque
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
BAUD=115200
SYSCLK_NOMINAL=96_000_000
TX_RING_BYTES=1024
RATES=(5,10,20,40,60,80,100,120,140,160,180,200,240,300,400)
PHASE_SEC=3.0
MAX_PRN_CPU_PERCENT=3.0
MAX_RING_PEAK=768


def parse_packet(line):
    fields=line.strip().split(",")
    if not fields or not fields[0]:return None
    p={"type":fields[0]}
    for field in fields[1:]:
        if "=" not in field:continue
        key,val=field.split("=",1)
        if not key:continue
        try:p[key]=int(val)
        except ValueError:p[key]=val
    return p


def elapsed32(a,b):
    if not isinstance(a,int) or not isinstance(b,int):return None
    return (b-a) & 0xFFFFFFFF


def assess(rate, produced, observed, start, end, system_start, system_end,
           time_s, raw_bytes, ping_sent=0, ping_acked=0, ping_rtt_ms=None):
    """Conservative estimate, not an electrically verified maximum frequency."""
    delta=lambda key:elapsed32(start.get(key),end.get(key))
    sysdelta=lambda key:elapsed32(system_start.get(key),system_end.get(key))
    seen=[int(x["seq"]) for x in observed if x.get("type")=="U_PROBE"
          and isinstance(x.get("seq"),int)]
    gaps=(max(0,produced-len(set(seen))) if produced is not None else None)
    cpu_cycles=sum((delta(k) or 0) for k in ("fmtcy","dmacy","rxcy"))
    cpu_pct=100.0*cpu_cycles/(SYSCLK_NOMINAL*max(time_s,1e-3))
    measured=round(len(seen)/max(time_s,1e-3),2)
    accepted=delta("accb")
    dma_bytes=delta("doneb")
    dropped=delta("drops")
    late=sysdelta("late")
    gap=sysdelta("gap")
    stale=sysdelta("stale")
    i2cerr=sysdelta("i2cerr")
    timeout=sysdelta("to")
    rxerror=sysdelta("rxerr")
    rxoverflow=sysdelta("rxov")
    rxbytes=sysdelta("rxb")
    dmaerrors=delta("dmaerr")
    skipped=end.get("skip",0)
    times=sorted(ping_rtt_ms or [])
    median_rtt=round(statistics.median(times),2) if times else None
    p95_rtt=round(times[max(0, (95*len(times)+99)//100-1)],2) if times else None
    checks={
       "no_tx_drops":dropped==0,
       "no_frame_loss_on_host":gaps==0 and produced is not None,
       "generation_rate_at_least_95pct":produced is not None and produced>=0.95*rate*time_s,
       "no_missed_generator_deadlines":skipped==0,
       "queue_peak_below_75pct":end.get("peak",TX_RING_BYTES)>=0 and
                                end.get("peak",TX_RING_BYTES)<=MAX_RING_PEAK,
       "total_uart_cpu_below_3pct":cpu_pct<=MAX_PRN_CPU_PERCENT,
       "no_control_deadline_slip":late==0 and gap==0,
       "no_gyro_i2c_fault":stale==0 and i2cerr==0 and timeout==0,
       "no_uart_rx_fault":rxerror==0 and rxoverflow==0,
       "no_uart_dma_error":dmaerrors==0,
       "pa10_rx_bytes_accounted":ping_sent==0 or
                                (rxbytes is not None and rxbytes>=5*ping_sent),
       "pa10_ping_acks_complete":ping_sent==0 or ping_acked==ping_sent,
    }
    return {
      "requested_hz":rate, "observed_probe_hz":measured,
      "produced_probes":produced, "host_verified_probes":len(seen),
      "host_probe_sequence_missing":gaps,
      "skipped_deadlines":skipped,
      "duration_s":round(time_s,3),
      "host_total_received_bytes":raw_bytes,
      "host_received_Bps":round(raw_bytes/max(time_s,1e-3),1),
      "tx_dma_enqueued_bytes":accepted,"tx_dma_completed_bytes":dma_bytes,
      "transferred_Bps_nominal":round((dma_bytes or 0)/max(time_s,1e-3),1),
      "tx_dropped_messages":dropped,
      "tx_buffer_peak_bytes":end.get("peak"),
      "cpu_uart_cycles":cpu_cycles,
      "cpu_uart_pct_nominal_96m":round(cpu_pct,3),
      "cpu_printf_pct":round(100*(delta("fmtcy") or 0)/(SYSCLK_NOMINAL*max(time_s,1e-3)),3),
      "cpu_dma_irq_pct":round(100*(delta("dmacy") or 0)/(SYSCLK_NOMINAL*max(time_s,1e-3)),3),
      "cpu_rx_irq_pct":round(100*(delta("rxcy") or 0)/(SYSCLK_NOMINAL*max(time_s,1e-3)),3),
      "tim11_late_increase":late,
      "tim11_gaps_increase":gap,
      "gyro_stale_increase":stale,
      "i2c_error_increase":i2cerr,
      "i2c_timeout_increase":timeout,
      "rx_error_increase":rxerror,
      "rx_overflow_increase":rxoverflow,
      "rx_accepted_bytes_increase":rxbytes,
      "uart_dma_error_increase":dmaerrors,
      "host_ping_commands_sent":ping_sent,
      "mcu_ping_acks_received":ping_acked,
      "ping_host_rtt_p50_ms":median_rtt,
      "ping_host_rtt_p95_ms":p95_rtt,
      "ping_host_rtt_max_ms":round(times[-1],2) if times else None,
      "checks":checks,"stable":all(checks.values()),
    }


class Console:
    def __init__(self, port, outdir, baud=BAUD):
        import serial
        self.ser=serial.Serial(port, baud, timeout=0.06, write_timeout=0.6)
        self.raw=(outdir/"uart_profile_raw.log").open("w",encoding="utf-8")
        self.entries=[]
        self.bytes=0
        self.created=time.monotonic()
        self.ping_pending=deque()
        self.ping_rtt_ms=[]
    def send(self,cmd):
        self.ser.write((cmd+"\n").encode("ascii"))
        self.ser.flush()
        if cmd=="PING":self.ping_pending.append(time.monotonic())
    def read(self):
        packet=self.ser.readline()
        if not packet:return None
        self.bytes+=len(packet)
        raw=packet.decode("utf-8",errors="replace").strip()
        t=time.monotonic()-self.created
        self.raw.write(f"{t:.4f}\t{raw}\n")
        item=parse_packet(raw)
        if item:
            item["host_s"]=round(t,4)
            self.entries.append(item)
            if item.get("type")=="CMD_PING" and self.ping_pending:
                self.ping_rtt_ms.append((time.monotonic()-self.ping_pending.popleft())*1000)
        return item
    def until(self, pred, seconds=5):
        limit=time.monotonic()+seconds
        while time.monotonic()<limit:
            p=self.read()
            if p and pred(p):return p
        raise TimeoutError("UART missing awaited response")
    def status(self):
        # Benchmark source must be stopped or low-rate; after TXBENCH 0
        # the 1024-byte queue normally drains within ~100ms.
        self.send("TXSTAT")
        tx=self.until(lambda x:x.get("type")=="U_TX",5)
        sy=self.until(lambda x:x.get("type")=="U_SYS" and
                      x.get("ms",0)>=tx.get("ms",0),5)
        return tx,sy
    def calibrate(self):
        until=time.monotonic()+55.0
        while time.monotonic()<until:
            self.send("STATUS")
            nxt=time.monotonic()+.7
            while time.monotonic()<nxt:
                p=self.read()
                if p and p.get("type")=="CMD_STATUS":
                    if p.get("locked") != 1:raise RuntimeError("Motor outputs NOT locked; stop")
                    if p.get("cal")==3:return
                    if p.get("cal")==4:raise RuntimeError("MPU calibration failed")
        raise TimeoutError("MPU calibration not complete; no UART stress started")
    def stop(self):
        try:
            self.send("TXBENCH 0")
            self.send("!")
        except Exception:pass
    def close(self):
        self.ser.close()
        self.raw.close()


def flash():
    subprocess.run([sys.executable,"-m","platformio","run",
                    "-e","integration_diag_uart_96m","-t","upload"],
                   check=True,cwd=ROOT,timeout=240)


def run(port,output=None,upload=False,rates=RATES,duration=PHASE_SEC):
    outdir=output or ROOT/"results"/("uart_rate_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    outdir.mkdir(parents=True,exist_ok=True)
    if upload:flash()
    con=None
    rows=[]
    report=None
    try:
        con=Console(port,outdir)
        con.calibrate()
        con.send("TXBENCH 0")
        con.until(lambda x:x.get("type")=="U_ACK" and x.get("hz")==0,4)
        start,sys_start=con.status()
        print(f"UART1: 115200 8N1, theoretical cap {BAUD//10} B/s; real TX DMA profiling",flush=True)
        for rate in rates:
            con.send("TXBENCH "+str(rate))
            ack=con.until(lambda x:x.get("type")=="U_ACK" and x.get("hz")==rate,4)
            if ack.get("ok")!=1:raise RuntimeError(f"UART bench refused rate {rate}: {ack}")
            start_host=time.monotonic()
            first_packet_idx=len(con.entries)
            first_bytes=con.bytes
            first_latency_idx=len(con.ping_rtt_ms)
            next_ping=time.monotonic()
            ping_sent=0
            while time.monotonic()-start_host<duration:
                if time.monotonic()>=next_ping:
                    con.send("PING")  # PA10 RX load: 10 commands/s, ACK on PA9.
                    ping_sent+=1
                    next_ping+=0.1
                x=con.read()
                if x and x.get("type")=="U_ABORT":
                    raise RuntimeError(f"MCU benchmark aborted at {rate}Hz: {x}")
            actual_duration=time.monotonic()-start_host
            live_rx_bytes=con.bytes-first_bytes
            con.send("TXBENCH 0")
            con.until(lambda x:x.get("type")=="U_ACK" and x.get("hz")==0,5)
            # Wait for complete DMA queue drain before final counter snapshot;
            # no probing continues after stopping. This matters at overload.
            drain_until=time.monotonic()+1.3
            while time.monotonic()<drain_until:con.read()
            end,sys_end=con.status()
            segment=con.entries[first_packet_idx:]
            probes=[x for x in segment if x.get("type")=="U_PROBE"]
            ping_acked=sum(1 for x in segment if x.get("type")=="CMD_PING")
            score=assess(rate,end.get("seq"),probes,start,end,
                         sys_start,sys_end,actual_duration,live_rx_bytes,
                         ping_sent,ping_acked,
                         con.ping_rtt_ms[first_latency_idx:])
            rows.append(score)
            print(f"{rate:>3}Hz: delivered={score['host_verified_probes']}/{score['produced_probes']} "
                  f"rate={score['observed_probe_hz']}Hz "
                  f"peakQ={score['tx_buffer_peak_bytes']}B drops={score['tx_dropped_messages']} "
                  f"CPU={score['cpu_uart_pct_nominal_96m']}% "
                  f"TIM11late={score['tim11_late_increase']} "
                  f"{'PASS' if score['stable'] else 'LIMIT'}",flush=True)
            start,sys_start=end,sys_end
        passed=[x for x in rows if x["stable"]]
        # The last tested contiguous PASS, never an isolated high-rate outlier.
        isolated=max((x["requested_hz"] for x in passed),default=None)
        prefix=[]
        for x in rows:
            if not x["stable"]:break
            prefix.append(x["requested_hz"])
        safe=max(prefix,default=None)
        recommended=(max((x for x in prefix if x<=0.8*safe),default=prefix[0])
                     if safe and prefix else None)
        # A tested rate, NOT a thermal or motor-on safety guarantee.
        report={"baud":BAUD,"framing":"8N1",
                "theoretical_line_bytes_per_second":BAUD//10,
                "profile_name":"96MHz_HSI_TRIM25_motor_off_all_sensors_active",
                "candidate_rates_hz":list(rates),
                "requested_sample_s":duration,
                "highest_individual_pass_hz":isolated,
                "chosen_tested_sustainable_hz":safe,
                "recommended_log_rate_with_20pct_headroom_hz":recommended,
                "admission_requirements":{
                    "tx_drops":0,"sequence_gaps":0,
                    "queue_peak_bytes_max":MAX_RING_PEAK,
                    "uart_cpu_pct_of_nominal_96MHz_max":MAX_PRN_CPU_PERCENT,
                    "TIM11_irq_jitter_late_count":0,
                    "gyro_and_i2c_errors":0},
                "observations":rows,
                "physical_clock_independently_measured":False,
                "motor_on_stress_tested":False,
                "note":"Run main long motor PWM pipeline separately before qualifying concurrency under motor loads"}
        return report
    finally:
        if con:
            con.stop()
            con.close()
        (outdir/"uart_rate_rows.csv").write_text(
            "" if not rows else to_csv(rows),encoding="utf-8")
        if report:
            (outdir/"uart_rate_summary.json").write_text(
                json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
        print("Saved UART profile results:",outdir,flush=True)


def to_csv(rows):
    import io
    buffer=io.StringIO()
    simple=[{k:v for k,v in r.items() if not isinstance(v,dict)} for r in rows]
    if simple:
        writer=csv.DictWriter(buffer,fieldnames=list(simple[0]))
        writer.writeheader()
        writer.writerows(simple)
    return buffer.getvalue()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port",required=True,help="USART1 USB-UART adapter, e.g. COM6")
    p.add_argument("--flash",action="store_true",help="Flash motor-locked 96MHz UART profiling firmware first")
    p.add_argument("--output",type=Path,help="Output directory")
    p.add_argument("--duration",type=float,default=PHASE_SEC,
                   help="Measurement seconds per requested Hz; default 3")
    p.add_argument("--rates",default=",".join(map(str,RATES)),
                   help="Comma-separated send frequencies in frames/second")
    args=p.parse_args()
    try:
        rates=tuple(int(x) for x in args.rates.split(","))
    except ValueError:p.error("Bad --rates")
    if not rates or any(x<5 or x>400 for x in rates):p.error("Rates must be 5..400 Hz")
    if args.duration<2 or args.duration>10:p.error("--duration must be 2..10 seconds")
    r=run(args.port,args.output,args.flash,rates,args.duration)
    print("Highest PASS frequency among TESTED rates:",
          r["chosen_tested_sustainable_hz"],"Hz")
    return 0 if r["chosen_tested_sustainable_hz"] else 2


if __name__=="__main__":
    raise SystemExit(main())
