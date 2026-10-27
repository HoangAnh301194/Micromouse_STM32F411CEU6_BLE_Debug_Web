#!/usr/bin/env python3
"""Gated P0-P3 Micromouse Encoder qualification.

P0 software/native and motor-OFF hardware health -> P1 capped 0-30% motor-ON
-> P2 complete OFF/ON 0-100% characterization -> P3 10-alpha shortlist.
No automated production PID/EMA parameter change. Real motors REQUIRE opt-in.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OFF_ENV = "integration_diag_encoder_motor_off_96m"
LIMITED_ENV = "integration_diag_encoder_motor_on_limited_30_96m"
BASELINE_OFF_MAX_ENC_CYCLES = 7211   # physical 20261020_083902 motor-OFF


def load_json(p: Path):
    return json.loads(p.read_text(encoding="utf-8"))


def subprocess_live(name, cmd, directory, log, result):
    """Stream and save all subprocess stdout/stderr; return success only on rc=0."""
    print(f"\n===== {name} =====", flush=True)
    print(" ".join(map(str, cmd)), flush=True)
    try:
        with log.open("w", encoding="utf-8") as out:
            with subprocess.Popen(
                list(map(str, cmd)), cwd=ROOT, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, errors="replace",
                bufsize=1
            ) as proc:
                for line in proc.stdout:
                    out.write(line)
                    out.flush()
                    print(line, end="", flush=True)
                rc = proc.wait()
    except (OSError, subprocess.SubprocessError) as exc:
        result[name] = {"status": "ERROR", "reason": str(exc), "log": str(log)}
        raise RuntimeError(f"{name}: {exc}") from exc
    result[name] = {
        "status": "PASS" if rc == 0 else "FAIL",
        "returncode": rc, "log": str(log.relative_to(directory))
    }
    if rc != 0:
        raise RuntimeError(f"{name} command failed (rc={rc}), see {log}")
    return result[name]


def get_phase(report_file, mode):
    if not report_file.is_file():
        raise RuntimeError("Missing mandatory hardware summary: " + str(report_file))
    data = load_json(report_file)
    phases = [x for x in data.get("results", []) if x.get("mode") == mode]
    if len(phases) != 1:
        raise RuntimeError(f"Expected one {mode} report in {report_file}")
    return phases[0]


def strict_phase(phase, stages):
    checks = phase.get("checks", {})
    if (phase.get("stages") != stages or not phase.get("software_pass")
        or not checks or not all(checks.values())
        or phase.get("faults") or phase.get("host_abort")):
        raise RuntimeError(
            f"{phase.get('mode')} stage/health gate failed: "
            f"stage={phase.get('stages')}/{stages}, failures="
            f"{[k for k, v in checks.items() if not v]}, "
            f"faults={phase.get('faults')}, abort={phase.get('host_abort')}"
        )
    return phase


def check_queue(phase):
    q = phase.get("encoder_shadow_queue") or {}
    if (not q or q.get("lost") != 0 or q.get("edge") != 0
        or q.get("peak", 9999) >= 128):
        raise RuntimeError("Encoder shadow FIFO is not lossless or backlog too high: " + repr(q))
    if q.get("in", -1) < 1000 or q.get("out", -1) < 1000:
        raise RuntimeError("Insufficient real 1-kHz FIFO evidence: " + repr(q))
    return q


def check_heartbeat(phase):
    hb = phase.get("host_heartbeat") or {}
    sent, ack = hb.get("ping_sent", 0), hb.get("ping_ack_received", 0)
    if (sent < 100 or ack < sent - 3
        or hb.get("max_inter_ping_gap_ms", 9999) >= 500):
        raise RuntimeError("UART heartbeat not robust enough: " + repr(hb))
    return hb


def check_p0(p):
    off = strict_phase(p, 33)
    t = off.get("timer") or {}
    u = off.get("uart") or {}
    if (t.get("encmaxcy", 999999) >= 2000
        or t.get("maxcy", 999999) >= 3500
        or t.get("encmaxcy", 999999) >= BASELINE_OFF_MAX_ENC_CYCLES
        or u.get("rx_err") != 0 or u.get("tx_drop") != 0):
        raise RuntimeError("P0: Encoder ISR or UART failed timing target: " +
                           repr({"timer": t, "uart": u}))
    return {
        "status": "PASS", "max_encoder_cycles": t["encmaxcy"],
        "max_irq_cycles": t["maxcy"], "baseline_encoder_cycles": BASELINE_OFF_MAX_ENC_CYCLES,
        "improvement_percent": round(
            100 * (1 - t["encmaxcy"] / BASELINE_OFF_MAX_ENC_CYCLES), 2),
        "queue": check_queue(off), "heartbeat": check_heartbeat(off)
    }


def check_p1(on):
    strict_phase(on, 12)
    if on.get("max_pwm") != 30 or on.get("expected_stages") != 12:
        raise RuntimeError("P1 wrong limited firmware contract: " + repr(on))
    return {"status": "PASS", "stages": 12, "compiled_max_pwm_percent": 30,
            "timer": on["timer"], "uart": on["uart"],
            "queue": check_queue(on), "heartbeat": check_heartbeat(on)}


def check_p2(on, off):
    strict_phase(off, 33)
    strict_phase(on, 33)
    if on.get("max_pwm") != 100 or on.get("expected_stages") != 33:
        raise RuntimeError("P2 wrong full firmware contract: " + repr(on))
    return {"status": "PASS", "off_stages": 33, "on_stages": 33,
            "timer": on["timer"], "uart": on["uart"],
            "queue": check_queue(on), "heartbeat": check_heartbeat(on)}


def decide_alpha(report, qualified_p2):
    """Open-loop proxy A/B candidates only. No evidence of closed-loop PID gain."""
    if not qualified_p2 or report.get("status") != "COMPLETE_PROVISIONAL":
        raise RuntimeError("P3 requires complete alpha statistics after P2 passed")
    if (report.get("shadow_bank_version") != "ten_alpha"
        or report.get("candidate_alphas") is None
        or len(report["candidate_alphas"]) != 10
        or report.get("missing_metric_frames_count") != 0
        or report.get("duplicated_frames_count") != 0):
        raise RuntimeError("P3 incomplete/mixed shadow-alphas: " + repr(report.get("issues")))
    overall = report.get("ranking") or []
    wheel = report.get("ranking_by_wheel") or {}
    left = wheel.get("L") or []
    right = wheel.get("R") or []
    overlap = {x["alpha"] for x in left[:3]} & {x["alpha"] for x in right[:3]}
    candidates = [x["alpha"] for x in overall if x["alpha"] in overlap]
    if not candidates:
        return {
            "status": "WHEEL_RANKINGS_DISAGREE",
            "production_alpha": 0.30,
            "decision": "Do not change PID or production EMA; wheel rankings conflict.",
            "wheel_top3": {"L": left[:3], "R": right[:3]},
        }
    return {
        "status": "PROVISIONAL_FOR_PID_FOLLOWUP",
        "candidate_alpha": candidates[0],
        "alternate_alpha": next((a for a in candidates[1:]), None),
        "production_alpha": 0.30,
        "wheel_top3": {"L": left[:3], "R": right[:3]},
        "comparison": overall,
        "decision": ("Selected using the SAME ENCODER's 20ms count proxy. "
                     "A separate controlled closed-loop speed/PID test and "
                     "repeatability evidence are required BEFORE firmware changes."),
        "automatic_pid_changes": False,
    }


def gate_sequence(report):
    return (report.get("P0", {}).get("status") == "PASS" and
            report.get("P1", {}).get("status") == "PASS")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--outdir", type=Path)
    parser.add_argument("--fixture-ready", action="store_true",
                        help="Wheels securely raised/guarded, operator stays present.")
    parser.add_argument("--independent-vm-cutoff", action="store_true",
                        help="Operator confirms an independent motor supply cutoff.")
    parser.add_argument("--allow-100-percent", action="store_true",
                        help="Explicitly authorize advancing from passing 30pct gate to 100pct.")
    parser.add_argument("--offline-only", action="store_true",
                        help="Software/build preflight only; never open COM or energize motors.")
    args = parser.parse_args(argv)
    if not args.offline_only and not (
        args.fixture_ready and args.independent_vm_cutoff and args.allow_100_percent
    ):
        parser.error("Physical run requires --fixture-ready --independent-vm-cutoff "
                     "--allow-100-percent")
    if args.offline_only and (args.fixture_ready or args.independent_vm_cutoff
                              or args.allow_100_percent):
        parser.error("--offline-only does not accept physical authorization flags")
    out = args.outdir or ROOT / "results" / (
        "encoder_p0_p3_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True, exist_ok=False)
    report = {
        "pipeline": "P0 software+OFF / P1 <=30% / P2 full / P3 ten-alpha",
        "mode": "OFFLINE_ONLY" if args.offline_only else "PHYSICAL_SUPERVISED",
        "status": "STARTED", "P0": {"status": "NOT_RUN"},
        "P1": {"status": "NOT_RUN"}, "P2": {"status": "NOT_RUN"},
        "P3": {"status": "NOT_RUN"}, "commands": {},
        "production_alpha_changed": False,
    }
    failed = False
    try:
        # Gate 0: all existing native, static, offline regressions + both builds.
        subprocess_live("preflight",
            [sys.executable, "tools/encoder_qualification_pipeline.py",
             "--mode", "offline", "--outdir", out / "preflight"],
            out, out / "preflight.log", report["commands"])
        subprocess_live("build_limited",
            [sys.executable, "-m", "platformio", "run", "-c", "platformio_legacy.ini", "-e", LIMITED_ENV],
            out, out / "build_limited.log", report["commands"])
        if args.offline_only:
            report["status"] = "SOFTWARE_ONLY_PASS"
            return 0
        # OFF (P0) then limited ON (P1) in one supervised serial session;
        # system_uart_pipeline itself refuses ON if the OFF checks fail.
        subprocess_live("P0_P1_limited",
            [sys.executable, "tools/system_uart_pipeline.py",
             "--port", args.port, "--flash", "--motor-on",
             "--limited-30", "--encoder-telemetry",
             "--off-encmaxcy-limit", "2000",
             "--outdir", out / "P0_P1_limited"],
            out, out / "P0_P1_limited.log", report["commands"])
        file1 = out / "P0_P1_limited" / "summary.json"
        report["P0"] = check_p0(get_phase(file1, "OFF"))
        report["P1"] = check_p1(get_phase(file1, "ON"))
        if not gate_sequence(report):
            raise RuntimeError("P0/P1 gate did not pass; 100pct profile prohibited")

        # Full 0..100% only AFTER BOTH gates pass, with a separate firmware
        # flash and unchanged MCU watchdog/locked STOP behavior.
        subprocess_live("P2_full",
            [sys.executable, "tools/system_uart_pipeline.py",
             "--port", args.port, "--flash", "--motor-on", "--encoder-telemetry",
             "--off-encmaxcy-limit", "2000",
             "--outdir", out / "P2_full"],
            out, out / "P2_full.log", report["commands"])
        file2 = out / "P2_full" / "summary.json"
        report["P2"] = check_p2(get_phase(file2, "ON"), get_phase(file2, "OFF"))

        # Motor tests finished: now perform offline interpretation only.
        from encoder_shadow_ab import analyze
        alpha = analyze(out / "P2_full", out)
        report["P3"] = decide_alpha(alpha, report["P2"]["status"] == "PASS")
        (out / "alpha_pid_decision.json").write_text(
            json.dumps(report["P3"], indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        report["status"] = ("ALL_GATES_PASS_PID_VALIDATION_PENDING" if
                            report["P3"]["status"] == "PROVISIONAL_FOR_PID_FOLLOWUP"
                            else "P0_P2_PASS_P3_UNRESOLVED")
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as exc:
        report["status"] = "HALTED_UNQUALIFIED"
        report["failure"] = str(exc)
        failed = True
        print("GATED PIPELINE HALTED:", exc, file=sys.stderr)
    finally:
        # Never leave the last installed firmware motor-capable intentionally,
        # whether successful, failed, or interrupted. Operator still needs
        # the independent physical VM cutoff.
        if not args.offline_only:
            try:
                subprocess_live("restore_motor_off",
                    [sys.executable, "-m", "platformio", "run",
                     "-c", "platformio_legacy.ini", "-e", OFF_ENV, "-t", "upload"],
                    out, out / "restore_motor_off.log", report["commands"])
                report["restored_motor_off"] = True
            except (OSError, RuntimeError, KeyboardInterrupt) as exc:
                report["restored_motor_off"] = False
                report["status"] = "RESTORE_FAILED_DISCONNECT_VM"
                report["restore_error"] = str(exc)
                failed = True
        (out / "summary.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        (out / "REPORT.md").write_text(
            "# Encoder P0–P3 qualification\n\n"
            f"- Status: **{report['status']}**\n"
            f"- P0: {report['P0']['status']}\n"
            f"- P1: {report['P1']['status']}\n"
            f"- P2: {report['P2']['status']}\n"
            f"- P3: {report['P3']['status']}\n"
            f"- Production alpha changed: **NO**\n"
            f"- Motor-off restoration: {report.get('restored_motor_off', 'N/A')}\n"
            f"- Failure: {report.get('failure', 'None')}\n\n"
            "P3 alpha is an open-loop proxy shortlist, not closed-loop PID "
            "accuracy validation.\n",
            encoding="utf-8")
        print("Qualification evidence:", out, flush=True)
    return 2 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
