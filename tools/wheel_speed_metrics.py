"""Wheel metrics adapted from giomuadongbac tools/motion_experiment.py (2d07838)."""
import math
from typing import Any

def _wheel_metrics(
    t_ms: list[float],
    refs: list[float],
    meas: list[float],
    pwm_cmd: list[float],
    pwm_max_pm: float,
) -> dict[str, Any]:
    n = len(t_ms)
    if n == 0:
        return {"samples": 0, "verdict": "NO_DATA"}

    active_idx = [i for i in range(n) if abs(refs[i]) > 1e-3]
    if not active_idx:
        # Idle or zero reference: only report mean meas and PWM check
        nonzero_pwm = sum(1 for p in pwm_cmd if abs(p) > 0)
        return {
            "samples": n,
            "mean_ref_pps": 0.0,
            "mean_meas_pps": round(sum(meas) / n, 2),
            "steady_state_error_pps": None,
            "steady_state_error_pct": "N/A (ref=0)",
            "mae_pps": round(sum(abs(m) for m in meas) / n, 2),
            "rmse_pps": round(math.sqrt(sum(m * m for m in meas) / n), 2),
            "max_abs_error_pps": round(max(abs(m) for m in meas), 2),
            "rise_time_10_90_ms": "N/A (ref=0)",
            "overshoot_pct": "N/A (ref=0)",
            "settling_time_5pct_ms": "N/A (ref=0)",
            "steady_std_pps": "N/A (ref=0)",
            "pwm_sat_pct": round(100.0 * sum(1 for p in pwm_cmd if abs(p) >= pwm_max_pm - 1.0) / n, 2),
            "nonzero_pwm_when_ref_zero": nonzero_pwm,
        }

    # Find longest contiguous segment with near-constant reference for step/steady analysis
    best_seg: list[int] = []
    cur_seg: list[int] = [active_idx[0]]
    for idx in active_idx[1:]:
        if idx == cur_seg[-1] + 1 and abs(refs[idx] - refs[cur_seg[-1]]) <= max(5.0, 0.02 * abs(refs[cur_seg[-1]])):
            cur_seg.append(idx)
        else:
            if len(cur_seg) > len(best_seg):
                best_seg = cur_seg
            cur_seg = [idx]
    if len(cur_seg) > len(best_seg):
        best_seg = cur_seg

    r_seg = [refs[i] for i in best_seg]
    m_seg = [meas[i] for i in best_seg]
    t_seg = [t_ms[i] for i in best_seg]
    p_seg = [pwm_cmd[i] for i in best_seg]

    mean_ref = sum(r_seg) / len(r_seg)
    errs = [r_seg[k] - m_seg[k] for k in range(len(best_seg))]
    abs_errs = [abs(e) for e in errs]
    mae = sum(abs_errs) / len(abs_errs)
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    max_abs_err = max(abs_errs)

    # Separate transient vs steady-state: last 45% of constant-ref segment (min 5 samples)
    steady_start = max(0, int(0.55 * len(best_seg)))
    steady_m = m_seg[steady_start:]
    steady_r = r_seg[steady_start:]
    mean_steady_ref = sum(steady_r) / len(steady_r)
    mean_steady_meas = sum(steady_m) / len(steady_m)
    ss_err_pps = mean_steady_ref - mean_steady_meas
    ss_err_pct: float | str = (
        round(100.0 * abs(ss_err_pps) / abs(mean_steady_ref), 2)
        if abs(mean_steady_ref) > 1e-3
        else "N/A (ref=0)"
    )
    steady_var = sum((x - mean_steady_meas) ** 2 for x in steady_m) / max(1, len(steady_m) - 1)
    steady_std = math.sqrt(steady_var)

    # Rise time (10% -> 90%) & Overshoot & Settling time (±5%) only for clean step (>=12 samples)
    rise_time_ms: float | str = "N/A (insufficient steady segment)"
    overshoot_pct: float | str = "N/A (insufficient steady segment)"
    settling_ms: float | str = "N/A (insufficient steady segment)"

    if len(best_seg) >= 12 and abs(mean_steady_ref) > 50.0:
        sign = 1.0 if mean_steady_ref > 0 else -1.0
        target_mag = abs(mean_steady_ref)
        m_signed = [sign * x for x in m_seg]
        t0 = t_seg[0]
        t10 = next((t_seg[k] for k in range(len(m_signed)) if m_signed[k] >= 0.1 * target_mag), None)
        t90 = next((t_seg[k] for k in range(len(m_signed)) if m_signed[k] >= 0.9 * target_mag), None)
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_time_ms = round(t90 - t10, 1)
        else:
            rise_time_ms = "N/A (did not reach 90%)"

        peak = max(m_signed)
        overshoot_pct = round(max(0.0, 100.0 * (peak - target_mag) / target_mag), 2)

        band = 0.05 * target_mag
        outside = [k for k in range(len(m_signed)) if abs(m_signed[k] - target_mag) > band]
        if not outside:
            settling_ms = 0.0
        elif outside[-1] < len(m_signed) - 3:
            settling_ms = round(t_seg[outside[-1] + 1] - t0, 1)
        else:
            settling_ms = "N/A (not settled within ±5%)"

    sat_count = sum(1 for p in p_seg if abs(p) >= pwm_max_pm - 1.0)
    nonzero_zero_ref = sum(1 for i in range(n) if abs(refs[i]) <= 1e-3 and abs(pwm_cmd[i]) > 0)

    return {
        "samples": len(best_seg),
        "mean_ref_pps": round(mean_ref, 2),
        "mean_meas_pps": round(mean_steady_meas, 2),
        "steady_state_error_pps": round(ss_err_pps, 2),
        "steady_state_error_pct": ss_err_pct,
        "mae_pps": round(mae, 2),
        "rmse_pps": round(rmse, 2),
        "max_abs_error_pps": round(max_abs_err, 2),
        "rise_time_10_90_ms": rise_time_ms,
        "overshoot_pct": overshoot_pct,
        "settling_time_5pct_ms": settling_ms,
        "steady_std_pps": round(steady_std, 2),
        "pwm_sat_pct": round(100.0 * sat_count / len(p_seg), 2),
        "nonzero_pwm_when_ref_zero": nonzero_zero_ref,
    }



def summarize_wheels(rows, pwm_max_pm=200):
    rows = list(rows)
    out = {}
    for side in ("L", "R"):
        keys = ["ref_"+side, "speed_"+side, "pwm_"+side]
        selected = [r for r in rows if all(k in r for k in keys)]
        out[side] = _wheel_metrics([r["t"]*1000 for r in selected],
                                  [r[keys[0]] for r in selected],
                                  [r[keys[1]] for r in selected],
                                  [r[keys[2]] for r in selected], pwm_max_pm)
        out[side]["pwm_basis"] = "applied"
    active = [r for r in rows if r.get("ref_L",0)>0 and r.get("ref_R",0)>0
              and "speed_L" in r and "speed_R" in r]
    out["lr_mean_abs_diff_pps"] = round(sum(abs(r["speed_L"]-r["speed_R"]) for r in active)/len(active),2) if active else None
    dt = rows[-1]["t"]-rows[0]["t"] if len(rows)>1 else 0
    out["telemetry_hz"] = round((len(rows)-1)/dt,2) if dt>0 else None
    out["samples_total"] = len(rows)
    return out

def save_wheel_summary(csv_path, metrics, result):
    import json
    from pathlib import Path
    path = Path(csv_path).with_suffix(".metrics.json")
    path.write_text(json.dumps({"result":result,"metrics":metrics}, indent=2, ensure_ascii=False), encoding="utf-8")
