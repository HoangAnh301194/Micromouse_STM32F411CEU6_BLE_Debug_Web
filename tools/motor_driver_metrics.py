"""Offline Motor Driver measurements; never opens a port or changes firmware."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import statistics

import motor_auto_pipeline as motor_auto
import motor_functional_trace_pipeline as functional_trace


@dataclass(frozen=True)
class MetricOptions:
    min_repeats: int = 10
    plateau_min_ms: int = 300
    plateau_start_ms: int = 0
    plateau_max_drift_pct: float = 5.0
    max_sample_gap_ms: int = 40
    min_counts_per_interval: int = 10


def distribution(values: list[float]) -> dict:
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Non-finite measurement")
    mean = statistics.mean(values) if values else None
    deviation = statistics.stdev(values) if len(values) >= 2 else None
    return {
        "n": len(values), "mean": mean, "sample_std": deviation,
        "cv_pct": 100.0 * deviation / abs(mean)
        if deviation is not None and mean is not None and mean != 0 else None,
    }


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * percent / 100.0
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (rank - lower)


def trace_metrics(content: str) -> dict:
    result = functional_trace.analyze_text(content)
    samples = [functional_trace.fields(line.strip(), "MFD")
               for line in content.splitlines() if line.strip().startswith("MFD,")]
    periods = [(later["us"] - earlier["us"]) & 0xFFFFFFFF
               for earlier, later in zip(samples, samples[1:])]
    jitter = [abs(period - 1000) for period in periods]
    result.update({"period_count": len(periods),
                   "absolute_jitter_p95_us": percentile(jitter, 95),
                   "absolute_jitter_p99_us": percentile(jitter, 99),
                   "absolute_jitter_max_us": max(jitter)})
    return result


def validate_samples(rows: list[dict], options: MetricOptions) -> None:
    seen = set()
    previous_by_phase = {}
    for row in rows:
        key = (row["round"], row["stage"], row["phase"], row["elapsed"])
        if key in seen:
            raise ValueError("Duplicate motor sample")
        seen.add(key)
        phase = key[:3]
        previous = previous_by_phase.get(phase)
        timestamp = int(row["ms"])
        if not 0 <= timestamp <= 0xFFFFFFFF:
            raise ValueError("MCU millisecond timestamp outside uint32")
        if previous is not None:
            elapsed = int(row["elapsed"]) - int(previous["elapsed"])
            period = (timestamp - int(previous["ms"])) & 0xFFFFFFFF
            if not 0 < period <= options.max_sample_gap_ms or elapsed <= 0:
                raise ValueError("Nonmonotonic timestamp or missing motor samples")
            if abs(period - elapsed) > 2:
                raise ValueError("Elapsed time and MCU timestamp disagree")
        previous_by_phase[phase] = row


def speed_metrics(rows: list[dict], wheel: str, command: int,
                  pulse_ms: int, options: MetricOptions) -> tuple[dict, list[dict]]:
    drive = [row for row in rows if row["phase"] == "drive"
             and int(row["elapsed"]) < pulse_ms]
    direction = 1 if command > 0 else -1
    windows = [[]]
    for previous, current in zip(drive, drive[1:]):
        if (int(previous[f"applied_{wheel}"]) != command or
            int(current[f"applied_{wheel}"]) != command or
            int(previous["elapsed"]) < options.plateau_start_ms):
            if windows[-1]:
                windows.append([])
            continue
        period = (int(current["ms"]) - int(previous["ms"])) & 0xFFFFFFFF
        if not 0 < period <= options.max_sample_gap_ms:
            raise ValueError("Invalid speed interval")
        delta = direction * (int(current[f"enc_{wheel}"]) - int(previous[f"enc_{wheel}"]))
        windows[-1].append({"elapsed_ms": int(current["elapsed"]), "interval_ms": period,
                       "delta_counts": delta, "speed_pps": 1000.0 * delta / period})
    speeds = max(windows, key=lambda window: sum(row["interval_ms"] for row in window))
    span = sum(row["interval_ms"] for row in speeds)
    result = {"status": "INSUFFICIENT_PLATEAU", "span_ms": span,
              "interval_count": len(speeds), "cv_speed_pct": None,
              "window_selection": "constant applied duty, duration, drift and count-resolution heuristic"}
    if len(speeds) < 5 or span < options.plateau_min_ms:
        return result, speeds
    values = [row["speed_pps"] for row in speeds]
    mean = statistics.mean(values)
    if mean <= 0:
        result["status"] = "NO_FORWARD_RESPONSE"
        return result, speeds
    section = max(1, len(values) // 3)
    drift = 100.0 * (statistics.mean(values[-section:]) - statistics.mean(values[:section])) / abs(mean)
    result.update({"mean_speed_pps": mean, "drift_pct": drift,
                   "one_count_step_relative_pct": max(100000.0 / row["interval_ms"] / mean for row in speeds)})
    if abs(drift) > options.plateau_max_drift_pct:
        result["status"] = "NONSTATIONARY_WINDOW"
    elif min(abs(row["delta_counts"]) for row in speeds) < options.min_counts_per_interval:
        result["status"] = "LOW_COUNT_RESOLUTION"
    else:
        result.update({"status": "MEASURED_PLATEAU_CANDIDATE",
                       "cv_speed_pct": distribution(values)["cv_pct"],
                       "speed_p05_pps": percentile(values, 5),
                       "speed_p95_pps": percentile(values, 95)})
    return result, speeds


def analyze_logs(paths: list[Path], options: MetricOptions) -> dict:
    sources, errors, trials, samples, speed_samples = [], [], [], [], []
    groups = {}
    signatures = set()
    startup = []
    for path in paths:
        try:
            content = path.read_text(encoding="utf-8-sig")
            lines = content.splitlines()
            frames = [motor_auto.split_record(line) for line in lines]
            semantic_lines = []
            for line in lines:
                frame = line.split("\t", 1)[-1].strip()
                if frame.startswith("MOTOR_AUTO_"):
                    keys = [part.split("=", 1)[0] for part in frame.split(",")[1:] if "=" in part]
                    if len(keys) != len(set(keys)):
                        raise ValueError("Duplicate UART field")
                    semantic_lines.append(frame)
            signature = hashlib.sha256("\n".join(semantic_lines).encode()).hexdigest()
            if signature in signatures:
                raise ValueError("Repeated capture: refusing to count the same experiment twice")
            profiles = [fields for name, fields in frames if name == "MOTOR_AUTO_PROFILE"]
            versions = {int(profile["version"]) for profile in profiles}
            if len(versions) != 1:
                raise ValueError("Missing or mixed motor firmware profiles")
            version = versions.pop()
            if version not in (4, 6, 7, 8, 14, 16, 17, 18):
                raise ValueError("Unsupported motor firmware protocol")
            if any(int(profile.get("p0_locked", 0)) == 1 for profile in profiles):
                raise ValueError("Locked firmware cannot supply a powered motor experiment")
            motor_auto.configure_from_saved_log(lines, high=version in (6, 8, 16, 18),
                                                 sensors=version in (7, 8, 17, 18))
            base, rows = motor_auto.analyze(lines)
            if base["status"] != "TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL":
                raise ValueError("; ".join(base["errors"][:6]))
            validate_samples(rows, options)
            if any(reading[field] is None for trial in base["trials"]
                   for reading in trial["wheel_readings"]
                   for field in ("encoder_drive", "encoder_after_post_ms", "post_stop_delta")):
                raise ValueError("Missing measured END/POST_END encoder count")
            signatures.add(signature)
            source_id = hashlib.sha256(path.read_bytes()).hexdigest()
            sources.append({"path": str(path), "sha256": source_id, "profile_version": version,
                            "status": "MEASURED_SAVED_LOG", "image_identity_verified": False,
                            "sensor_integration": base["sensor_integration"]})
            commands = {(str(trial["round"]), str(trial["stage"])):
                        (trial["cmd_left"], trial["cmd_right"]) for trial in base["trials"]}
            samples.extend({"source_sha256": source_id, "profile_version": version,
                            "requested_l": commands[(row["round"], row["stage"])][0],
                            "requested_r": commands[(row["round"], row["stage"])][1], **row} for row in rows)
            startup.append({"source_sha256": source_id, **base["phase_findings"]["startup_threshold_heuristic"]})
            for trial in base["trials"]:
                selected = [row for row in rows if int(row["round"]) == trial["round"]
                            and int(row["stage"]) == trial["stage"]]
                pulse_ms = motor_auto.stage_pulse_ms(trial["stage"])
                for reading in trial["wheel_readings"]:
                    wheel = reading["wheel"]
                    command = reading["requested_permille"]
                    if command == 0:
                        continue
                    steady, speeds = speed_metrics(selected, wheel, command, pulse_ms, options)
                    identity = {"source_sha256": source_id, "round": trial["round"],
                                "stage": trial["stage"], "wheel": wheel}
                    speed_samples.extend({**identity, **speed} for speed in speeds)
                    post = [row for row in selected if row["phase"] == "post"]
                    positions = [reading["encoder_drive"], *[int(row[f"enc_{wheel}"]) for row in post],
                                 reading["encoder_after_post_ms"]]
                    stop_path = sum(abs(later - earlier) for earlier, later in zip(positions, positions[1:]))
                    trials.append({**identity, "test": trial["test"], "command_permille": command,
                                   "pulse_ms": pulse_ms, "steady_speed": steady, **reading,
                                   "post_stop_observed_path_counts": stop_path})
                    key = (version, wheel, trial["test"], trial["cmd_left"], trial["cmd_right"],
                           trial["rise_rate_permille_per_ms"], pulse_ms)
                    group = groups.setdefault(key, {"profile_version": version, "wheel": wheel,
                        "test": trial["test"], "command_left": trial["cmd_left"],
                        "command_right": trial["cmd_right"], "rise_permille_per_ms": trial["rise_rate_permille_per_ms"],
                        "pulse_ms": pulse_ms, "counts": [], "successful_starts": 0,
                        "first_five_counts_ms": [], "post_stop_counts": []})
                    group["counts"].append(reading["encoder_drive"])
                    group["successful_starts"] += int(reading["sustained_count_signal"])
                    if reading["first_five_counts_ms"] is not None:
                        group["first_five_counts_ms"].append(reading["first_five_counts_ms"])
                    group["post_stop_counts"].append(reading["post_stop_delta"])
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append({"path": str(path), "error": str(error)})
    repeats = []
    for group in groups.values():
        counts = distribution(group["counts"])
        repeats.append({**group, "count_statistics": counts,
                        "first_five_count_statistics_ms": distribution(group["first_five_counts_ms"]),
                        "post_stop_statistics_counts": distribution(group["post_stop_counts"]),
                        "status": "MEASURED_SAVED_LOG" if counts["n"] >= options.min_repeats else "LOW_REPEATS",
                        "setup_verified": False})
    slew_pairs, dual_pairs = [], []
    for previous in repeats:
        for current in repeats:
            common = ("profile_version", "wheel", "command_left", "command_right", "pulse_ms")
            if all(previous[key] == current[key] for key in common):
                if previous["test"] == "slew_slow" and current["test"] == "slew_fast":
                    slew_pairs.append({**{key: current[key] for key in common},
                        "rise_slow": previous["rise_permille_per_ms"], "rise_fast": current["rise_permille_per_ms"],
                        "mean_counts_slow": previous["count_statistics"]["mean"],
                        "mean_counts_fast": current["count_statistics"]["mean"]})
            if (previous["wheel"] == "l" and current["wheel"] == "r" and
                all(previous[key] == current[key] for key in ("profile_version", "test", "command_left",
                    "command_right", "rise_permille_per_ms", "pulse_ms")) and
                current["command_left"] != 0 and current["command_right"] != 0):
                left, right = previous["count_statistics"]["mean"], current["count_statistics"]["mean"]
                dual_pairs.append({"profile_version": current["profile_version"],
                                   "command_left": current["command_left"], "command_right": current["command_right"],
                                   "relative_bias_pct": 200 * (left - right) / (abs(left) + abs(right))
                                   if abs(left) + abs(right) else None})
    return {"sources": sources, "errors": errors, "trials": trials, "repeat_statistics": repeats,
            "startup_heuristics_by_capture": startup, "rise_slew_comparisons": slew_pairs,
            "dual_wheel_comparisons": dual_pairs, "samples": samples, "speed_samples": speed_samples,
            "limits": "Saved logs do not verify the flashed image, battery, load, fall rate or deadband setup. No automatic calibration or current/temperature estimate."}
