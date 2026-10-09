#!/usr/bin/env python3
"""Strict, machine-readable evaluator for repeated PID flight trials."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path


TARGETS = {
    "hover_xy_rms_m": 0.04,
    "hover_xy_max_m": 0.10,
    "hover_z_rms_m": 0.02,
    "hover_z_peak_to_peak_m": 0.04,
    "hover_yaw_peak_to_peak_deg": 2.0,
    "hover_ground_xy_speed_rms_mps": 0.035,
    "hover_roll_rms_deg": 0.35,
    "hover_pitch_rms_deg": 0.35,
    "hover_estimator_xy_rms_m": 0.05,
    "worst_stop_displacement_m": 0.40,
    "worst_z_rise_after_stop_m": 0.02,
    "worst_abs_z_displacement_after_stop_m": 0.15,
    "mean_settling_time_s": 6.0,
}

WEIGHTS = {
    "hover_xy_rms_m": 2.0,
    "hover_xy_max_m": 1.0,
    "hover_z_rms_m": 1.5,
    "hover_z_peak_to_peak_m": 0.75,
    "hover_yaw_peak_to_peak_deg": 1.5,
    "hover_ground_xy_speed_rms_mps": 1.5,
    "hover_roll_rms_deg": 0.5,
    "hover_pitch_rms_deg": 0.5,
    "hover_estimator_xy_rms_m": 0.5,
    "worst_stop_displacement_m": 2.0,
    "worst_z_rise_after_stop_m": 0.75,
    "worst_abs_z_displacement_after_stop_m": 0.75,
    "mean_settling_time_s": 1.0,
}


def trial_metrics(data: dict) -> dict[str, float]:
    settling = [
        float(stop["settling_time_s"])
        for stop in data.get("stops", {}).values()
        if stop.get("settling_time_s") is not None
    ]
    missing_settles = len(data.get("stops", {})) - len(settling)
    settling.extend([7.0] * missing_settles)
    z_rise = float(data["aggregate"]["worst_z_rise_after_stop_m"])
    z_drop = float(data["aggregate"].get("worst_z_drop_after_stop_m", 0.0))
    return {
        "hover_xy_rms_m": float(data["hover"]["xy_rms_m"]),
        "hover_xy_max_m": float(data["hover"]["xy_max_m"]),
        "hover_z_rms_m": float(data["hover"]["z_rms_m"]),
        "hover_z_peak_to_peak_m": float(data["hover"]["z_peak_to_peak_m"]),
        "hover_yaw_peak_to_peak_deg": float(data["hover"]["yaw_peak_to_peak_deg"]),
        "hover_ground_xy_speed_rms_mps": float(data["hover"].get("ground_xy_speed_rms_mps", 0.0)),
        "hover_roll_rms_deg": float(data["hover"].get("roll_rms_deg", 0.0)),
        "hover_pitch_rms_deg": float(data["hover"].get("pitch_rms_deg", 0.0)),
        "hover_estimator_xy_rms_m": float(data["hover"].get("estimator_xy_rms_m", 0.0)),
        "worst_stop_displacement_m": float(data["aggregate"]["worst_stop_displacement_m"]),
        "worst_z_rise_after_stop_m": float(data["aggregate"]["worst_z_rise_after_stop_m"]),
        "worst_abs_z_displacement_after_stop_m": max(abs(z_rise), abs(z_drop)),
        "mean_settling_time_s": statistics.fmean(settling) if settling else 7.0,
    }


def evaluate(paths: list[Path]) -> dict:
    if not paths:
        raise ValueError("at least one PID trial is required")
    repeats = [trial_metrics(json.loads(path.read_text())) for path in paths]
    means = {name: statistics.fmean(item[name] for item in repeats) for name in TARGETS}
    standard_deviations = {
        name: statistics.pstdev(item[name] for item in repeats) for name in TARGETS
    }
    normalized_cost = sum(
        WEIGHTS[name] * (means[name] / target) ** 2 for name, target in TARGETS.items()
    ) / sum(WEIGHTS.values())
    repeatability_penalty = sum(
        WEIGHTS[name] * standard_deviations[name] / max(TARGETS[name], 1.0e-9)
        for name in TARGETS
    ) / sum(WEIGHTS.values())
    score = normalized_cost + 0.35 * repeatability_penalty
    finite = math.isfinite(score) and all(
        math.isfinite(value) for value in means.values()
    )
    gates = {
        "hover_xy_rms": means["hover_xy_rms_m"] <= 0.04,
        "hover_xy_envelope": max(
            item["hover_xy_max_m"] for item in repeats
        ) <= 0.10,
        "hover_z_rms": means["hover_z_rms_m"] <= 0.025,
        "yaw_peak_to_peak": means["hover_yaw_peak_to_peak_deg"] <= 2.5,
        "stop_displacement": max(
            item["worst_stop_displacement_m"] for item in repeats
        ) <= 0.50,
        "post_stop_climb": means["worst_z_rise_after_stop_m"] <= 0.025,
        "post_stop_vertical_displacement": max(
            item["worst_abs_z_displacement_after_stop_m"] for item in repeats
        ) <= 0.15,
        "settling_time": means["mean_settling_time_s"] <= 7.0,
    }
    safety_gates = {
        "bounded_xy": means["hover_xy_rms_m"] <= 0.50,
        "bounded_z": means["hover_z_rms_m"] <= 0.08,
        "bounded_yaw": means["hover_yaw_peak_to_peak_deg"] <= 15.0,
        "bounded_stop": means["worst_stop_displacement_m"] <= 2.0,
        "bounded_post_stop_climb": means["worst_z_rise_after_stop_m"] <= 0.20,
        "bounded_post_stop_vertical_displacement": means[
            "worst_abs_z_displacement_after_stop_m"
        ] <= 0.30,
    }
    return {
        "pass": finite and all(gates.values()),
        "safe": finite and all(safety_gates.values()),
        "score": round(float(score), 8),
        "repeat_count": len(repeats),
        "metrics_mean": means,
        "metrics_stddev": standard_deviations,
        "metrics_worst": {name: max(item[name] for item in repeats) for name in TARGETS},
        "gates": gates,
        "safety_gates": safety_gates,
        "trial_files": [str(path) for path in paths],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trials", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = evaluate(args.trials)
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    print(encoded, end="")


if __name__ == "__main__":
    main()
