#!/usr/bin/env python3
"""Verify that one shared Isaac world sustains a requested drone count."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime

from parallel_pid_optimizer import RESULTS, ROOT, run_batch
from pid_optimizer import atomic_json, load_persisted_baseline


def capacity_passed(result: dict, requested_drones: int) -> bool:
    """Capacity concerns completed vehicles and timing, not controller quality."""
    return (
        result["replica_count"] == requested_drones
        and result["timing"]["real_time_factor_worst"] >= 0.95
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", type=int, default=10)
    parser.add_argument("--run-id", default=datetime.now().strftime("capacity_%Y%m%d_%H%M%S"))
    parser.add_argument("--protocol", choices=("hover", "basic", "motion", "stress"), default="hover")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--high-speed", type=float, default=3.0)
    parser.add_argument("--warmup-seconds", type=float, default=4.0)
    parser.add_argument("--hover-seconds", type=float, default=10.0)
    args = parser.parse_args()
    if not 1 <= args.drones <= 16:
        parser.error("drones must be between 1 and 16")

    config = json.loads((ROOT / "docker/flight_control/pid_search_space.json").read_text())
    parameter_path = ROOT / "docker/flight_control/isaac_sim_tuned.parm"
    parameters = load_persisted_baseline(config, parameter_path)
    output = RESULTS / args.run_id
    if output.exists():
        raise RuntimeError(f"Run already exists: {output}")
    started = time.monotonic()
    ranking = run_batch(
        output,
        [{"name": "capacity_incumbent", "parameters": parameters}],
        parameter_path.read_text(),
        args.protocol,
        args.speed,
        args.high_speed,
        args.drones,
        args.warmup_seconds,
        args.hover_seconds,
    )
    result = ranking[0]
    capacity_pass = capacity_passed(result, args.drones)
    summary = {
        "run_id": args.run_id,
        "drone_count": args.drones,
        "protocol": args.protocol,
        "wall_duration_s": time.monotonic() - started,
        "capacity_pass": capacity_pass,
        "controller_pass": result["pass"],
        "replicas_completed": result["replica_count"],
        "real_time_factor_mean": result["timing"]["real_time_factor_mean"],
        "real_time_factor_worst": result["timing"]["real_time_factor_worst"],
        "hover_xy_max_worst_m": result["hover_worst"].get("xy_max_m"),
        "hover_z_max_worst_m": result["hover_worst"].get("z_max_abs_m"),
        "trajectory_position_rmse_worst_m": result.get("trajectory_worst", {}).get(
            "position_rmse_m"
        ),
        "trajectory_position_max_error_worst_m": result.get("trajectory_worst", {}).get(
            "position_max_error_m"
        ),
        "ideal_position_final_error_worst_m": result["dynamic_worst"].get(
            "worst_ideal_position_final_error_m"
        ),
        "ideal_position_tail_rmse_worst_m": result["dynamic_worst"].get(
            "worst_ideal_position_tail_rmse_m"
        ),
        "high_speed_altitude_deviation_worst_m": result["dynamic_worst"].get(
            "high_speed_altitude_deviation_m"
        ),
        "abrupt_reversal_altitude_deviation_worst_m": result["dynamic_worst"].get(
            "abrupt_reversal_altitude_deviation_m"
        ),
        "high_speed_stop_displacement_worst_m": result["dynamic_worst"].get(
            "high_speed_stop_displacement_m"
        ),
        "yaw_final_error_worst_deg": result.get("yaw_worst", {}).get("final_error_deg"),
        "yaw_overshoot_worst_deg": result.get("yaw_worst", {}).get("overshoot_deg"),
        "failed_gates": result.get("failed_gates", []),
        "score": result["score"],
    }
    atomic_json(output / "capacity_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    if not summary["capacity_pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
