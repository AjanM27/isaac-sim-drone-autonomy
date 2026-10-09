#!/usr/bin/env python3
"""Benchmark an ArduPilot fleet in the local warehouse."""

from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime

from benchmark_parallel_capacity import capacity_passed
from parallel_pid_optimizer import RESULTS, ROOT, run_batch
from pid_optimizer import atomic_json, load_persisted_baseline


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drones", nargs="+", type=int, default=[6, 8, 10])
    parser.add_argument("--protocol", choices=("hover", "demo", "basic"), default="basic")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--warmup-seconds", type=float, default=4.0)
    parser.add_argument("--hover-seconds", type=float, default=8.0)
    parser.add_argument("--run-id", default=datetime.now().strftime("warehouse_fleet_%Y%m%d_%H%M%S"))
    parser.add_argument(
        "--visible", action="store_true", help="show the Isaac Sim window during the benchmark"
    )
    parser.add_argument("--record", action="store_true", help="record each visible run to MP4")
    args = parser.parse_args()
    if any(not 1 <= count <= 16 for count in args.drones):
        parser.error("drone counts must be between 1 and 16")
    if args.record and not args.visible:
        parser.error("--record requires --visible")

    config = json.loads((ROOT / "docker/flight_control/pid_search_space.json").read_text())
    parameter_path = ROOT / "docker/flight_control/isaac_sim_tuned.parm"
    parameters = load_persisted_baseline(config, parameter_path)
    root_output = RESULTS / args.run_id
    root_output.mkdir(parents=True, exist_ok=False)
    summaries = []
    os.environ["PARALLEL_SCENE_ENVIRONMENT"] = "warehouse"
    os.environ["PARALLEL_SCENE_SPACING"] = "2.5"
    os.environ["PARALLEL_SCENE_HEADLESS"] = "0" if args.visible else "1"
    for count in args.drones:
        print(f"\n=== Warehouse fleet benchmark: {count} drones ===", flush=True)
        output = root_output / f"drones_{count}"
        record_path = output / f"warehouse_{count}_drone_{args.protocol}.mp4"
        if args.record:
            os.environ["PARALLEL_RECORD_OUTPUT"] = str(record_path)
        else:
            os.environ.pop("PARALLEL_RECORD_OUTPUT", None)
        started = time.monotonic()
        ranking = run_batch(
            output,
            [{"name": "fleet_incumbent", "parameters": parameters}],
            parameter_path.read_text(),
            args.protocol,
            args.speed,
            max(2.0, args.speed),
            count,
            args.warmup_seconds,
            args.hover_seconds,
        )
        result = ranking[0]
        takeoff_times = [
            json.loads((output / f"env_{index}.json").read_text())
            .get("synchronization", {})
            .get("takeoff_command_monotonic_s")
            for index in range(count)
        ]
        valid_takeoff_times = [value for value in takeoff_times if value is not None]
        summary = {
            "drone_count": count,
            "all_drones_completed": result["replica_count"] == count,
            "teleoperation_motion_pass": (
                args.protocol == "hover" or "basic_motion" not in result.get("failed_gates", [])
            ),
            "capacity_pass": capacity_passed(result, count),
            "rtf_mean": result["timing"]["real_time_factor_mean"],
            "rtf_worst": result["timing"]["real_time_factor_worst"],
            "takeoff_command_spread_s": (
                max(valid_takeoff_times) - min(valid_takeoff_times)
                if len(valid_takeoff_times) == count else None
            ),
            "wall_duration_s": time.monotonic() - started,
            "failed_gates": result.get("failed_gates", []),
            "recording": str(record_path) if args.record and record_path.is_file() else None,
        }
        summaries.append(summary)
        print(json.dumps(summary, indent=2), flush=True)
    report = {
        "run_id": args.run_id,
        "environment": "warehouse",
        "sensor_profile": "flight-only; RTX cameras and LiDAR disabled",
        "protocol": args.protocol,
        "visible": args.visible,
        "recorded": args.record,
        "results": summaries,
    }
    atomic_json(root_output / "warehouse_fleet_summary.json", report)
    print(json.dumps(report, indent=2), flush=True)
    if args.protocol == "demo":
        successful = all(
            item["all_drones_completed"] and item["teleoperation_motion_pass"]
            for item in summaries
        )
    else:
        successful = all(
            item["capacity_pass"] and item["teleoperation_motion_pass"]
            for item in summaries
        )
    if not successful:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
