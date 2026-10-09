#!/usr/bin/env python3
"""Human-readable live status for an autonomous multi-drone tuning run."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path


def load(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def number(value, digits: int = 3, suffix: str = "") -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "-"
    return f"{numeric:.{digits}f}{suffix}" if math.isfinite(numeric) else "-"


def latest_batch(run_dir: Path) -> Path | None:
    if (run_dir / "batch_status.json").exists():
        return run_dir
    statuses = list(run_dir.glob("generation_*/batch_status.json"))
    return max(statuses, key=lambda path: path.stat().st_mtime).parent if statuses else None


def vector(value) -> str:
    if not isinstance(value, list) or len(value) != 3:
        return "-"
    return ",".join(number(component, 2) for component in value)


def failed_gates(result: dict) -> str:
    failed = [name for name, gate in result.get("gates", {}).items() if not gate.get("pass", False)]
    return ",".join(failed) if failed else "none"


def render(run_dir: Path, optimizer_pid: int | None) -> str:
    final = load(run_dir / "final_result.json")
    state = load(run_dir / "state.json")
    batch_dir = latest_batch(run_dir)
    batch = load(batch_dir / "batch_status.json") if batch_dir is not None else {}
    running = bool(optimizer_pid and Path(f"/proc/{optimizer_pid}").exists())
    status = (
        "complete" if final or batch.get("status") == "complete"
        else "running" if running or batch.get("status") in ("starting", "running", "retrying")
        else state.get("status", batch.get("status", "stopped"))
    )
    lines = [f"PID tuning run: {run_dir.name}", f"Status: {status}"]
    if optimizer_pid:
        lines[-1] += f"  optimizer PID: {optimizer_pid}"

    if batch_dir is not None:
        lines.extend(
            [
                "",
                f"Current/latest batch: {batch_dir.name}",
                f"Batch status: {batch.get('status', 'unknown')}  "
                f"protocol: {batch.get('protocol', '-')}  drones: {batch.get('drone_count', '-')}  "
                f"speed: {number(batch.get('speed_mps'), 1)}/{number(batch.get('high_speed_mps'), 1)} m/s",
                "",
                "Drone  Candidate                       Rep  Status     Phase             "
                "Progress  Position xyz (m)       Velocity xyz (m/s)   Samples  Score   Pass   RTF",
                "-" * 146,
            ]
        )
        count = int(batch.get("drone_count", 0))
        candidate_by_drone = {int(item["drone"]): item for item in batch.get("candidates", [])}
        for index in range(count):
            progress = load(batch_dir / f"env_{index}_progress.json")
            result = load(batch_dir / f"env_{index}.json")
            candidate = candidate_by_drone.get(index, {})
            phase_progress = progress.get("phase_progress")
            progress_text = number(100.0 * phase_progress, 0, "%") if phase_progress is not None else "-"
            score = result.get("score", progress.get("result", {}).get("score"))
            passed = result.get("pass", progress.get("result", {}).get("pass"))
            timing = result.get("timing", {}).get(
                "real_time_factor", progress.get("result", {}).get("real_time_factor")
            )
            lines.append(
                f"{index:>5}  {str(candidate.get('name', progress.get('candidate', '-')))[:31]:<31}  "
                f"{str(candidate.get('replica', progress.get('replica', '-'))):>3}  "
                f"{str(progress.get('status', 'waiting'))[:10]:<10} "
                f"{str(progress.get('phase', '-'))[:17]:<17} "
                f"{progress_text:>8}  {vector(progress.get('position_m')):<23} "
                f"{vector(progress.get('velocity_mps')):<22} "
                f"{str(progress.get('sample_count', 0)):>7}  {number(score):>6}  "
                f"{str(passed) if passed is not None else '-':>5}  {number(timing):>5}"
            )
            if result:
                hover = result.get("hover", {})
                dynamic = result.get("dynamic", {})
                lines.append(
                    "       metrics: "
                    f"hover xy={number(hover.get('xy_max_m'), 4)} m, "
                    f"z={number(hover.get('z_max_abs_m'), 4)} m, "
                    f"yaw p-p={number(hover.get('yaw_peak_to_peak_deg'), 2)} deg; "
                    f"stop={number(dynamic.get('standard_stop_displacement_m'), 3)} m, "
                    f"reversal={number(dynamic.get('reversal_stop_displacement_m'), 3)} m; "
                    f"actual-ideal final={number(dynamic.get('worst_ideal_position_final_error_m'), 3)} m, "
                    f"tail={number(dynamic.get('worst_ideal_position_tail_rmse_m'), 3)} m; "
                    f"failed gates={failed_gates(result)}"
                )
                yaw_values = [value for value in result.get("yaw", {}).values() if value.get("valid")]
                if yaw_values or "high_speed_altitude_deviation_m" in dynamic:
                    lines.append(
                        "       stress: "
                        f"high-speed z={number(dynamic.get('high_speed_altitude_deviation_m'), 4)} m, "
                        f"abrupt z={number(dynamic.get('abrupt_reversal_altitude_deviation_m'), 4)} m, "
                        f"yaw final={number(max((value.get('yaw_final_error_deg', 0.0) for value in yaw_values), default=0.0), 2)} deg, "
                        f"yaw overshoot={number(max((value.get('yaw_overshoot_deg', 0.0) for value in yaw_values), default=0.0), 2)} deg"
                    )
            elif progress:
                lines.append(
                    "       live: "
                    f"elapsed={number(progress.get('elapsed_s'), 1)} s, "
                    f"command xyz={vector(progress.get('command_velocity_mps'))} m/s"
                )
            if progress.get("message"):
                lines.append(f"       message: {progress['message']}")

        ranking = batch.get("ranking", [])
        if ranking:
            lines.extend(["", "Batch ranking:"])
            for item in ranking:
                lines.append(
                    f"  {item['rank']}. {item['candidate']}: score={number(item['score'], 4)}, "
                    f"pass={item['pass']}, replicas={item['replicas']}, "
                    f"hover max={number(item.get('hover_xy_max_worst_m'), 4)} m, "
                    f"hover z={number(item.get('hover_z_max_worst_m'), 4)} m, "
                    f"stop={number(item.get('standard_stop_worst_m'), 3)} m, "
                    f"ideal final={number(item.get('ideal_final_error_worst_m'), 3)} m, "
                    f"trajectory max={number(item.get('trajectory_max_error_worst_m'), 3)} m, "
                    f"yaw={number(item.get('yaw_final_error_worst_deg'), 2)} deg, "
                    f"yaw-z={number(item.get('yaw_altitude_deviation_worst_m'), 3)} m, "
                    f"failed={','.join(item.get('failed_gates', [])) or 'none'}, "
                    f"worst RTF={number(item.get('real_time_factor_worst'), 3)}"
                )

    result = final or state
    history = result.get("history", [])
    if history:
        lines.extend(["", f"Completed optimizer batches: {len(history)}"])
        for item in history[-5:]:
            ranking = item.get("ranking", [])
            winner = ranking[0] if ranking else {}
            lines.append(
                f"  generation {item.get('generation', '-')}: {item.get('stage', '-')} "
                f"optimizer={item.get('optimizer', 'legacy')}  "
                f"sigma={number(item.get('optimizer_state', {}).get('sigma'), 6)}  best="
                f"{winner.get('candidate', {}).get('name', '-')}  "
                f"score={number(winner.get('score'), 4)}  pass={winner.get('pass', '-')}  "
                f"accepted={item.get('accepted', '-')}"
            )
    if final:
        validation = final.get("validation", [{}])[0]
        lines.extend(
            [
                "",
                f"Final validation: {final.get('validation_pass', False)}",
                f"Final score: {number(validation.get('score'), 4)}  "
                f"hover max: {number(validation.get('hover_worst', {}).get('xy_max_m'), 4)} m  "
                f"worst RTF: {number(validation.get('timing', {}).get('real_time_factor_worst'), 3)}",
            ]
        )
    return "\n".join(lines)


def render_all(results_dir: Path) -> str:
    lines = [
        "PID tuning run history",
        "Run                                      Status      Drones  Controller  Score    Worst RTF  Hover max",
        "-" * 108,
    ]
    runs = [path for path in results_dir.iterdir() if path.is_dir() and path.name != "runtime"]
    for run in sorted(runs, key=lambda path: path.stat().st_mtime, reverse=True):
        final = load(run / "final_result.json")
        batch_dir = latest_batch(run)
        batch = load(batch_dir / "batch_status.json") if batch_dir else {}
        ranking = batch.get("ranking", [])
        best = ranking[0] if ranking else {}
        status = "complete" if final or batch.get("status") == "complete" else batch.get("status", "stopped")
        validation = final.get("validation", [{}])[0] if final else {}
        controller = final.get("validation_pass", best.get("pass", "-"))
        score = validation.get("score", best.get("score"))
        rtf = validation.get("timing", {}).get(
            "real_time_factor_worst", best.get("real_time_factor_worst")
        )
        hover = validation.get("hover_worst", {}).get(
            "xy_max_m", best.get("hover_xy_max_worst_m")
        )
        lines.append(
            f"{run.name[:40]:<40} {str(status)[:11]:<11} "
            f"{str(batch.get('drone_count', '-')):>6}  {str(controller):>10}  "
            f"{number(score, 4):>7}  {number(rtf, 3):>9}  {number(hover, 4):>9}"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--pid", type=int)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=float, default=2.0)
    args = parser.parse_args()
    if args.all:
        print(render_all(args.run_dir))
        return
    while True:
        if args.watch:
            print("\033[2J\033[H", end="")
        print(render(args.run_dir, args.pid), flush=True)
        if not args.watch:
            return
        final_exists = (args.run_dir / "final_result.json").exists()
        running = bool(args.pid and Path(f"/proc/{args.pid}").exists())
        if final_exists or not running:
            return
        time.sleep(max(0.5, args.interval))


if __name__ == "__main__":
    main()
