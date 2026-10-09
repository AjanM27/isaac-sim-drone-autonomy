#!/usr/bin/env python3
"""Staged, vectorized PID optimizer using parallel Isaac/ArduPilot clones."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from adaptive_cmaes import BoundedCMAES
from pid_optimizer import atomic_json, append_decision, duration_seconds, load_persisted_baseline


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "simulation/room_quadcopter/results/pid_tuning/parallel"
MISSION_ROOT = ROOT / ".omc/autoresearch/parallel-pid-minimum"
DEFAULT_STAGES = (
    "attitude_rate",
    "attitude_angle",
    "vertical",
    "horizontal_velocity",
    "horizontal_position",
    "yaw",
    "braking",
)
PROTOCOL_BY_STAGE = {
    # Every controller that can affect translation must be ranked on dynamic
    # response.  The old hover-only search could select a quiet-looking gain
    # that climbed, coupled axes, or oscillated during a reversal.
    "attitude_rate": "stress",
    "attitude_angle": "stress",
    "vertical": "stress",
    "horizontal_velocity": "stress",
    "horizontal_velocity_p": "stress",
    "horizontal_velocity_id": "stress",
    "horizontal_position": "stress",
    "yaw": "stress",
    "braking": "stress",
}


def render_parameters(template: str, parameters: dict[str, float]) -> str:
    text = template
    for name, value in parameters.items():
        replacement = f"{name:<16}{float(value):.6f}"
        text, count = re.subn(
            rf"(?m)^{re.escape(name)}[ \t]+[-+0-9.eE]+[ \t]*$", replacement, text
        )
        if count != 1:
            raise RuntimeError(f"Expected exactly one {name} in parameter template, got {count}")
    return text


def replicate_candidates(candidates: list[dict], replicas: int) -> list[dict]:
    expanded = []
    # Interleave replicas so each candidate is distributed across the shared
    # grid instead of occupying one spatial cluster. This reduces position and
    # scheduler-order bias in simultaneous comparisons.
    for replica in range(replicas):
        for candidate_index, candidate in enumerate(candidates):
            candidate_id = f"candidate_{candidate_index:02d}_{candidate['name']}"
            item = {
                "candidate_id": candidate_id,
                "name": candidate["name"],
                "replica": replica,
                "parameters": dict(candidate["parameters"]),
            }
            expanded.append(item)
    return expanded


def aggregate_candidates(summaries: list[dict]) -> list[dict]:
    """Aggregate replicated environments so noisy single clones cannot win."""
    grouped: dict[str, list[dict]] = {}
    for summary in summaries:
        candidate = summary["candidate"]
        candidate_id = str(candidate.get("candidate_id", candidate["name"]))
        grouped.setdefault(candidate_id, []).append(summary)
    ranking = []
    for candidate_id, replicas in grouped.items():
        scores = [float(item["score"]) for item in replicas]
        hover_names = {
            name
            for item in replicas
            for name, value in item["hover"].items()
            if isinstance(value, (int, float)) and not isinstance(value, bool)
        }
        hover_mean = {
            name: statistics.fmean(float(item["hover"][name]) for item in replicas)
            for name in hover_names
            if all(name in item["hover"] for item in replicas)
        }
        hover_worst = {
            name: max(float(item["hover"][name]) for item in replicas)
            for name in hover_names
            if all(name in item["hover"] for item in replicas)
        }
        score_stddev = statistics.pstdev(scores)
        timing_factors = [float(item.get("timing", {}).get("real_time_factor", 0.0)) for item in replicas]
        yaw_values = [
            phase
            for item in replicas
            for phase in item.get("yaw", {}).values()
            if phase.get("valid", False)
        ]
        failed_gates = sorted({
            name
            for item in replicas
            for name, gate in item.get("gates", {}).items()
            if not gate.get("pass", False)
        })
        dynamic_names = (
            "tracking_position_rmse_m",
            "tracking_velocity_rmse_mps",
            "steady_velocity_rmse_mps",
            "cross_axis_rms_mps",
            "steady_cross_axis_rms_mps",
            "max_altitude_rise_m",
            "max_abs_altitude_error_m",
            "high_speed_altitude_deviation_m",
            "abrupt_reversal_altitude_deviation_m",
            "moving_yaw_altitude_deviation_m",
            "velocity_jerk_rms_mps3",
            "vertical_tracking_rmse_mps",
            "worst_stop_displacement_m",
            "worst_stop_settling_time_s",
            "worst_stop_altitude_rise_m",
            "standard_stop_displacement_m",
            "reversal_stop_displacement_m",
            "high_speed_stop_displacement_m",
            "abrupt_stop_displacement_m",
            "horizontal_stop_altitude_rise_m",
            "vertical_stop_overshoot_m",
            "worst_ideal_position_final_error_m",
            "worst_ideal_position_tail_rmse_m",
        )
        dynamic_mean = {
            name: statistics.fmean(float(item["dynamic"][name]) for item in replicas)
            for name in dynamic_names
            if all(name in item.get("dynamic", {}) for item in replicas)
        }
        dynamic_worst = {
            name: max(float(item["dynamic"][name]) for item in replicas)
            for name in dynamic_names
            if all(name in item.get("dynamic", {}) for item in replicas)
        }
        trajectory_names = (
            "position_rmse_m",
            "position_max_error_m",
            "position_final_error_m",
            "position_tail_rmse_m",
            "xy_rmse_m",
            "xy_max_error_m",
            "xy_final_error_m",
            "x_rmse_m",
            "x_max_abs_error_m",
            "y_rmse_m",
            "y_max_abs_error_m",
            "z_rmse_m",
            "z_max_abs_error_m",
        )
        trajectory_mean = {
            name: statistics.fmean(float(item["trajectory"][name]) for item in replicas)
            for name in trajectory_names
            if all(name in item.get("trajectory", {}) for item in replicas)
        }
        trajectory_worst = {
            name: max(float(item["trajectory"][name]) for item in replicas)
            for name in trajectory_names
            if all(name in item.get("trajectory", {}) for item in replicas)
        }
        ranking.append(
            {
                "candidate_id": candidate_id,
                "candidate": {
                    "name": replicas[0]["candidate"]["name"],
                    "parameters": replicas[0]["candidate"]["parameters"],
                },
                "replica_count": len(replicas),
                "replica_environments": [item["environment"] for item in replicas],
                "replica_scores": scores,
                "score_mean": statistics.fmean(scores),
                "score_stddev": score_stddev,
                "score": statistics.fmean(scores) + 0.50 * score_stddev,
                "hover": hover_mean,
                "hover_worst": hover_worst,
                "timing": {
                    "real_time_factor_mean": statistics.fmean(timing_factors),
                    "real_time_factor_worst": min(timing_factors),
                },
                "dynamic": dynamic_mean,
                "dynamic_worst": dynamic_worst,
                "trajectory": trajectory_mean,
                "trajectory_worst": trajectory_worst,
                "yaw_worst": {
                    "final_error_deg": max(
                        (float(value["yaw_final_error_deg"]) for value in yaw_values), default=0.0
                    ),
                    "overshoot_deg": max(
                        (float(value["yaw_overshoot_deg"]) for value in yaw_values), default=0.0
                    ),
                    "tail_peak_to_peak_deg": max(
                        (float(value["yaw_tail_peak_to_peak_deg"]) for value in yaw_values),
                        default=0.0,
                    ),
                    "altitude_deviation_m": max(
                        (float(value["max_altitude_deviation_m"]) for value in yaw_values),
                        default=0.0,
                    ),
                },
                "failed_gates": failed_gates,
                "pass": all(item.get("pass", False) for item in replicas),
            }
        )
    return sorted(ranking, key=lambda item: (not item["pass"], item["score"]))


def write_batch(batch_dir: Path, candidates: list[dict], template: str) -> None:
    profiles = batch_dir / "profiles"
    profiles.mkdir(parents=True, exist_ok=True)
    for index, candidate in enumerate(candidates):
        (profiles / f"env_{index}.parm").write_text(
            render_parameters(template, candidate["parameters"])
        )
    atomic_json(batch_dir / "manifest.json", {"candidates": candidates})


def run_batch(
    batch_dir: Path,
    candidates: list[dict],
    template: str,
    protocol: str,
    speed: float,
    high_speed: float,
    replicas: int,
    warmup_seconds: float,
    hover_seconds: float,
):
    expanded = replicate_candidates(candidates, replicas)
    write_batch(batch_dir, expanded, template)
    batch_status = {
        "status": "starting",
        "batch": batch_dir.name,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "protocol": protocol,
        "speed_mps": speed,
        "high_speed_mps": high_speed,
        "drone_count": len(expanded),
        "candidates": [
            {
                "drone": index,
                "name": candidate["name"],
                "candidate_id": candidate.get("candidate_id", ""),
                "replica": candidate.get("replica", 0),
            }
            for index, candidate in enumerate(expanded)
        ],
    }
    atomic_json(batch_dir / "batch_status.json", batch_status)
    command = [
        "bash",
        str(ROOT / "docker/flight_control/run_parallel_pid_batch.sh"),
        str(len(expanded)),
        str(batch_dir / "profiles"),
        str(batch_dir),
        protocol,
        str(speed),
        str(high_speed),
        str(warmup_seconds),
        str(hover_seconds),
    ]
    last_error = None
    for attempt in range(1, 4):
        try:
            batch_status.update({"status": "running", "attempt": attempt})
            atomic_json(batch_dir / "batch_status.json", batch_status)
            print(
                f"[batch] {batch_dir.name}: {len(expanded)} drones, "
                f"protocol={protocol}, attempt={attempt}",
                flush=True,
            )
            subprocess.run(command, cwd=ROOT, check=True)
            result_files = sorted(
                path for path in batch_dir.glob("env_*.json")
                if re.fullmatch(r"env_\d+\.json", path.name)
            )
            if len(result_files) != len(expanded):
                raise RuntimeError(
                    f"batch produced {len(result_files)} of {len(expanded)} result files"
                )
            break
        except (subprocess.CalledProcessError, RuntimeError) as error:
            last_error = error
            batch_status.update(
                {"status": "retrying" if attempt < 3 else "failed", "attempt": attempt,
                 "message": str(error)}
            )
            atomic_json(batch_dir / "batch_status.json", batch_status)
            logs = batch_dir / "logs"
            if logs.exists():
                logs.rename(batch_dir / f"logs_failed_attempt_{attempt}")
            for pattern in ("env_*.json", "env_*.csv", "env_*_progress.json", "env_*_diagnostics.png",
                            "candidate_comparison.png", "batch_summary.json"):
                for partial in batch_dir.glob(pattern):
                    partial.unlink()
            if attempt < 3:
                append_decision(
                    MISSION_ROOT / "infrastructure-retries.md",
                    f"- `{batch_dir.name}` attempt {attempt} failed: `{error}`; retrying.",
                )
                time.sleep(2.0)
    else:
        raise RuntimeError(f"Batch failed after three clean attempts: {batch_dir}") from last_error
    ranking = json.loads((batch_dir / "batch_summary.json").read_text())["ranking"]
    batch_status.update(
        {
            "status": "complete",
            "completed_at": datetime.now(timezone.utc).isoformat(),
            "ranking": [
                {
                    "rank": rank,
                    "candidate": item["candidate"]["name"],
                    "pass": item["pass"],
                    "score": item["score"],
                    "replicas": item["replica_count"],
                    "real_time_factor_worst": item["timing"]["real_time_factor_worst"],
                    "hover_xy_max_worst_m": item["hover_worst"].get("xy_max_m"),
                    "hover_z_max_worst_m": item["hover_worst"].get("z_max_abs_m"),
                    "standard_stop_worst_m": item["dynamic_worst"].get(
                        "standard_stop_displacement_m"
                    ),
                    "ideal_final_error_worst_m": item["dynamic_worst"].get(
                        "worst_ideal_position_final_error_m"
                    ),
                    "ideal_tail_rmse_worst_m": item["dynamic_worst"].get(
                        "worst_ideal_position_tail_rmse_m"
                    ),
                    "trajectory_max_error_worst_m": item.get("trajectory_worst", {}).get(
                        "position_max_error_m"
                    ),
                    "reversal_stop_worst_m": item["dynamic_worst"].get(
                        "reversal_stop_displacement_m"
                    ),
                    "failed_gates": item.get("failed_gates", []),
                    "yaw_final_error_worst_deg": item.get("yaw_worst", {}).get(
                        "final_error_deg"
                    ),
                    "yaw_altitude_deviation_worst_m": item.get("yaw_worst", {}).get(
                        "altitude_deviation_m"
                    ),
                }
                for rank, item in enumerate(ranking, start=1)
            ],
        }
    )
    atomic_json(batch_dir / "batch_status.json", batch_status)
    return ranking


def acceptable_improvement(winner: dict, control: dict, minimum_fraction: float) -> bool:
    # Continue improving a failed baseline, but never trade a passing control
    # for a failing candidate. Gate penalties let CMA prioritize restoring a
    # failed safety property before polishing the continuous score.
    if control.get("pass", False) and not winner.get("pass", False):
        return False
    control_fitness = optimizer_fitness(control)
    winner_fitness = optimizer_fitness(winner)
    improvement = control_fitness - winner_fitness
    if improvement < max(0.001, abs(control_fitness) * minimum_fraction):
        return False
    control_hover = control.get("hover_worst", control.get("hover", {}))
    winner_hover = winner.get("hover_worst", winner.get("hover", {}))
    control_dynamic = control.get("dynamic_worst", control.get("dynamic", {}))
    winner_dynamic = winner.get("dynamic_worst", winner.get("dynamic", {}))
    control_trajectory = control.get("trajectory_worst", control.get("trajectory", {}))
    winner_trajectory = winner.get("trajectory_worst", winner.get("trajectory", {}))
    critical = (
        (winner_hover, control_hover, "xy_max_m", 0.015),
        (winner_hover, control_hover, "z_max_abs_m", 0.010),
        (winner_hover, control_hover, "yaw_peak_to_peak_deg", 0.75),
        (winner_dynamic, control_dynamic, "max_altitude_rise_m", 0.015),
        (winner_dynamic, control_dynamic, "standard_stop_displacement_m", 0.08),
        (winner_dynamic, control_dynamic, "reversal_stop_displacement_m", 0.12),
        (winner_dynamic, control_dynamic, "high_speed_altitude_deviation_m", 0.010),
        (winner_dynamic, control_dynamic, "abrupt_reversal_altitude_deviation_m", 0.010),
        (winner_dynamic, control_dynamic, "moving_yaw_altitude_deviation_m", 0.010),
        (winner_dynamic, control_dynamic, "worst_ideal_position_final_error_m", 0.025),
        (winner_dynamic, control_dynamic, "worst_ideal_position_tail_rmse_m", 0.025),
        (winner_trajectory, control_trajectory, "position_max_error_m", 0.10),
        (winner_trajectory, control_trajectory, "z_max_abs_error_m", 0.015),
    )
    return all(
        key not in winner_values or key not in control_values
        or float(winner_values[key]) <= float(control_values[key]) + tolerance
        for winner_values, control_values, key, tolerance in critical
    )


def promote(parameters: dict[str, float], destination: Path) -> None:
    destination.write_text(render_parameters(destination.read_text(), parameters))


def group_coordinates(group: dict) -> list[tuple[str, list[str], float, float]]:
    coordinates = []
    for knob, limits in group["parameters"].items():
        coordinates.append(
            (
                knob,
                list(limits.get("members", [knob])),
                float(limits["minimum"]),
                float(limits["maximum"]),
            )
        )
    return coordinates


def encode_parameters(parameters: dict[str, float], coordinates) -> list[float]:
    encoded = []
    for _knob, members, minimum, maximum in coordinates:
        value = statistics.fmean(float(parameters[name]) for name in members)
        encoded.append((value - minimum) / (maximum - minimum))
    return encoded


def decode_parameters(
    point: list[float], base: dict[str, float], coordinates, precision: int
) -> dict[str, float]:
    parameters = dict(base)
    for normalized, (_knob, members, minimum, maximum) in zip(point, coordinates):
        value = round(minimum + min(1.0, max(0.0, normalized)) * (maximum - minimum), precision)
        for name in members:
            parameters[name] = value
    return parameters


def optimizer_fitness(result: dict) -> float:
    """Continuous score plus explicit penalties for unsafe failed gates."""
    failed = result.get("failed_gates", [])
    return float(result["score"]) + 5.0 * len(failed)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--batch-size", type=int, default=10)
    parser.add_argument("--replicas", type=int, default=2)
    parser.add_argument("--generations-per-stage", type=int, default=8)
    parser.add_argument("--stages", nargs="*", default=list(DEFAULT_STAGES))
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--high-speed", type=float, default=3.0)
    parser.add_argument("--minimum-improvement", type=float, default=0.002)
    parser.add_argument("--cma-sigma", type=float, default=0.20)
    parser.add_argument("--cma-tolerance", type=float, default=0.002)
    parser.add_argument("--cma-patience", type=int, default=4)
    parser.add_argument("--seed", type=int, default=27)
    parser.add_argument("--max-runtime", type=duration_seconds, default=4 * 60 * 60.0)
    parser.add_argument("--warmup-seconds", type=float, default=12.0)
    parser.add_argument("--hover-seconds", type=float, default=30.0)
    parser.add_argument("--promote", action="store_true")
    args = parser.parse_args()
    if not 2 <= args.batch_size <= 16:
        parser.error("batch-size must be between 2 and 16")
    if args.replicas < 2 or args.batch_size // args.replicas < 4:
        parser.error("replicas must be >=2 and batch-size must fit at least four CMA candidates")
    if args.generations_per_stage < 1:
        parser.error("generations-per-stage must be positive")
    if args.high_speed < max(2.0, args.speed):
        parser.error("high-speed must be at least 2 m/s and not lower than speed")
    if not 0.0 < args.cma_sigma <= 0.5:
        parser.error("cma-sigma must be in (0, 0.5]")
    if not 0.0 < args.cma_tolerance < args.cma_sigma:
        parser.error("cma-tolerance must be positive and smaller than cma-sigma")
    if args.cma_patience < 2:
        parser.error("cma-patience must be at least two generations")

    config_path = ROOT / "docker/flight_control/pid_search_space.json"
    parameter_path = ROOT / "docker/flight_control/isaac_sim_tuned.parm"
    config = json.loads(config_path.read_text())
    groups = {item["name"]: item for item in config["groups"]}
    unknown = set(args.stages) - groups.keys()
    if unknown:
        parser.error(f"unknown stages: {sorted(unknown)}")
    incumbent = load_persisted_baseline(config, parameter_path)
    original_parameters = dict(incumbent)
    template = parameter_path.read_text()
    run_dir = RESULTS / args.run_id
    if run_dir.exists():
        raise RuntimeError(f"Run already exists: {run_dir}")
    run_dir.mkdir(parents=True)
    mission_run = MISSION_ROOT / "runs" / args.run_id
    mission_run.mkdir(parents=True, exist_ok=True)
    append_decision(
        mission_run / "decision-log.md",
        f"# Parallel PID minimum decision log\n\nRun `{args.run_id}` started.",
    )
    history = []
    deadline = time.monotonic() + args.max_runtime
    population_size = args.batch_size // args.replicas

    generation = 0
    for stage_name in args.stages:
        if time.monotonic() >= deadline:
            break
        group = groups[stage_name]
        protocol = PROTOCOL_BY_STAGE[stage_name]
        coordinates = group_coordinates(group)
        cma = BoundedCMAES(
            encode_parameters(incumbent, coordinates),
            population_size,
            sigma=args.cma_sigma,
            seed=args.seed + len(history) * 101 + len(stage_name),
        )
        for stage_generation in range(args.generations_per_stage):
            if time.monotonic() >= deadline:
                break
            generation += 1
            points = [encode_parameters(incumbent, coordinates), *cma.ask(population_size - 1)]
            candidates = []
            for candidate_index, point in enumerate(points):
                candidates.append(
                    {
                        "name": "incumbent" if candidate_index == 0 else (
                            f"cma_{stage_name}_g{stage_generation + 1:02d}_p{candidate_index:02d}"
                        ),
                        "parameters": decode_parameters(
                            point, incumbent, coordinates, int(config["precision"])
                        ),
                        "normalized_point": point,
                    }
                )
            batch_dir = run_dir / f"generation_{generation:03d}_{stage_name}_cma"
            ranking = run_batch(
                batch_dir, candidates, template, protocol, args.speed, args.high_speed,
                args.replicas, args.warmup_seconds, args.hover_seconds,
            )
            by_name = {item["candidate"]["name"]: item for item in ranking}
            ordered_results = [by_name[item["name"]] for item in candidates]
            fitness = [optimizer_fitness(item) for item in ordered_results]
            cma.tell(points, fitness)
            control = ordered_results[0]
            acceptable = [
                item for item in ordered_results[1:]
                if acceptable_improvement(item, control, args.minimum_improvement)
            ]
            accepted = bool(acceptable)
            if accepted:
                winner = min(acceptable, key=optimizer_fitness)
                incumbent = {
                    name: round(float(value), int(config["precision"]))
                    for name, value in winner["candidate"]["parameters"].items()
                }
            record = {
                "generation": generation,
                "stage": stage_name,
                "stage_generation": stage_generation,
                "optimizer": "bounded_cma_es",
                "optimizer_state": cma.state(),
                "ranking": ranking,
                "control": control,
                "accepted": accepted,
                "accepted_winner": winner if accepted else None,
            }
            history.append(record)
            evaluation = {
                "pass": bool(ranking[0].get("pass", False)),
                "score": float(ranking[0]["score"]),
                "stage": stage_name,
                "optimizer": "bounded_cma_es",
                "sigma": cma.sigma,
                "stagnant_generations": cma.stagnant_generations,
                "ranking": ranking,
            }
            atomic_json(
                mission_run / "evaluations" / f"iteration-{generation:04d}.json", evaluation
            )
            append_decision(
                mission_run / "decision-log.md",
                f"## Iteration {generation:04d}\n\n"
                f"- Stage: `{stage_name}`\n"
                f"- Optimizer: bounded CMA-ES\n"
                f"- Adaptive normalized sigma: `{cma.sigma:.8f}`\n"
                f"- Best candidate: `{ranking[0]['candidate']['name']}`\n"
                f"- Variance-penalized score: `{ranking[0]['score']:.8f}`\n"
                f"- Accepted: `{accepted}`\n"
                f"- Passed all replicas: `{ranking[0]['pass']}`",
            )
            atomic_json(
                run_dir / "state.json",
                {
                    "status": "running",
                    "optimizer": "bounded_cma_es",
                    "active_stage": stage_name,
                    "adaptive_sigma": cma.sigma,
                    "incumbent_parameters": incumbent,
                    "history": history,
                },
            )
            if cma.converged(args.cma_tolerance, args.cma_patience):
                append_decision(
                    mission_run / "decision-log.md",
                    f"- Stage `{stage_name}` converged after {cma.generation} CMA generations.",
                )
                break

    confirmation = {
        "required": incumbent != original_parameters,
        "pass": True,
        "reason": "No candidate displaced the incumbent.",
    }
    if incumbent != original_parameters:
        generation += 1
        baseline_confirmation = run_batch(
            run_dir / f"generation_{generation:03d}_confirmation_baseline",
            [{"name": "confirmation_baseline", "parameters": original_parameters}],
            template, "stress", args.speed, args.high_speed, args.replicas,
            args.warmup_seconds, args.hover_seconds,
        )[0]
        generation += 1
        winner_confirmation = run_batch(
            run_dir / f"generation_{generation:03d}_confirmation_winner",
            [{"name": "confirmation_winner", "parameters": incumbent}],
            template, "stress", args.speed, args.high_speed, args.replicas,
            args.warmup_seconds, args.hover_seconds,
        )[0]
        confirmed = acceptable_improvement(
            winner_confirmation, baseline_confirmation, args.minimum_improvement
        )
        confirmation = {
            "required": True,
            "pass": confirmed,
            "baseline": baseline_confirmation,
            "winner": winner_confirmation,
            "reason": (
                "Winner repeated its improvement on a fresh start."
                if confirmed
                else "Fresh-start confirmation did not reproduce the search improvement."
            ),
        }
        if not confirmed:
            incumbent = original_parameters

    generation += 1
    validation_candidates = [
        {"name": "validation", "parameters": dict(incumbent)}
    ]
    validation_dir = run_dir / f"generation_{generation:03d}_validation"
    validation = run_batch(
        validation_dir, validation_candidates, template, "stress", args.speed, args.high_speed,
        args.replicas, args.warmup_seconds, args.hover_seconds,
    )
    validation_pass = all(
        item.get("pass", False)
        and item["hover_worst"].get("xy_max_m", math.inf) < 0.10
        and item["hover_worst"].get("z_max_abs_m", math.inf) < 0.10
        and item["dynamic_worst"].get("max_altitude_rise_m", math.inf) < 0.15
        and item["dynamic_worst"].get("horizontal_stop_altitude_rise_m", math.inf) < 0.10
        and item["dynamic_worst"].get("standard_stop_displacement_m", math.inf) < 0.75
        and item["dynamic_worst"].get("reversal_stop_displacement_m", math.inf) < 1.50
        and item["dynamic_worst"].get("high_speed_altitude_deviation_m", math.inf) < 0.05
        and item["dynamic_worst"].get("abrupt_reversal_altitude_deviation_m", math.inf) < 0.05
        and item["dynamic_worst"].get("moving_yaw_altitude_deviation_m", math.inf) < 0.05
        and item.get("yaw_worst", {}).get("final_error_deg", math.inf) < 5.0
        and item.get("yaw_worst", {}).get("overshoot_deg", math.inf) < 8.0
        and item["dynamic_worst"].get("vertical_stop_overshoot_m", math.inf) < 0.45
        and item["dynamic_worst"].get(
            "worst_ideal_position_final_error_m", math.inf
        ) < 0.10
        and item["dynamic_worst"].get(
            "worst_ideal_position_tail_rmse_m", math.inf
        ) < 0.10
        for item in validation
    )
    final = {
        "status": "complete",
        "run_id": args.run_id,
        "completed_at": datetime.now(timezone.utc).isoformat(),
        "parameters": incumbent,
        "confirmation": confirmation,
        "validation_pass": validation_pass,
        "validation": validation,
        "history": history,
        "runtime_limit_seconds": args.max_runtime,
    }
    atomic_json(
        run_dir / "state.json",
        {
            "status": "complete",
            "optimizer": "bounded_cma_es",
            "active_stage": history[-1]["stage"] if history else None,
            "adaptive_sigma": (
                history[-1].get("optimizer_state", {}).get("sigma") if history else None
            ),
            "incumbent_parameters": incumbent,
            "validation_pass": validation_pass,
            "completed_at": final["completed_at"],
            "history": history,
        },
    )
    atomic_json(run_dir / "final_result.json", final)
    atomic_json(mission_run / "final_result.json", final)
    if args.promote:
        if not validation_pass or not confirmation["pass"]:
            raise RuntimeError("Refusing promotion because one or more validation clones failed")
        promote(incumbent, parameter_path)
    print(json.dumps(final, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
