#!/usr/bin/env python3
"""Resume-safe, four-decimal coordinate search using fresh Isaac/SITL trials."""

from __future__ import annotations

import argparse
import json
import os
import re
import statistics
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from evaluate_pid_candidate import evaluate


ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = ROOT / "simulation/room_quadcopter/results/pid_tuning/autotune"
MISSION_ROOT = ROOT / ".omc/autoresearch/precision-pid-tuning"


def duration_seconds(text: str) -> float:
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)([smh]?)", text.strip().lower())
    if not match:
        raise argparse.ArgumentTypeError("duration must look like 900s, 30m, or 2h")
    scale = {"": 1.0, "s": 1.0, "m": 60.0, "h": 3600.0}[match.group(2)]
    return float(match.group(1)) * scale


def atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def append_decision(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(text.rstrip() + "\n\n")


def parameter_specs(config: dict):
    """Yield search knobs and the real ArduPilot parameters they control."""
    for group in config["groups"]:
        for knob, limits in group["parameters"].items():
            yield group, knob, limits, limits.get("members", [knob])


def load_persisted_baseline(config: dict, parameter_file: Path) -> dict[str, float]:
    """Read the search parameters from the controller file used by SITL."""
    names = {name for _, _, _, members in parameter_specs(config) for name in members}
    values: dict[str, float] = {}
    for raw_line in parameter_file.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) >= 2 and fields[0] in names:
            values[fields[0]] = round(float(fields[1]), int(config["precision"]))
    missing = names - values.keys()
    if missing:
        raise RuntimeError(f"Missing search parameters in {parameter_file}: {sorted(missing)}")
    return values


def candidate_stream(config: dict, incumbent: dict, groups: set[str], history: set[str]):
    precision = int(config["precision"])
    for group in config["groups"]:
        if groups and group["name"] not in groups:
            continue
        for parameter, limits in group["parameters"].items():
            members = limits.get("members", [parameter])
            center = statistics.fmean(float(incumbent[name]) for name in members)
            step = float(limits["step"])
            while step >= 10 ** (-precision):
                for direction in (-1.0, 1.0):
                    value = round(center + direction * step, precision)
                    if not limits["minimum"] <= value <= limits["maximum"]:
                        continue
                    candidate = dict(incumbent)
                    for name in members:
                        candidate[name] = value
                    key = json.dumps(candidate, sort_keys=True)
                    if key not in history:
                        yield group["name"], parameter, step, candidate
                step = round(step / 2.0, precision + 2)


def passes_regression_guard(result: dict, incumbent_metrics: dict | None, config: dict) -> bool:
    """Reject aggregate improvements that substantially damage a critical behavior."""
    if not incumbent_metrics:
        return True
    candidate_metrics = result.get("metrics_mean", {})
    limit = float(config.get("maximum_critical_regression", 0.20))
    for name in config.get("critical_metrics", []):
        previous = float(incumbent_metrics[name])
        current = float(candidate_metrics[name])
        if current > previous * (1.0 + limit):
            return False
    return True


def run_candidate(
    run_id: str,
    iteration: int,
    parameters: dict,
    repeats: int,
    run_dir: Path,
    trial_args: list[str],
) -> dict:
    candidate_name = f"iteration-{iteration:04d}"
    profile_path = run_dir / "profiles" / f"{candidate_name}.json"
    atomic_json(
        profile_path,
        {"candidate": {"description": candidate_name, "parameters": parameters}},
    )
    container_profile = Path("/results/pid_tuning/autotune") / run_id / "profiles" / profile_path.name
    trial_paths = []
    for repeat in range(1, repeats + 1):
        trial_name = f"auto_{run_id}_i{iteration:04d}_r{repeat}"
        environment = os.environ.copy()
        environment["PID_PROFILES_PATH"] = str(container_profile)
        command = [
            "bash",
            str(ROOT / "docker/flight_control/run_tuning_trial_fresh.sh"),
            "candidate",
            trial_name,
            "--output-dir",
            str(Path("/results/pid_tuning/autotune") / run_id / "trials"),
            *trial_args,
        ]
        subprocess.run(command, cwd=ROOT, env=environment, check=True)
        trial_paths.append(run_dir / "trials" / f"{trial_name}.json")
    result = evaluate(trial_paths)
    result.update({"iteration": iteration, "parameters": parameters})
    return result


def promote_parameters(parameters: dict, destination: Path) -> None:
    text = destination.read_text()
    for name, value in parameters.items():
        replacement = f"{name:<16}{float(value):.4f}"
        text, count = re.subn(
            rf"(?m)^{re.escape(name)}[ \t]+[-+0-9.eE]+[ \t]*$", replacement, text
        )
        if count != 1:
            raise RuntimeError(f"Expected one {name} line in {destination}, found {count}")
    destination.write_text(text)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT / "docker/flight_control/pid_search_space.json")
    parser.add_argument("--run-id", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-runtime", type=duration_seconds, default=2 * 3600.0)
    parser.add_argument("--max-evaluations", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--groups", nargs="*", default=[])
    parser.add_argument("--promote", action="store_true")
    parser.add_argument("--hover-seconds", type=float, default=25.0)
    parser.add_argument("--warmup-seconds", type=float, default=12.0)
    parser.add_argument("--move-seconds", type=float, default=2.0)
    parser.add_argument("--settle-seconds", type=float, default=7.0)
    parser.add_argument("--speed", type=float, default=0.7)
    args = parser.parse_args()
    if args.max_evaluations < 1 or args.repeats < 1:
        parser.error("max-evaluations and repeats must be positive")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.run_id):
        parser.error("run-id may contain only letters, numbers, '.', '_', and '-'")

    config = json.loads(args.config.read_text())
    groups = set(args.groups)
    known_groups = {item["name"] for item in config["groups"]}
    if groups - known_groups:
        parser.error(f"unknown groups: {sorted(groups - known_groups)}")
    baseline = load_persisted_baseline(
        config, ROOT / "docker/flight_control/isaac_sim_tuned.parm"
    )
    protocol = {
        "repeats": args.repeats,
        "groups": sorted(groups),
        "hover_seconds": args.hover_seconds,
        "warmup_seconds": args.warmup_seconds,
        "move_seconds": args.move_seconds,
        "settle_seconds": args.settle_seconds,
        "speed": args.speed,
    }
    run_dir = RESULTS_ROOT / args.run_id
    state_path = run_dir / "state.json"
    decision_log = run_dir / "decision-log.md"
    mission_run = MISSION_ROOT / "runs" / args.run_id
    mission_run.mkdir(parents=True, exist_ok=True)
    if args.resume:
        state = json.loads(state_path.read_text())
        if state.get("protocol", protocol) != protocol:
            parser.error(
                "resume trial protocol differs from the original run; "
                "use the original repeats/groups/timings/speed"
            )
        state.setdefault("protocol", protocol)
        # Migrate early optimizer state written before failed baselines were
        # retained as the comparison point.
        if state.get("iteration", 0) and state.get("incumbent_score") is None:
            baseline_result = state["history"][0]["evaluation"]
            state["incumbent_score"] = baseline_result["score"]
            state["incumbent_pass"] = baseline_result["pass"]
        state.setdefault("incumbent_pass", False)
        state.setdefault("incumbent_metrics_mean", None)
        state["status"] = "running"
    else:
        if state_path.exists():
            raise RuntimeError(f"Run {args.run_id} already exists; pass --resume")
        state = {
            "run_id": args.run_id,
            "started_at": datetime.now(timezone.utc).isoformat(),
            "iteration": 0,
            "incumbent_parameters": baseline,
            "incumbent_score": None,
            "incumbent_pass": False,
            "incumbent_metrics_mean": None,
            "history": [],
            "status": "running",
            "protocol": protocol,
        }
        append_decision(decision_log, f"# PID optimizer decision log\n\nRun `{args.run_id}` started.")

    deadline = time.monotonic() + args.max_runtime
    trial_args = [
        "--hover-seconds", str(args.hover_seconds),
        "--warmup-seconds", str(args.warmup_seconds),
        "--move-seconds", str(args.move_seconds),
        "--settle-seconds", str(args.settle_seconds),
        "--speed", str(args.speed),
    ]
    try:
        while state["iteration"] < args.max_evaluations and time.monotonic() < deadline:
            iteration = state["iteration"] + 1
            if state["iteration"] == 0:
                group, parameter, step = "baseline", "baseline", 0.0
                candidate = dict(state["incumbent_parameters"])
            else:
                history = {item["key"] for item in state["history"]}
                stream = candidate_stream(config, state["incumbent_parameters"], groups, history)
                try:
                    group, parameter, step, candidate = next(stream)
                except StopIteration:
                    state["status"] = "search_exhausted"
                    break
            key = json.dumps(candidate, sort_keys=True)
            started = datetime.now(timezone.utc).isoformat()
            try:
                result = run_candidate(
                    args.run_id, iteration, candidate, args.repeats, run_dir, trial_args
                )
            except subprocess.CalledProcessError as error:
                result = {
                    "pass": False,
                    # Keep state.json strict JSON; Infinity is accepted by
                    # Python but rejected by many downstream JSON readers.
                    "score": 1.0e12,
                    "error": f"trial process exited {error.returncode}",
                    "iteration": iteration,
                    "parameters": candidate,
                }
            incumbent_score = state["incumbent_score"]
            is_baseline = iteration == 1
            improved = is_baseline or (
                result.get("safe", False)
                and result["score"] <= incumbent_score - config["minimum_improvement"]
                and passes_regression_guard(
                    result, state.get("incumbent_metrics_mean"), config
                )
            )
            if improved:
                state["incumbent_parameters"] = candidate
                state["incumbent_score"] = result["score"]
                state["incumbent_pass"] = result["pass"]
                state["incumbent_metrics_mean"] = result.get("metrics_mean")
            record = {
                "key": key,
                "group": group,
                "parameter": parameter,
                "step": round(step, config["precision"]),
                "started_at": started,
                "improved": improved,
                "evaluation": result,
            }
            state["history"].append(record)
            state["iteration"] = iteration
            state["updated_at"] = datetime.now(timezone.utc).isoformat()
            atomic_json(run_dir / "evaluations" / f"iteration-{iteration:04d}.json", result)
            atomic_json(mission_run / "evaluations" / f"iteration-{iteration:04d}.json", result)
            atomic_json(state_path, state)
            verdict = "accepted" if improved else "rejected"
            entry = (
                f"## Iteration {iteration:04d}: {verdict}\n\n"
                f"- Group/parameter: `{group}` / `{parameter}`\n"
                f"- Step: `{step:.4f}`\n"
                f"- Score: `{result['score']}`\n"
                f"- Passed precision gates: `{result['pass']}`\n"
                f"- Passed bounded-flight gates: `{result.get('safe', False)}`\n"
                f"- Parameters: `{json.dumps(candidate, sort_keys=True)}`"
            )
            append_decision(decision_log, entry)
            append_decision(mission_run / "decision-log.md", entry)
    finally:
        if state.get("status") == "running":
            state["status"] = (
                "max_evaluations" if state["iteration"] >= args.max_evaluations else "max_runtime"
            )
        state["updated_at"] = datetime.now(timezone.utc).isoformat()
        atomic_json(state_path, state)
        best = {
            "profile": "autoresearch_best",
            "score": state["incumbent_score"],
            "pass": state.get("incumbent_pass", False),
            "parameters": state["incumbent_parameters"],
            "source_run": args.run_id,
            "iterations": state["iteration"],
        }
        atomic_json(run_dir / "best_profile.json", best)
        if args.promote:
            if args.repeats < 2:
                raise RuntimeError("Refusing promotion without at least two repeats")
            if state["incumbent_score"] is None or not state.get("incumbent_pass", False):
                raise RuntimeError("No passing evaluated profile is available to promote")
            promote_parameters(
                state["incumbent_parameters"],
                ROOT / "docker/flight_control/isaac_sim_tuned.parm",
            )
        print(json.dumps(best, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
