#!/usr/bin/env python3
"""Inspect and change ArduPilot controller parameters through MAVROS."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import re
import time
from pathlib import Path


PARAM_NODE = "/drone_1/param"
DEFAULT_PROFILES = Path("/workspace/docker/flight_control/pid_profiles.json")
ACTIVE_SNAPSHOT = Path("/results/pid_tuning/active_profile.json")


def ros2(*arguments: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["ros2", *arguments], text=True, capture_output=True, check=check
    )


def load_profiles(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def wait_for_parameter_node() -> None:
    result = ros2("node", "list")
    if PARAM_NODE not in result.stdout.splitlines():
        raise RuntimeError(
            f"{PARAM_NODE} is unavailable. Start the flight-control stack and wait for MAVROS."
        )
    # MAVROS exposes ArduPilot parameters through a ROS parameter node, but a
    # fresh process may not have populated that cache yet.  Pull explicitly so
    # live tuning is deterministic after every SITL restart.
    deadline = time.monotonic() + 75.0
    last_output = ""
    while time.monotonic() < deadline:
        pull = ros2(
            "service",
            "call",
            f"{PARAM_NODE}/pull",
            "mavros_msgs/srv/ParamPull",
            "{force_pull: true}",
        )
        last_output = pull.stdout + pull.stderr
        if "success=true" in pull.stdout.replace(" ", "").lower():
            return
        time.sleep(2.0)
    raise RuntimeError(f"MAVROS parameter pull did not complete: {last_output}")


def get_parameter(name: str) -> str:
    result = ros2("param", "get", PARAM_NODE, name)
    return result.stdout.strip()


def numeric_value(output: str) -> float:
    match = re.search(r"(?:is:|value:)\s*(-?[0-9.eE+]+)\s*$", output)
    if not match:
        raise RuntimeError(f"Could not parse ROS parameter output: {output}")
    return float(match.group(1))


def set_parameter(name: str, value: object, attempts: int = 4) -> None:
    """Set a live FCU parameter, tolerating short MAVLink cache races."""
    outputs = []
    for attempt in range(1, attempts + 1):
        result = ros2("param", "set", PARAM_NODE, name, str(value).lower(), check=False)
        output = result.stdout + result.stderr
        outputs.append(f"attempt {attempt}: {output.strip()}")
        if result.returncode == 0 and "successful" in result.stdout.lower():
            return
        time.sleep(0.75 * attempt)
    raise RuntimeError(f"Failed to set {name} after {attempts} attempts: {'; '.join(outputs)}")


def write_snapshot(profile: str, parameters: dict[str, float]) -> None:
    ACTIVE_SNAPSHOT.parent.mkdir(parents=True, exist_ok=True)
    with ACTIVE_SNAPSHOT.open("w", encoding="utf-8") as stream:
        json.dump(
            {"profile": profile, "parameters": parameters},
            stream,
            indent=2,
            sort_keys=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Apply ArduPilot PID changes live; no container restart is required."
    )
    parser.add_argument("--profiles", type=Path, default=DEFAULT_PROFILES)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("profiles")
    show = commands.add_parser("show")
    show.add_argument("names", nargs="*")
    apply_profile = commands.add_parser("apply")
    apply_profile.add_argument("profile")
    set_one = commands.add_parser("set")
    set_one.add_argument("name")
    set_one.add_argument("value", type=float)
    args = parser.parse_args()

    profiles = load_profiles(args.profiles)
    if args.command == "profiles":
        for name, profile in profiles.items():
            print(f"{name:16s} {profile.get('description', '')}")
        return

    wait_for_parameter_node()
    default_names = sorted(
        {key for profile in profiles.values() for key in profile["parameters"]}
    )
    if args.command == "show":
        for name in args.names or default_names:
            print(f"{name}: {get_parameter(name)}")
        return
    if args.command == "set":
        before = get_parameter(args.name)
        set_parameter(args.name, args.value)
        after = get_parameter(args.name)
        print(f"{args.name}: {before} -> {after}")
        active = {}
        if ACTIVE_SNAPSHOT.exists():
            active = json.loads(ACTIVE_SNAPSHOT.read_text()).get("parameters", {})
        active[args.name] = numeric_value(after)
        write_snapshot("custom", active)
        return
    if args.profile not in profiles:
        raise RuntimeError(f"Unknown profile {args.profile!r}; use the profiles command")
    print(f"Applying {args.profile}: {profiles[args.profile].get('description', '')}")
    active = {}
    for name, value in profiles[args.profile]["parameters"].items():
        before = get_parameter(name)
        set_parameter(name, value)
        after = get_parameter(name)
        print(f"  {name}: {before} -> {after}")
        active[name] = numeric_value(after)
    write_snapshot(args.profile, active)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"PID tuning error: {error}", file=sys.stderr)
        sys.exit(1)
