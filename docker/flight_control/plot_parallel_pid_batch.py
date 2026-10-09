#!/usr/bin/env python3
"""Generate diagnostic plots and a compact comparison table for one PID batch."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


GROUPS = (
    ("Position (ENU)", ("enu_x", "enu_y", "enu_z", "ideal_x", "ideal_y", "ideal_z"), "m"),
    ("Position error (actual - ideal)",
     ("position_error_x", "position_error_y", "position_error_z", "position_error_3d"), "m"),
    ("Velocity (ENU)", ("enu_vx", "enu_vy", "enu_vz", "command_vx", "command_vy", "command_vz"), "m/s"),
    ("Acceleration", ("ax", "ay", "az"), "m/s²"),
    ("Attitude", ("roll_deg", "pitch_deg", "yaw_deg"), "deg"),
)


def rows(path: Path):
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def plot_candidate(csv_path: Path, summary: dict, output: Path) -> None:
    data = rows(csv_path)
    time_values = np.array([float(row["time"]) for row in data])
    time_values -= time_values[0]
    fig, axes = plt.subplots(len(GROUPS), 1, figsize=(12, 16), sharex=True)
    for axis, (title, fields, units) in zip(axes, GROUPS):
        for field in fields:
            axis.plot(time_values, [float(row.get(field, "nan")) for row in data], label=field)
        axis.set_ylabel(units)
        axis.set_title(title)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right", ncols=3)
    axes[-1].set_xlabel("wall time since measurement start (s)")
    hover = summary["hover"]
    fig.suptitle(
        f"env {summary['environment']} | score={summary['score']:.4f} | "
        f"hover max={hover.get('xy_max_m', float('nan')):.4f} m | pass={summary['pass']}"
    )
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("batch_dir", type=Path)
    args = parser.parse_args()
    summaries = []
    result_paths = [
        path for path in args.batch_dir.glob("env_*.json")
        if re.fullmatch(r"env_\d+\.json", path.name)
    ]
    for path in sorted(result_paths):
        summary = json.loads(path.read_text())
        index = int(summary["environment"])
        plot_candidate(
            args.batch_dir / f"env_{index}.csv",
            summary,
            args.batch_dir / f"env_{index}_diagnostics.png",
        )
        summaries.append(summary)
    if not summaries:
        raise RuntimeError(f"No env_*.json files found under {args.batch_dir}")

    labels = [f"env_{item['environment']}" for item in summaries]
    score = [item["score"] for item in summaries]
    xy_rms = [item["hover"].get("xy_rms_m", np.nan) for item in summaries]
    xy_max = [item["hover"].get("xy_max_m", np.nan) for item in summaries]
    high_speed_z = [
        item.get("dynamic", {}).get("high_speed_altitude_deviation_m", np.nan)
        for item in summaries
    ]
    abrupt_z = [
        item.get("dynamic", {}).get("abrupt_reversal_altitude_deviation_m", np.nan)
        for item in summaries
    ]
    yaw_error = [
        max((phase.get("yaw_final_error_deg", np.nan) for phase in item.get("yaw", {}).values()),
            default=np.nan)
        for item in summaries
    ]
    yaw_overshoot = [
        max((phase.get("yaw_overshoot_deg", np.nan) for phase in item.get("yaw", {}).values()),
            default=np.nan)
        for item in summaries
    ]
    ideal_final = [
        item.get("dynamic", {}).get("worst_ideal_position_final_error_m", np.nan)
        for item in summaries
    ]
    ideal_tail = [
        item.get("dynamic", {}).get("worst_ideal_position_tail_rmse_m", np.nan)
        for item in summaries
    ]
    standard_stop = [
        item.get("dynamic", {}).get("standard_stop_displacement_m", np.nan)
        for item in summaries
    ]
    high_speed_stop = [
        item.get("dynamic", {}).get("high_speed_stop_displacement_m", np.nan)
        for item in summaries
    ]
    fig, axes = plt.subplots(2, 3, figsize=(19, 9))
    axes = axes.ravel()
    axes[0].bar(labels, score)
    axes[0].set_title("Total objective (lower is better)")
    axes[0].set_ylabel("normalized score")
    width = 0.38
    positions = np.arange(len(labels))
    axes[1].bar(positions - width / 2, xy_rms, width, label="XY RMS")
    axes[1].bar(positions + width / 2, xy_max, width, label="XY max")
    axes[1].axhline(0.10, color="red", linestyle="--", label="0.10 m hard gate")
    axes[1].set_xticks(positions, labels)
    axes[1].set_title("Hover position error")
    axes[1].set_ylabel("m")
    axes[1].legend()
    axes[2].bar(positions - width / 2, high_speed_z, width, label="3 m/s")
    axes[2].bar(positions + width / 2, abrupt_z, width, label="full reversal")
    axes[2].axhline(0.05, color="red", linestyle="--", label="0.05 m hard gate")
    axes[2].set_xticks(positions, labels)
    axes[2].set_title("Altitude coupling during aggressive XY motion")
    axes[2].set_ylabel("maximum deviation (m)")
    axes[2].legend()
    axes[3].bar(positions - width / 2, yaw_error, width, label="final error")
    axes[3].bar(positions + width / 2, yaw_overshoot, width, label="overshoot")
    axes[3].axhline(5.0, color="red", linestyle="--", label="5 deg final-error gate")
    axes[3].set_xticks(positions, labels)
    axes[3].set_title("Worst yaw-step response")
    axes[3].set_ylabel("degrees")
    axes[3].legend()
    axes[4].bar(positions - width / 2, ideal_final, width, label="final error")
    axes[4].bar(positions + width / 2, ideal_tail, width, label="tail RMSE")
    axes[4].axhline(0.10, color="red", linestyle="--", label="0.10 m convergence gate")
    axes[4].set_xticks(positions, labels)
    axes[4].set_title("Actual minus continuous ideal position")
    axes[4].set_ylabel("3D error (m)")
    axes[4].legend()
    axes[5].bar(positions - width / 2, standard_stop, width, label="1 m/s stop")
    axes[5].bar(positions + width / 2, high_speed_stop, width, label="3 m/s stop")
    axes[5].set_xticks(positions, labels)
    axes[5].set_title("Braking travel")
    axes[5].set_ylabel("m")
    axes[5].legend()
    for axis in axes:
        axis.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.batch_dir / "candidate_comparison.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
