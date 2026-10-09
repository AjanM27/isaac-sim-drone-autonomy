#!/usr/bin/env python3
"""Plot repeat-averaged PID metrics and ground-truth response traces."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("trials", nargs="+", type=Path)
    args = parser.parse_args()
    summaries = [json.loads(path.read_text()) for path in args.trials]
    labels = [item["name"] for item in summaries]
    metrics = {
        "Hover XY RMS (cm)": [item["hover"]["xy_rms_m"] * 100 for item in summaries],
        "Hover Z RMS (cm)": [item["hover"]["z_rms_m"] * 100 for item in summaries],
        "Yaw peak-to-peak (deg)": [item["hover"]["yaw_peak_to_peak_deg"] for item in summaries],
        "Worst stop (cm)": [item["aggregate"]["worst_stop_displacement_m"] * 100 for item in summaries],
    }
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for ax, (title, values) in zip(axes.flat, metrics.items()):
        ax.bar(range(len(values)), values)
        ax.set_title(title)
        ax.set_xticks(range(len(values)), labels, rotation=25, ha="right")
        ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    for path, label in zip(args.trials, labels):
        rows = list(csv.DictReader(path.with_suffix(".csv").open(encoding="utf-8")))
        hover = [row for row in rows if row["phase"] == "hover"]
        if hover:
            t = np.asarray([float(row["time"]) for row in hover])
            t -= t[0]
            x = np.asarray([float(row["x"]) for row in hover])
            y = np.asarray([float(row["y"]) for row in hover])
            z = np.asarray([float(row["z"]) for row in hover])
            yaw = np.unwrap(np.radians([float(row["yaw_deg"]) for row in hover]))
            axes[0].plot(t, np.hypot(x - x[0], y - y[0]) * 100, label=label)
            axes[1].plot(t, (z - np.mean(z[: min(20, len(z))])) * 100, label=label)
            axes[2].plot(t, np.degrees(yaw - yaw[0]), label=label)
    axes[0].set_ylabel("XY drift (cm)")
    axes[1].set_ylabel("Z error (cm)")
    axes[2].set_ylabel("Yaw change (deg)")
    axes[2].set_xlabel("Simulation time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output.with_name(args.output.stem + "_responses.png"), dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
