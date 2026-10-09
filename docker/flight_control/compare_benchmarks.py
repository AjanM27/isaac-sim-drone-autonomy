#!/usr/bin/env python3
"""Create a concise three-speed Checkpoint-1 comparison."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    metrics = [json.loads(path.read_text()) for path in sorted(args.root.glob("square_*/metrics.json"))]
    if not metrics:
        raise RuntimeError(f"No benchmark metrics found under {args.root}")
    metrics.sort(key=lambda item: item["run"]["speed_mps"])
    speeds = [item["run"]["speed_mps"] for item in metrics]
    slam_rmse = [item["evaluation"]["slam_position"]["rmse_m"] for item in metrics]
    mav_rmse = [item["evaluation"]["mavros_position"]["rmse_m"] for item in metrics]
    icp_p95 = [item["icp"]["processing_p95_ms"] for item in metrics]
    drift = [item["evaluation"]["slam_final_loop_drift_m"] for item in metrics]

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(speeds, mav_rmse, "o-", label="MAVROS")
    axes[0].plot(speeds, slam_rmse, "o-", label="RTAB-Map")
    axes[0].plot(speeds, drift, "o--", label="SLAM loop drift")
    axes[0].set(xlabel="Command speed (m/s)", ylabel="Error (m)", title="Pose accuracy vs speed")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(speeds, icp_p95, "o-", color="tab:purple")
    axes[1].axhline(200.0, linestyle="--", color="tab:red", label="5 Hz scan interval")
    axes[1].set(xlabel="Command speed (m/s)", ylabel="ICP p95 (ms)", title="ICP processing budget")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(args.root / "speed_comparison.png", dpi=160)
    plt.close(fig)

    lines = [
        "# Checkpoint 1: dynamic SLAM speed comparison",
        "",
        "| Speed | MAVROS RMSE | SLAM RMSE | SLAM max | Final loop drift | ICP p95 | Lost | No-deskew reliable |",
        "|---:|---:|---:|---:|---:|---:|---:|:---:|",
    ]
    for item in metrics:
        speed = item["run"]["speed_mps"]
        mav = item["evaluation"]["mavros_position"]
        slam = item["evaluation"]["slam_position"]
        icp = item["icp"]
        lines.append(
            f"| {speed:.2f} m/s | {mav['rmse_m']:.3f} m | {slam['rmse_m']:.3f} m | "
            f"{slam['max_m']:.3f} m | {item['evaluation']['slam_final_loop_drift_m']:.3f} m | "
            f"{icp['processing_p95_ms']:.1f} ms | {icp['lost_updates']}/{icp['updates']} | "
            f"{item['without_deskewing_reliable']} |"
        )
    reliable_speeds = [item["run"]["speed_mps"] for item in metrics if item["without_deskewing_reliable"]]
    lines.extend(
        [
            "",
            "## Decision",
            "",
            (
                f"XYZ-only scans without deskewing passed through {max(reliable_speeds):.2f} m/s under this benchmark."
                if reliable_speeds
                else "No tested speed met the no-deskew reliability gates."
            ),
            "Per-point timestamps and deskewing are the next priority if the 1.0 m/s run fails, "
            "or before increasing speed/rotation rate beyond this validated envelope.",
            "",
            "Reliability gate: SLAM RMSE <= 0.30 m, maximum error <= 0.75 m, lost rate <= 1%, "
            "and no odometry jump exceeding ground-truth motion by 0.30 m.",
        ]
    )
    (args.root / "SUMMARY.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
