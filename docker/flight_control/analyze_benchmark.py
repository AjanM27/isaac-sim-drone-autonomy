#!/usr/bin/env python3
"""Calculate ATE, command tracking, and RTAB-Map health for one benchmark."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def finite_rows(array: np.ndarray) -> np.ndarray:
    return np.all(np.isfinite(array), axis=1)


def stats(error: np.ndarray) -> dict[str, float | int]:
    error = error[np.isfinite(error)]
    if not len(error):
        return {"samples": 0, "rmse_m": math.nan, "max_m": math.nan, "final_m": math.nan}
    return {
        "samples": int(len(error)),
        "rmse_m": float(np.sqrt(np.mean(error**2))),
        "max_m": float(np.max(error)),
        "final_m": float(error[-1]),
    }


def first_pose_align(reference: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    """Translation-only alignment, preserving estimator scale and yaw errors."""
    valid = finite_rows(reference) & finite_rows(estimate)
    aligned = np.full_like(estimate, np.nan)
    if not np.any(valid):
        return aligned
    first = int(np.flatnonzero(valid)[0])
    aligned[valid] = estimate[valid] - estimate[first] + reference[first]
    return aligned


def initial_pose_align(
    reference: np.ndarray,
    estimate: np.ndarray,
    reference_yaw_deg: np.ndarray,
    estimate_yaw_deg: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Align only at t=0, preserving every subsequent scale/yaw/drift error."""
    valid = finite_rows(reference) & finite_rows(estimate)
    valid &= np.isfinite(reference_yaw_deg) & np.isfinite(estimate_yaw_deg)
    aligned = np.full_like(estimate, np.nan)
    if not np.any(valid):
        return aligned, math.nan
    first = int(np.flatnonzero(valid)[0])
    yaw_delta_deg = float(reference_yaw_deg[first] - estimate_yaw_deg[first])
    yaw_delta = math.radians(yaw_delta_deg)
    cosine = math.cos(yaw_delta)
    sine = math.sin(yaw_delta)
    rotation = np.asarray(((cosine, sine), (-sine, cosine)))
    aligned[valid, :2] = (
        (estimate[valid, :2] - estimate[first, :2]) @ rotation
        + reference[first, :2]
    )
    aligned[valid, 2] = estimate[valid, 2] - estimate[first, 2] + reference[first, 2]
    return aligned, yaw_delta_deg


def numeric(rows: list[dict[str, str]], names: tuple[str, ...]) -> np.ndarray:
    return np.asarray([[float(row[name]) for name in names] for row in rows], dtype=float)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir
    all_rows = list(csv.DictReader((run_dir / "trajectory.csv").open(encoding="utf-8")))
    # Startup/takeoff can be long while EKF initializes, and landing duration
    # varies with SITL landed-state detection. Keep both in the raw trace, but
    # evaluate trajectory accuracy only over the identical commanded route.
    rows = [row for row in all_rows if row["phase"] not in ("preflight", "startup_takeoff", "landing")]
    if len(rows) < 20:
        raise RuntimeError(f"Only {len(rows)} dynamic-trajectory samples were recorded")
    metadata = json.loads((run_dir / "run_metadata.json").read_text())

    time_s = numeric(rows, ("time",))[:, 0]
    time_s -= time_s[0]
    gt = numeric(rows, ("gt_x", "gt_y", "gt_z"))
    mav = numeric(rows, ("mav_x", "mav_y", "mav_z"))
    slam = numeric(rows, ("slam_x", "slam_y", "slam_z"))
    desired_mav = numeric(rows, ("desired_mav_x", "desired_mav_y", "desired_mav_z"))
    gt_yaw = numeric(rows, ("gt_yaw_deg",))[:, 0]
    slam_yaw = numeric(rows, ("slam_yaw_deg",))[:, 0]

    # MAVROS local ENU and Isaac world are related by the validated Iris mount:
    # Isaac x=MAVROS y, Isaac y=-MAVROS x, z=z. Align translation at first pose.
    mav_world_raw = np.column_stack((mav[:, 1], -mav[:, 0], mav[:, 2]))
    mav_world = first_pose_align(gt, mav_world_raw)
    desired_world = np.column_stack((desired_mav[:, 1], -desired_mav[:, 0], desired_mav[:, 2]))
    desired_world = first_pose_align(gt, desired_world)

    # RTAB-Map odometry owns an arbitrary local frame. Align its first valid
    # pose exactly to Isaac, then preserve every later yaw, scale, and drift
    # error. A whole-trajectory least-squares fit would incorrectly move t=0.
    slam_world, slam_alignment_yaw = initial_pose_align(gt, slam, gt_yaw, slam_yaw)
    mav_error = np.linalg.norm(mav_world - gt, axis=1)
    slam_error = np.linalg.norm(slam_world - gt, axis=1)
    tracking_error = np.linalg.norm(desired_world - gt, axis=1)
    valid_mav_raw = finite_rows(gt) & finite_rows(mav_world_raw)
    valid_slam_raw = finite_rows(gt) & finite_rows(slam)
    mav_raw_first = int(np.flatnonzero(valid_mav_raw)[0])
    slam_raw_first = int(np.flatnonzero(valid_slam_raw)[0])

    valid_slam = finite_rows(slam_world)
    slam_steps = np.linalg.norm(np.diff(slam_world[valid_slam], axis=0), axis=1)
    gt_steps = np.linalg.norm(np.diff(gt[valid_slam], axis=0), axis=1)
    jump_excess = slam_steps - gt_steps if len(slam_steps) else np.asarray([])

    initial = np.flatnonzero(valid_slam)[0] if np.any(valid_slam) else None
    final = np.flatnonzero(valid_slam)[-1] if np.any(valid_slam) else None
    slam_loop_drift = (
        float(np.linalg.norm((slam_world[final] - slam_world[initial]) - (gt[final] - gt[initial])))
        if initial is not None and final is not None
        else math.nan
    )

    diagnostics_path = run_dir / "slam_diagnostics.csv"
    icp_rows = list(csv.DictReader(diagnostics_path.open(encoding="utf-8"))) if diagnostics_path.exists() else []
    if icp_rows:
        start_time = float(rows[0]["time"])
        end_time = float(rows[-1]["time"])
        icp_rows = [row for row in icp_rows if start_time <= float(row["time"]) <= end_time]
    icp_times = np.asarray([float(r["processing_time_s"]) for r in icp_rows], dtype=float)
    icp_ratios = np.asarray([float(r["inlier_ratio"]) for r in icp_rows], dtype=float)
    icp_lost = np.asarray([int(r["lost"]) for r in icp_rows], dtype=int)
    scan_counts = np.asarray([int(r["scan_points"]) for r in icp_rows], dtype=float)
    scan_counts = scan_counts[scan_counts > 0]
    map_counts = np.asarray([int(r["map_points"]) for r in icp_rows], dtype=float) if icp_rows else np.asarray([0.0])

    diagnostic_summary = {}
    summary_path = diagnostics_path.with_suffix(".summary")
    if summary_path.exists():
        for line in summary_path.read_text().splitlines():
            key, _, value = line.partition("=")
            diagnostic_summary[key] = value
    voxel_ratio = metadata.get("final_map_voxel_unique_ratio")
    if diagnostic_summary.get("voxel_unique_ratio") not in (None, "nan"):
        voxel_ratio = float(diagnostic_summary["voxel_unique_ratio"])

    icp_time_valid = icp_times[np.isfinite(icp_times) & (icp_times >= 0)]
    ratio_valid = icp_ratios[np.isfinite(icp_ratios)]
    lost_rate = float(np.mean(icp_lost == 1)) if len(icp_lost) else math.nan
    max_jump = float(np.max(jump_excess)) if len(jump_excess) else math.nan
    slam_metrics = stats(slam_error)
    reliable = bool(
        slam_metrics["samples"] > 20
        and slam_metrics["rmse_m"] <= 0.30
        and slam_metrics["max_m"] <= 0.75
        and lost_rate <= 0.01
        and (not math.isfinite(max_jump) or max_jump <= 0.30)
    )

    metrics = {
        "run": metadata,
        "evaluation": {
            "alignment": "Both estimates are anchored exactly at their first valid Isaac sample; no trajectory-wide fitting or scale",
            "slam_alignment_yaw_deg": slam_alignment_yaw,
            "raw_initial_offset_m": {
                "mavros": float(np.linalg.norm(mav_world_raw[mav_raw_first] - gt[mav_raw_first])),
                "slam": float(np.linalg.norm(slam[slam_raw_first] - gt[slam_raw_first])),
            },
            "aligned_initial_error_m": {
                "mavros": float(mav_error[np.flatnonzero(np.isfinite(mav_error))[0]]),
                "slam": float(slam_error[np.flatnonzero(np.isfinite(slam_error))[0]]),
            },
            "mavros_position": stats(mav_error),
            "slam_position": slam_metrics,
            "command_tracking": stats(tracking_error),
            "slam_final_loop_drift_m": slam_loop_drift,
        },
        "icp": {
            "updates": int(len(icp_rows)),
            "lost_updates": int(np.sum(icp_lost == 1)),
            "lost_rate": lost_rate,
            "processing_mean_ms": float(np.mean(icp_time_valid) * 1000.0) if len(icp_time_valid) else math.nan,
            "processing_p95_ms": float(np.percentile(icp_time_valid, 95) * 1000.0) if len(icp_time_valid) else math.nan,
            "processing_max_ms": float(np.max(icp_time_valid) * 1000.0) if len(icp_time_valid) else math.nan,
            "inlier_ratio_mean": float(np.mean(ratio_valid)) if len(ratio_valid) else math.nan,
            "inlier_ratio_min": float(np.min(ratio_valid)) if len(ratio_valid) else math.nan,
        },
        "scan_map_stability": {
            "scan_samples": int(len(scan_counts)),
            "scan_points_mean": float(np.mean(scan_counts)) if len(scan_counts) else math.nan,
            "scan_points_cv": float(np.std(scan_counts) / np.mean(scan_counts)) if len(scan_counts) else math.nan,
            "map_points_final": int(map_counts[-1]),
            "map_points_max": int(np.max(map_counts)),
            "final_map_voxel_unique_ratio": voxel_ratio,
            "max_odometry_jump_excess_m": max_jump,
            "odometry_discontinuity_detected": bool(math.isfinite(max_jump) and max_jump > 0.30),
            "map_duplication_suspected": bool((voxel_ratio or 1.0) < 0.85),
        },
        "without_deskewing_reliable": reliable,
        "deskew_recommendation": (
            "Current XYZ-only scans passed this speed; evaluate all three speeds before prioritizing deskewing."
            if reliable
            else "Prioritize per-point timestamps and RTAB-Map deskewing before increasing flight speed."
        ),
    }
    (run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(gt[:, 0], gt[:, 1], label="Isaac ground truth", linewidth=2)
    ax.plot(mav_world[:, 0], mav_world[:, 1], label="ArduPilot/MAVROS", alpha=0.85)
    ax.plot(slam_world[:, 0], slam_world[:, 1], label="RTAB-Map", alpha=0.85)
    ax.plot(desired_world[:, 0], desired_world[:, 1], "--", label="Commanded", alpha=0.7)
    ax.set(xlabel="Isaac X (m)", ylabel="Isaac Y (m)", title=f"Checkpoint-1 trajectory: {metadata['speed_mps']:.2f} m/s")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(run_dir / "trajectory_xy.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(time_s, mav_error, label="MAVROS vs ground truth")
    ax.plot(time_s, slam_error, label="RTAB-Map vs ground truth")
    ax.plot(time_s, tracking_error, label="Command tracking", alpha=0.75)
    ax.set(xlabel="Simulation time (s)", ylabel="3D position error (m)", title="Trajectory error over time")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(run_dir / "position_error.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(time_s, gt[:, 2], label="Ground truth Z")
    axes[0].plot(time_s, desired_world[:, 2], "--", label="Commanded Z")
    axes[0].set_ylabel("Height (m)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(time_s, numeric(rows, ("gt_yaw_deg",))[:, 0], label="Ground truth yaw")
    axes[1].plot(time_s, numeric(rows, ("desired_yaw_deg",))[:, 0] - 90.0, "--", label="Commanded Isaac yaw")
    axes[1].set(xlabel="Simulation time (s)", ylabel="Yaw (deg)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(run_dir / "altitude_yaw.png", dpi=160)
    plt.close(fig)

    report = f"""# Checkpoint-1 run: {metadata['name']}

- Requested speed: {metadata['speed_mps']:.2f} m/s
- MAVROS position RMSE / max / final: {metrics['evaluation']['mavros_position']['rmse_m']:.3f} / {metrics['evaluation']['mavros_position']['max_m']:.3f} / {metrics['evaluation']['mavros_position']['final_m']:.3f} m
- RTAB-Map position RMSE / max / final: {slam_metrics['rmse_m']:.3f} / {slam_metrics['max_m']:.3f} / {slam_metrics['final_m']:.3f} m
- RTAB-Map loop drift: {slam_loop_drift:.3f} m
- ICP mean / p95 / max: {metrics['icp']['processing_mean_ms']:.1f} / {metrics['icp']['processing_p95_ms']:.1f} / {metrics['icp']['processing_max_ms']:.1f} ms
- ICP lost updates: {metrics['icp']['lost_updates']} / {metrics['icp']['updates']}
- Mean/min ICP inlier ratio: {metrics['icp']['inlier_ratio_mean']:.3f} / {metrics['icp']['inlier_ratio_min']:.3f}
- Final/max map points: {metrics['scan_map_stability']['map_points_final']} / {metrics['scan_map_stability']['map_points_max']}
- Map duplication suspected: {metrics['scan_map_stability']['map_duplication_suspected']}
- Reliable without deskewing at this speed: {reliable}

{metrics['deskew_recommendation']}
"""
    (run_dir / "report.md").write_text(report)
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
