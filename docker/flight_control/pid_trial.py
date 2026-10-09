#!/usr/bin/env python3
"""Repeatable hover and direction-change trial using Isaac ground truth."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from copy import deepcopy
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

from teleop import BRAKE_STABLE_SECONDS, Teleop


def euler(q) -> tuple[float, float, float]:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return tuple(math.degrees(v) for v in (roll, pitch, math.atan2(siny, cosy)))


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else 0.0


class PidTrial(Teleop):
    def __init__(self) -> None:
        super().__init__()
        self.phase = "preflight"
        self.samples: list[dict[str, float | str]] = []
        self.latest_velocity = (0.0, 0.0, 0.0)
        self.create_subscription(TFMessage, "/tf", self.tf_callback, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped,
            "/drone_1/local_position/velocity_local",
            self.velocity_callback,
            qos_profile_sensor_data,
        )

    def velocity_callback(self, message: TwistStamped) -> None:
        velocity = message.twist.linear
        self.latest_velocity = (velocity.x, velocity.y, velocity.z)

    def tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") != "world"
                or transform.child_frame_id.lstrip("/") != "drone_base"
            ):
                continue
            p = transform.transform.translation
            roll, pitch, yaw = euler(transform.transform.rotation)
            stamp = transform.header.stamp
            est = self.local_pose.pose.position if self.local_pose else None
            self.samples.append({
                "time": stamp.sec + stamp.nanosec * 1.0e-9,
                "phase": self.phase,
                "x": p.x, "y": p.y, "z": p.z,
                "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
                "est_x": est.x if est else math.nan,
                "est_y": est.y if est else math.nan,
                "est_z": est.z if est else math.nan,
                "est_vx": self.latest_velocity[0],
                "est_vy": self.latest_velocity[1],
                "est_vz": self.latest_velocity[2],
            })

    def run_velocity(self, phase: str, duration: float, velocity: tuple[float, float, float]) -> None:
        if self.state.mode != "GUIDED" and not self.request_flight_mode("GUIDED"):
            raise RuntimeError(f"ArduPilot did not enter GUIDED before {phase}")
        self.phase = phase
        finish = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < finish:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.send_velocity(*velocity)
            time.sleep(0.04)

    def run_hold(self, phase: str, duration: float) -> None:
        target = self.capture_position_hold()
        if target is None:
            raise RuntimeError("No local pose was available when position hold was requested")
        target = deepcopy(target)
        self.phase = phase
        finish = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < finish:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.send_position_hold(target)
            time.sleep(0.04)

    def run_brake_hold(self, phase: str, duration: float) -> None:
        """Exercise the same Guided zero-velocity brake used by teleoperation."""
        self.phase = phase
        finish = time.monotonic() + duration
        stopped_since = None
        target = None
        while rclpy.ok() and time.monotonic() < finish:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.monotonic()
            if target is None:
                self.send_active_brake()
                if self.is_nearly_stopped():
                    stopped_since = stopped_since or now
                else:
                    stopped_since = None
                if (
                    stopped_since is not None
                    and now - stopped_since >= BRAKE_STABLE_SECONDS
                ):
                    target = self.capture_position_hold()
            if target is not None:
                self.send_position_hold(target)
            time.sleep(0.04)


def phase_samples(samples, phase: str):
    return [sample for sample in samples if sample["phase"] == phase]


def attitude_metrics(samples) -> dict[str, float]:
    if not samples:
        return {}
    return {
        "roll_rms_deg": rms([float(s["roll_deg"]) for s in samples]),
        "pitch_rms_deg": rms([float(s["pitch_deg"]) for s in samples]),
        "max_abs_roll_deg": max(abs(float(s["roll_deg"])) for s in samples),
        "max_abs_pitch_deg": max(abs(float(s["pitch_deg"])) for s in samples),
        "yaw_peak_to_peak_deg": max(float(s["yaw_deg"]) for s in samples) - min(float(s["yaw_deg"]) for s in samples),
    }


def ground_velocity_metrics(samples) -> dict[str, float]:
    speeds = []
    for previous, current in zip(samples, samples[1:]):
        dt = float(current["time"]) - float(previous["time"])
        if dt <= 1.0e-6:
            continue
        speeds.append(math.hypot(
            (float(current["x"]) - float(previous["x"])) / dt,
            (float(current["y"]) - float(previous["y"])) / dt,
        ))
    return {
        "ground_xy_speed_rms_mps": rms(speeds),
        "ground_xy_speed_max_mps": max(speeds, default=0.0),
    }


def estimator_metrics(samples) -> dict[str, float]:
    """Compare relative MAVROS motion with Isaac's world-frame truth."""
    names = ("est_x", "est_y", "est_z")
    usable = [s for s in samples if all(math.isfinite(float(s[name])) for name in names)]
    if len(usable) < 2:
        return {}
    ref_count = min(len(usable), 20)
    gx0 = statistics.fmean(float(s["x"]) for s in usable[:ref_count])
    gy0 = statistics.fmean(float(s["y"]) for s in usable[:ref_count])
    gz0 = statistics.fmean(float(s["z"]) for s in usable[:ref_count])
    ex0 = statistics.fmean(float(s["est_x"]) for s in usable[:ref_count])
    ey0 = statistics.fmean(float(s["est_y"]) for s in usable[:ref_count])
    ez0 = statistics.fmean(float(s["est_z"]) for s in usable[:ref_count])
    xy_errors = []
    z_errors = []
    for sample in usable:
        # MAVROS local pose is ENU: +Y maps to Isaac +X and -X maps to Isaac +Y.
        estimated_dx = float(sample["est_y"]) - ey0
        estimated_dy = -(float(sample["est_x"]) - ex0)
        estimated_dz = float(sample["est_z"]) - ez0
        truth_dx = float(sample["x"]) - gx0
        truth_dy = float(sample["y"]) - gy0
        truth_dz = float(sample["z"]) - gz0
        xy_errors.append(math.hypot(truth_dx - estimated_dx, truth_dy - estimated_dy))
        z_errors.append(truth_dz - estimated_dz)
    return {
        "estimator_xy_rms_m": rms(xy_errors),
        "estimator_xy_max_m": max(xy_errors),
        "estimator_z_rms_m": rms(z_errors),
        "estimator_z_max_abs_m": max(abs(value) for value in z_errors),
    }


def summarize_hover(samples) -> dict[str, float]:
    if len(samples) < 2:
        return {}
    ref_count = max(1, min(len(samples), 20))
    ref_x = statistics.fmean(float(s["x"]) for s in samples[:ref_count])
    ref_y = statistics.fmean(float(s["y"]) for s in samples[:ref_count])
    ref_z = statistics.fmean(float(s["z"]) for s in samples[:ref_count])
    radial = [math.hypot(float(s["x"]) - ref_x, float(s["y"]) - ref_y) for s in samples]
    z_error = [float(s["z"]) - ref_z for s in samples]
    return {
        "duration_sim_s": float(samples[-1]["time"]) - float(samples[0]["time"]),
        "reference_x_m": ref_x, "reference_y_m": ref_y, "reference_z_m": ref_z,
        "xy_rms_m": rms(radial), "xy_max_m": max(radial), "xy_final_m": radial[-1],
        "z_rms_m": rms(z_error), "z_peak_to_peak_m": max(z_error) - min(z_error),
        "z_final_error_m": z_error[-1],
        **attitude_metrics(samples),
        **ground_velocity_metrics(samples),
        **estimator_metrics(samples),
    }


def summarize_stop(samples) -> dict[str, float | None]:
    if len(samples) < 2:
        return {}
    start = samples[0]
    dx = [float(s["x"]) - float(start["x"]) for s in samples]
    dy = [float(s["y"]) - float(start["y"]) for s in samples]
    radial = [math.hypot(x, y) for x, y in zip(dx, dy)]
    dz = [float(s["z"]) - float(start["z"]) for s in samples]
    settled = None
    for index, sample in enumerate(samples):
        remaining = [s for s in samples[index:] if float(s["time"]) <= float(sample["time"]) + 1.0]
        if remaining and max(math.hypot(float(s["est_vx"]), float(s["est_vy"])) for s in remaining) <= 0.08:
            settled = float(sample["time"]) - float(start["time"])
            break
    return {
        "max_stop_displacement_m": max(radial), "final_stop_error_m": radial[-1],
        "max_abs_x_displacement_m": max(abs(value) for value in dx),
        "max_abs_y_displacement_m": max(abs(value) for value in dy),
        "z_rise_after_stop_m": max(dz), "z_drop_after_stop_m": min(dz),
        "z_peak_to_peak_m": max(dz) - min(dz), "settling_time_s": settled,
        **attitude_metrics(samples),
    }


def write_outputs(csv_path: Path, summary_path: Path, samples, summary: dict) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="pid_trial")
    parser.add_argument("--hover-seconds", type=float, default=30.0)
    parser.add_argument("--warmup-seconds", type=float, default=12.0)
    parser.add_argument("--move-seconds", type=float, default=2.0)
    parser.add_argument("--settle-seconds", type=float, default=7.0)
    parser.add_argument("--speed", type=float, default=0.7)
    parser.add_argument("--hover-only", action="store_true")
    parser.add_argument(
        "--stop-policy",
        choices=("immediate-position", "brake-then-lock"),
        default="brake-then-lock",
    )
    parser.add_argument(
        "--hover-policy", choices=("velocity", "position"), default="position"
    )
    parser.add_argument("--output-dir", type=Path, default=Path("/results/pid_tuning"))
    args = parser.parse_args()

    rclpy.init()
    node = PidTrial()
    try:
        node.prepare_guided()
        node.run_velocity("warmup", args.warmup_seconds, (0.0, 0.0, 0.0))
        if args.hover_policy == "position":
            node.run_hold("hover", args.hover_seconds)
        else:
            node.run_velocity("hover", args.hover_seconds, (0.0, 0.0, 0.0))
        directions = {
            "x_positive": (args.speed, 0.0, 0.0),
            "x_negative": (-args.speed, 0.0, 0.0),
            "y_positive": (0.0, args.speed, 0.0),
            "y_negative": (0.0, -args.speed, 0.0),
        }
        if not args.hover_only:
            for name, velocity in directions.items():
                node.run_velocity(f"move_{name}", args.move_seconds, velocity)
                if args.stop_policy == "immediate-position":
                    node.run_hold(f"stop_{name}", args.settle_seconds)
                else:
                    node.run_brake_hold(f"stop_{name}", args.settle_seconds)
        measured = [s for s in node.samples if s["phase"] not in ("preflight", "warmup")]
        if len(measured) < 50:
            raise RuntimeError(f"Only {len(measured)} ground-truth samples were recorded")
        # pid_tune.py writes one authoritative snapshot at the tuning root;
        # per-run output directories may be nested several levels below it.
        active_path = Path("/results/pid_tuning/active_profile.json")
        active = json.loads(active_path.read_text()) if active_path.exists() else None
        summary = {
            "name": args.name,
            "hover_policy": args.hover_policy,
            "stop_policy": args.stop_policy,
            "active_pid": active,
            "sample_count": len(measured),
            "hover": summarize_hover(phase_samples(measured, "hover")),
            "stops": {
                name: summarize_stop(phase_samples(measured, f"stop_{name}"))
                for name in directions if not args.hover_only
            },
        }
        stop_values = list(summary["stops"].values())
        if stop_values:
            incomplete = [name for name, value in summary["stops"].items() if not value]
            if incomplete:
                raise RuntimeError(f"No complete ground-truth samples for stop phases: {incomplete}")
            summary["aggregate"] = {
                "worst_stop_displacement_m": max(float(v["max_stop_displacement_m"]) for v in stop_values),
                "worst_z_rise_after_stop_m": max(float(v["z_rise_after_stop_m"]) for v in stop_values),
                "worst_z_drop_after_stop_m": min(float(v["z_drop_after_stop_m"]) for v in stop_values),
            }
        csv_path = args.output_dir / f"{args.name}.csv"
        summary_path = args.output_dir / f"{args.name}.json"
        write_outputs(csv_path, summary_path, measured, summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        print(f"Trace: {csv_path}\nSummary: {summary_path}")
    finally:
        node.request_land()
        land_deadline = time.monotonic() + 35.0
        while rclpy.ok() and node.state.armed and time.monotonic() < land_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.local_pose is not None and node.local_pose.pose.position.z <= 0.15:
                break
        node.request_sim_ground_disarm()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
