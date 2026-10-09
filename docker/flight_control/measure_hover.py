#!/usr/bin/env python3
"""Measure an ArduPilot hover against Isaac Sim ground-truth transforms."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny_cosp, cosy_cosp)


def unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        delta = value - result[-1]
        while delta > math.pi:
            value -= 2.0 * math.pi
            delta = value - result[-1]
        while delta < -math.pi:
            value += 2.0 * math.pi
            delta = value - result[-1]
        result.append(value)
    return result


def stats(values: list[float]) -> dict[str, float]:
    if not values:
        return {}
    return {
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "peak_to_peak": max(values) - min(values),
    }


def slope(times: list[float], values: list[float]) -> float:
    """Least-squares slope in value units per simulation second."""
    mean_t = statistics.fmean(times)
    mean_v = statistics.fmean(values)
    denominator = sum((value - mean_t) ** 2 for value in times)
    if denominator <= 0.0:
        return 0.0
    return sum((t - mean_t) * (v - mean_v) for t, v in zip(times, values)) / denominator


class HoverMeasurement(Node):
    def __init__(self) -> None:
        super().__init__("isaac_hover_measurement")
        self.ground_z: float | None = None
        self.latest_estimator_pose: tuple[float, ...] | None = None
        self.latest_estimator_velocity: tuple[float, ...] | None = None
        self.samples: list[dict[str, float]] = []
        self.create_subscription(TFMessage, "/tf", self.tf_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped,
            "/drone_1/local_position/pose",
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            "/drone_1/local_position/velocity_local",
            self.velocity_callback,
            qos_profile_sensor_data,
        )
        self.velocity_setpoint = self.create_publisher(
            TwistStamped, "/drone_1/setpoint_velocity/cmd_vel", 10
        )
        self.position_setpoint = self.create_publisher(
            PositionTarget, "/drone_1/setpoint_raw/local", 10
        )
        self.hold_target: PoseStamped | None = None
        self.collecting = False

    def pose_callback(self, message: PoseStamped) -> None:
        p = message.pose.position
        q = message.pose.orientation
        self.latest_estimator_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def velocity_callback(self, message: TwistStamped) -> None:
        v = message.twist.linear
        self.latest_estimator_velocity = (v.x, v.y, v.z)

    def activate_position_hold(self) -> None:
        """Capture the current MAVROS ENU pose as a fixed Guided target."""
        if self.latest_estimator_pose is None or self.hold_target is not None:
            return
        p = self.latest_estimator_pose
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = p[0]
        target.pose.position.y = p[1]
        target.pose.position.z = p[2]
        target.pose.orientation.x = p[3]
        target.pose.orientation.y = p[4]
        target.pose.orientation.z = p[5]
        target.pose.orientation.w = p[6]
        self.hold_target = target

    def publish_hold(self, policy: str) -> None:
        """Replay one teleop idle policy without changing FCU PID gains."""
        if policy == "none":
            return
        if policy == "velocity":
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "drone_base"
            self.velocity_setpoint.publish(message)
            return
        self.activate_position_hold()
        if self.hold_target is not None:
            message = PositionTarget()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.hold_target.header.frame_id
            message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            message.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
                | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                | PositionTarget.IGNORE_YAW_RATE
            )
            message.position = self.hold_target.pose.position
            message.yaw = math.pi / 2.0
            self.position_setpoint.publish(message)

    def tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id.lstrip("/") != "world" or transform.child_frame_id.lstrip("/") != "drone_base":
                continue
            p = transform.transform.translation
            q = transform.transform.rotation
            if self.ground_z is None:
                self.ground_z = p.z
            if not self.collecting:
                return
            roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            sample = {
                "time": transform.header.stamp.sec + transform.header.stamp.nanosec * 1.0e-9,
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
            if self.latest_estimator_pose is not None:
                ep = self.latest_estimator_pose
                sample.update({"est_x": ep[0], "est_y": ep[1], "est_z": ep[2]})
            if self.latest_estimator_velocity is not None:
                ev = self.latest_estimator_velocity
                sample.update({"est_vx": ev[0], "est_vy": ev[1], "est_vz": ev[2]})
            self.samples.append(sample)


def summarize(samples: list[dict[str, float]], ground_z: float, target_height: float) -> dict:
    target_z = ground_z + target_height
    times = [s["time"] for s in samples]
    xs = [s["x"] for s in samples]
    ys = [s["y"] for s in samples]
    zs = [s["z"] for s in samples]
    rolls = [math.degrees(s["roll"]) for s in samples]
    pitches = [math.degrees(s["pitch"]) for s in samples]
    yaws = [math.degrees(v) for v in unwrap([s["yaw"] for s in samples])]
    velocities = []
    for previous, current in zip(samples, samples[1:]):
        dt = current["time"] - previous["time"]
        if dt > 1.0e-4:
            velocities.append(
                (
                    (current["x"] - previous["x"]) / dt,
                    (current["y"] - previous["y"]) / dt,
                    (current["z"] - previous["z"]) / dt,
                )
            )
    radial = [math.hypot(x - xs[0], y - ys[0]) for x, y in zip(xs, ys)]
    z_errors = [z - target_z for z in zs]
    midpoint = len(samples) // 2
    summary = {
        "sample_count": len(samples),
        "duration_seconds": times[-1] - times[0],
        "ground_truth": {
            "ground_z": ground_z,
            "target_z": target_z,
            "x": stats(xs),
            "y": stats(ys),
            "z": stats(zs),
            "z_error": {
                **stats(z_errors),
                "rmse": math.sqrt(statistics.fmean(e * e for e in z_errors)),
                "trend_per_second": slope(times, z_errors),
                "first_half": stats(z_errors[:midpoint]),
                "second_half": stats(z_errors[midpoint:]),
            },
            "radial_drift": {**stats(radial), "final": radial[-1]},
            "roll_deg": stats(rolls),
            "pitch_deg": stats(pitches),
            "yaw_deg_unwrapped": {**stats(yaws), "change": yaws[-1] - yaws[0]},
        },
    }
    if velocities:
        summary["ground_truth"]["derived_velocity"] = {
            "vx": stats([v[0] for v in velocities]),
            "vy": stats([v[1] for v in velocities]),
            "vz": stats([v[2] for v in velocities]),
        }
    if all("est_x" in s for s in samples):
        summary["ardupilot_estimator"] = {
            "x": stats([s["est_x"] for s in samples]),
            "y": stats([s["est_y"] for s in samples]),
            "z": stats([s["est_z"] for s in samples]),
            # MAVROS reports ENU (X east, Y north), while Isaac's world used
            # by this bridge is X north, Y west. Compare equivalent axes.
            "enu_minus_converted_ground_truth": {
                "x": stats([s["est_x"] - (-s["y"]) for s in samples]),
                "y": stats([s["est_y"] - s["x"] for s in samples]),
                # MAVROS local Z starts at zero at the launch pose.
                "z": stats([s["est_z"] - (s["z"] - ground_z) for s in samples]),
            },
        }
    return summary


def write_trace(path: str, samples: list[dict[str, float]]) -> None:
    """Write every collected ground-truth/estimator sample for later comparison."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    columns = (
        "time", "x", "y", "z", "roll", "pitch", "yaw",
        "est_x", "est_y", "est_z", "est_vx", "est_vy", "est_vz",
    )
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(samples)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-height", type=float, default=1.5)
    parser.add_argument(
        "--ground-z",
        type=float,
        default=None,
        help="Known Isaac ground Z; allows measuring an already-airborne vehicle",
    )
    parser.add_argument("--warmup", type=float, default=12.0)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--timeout", type=float, default=150.0)
    parser.add_argument(
        "--hold-policy",
        choices=("none", "velocity", "position"),
        default="none",
        help="Replay the takeoff target, previous zero velocity, or position hold",
    )
    parser.add_argument("--output-csv", default=None)
    args = parser.parse_args()

    rclpy.init()
    node = HoverMeasurement()
    if args.ground_z is not None:
        node.ground_z = args.ground_z
    deadline = time.monotonic() + args.timeout
    try:
        print("Waiting for Isaac ground truth and a 1.5 m hover...", flush=True)
        airborne_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.publish_hold(args.hold_policy)
            if node.ground_z is None:
                airborne_since = None
                continue
            # Temporarily collect one live transform to detect climb without
            # relying on the estimator.
            node.collecting = True
            rclpy.spin_once(node, timeout_sec=0.05)
            node.publish_hold(args.hold_policy)
            node.collecting = False
            if not node.samples:
                continue
            latest_z = node.samples[-1]["z"]
            node.samples.clear()
            if latest_z >= node.ground_z + 0.75 * args.target_height:
                airborne_since = airborne_since or time.monotonic()
                if time.monotonic() - airborne_since >= args.warmup:
                    break
            else:
                airborne_since = None
        else:
            raise RuntimeError("Timed out waiting for a stable hover.")

        node.samples.clear()
        node.collecting = True
        finish = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < finish:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.publish_hold(args.hold_policy)
        node.collecting = False
        if len(node.samples) < 20:
            raise RuntimeError(f"Only received {len(node.samples)} Isaac ground-truth samples.")
        if args.output_csv:
            write_trace(args.output_csv, node.samples)
        result = {
            "hold_policy": args.hold_policy,
            "trace_csv": args.output_csv,
            **summarize(node.samples, node.ground_z, args.target_height),
        }
        print(json.dumps(result, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
