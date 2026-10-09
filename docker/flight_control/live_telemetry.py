#!/usr/bin/env python3
"""Display and record live Isaac/MAVROS vehicle telemetry."""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


def euler_degrees(q) -> tuple[float, float, float]:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class LiveTelemetry(Node):
    def __init__(self) -> None:
        super().__init__("isaac_live_flight_telemetry")
        self.state = State()
        self.truth = None
        self.pose: PoseStamped | None = None
        self.velocity: TwistStamped | None = None
        self.create_subscription(State, "/drone_1/state", self._state, 10)
        self.create_subscription(PoseStamped, "/drone_1/local_position/pose", self._pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/drone_1/local_position/velocity_local", self._velocity, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self._tf, qos_profile_sensor_data)

    def _state(self, message: State) -> None:
        self.state = message

    def _pose(self, message: PoseStamped) -> None:
        self.pose = message

    def _velocity(self, message: TwistStamped) -> None:
        self.velocity = message

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id.lstrip("/") == "world" and transform.child_frame_id.lstrip("/") == "drone_base":
                self.truth = transform.transform


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=Path("/results/pid_tuning/live_telemetry.csv"))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = LiveTelemetry()
    fields = ["wall_time", "mode", "armed", "gt_x", "gt_y", "gt_z", "roll_deg", "pitch_deg", "yaw_deg", "mav_x", "mav_y", "mav_z", "vx", "vy", "vz", "speed_xy"]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        try:
            while rclpy.ok():
                deadline = time.monotonic() + 1.0 / max(args.rate, 0.2)
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                if node.truth is None or node.pose is None or node.velocity is None:
                    print("Waiting for Isaac ground truth and MAVROS pose/velocity...", flush=True)
                    continue
                gt = node.truth.translation
                roll, pitch, yaw = euler_degrees(node.truth.rotation)
                mav = node.pose.pose.position
                vel = node.velocity.twist.linear
                row = {
                    "wall_time": time.time(), "mode": node.state.mode, "armed": node.state.armed,
                    "gt_x": gt.x, "gt_y": gt.y, "gt_z": gt.z,
                    "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
                    "mav_x": mav.x, "mav_y": mav.y, "mav_z": mav.z,
                    "vx": vel.x, "vy": vel.y, "vz": vel.z,
                    "speed_xy": math.hypot(vel.x, vel.y),
                }
                writer.writerow(row)
                stream.flush()
                print(
                    f"{node.state.mode:>7} {'ARMED' if node.state.armed else 'SAFE ':>5} | "
                    f"GT xyz=({gt.x:+7.3f},{gt.y:+7.3f},{gt.z:+6.3f}) m | "
                    f"RPY=({roll:+6.2f},{pitch:+6.2f},{yaw:+7.2f}) deg | "
                    f"VEL=({vel.x:+6.3f},{vel.y:+6.3f},{vel.z:+6.3f}) m/s | "
                    f"MAV xyz=({mav.x:+7.3f},{mav.y:+7.3f},{mav.z:+6.3f})",
                    flush=True,
                )
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
