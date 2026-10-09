#!/usr/bin/env python3
"""Measure Guided-mode stopping distance from Isaac ground truth."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


class BrakingMeasurement(Node):
    def __init__(self) -> None:
        super().__init__("isaac_braking_measurement")
        self.world_samples: list[tuple[float, float, float, float]] = []
        self.local_pose: PoseStamped | None = None
        self.create_subscription(TFMessage, "/tf", self.tf_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped,
            "/drone_1/local_position/pose",
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.velocity = self.create_publisher(
            TwistStamped, "/drone_1/setpoint_velocity/cmd_vel", 10
        )
        self.position = self.create_publisher(
            PositionTarget, "/drone_1/setpoint_raw/local", 10
        )
        self.set_mode = self.create_client(SetMode, "/drone_1/set_mode")

    def tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") == "world"
                and transform.child_frame_id.lstrip("/") == "drone_base"
            ):
                p = transform.transform.translation
                stamp = transform.header.stamp
                self.world_samples.append(
                    (stamp.sec + stamp.nanosec * 1.0e-9, p.x, p.y, p.z)
                )

    def pose_callback(self, message: PoseStamped) -> None:
        self.local_pose = message

    def publish_velocity(self, x: float, y: float, z: float = 0.0) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = x
        message.twist.linear.y = y
        message.twist.linear.z = z
        self.velocity.publish(message)

    def publish_position(self, target: PoseStamped) -> None:
        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = target.header.frame_id or "map"
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position = target.pose.position
        message.yaw = math.pi / 2.0
        self.position.publish(message)

    def request_mode(self, mode: str) -> None:
        if not self.set_mode.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MAVROS set_mode service is unavailable")
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or not future.result().mode_sent:
            raise RuntimeError(f"ArduPilot rejected {mode} mode")


def run_for(
    node: BrakingMeasurement,
    seconds: float,
    velocity: tuple[float, float, float] | None = None,
    position: PoseStamped | None = None,
) -> None:
    finish = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < finish:
        rclpy.spin_once(node, timeout_sec=0.01)
        if velocity is not None:
            node.publish_velocity(*velocity)
        if position is not None:
            node.publish_position(position)
        time.sleep(0.04)


def summarize(
    samples: list[tuple[float, float, float, float]],
    stop_index: int,
) -> dict[str, float | None]:
    stop = samples[stop_index]
    before = samples[max(0, stop_index - 30):stop_index + 1]
    dx = before[-1][1] - before[0][1]
    dy = before[-1][2] - before[0][2]
    magnitude = math.hypot(dx, dy)
    if magnitude < 1.0e-6:
        heading = (1.0, 0.0)
    else:
        heading = (dx / magnitude, dy / magnitude)

    post = samples[stop_index:]
    along = [
        (sample[1] - stop[1]) * heading[0] + (sample[2] - stop[2]) * heading[1]
        for sample in post
    ]
    radial = [math.hypot(sample[1] - stop[1], sample[2] - stop[2]) for sample in post]
    speeds: list[tuple[float, float]] = []
    for previous, current in zip(post, post[1:]):
        dt = current[0] - previous[0]
        if dt > 1.0e-4:
            speeds.append(
                (
                    current[0],
                    math.hypot(current[1] - previous[1], current[2] - previous[2]) / dt,
                )
            )
    settled_at = None
    for index, (stamp, speed) in enumerate(speeds):
        window = [value for when, value in speeds[index:] if when <= stamp + 0.5]
        if window and max(window) <= 0.05:
            settled_at = stamp - stop[0]
            break
    return {
        "initial_speed_mps": magnitude / max(1.0e-6, before[-1][0] - before[0][0]),
        "max_forward_distance_m": max(along),
        "max_radial_distance_m": max(radial),
        "final_distance_m": radial[-1],
        "settling_time_s": settled_at,
        "altitude_change_m": post[-1][3] - stop[3],
        "measurement_duration_s": post[-1][0] - stop[0],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--travel-seconds", type=float, default=3.0)
    parser.add_argument("--settle-seconds", type=float, default=8.0)
    parser.add_argument(
        "--stop-mode",
        choices=("velocity", "position"),
        default="velocity",
    )
    args = parser.parse_args()

    rclpy.init()
    node = BrakingMeasurement()
    try:
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.world_samples and node.local_pose is not None:
                break
        if not node.world_samples or node.local_pose is None:
            raise RuntimeError("No live Isaac transform or MAVROS local pose arrived.")

        node.request_mode("GUIDED")
        # A service call creates a short DDS delivery gap for this transient
        # test node. Re-establish subscriptions before starting the measured
        # acceleration window so low-render-rate runs are not discarded.
        run_for(node, 1.0, velocity=(0.0, 0.0, 0.0))
        node.world_samples.clear()
        run_for(node, args.travel_seconds, velocity=(args.speed, 0.0, 0.0))
        if len(node.world_samples) < 10:
            raise RuntimeError(
                f"Too few ground-truth samples during acceleration: {len(node.world_samples)}"
            )
        stop_index = len(node.world_samples) - 1

        # Capture the FCU pose at the instant stop is requested. Position mode
        # makes ArduPilot brake and return to that point; velocity mode is the
        # legacy zero-velocity behavior used for the baseline.
        hold_target = PoseStamped()
        hold_target.header.frame_id = node.local_pose.header.frame_id or "map"
        hold_target.pose = node.local_pose.pose
        node.publish_velocity(0.0, 0.0, 0.0)
        if args.stop_mode == "position":
            run_for(node, args.settle_seconds, position=hold_target)
        else:
            run_for(node, args.settle_seconds, velocity=(0.0, 0.0, 0.0))

        result = {"stop_mode": args.stop_mode, **summarize(node.world_samples, stop_index)}
        print(json.dumps(result, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
