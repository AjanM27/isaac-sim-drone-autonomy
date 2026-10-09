#!/usr/bin/env python3
"""Fly and record the Checkpoint-1 square trajectory at a requested speed."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from teleop import Teleop


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_degrees(q) -> float:
    return math.degrees(
        math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
    )


class BenchmarkRecorder(Teleop):
    def __init__(self) -> None:
        super().__init__()
        self.phase = "startup_takeoff"
        self.desired_position = (math.nan, math.nan, math.nan)
        self.desired_velocity = (0.0, 0.0, 0.0)
        self.desired_yaw = math.nan
        self.slam_odom: Odometry | None = None
        self.scan_points = 0
        self.map_points = 0
        self.path_poses = 0
        self.last_map: PointCloud2 | None = None
        self.samples: list[dict[str, float | int | str]] = []
        self.create_subscription(Odometry, "/slam/odom", self._slam_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/slam/lidar/points", self._scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/cloud_map", self._map, qos_profile_sensor_data)
        self.create_subscription(RosPath, "/mapPath", self._path, 10)
        self.create_subscription(TFMessage, "/tf", self._tf, qos_profile_sensor_data)

    def _slam_odom(self, message: Odometry) -> None:
        self.slam_odom = message

    def _scan(self, message: PointCloud2) -> None:
        self.scan_points = int(message.width * message.height)

    def _map(self, message: PointCloud2) -> None:
        self.map_points = int(message.width * message.height)
        self.last_map = message

    def _path(self, message: RosPath) -> None:
        self.path_poses = len(message.poses)

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") != "world"
                or transform.child_frame_id.lstrip("/") != "drone_base"
            ):
                continue
            gt = transform.transform
            mav = self.local_pose.pose if self.local_pose is not None else None
            slam = self.slam_odom.pose.pose if self.slam_odom is not None else None
            self.samples.append(
                {
                    "time": stamp_seconds(transform.header.stamp),
                    "phase": self.phase,
                    "desired_mav_x": self.desired_position[0],
                    "desired_mav_y": self.desired_position[1],
                    "desired_mav_z": self.desired_position[2],
                    "desired_vx": self.desired_velocity[0],
                    "desired_vy": self.desired_velocity[1],
                    "desired_vz": self.desired_velocity[2],
                    "desired_yaw_deg": math.degrees(self.desired_yaw),
                    "gt_x": gt.translation.x,
                    "gt_y": gt.translation.y,
                    "gt_z": gt.translation.z,
                    "gt_yaw_deg": yaw_degrees(gt.rotation),
                    "mav_x": mav.position.x if mav else math.nan,
                    "mav_y": mav.position.y if mav else math.nan,
                    "mav_z": mav.position.z if mav else math.nan,
                    "mav_yaw_deg": yaw_degrees(mav.orientation) if mav else math.nan,
                    "slam_x": slam.position.x if slam else math.nan,
                    "slam_y": slam.position.y if slam else math.nan,
                    "slam_z": slam.position.z if slam else math.nan,
                    "slam_yaw_deg": yaw_degrees(slam.orientation) if slam else math.nan,
                    "slam_stamp": stamp_seconds(self.slam_odom.header.stamp) if self.slam_odom else math.nan,
                    "scan_points": self.scan_points,
                    "map_points": self.map_points,
                    "map_path_poses": self.path_poses,
                }
            )

    def publish_target(
        self,
        phase: str,
        position: np.ndarray,
        velocity: np.ndarray,
        yaw: float,
    ) -> None:
        self.phase = phase
        self.desired_position = tuple(float(v) for v in position)
        self.desired_velocity = tuple(float(v) for v in velocity)
        self.desired_yaw = yaw
        self.send_position_velocity(self.desired_position, self.desired_velocity, yaw)

    def hold(self, phase: str, position: np.ndarray, yaw: float, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.publish_target(phase, position, np.zeros(3), yaw)
            time.sleep(0.04)

    def segment(
        self,
        index: int,
        start: np.ndarray,
        delta: np.ndarray,
        speed: float,
        yaw: float,
    ) -> np.ndarray:
        length = float(np.linalg.norm(delta[:2]))
        unit = delta / length
        started = time.monotonic()
        duration = length / speed
        while rclpy.ok():
            elapsed = min(time.monotonic() - started, duration)
            position = start + unit * speed * elapsed
            velocity = unit * speed if elapsed < duration else np.zeros(3)
            rclpy.spin_once(self, timeout_sec=0.01)
            self.publish_target(f"segment_{index}", position, velocity, yaw)
            if elapsed >= duration:
                break
            time.sleep(0.04)
        end = start + delta
        self.hold(f"corner_{index}", end, yaw, 1.5)
        return end

    def final_map_voxel_ratio(self, voxel_size: float) -> float | None:
        cloud = self.last_map
        if cloud is None or cloud.width * cloud.height == 0:
            return None
        offsets = {field.name: field.offset for field in cloud.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            return None
        prefix = ">" if cloud.is_bigendian else "<"
        points = []
        for index in range(cloud.width * cloud.height):
            base = index * cloud.point_step
            xyz = tuple(
                struct.unpack_from(prefix + "f", cloud.data, base + offsets[axis])[0]
                for axis in ("x", "y", "z")
            )
            if all(math.isfinite(v) for v in xyz):
                points.append(xyz)
        if not points:
            return None
        voxels = np.floor(np.asarray(points) / voxel_size).astype(np.int64)
        return float(len(np.unique(voxels, axis=0)) / len(points))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, required=True, choices=(0.25, 0.5, 1.0))
    parser.add_argument("--side-length", type=float, default=2.0)
    parser.add_argument("--name")
    parser.add_argument("--output-dir", type=Path, default=Path("/results/checkpoint1"))
    parser.add_argument("--voxel-size", type=float, default=0.15)
    args = parser.parse_args()
    name = args.name or f"square_{str(args.speed).replace('.', 'p')}_mps"

    rclpy.init()
    node = BenchmarkRecorder()
    failure = None
    try:
        node.prepare_guided()
        if node.local_pose is None:
            raise RuntimeError("MAVROS local pose did not become available")
        origin = np.array(
            [
                node.local_pose.pose.position.x,
                node.local_pose.pose.position.y,
                node.local_pose.pose.position.z,
            ],
            dtype=float,
        )
        # MAVROS local coordinates corresponding to an Isaac-world CCW square:
        # world +X, +Y, -X, -Y. Yaw values are NED as required by PositionTarget.
        route = (
            (np.array([0.0, args.side_length, 0.0]), math.pi / 2.0),
            (np.array([-args.side_length, 0.0, 0.0]), math.pi),
            (np.array([0.0, -args.side_length, 0.0]), -math.pi / 2.0),
            (np.array([args.side_length, 0.0, 0.0]), 0.0),
        )
        node.hold("initial_hover", origin, route[0][1], 4.0)
        position = origin.copy()
        for index, (delta, yaw) in enumerate(route, start=1):
            node.hold(f"turn_{index}", position, yaw, 2.0)
            position = node.segment(index, position, delta, args.speed, yaw)
        node.hold("return_hold", origin, route[0][1], 5.0)
    except Exception as error:  # preserve partial evidence before failing
        failure = f"{type(error).__name__}: {error}"
        raise
    finally:
        # Landing is part of the benchmark trajectory and must be captured,
        # even when an earlier phase raises and only partial evidence exists.
        node.phase = "landing"
        node.desired_velocity = (0.0, 0.0, 0.0)
        node.request_land()
        deadline = time.monotonic() + 35.0
        while rclpy.ok() and node.state.armed and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.request_sim_ground_disarm()
        output_dir = args.output_dir / name
        output_dir.mkdir(parents=True, exist_ok=True)
        if node.samples:
            with (output_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(node.samples[0]))
                writer.writeheader()
                writer.writerows(node.samples)
        metadata = {
            "name": name,
            "speed_mps": args.speed,
            "side_length_m": args.side_length,
            "sample_count": len(node.samples),
            "failure": failure,
            "final_map_voxel_unique_ratio": node.final_map_voxel_ratio(args.voxel_size),
            "final_map_points": node.map_points,
            "final_path_poses": node.path_poses,
            "deskewing": False,
        }
        (output_dir / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"Raw benchmark output: {output_dir}")


if __name__ == "__main__":
    main()
