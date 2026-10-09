#!/usr/bin/env python3
"""Record RTAB-Map diagnostics in the container that owns its message types."""

from __future__ import annotations

import argparse
import csv
import math
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import PointCloud2


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class DiagnosticsRecorder(Node):
    def __init__(self) -> None:
        super().__init__("checkpoint1_slam_diagnostics")
        self.rows: list[dict[str, float | int]] = []
        self.scan_points = 0
        self.map_points = 0
        self.last_map: PointCloud2 | None = None
        self.create_subscription(PointCloud2, "/slam/lidar/points", self._scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/cloud_map", self._map, qos_profile_sensor_data)
        self.create_subscription(OdomInfo, "/slam/odom_info", self._info, qos_profile_sensor_data)

    def _scan(self, message: PointCloud2) -> None:
        self.scan_points = int(message.width * message.height)

    def _map(self, message: PointCloud2) -> None:
        self.map_points = int(message.width * message.height)
        self.last_map = message

    def _info(self, message: OdomInfo) -> None:
        self.rows.append(
            {
                "time": stamp_seconds(message.header.stamp),
                "lost": int(message.lost),
                "inlier_ratio": float(message.icp_inliers_ratio),
                "correspondences": int(message.icp_correspondences),
                "processing_time_s": float(message.time_estimation),
                "local_scan_map_size": int(message.local_scan_map_size),
                "key_frame_added": int(message.key_frame_added),
                "scan_points": self.scan_points,
                "map_points": self.map_points,
            }
        )

    def map_voxel_ratio(self, voxel_size: float) -> float | None:
        cloud = self.last_map
        if cloud is None or cloud.width * cloud.height == 0:
            return None
        offsets = {field.name: field.offset for field in cloud.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            return None
        prefix = ">" if cloud.is_bigendian else "<"
        voxels = set()
        finite_points = 0
        for index in range(cloud.width * cloud.height):
            base = index * cloud.point_step
            xyz = tuple(
                struct.unpack_from(prefix + "f", cloud.data, base + offsets[axis])[0]
                for axis in ("x", "y", "z")
            )
            if all(math.isfinite(v) for v in xyz):
                finite_points += 1
                voxels.add(tuple(math.floor(v / voxel_size) for v in xyz))
        return len(voxels) / finite_points if finite_points else None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--voxel-size", type=float, default=0.15)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.stop_file.unlink(missing_ok=True)

    rclpy.init()
    node = DiagnosticsRecorder()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not args.stop_file.exists():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node.rows:
            with args.output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(node.rows[0]))
                writer.writeheader()
                writer.writerows(node.rows)
        ratio = node.map_voxel_ratio(args.voxel_size)
        args.output.with_suffix(".summary").write_text(
            f"updates={len(node.rows)}\n"
            f"map_points={node.map_points}\n"
            f"voxel_unique_ratio={ratio if ratio is not None else 'nan'}\n"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
