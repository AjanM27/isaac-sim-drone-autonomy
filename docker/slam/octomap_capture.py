#!/usr/bin/env python3
"""Persist RTAB-Map's canonical OctoMap and lightweight diagnostics."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from octomap_msgs.msg import Octomap
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class OctomapCapture(Node):
    def __init__(self, output: Path) -> None:
        super().__init__("octomap_capture")
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.latest: Octomap | None = None
        self.occupied_points = 0
        self.frontier_points = 0
        self.messages = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Octomap, "/octomap_binary", self.on_map, qos)
        self.create_subscription(PointCloud2, "/octomap_occupied_space", self.on_occupied, qos)
        self.create_subscription(PointCloud2, "/octomap_global_frontier_space", self.on_frontier, qos)
        self.create_timer(10.0, self.save)

    def on_map(self, message: Octomap) -> None:
        self.latest = message
        self.messages += 1

    def on_occupied(self, message: PointCloud2) -> None:
        self.occupied_points = message.width * message.height

    def on_frontier(self, message: PointCloud2) -> None:
        self.frontier_points = message.width * message.height

    def save(self) -> None:
        if self.latest is None:
            return
        message = self.latest
        temporary = self.output / "octomap_binary.tmp.npz"
        np.savez_compressed(
            temporary,
            data=np.asarray(message.data, dtype=np.int8),
            tree_id=message.id,
            binary=message.binary,
            resolution=message.resolution,
            frame=message.header.frame_id,
            stamp_sec=message.header.stamp.sec,
            stamp_nanosec=message.header.stamp.nanosec,
            schema="ros_octomap_msgs_binary_v1",
        )
        temporary.replace(self.output / "octomap_binary.npz")
        status = {
            "schema": "ros_octomap_msgs_binary_v1",
            "messages": self.messages,
            "binary_payload_bytes": len(message.data),
            "resolution_m": message.resolution,
            "frame": message.header.frame_id,
            "occupied_points": self.occupied_points,
            "global_frontier_points": self.frontier_points,
            "updated_wall_time": time.time(),
        }
        (self.output / "octomap_status.json").write_text(json.dumps(status, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = OctomapCapture(args.output_dir)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
