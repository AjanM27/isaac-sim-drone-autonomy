#!/usr/bin/env python3
"""Republish a saved sparse voxel map for inspection in RViz."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from voxel_occupancy import FREE, OCCUPIED, UNKNOWN


class SavedVoxelMapPublisher(Node):
    def __init__(self, path: Path) -> None:
        super().__init__("saved_voxel_map_publisher")
        archive = np.load(path, allow_pickle=False)
        self.cells = archive["cells"].astype(np.float32, copy=False)
        self.log_odds = archive["log_odds"].astype(np.float32, copy=False)
        self.states = archive["states"].astype(np.int8, copy=False)
        self.resolution = float(archive["resolution"])
        self.frame = str(archive["frame"])
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publishers = {
            OCCUPIED: self.create_publisher(PointCloud2, "/exploration/occupied_voxels", qos),
            FREE: self.create_publisher(PointCloud2, "/exploration/free_voxels", qos),
            UNKNOWN: self.create_publisher(PointCloud2, "/exploration/uncertain_voxels", qos),
        }
        self.tf = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.publish)
        self.publish()
        self.get_logger().info(
            f"Loaded {len(self.cells)} evidence cells from {path}; frame={self.frame}, "
            f"resolution={self.resolution:.3f} m"
        )

    def cloud(self, mask: np.ndarray) -> PointCloud2:
        selected = self.cells[mask]
        data = np.empty((len(selected), 4), dtype=np.float32)
        if len(selected):
            data[:, :3] = (selected + 0.5) * self.resolution
            data[:, 3] = self.log_odds[mask]
        message = PointCloud2()
        message.header.frame_id = self.frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.height = 1
        message.width = len(selected)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = 16 * message.width
        message.is_dense = True
        message.data = data.tobytes()
        return message

    def publish(self) -> None:
        for state, publisher in self.map_publishers.items():
            publisher.publish(self.cloud(self.states == state))
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = self.frame
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("map", type=Path)
    args = parser.parse_args()
    if not args.map.is_file():
        parser.error(f"map does not exist: {args.map}")
    rclpy.init()
    node = SavedVoxelMapPublisher(args.map)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
