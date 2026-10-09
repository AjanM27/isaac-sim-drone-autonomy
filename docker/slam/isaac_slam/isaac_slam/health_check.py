"""Validate that the lidar-only RTAB-Map pipeline is producing a coherent map."""

import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


class SlamHealthCheck(Node):
    def __init__(self) -> None:
        super().__init__("slam_health_check")
        self.cloud_width = 0
        self.path_poses = 0
        self.odom_frames = None
        self.tf_edges = set()

        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(PointCloud2, "/cloud_map", self._cloud, qos)
        self.create_subscription(Odometry, "/slam/odom", self._odom, qos)
        self.create_subscription(Path, "/mapPath", self._path, 10)
        self.create_subscription(TFMessage, "/tf", self._tf, qos)

    def _cloud(self, message: PointCloud2) -> None:
        self.cloud_width = max(self.cloud_width, message.width)

    def _odom(self, message: Odometry) -> None:
        self.odom_frames = (message.header.frame_id, message.child_frame_id)

    def _path(self, message: Path) -> None:
        self.path_poses = max(self.path_poses, len(message.poses))

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.tf_edges.add((transform.header.frame_id, transform.child_frame_id))

    def ready(self) -> bool:
        required_edges = {("map", "icp_odom"), ("icp_odom", "slam_base")}
        return (
            self.cloud_width > 0
            and self.path_poses > 0
            and self.odom_frames == ("icp_odom", "slam_base")
            and required_edges.issubset(self.tf_edges)
        )


def main() -> None:
    rclpy.init()
    node = SlamHealthCheck()
    deadline = time.monotonic() + 35.0

    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.25)

        print(f"cloud_map_points={node.cloud_width}")
        print(f"map_path_poses={node.path_poses}")
        print(f"odometry_frames={node.odom_frames}")
        print(f"tf_edges={sorted(node.tf_edges)}")
        if not node.ready():
            if node.cloud_width == 0 and node.path_poses <= 2 and node.odom_frames:
                raise SystemExit(
                    "SLAM pipeline is live, but the accumulated map is still empty. "
                    "Take off and move the drone far enough to create a keyframe, then retry."
                )
            raise SystemExit("3D SLAM health check timed out before all outputs became live")
        print("3D SLAM healthy: map, path, ICP odometry, and TF chain are live.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
