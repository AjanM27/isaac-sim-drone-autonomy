#!/usr/bin/env bash
set -euo pipefail

container="${SLAM_CONTAINER:-isaac-rtabmap-slam}"

run_ros() {
  docker exec "${container}" bash -lc "source /opt/ros/jazzy/setup.bash && source /slam_ws/install/setup.bash && $*"
}

echo "Checking RTAB-Map nodes and live 3D SLAM outputs..."
run_ros "ros2 node list" | grep -qx '/icp_odometry'
run_ros "ros2 node list" | grep -qx '/rtabmap'
run_ros "ros2 run isaac_slam slam_health_check"
