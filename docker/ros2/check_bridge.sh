#!/usr/bin/env bash
set -euo pipefail

CONTAINER="isaac-sim-ros2"

if ! docker inspect "${CONTAINER}" >/dev/null 2>&1; then
  echo "ROS 2 container '${CONTAINER}' is not running. Start it with docker/ros2/start.sh." >&2
  exit 1
fi

echo "Waiting for one Isaac Sim /clock message (make sure the timeline is advancing)..."
set +e
timeout --foreground 20s docker exec -i "${CONTAINER}" bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /clock --once'
status=$?
set -e
if [[ "${status}" -eq 124 ]]; then
  echo "ROS 2 sees /clock, but no messages arrived." >&2
  echo "Make sure Isaac Sim has been restarted with the updated launcher and its timeline is playing." >&2
  exit 124
fi
if [[ "${status}" -ne 0 ]]; then
  echo "Could not read /clock. Check that the ROS 2 container is running." >&2
  exit "${status}"
fi
echo "ROS 2 bridge check passed."
