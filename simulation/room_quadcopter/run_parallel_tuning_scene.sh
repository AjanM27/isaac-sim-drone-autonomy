#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
# Copter's 400 Hz controller enforces an approximately 1.8x gyro-rate margin.
export ARDUPILOT_JSON_SENSOR_RATE_HZ="${ARDUPILOT_JSON_SENSOR_RATE_HZ:-800}"
export ISAAC_DISABLE_ROTOR_VISUALS="${ISAAC_DISABLE_ROTOR_VISUALS:-1}"

exec "${root}/scripts/isaac_python.sh" \
  "${root}/simulation/room_quadcopter/scripts/parallel_tuning_scene.py" "$@"
