#!/usr/bin/env bash
set -euo pipefail

RVIZ_CONFIG="${1:?RViz configuration path is required}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
RVIZ_WAIT_SECONDS="${RVIZ_WAIT_SECONDS:-90}"

if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "RViz configuration not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 setup file not found: ${ROS_SETUP}; skipping RViz2" >&2
  exit 0
fi

# ROS setup files intentionally probe unset variables, so suspend nounset only
# while sourcing them and restore it immediately afterwards.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "RViz2 is not installed; Isaac Sim will continue without it" >&2
  exit 0
fi

# Avoid opening a second project-owned RViz window.
if pgrep -af "rviz2.*${RVIZ_CONFIG}" >/dev/null 2>&1; then
  echo "Project RViz2 window is already running"
  exit 0
fi

echo "Waiting for Isaac Sim ROS topics before opening RViz2..."
deadline=$((SECONDS + RVIZ_WAIT_SECONDS))
while (( SECONDS < deadline )); do
  topics="$(timeout 2s ros2 topic list 2>/dev/null || true)"
  if grep -qx '/clock' <<<"${topics}" && grep -qx '/tf' <<<"${topics}" && grep -qx '/lidar/points' <<<"${topics}"; then
    # DDS discovery can retain topic names briefly after the previous Isaac
    # process exits. Require a fresh timestamp near the start of the new run,
    # otherwise RViz caches the old high timestamp and floods TF_OLD_DATA until
    # the restarted simulation catches up.
    clock_sec="$(timeout 2s ros2 topic echo /clock --once 2>/dev/null | awk '/sec:/ {print $2; exit}' || true)"
    if [[ "${clock_sec}" =~ ^[0-9]+$ ]] && (( clock_sec <= 10 )); then
      echo "Opening RViz2 with the drone configuration"
      exec rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true
    fi
  fi
  sleep 1
done

echo "RViz2 was not opened: /clock, /tf, and /lidar/points were not available after ${RVIZ_WAIT_SECONDS}s" >&2
exit 0
