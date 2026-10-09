#!/usr/bin/env bash
set -euo pipefail

NS="${NS:-/drone_1}"
ALTITUDE="${ALTITUDE:-1.5}"
HOVER_VERIFY_SECONDS="${HOVER_VERIFY_SECONDS:-30}"
MAVROS_CONTAINER="${MAVROS_CONTAINER:-isaac-mavros}"

ros2_in_mavros() {
  # Each docker-exec CLI process otherwise probes the same Fast DDS shared-
  # memory port as the long-running MAVROS node.  Use UDP for these short-lived
  # clients; discovery and message semantics remain unchanged.
  docker exec -i "${MAVROS_CONTAINER}" bash -lc \
    "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; source /opt/ros/jazzy/setup.bash && $*"
}

echo "Waiting for MAVROS to report an ArduPilot connection..."
connected=1
for _ in $(seq 1 40); do
  # `--once` can return the retained pre-connection state immediately. Retry
  # it rather than using a pipe that can terminate on that stale message.
  state_message="$(ros2_in_mavros "timeout 3 ros2 topic echo --once ${NS}/state" 2>/dev/null || true)"
  if grep -q "connected: true" <<<"${state_message}"; then
    connected=0
    break
  fi
  sleep 3
done
if [ "${connected}" -ne 0 ]; then
  echo "MAVROS is not connected. Check docker/flight_control logs." >&2
  exit 1
fi

echo "Waiting for ArduPilot initialization to complete..."
# SITL has a configured home, but it does not necessarily broadcast a
# HOME_POSITION message at connection time. Request the standard MAVLink home
# message explicitly, then consume MAVROS's transient-local publication.
home_ready=1
for _ in $(seq 1 20); do
  ros2_in_mavros "ros2 service call ${NS}/cmd/command mavros_msgs/srv/CommandLong '{broadcast: false, command: 410, confirmation: 0, param1: 0.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}'" >/tmp/isaac_get_home.out
  if grep -Eq "success[=:][[:space:]]*(true|True)|result[=:][[:space:]]*0" /tmp/isaac_get_home.out; then
    home_ready=0
    break
  fi
  sleep 3
done
if [ "${home_ready}" -ne 0 ]; then
  echo "ArduPilot rejected MAV_CMD_GET_HOME_POSITION:" >&2
  cat /tmp/isaac_get_home.out >&2
  exit 1
fi
# SITL has a configured home at boot.  MAVROS may not replay its retained
# HOME_POSITION topic to a client that joins later, so the FCU's positive
# MAV_CMD_GET_HOME_POSITION acknowledgement is the readiness signal here.
sleep 3

echo "Setting GUIDED mode..."
ros2_in_mavros "ros2 service call ${NS}/set_mode mavros_msgs/srv/SetMode '{base_mode: 0, custom_mode: GUIDED}'" >/tmp/isaac_set_mode.out
if ! grep -Eqi "mode_sent[=:][[:space:]]*true|success[=:][[:space:]]*true" /tmp/isaac_set_mode.out; then
  echo "ArduPilot did not accept GUIDED mode:" >&2
  cat /tmp/isaac_set_mode.out >&2
  exit 1
fi
sleep 3

echo "Arming..."
for attempt in $(seq 1 30); do
  ros2_in_mavros "ros2 service call ${NS}/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'" >/tmp/isaac_arm.out
  if grep -q "success=True\|success: true" /tmp/isaac_arm.out; then
    break
  fi
  if [ "${attempt}" -eq 30 ]; then
    echo "ArduPilot refused arming. Full response:" >&2
    cat /tmp/isaac_arm.out >&2
    echo "Check /drone_1/statustext/recv for the PREARM reason." >&2
    exit 1
  fi
  sleep 2
done

echo "Taking off to ${ALTITUDE} m..."
ros2_in_mavros "ros2 service call ${NS}/cmd/takeoff mavros_msgs/srv/CommandTOL '{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${ALTITUDE}}'" >/tmp/isaac_takeoff.out
if ! grep -q "success=True\|success: true" /tmp/isaac_takeoff.out; then
  echo "ArduPilot did not accept takeoff:" >&2
  cat /tmp/isaac_takeoff.out >&2
  exit 1
fi

echo "Verifying sustained GUIDED hover for ${HOVER_VERIFY_SECONDS} seconds..."
sleep "${HOVER_VERIFY_SECONDS}"
post_takeoff_state="$(ros2_in_mavros "timeout 5 ros2 topic echo --once ${NS}/state" 2>/dev/null || true)"
if ! grep -q "armed: true" <<<"${post_takeoff_state}" || \
   ! grep -q "guided: true" <<<"${post_takeoff_state}" || \
   ! grep -q "mode: GUIDED" <<<"${post_takeoff_state}"; then
  echo "Vehicle did not remain armed in GUIDED mode after takeoff:" >&2
  printf '%s\n' "${post_takeoff_state}" >&2
  echo "Recent ArduPilot status messages:" >&2
  ros2_in_mavros "timeout 5 ros2 topic echo ${NS}/statustext/recv" 2>/dev/null | tail -n 80 >&2 || true
  exit 1
fi

echo "Flight-control test passed: connected, armed, takeoff accepted, and GUIDED hover sustained."
