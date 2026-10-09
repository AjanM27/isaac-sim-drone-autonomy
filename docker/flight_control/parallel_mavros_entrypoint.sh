#!/usr/bin/env bash
set -euo pipefail

count="${1:?environment count is required}"
port_stride="${PID_PORT_STRIDE:-10}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p /results/logs
pids=()

terminate() {
  for pid in "${pids[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait || true
}
trap terminate EXIT INT TERM

for ((index=0; index<count; index++)); do
  mavlink_port=$((14550 + index * port_stride))
  ros2 launch mavros node.launch \
    fcu_url:="tcp://127.0.0.1:${mavlink_port}" \
    gcs_url:="udp://@" \
    pluginlists_yaml:="/workspace/docker/flight_control/tuning_mavros_plugins.yaml" \
    config_yaml:="/opt/ros/jazzy/share/mavros/launch/apm_config.yaml" \
    namespace:="tuning/env_${index}" \
    tgt_system:="$((index + 1))" \
    tgt_component:="1" \
    >"/results/logs/mavros_env_${index}.log" 2>&1 &
  pids+=("$!")
done

wait -n "${pids[@]}"
exit 1
