#!/usr/bin/env bash
set -euo pipefail

count="${1:?environment count is required}"
port_stride="${PID_PORT_STRIDE:-10}"
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
  work="/tmp/parallel_sitl_${index}"
  mkdir -p "${work}/logs"
  json_port=$((9002 + index * port_stride))
  mavlink_port=$((14550 + index * port_stride))
  (
    cd "${work}"
    exec /ardupilot/build/sitl/bin/arducopter \
      --instance "${index}" \
      --model JSON:127.0.0.1 \
      --sim-port-in "${json_port}" \
      --home=-35.363262,149.165237,584,0 \
      --serial0="tcp:${mavlink_port}" \
      --sysid "$((index + 1))" \
      --defaults "/profiles/env_${index}.parm" \
      --wipe \
      --speedup 1
  ) >"/results/logs/sitl_env_${index}.log" 2>&1 &
  pids+=("$!")
done

wait -n "${pids[@]}"
exit 1
