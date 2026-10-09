#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime_dir="${root}/simulation/room_quadcopter/results/pid_tuning/parallel/runtime"
pid_file="${runtime_dir}/optimizer.pid"
active_file="${runtime_dir}/active_run"
results_dir="${root}/simulation/room_quadcopter/results/pid_tuning/parallel"

if [[ "${1:-}" == "--all" ]]; then
  exec python3 "${root}/docker/flight_control/pid_tuning_status.py" "${results_dir}" --all
fi

explicit_run=false
if [[ "${1:-}" == "--run" ]]; then
  explicit_run=true
  run_id="${2:?--run requires a run id}"
  shift 2
else
  [[ -f "${active_file}" ]] || { echo "No autonomous tuning run has been started."; exit 1; }
  run_id="$(<"${active_file}")"
fi
pid=""
if [[ "${explicit_run}" == false && -f "${pid_file}" ]]; then
  pid="$(<"${pid_file}")"
fi
run_dir="${results_dir}/${run_id}"
[[ -d "${run_dir}" ]] || { echo "PID tuning run not found: ${run_id}" >&2; exit 1; }
status_args=("${run_dir}")
if [[ "${pid}" =~ ^[0-9]+$ ]]; then
  status_args+=(--pid "${pid}")
fi
exec python3 "${root}/docker/flight_control/pid_tuning_status.py" "${status_args[@]}" "$@"
