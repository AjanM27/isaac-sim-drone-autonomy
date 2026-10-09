#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime_dir="${root}/simulation/room_quadcopter/results/pid_tuning/parallel/runtime"
pid_file="${runtime_dir}/optimizer.pid"

if [[ -f "${pid_file}" ]]; then
  pid="$(<"${pid_file}")"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    command_line="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
    if [[ "${command_line}" == *"parallel_pid_optimizer.py"* ]]; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  fi
fi

for _ in $(seq 1 20); do
  pgrep -f '[p]arallel_pid_optimizer.py|[r]un_parallel_pid_batch.sh' >/dev/null || break
  sleep 0.25
done

scene_pids="$(pgrep -f '[p]arallel_tuning_scene.py' || true)"
if [[ -n "${scene_pids}" ]]; then
  kill -TERM ${scene_pids} 2>/dev/null || true
fi
bash "${root}/docker/flight_control/stop_parallel_stack.sh"
echo "Autonomous PID tuning processes stopped. Existing result files were preserved."
