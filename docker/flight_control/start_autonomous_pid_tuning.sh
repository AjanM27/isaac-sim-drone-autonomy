#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime_dir="${root}/simulation/room_quadcopter/results/pid_tuning/parallel/runtime"
pid_file="${runtime_dir}/optimizer.pid"
mkdir -p "${runtime_dir}"

required_files=(
  "${root}/scripts/isaac_python.sh"
  "${root}/simulation/room_quadcopter/assets/iris/iris.usd"
  "${root}/docker/flight_control/isaac_sim_tuned.parm"
  "${root}/docker/flight_control/pid_search_space.json"
)
for required in "${required_files[@]}"; do
  [[ -e "${required}" ]] || { echo "Offline preflight failed: missing ${required}" >&2; exit 1; }
done
command -v docker >/dev/null || { echo "Offline preflight failed: docker is unavailable" >&2; exit 1; }
docker info >/dev/null 2>&1 || { echo "Offline preflight failed: Docker daemon is unavailable" >&2; exit 1; }
for image in isaac-flight-control-ardupilot-sitl:local isaac-flight-control-mavros:local; do
  docker image inspect "${image}" >/dev/null 2>&1 || {
    echo "Offline preflight failed: local image ${image} is missing" >&2
    echo "Build the images once while dependencies are available, then retry." >&2
    exit 1
  }
done
non_performance_cpus=()
for governor_file in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
  [[ -r "${governor_file}" ]] || continue
  if [[ "$(<"${governor_file}")" != "performance" ]]; then
    non_performance_cpus+=("${governor_file}")
  fi
done
if ((${#non_performance_cpus[@]})); then
  echo "Offline preflight failed: multi-drone tuning requires CPU performance mode." >&2
  echo "Enable the performance governor, then retry." >&2
  exit 1
fi

if [[ -f "${pid_file}" ]]; then
  old_pid="$(<"${pid_file}")"
  if [[ "${old_pid}" =~ ^[0-9]+$ ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "Autonomous PID tuning is already running as PID ${old_pid}."
    exit 1
  fi
fi
if pgrep -f '[p]arallel_pid_optimizer.py|[p]arallel_tuning_scene.py|[r]un_parallel_pid_batch.sh' >/dev/null; then
  echo "Offline preflight failed: another tuning process or cleanup is still active." >&2
  echo "Run docker/flight_control/stop_autonomous_pid_tuning.sh, then retry." >&2
  exit 1
fi

run_id="${1:-autonomous_empty_$(date +%Y%m%d_%H%M%S)}"
shift || true
log_file="${runtime_dir}/${run_id}.log"

cd "${root}"
setsid bash docker/flight_control/run_parallel_pid_optimizer.sh \
  --run-id "${run_id}" \
  --batch-size "${PID_TUNING_BATCH_SIZE:-10}" \
  --replicas 2 \
  --generations-per-stage "${PID_TUNING_GENERATIONS_PER_STAGE:-12}" \
  --stages attitude_rate attitude_angle vertical horizontal_velocity horizontal_position yaw braking \
  --speed 1.0 \
  --high-speed 3.0 \
  --warmup-seconds 6 \
  --hover-seconds 15 \
  --max-runtime "${PID_TUNING_MAX_RUNTIME:-8h}" \
  --promote \
  "$@" >"${log_file}" 2>&1 < /dev/null &
pid=$!
printf '%s\n' "${pid}" >"${pid_file}"
printf '%s\n' "${run_id}" >"${runtime_dir}/active_run"
echo "Started autonomous PID tuning:"
echo "  run: ${run_id}"
echo "  pid: ${pid}"
echo "  log: ${log_file}"
echo "Live status:"
echo "  bash docker/flight_control/status_autonomous_pid_tuning.sh --watch"
