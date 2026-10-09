#!/usr/bin/env bash
set -euo pipefail

count="${1:?environment count is required}"
profiles="$(realpath "${2:?profile directory is required}")"
output="$(realpath -m "${3:?output directory is required}")"
protocol="${4:-hover}"
speed="${5:-1.0}"
high_speed="${6:-3.0}"
warmup_seconds="${7:-12.0}"
hover_seconds="${8:-35.0}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
scene_log="${output}/logs/isaac_parallel_scene.log"
scene_pid=""
scene_pgid=""
record_pid=""

stop_recording() {
  if [[ -n "${record_pid}" ]] && kill -0 "${record_pid}" 2>/dev/null; then
    kill -INT "${record_pid}" 2>/dev/null || true
    wait "${record_pid}" 2>/dev/null || true
  fi
  record_pid=""
}

cleanup() {
  stop_recording
  if [[ -n "${scene_pgid}" ]]; then
    kill -TERM -- "-${scene_pgid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      pgrep -g "${scene_pgid}" >/dev/null || break
      sleep 0.25
    done
    if pgrep -g "${scene_pgid}" >/dev/null; then
      kill -KILL -- "-${scene_pgid}" 2>/dev/null || true
    fi
  elif [[ -n "${scene_pid}" ]]; then
    kill "${scene_pid}" 2>/dev/null || true
  fi
  bash "${root}/docker/flight_control/stop_parallel_stack.sh"
}
trap cleanup EXIT INT TERM

mkdir -p "${output}/logs"
# A killed parent shell must not leave an old simulator owning the candidate
# ports or appending to this batch's log.
stale_pids="$(pgrep -f '[p]arallel_tuning_scene.py' || true)"
if [[ -n "${stale_pids}" ]]; then
  kill -TERM ${stale_pids} 2>/dev/null || true
  for _ in $(seq 1 20); do
    pgrep -f '[p]arallel_tuning_scene.py' >/dev/null || break
    sleep 0.25
  done
  stale_pids="$(pgrep -f '[p]arallel_tuning_scene.py' || true)"
  if [[ -n "${stale_pids}" ]]; then
    kill -KILL ${stale_pids} 2>/dev/null || true
  fi
fi
bash "${root}/docker/flight_control/start_parallel_stack.sh" \
  "${count}" "${profiles}" "${output}"

cd "${root}"
headless_flag="--headless"
if [[ "${PARALLEL_SCENE_HEADLESS:-1}" == "0" ]]; then
  headless_flag="--no-headless"
fi
setsid env ISAAC_AUTO_RVIZ=0 FLIGHT_CONTROL_RGB_CAPTURE=0 \
  ISAAC_VIEWPORT_WIDTH="${PARALLEL_VIEWPORT_WIDTH:-960}" \
  ISAAC_VIEWPORT_HEIGHT="${PARALLEL_VIEWPORT_HEIGHT:-540}" \
  ISAAC_DLSS_MODE="${PARALLEL_DLSS_MODE:-0}" \
  ISAAC_RTX_SHADOWS="${PARALLEL_RTX_SHADOWS:-0}" \
  simulation/room_quadcopter/run_parallel_tuning_scene.sh \
  --env-count "${count}" "${headless_flag}" --run-seconds 900 \
  --environment "${PARALLEL_SCENE_ENVIRONMENT:-empty}" \
  --spacing "${PARALLEL_SCENE_SPACING:-30.0}" \
  --physics-rate "${PARALLEL_PHYSICS_RATE:-100.0}" \
  >"${scene_log}" 2>&1 &
scene_pid=$!
scene_pgid="$(ps -o pgid= -p "${scene_pid}" | tr -d ' ')"

for _ in $(seq 1 120); do
  if grep -q "Parallel shared-world ready" "${scene_log}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${scene_pid}" 2>/dev/null; then
    echo "Parallel Isaac scene exited during startup" >&2
    tail -120 "${scene_log}" >&2
    exit 1
  fi
  sleep 1
done
grep -q "Parallel shared-world ready" "${scene_log}" || {
  echo "Parallel Isaac scene did not become ready" >&2
  tail -120 "${scene_log}" >&2
  exit 1
}

if [[ -n "${PARALLEL_RECORD_OUTPUT:-}" ]]; then
  command -v ffmpeg >/dev/null || {
    echo "ffmpeg is required to record the fleet demo" >&2
    exit 1
  }
  resolution="$(xdpyinfo -display "${DISPLAY:-:0}" 2>/dev/null | awk '/dimensions:/{print $2; exit}')"
  [[ -n "${resolution}" ]] || {
    echo "Could not determine X11 display resolution for recording" >&2
    exit 1
  }
  mkdir -p "$(dirname "${PARALLEL_RECORD_OUTPUT}")"
  ffmpeg -nostdin -hide_banner -loglevel warning -stats -y \
    -f x11grab -draw_mouse 0 -framerate "${PARALLEL_RECORD_FPS:-24}" -video_size "${resolution}" \
    -i "${DISPLAY:-:0}+0,0" \
    -c:v h264_nvenc -preset p5 -tune hq -rc vbr -cq 20 -b:v 0 \
    -pix_fmt yuv420p -movflags +faststart "${PARALLEL_RECORD_OUTPUT}" \
    >"${output}/logs/recording.log" 2>&1 &
  record_pid=$!
  sleep 2
  kill -0 "${record_pid}" 2>/dev/null || {
    echo "Fleet recording failed to start" >&2
    cat "${output}/logs/recording.log" >&2
    exit 1
  }
  echo "Recording visible fleet demo to ${PARALLEL_RECORD_OUTPUT}"
fi

docker exec isaac-pid-mavros-batch bash -lc \
  "source /opt/ros/jazzy/setup.bash && python3 /workspace/docker/flight_control/parallel_pid_trial.py \
   --env-count '${count}' --manifest /results/manifest.json --output-dir /results \
   --protocol '${protocol}' --speed '${speed}' --high-speed '${high_speed}' \
   --warmup-seconds '${warmup_seconds}' --hover-seconds '${hover_seconds}'"

stop_recording

python3 docker/flight_control/plot_parallel_pid_batch.py "${output}"
echo "Parallel PID batch complete: ${output}"
