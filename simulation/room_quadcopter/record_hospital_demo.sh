#!/usr/bin/env bash
set -euo pipefail

duration="${1:-120}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
output_dir="${root}/simulation/room_quadcopter/results/videos"
timestamp="$(date +%Y%m%d_%H%M%S)"
output="${output_dir}/hospital_slam_${timestamp}.mp4"
display="${DISPLAY:-:0.0}"
resolution="$(xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2; exit}')"

[[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "Duration must be a positive number of seconds." >&2
  exit 1
}
[[ -n "${resolution}" ]] || {
  echo "Could not determine the X11 desktop resolution." >&2
  exit 1
}
mkdir -p "${output_dir}"

echo "Recording the full ${resolution} desktop for ${duration}s to:"
echo "  ${output}"
echo "Arrange Isaac Sim and RViz now; recording starts in 3 seconds."
sleep 3
exec ffmpeg -hide_banner -loglevel warning -stats -y \
  -f x11grab -draw_mouse 1 -framerate 30 -video_size "${resolution}" \
  -i "${display}+0,0" -t "${duration}" \
  -c:v h264_nvenc -preset p5 -tune hq -rc vbr -cq 20 -b:v 0 \
  -pix_fmt yuv420p -movflags +faststart "${output}"
