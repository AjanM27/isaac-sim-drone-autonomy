#!/usr/bin/env bash
set -euo pipefail

container="${MAVROS_CONTAINER:-isaac-mavros}"
topic="${LIDAR_TOPIC:-/lidar/points}"
expected_frame="${LIDAR_FRAME:-lidar_3d_link}"

if ! docker inspect "${container}" >/dev/null 2>&1; then
  echo "Container '${container}' is not running." >&2
  exit 1
fi

echo "Waiting for one non-empty 3D LiDAR PointCloud2 message on ${topic}..."
docker exec -i "${container}" bash -lc '
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  source /opt/ros/jazzy/setup.bash
  deadline=$((SECONDS + 30))
  type=""
  while [[ $SECONDS -lt $deadline ]]; do
    type=$(ros2 topic type "$1" 2>/dev/null || true)
    [[ -n "$type" ]] && break
    sleep 1
  done
  if [[ "$type" != "sensor_msgs/msg/PointCloud2" ]]; then
    echo "Unexpected topic type: ${type:-<none>}" >&2
    exit 2
  fi
  if ! header=$(timeout --foreground --signal=INT 25s ros2 topic echo "$1" --once --field header); then
    echo "No 3D LiDAR header arrived within 25 seconds." >&2
    exit 3
  fi
  printf "%s\n" "$header"
  if ! grep -q "frame_id: $2" <<<"$header"; then
    echo "Unexpected point-cloud frame; expected $2." >&2
    exit 4
  fi
  if ! sample=$(timeout --foreground --signal=INT 25s ros2 topic echo "$1" --once --field width); then
    echo "No 3D LiDAR point count arrived within 25 seconds." >&2
    exit 5
  fi
  width=$(sed -n "s/.*data: *\([0-9][0-9]*\).*/\1/p; s/^\([0-9][0-9]*\)$/\1/p" <<<"$sample" | head -n1)
  if [[ -z "$width" || "$width" -le 0 ]]; then
    echo "Point cloud is empty (width=${width:-unknown})." >&2
    exit 6
  fi
  echo "Point count: $width"
' bash "${topic}" "${expected_frame}"

echo "3D LiDAR bridge check passed. RViz display: Ouster OS1 3D LiDAR."
