#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_FILE="${ROOT}/docker/flight_control/compose.yml"

required_images=(
  isaac-flight-control-ardupilot-sitl:local
  isaac-flight-control-mavros:local
  flight_control-rtabmap-slam:latest
)
missing_images=()
for image in "${required_images[@]}"; do
  if ! docker image inspect "${image}" >/dev/null 2>&1; then
    missing_images+=("${image}")
  fi
done

if [[ "${REBUILD_FLIGHT_STACK:-0}" == "1" || ${#missing_images[@]} -gt 0 ]]; then
  if [[ ${#missing_images[@]} -gt 0 ]]; then
    echo "Building missing flight-control images: ${missing_images[*]}"
  else
    echo "REBUILD_FLIGHT_STACK=1: rebuilding flight-control images"
  fi
  docker compose -f "${COMPOSE_FILE}" build
fi

# Normal launches are deliberately --no-build: BuildKit resolves base-image
# metadata online even when every build layer is cached. Prebuilt local images
# therefore make repeated simulator launches fully offline and much faster.
docker compose -f "${COMPOSE_FILE}" up -d --no-build
# `up -d` leaves unchanged containers running. Restart all services so every
# simulator session begins with a fresh JSON timestamp and EKF/logger state.
docker compose -f "${COMPOSE_FILE}" restart ardupilot-sitl mavros rtabmap-slam

# ArduPilot cannot complete sensor/EKF initialization until Isaac starts
# sending JSON physics packets. Waiting for "ArduPilot Ready" here creates a
# guaranteed 45-second pre-simulator delay. Teleop performs the real readiness
# gate after Isaac is running.
echo "Flight-control containers started; Isaac will now provide SITL sensor data."
