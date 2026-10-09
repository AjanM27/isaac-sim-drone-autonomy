#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
failures=0
warnings=0

pass() { printf 'PASS  %s\n' "$*"; }
warn() { printf 'WARN  %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf 'FAIL  %s\n' "$*"; failures=$((failures + 1)); }

if [[ "$(. /etc/os-release 2>/dev/null; printf '%s' "${ID:-}")" == "ubuntu" ]]; then
  pass "Ubuntu detected"
else
  fail "Ubuntu is required"
fi

arch="$(uname -m)"
[[ "${arch}" == "x86_64" ]] && pass "x86_64 architecture" || fail "x86_64 required (found ${arch})"

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  gpu="$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
  driver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
  pass "NVIDIA GPU: ${gpu}; driver ${driver}"
else
  fail "working NVIDIA driver not found"
fi

if command -v docker >/dev/null 2>&1; then
  pass "Docker CLI installed"
  docker info >/dev/null 2>&1 && pass "Docker daemon reachable" || fail "Docker daemon unavailable or user lacks docker-group access"
  docker compose version >/dev/null 2>&1 && pass "Docker Compose plugin installed" || fail "Docker Compose plugin missing"
else
  fail "Docker not installed"
fi

if runtime="$("${ROOT}/scripts/isaac_python.sh" --print-runtime 2>/dev/null)"; then
  pass "Isaac Sim Python runtime found: ${runtime}"
else
  fail "Isaac Sim runtime missing"
fi

[[ -f /opt/ros/jazzy/setup.bash ]] && pass "ROS 2 Jazzy host installation found" || warn "ROS 2 Jazzy host tools absent; container flight stack still works, but automatic RViz does not"

if python3 "${ROOT}/scripts/download_assets.py" --profile core --profile warehouse --verify-only >/dev/null 2>&1; then
  pass "core and warehouse assets verified"
else
  fail "core/warehouse assets missing or modified"
fi

if python3 "${ROOT}/scripts/download_assets.py" --profile hospital --verify-only >/dev/null 2>&1; then
  pass "hospital assets verified"
else
  warn "hospital assets not installed; hospital launchers will be unavailable"
fi

for image in isaac-flight-control-ardupilot-sitl:local isaac-flight-control-mavros:local flight_control-rtabmap-slam:latest; do
  docker image inspect "${image}" >/dev/null 2>&1 && pass "container image ${image}" || warn "container image ${image} has not been built yet"
done

printf '\nDoctor summary: %d failure(s), %d warning(s)\n' "${failures}" "${warnings}"
((failures == 0))
