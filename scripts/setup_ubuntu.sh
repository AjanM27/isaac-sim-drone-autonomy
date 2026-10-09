#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_VERSION="${ISAAC_SIM_VERSION:-6.0.0.1}"
accept_eula=0
skip_system=0
skip_assets=0
skip_images=0
assets_profile=all

usage() {
  cat <<'EOF'
Usage: ./scripts/setup_ubuntu.sh --accept-eula [options]

Installs the host prerequisites, Docker, ROS 2 Jazzy/RViz, an isolated Isaac
Sim Python environment, offline assets, and the project container images.

Options:
  --accept-eula       Confirm NVIDIA Omniverse EULA acceptance (required)
  --core-assets       Install only core + warehouse assets, not Hospital
  --skip-system       Do not install apt/Docker/ROS packages
  --skip-assets       Do not download project assets
  --skip-images       Do not build ArduPilot/MAVROS/RTAB-Map images
  --help              Show this help
EOF
}

while (($#)); do
  case "$1" in
    --accept-eula) accept_eula=1 ;;
    --core-assets) assets_profile=core ;;
    --skip-system) skip_system=1 ;;
    --skip-assets) skip_assets=1 ;;
    --skip-images) skip_images=1 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ "$(uname -m)" == "x86_64" ]] || { echo "Isaac Sim requires an x86_64 Ubuntu workstation." >&2; exit 1; }
. /etc/os-release
[[ "${ID}" == "ubuntu" && "${VERSION_ID}" == "24.04" ]] || {
  echo "This reproducible setup targets Ubuntu 24.04 x86_64; found ${PRETTY_NAME}." >&2
  exit 1
}
((accept_eula)) || {
  echo "Isaac Sim requires acceptance of the NVIDIA Omniverse EULA:" >&2
  echo "https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html" >&2
  echo "Review it, then rerun with --accept-eula." >&2
  exit 2
}
export OMNI_KIT_ACCEPT_EULA=YES
touch "${ROOT}/.isaac-sim-eula-accepted"

if ((skip_system == 0)); then
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl git gnupg python3.12 python3.12-venv \
    python3-pip ripgrep ffmpeg x11-utils util-linux jq ubuntu-drivers-common

  if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    printf 'Types: deb\nURIs: https://download.docker.com/linux/ubuntu\nSuites: %s\nComponents: stable\nSigned-By: /etc/apt/keyrings/docker.asc\n' \
      "${UBUNTU_CODENAME}" | sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  fi
  sudo usermod -aG docker "${USER}"

  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe
    ros_apt_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | jq -r .tag_name)"
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.$(. /etc/os-release && echo "${UBUNTU_CODENAME}")_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-desktop
  fi
fi

if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi >/dev/null 2>&1; then
  cat >&2 <<'EOF'
A working NVIDIA driver was not detected. Install the driver recommended by
Ubuntu (`sudo ubuntu-drivers install`), reboot, and rerun this setup command.
Isaac Sim cannot be validated before the reboot.
EOF
  exit 1
fi

python3.12 -m venv "${ROOT}/.venv/isaacsim"
"${ROOT}/.venv/isaacsim/bin/python" -m pip install --upgrade pip wheel
"${ROOT}/.venv/isaacsim/bin/python" -m pip install \
  "torch==2.11.0" \
  --index-url https://download.pytorch.org/whl/cu128
"${ROOT}/.venv/isaacsim/bin/python" -m pip install \
  "isaacsim[all,extscache]==${ISAAC_SIM_VERSION}" \
  --extra-index-url https://pypi.nvidia.com

if ((skip_assets == 0)); then
  if [[ "${assets_profile}" == "all" ]]; then
    python3 "${ROOT}/scripts/download_assets.py" --profile all
    "${ROOT}/simulation/room_quadcopter/vendor_hospital.sh" --verify-only
  else
    python3 "${ROOT}/scripts/download_assets.py" --profile core --profile warehouse
  fi
  "${ROOT}/scripts/isaac_python.sh" \
    "${ROOT}/simulation/room_quadcopter/scripts/vendor_simple_warehouse.py"
fi

if ! docker info >/dev/null 2>&1; then
  cat >&2 <<'EOF'
Docker was installed and your account was added to the docker group. Log out
and back in (or reboot), then rerun this same setup command. Completed downloads
are cached and will not be repeated.
EOF
  exit 1
fi

if ((skip_images == 0)); then
  REBUILD_FLIGHT_STACK=1 "${ROOT}/docker/flight_control/start.sh"
  "${ROOT}/docker/flight_control/stop.sh"
fi

echo
echo "Setup complete. Validation report:"
"${ROOT}/scripts/doctor.sh"
echo
echo "Start the warehouse simulation with:"
echo "  ./simulation/room_quadcopter/run_room_ardupilot.sh"
