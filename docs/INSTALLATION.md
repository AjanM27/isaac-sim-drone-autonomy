# Installation

This guide starts with a fresh Ubuntu 24.04 x86_64 workstation. The project
bootstrap installs every software dependency; ROS 2, Docker, ArduPilot, MAVROS
and RTAB-Map do not need to be installed manually.

## 1. Hardware and operating-system requirements

| Component | Required for this project |
|---|---|
| Operating system | Ubuntu 24.04 LTS, x86_64 |
| GPU | NVIDIA RTX, 16 GB VRAM recommended |
| Driver | A driver supported by Isaac Sim 6; use NVIDIA's current requirement |
| RAM | 32 GB minimum; 64 GB recommended for multi-drone/SLAM runs |
| Storage | 70 GB free for Isaac Sim, containers, assets and caches |
| Display | Local X11/XWayland desktop for Isaac Sim and RViz GUI |

NVIDIA's [Isaac Sim 6 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)
are authoritative. Integrated graphics and non-NVIDIA GPUs cannot run the RTX
sensor/rendering stack.

## 2. Clone and bootstrap

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/AjanM27/isaac-sim-drone-autonomy.git
cd isaac-sim-drone-autonomy
./scripts/setup_ubuntu.sh --accept-eula
```

Before using `--accept-eula`, read the
[NVIDIA Omniverse EULA](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html).
The setup records acceptance only in the local ignored marker
`.isaac-sim-eula-accepted`.

The command performs these idempotent phases:

1. verifies Ubuntu 24.04 and x86_64;
2. installs command-line utilities and recording tools;
3. installs Docker Engine, Buildx and Compose from Docker's apt repository;
4. installs ROS 2 Jazzy Desktop, including RViz2;
5. verifies the NVIDIA driver;
6. creates `.venv/isaacsim` and installs `isaacsim[all,extscache]==6.0.0.1`;
7. downloads 1.24 GiB of checksummed core, warehouse and hospital assets;
8. validates USD dependency composition;
9. builds pinned ArduPilot, MAVROS and RTAB-Map container images;
10. runs the project doctor.

The Isaac Sim pip installation follows NVIDIA's official
[Python installation procedure](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_python.html).

## 3. Expected one-time interruptions

### NVIDIA driver is missing

The setup stops before installing Isaac Sim. Install Ubuntu's recommended
driver and reboot:

```bash
sudo ubuntu-drivers install
sudo reboot
```

Then rerun the original setup command. Driver installation and reboot cannot
be safely hidden inside a project script.

### Docker was just installed

The setup adds your account to the `docker` group. If the current login session
does not see the new group, log out and back in (or reboot), then rerun setup.
Already downloaded files are checksum-verified and reused.

## 4. Validate

```bash
./scripts/doctor.sh
```

All `FAIL` lines must be corrected. A Hospital warning is acceptable only if
you deliberately selected the smaller core-only installation.

Run focused checks after starting a simulator:

```bash
bash docker/flight_control/test_flight_control.sh
bash docker/flight_control/check_3d_lidar.sh
bash docker/slam/check_slam.sh
```

## Installation options

Install only the warehouse and flight essentials (about 0.25 GiB of assets):

```bash
./scripts/setup_ubuntu.sh --accept-eula --core-assets
```

Useful recovery options:

```bash
# Keep existing apt/Docker/ROS setup
./scripts/setup_ubuntu.sh --accept-eula --skip-system

# Keep downloaded assets
./scripts/setup_ubuntu.sh --accept-eula --skip-assets

# Do not build containers yet
./scripts/setup_ubuntu.sh --accept-eula --skip-images
```

## Use an existing Isaac Sim installation

The launcher searches these locations in order:

1. `ISAAC_SIM_PYTHON`;
2. `$ISAAC_SIM_ROOT/python.sh`;
3. this repository's `.venv/isaacsim/bin/python`;
4. common package locations under `$HOME`.

To use an existing binary package:

```bash
export ISAAC_SIM_ROOT=/absolute/path/to/isaac_sim
./simulation/room_quadcopter/run_room_ardupilot.sh
```

No source file contains a user-specific home-directory path.

## Assets and offline operation

`simulation/room_quadcopter/asset_manifest.json` contains the destination,
public NVIDIA source URL, byte count and SHA-256 for every downloaded file.
Use the standalone installer directly:

```bash
python3 scripts/download_assets.py --profile core --profile warehouse
python3 scripts/download_assets.py --profile hospital
python3 scripts/download_assets.py --profile all --verify-only
```

Downloads are resumable (`.part` files) and installed atomically only after
hash verification. Normal launches use only these local files and existing
Docker images. Building containers or reinstalling assets requires internet;
repeated simulation, teleoperation, SLAM and exploration do not.

For a physically isolated machine, complete setup on a matching online
workstation, clone/copy the repository including ignored `assets/`, save the
three Docker images with `docker save`, and restore them with `docker load`.
The Isaac Sim `.venv` contains absolute paths and should be recreated on the
target rather than copied.

## Updating

```bash
git pull --ff-only
./scripts/setup_ubuntu.sh --accept-eula
```

The setup only downloads missing or changed assets. Rebuild containers after a
Dockerfile change with:

```bash
REBUILD_FLIGHT_STACK=1 bash docker/flight_control/start.sh
bash docker/flight_control/stop.sh
```
