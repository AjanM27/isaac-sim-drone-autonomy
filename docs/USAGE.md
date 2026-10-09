# Usage and launch reference

Run commands from the repository root. Do not launch two single-drone Isaac
processes simultaneously: both use ArduPilot JSON UDP port `9002`.

## Warehouse flight and teleoperation

Terminal 1:

```bash
./simulation/room_quadcopter/run_room_ardupilot.sh
```

Terminal 2, after Isaac starts publishing sensor data:

```bash
bash docker/flight_control/teleop.sh
```

Terminal 3, optional live pose/velocity/attitude display:

```bash
bash docker/flight_control/live_telemetry.sh
```

Teleoperation sends camera-relative velocity targets while moving and a fixed
position target after braking. ArduPilot—not the keyboard program—owns the
position/velocity, attitude, angular-rate and motor loops.

## Manual Hospital SLAM

```bash
./simulation/room_quadcopter/run_hospital_demo.sh
```

The launcher starts a fresh ArduPilot/MAVROS/RTAB-Map stack, opens the local
Hospital in Isaac Sim, and opens RViz with camera, TF, drone model, 3D point
cloud, trajectory and OctoMap displays. Fly with the normal teleop command.

Record the desktop after arranging Isaac Sim and RViz:

```bash
./simulation/room_quadcopter/record_hospital_demo.sh 120
```

## Autonomous Hospital exploration

```bash
./simulation/room_quadcopter/run_hospital_exploration.sh
```

Optional bounded run:

```bash
./simulation/room_quadcopter/run_hospital_exploration.sh \
  --max-duration 1200 --speed 0.35
```

The explorer arms, takes off, selects reachable 3D frontiers, executes inflated
voxel paths, replans when occupancy changes, returns near the start, and lands.
Results are written to a timestamped directory beneath
`simulation/room_quadcopter/results/exploration/`.

View the most recent saved map:

```bash
./simulation/room_quadcopter/view_voxel_map.sh
```

## Checkpoint-1 trajectory and SLAM benchmark

```bash
bash docker/flight_control/run_checkpoint1.sh
```

This runs isolated 0.25, 0.5 and 1.0 m/s square trajectories and compares:

- Isaac ground-truth pose;
- MAVROS/ArduPilot local pose;
- RTAB-Map `/slam/odom`;
- ICP timing, scan stability and final map duplication metrics.

Outputs include synchronized CSV, JSON metrics, Markdown reports and plots
under `results/checkpoint1/`. See
[CHECKPOINT1.md](../simulation/room_quadcopter/CHECKPOINT1.md).

## PID tuning

Inspect or change a live parameter:

```bash
bash docker/flight_control/pid_tune.sh show
bash docker/flight_control/pid_tune.sh profiles
bash docker/flight_control/pid_tune.sh apply checkpoint1_final
```

Start the autonomous shared-world optimizer:

```bash
bash docker/flight_control/start_autonomous_pid_tuning.sh
bash docker/flight_control/status_autonomous_pid_tuning.sh --watch
```

Stop while retaining completed trials:

```bash
bash docker/flight_control/stop_autonomous_pid_tuning.sh
```

The optimizer uses bounded CMA-ES, multiple simultaneous drones, confirmation
runs and hard safety/RTF gates. See
[PID_TUNING.md](../docker/flight_control/PID_TUNING.md).

## Multi-drone warehouse operation

```bash
# Start nine drones in the visible warehouse
./simulation/room_quadcopter/run_warehouse_fleet.sh 9

# Control drone 1 (use 1..9 for the selected vehicle)
bash docker/flight_control/fleet_teleop.sh 1

# Run and record the synchronized demonstration protocol
./simulation/room_quadcopter/record_warehouse_fleet_demo.sh 9
```

The high-capacity path disables expensive sensor/render features. Ten physics
vehicles and flight stacks do not imply that ten simultaneous RGB cameras,
LiDAR pipelines and RTAB-Map processes will maintain RTF 1.0.

## ROS 2 topics

Principal simulator topics:

| Topic | Type/purpose |
|---|---|
| `/clock` | simulation time |
| `/tf`, `/tf_static` | world, drone and sensor transforms |
| `/camera/front/image_raw` | forward RGB image |
| `/lidar/points` | raw 3D LiDAR cloud |
| `/imu/data` | simulated IMU |
| `/range/down`, `/range/up` | floor/ceiling clearance |
| `/slam/odom` | RTAB-Map ICP odometry |
| `/cloud_map` | accumulated SLAM cloud |
| `/octomap_binary` | compact probabilistic occupancy map |
| `/drone_1/local_position/pose` | MAVROS local pose |
| `/drone_1/setpoint_raw/local` | position/velocity command input |

Inspect topics from the MAVROS container without installing extra host tools:

```bash
docker exec isaac-mavros bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic list'
```

## Environment variables

| Variable | Default | Meaning |
|---|---:|---|
| `ISAAC_SIM_ROOT` | auto | existing binary-package root |
| `ISAAC_SIM_PYTHON` | auto | explicit Isaac-compatible Python executable |
| `ROS_DOMAIN_ID` | `0` | DDS domain shared by host and containers |
| `ISAAC_AUTO_RVIZ` | `1` | open project RViz configuration |
| `FLIGHT_CONTROL_RGB_CAPTURE` | `1` | camera rendering/publication |
| `ISAAC_CAMERA_FPS` | `5` | RGB publication rate |
| `ISAAC_VIEWPORT_WIDTH` | `960` | viewport width |
| `ISAAC_VIEWPORT_HEIGHT` | `540` | viewport height |
| `ISAAC_DLSS_MODE` | `0` | project DLSS performance setting |
| `SLAM_VOXEL_SIZE` | `0.15` | RTAB-Map occupancy resolution, metres |
| `SLAM_RESET_DATABASE` | `true` | start a fresh RTAB-Map database |
| `REBUILD_FLIGHT_STACK` | `0` | force all container image rebuilds |

## Shutdown and cleanup

```bash
bash docker/flight_control/stop.sh
bash docker/flight_control/stop_parallel_stack.sh
```

Generated data can be deleted from `simulation/room_quadcopter/results/`
without affecting source or installed assets. Do not delete `assets/` if the
machine must continue to launch offline.
