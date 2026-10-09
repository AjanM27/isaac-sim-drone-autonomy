# First Isaac Sim milestone: Simple Warehouse + custom iris drone

This is the first standalone scene for the rescue-drone simulation:

- a local, vendored copy of the requested Simple Warehouse environment at
  `assets/simple_warehouse/warehouse.usd`, including its local `Props` and
  material/texture dependencies;
- the project-local custom `assets/iris/iris.usd` drone;
- physics-enabled static room geometry;
- ROS 2 bridge startup and `/clock` publishing;
- attached front camera, 3D RTX LiDAR, IMU, and upward/downward range sensors.

Optical flow is a vision measurement of texture and motion; it is not a direct
ground-distance sensor. The downward ToF/range sensor is therefore the
authoritative source for ground distance on `/range/down`. In GUI mode, Isaac
Sim opens in its normal Perspective view, framed on the warehouse; the sensor
cameras render and publish independently.

## Runtime and drone asset

The reproducible setup installs Isaac Sim `6.0.0.1`. The launch wrapper also
supports an existing Isaac Sim 6 binary package through `ISAAC_SIM_ROOT`.
The public repository includes the project Iris USD, so no file from a user's
Downloads directory is required. Override it only when testing another model:

```bash
DRONE_USD_PATH=/path/to/iris.usd simulation/room_quadcopter/run_room.sh
```

## Run

From the workspace root:

```bash
simulation/room_quadcopter/run_room.sh
```

This opens Isaac Sim, loads the local Simple Warehouse environment without a
network request, saves the generated stage as
`simulation/room_quadcopter/stages/room_quadcopter.usd`, and starts the
timeline. The ROS 2 bridge is enabled explicitly; the former `Simulation FPS`
window is intentionally disabled to keep the viewport uncluttered and fast.
The scene starts playing automatically; use `--start-paused` when you want to
inspect the initial pose before physics runs. Relaunching the command creates
a clean stage reset.

The current phase-1 controller holds the drone near 1.5 m using four
propeller thrust forces. It drives the real `rotor0`–`rotor3` revolute joints
so the propellers spin visibly, applies each thrust at that rotor's position,
and temporarily uses a level-attitude stabilizer:

```bash
simulation/room_quadcopter/run_room.sh --target-height 1.5
```

Disable it only when testing the unpowered rigid body:

```bash
simulation/room_quadcopter/run_room.sh --no-hover
```

This is a motor/thrust milestone, not full aerodynamic CFD. The fallback
controller remains available for comparison, but Phase 7 now moves flight
control to ArduPilot SITL/MAVLink through the JSON physics boundary below.

## Phase 7: ArduPilot flight-control integration

The project now includes an ArduPilot JSON physics boundary. In this mode,
ArduPilot owns stabilization, arming, takeoff, landing, and motor PWM output;
Isaac Sim owns the rigid-body physics and returns simulated IMU, pose, velocity,
and range data. The local collective altitude controller is not used.

Start the native Isaac scene in a separate terminal:

```bash
simulation/room_quadcopter/run_room.sh --flight-control ardupilot
```

Build and start ArduPilot SITL, MAVROS 2, and the RTAB-Map 3D SLAM service:

```bash
docker/flight_control/start.sh
```

MAVROS is exposed under `/drone_1`. Once Isaac reports that ArduPilot
SITL has connected, run the command-path smoke test:

```bash
docker/flight_control/test_flight_control.sh
```

For keyboard teleoperation, keep Isaac Sim running and execute this in another
terminal:

```bash
bash docker/flight_control/teleop.sh
```

The teleoperator arms through MAVROS and uses ArduPilot Guided velocity and
position setpoints.
`w/s` move forward/back relative to the front camera, `a/d` move left/right
relative to that camera, and `r/f` move up/down. The teleoperator includes the
Iris asset's 90-degree camera-to-flight-controller mounting offset. Select one of
five speed gears with `1-5`, or step down/up with `-`/`+`. It starts in Cruise
at 1.0 m/s horizontally and 0.5 m/s vertically; the range is 0.25-2.0 m/s
horizontally and 0.2-1.0 m/s vertically. `space` first commands zero velocity,
waits until measured motion has nearly stopped, and only then locks the current
FCU local position. This avoids asking the position controller to chase a point
captured while the drone still has momentum. The validated 0.7 m/s test needed
up to 0.332 m to brake; the strict sub-0.10 m requirement applies to stationary
hover, not high-speed stopping distance. `q` lands and exits. Hold a
movement key to fly; releasing it for 0.6 seconds engages the same brake/hold automatically as
protection against a missed key release. Pending keyboard repeats are drained
as a batch, so a new direction, `space`, or `q` takes priority immediately over
repeats from the previously held key. These are local-frame flight-controller
commands, not direct motor commands. A real vehicle cannot stop with zero
distance; use Precision or Indoor speed (keys `1` or `2`) when minimum stopping
distance is more important than cruise speed.

For repeatable ground-truth checks, run these inside the MAVROS container while
the aircraft is hovering. Do not run keyboard teleop at the same time as the
braking measurement; only one process should publish flight setpoints:

```bash
docker exec isaac-mavros bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /workspace/docker/flight_control/measure_hover.py --ground-z 0.0690559968'

docker exec isaac-mavros bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /workspace/docker/flight_control/measure_braking.py --stop-mode position'
```

Stop the flight-control services with:

```bash
docker/flight_control/stop.sh
```

The interface is intentionally staged: first verify MAVROS connection and
telemetry, then arm/takeoff/land, then position and velocity commands, and only
after those pass tune the custom drone mass, inertia, motor thrust curve, and
sensor timing against measured hardware data.

## Sensor layout

The generated USD attaches the sensors below to the detected rigid drone body
(`/Drone/body` for the current `iris.usd`):

| Prim | Simulation sensor | Mount / rate | Intended hardware |
|---|---|---|---|
| `/Drone/body/CameraFront` | RTX camera mount | forward, z=0.02 m, 5 Hz default | Raspberry Pi Camera Module 3 Wide |
| `/Drone/body/Lidar3D` | Ouster OS1 Rev6-style RTX LiDAR | 32 channels, 512 columns, 10 Hz, 360° | 3D mapping/perception sensor |
| `/Drone/body/Imu` | IMU mount/configuration | body origin, 200 Hz target | flight-controller IMU equivalent |
| `/Drone/body/ToFDown` | downward range mount/configuration | 0.05–12 m, 30 Hz target | MicoAir ToF range sensor; ground distance |
| `/Drone/body/ToFUp` | upward range mount/configuration | 0.05–12 m, 30 Hz target | ceiling/overhead clearance |

The `/clock` ROS 2 topic is live and reachable from the companion container.
The launcher also publishes standard ROS 2 messages on:

```text
/camera/front/image_raw
/camera/front/camera_info
/lidar/points (`sensor_msgs/msg/PointCloud2`)
/slam/lidar/points (the same unfiltered cloud on SLAM's independent frame tree)
/imu/data
/range/down
/range/up
/tf
/tf_static
```

The simulator ground-truth TF tree is rooted at `world`. SLAM uses a parallel
estimated base frame so it never competes with Isaac Sim for `drone_base`:

```text
world
├── drone_base                   (Isaac ground truth)
│   ├── camera_front_optical_frame
│   ├── lidar_3d_link
│   ├── imu_link
│   ├── tof_down_link
│   └── tof_up_link
└── map
    └── icp_odom
        └── slam_base            (RTAB-Map estimate)
            └── slam_lidar_link
```

RViz2 opens automatically with Fixed Frame `map`, TF, the Iris visual proxy,
the unfiltered live 3D cloud, RTAB-Map's accumulated `/cloud_map`, ICP odometry,
the SLAM trajectory, both camera topics, and the downward range display
configured. Disable automatic launch with `ISAAC_AUTO_RVIZ=0`.

Because Isaac Sim publishes simulation time, start RViz with simulated time
enabled and use the included configuration:

```bash
rviz2 -d simulation/room_quadcopter/rviz/room_quadcopter.rviz \
  --ros-args -p use_sim_time:=true
```

The launcher uses ROS 2 Jazzy subnet discovery so host RViz and the Docker ROS
2 companion can discover Isaac Sim over the host network.

The default rendering-performance preset uses DLSS Performance mode, a
1280x720 viewport, and disables expensive secondary RTX effects. Override it
with `ISAAC_ENABLE_RTX_OPTIMIZATION=0`, or change `ISAAC_DLSS_MODE`,
`ISAAC_VIEWPORT_WIDTH`, and `ISAAC_VIEWPORT_HEIGHT`.

The camera topics use RGB render products attached to the actual USD camera
prims. The 3D LiDAR is authored from Isaac Sim's bundled Ouster OS1 Rev6
32-channel, 10 Hz, 512-column configuration, so no remote LiDAR asset is needed
at runtime. Isaac's RTX sensor produces the native Cartesian returns; the room
bridge publishes them as `sensor_msgs/msg/PointCloud2` on `/lidar/points` for
reliable operation with Isaac Sim 6.0.1-rc.7. The raw topic retains floor
returns for mapping, learning, and RViz; no warehouse-floor filter is applied.
The completed Cartesian scans currently arrive at about 5 Hz in the full
rendering pipeline. The downward range value
follows the ToF prim's world height above the warehouse floor, so it changes
when the drone is moved vertically. IMU data remains a stable compatibility
publisher until the experimental physics IMU path is enabled safely.

With the simulator running, verify a non-empty cloud and its TF frame with:

```bash
bash docker/flight_control/check_3d_lidar.sh
```

## 3D LiDAR SLAM

The `rtabmap-slam` Compose service runs ROS 2 Jazzy RTAB-Map with point-to-plane
ICP odometry. It consumes `/slam/lidar/points`, publishes estimated odometry on
`/slam/odom`, the optimized transform chain `map -> icp_odom -> slam_base`, an
accumulated PointCloud2 map on `/cloud_map`, and its trajectory on `/mapPath`.
The map database is stored under `simulation/room_quadcopter/results/slam` and
is reset on each normal stack start. Set `SLAM_RESET_DATABASE=false` before
starting the stack to continue an existing database.

The default indoor voxel size is 0.15 m. Override it before startup with
`SLAM_VOXEL_SIZE`, for example `SLAM_VOXEL_SIZE=0.10`. LiDAR deskewing is
deliberately disabled because the current Cartesian PointCloud2 has XYZ fields
but no per-point timestamps. The floor is retained in both ICP and the 3D map.

Start and validate the integrated stack with:

```bash
docker/flight_control/start.sh
simulation/room_quadcopter/run_room.sh --flight-control ardupilot
# In another terminal, after the scene is publishing:
bash docker/slam/check_slam.sh
```

The accumulated `/cloud_map` is intentionally empty while the vehicle remains
motionless at its starting pose. Take off and move far enough to create at least
one SLAM keyframe before running the full health check.

For the repeatable moving-flight Checkpoint-1 benchmark, including isolated
0.25/0.5/1.0 m/s runs, synchronized ground-truth/MAVROS/RTAB-Map traces, plots,
and the current deskew decision, see [`CHECKPOINT1.md`](CHECKPOINT1.md).

The first Docker build downloads RTAB-Map and its PCL dependencies. Later
starts use the local image and do not reinstall packages.

For motion-compensated LiDAR scans, set `ISAAC_LIDAR_MOTION_BVH=1`. This is
disabled by default because it adds a large RTX pipeline compilation and VRAM
cost; uncompensated scans are preferable for the current indoor teleoperation
milestone.

Real camera images are enabled during ordinary ArduPilot flight control at
320x240 and 5 Hz by default. Setting `FLIGHT_CONTROL_RGB_CAPTURE=0` disables
camera rendering and publication completely, which is used by autonomous
exploration to preserve real-time factor.

## Autonomous hospital exploration

The hospital launcher uses only vendored local USD/material assets, starts one
Iris, ArduPilot, MAVROS, RTAB-Map and RViz, then performs direction-aware
frontier exploration:

```bash
simulation/room_quadcopter/run_hospital_exploration.sh
```

It incrementally ray-traces the SLAM-aligned 3D LiDAR into a 30 cm hashed voxel
map, uses temporal hit/miss evidence to reject transient LiDAR noise, clusters
3D free/unknown boundaries at multiple resolution, scores their information
gain, and plans collision-checked XYZ paths with 3D A*. Occupancy is inflated
by the Iris horizontal and vertical footprint and every active path is
revalidated as new scans arrive. Downward/upward ToF stop horizontal motion and
climb or descend when the lower/upper 0.60 m safety volume is obstructed.
Reached and invalid frontiers are remembered, and the vehicle returns over the
shortest known-safe voxel path (with breadcrumbs as fallback) before landing.
RTAB-Map's full `/cloud_map` and raw floor
returns remain unchanged; exploration no longer depends on a flattened 2D map.
Planning uses RTAB-Map's continuous `icp_odom` frame so loop-closure corrections
cannot jump the controller target. RViz still transforms the result into `map`.
The hospital's measured local bounds constrain the sparse map so rays through
doors or windows cannot create infinite exterior frontiers. By default there is
no time cutoff: completion requires eight consecutive checks with no reachable
3D frontier. Stalled targets are abandoned after 15 seconds. `summary.json`
distinguishes `frontier_exhausted` (complete) from `time_limit` (incomplete) and
records remaining frontiers, observed volume/area, safety minima and return error.

Mapping deliberately has two layers. RTAB-Map's 15 cm probabilistic OctoMap is
the canonical global, persistent and future multi-drone exchange format. It
publishes `/octomap_binary` plus RViz-ready `/octomap_occupied_space` and
`/octomap_global_frontier_space`. The explorer separately keeps a coarser 30 cm
hashed leaf cache in `icp_odom`; this makes local frontier and A* queries cheap
and protects vehicle control from loop-closure frame jumps. It is a planning
cache, not a second authoritative global map.

For low-latency multi-drone updates, each drone can additionally publish only
changed cache leaves on `/drone_1/exploration/voxel_delta`. It is a standard
PointCloud2 with voxel-centre `x/y/z`, quantized log-odds in `intensity`, and
stable free/unknown/occupied classification in `state`. The corresponding
`/drone_1/exploration/voxel_metadata` topic describes resolution, frame and
hysteresis thresholds. A future fusion node should transform each robot's
OctoMap into a shared frame and combine occupancy evidence; the delta stream is
an optimization and must not be treated as a complete map by itself.

RViz enables RTAB-Map's occupied OctoMap by default. The coarser exploration
cache can be enabled as a diagnostic; its frontiers and active route remain on
`/exploration/voxel_markers` and `/exploration/planned_path_3d`. All use SLAM
simulation timestamps and transform into RViz's fixed `map` frame.

Each run saves `exploration_3d.csv`, `voxel_map.npz`,
`voxel_map_metadata.json`, `octomap_binary.npz`, `octomap_status.json`, `summary.json`,
`isaac.log`, `rtf.txt`, and a copy of `rtabmap.db` under
`results/exploration/hospital_exploration_*`. Override
duration or speed when a bounded test rather than full exploration is needed:

```bash
simulation/room_quadcopter/run_hospital_exploration.sh --max-duration 1200 --speed 0.35
```

After a run, reopen the latest completed stabilized voxel map in RViz with:

```bash
simulation/room_quadcopter/view_voxel_map.sh
```

Pass a run directory or a specific `voxel_map.npz` to inspect an older result.
Occupied cells are enabled by default; `/exploration/free_voxels` and
`/exploration/uncertain_voxels` are also published for diagnostics.

Read the ground distance from ROS 2 with:

```bash
ros2 topic echo /range/down sensor_msgs/msg/Range
```

The value is in metres (`msg.range`). Use `/range/up` in the same way for
ceiling clearance. These are lightweight physics raycasts and require no
downward camera or render product.

To validate stage creation without opening a window:

```bash
simulation/room_quadcopter/run_room.sh --headless
```

To run a short physics smoke test without the GUI:

```bash
simulation/room_quadcopter/run_room.sh --headless --run-seconds 5
```

Then start the ROS 2 companion and verify the bridge:

```bash
docker/ros2/start.sh
docker/ros2/check_bridge.sh
```

The warehouse asset is intentionally local. You can override it only with
another local USD using `WAREHOUSE_USD_PATH`:

```bash
WAREHOUSE_USD_PATH=/path/to/warehouse.usd simulation/room_quadcopter/run_room.sh
```

This scene no longer depends on the stock drone asset service. For the Docker
ROS 2 setup, see [`docker/ros2/README.md`](../../docker/ros2/README.md).
