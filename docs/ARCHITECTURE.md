# Architecture

## Process boundary

The platform intentionally separates GPU simulation from portable robotics
services:

| Process | Location | Responsibility |
|---|---|---|
| Isaac Sim | native host process | PhysX, rendering, sensors, truth state, motor forces |
| ArduPilot SITL | Docker, host network | cascaded flight control and motor PWM |
| MAVROS | Docker, host network | MAVLink-to-ROS 2 services, state and setpoints |
| RTAB-Map | Docker, host network | ICP odometry, graph SLAM, cloud and OctoMap |
| RViz2 | host | operator visualization |
| Teleop/explorer/benchmarks | MAVROS container | flight commands and measurements |

Host networking is required for low-friction DDS discovery and ArduPilot's
JSON/MAVLink sockets. Fast DDS is forced to UDPv4 in containers because host
and container `/dev/shm` are not shared.

## Flight data path

1. Isaac evaluates rigid-body pose, velocity and IMU state at physics rate.
2. The JSON bridge sends timestamped synthetic sensor packets to UDP `9002`.
3. ArduPilot's EKF and cascaded controllers calculate four motor PWM outputs.
4. Isaac converts normalized PWM into individual thrust and reaction torque.
5. MAVROS exposes ArduPilot state and accepts GUIDED position/velocity targets.

Visual rotor animation is independent of force generation. Sensors are
massless attachments; the flight body uses explicitly authored Iris mass,
centre-of-mass and inertia to keep vehicle dynamics independent of decorative
sensor meshes.

## Control layers

```text
position target
  -> horizontal/vertical position controller
  -> desired velocity
  -> velocity/acceleration controller
  -> desired attitude and thrust
  -> attitude controller
  -> angular-rate PID
  -> motor mixer/PWM
  -> Isaac thrust and torque
```

Manual and autonomous clients do not command motors directly. Translation uses
combined position/velocity setpoints where appropriate; teleoperation uses
camera-relative velocity while a key is active and changes to position hold
after bounded zero-velocity braking.

## Perception and frames

The drone publishes front camera, IMU, OS1 point cloud, upward/downward range,
ground truth and a complete TF tree. Important frames are:

```text
world -> map -> icp_odom -> slam_base
world -> drone_base -> camera_front
                    -> lidar_link
                    -> imu_link
                    -> range_down / range_up
```

`world -> map` is calibrated once at startup. RTAB-Map owns the subsequent
`map -> icp_odom` correction. Ground truth is recorded for evaluation and is
not fed continuously into SLAM.

## Mapping representations

- `/cloud_map`: high-detail accumulated points for inspection.
- `/octomap_binary`: canonical 15 cm probabilistic free/occupied/unknown map
  for persistence and future robot-to-robot exchange.
- explorer sparse cache: 30 cm quantized log-odds leaves in `icp_odom` for
  fast local A*, inflation and frontier queries.

The canonical global map and local planning cache are deliberately separate.
Loop-closure corrections may move the global map; they must not jump an active
flight-control target.

## Reproducibility boundary

Versioned in Git:

- all project source and configuration;
- Dockerfiles and pinned ArduPilot ref;
- tuned parameter/search profiles;
- asset URL/size/SHA-256 manifest;
- RViz and launch configuration;
- test and benchmark methodology.

Generated locally:

- Isaac Sim pip environment and extension cache;
- NVIDIA/CC BY asset files;
- Docker images and containers;
- stages, logs, maps, plots and recordings.

This keeps the repository reviewable while making the complete runtime
reconstructible from one setup command.
