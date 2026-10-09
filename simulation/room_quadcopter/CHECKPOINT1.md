# Checkpoint 1: dynamic flight and 3D SLAM validation

## Outcome

The ArduPilot controller now uses the validated `checkpoint1_final` profile.
Its final horizontal values are `PSC_POSXY_P=1.0500`, `PSC_VELXY_P=2.2000`,
`PSC_VELXY_I=1.0000`, `PSC_VELXY_D=0.2500`, and `PSC_JERK_XY=20.0000`.
The benchmark flies
a 2 m square in the warehouse with takeoff, four straight segments, explicit
90-degree yaw changes, a return hold, and landing.

The trajectory command is `mavros_msgs/PositionTarget` on
`/drone_1/setpoint_raw/local`. Position and velocity are enabled together
(ArduPilot Guided mask 3520): position supplies endpoint authority, velocity is
feed-forward, and ArduPilot retains jerk limiting, position/velocity control,
attitude/rate control, mixing, and motor PWM ownership.

## Run

Run the complete isolated three-speed benchmark from the repository root:

```bash
bash docker/flight_control/run_checkpoint1.sh
```

The script refuses to run alongside another ArduPilot Isaac process. For every
speed it restarts SITL, MAVROS, RTAB-Map, and Isaac; waits for `/clock` and a
real 3D LiDAR cloud; flies; lands; saves logs; calculates metrics; and stops the
headless simulator. It uses existing Docker images and the local warehouse, so
normal reruns do not require internet access.

To run one speed while an independently started simulator is active:

```bash
bash docker/flight_control/run_benchmark.sh 0.5 my_0p5_repeat
```

Outputs are stored under
`simulation/room_quadcopter/results/checkpoint1/<run>/`:

- `trajectory.csv`: synchronized command, Isaac ground truth, MAVROS pose,
  RTAB-Map odometry, scan count, and map count;
- `slam_diagnostics.csv`: each unique RTAB-Map ICP update with loss flag,
  inlier ratio, correspondences, processing time, and local-map size;
- `metrics.json` and `report.md`;
- `trajectory_xy.png`, `position_error.png`, and `altitude_yaw.png`;
- isolated Isaac, ArduPilot, and RTAB-Map logs.

`SUMMARY.md` and `speed_comparison.png` compare all speeds.

## Verified final-profile results (2026-09-05)

Dynamic metrics exclude variable EKF startup and landing waits. MAVROS uses the
known Iris axis conversion. Both MAVROS and RTAB-Map are anchored exactly once
at their first valid Isaac sample; there is no whole-trajectory fit and no
scale correction. Consequently aligned error at `t=0` is exactly zero while
all subsequent yaw, scale, and drift errors remain measurable.

| Speed | Raw MAV t0 | Raw SLAM t0 | Aligned t0 | MAVROS RMSE | SLAM RMSE | SLAM max | Loop drift | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 0.25 m/s | 0.020 m | 0.101 m | 0.000 m | 0.131 m | 0.219 m | 0.413 m | 0.059 m | pass |
| 0.50 m/s | 0.068 m | 0.055 m | 0.000 m | 0.176 m | 0.399 m | 0.759 m | 0.137 m | fail |
| 1.00 m/s | 0.086 m | 0.106 m | 0.000 m | 0.324 m | 0.671 m | 1.673 m | 0.119 m | fail |

The raw offsets above are expected because MAVROS and RTAB-Map own local
origins rather than Isaac world coordinates. The simulator now also publishes
the static `world -> map` transform from the drone body's actual spawn pose, so
RViz starts SLAM at the same physical location instead of the warehouse
origin. This transform is calibrated once at startup and does not feed live
ground truth into the estimator.

The reliability gate is SLAM RMSE <= 0.30 m, maximum error <= 0.75 m, lost
updates <= 1%, and no odometry correction exceeding simultaneous ground-truth
motion by 0.30 m. All runs had stable point counts, a final voxel-unique ratio
of 1.0, and no map-duplication signal. ICP p95 is far below the 200 ms scan
period and no update reported lost, so CPU processing is not the bottleneck.
The speed-dependent errors and corner corrections are consistent with motion
distortion in completed XYZ-only scans.

## Decision and next checkpoint

The corrected initial-only evaluation still validates the no-deskew pipeline
only at 0.25 m/s for this path and
turn rate. Before faster autonomous exploration, publish a per-point time field
from the Isaac RTX LiDAR, transform points over the acquisition interval using
the measured vehicle motion, enable RTAB-Map ICP deskewing, and rerun these
unchanged speed gates. After deskewing passes, add obstacle-aware planning and
collision-stop tests before RL or multi-drone work.

## Method and references

The tuning order follows ArduPilot's documented plant-first approach and the
AutoTune principle of finding response limits one controller group at a time,
then backing away from instability. Guided trajectories use ArduPilot's
documented combined position/velocity interface.

- ArduPilot tuning process: https://ardupilot.org/copter/docs/common-tuning.html
- ArduPilot AutoTune: https://ardupilot.org/copter/docs/autotune.html
- Guided position/velocity commands: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- ArduPilot JSON SITL interface: https://ardupilot.org/dev/docs/sitl-with-JSON.html
- RTAB-Map 3D LiDAR deskew example: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/lidar3d_assemble.launch.py
