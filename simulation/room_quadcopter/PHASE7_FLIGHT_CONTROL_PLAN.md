# Phase 7: Flight-control integration record

## Architecture

```text
ROS 2 mission commands -> MAVROS 2 -> MAVLink UDP -> ArduPilot Copter SITL
                                                        |
                                                        | JSON physics UDP 9002
                                                        v
                                            Isaac Sim rigid body and sensors
```

ArduPilot owns arming, flight modes, attitude and position control, failsafes,
estimation, and motor mixing. Isaac Sim owns the rigid-body world, collisions,
sensors, and conversion of PWM motor outputs into rotor thrust. ROS 2 owns
mission commands, telemetry, mapping, planning, and later swarm coordination.
No custom flight controller should be added while ArduPilot provides this
behavior.

## Implemented milestone

The reproducible stack targets Isaac Sim 6.0, the project-local
`assets/iris/iris.usd`, ArduPilot Copter 4.6.2 SITL, and MAVROS 2 on ROS 2
Jazzy. Host networking exposes JSON physics on UDP 9002 and MAVLink to ROS 2.
The vehicle namespace is `/drone_1`.

The first gate passes when Isaac prints:

```text
ArduPilot SITL connected; Isaac is applying its PWM motor outputs
```

This proves that ArduPilot produces PWM, Isaac applies it to the four
propellers, and Isaac returns simulated state.

## Run order

```bash
./simulation/room_quadcopter/run_room_ardupilot.sh
bash docker/flight_control/test_flight_control.sh
```

The first command starts the companion containers before opening Isaac Sim.
The explicit two-step form remains available for debugging with
`docker/flight_control/start.sh` followed by `run_room.sh --flight-control
ardupilot`.

Primary MAVROS paths are `/drone_1/state`, `/drone_1/extended_state`,
`/drone_1/local_position/pose`, `/drone_1/local_position/velocity_local`,
`/drone_1/imu/data`, `/drone_1/rangefinder/rangefinder`,
`/drone_1/setpoint_position/local`, and `/drone_1/setpoint_velocity/cmd_vel`.
Isaac sensor paths remain `/camera/front/image_raw`, `/lidar/points`,
`/imu/data`, `/range/down`, `/range/up`, `/tf`, and `/tf_static`.

## Validation gates

### 1. Transport health

Both containers must remain up. ArduPilot must report its JSON interface on
127.0.0.1:9002, Isaac must report the PWM connection, and MAVROS must publish
`/drone_1/state` with `connected: true` while Isaac is running. Verify `/clock`,
`/tf`, `/imu/data`, `/lidar/points`, and `/range/down` as well.

### 2. Actuator ownership

Start disarmed and verify the four rotor outputs are at the disarmed value. Arm
through MAVROS, confirm the four USD revolute joints spin, and confirm applied
forces originate only from ArduPilot PWM. Stopping ArduPilot must remove the
motor command path; no altitude PID may continue running.

### 3. Basic flight modes

Use MAVROS to select `GUIDED`, arm, take off to 0.5 m, hold for 20 seconds, log
pose/state/PWM/range, land, and disarm. Repeat at 1.5 m only after the low-height
test is stable. A visual hover caused by scripted force does not count.

### 4. State estimation

The adapter now reads PhysX rigid-body linear/angular velocity, computes
body-frame gyro and specific force, and converts Isaac's Z-up world frame to
ArduPilot NED/FRD. Continue validating those conversions against TF and compare
the ArduPilot rangefinder with `/range/down` after dynamics changes.

### 5. Position and missions

After stable GUIDED hover, test one-axis local position motion, yaw separately,
a square, and waypoint missions. Add warehouse obstacles, validate stopping
distance, and record a rosbag for every test.

### 6. Perception and navigation

Feed simulated IMU and range data through the configured ArduPilot interfaces.
Choose one authoritative source for local position: ArduPilot EKF, external
vision, or a fused ROS estimator. Then add LiDAR filtering and camera processing
as independent ROS 2 nodes. Test localization before the planner, and planner
commands before the swarm layer.

### 7. Multi-vehicle and hardware handoff

Allocate unique `SYSID`, MAVLink ports, JSON ports, ROS namespace, TF prefix,
and container service name from one template for every vehicle. For hardware,
keep ROS 2 interfaces stable while replacing only SITL transport and simulator
sensor sources. Validate motor order, spin direction, PWM endpoints, failsafes,
arming checks, and frame conventions on a restrained vehicle before flight.

## Known limitations

- PWM-to-thrust is a starting calibration, not a validated aerodynamic model.
- The ArduPilot JSON `rng_1` value is derived from local vertical displacement;
  ROS `/range/down` and `/range/up` use scene raycasts. Uneven-terrain control
  should consume a raycast range source through the flight controller.
- The simulated IMU is derived from ideal PhysX state with filtering and
  clipping, not a calibrated noise/bias model of the physical sensors.
- ArduPilot parameters require tuning for this vehicle's mass, inertia, thrust,
  and rotor geometry.
- Arm/takeoff tests are valid only while Isaac is running; a stopped simulator
  can leave a stale ROS graph without live telemetry.

Custom ROS PIDs, direct ROS-to-motor commands, RL control, swarm coordination,
and sim-to-real claims are deferred until the single-vehicle gates pass.
