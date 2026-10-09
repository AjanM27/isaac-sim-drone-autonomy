# Troubleshooting

Start with:

```bash
./scripts/doctor.sh
```

## Isaac Sim runtime was not found

Run the bootstrap, or export the root of an existing binary installation:

```bash
./scripts/setup_ubuntu.sh --accept-eula
# or
export ISAAC_SIM_ROOT=/absolute/path/to/isaac_sim
```

See what the launcher selected:

```bash
./scripts/isaac_python.sh --print-runtime
```

## Docker permission denied

Your current login predates membership in the `docker` group. Log out and back
in, then verify `docker info`. Do not permanently run the simulation scripts
with `sudo`, because generated results would become root-owned.

## MAVROS waits forever or is disconnected

1. Start the flight containers through the normal launcher.
2. Start Isaac Sim and wait for `ArduPilot SITL connected`.
3. Check:

   ```bash
   docker logs --tail 100 isaac-ardupilot-sitl
   docker logs --tail 100 isaac-mavros
   ```

4. Ensure no stale simulator owns UDP `9002`:

   ```bash
   ss -lunp | grep 9002
   ```

5. Stop everything and relaunch:

   ```bash
   bash docker/flight_control/stop.sh
   pkill -f create_room_quadcopter.py || true
   ./simulation/room_quadcopter/run_room_ardupilot.sh
   ```

## ArduPilot refuses arming

Read the actual PREARM reason:

```bash
docker exec isaac-mavros bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /drone_1/statustext/recv'
```

Typical causes are missing JSON sensor packets, an insufficient sensor rate,
or an EKF that has not yet established local position. Do not bypass pre-arm
checks; fix the simulator timing/input problem.

## RViz does not open

Confirm ROS 2 Jazzy desktop is installed and the simulator publishes topics:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
rviz2 -d simulation/room_quadcopter/rviz/room_quadcopter.rviz \
  --ros-args -p use_sim_time:=true
```

On Wayland, Isaac Sim and RViz normally use XWayland. Verify `DISPLAY` is set.

## RViz shows topics but no data

All participants must use the same `ROS_DOMAIN_ID`. Containers force Fast DDS
to UDP because private container shared memory can otherwise produce visible
topic names with no delivered messages. Stop old containers before changing
the domain.

## Environment or drone is missing

Verify or repair downloaded files:

```bash
python3 scripts/download_assets.py --profile all --verify-only
python3 scripts/download_assets.py --profile all
```

The second command only downloads missing or checksum-mismatched files.

## Hospital launches slowly

The first composition warms USD, material and shader caches. Later launches
reuse them. Confirm the asset verifier passes; unresolved references can cause
long network timeouts.

## Low real-time factor

- Close other Isaac Sim instances.
- Use the default 960×540 viewport and DLSS performance mode.
- Disable RGB capture for mapping-only runs:

  ```bash
  FLIGHT_CONTROL_RGB_CAPTURE=0 ./simulation/room_quadcopter/run_hospital_demo.sh
  ```

- Use headless runs for benchmarks.
- Enable the CPU performance governor for multi-drone tuning.
- Sensor-rich ten-drone SLAM is intentionally not claimed to run at RTF 1.0.

## Reset runtime state

```bash
bash docker/flight_control/stop.sh
docker rm -f isaac-ardupilot-sitl isaac-mavros isaac-rtabmap-slam 2>/dev/null || true
```

Generated results are under `simulation/room_quadcopter/results/`; moving that
directory aside resets maps and logs without changing code or assets.
