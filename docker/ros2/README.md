# ROS 2 companion container

The container is intentionally separate from Isaac Sim. Isaac Sim keeps the
native GPU/GUI installation, while ROS 2 Jazzy nodes run in a portable
container. Host networking and IPC let Fast DDS discover the Isaac Sim bridge
on the same machine.

## Start and verify

Terminal 1:

```bash
docker/ros2/start.sh
docker compose -f docker/ros2/compose.yml exec ros2 ros2 topic list
```

Terminal 2:

```bash
simulation/room_quadcopter/run_room.sh
```

Press Isaac Sim's Play button, then in Terminal 1 run:

```bash
docker compose -f docker/ros2/compose.yml exec ros2 ros2 topic echo /clock
```

Seeing clock messages confirms that the container can reach Isaac Sim's ROS 2
bridge. Stop the container with:

```bash
docker/ros2/stop.sh
```

The project uses `ROS_DOMAIN_ID=0` and `rmw_fastrtps_cpp` by default. Override
either consistently on both sides if your team uses a different domain or
middleware.

The same check is available as a script:

```bash
bash docker/ros2/check_bridge.sh
```

It waits for one `/clock` message and fails if Isaac Sim is not running,
paused, or using a different ROS domain/middleware.
