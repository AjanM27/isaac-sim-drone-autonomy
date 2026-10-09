#!/usr/bin/env python3
"""Keyboard teleop using ArduPilot Guided velocity and position control."""

import os
import argparse
import math
import sys
import termios
import time
import tty
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, HomePosition, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


SPEED_PROFILES = (
    ("Precision", 0.25, 0.20),
    ("Indoor", 0.50, 0.30),
    ("Cruise", 1.00, 0.50),
    ("Fast", 1.50, 0.70),
    ("Maximum", 2.00, 1.00),
)
DEFAULT_SPEED_LEVEL = 2
MOVEMENT_TIMEOUT_SECONDS = 0.60
BRAKE_SPEED_THRESHOLD_MPS = 0.04
BRAKE_VERTICAL_THRESHOLD_MPS = 0.05
BRAKE_TIMEOUT_SECONDS = 3.0
BRAKE_STABLE_SECONDS = 0.25
# The Iris asset's front-camera axis is +90 degrees from the FCU/local-pose
# forward axis. Without this mount correction, W commands camera-right.
CAMERA_HEADING_OFFSET_ENU = math.pi / 2.0
# MAVROS raw local targets use NED yaw. Isaac world is ENU, so an Isaac-world
# heading of zero corresponds to +90 degrees in the NED heading convention.
ISAAC_WORLD_ZERO_YAW_NED = math.pi / 2.0
MOVEMENT_DIRECTIONS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "r": (0.0, 0.0, 1.0),
    "f": (0.0, 0.0, -1.0),
}


def slew_velocity(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    dt: float,
    horizontal_accel: float = 2.5,
    vertical_accel: float = 1.5,
) -> tuple[float, float, float]:
    """Acceleration-limit operator setpoints without replacing ArduPilot control."""
    dx, dy = target[0] - current[0], target[1] - current[1]
    distance = math.hypot(dx, dy)
    horizontal_step = max(0.0, horizontal_accel * dt)
    scale = min(1.0, horizontal_step / distance) if distance else 1.0
    z_step = max(0.0, vertical_accel * dt)
    dz = max(-z_step, min(z_step, target[2] - current[2]))
    return (
        current[0] + dx * scale,
        current[1] + dy * scale,
        current[2] + dz,
    )


def speed_status(level: int) -> str:
    """Return the concise status shown whenever the speed gear changes."""
    name, horizontal, vertical = SPEED_PROFILES[level]
    return (
        f"Speed {level + 1}/{len(SPEED_PROFILES)} [{name}]: "
        f"horizontal={horizontal:.2f} m/s, vertical={vertical:.2f} m/s"
    )


def velocity_for_key(key: str | None, level: int) -> tuple[float, float, float]:
    """Convert a movement key and speed gear into camera/body FLU velocity."""
    if key not in MOVEMENT_DIRECTIONS:
        return (0.0, 0.0, 0.0)
    _, horizontal, vertical = SPEED_PROFILES[level]
    x, y, z = MOVEMENT_DIRECTIONS[key]
    return (x * horizontal, y * horizontal, z * vertical)


class Teleop(Node):
    def __init__(
        self,
        namespace: str = "/drone_1",
        node_name: str = "ardupilot_keyboard_teleop",
    ) -> None:
        super().__init__(node_name)
        self.namespace = "/" + namespace.strip("/")
        self.state = State()
        self.extended_state = ExtendedState()
        self.extended_state_received = False
        self.home_received = False
        self.gps_received = False
        self.local_pose: PoseStamped | None = None
        self.local_velocity: TwistStamped | None = None
        self.last_takeoff_request_time: float | None = None
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(State, f"{self.namespace}/state", self.state_callback, latched_qos)
        self.create_subscription(
            ExtendedState,
            f"{self.namespace}/extended_state",
            self.extended_state_callback,
            latched_qos,
        )
        self.create_subscription(
            HomePosition, f"{self.namespace}/home_position/home", self.home_callback, latched_qos
        )
        self.create_subscription(
            NavSatFix,
            f"{self.namespace}/global_position/raw/fix",
            self.gps_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            f"{self.namespace}/local_position/pose",
            self.local_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            f"{self.namespace}/local_position/velocity_local",
            self.local_velocity_callback,
            qos_profile_sensor_data,
        )
        self.raw_setpoint = self.create_publisher(
            PositionTarget, f"{self.namespace}/setpoint_raw/local", 10
        )
        self.set_mode = self.create_client(SetMode, f"{self.namespace}/set_mode")
        self.arm = self.create_client(CommandBool, f"{self.namespace}/cmd/arming")
        self.takeoff = self.create_client(CommandTOL, f"{self.namespace}/cmd/takeoff")
        self.command = self.create_client(CommandLong, f"{self.namespace}/cmd/command")

    def state_callback(self, message: State) -> None:
        self.state = message

    def extended_state_callback(self, message: ExtendedState) -> None:
        self.extended_state = message
        self.extended_state_received = True

    def home_callback(self, _message: HomePosition) -> None:
        self.home_received = True

    def gps_callback(self, message: NavSatFix) -> None:
        self.gps_received = message.status.status >= 0

    def local_pose_callback(self, message: PoseStamped) -> None:
        self.local_pose = message

    def local_velocity_callback(self, message: TwistStamped) -> None:
        self.local_velocity = message

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """Process callbacks; batch tools may provide one shared executor."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def camera_velocity_to_local(
        self, forward: float, left: float, up: float
    ) -> tuple[float, float, float]:
        """Rotate camera/body FLU velocity into the MAVROS local ENU frame."""
        if self.local_pose is None:
            return (forward, left, up)
        q = self.local_pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ) + CAMERA_HEADING_OFFSET_ENU
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            cosine * forward - sine * left,
            sine * forward + cosine * left,
            up,
        )

    def send_camera_velocity(self, forward: float, left: float, up: float) -> None:
        """Command velocity relative to the forward-facing camera."""
        self.send_velocity(*self.camera_velocity_to_local(forward, left, up))

    def is_nearly_stopped(self) -> bool:
        """Return true once measured local velocity is safe to position-lock."""
        if self.local_velocity is None:
            return False
        velocity = self.local_velocity.twist.linear
        return (
            math.hypot(velocity.x, velocity.y) <= BRAKE_SPEED_THRESHOLD_MPS
            and abs(velocity.z) <= BRAKE_VERTICAL_THRESHOLD_MPS
        )

    def send_velocity(
        self, x: float, y: float, z: float, yaw: float = ISAAC_WORLD_ZERO_YAW_NED
    ) -> None:
        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.velocity.x = x
        message.velocity.y = y
        message.velocity.z = z
        message.yaw = yaw
        self.raw_setpoint.publish(message)

    def send_active_brake(self) -> None:
        """Request a monotonic stop through ArduPilot's Guided velocity loop.

        Experiments with reverse-velocity and acceleration feed-forward targets
        excited the delayed plant around zero. A zero target is slower but
        stable, and leaves all braking/attitude/motor control inside ArduPilot.
        """
        self.send_velocity(0.0, 0.0, 0.0)

    def capture_position_hold(self) -> PoseStamped | None:
        """Capture the FCU's current local pose as a fixed Guided target."""
        if self.local_pose is None:
            return None
        target = deepcopy(self.local_pose)
        target.header.frame_id = target.header.frame_id or "map"
        return target

    def send_position_hold(
        self, target: PoseStamped, yaw: float = ISAAC_WORLD_ZERO_YAW_NED
    ) -> None:
        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = target.header.frame_id or "map"
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position = target.pose.position
        message.yaw = yaw
        self.raw_setpoint.publish(message)

    def send_position_velocity(
        self,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        yaw: float = ISAAC_WORLD_ZERO_YAW_NED,
    ) -> None:
        """Send ArduPilot a jerk-limited position target with velocity feed-forward.

        Copter Guided mode supports position and velocity together (type mask
        3520).  This is preferable to a staircase of position-only targets for
        repeatable trajectories and leaves acceleration shaping to ArduPilot.
        """
        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position.x, message.position.y, message.position.z = position
        message.velocity.x, message.velocity.y, message.velocity.z = velocity
        message.yaw = yaw
        self.raw_setpoint.publish(message)

    def enable_local_position_stream(self) -> None:
        """Ask ArduPilot for LOCAL_POSITION_NED at 20 Hz via MAVLink."""
        if not self.command.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MAVROS command service is unavailable")
        for attempt in range(3):
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
            request.param1 = 32.0  # MAVLINK_MSG_ID_LOCAL_POSITION_NED
            request.param2 = 50_000.0
            future = self.command.call_async(request)
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                self.spin_once(timeout_sec=0.05)
            if future.done() and future.result().success:
                return
            # Simultaneous SITL startups can race the interval request while
            # MAVROS is already receiving the default local-pose stream.
            if self.local_pose is not None:
                return
            if attempt < 2:
                time.sleep(0.5)
        # This command is an optimization, not a prerequisite. MAVROS normally
        # receives LOCAL_POSITION_NED at ArduPilot's default stream rate; the
        # caller below independently waits for an actual pose before arming.
        self.get_logger().warning(
            "ArduPilot did not accept the 20 Hz local-position request; using the default stream"
        )

    def request_land(self) -> None:
        """Stop horizontal motion and put an armed vehicle into LAND mode."""
        self.send_velocity(0.0, 0.0, 0.0)
        if not self.state.armed or not self.set_mode.wait_for_service(timeout_sec=3.0):
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "LAND"
        future = self.set_mode.call_async(request)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.1)
        if future.done() and future.result().mode_sent:
            print("LAND requested; ArduPilot will descend and disarm.")
        else:
            print("Warning: LAND request was not acknowledged; check /drone_1/state.")

    def request_flight_mode(self, mode_name: str, timeout: float = 4.0) -> bool:
        """Request a mode and wait until the FCU reports that exact mode."""
        if not self.set_mode.wait_for_service(timeout_sec=timeout):
            return False
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode_name
        future = self.set_mode.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.05)
            if future.done() and future.result().mode_sent and self.state.mode == mode_name:
                return True
        return False

    def request_sim_ground_disarm(self) -> bool:
        """Force-disarm only when the explicit SITL flag and ground pose agree."""
        if (
            not self.state.armed
            or self.local_pose is None
            or self.local_pose.pose.position.z > 0.15
            or os.environ.get("ISAAC_SIM_FORCE_GROUND_DISARM") != "1"
        ):
            return not self.state.armed
        print("SITL landed-state is stuck; using simulation-only forced disarm...")
        request = CommandLong.Request()
        request.broadcast = False
        request.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        request.param1 = 0.0
        request.param2 = 21196.0
        future = self.command.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.state.armed and time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.1)
        return not self.state.armed

    def prepare_guided(self, *, takeoff: bool = True) -> None:
        print("Waiting for MAVROS connection and ArduPilot GPS/EKF initialization...")
        deadline = time.monotonic() + 90.0
        while (
            rclpy.ok()
            and not (self.state.connected and (self.home_received or self.gps_received))
            and time.monotonic() < deadline
        ):
            self.spin_once(timeout_sec=0.2)
        if not self.state.connected:
            raise RuntimeError("MAVROS did not connect to ArduPilot within 90 seconds.")
        if not (self.home_received or self.gps_received):
            raise RuntimeError(
                "ArduPilot has no valid home/GPS position. Check /drone_1/statustext/recv."
            )
        self.enable_local_position_stream()
        # GPS/home availability precedes a usable EKF local-position estimate.
        # Arming immediately in that window produces "Need Position Estimate".
        # Wait for the same local pose required by GUIDED control before sending
        # any arming request.
        position_deadline = time.monotonic() + 30.0
        while rclpy.ok() and self.local_pose is None and time.monotonic() < position_deadline:
            self.spin_once(timeout_sec=0.2)
        if self.local_pose is None:
            raise RuntimeError("ArduPilot did not produce a local EKF position estimate.")
        # A previous test or teleop session can leave the aircraft airborne.
        # LAND first and wait for the FCU to disarm; an in-air direct-disarm
        # request can be refused or race the subsequent GUIDED takeoff.
        if self.state.armed:
            print("Vehicle is armed from a previous session; landing before teleop...")
            self.request_land()
            land_deadline = time.monotonic() + 45.0
            while rclpy.ok() and self.state.armed and time.monotonic() < land_deadline:
                self.spin_once(timeout_sec=0.2)
            # SITL can occasionally remain armed in LAND after ground contact
            # because its landed-state message is missing. Only request a
            # normal disarm when local altitude confirms that it is on ground.
            if (
                self.state.armed
                and self.local_pose is not None
                and self.local_pose.pose.position.z <= 0.15
                and self.arm.wait_for_service(timeout_sec=3.0)
            ):
                print("Vehicle is on ground but still armed; requesting disarm...")
                disarm = CommandBool.Request()
                disarm.value = False
                disarm_future = self.arm.call_async(disarm)
                disarm_deadline = time.monotonic() + 5.0
                while (
                    rclpy.ok()
                    and self.state.armed
                    and time.monotonic() < disarm_deadline
                ):
                    self.spin_once(timeout_sec=0.1)
            self.request_sim_ground_disarm()
            if self.state.armed:
                raise RuntimeError("Vehicle did not disarm after LAND; teleop will not take over.")
            time.sleep(1.0)
        if not self.set_mode.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MAVROS set_mode service is unavailable")
        mode = SetMode.Request()
        mode.base_mode = 0
        mode.custom_mode = "GUIDED"
        mode_future = self.set_mode.call_async(mode)
        mode_deadline = time.monotonic() + 5.0
        while rclpy.ok() and not mode_future.done() and time.monotonic() < mode_deadline:
            self.spin_once(timeout_sec=0.1)
        if not mode_future.done() or not mode_future.result().mode_sent:
            raise RuntimeError("ArduPilot did not accept GUIDED mode.")
        time.sleep(1.0)
        if not self.state.armed:
            if not self.arm.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MAVROS arming service is unavailable")
            request = CommandBool.Request()
            request.value = True
            future = self.arm.call_async(request)
            last_arm_request = time.monotonic()
            deadline = time.monotonic() + 60.0
            while rclpy.ok() and not self.state.armed and time.monotonic() < deadline:
                # A failed pre-arm response can arrive immediately while EKF
                # health is still converging. Retrying on every queued callback
                # floods MAVROS with hundreds of service requests per second
                # and can segfault rclpy. One request per second is sufficient.
                if (
                    future.done()
                    and not future.result().success
                    and time.monotonic() - last_arm_request >= 1.0
                ):
                    future = self.arm.call_async(request)
                    last_arm_request = time.monotonic()
                self.spin_once(timeout_sec=0.2)
            if not self.state.armed:
                raise RuntimeError(
                    "ArduPilot did not arm within 60 seconds. Check /drone_1/statustext/recv."
                )
        if not takeoff:
            return
        self.takeoff_and_wait()

    def takeoff_and_wait(self, altitude: float = 1.5) -> None:
        """Request takeoff and wait for a stable airborne local pose.

        Fleet callers arm every vehicle first, synchronize at a process
        barrier, and then call this method so lift-off commands are issued in
        the same scheduler interval.
        """
        # Armed does not mean airborne.  A previous command can arm and then
        # exit before takeoff; treating that state as flight-ready lets the
        # first velocity key release the fixture without a valid takeoff
        # target.  Use MAVLink EXTENDED_SYS_STATE to decide explicitly.
        state_deadline = time.monotonic() + 3.0
        while (
            rclpy.ok()
            and not self.extended_state_received
            and time.monotonic() < state_deadline
        ):
            self.spin_once(timeout_sec=0.1)
        needs_takeoff = (
            not self.extended_state_received
            or self.extended_state.landed_state != ExtendedState.LANDED_STATE_IN_AIR
        )
        if needs_takeoff:
            if not self.takeoff.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("MAVROS takeoff service is unavailable")
            takeoff = CommandTOL.Request()
            takeoff.min_pitch = 0.0
            takeoff.yaw = 0.0
            takeoff.latitude = 0.0
            takeoff.longitude = 0.0
            takeoff.altitude = altitude
            self.last_takeoff_request_time = time.monotonic()
            takeoff_future = self.takeoff.call_async(takeoff)
            takeoff_deadline = time.monotonic() + 8.0
            while rclpy.ok() and not takeoff_future.done() and time.monotonic() < takeoff_deadline:
                self.spin_once(timeout_sec=0.1)
            if not takeoff_future.done() or not takeoff_future.result().success:
                raise RuntimeError(
                    f"ArduPilot did not accept the {altitude:.1f} m takeoff command."
                )
            print("Takeoff accepted. Waiting for climb before velocity teleoperation.")
            # Discard the pre-takeoff pose and keep spinning while climbing.
            # A plain sleep leaves local_pose frozen at ground level, causing
            # the first position-hold setpoint to command the aircraft down.
            self.local_pose = None
            climb_deadline = time.monotonic() + 15.0
            airborne_since = None
            climb_ready = False
            while rclpy.ok() and time.monotonic() < climb_deadline:
                self.spin_once(timeout_sec=0.1)
                if self.local_pose is not None and self.local_pose.pose.position.z >= 1.2:
                    airborne_since = airborne_since or time.monotonic()
                    if time.monotonic() - airborne_since >= 1.0:
                        climb_ready = True
                        break
                else:
                    airborne_since = None
            if not climb_ready:
                raise RuntimeError("Takeoff did not reach a stable airborne local pose.")
        else:
            print("Vehicle is already airborne; keeping its current altitude for teleoperation.")
        pose_deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.local_pose is None and time.monotonic() < pose_deadline:
            self.spin_once(timeout_sec=0.1)
        if self.local_pose is None:
            raise RuntimeError("No MAVROS local pose arrived for position hold.")


def read_pending_keys() -> list[str]:
    """Drain queued terminal input so the newest operator command wins."""
    keys = []
    while sys.stdin in select_inputs():
        chunk = os.read(sys.stdin.fileno(), 4096)
        if not chunk:
            break
        keys.extend(chunk.decode(errors="ignore").lower())
    return keys


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleoperate one ArduPilot vehicle")
    parser.add_argument(
        "--namespace",
        default=os.environ.get("TELEOP_NAMESPACE", "/drone_1"),
        help="MAVROS namespace, for example /tuning/env_0",
    )
    args = parser.parse_args()
    rclpy.init()
    safe_name = args.namespace.strip("/").replace("/", "_") or "root"
    node = Teleop(args.namespace, f"ardupilot_keyboard_teleop_{safe_name}")
    old_settings = termios.tcgetattr(sys.stdin)
    speed_level = DEFAULT_SPEED_LEVEL
    try:
        node.prepare_guided()
        tty.setcbreak(sys.stdin.fileno())
        print(f"\nTeleop active for {node.namespace} (hold a movement key; release it to hover):")
        print("  w/s = forward/back     a/d = left/right     r/f = up/down")
        print("  -/+ = slower/faster    1-5 = select speed   SPACE = brake/hold")
        print("  q = land and quit")
        print(speed_status(speed_level), flush=True)
        movement_key = None
        last_movement_at = 0.0
        command = (0.0, 0.0, 0.0)
        sent_command = (0.0, 0.0, 0.0)
        last_command_update = time.monotonic()
        braking = False
        braking_started_at = 0.0
        stopped_since = None
        # Capture a fixed hover point immediately after takeoff. A long Isaac
        # ground-truth trial showed less drift than either leaving the one-shot
        # takeoff target untouched or continuously commanding zero velocity.
        hold_target = node.capture_position_hold()
        while rclpy.ok():
            node.spin_once(timeout_sec=0.01)
            quit_requested = False
            for key in read_pending_keys():
                if key == "q":
                    quit_requested = True
                    break
                if key in MOVEMENT_DIRECTIONS:
                    movement_key = key
                    last_movement_at = time.monotonic()
                    command = velocity_for_key(movement_key, speed_level)
                    hold_target = None
                    braking = False
                    stopped_since = None
                elif key in "12345":
                    speed_level = int(key) - 1
                    command = velocity_for_key(movement_key, speed_level)
                    print(speed_status(speed_level), flush=True)
                elif key in ("+", "=", "]"):
                    speed_level = min(len(SPEED_PROFILES) - 1, speed_level + 1)
                    command = velocity_for_key(movement_key, speed_level)
                    print(speed_status(speed_level), flush=True)
                elif key in ("-", "_", "["):
                    speed_level = max(0, speed_level - 1)
                    command = velocity_for_key(movement_key, speed_level)
                    print(speed_status(speed_level), flush=True)
                elif key == " ":
                    movement_key = None
                    command = velocity_for_key(movement_key, speed_level)
                    node.send_active_brake()
                    sent_command = (0.0, 0.0, 0.0)
                    hold_target = None
                    braking = True
                    braking_started_at = time.monotonic()
                    stopped_since = None
                    print("STOP: cancelling velocity, then locking position", flush=True)
            if quit_requested:
                break
            if (
                movement_key is not None
                and time.monotonic() - last_movement_at >= MOVEMENT_TIMEOUT_SECONDS
            ):
                movement_key = None
                command = velocity_for_key(movement_key, speed_level)
                node.send_active_brake()
                sent_command = (0.0, 0.0, 0.0)
                hold_target = None
                braking = True
                braking_started_at = time.monotonic()
                stopped_since = None
            if braking:
                node.send_active_brake()
                now = time.monotonic()
                if node.is_nearly_stopped():
                    stopped_since = stopped_since or now
                else:
                    stopped_since = None
                stable_stop = (
                    stopped_since is not None
                    and now - stopped_since >= BRAKE_STABLE_SECONDS
                )
                if stable_stop or now - braking_started_at >= BRAKE_TIMEOUT_SECONDS:
                    hold_target = node.capture_position_hold()
                    braking = False
                    stopped_since = None
                    print("STOP: position locked", flush=True)
            elif hold_target is not None:
                node.send_position_hold(hold_target)
            elif movement_key is not None:
                now = time.monotonic()
                sent_command = slew_velocity(
                    sent_command, command, min(0.10, now - last_command_update)
                )
                last_command_update = now
                node.send_camera_velocity(*sent_command)
            else:
                last_command_update = time.monotonic()
            time.sleep(0.05)
    finally:
        node.request_land()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


def select_inputs():
    import select

    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    return readable


if __name__ == "__main__":
    main()
