#!/usr/bin/env python3
"""Run multiple ArduPilot-controlled Iris vehicles in one shared world."""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
import traceback
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-count", type=int, default=8)
    parser.add_argument("--spacing", type=float, default=30.0)
    parser.add_argument("--environment", choices=("empty", "warehouse"), default="empty")
    parser.add_argument("--warehouse-usd", type=Path)
    parser.add_argument("--json-port", type=int, default=9002)
    parser.add_argument("--port-stride", type=int, default=10)
    parser.add_argument("--run-seconds", type=float, default=240.0)
    parser.add_argument("--telemetry-rate", type=float, default=25.0)
    parser.add_argument("--physics-rate", type=float, default=100.0)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if not 1 <= args.env_count <= 16:
        parser.error("env-count must be between 1 and 16")
    minimum_spacing = 2.5 if args.environment == "warehouse" else 20.0
    if args.spacing < minimum_spacing:
        parser.error(
            f"spacing must be at least {minimum_spacing:g} m for {args.environment} mode"
        )
    if not 10.0 <= args.telemetry_rate <= 100.0:
        parser.error("telemetry-rate must be between 10 and 100 Hz")
    if not 60.0 <= args.physics_rate <= 240.0:
        parser.error("physics-rate must be between 60 and 240 Hz")
    return args


ARGS = parse_args()
APP_CONFIG = {
        "headless": ARGS.headless,
        "create_new_stage": False,
        "extra_args": [
            "--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge",
            "--/renderer/multiGpu/autoEnable=false",
            "--/renderer/multiGpu/enabled=false",
            # Allow several 100 Hz physics substeps per rendered GUI frame.
            # Without this, a nine-drone visible run tracks viewport FPS and
            # falls below real time even though CPU/GPU capacity is available.
            "--/app/player/useFixedTimeStepping=false",
            "--/persistent/simulation/minFrameRate=15",
        ],
    }
if isaac_root := os.environ.get("ISAAC_SIM_ROOT"):
    experience = Path(isaac_root) / "apps" / (
        "isaacsim.exp.base.python.kit" if ARGS.headless else "isaacsim.exp.full.kit"
    )
    if experience.is_file():
        APP_CONFIG["experience"] = str(experience)
APP = SimulationApp(APP_CONFIG)

# Isaac modules below require a running SimulationApp.
import omni.timeline
import omni.usd
import rclpy
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from nav_msgs.msg import Odometry
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics
from rclpy.qos import qos_profile_sensor_data

from create_room_quadcopter import ArduPilotJsonPhysicsBridge, _apply_rtx_settings


def find_body(stage, vehicle_root: str) -> str:
    preferred = []
    rigid = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(f"{vehicle_root}/"):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid.append(path)
        if any(token in path.lower() for token in ("/body", "base_link", "chassis")):
            preferred.append(path)
    candidates = preferred or rigid
    if not candidates:
        raise RuntimeError(f"No rigid drone body found below {vehicle_root}")
    return sorted(candidates, key=len)[0]


def configure_vehicle(stage, drone_root: str) -> tuple[str, float]:
    body_path = find_body(stage, drone_root)
    body = stage.GetPrimAtPath(body_path)
    mass = UsdPhysics.MassAPI.Apply(body)
    mass.CreateMassAttr().Set(1.5)
    mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.029125, 0.029125, 0.055225))
    mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    ).ComputeWorldBound(stage.GetPrimAtPath(drone_root)).ComputeAlignedRange()
    return body_path, max(0.0, -float(bounds.GetMin()[2]) + 0.003)


def grid_positions(count: int, spacing: float) -> list[tuple[float, float]]:
    """Return a centered square grid of spawn locations in one world."""
    columns = math.ceil(math.sqrt(count))
    rows = math.ceil(count / columns)
    positions = []
    for index in range(count):
        column = index % columns
        row = index // columns
        positions.append((
            (column - (columns - 1) / 2.0) * spacing,
            (row - (rows - 1) / 2.0) * spacing,
        ))
    return positions


class GroundTruthPublisher:
    def __init__(self, count: int) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("isaac_parallel_tuning_ground_truth")
        self.odom = [
            self.node.create_publisher(
                Odometry, f"/tuning/env_{index}/ground_truth", qos_profile_sensor_data
            )
            for index in range(count)
        ]

    def publish(self, index: int, sim_time: float, state: dict | None) -> None:
        if not state or not rclpy.ok():
            return
        sec = int(sim_time)
        nanosec = int((sim_time - sec) * 1.0e9)
        odom = Odometry()
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec
        odom.header.frame_id = f"tuning_env_{index}_world"
        odom.child_frame_id = f"tuning_env_{index}_base"
        p = state["position"]
        q = state["orientation"]
        v = state["linear_velocity"]
        w = state["angular_velocity"]
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = p
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = v
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = w
        self.odom[index].publish(odom)

    def close(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    bridge_objects = []
    publisher = None
    callback_id = None
    sender_thread = None
    sender_running = threading.Event()
    try:
        APP.update()
        import isaacsim.core.experimental.utils.app as app_utils

        app_utils.enable_extension("isaacsim.ros2.bridge")
        APP.update()
        rtx_diagnostics = _apply_rtx_settings(APP, ARGS.headless)
        create_new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        physics = UsdPhysics.Scene.Define(stage, "/physicsScene")
        physics.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics.CreateGravityMagnitudeAttr().Set(9.81)
        PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim()).CreateTimeStepsPerSecondAttr().Set(
            int(round(ARGS.physics_rate))
        )

        stage.DefinePrim("/World", "Xform")
        stage.DefinePrim("/World/Drones", "Xform")
        positions = grid_positions(ARGS.env_count, ARGS.spacing)
        if ARGS.environment == "warehouse":
            default_warehouse = (
                Path(__file__).resolve().parents[1]
                / "assets/simple_warehouse/warehouse.usd"
            )
            warehouse_path = (
                ARGS.warehouse_usd
                or Path(os.environ.get("WAREHOUSE_USD_PATH", str(default_warehouse)))
            ).expanduser().resolve()
            if not warehouse_path.is_file():
                raise FileNotFoundError(f"Local warehouse USD not found: {warehouse_path}")
            warehouse_stage = Usd.Stage.Open(str(warehouse_path))
            if warehouse_stage is None:
                raise RuntimeError(f"Could not compose warehouse USD: {warehouse_path}")
            mesh_count = sum(
                1 for prim in warehouse_stage.Traverse() if prim.IsA(UsdGeom.Mesh)
            )
            if mesh_count < 100:
                raise RuntimeError(
                    f"Warehouse asset is incomplete ({mesh_count} composed meshes): "
                    f"{warehouse_path}"
                )
            stage.DefinePrim("/World/Warehouse", "Xform").GetReferences().AddReference(
                str(warehouse_path)
            )
            print(
                f"Local warehouse validated: {mesh_count} composed meshes",
                flush=True,
            )
        else:
            floor_extent = max(
                max((abs(value) for position in positions for value in position), default=0.0)
                + 15.0,
                20.0,
            )
            floor = UsdGeom.Cube.Define(stage, "/World/Ground")
            floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
            floor.AddScaleOp().Set(Gf.Vec3f(floor_extent, floor_extent, 0.05))
            UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

        bundled_drone = Path(__file__).resolve().parents[1] / "assets/iris/iris.usd"
        drone_path = Path(os.environ.get("DRONE_USD_PATH", str(bundled_drone))).expanduser()
        if not drone_path.is_file():
            raise FileNotFoundError(f"Iris USD not found: {drone_path}")
        drone_roots = [f"/World/Drones/Drone_{index}" for index in range(ARGS.env_count)]
        for drone_root in drone_roots:
            add_reference_to_stage(str(drone_path), drone_root)
        APP.update()
        body_paths = []
        for drone_root, (x, y) in zip(drone_roots, positions):
            body_path, spawn_z = configure_vehicle(stage, drone_root)
            UsdGeom.XformCommonAPI(stage.GetPrimAtPath(drone_root)).SetTranslate(
                Gf.Vec3d(x, y, spawn_z)
            )
            body_paths.append(body_path)
        if not ARGS.headless:
            light = UsdLux.DistantLight.Define(stage, "/World/TuningLight")
            light.CreateIntensityAttr(1800.0)
            light.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))
            try:
                import numpy as np
                from isaacsim.core.utils.viewports import set_camera_view

                center_x = sum(position[0] for position in positions) / len(positions)
                center_y = sum(position[1] for position in positions) / len(positions)
                set_camera_view(
                    eye=np.array((center_x + 12.0, center_y - 14.0, 9.0)),
                    target=np.array((center_x, center_y, 1.5)),
                )
                print("Viewport framed on the complete drone fleet", flush=True)
            except Exception as error:
                print(f"Fleet viewport framing skipped: {error}", flush=True)
        APP.update()

        for index, body_path in enumerate(body_paths):
            bridge_objects.append(
                ArduPilotJsonPhysicsBridge(
                    stage,
                    body_path,
                    ARGS.json_port + index * ARGS.port_stride,
                    local_position_origin=True,
                    external_state_sender=True,
                )
            )
        sender_running.set()

        def shared_state_sender() -> None:
            period = 1.0 / float(os.environ.get("ARDUPILOT_JSON_SENSOR_RATE_HZ", "800"))
            next_send = time.monotonic()
            while sender_running.is_set():
                now = time.monotonic()
                if now < next_send:
                    time.sleep(next_send - now)
                elif now - next_send > 2.0 * period:
                    next_send = now
                for bridge in bridge_objects:
                    bridge.send_latest_state_once()
                next_send += period

        sender_thread = threading.Thread(
            target=shared_state_sender,
            name="parallel-ardupilot-json-state-sender",
            daemon=True,
        )
        sender_thread.start()
        publisher = GroundTruthPublisher(ARGS.env_count)
        sim_time = [0.0]
        telemetry_period = 1.0 / ARGS.telemetry_rate
        next_telemetry_time = [0.0]

        physics_failure = [None]

        def physics_step(dt: float, _context) -> None:
            if physics_failure[0] is not None:
                return
            sim_time[0] += float(dt)
            try:
                for index, bridge in enumerate(bridge_objects):
                    bridge.update(sim_time[0])
                if sim_time[0] + 1.0e-9 >= next_telemetry_time[0]:
                    for index, bridge in enumerate(bridge_objects):
                        publisher.publish(index, sim_time[0], bridge.ground_truth_snapshot())
                    next_telemetry_time[0] = sim_time[0] + telemetry_period
            except Exception as error:
                # Avoid logging the same callback failure on every physics step.
                physics_failure[0] = error
                traceback.print_exc()

        # The existing SingleRigidPrim bridge exchanges NumPy arrays and uses
        # the CPU PhysX backend. GPU mode requires a tensor-native bridge and
        # must not be enabled silently because it corrupts the flight run.
        SimulationManager.setup_simulation(dt=1.0 / ARGS.physics_rate, device="cpu")
        callback_id = SimulationManager.register_callback(
            physics_step, event=SimulationEvent.PHYSICS_PRE_STEP
        )
        timeline = omni.timeline.get_timeline_interface()
        app_utils.play()
        timeline.play()
        print(
            f"Parallel shared-world ready: {ARGS.env_count} drones in {ARGS.environment}, "
            f"JSON ports {ARGS.json_port}..{ARGS.json_port + (ARGS.env_count - 1) * ARGS.port_stride}, "
            f"telemetry {ARGS.telemetry_rate:.0f} Hz, "
            f"physics {ARGS.physics_rate:.0f} Hz, "
            f"viewport={rtx_diagnostics['viewport_width']}x{rtx_diagnostics['viewport_height']}",
            flush=True,
        )
        deadline = time.monotonic() + ARGS.run_seconds
        wall_start = time.monotonic()
        last_status = wall_start
        while APP.is_running() and time.monotonic() < deadline:
            APP.update()
            if physics_failure[0] is not None:
                raise RuntimeError("Parallel physics callback failed") from physics_failure[0]
            # ArduPilot's JSON loop is wall-clock paced. An unconstrained
            # headless Kit loop can advance PhysX several simulated seconds
            # per wall second, making every controller appear unstable. Keep
            # the 100 Hz plant at real time; the 800 Hz JSON sender safely
            # repeats the latest physical state between plant updates.
            # Pace from the callback's actual simulated time. Kit may choose a
            # 60 Hz update even when PhysX substeps are configured at 100 Hz;
            # assuming either rate causes a deterministic time-scale error.
            remaining = wall_start + sim_time[0] - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            now = time.monotonic()
            if now - last_status >= 5.0:
                wall_elapsed = max(1.0e-6, now - wall_start)
                print(
                    f"Fleet status: drones={ARGS.env_count}, "
                    f"RTF={sim_time[0] / wall_elapsed:.3f}, "
                    f"sim={sim_time[0]:.1f}s",
                    flush=True,
                )
                last_status = now
    finally:
        sender_running.clear()
        if sender_thread is not None:
            sender_thread.join(timeout=1.0)
        if callback_id is not None:
            SimulationManager.deregister_callback(callback_id)
        for bridge in bridge_objects:
            bridge.close()
        if publisher is not None:
            publisher.close()
        APP.close()


if __name__ == "__main__":
    main()
