#!/usr/bin/env python3
"""Create and run the single-warehouse, single-custom-drone scene."""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import threading
import time
import traceback
from pathlib import Path

from isaacsim import SimulationApp


# ---------------------------------------------------------------------------
# RTX 5080 renderer and performance configuration
# ---------------------------------------------------------------------------
# All values default to settings that are safe and performant on the NVIDIA RTX
# 5080 Laptop GPU. Every value can be overridden through the corresponding
# environment variable (see run_room.sh for the documented defaults).

def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        return int(float(raw))
    except (TypeError, ValueError):
        print(f"Invalid integer for {name}={raw!r}; using default {default}", flush=True)
        return default


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        print(f"Invalid float for {name}={raw!r}; using default {default}", flush=True)
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _set_setting(settings, method: str, path: str, value) -> None:
    """Set a Carb setting across Isaac Sim/Carb API naming variants."""
    setter = getattr(settings, method, None)
    if setter is None:
        snake_methods = {
            "setInt": "set_int",
            "setFloat": "set_float",
            "setBool": "set_bool",
            "setString": "set_string",
        }
        setter = getattr(settings, snake_methods.get(method, method), None)
    if setter is None:
        raise AttributeError(f"Carb settings API has no setter for {method}")
    setter(path, value)


def _resolve_gpu_index() -> int:
    """Pick the NVIDIA GPU index for Isaac Sim, preferring a discrete NVIDIA device.

    On the target laptop the RTX 5080 is Vulkan GPU 0 and the Intel iGPU is
    GPU 1. We verify the selection instead of hard-coding it so the launcher
    still works on a different machine or after a driver/PCIe reordering.
    """
    requested = os.environ.get("ISAAC_GPU_INDEX", "")
    if requested.strip():
        return _env_int("ISAAC_GPU_INDEX", 0)
    # Auto-detect: prefer the first Vulkan-discrete GPU, which is the NVIDIA
    # dGPU on Optimus laptops. If detection fails, leave it to the driver.
    try:
        import subprocess

        out = subprocess.check_output(
            ["vulkaninfo", "--summary"], stderr=subprocess.DEVNULL, text=True, timeout=5
        )
        for line in out.splitlines():
            lower = line.strip().lower()
            if lower.startswith("gpun") and "nvidia" in lower:
                return int(lower.split()[0].replace("gpu", ""))
    except Exception:
        pass
    return 0


def _rtx_renderer_config() -> dict:
    """Build the SimulationApp config dict entries for RTX rendering on the 5080."""
    config = {
        # Ray-traced lighting is the default Isaac Sim RTX mode; keeping it on
        # gives the visual quality the warehouse was authored for without the
        # path-tracer's per-frame cost.
        "renderer": os.environ.get("ISAAC_RENDERER", "RayTracedLighting"),
        "enable_livestream": False,
        "enable_mesh_input": False,
        "renderer_gpu_index": _resolve_gpu_index(),
    }
    return config


def _apply_rtx_settings(simulation_app, headless: bool) -> dict:
    """Apply RTX/DLSS/viewport settings after Kit has started.

    These are applied through Carb settings so they take effect without editing
    the kit experience file. Returns a diagnostics dict for the startup banner.
    """
    import carb

    settings = carb.settings.get_settings()
    gpu_index = _resolve_gpu_index()
    enable_rtx = _env_bool("ISAAC_ENABLE_RTX_OPTIMIZATION", True)
    enable_ray_tracing = _env_bool("ISAAC_ENABLE_RAY_TRACING", True)
    dlss_mode = _env_int("ISAAC_DLSS_MODE", 0)
    viewport_width = _env_int("ISAAC_VIEWPORT_WIDTH", 1280)
    viewport_height = _env_int("ISAAC_VIEWPORT_HEIGHT", 720)

    # Force single-GPU; the Intel iGPU must never be selected for rendering.
    _set_setting(settings, "setInt", "/renderer/multiGpu/enabled", 0)
    _set_setting(settings, "setInt", "/renderer/multiGpu/autoEnable", 0)
    _set_setting(settings, "setInt", "/renderer/multiGpu/gpuIndex", gpu_index)

    # Persistent caches so repeated launches do not recompile shaders or
    # re-stream warehouse textures. Keep them on local disk only.
    cache_dir = os.environ.get(
        "ISAAC_SHADER_CACHE_DIR",
        str(Path(__file__).resolve().parents[1] / "cache"),
    )
    try:
        Path(cache_dir).mkdir(parents=True, exist_ok=True)
        _set_setting(settings, "setString", "/persistent/isaac/Cache/Root", cache_dir)
    except OSError:
        print(f"Shader cache directory not writable: {cache_dir}", flush=True)

    if enable_rtx:
        # Use the real-time RTX 2.0 path (Hydra RTX) with path tracing off.
        _set_setting(settings, "setBool", "/persistent/rtx/modes/rt2/enabled", True)
        _set_setting(settings, "setBool", "/persistent/rtx/modes/rt/enabled", False)
        _set_setting(settings, "setBool", "/persistent/rtx/modes/pt/enabled", False)
        # Keep the expensive secondary effects configurable and off in the
        # default performance preset. Shadows retain enough depth perception
        # for teleoperation without burdening the sensor/control loop.
        _set_setting(
            settings,
            "setBool",
            "/rtx/reflections/enabled",
            _env_bool("ISAAC_RTX_REFLECTIONS", False),
        )
        _set_setting(
            settings,
            "setBool",
            "/rtx/shadows/enabled",
            _env_bool("ISAAC_RTX_SHADOWS", True),
        )
        _set_setting(
            settings,
            "setBool",
            "/rtx/ambientOcclusion/enabled",
            _env_bool("ISAAC_RTX_AMBIENT_OCCLUSION", False),
        )
        _set_setting(
            settings,
            "setBool",
            "/rtx/translucency/enabled",
            _env_bool("ISAAC_RTX_TRANSLUCENCY", False),
        )
        # Keep texture streaming resident so the warehouse does not re-stream.
        _set_setting(settings, "setBool", "/rtx-transient/resourcemanager/enableTextureStreaming", True)

        # DLSS upscaling for the interactive viewport. execMode 0=Performance,
        # 1=Balanced, 2=Quality, 3=Auto.
        dlss_mode = max(0, min(3, dlss_mode))
        _set_setting(settings, "setInt", "/rtx/post/dlss/execMode", dlss_mode)
        # DLSSG frame generation conflicts with synthetic data generation, so
        # it stays off (also set in the kit app). Keep it explicit.
        _set_setting(settings, "setBool", "/rtx-transient/dlssg/enabled", False)
        # Anti-aliasing op 4 = DLSS when available; fall back gracefully.
        if dlss_mode > 0 or not headless:
            _set_setting(settings, "setInt", "/rtx/post/aa/op", 4)
        # Motion blur and depth of field add per-frame cost without benefit for
        # a teleop/inspection viewport; disable both.
        _set_setting(settings, "setBool", "/rtx/post/motionblur/enabled", False)
        _set_setting(settings, "setBool", "/rtx/post/dof/enabled", False)
        _set_setting(
            settings,
            "setBool",
            "/rtx/indirectDiffuse/enabled",
            _env_bool("ISAAC_RTX_INDIRECT_DIFFUSE", False),
        )

    if not headless:
        # Set the interactive viewport resolution. The editor viewport uses
        # /app/renderer/resolution; sensor render products keep their own
        # resolution through the OmniGraph nodes.
        _set_setting(settings, "setInt", "/app/renderer/resolution/width", viewport_width)
        _set_setting(settings, "setInt", "/app/renderer/resolution/height", viewport_height)

    return {
        "gpu_index": gpu_index,
        "enable_rtx": enable_rtx,
        "enable_ray_tracing": enable_ray_tracing,
        "dlss_mode": dlss_mode if enable_rtx else 0,
        "viewport_width": viewport_width,
        "viewport_height": viewport_height,
        "headless": headless,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Create/save the stage without a GUI loop.")
    parser.add_argument("--no-save", action="store_true", help="Do not write the generated USD stage.")
    parser.add_argument("--start-paused", action="store_true", help="Leave the timeline paused after creating the scene.")
    parser.add_argument("--run-seconds", type=float, default=0.0, help="In headless mode, simulate for this many seconds before exiting.")
    parser.add_argument("--target-height", type=float, default=1.5, help="Hover height in metres.")
    parser.add_argument("--no-hover", action="store_true", help="Disable the phase-1 altitude hold controller.")
    parser.add_argument(
        "--flight-control",
        choices=("collective", "ardupilot"),
        default="collective",
        help="Use the fallback collective controller or ArduPilot SITL over JSON.",
    )
    parser.add_argument(
        "--sitl-json-port",
        type=int,
        default=9002,
        help="UDP port for ArduPilot's JSON physics interface.",
    )
    return parser.parse_args()


def add_static_cube(stage, path: str, location: tuple[float, float, float], scale: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.AddTranslateOp().Set(Gf.Vec3d(*location))
    cube.AddScaleOp().Set(Gf.Vec3f(*scale))
    # A cube without a rigid-body API is static; collision API makes it physical.
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def _set_display_color(prim, color: tuple[float, float, float]) -> None:
    from pxr import Gf, Sdf

    attr = prim.GetAttribute("primvars:displayColor")
    if not attr:
        attr = prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    attr.Set([Gf.Vec3f(*color)])


def add_sensor_visuals(stage, parent: str) -> None:
    """Add lightweight, non-colliding colored housings so mounts are visible."""
    from pxr import Gf, UsdGeom

    camera = UsdGeom.Cube.Define(stage, f"{parent}/CameraFront/VisualHousing")
    # Place the housing behind the lens (the camera looks along local -Z).
    camera.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.05))
    # Front camera: 30% of the previous dimensions (70% reduction).
    camera.AddScaleOp().Set(Gf.Vec3f(0.024, 0.0165, 0.0135))
    _set_display_color(camera.GetPrim(), (0.05, 0.25, 0.95))

    imu = UsdGeom.Cube.Define(stage, f"{parent}/Imu/VisualHousing")
    imu.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.04))
    imu.AddScaleOp().Set(Gf.Vec3f(0.035, 0.035, 0.02))
    _set_display_color(imu.GetPrim(), (0.85, 0.05, 0.75))

    tof = UsdGeom.Cone.Define(stage, f"{parent}/ToFDown/VisualHousing")
    tof.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    tof.AddScaleOp().Set(Gf.Vec3f(0.045, 0.045, 0.035))
    tof.CreateRadiusAttr(1.0)
    tof.CreateHeightAttr(2.0)
    _set_display_color(tof.GetPrim(), (0.95, 0.95, 0.05))


def add_ros_clock_graph() -> None:
    """Publish /clock through Isaac Sim's native ROS 2 graph node."""
    import omni.graph.core as og

    og.Controller.edit(
        {
            "graph_path": "/ActionGraph",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
            ],
        },
    )


def _sensor_target(path: str):
    import usdrt

    return [usdrt.Sdf.Path(path)]


def add_ros_camera_graph(graph_path: str, camera_path: str, frame_id: str, topic_name: str) -> None:
    """Create RGB and CameraInfo publishers for one camera prim."""
    import omni.graph.core as og

    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishImage.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishImage.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", _sensor_target(camera_path)),
                ("CreateRenderProduct.inputs:width", 640),
                ("CreateRenderProduct.inputs:height", 480),
                ("PublishImage.inputs:frameId", frame_id),
                ("PublishImage.inputs:topicName", topic_name),
                ("PublishImage.inputs:type", "rgb"),
                ("PublishCameraInfo.inputs:frameId", frame_id),
                ("PublishCameraInfo.inputs:topicName", f"{topic_name}/camera_info"),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)


def add_ros_imu_graph(imu_path: str) -> None:
    """Connect the Isaac IMU reader to a standard ROS 2 sensor_msgs/Imu publisher."""
    import omni.graph.core as og

    og.Controller.edit(
        {
            "graph_path": "/ROS2/Imu",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadImu", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "ReadImu.inputs:execIn"),
                ("ReadImu.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadImu.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadImu.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadImu.outputs:orientation", "PublishImu.inputs:orientation"),
                ("ReadImu.outputs:sensorTime", "PublishImu.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadImu.inputs:imuPrim", _sensor_target(imu_path)),
                ("ReadImu.inputs:readGravity", True),
                ("ReadImu.inputs:useLatestData", True),
                ("PublishImu.inputs:frameId", "imu_link"),
                ("PublishImu.inputs:topicName", "/imu/data"),
            ],
        },
    )


def create_lidar_runtime(lidar_path: str):
    """Create the supported runtime wrapper used by the ROS PointCloud2 bridge."""
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    lidar = Lidar(lidar_path, accumulate_outputs=None, tick_rate=None)
    # Isaac Sim 6.0.1-rc.7's registered ROS writer advertises a publisher but
    # does not emit this OmniLidar's frames. Read the native GMO result and
    # serialize it in RosSensorPublisher instead.
    sensor = LidarSensor(lidar, annotators=["generic-model-output"])
    return sensor


def add_ros_range_graph(raycast_path: str) -> None:
    """Publish the downward ToF ray as a one-beam LaserScan during Phase 1."""
    import omni.graph.core as og

    og.Controller.edit(
        {
            "graph_path": "/ROS2/ToFDown",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadRange", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                ("PublishRange", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "ReadRange.inputs:execIn"),
                ("ReadRange.outputs:execOut", "PublishRange.inputs:execIn"),
                ("ReadRange.outputs:depths", "PublishRange.inputs:linearDepthData"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadRange.inputs:raycastSensorPrim", _sensor_target(raycast_path)),
                ("PublishRange.inputs:frameId", "tof_down_link"),
                ("PublishRange.inputs:topicName", "/range/down"),
                ("PublishRange.inputs:horizontalFov", 0.0),
                ("PublishRange.inputs:horizontalResolution", 1.0),
                ("PublishRange.inputs:depthRange", [0.05, 12.0]),
            ],
        },
    )


def find_drone_sensor_parent(stage) -> str:
    """Prefer a rigid body/link inside iris.usd; fall back to the drone root."""
    from pxr import UsdPhysics

    preferred = []
    rigid_bodies = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith("/Drone/"):
            continue
        lowered = path.lower()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(path)
        if any(name in lowered for name in ("base_link", "body", "chassis", "frame")):
            preferred.append(path)
    for candidates in (preferred, rigid_bodies):
        if candidates:
            return sorted(candidates, key=len)[0]
    return "/Drone"


def localize_drone_materials(stage) -> int:
    """Replace the Iris asset's CDN material paths with vendored local files."""
    from urllib.parse import urlparse

    from pxr import Sdf

    local_root = Path(os.environ["OFFLINE_MATERIALS_ROOT"]).expanduser().resolve()
    remote_hosts = {"omniverse-content-production.s3-us-west-2.amazonaws.com"}
    localized = 0
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith("/Drone/"):
            continue
        for attribute in prim.GetAttributes():
            if attribute.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attribute.Get()
            if not isinstance(value, Sdf.AssetPath) or not value.path:
                continue
            parsed = urlparse(value.path)
            if parsed.hostname not in remote_hosts:
                continue
            local_path = local_root / parsed.path.lstrip("/")
            if not local_path.is_file():
                raise FileNotFoundError(
                    f"Offline drone material is missing for {value.path}: {local_path}"
                )
            attribute.Set(Sdf.AssetPath(str(local_path)))
            localized += 1
    print(f"Localized {localized} Iris material asset references", flush=True)
    return localized


def add_drone_sensors(stage) -> dict[str, str]:
    """Add one front camera, 3D lidar, IMU, and up/down ToF mounts."""
    from pxr import Gf, Sdf, UsdGeom

    parent = find_drone_sensor_parent(stage)
    print(f"Attaching sensors under drone link: {parent}", flush=True)

    front_camera_path = f"{parent}/CameraFront"
    front_camera = UsdGeom.Camera.Define(stage, front_camera_path)
    # Keep the lens 2 cm lower than before so its housing and optical axis stay
    # clear of the top-mounted LiDAR without changing camera orientation.
    UsdGeom.XformCommonAPI(front_camera).SetTranslate(Gf.Vec3d(0.16, 0.0, 0.02))
    # USD cameras look along local -Z; rotate that axis to drone-forward +X.
    # Roll-correct basis: local -Z -> +X and local +Y -> +Z.
    UsdGeom.Xformable(front_camera).AddOrientOp().Set(Gf.Quatf(0.5, 0.5, -0.5, -0.5))
    front_camera.GetPrim().GetAttribute("focalLength").Set(8.4)
    front_camera.GetPrim().GetAttribute("horizontalAperture").Set(20.955)
    front_camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))

    # Author the OmniLidar directly from the OS1 configuration bundled with
    # Isaac Sim. Referencing/flattening the visual OS1 asset creates a valid
    # prim but does not reliably produce GenericModelOutput in 6.0.1-rc.7.
    # The config-based path is NVIDIA's supported runtime API and is offline.
    from isaacsim.sensors.experimental.rtx import Lidar, RtxCamera

    # Apply OmniSensorAPI so Isaac 6 multi-tick renders this camera only at its
    # requested rate instead of on every viewport frame.
    front_camera.GetPrim().ApplyAPI("OmniSensorAPI")
    RtxCamera(
        front_camera_path,
        tick_rate=max(1.0, _env_float("ISAAC_CAMERA_FPS", 5.0)),
        reset_xform_op_properties=False,
    )

    lidar_path = f"{parent}/Lidar3D"
    lidar_usd_path = Path(os.environ["LIDAR_USD_PATH"]).expanduser().resolve()
    if not lidar_usd_path.is_file():
        raise FileNotFoundError(f"Offline OS1 LiDAR USD not found: {lidar_usd_path}")
    lidar = Lidar.create(
        path=lidar_path,
        usd_path=str(lidar_usd_path),
        variant="OS1_REV6_32ch10hz512res",
        attributes={
            "omni:sensor:Core:elementsCoordsType": "CARTESIAN",
            "omni:sensor:Core:outputFrameOfReference": "SENSOR",
        },
        accumulate_outputs=True,
        tick_rate=10.0,
        # Optical origin sits just above the housing/body instead of floating
        # above the airframe.
        translations=[[0.0, 0.0, 0.09]],
    )
    lidar_prim = lidar.prims[0]
    if lidar_prim is None or not lidar_prim.IsValid():
        raise RuntimeError("Could not create the Ouster OS1-style 3D RTX lidar")
    lidar_path = str(lidar_prim.GetPath())

    # Isaac Sim 6.0.1-rc.7 has a native crash in the experimental PhysX sensor
    # plugin when IMU/Raycast objects are created on this referenced aircraft.
    # Keep stable, correctly placed mount prims in the reusable USD; the live
    # physics-backed implementations will be enabled after that plugin is
    # upgraded or isolated.
    imu_path = f"{parent}/Imu"
    imu = UsdGeom.Xform.Define(stage, imu_path)
    imu_prim = imu.GetPrim()
    imu_prim.CreateAttribute("sensorType", Sdf.ValueTypeNames.Token).Set("imu")
    imu_prim.CreateAttribute("updateRateHz", Sdf.ValueTypeNames.Double).Set(200.0)

    tof_path = f"{parent}/ToFDown"
    tof = UsdGeom.Xform.Define(stage, tof_path)
    # Keep the ray origin below the body but above the floor at the 7 cm
    # ground-contact spawn pose.
    UsdGeom.XformCommonAPI(tof).SetTranslate(Gf.Vec3d(0.0, 0.0, -0.05))
    tof_prim = tof.GetPrim()
    tof_prim.CreateAttribute("sensorType", Sdf.ValueTypeNames.Token).Set("raycast_range")
    tof_prim.CreateAttribute("updateRateHz", Sdf.ValueTypeNames.Double).Set(30.0)
    tof_prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Double).Set(0.05)
    tof_prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Double).Set(12.0)
    tof_prim.CreateAttribute("direction", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, -1.0))

    tof_up_path = f"{parent}/ToFUp"
    tof_up = UsdGeom.Xform.Define(stage, tof_up_path)
    UsdGeom.XformCommonAPI(tof_up).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.12))
    tof_up_prim = tof_up.GetPrim()
    tof_up_prim.CreateAttribute("sensorType", Sdf.ValueTypeNames.Token).Set("raycast_range")
    tof_up_prim.CreateAttribute("updateRateHz", Sdf.ValueTypeNames.Double).Set(30.0)
    tof_up_prim.CreateAttribute("minRange", Sdf.ValueTypeNames.Double).Set(0.05)
    tof_up_prim.CreateAttribute("maxRange", Sdf.ValueTypeNames.Double).Set(12.0)
    tof_up_prim.CreateAttribute("direction", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, 1.0))
    add_sensor_visuals(stage, parent)

    return {
        "parent": parent,
        "front_camera": front_camera_path,
        "lidar": lidar_path,
        "imu": imu_path,
        "tof": tof_path,
        "tof_up": tof_up_path,
    }


def make_sensor_mounts_massless(stage, sensor_info: dict[str, str]) -> int:
    """Ensure attached sensors cannot alter the Iris rigid-body dynamics.

    Cameras and logical sensor mounts are normally plain USD transform/geometry
    prims. Referenced sensor assets can nevertheless gain physics APIs in a
    future Isaac asset revision. Strip those APIs from every sensor subtree so
    the explicitly authored Iris body mass and inertia remain authoritative.
    """
    from pxr import UsdPhysics

    sensor_roots = tuple(
        path for name, path in sensor_info.items() if name != "parent"
    )
    stripped = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not any(path == root or path.startswith(f"{root}/") for root in sensor_roots):
            continue
        for schema in (
            UsdPhysics.CollisionAPI,
            UsdPhysics.MassAPI,
            UsdPhysics.RigidBodyAPI,
        ):
            if prim.HasAPI(schema):
                prim.RemoveAPI(schema)
                stripped += 1
    print(
        "Sensor dynamics isolated: "
        f"removed {stripped} sensor physics APIs; sensors add no mass or inertia",
        flush=True,
    )
    return stripped


def enable_ros_bridge() -> None:
    """Enable RTX lidar and ROS 2 support before creating sensor graphs."""
    import isaacsim.core.experimental.utils.app as app_utils

    app_utils.enable_extension("isaacsim.sensors.experimental.rtx")
    app_utils.enable_extension("isaacsim.sensors.rtx.nodes")
    app_utils.enable_extension("isaacsim.ros2.bridge")


class RenderCameraSource:
    """Read RGB pixels from USD cameras through Isaac Sim's render products."""

    def __init__(self, front_path: str, width: int, height: int) -> None:
        import omni.replicator.core as rep

        self._rep = rep
        self._width = width
        self._height = height
        self._annotators = {}
        self._render_products = {}
        self._latest = {}
        for name, path in (("front", front_path),):
            render_product = rep.create.render_product(
                path,
                (width, height),
                name=f"RoomQuadcopter_{name}_rgb",
                force_new=True,
            )
            annotator = rep.annotators.get("rgb")
            annotator.attach(render_product)
            self._render_products[name] = render_product
            self._annotators[name] = annotator

    def capture(self) -> None:
        # The render product is driven by the camera's OmniSensorAPI tick rate
        # during the normal SimulationApp update. Read the completed frame;
        # never invoke Replicator's blocking orchestrator.step() here.
        for name, annotator in self._annotators.items():
            data = annotator.get_data(device="cpu", do_array_copy=True)
            if data is None:
                continue
            if hasattr(data, "shape") and len(data.shape) == 3 and data.shape[2] >= 3:
                self._latest[name] = data[:, :, :3].tobytes()

    def get(self, name: str) -> bytes | None:
        return self._latest.get(name)


class PropellerAltitudeController:
    """Four-propeller collective-thrust altitude controller.

    This is intentionally a controllable motor model rather than a full
    aerodynamic model: each rotor gets a thrust force at its actual world
    position, and its USD revolute joint is driven so the propeller visibly
    spins.  The next control milestone can replace the equal motor commands
    with a roll/pitch/yaw mixer.
    """

    def __init__(self, stage, body_path: str, target_height: float) -> None:
        import omni.physx
        from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics, UsdUtils

        self._stage = stage
        self._body_path = body_path
        self._target = target_height
        self._query = omni.physx.get_physx_simulation_interface()
        self._stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self._body_id = PhysicsSchemaTools.sdfPathToInt(body_path)
        self._last_height = None
        self._mass = 1.0
        prim = stage.GetPrimAtPath(body_path)
        if prim.IsValid():
            mass_attr = prim.GetAttribute("physics:mass")
            if mass_attr and mass_attr.HasAuthoredValue():
                self._mass = max(0.1, float(mass_attr.Get()))
        self._Usd = Usd
        self._Gf = Gf
        self._UsdGeom = UsdGeom
        self._last_time = None
        self._rotors = []
        # Calibrated so the 1.5 kg Iris hovers near ArduCopter's 0.5 throttle.
        self._thrust_coefficient = 1.31e-4  # N/(rad/s)^2
        self._max_motor_omega = 2.0 * 3.141592653589793 * 3200.0 / 60.0
        # Iris/PX4 rotor order matches ArduPilot QUAD/X motor order:
        # front-right, rear-left, front-left, rear-right.
        self._rotor_spin_signs = (1.0, 1.0, -1.0, -1.0)

        # The downloaded iris asset has this hierarchy beneath its default
        # prim /vehicle. Resolve it instead of hard-coding a guessed body path.
        vehicle_root = body_path.rsplit("/", 1)[0]
        for index in range(4):
            rotor = None
            joint = None
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if not path.startswith(f"{vehicle_root}/"):
                    continue
                if path.endswith(f"/rotor{index}") and prim.GetTypeName() == "Xform":
                    rotor = prim
                if path.endswith(f"/joint{index}") and prim.GetTypeName() == "PhysicsRevoluteJoint":
                    joint = prim
            if rotor is None:
                continue
            drive = None
            if joint is not None:
                try:
                    drive = UsdPhysics.DriveAPI.Apply(joint, UsdPhysics.Tokens.angular)
                    drive.CreateTypeAttr("force")
                    drive.CreateDampingAttr(2.0e4)
                    drive.CreateStiffnessAttr(0.0)
                    drive.CreateMaxForceAttr(1.0e6)
                except Exception:
                    drive = None
                    traceback.print_exc()
            self._rotors.append((rotor, drive))

        if len(self._rotors) != 4:
            raise RuntimeError(f"Expected 4 iris rotors, found {len(self._rotors)}")
        print(
            "Propeller thrust enabled on: "
            + ", ".join(str(rotor.GetPath()) for rotor, _ in self._rotors),
            flush=True,
        )

    def _set_motor_speed(self, drive, omega: float) -> None:
        if drive is None:
            return
        # USD angular drive target velocity is expressed in degrees/second.
        drive.CreateTargetVelocityAttr(float(omega * 180.0 / 3.141592653589793))

    def update(self, sim_time: float) -> None:
        from carb import Float3
        from pxr import Gf

        prim = self._stage.GetPrimAtPath(self._body_path)
        if not prim.IsValid():
            return
        world = self._UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(self._Usd.TimeCode.Default())
        position = world.ExtractTranslation()
        orientation = world.ExtractRotation().GetQuaternion()
        height = float(position[2])
        if self._last_height is None or self._last_time is None:
            vertical_velocity = 0.0
        else:
            dt = max(1.0 / 240.0, sim_time - self._last_time)
            vertical_velocity = (height - self._last_height) / dt
        self._last_height = height
        self._last_time = sim_time

        # PD acceleration command plus gravity compensation becomes collective
        # thrust, then is distributed across the four actual propellers.
        kp = 8.0
        kd = 4.5
        commanded_accel = 9.81 + kp * (self._target - height) - kd * vertical_velocity
        total_thrust = max(0.0, min(4.0 * 6.0, self._mass * commanded_accel))
        per_motor_thrust = total_thrust / 4.0
        omega = min(
            self._max_motor_omega,
            (per_motor_thrust / self._thrust_coefficient) ** 0.5
            if per_motor_thrust > 0.0 else 0.0,
        )
        thrust_local = Gf.Vec3d(0.0, 0.0, 1.0)
        thrust_world = world.TransformDir(thrust_local).GetNormalized()
        for index, (rotor, drive) in enumerate(self._rotors):
            self._set_motor_speed(drive, self._rotor_spin_signs[index] * omega)
            rotor_world = self._UsdGeom.Xformable(rotor).ComputeLocalToWorldTransform(
                self._Usd.TimeCode.Default()
            )
            rotor_position = rotor_world.ExtractTranslation()
            self._query.apply_force_at_pos(
                self._stage_id,
                self._body_id,
                Float3(
                    float(thrust_world[0] * per_motor_thrust),
                    float(thrust_world[1] * per_motor_thrust),
                    float(thrust_world[2] * per_motor_thrust),
                ),
                Float3(float(rotor_position[0]), float(rotor_position[1]), float(rotor_position[2])),
                "Force",
            )
        # Keep the body approximately level while validating collective thrust.
        # This is the temporary attitude stabilizer; replace it with motor
        # differential mixing after the altitude milestone is confirmed.
        sign = 1.0 if float(orientation.GetReal()) >= 0.0 else -1.0
        imaginary = orientation.GetImaginary()
        self._query.apply_torque(
            self._stage_id,
            self._body_id,
            Float3(
                -18.0 * sign * float(imaginary[0]),
                -18.0 * sign * float(imaginary[1]),
                -6.0 * sign * float(imaginary[2]),
            ),
        )


# Compatibility name for any external launchers importing the old class.
AltitudeHoldController = PropellerAltitudeController


class ArduPilotJsonPhysicsBridge(PropellerAltitudeController):
    """Connect Isaac vehicle physics to ArduPilot SITL's JSON backend.

    ArduPilot owns the flight-control loop and sends four PWM outputs. Isaac
    converts those motor outputs into thrust and returns the simulated IMU,
    pose, velocity, and downward range measurement using ArduPilot's official
    UDP JSON physics protocol.
    """

    _OUTPUT_HEADER = struct.Struct("<HHI16H")
    _OUTPUT_MAGIC = 18458

    def __init__(
        self,
        stage,
        body_path: str,
        port: int,
        *,
        local_position_origin: bool = False,
        external_state_sender: bool = False,
    ) -> None:
        super().__init__(stage, body_path, target_height=0.0)
        from isaacsim.core.prims import SingleRigidPrim

        self._body_prim = SingleRigidPrim(
            prim_path=body_path,
            name=f"ardupilot_drone_body_{port}",
            reset_xform_properties=False,
        )
        self._body_prim_initialized = False
        body_world = self._UsdGeom.Xformable(stage.GetPrimAtPath(body_path)).ComputeLocalToWorldTransform(
            self._Usd.TimeCode.Default()
        )
        self._ground_hold_position = tuple(float(v) for v in body_world.ExtractTranslation())
        self._position_origin = (
            self._ground_hold_position if local_position_origin else (0.0, 0.0, 0.0)
        )
        self._ground_hold_orientation = self._quaternion_components(body_world.ExtractRotationQuat())
        body_world_inverse = body_world.GetInverse()
        self._rotor_offsets = []
        for rotor, _drive in self._rotors:
            rotor_world = self._UsdGeom.Xformable(rotor).ComputeLocalToWorldTransform(
                self._Usd.TimeCode.Default()
            )
            self._rotor_offsets.append(
                tuple(float(v) for v in body_world_inverse.Transform(rotor_world.ExtractTranslation()))
            )
        print(f"Iris rotor offsets (body frame): {self._rotor_offsets}", flush=True)
        self._previous_velocity_ned = (0.0, 0.0, 0.0)
        self._previous_state_time = None
        self._filtered_acceleration_body = (0.0, 0.0, -9.80665)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # This bridge is the sole owner of the JSON physics port. Allowing
        # SO_REUSEADDR lets two Isaac processes bind UDP 9002 and interleave
        # incompatible timestamp streams, which ArduPilot reports as repeated
        # physics resets and ultimately as a zero-rate gyro pre-arm failure.
        self._socket.bind(("0.0.0.0", port))
        self._socket.setblocking(False)
        # JSON SITL returns PWM packets from the socket on which it expects
        # physics state.  Wait for that first packet, then reply to its source
        # address.  Do not seed this with the local listening port: doing so
        # sends Isaac state back to itself rather than to SITL.
        self._sensor_endpoint = None
        self._pwm = [1000, 1000, 1000, 1000]
        # Model motor/ESC spool dynamics instead of applying every 400 Hz PWM
        # correction as an instantaneous rotor-speed step.  The official
        # ArduPilot external multicopter example integrates motor RPM between
        # physics steps; this first-order equivalent removes the unphysical
        # high-frequency attitude excitation while retaining full authority.
        self._motor_commands = [0.0, 0.0, 0.0, 0.0]
        self._motor_time_constant = 0.060
        self._visual_rotors_enabled = not _env_bool("ISAAC_DISABLE_ROTOR_VISUALS", False)
        self._lightweight_rotor_visuals = _env_bool(
            "ISAAC_LIGHTWEIGHT_ROTOR_VISUALS", False
        )
        self._rotor_visual_ops = []
        self._rotor_visual_angles = [0.0] * 4
        self._last_rotor_visual_time = None
        if self._lightweight_rotor_visuals:
            self._setup_lightweight_rotor_visuals()
        self._last_motor_time = None
        self._last_frame = None
        self._physics_time = 0.0
        # Copter's rate controller runs at 400 Hz.  The JSON backend blocks on
        # every physics response, so a slower sender silently slows the flight
        # controller as well and makes the stock gains oscillatory.
        self._sensor_rate_hz = float(
            os.environ.get("ARDUPILOT_JSON_SENSOR_RATE_HZ", "800.0")
        )
        self._launch_released = False
        self._last_log_time = -1.0
        self._state_lock = threading.Lock()
        self._latest_state = None
        self._latest_state_payload_tail = None
        self._ground_truth_snapshot = None
        self._sender_running = True
        self._sender_thread = None
        if not external_state_sender:
            self._sender_thread = threading.Thread(
                target=self._state_sender_loop,
                name="ardupilot-json-state-sender",
                daemon=True,
            )
            self._sender_thread.start()
        print(f"ArduPilot JSON physics bridge listening on UDP {port}", flush=True)

    def _setup_lightweight_rotor_visuals(self) -> None:
        """Create body-attached animated blades without simulated rotor joints."""
        root = f"{self._body_path}/LightweightRotorVisuals"
        self._UsdGeom.Xform.Define(self._stage, root)
        for index, (rotor, _drive) in enumerate(self._rotors):
            # Keep the authored motor/rotor assembly attached through its USD
            # joint, but hide its static blade. The replacement is a plain
            # body child and therefore adds no PhysX bodies or constraints.
            self._UsdGeom.Imageable(rotor).MakeInvisible()
            mount = self._UsdGeom.Xform.Define(self._stage, f"{root}/Rotor_{index}")
            mount.AddTranslateOp().Set(self._Gf.Vec3d(*self._rotor_offsets[index]))
            spin = mount.AddRotateZOp(opSuffix="visualSpin")
            blade = self._UsdGeom.Cube.Define(
                self._stage, f"{root}/Rotor_{index}/Blade"
            )
            blade.AddScaleOp().Set(self._Gf.Vec3f(0.115, 0.012, 0.0025))
            _set_display_color(blade.GetPrim(), (0.12, 0.22, 0.28))
            tip = self._UsdGeom.Cube.Define(
                self._stage, f"{root}/Rotor_{index}/TipMarker"
            )
            tip.AddTranslateOp().Set(self._Gf.Vec3d(0.095, 0.0, 0.003))
            tip.AddScaleOp().Set(self._Gf.Vec3f(0.018, 0.015, 0.003))
            _set_display_color(tip.GetPrim(), (0.05, 0.45, 1.0))
            self._rotor_visual_ops.append(spin)
        print("Lightweight body-attached rotor animation enabled", flush=True)

    def _update_lightweight_rotor_visuals(self, sim_time: float, commands) -> None:
        if not self._rotor_visual_ops:
            return
        if self._last_rotor_visual_time is None:
            self._last_rotor_visual_time = sim_time
            return
        elapsed = sim_time - self._last_rotor_visual_time
        if elapsed < 1.0 / 20.0:
            return
        self._last_rotor_visual_time = sim_time
        for index, (operation, command) in enumerate(
            zip(self._rotor_visual_ops, commands)
        ):
            self._rotor_visual_angles[index] = (
                self._rotor_visual_angles[index]
                + self._rotor_spin_signs[index] * float(command) * 720.0 * elapsed
            ) % 360.0
            operation.Set(self._rotor_visual_angles[index])

    def _receive_motor_outputs(self) -> None:
        while True:
            try:
                packet, peer = self._socket.recvfrom(2048)
            except BlockingIOError:
                return
            if len(packet) < self._OUTPUT_HEADER.size:
                continue
            magic, rate, frame_count, *pwm = self._OUTPUT_HEADER.unpack_from(packet)
            if magic != self._OUTPUT_MAGIC:
                continue
            self._sensor_endpoint = peer
            self._last_frame = frame_count
            self._pwm = list(pwm[:4])

    @staticmethod
    def _quaternion_components(value) -> tuple[float, float, float, float]:
        """Return scalar-first components from NumPy, Gf, or tensor quaternions."""
        if hasattr(value, "GetReal") and hasattr(value, "GetImaginary"):
            imaginary = value.GetImaginary()
            return (
                float(value.GetReal()),
                float(imaginary[0]),
                float(imaginary[1]),
                float(imaginary[2]),
            )
        try:
            return tuple(float(value[index]) for index in range(4))
        except (TypeError, KeyError):
            components = []
            for name in ("w", "x", "y", "z"):
                component = getattr(value, name)
                components.append(float(component() if callable(component) else component))
            return tuple(components)

    def _state_payload(self, position, orientation, velocity, gyro, acceleration) -> bytes:
        # Send the full measured NED pose and FRD-frame IMU state expected by
        # ArduPilot's JSON backend.
        state = {
            # JSON SITL requires a strictly increasing physics timestamp.
            # Advance it by the rate requested in each PWM output packet.
            "timestamp": self._physics_time,
            "imu": {
                "gyro": list(gyro),
                "accel_body": list(acceleration),
            },
            "position": list(position),
            "velocity": list(velocity),
            "quaternion": list(orientation),
            "rng_1": max(0.05, -position[2]),
            # Match ArduPilot's JSON master example. Isaac's rendered wall
            # rate can differ from its fixed physics rate, so SITL must not
            # pace its scheduler from packet arrival timing.
            "no_time_sync": 1,
        }
        # ArduPilot's lightweight parser advances exactly past the JSON colon;
        # omit optional whitespace so arrays begin where that parser expects.
        # ArduPilot's JSON parser uses newlines as record separators and looks
        # for a complete record between two separators.  Both the leading and
        # trailing newline are required; an ending newline alone lets the
        # first packet through but stalls every packet after it.
        return ("\n" + json.dumps(state, separators=(",", ":")) + "\n").encode("ascii")

    @staticmethod
    def _state_payload_tail(position, orientation, velocity, gyro, acceleration) -> bytes:
        """Serialize the 100 Hz state once; the 800 Hz sender adds timestamps.

        Between PhysX updates ArduPilot must receive repeated sensor samples
        with increasing timestamps. Re-encoding every vector for every repeat
        dominated the shared-world Python bridge at larger fleet sizes.
        """
        state_without_timestamp = {
            "imu": {"gyro": list(gyro), "accel_body": list(acceleration)},
            "position": list(position),
            "velocity": list(velocity),
            "quaternion": list(orientation),
            "rng_1": max(0.05, -position[2]),
            "no_time_sync": 1,
        }
        encoded = json.dumps(state_without_timestamp, separators=(",", ":"))
        return ("," + encoded[1:] + "\n").encode("ascii")

    def _state_sender_loop(self) -> None:
        """Publish JSON sensor state at the rate required by Copter's EKF."""
        period = 1.0 / self._sensor_rate_hz
        next_send = time.monotonic()
        while self._sender_running:
            now = time.monotonic()
            if now < next_send:
                time.sleep(next_send - now)
                now = time.monotonic()
            elif now - next_send > 2.0 * period:
                # Do not replay a backlog of stale PhysX snapshots after an
                # RTX/OS scheduling hitch. Bursting old states while advancing
                # JSON timestamps corrupts EKF acceleration/position timing.
                # SITL's no_time_sync mode safely tolerates a slower interval.
                next_send = now
            self.send_latest_state_once()
            next_send += period

    def send_latest_state_once(self) -> None:
        """Send one immutable sensor snapshot; usable by a shared scheduler."""
        with self._state_lock:
            latest_state = self._latest_state
            payload_tail = self._latest_state_payload_tail
            endpoint = self._sensor_endpoint
        if latest_state is None or payload_tail is None or endpoint is None or self._socket is None:
            return
        try:
            timestamp = format(self._physics_time, ".9f").encode("ascii")
            self._socket.sendto(b'\n{"timestamp":' + timestamp + payload_tail, endpoint)
            self._physics_time += 1.0 / self._sensor_rate_hz
        except OSError as error:
            print(f"ArduPilot JSON state send failed: {error}", flush=True)

    def update(self, sim_time: float) -> None:
        from carb import Float3
        import numpy as np

        self._receive_motor_outputs()
        if not self._body_prim_initialized:
            self._body_prim.initialize()
            self._body_prim_initialized = True
        prim = self._stage.GetPrimAtPath(self._body_path)
        if not prim.IsValid():
            return
        target_commands = [
            max(0.0, min(1.0, (float(value) - 1100.0) / 800.0))
            for value in self._pwm[:4]
        ]
        motor_dt = (
            1.0 / 240.0
            if self._last_motor_time is None
            else max(1.0 / 1000.0, min(0.05, sim_time - self._last_motor_time))
        )
        self._last_motor_time = sim_time
        motor_alpha = motor_dt / (self._motor_time_constant + motor_dt)
        self._motor_commands = [
            current + motor_alpha * (target - current)
            for current, target in zip(self._motor_commands, target_commands)
        ]
        raw_commands = self._motor_commands
        self._update_lightweight_rotor_visuals(sim_time, raw_commands)
        # The visual Iris has no usable landing-gear collision. Keep it at its
        # computed ground-contact pose until commanded propeller thrust can
        # support its weight. This prevents estimator corruption during spool
        # up without suspending the vehicle above the floor.
        commanded_support_thrust = sum(
            self._thrust_coefficient * (command * self._max_motor_omega) ** 2
            for command in raw_commands
        )
        if not self._launch_released and commanded_support_thrust >= 0.95 * self._mass * 9.80665:
            self._launch_released = True
            print("Ground hold released: propellers reached weight-supporting thrust", flush=True)
        if not self._launch_released:
            body_position = np.array(self._ground_hold_position, dtype=np.float32)
            body_orientation = self._ground_hold_orientation
            self._body_prim.set_world_pose(
                position=body_position,
                orientation=np.array(body_orientation, dtype=np.float32),
            )
            self._body_prim.set_linear_velocity(np.zeros(3, dtype=np.float32))
            self._body_prim.set_angular_velocity(np.zeros(3, dtype=np.float32))
        else:
            body_position, live_orientation = self._body_prim.get_world_pose()
            body_orientation = self._quaternion_components(live_orientation)
        linear_velocity = self._body_prim.get_linear_velocity()
        angular_velocity = self._body_prim.get_angular_velocity()
        position = body_position
        orientation = body_orientation
        qw, qx, qy, qz = orientation
        # Body-to-Isaac-world rotation matrix (scalar-first quaternion).
        rotation = (
            (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)),
            (2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx)),
            (2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy)),
        )
        thrust_world = (rotation[0][2], rotation[1][2], rotation[2][2])
        yaw_reaction_torque = 0.0
        total_thrust = 0.0
        rotor_moment_body = [0.0, 0.0, 0.0]
        collective_command = sum(raw_commands) / 4.0
        # The authored Iris inertia now matches the plant, so preserve the
        # flight controller's full differential motor authority.
        moment_authority = 1.0
        for index, (rotor, drive) in enumerate(self._rotors):
            # Preserve ArduPilot's collective command while calibrating the
            # differential motor moment to this USD body's inertia.
            command = collective_command + moment_authority * (
                raw_commands[index] - collective_command
            )
            command = max(0.0, min(1.0, command))
            omega = command * self._max_motor_omega
            if self._visual_rotors_enabled:
                self._set_motor_speed(drive, self._rotor_spin_signs[index] * omega)
            # Motor PWM is now the source of thrust. There is no altitude PID
            # and no independent hover force in this mode.
            thrust = self._thrust_coefficient * omega * omega
            total_thrust += thrust
            # Airframe reaction torque is opposite the propeller spin.
            yaw_reaction_torque -= self._rotor_spin_signs[index] * thrust * 0.02
            offset = self._rotor_offsets[index]
            # Sum r x F in the Iris FLU body frame.  Applying the equivalent
            # wrench explicitly avoids PhysX point-force ambiguity while each
            # term still comes from its real rotor position and thrust.
            rotor_moment_body[0] += float(offset[1]) * thrust
            rotor_moment_body[1] -= float(offset[0]) * thrust
        rotor_moment_body[2] = yaw_reaction_torque
        # ArduPilot's reference external multicopter plant applies quadratic
        # rotational drag in the body frame with coefficient 0.2.  Applying
        # it in that same frame is important once the aircraft tilts: world-
        # axis damping can couple roll, pitch, and yaw and destabilize EKF yaw.
        angular_velocity_body = tuple(
            sum(rotation[row][column] * float(angular_velocity[row]) for row in range(3))
            for column in range(3)
        )
        for axis in range(3):
            rotor_moment_body[axis] -= (
                0.2 * angular_velocity_body[axis] * abs(angular_velocity_body[axis])
            )
        self._query.apply_force_at_pos(
            self._stage_id,
            self._body_id,
            Float3(
                float(thrust_world[0] * total_thrust),
                float(thrust_world[1] * total_thrust),
                float(thrust_world[2] * total_thrust),
            ),
            Float3(float(position[0]), float(position[1]), float(position[2])),
            "Force",
        )
        rotor_moment_world = tuple(
            sum(rotation[row][column] * rotor_moment_body[column] for column in range(3))
            for row in range(3)
        )
        self._query.apply_torque(
            self._stage_id,
            self._body_id,
            Float3(
                float(rotor_moment_world[0]),
                float(rotor_moment_world[1]),
                float(rotor_moment_world[2]),
            ),
        )
        # Basic body drag prevents an ideal vacuum-like plant.  ArduPilot's
        # own multicopter model includes both translational and rotational
        # damping; omitting them leaves XY and attitude oscillations undamped.
        drag_coefficient = 0.12
        self._query.apply_force_at_pos(
            self._stage_id,
            self._body_id,
            Float3(
                float(-drag_coefficient * linear_velocity[0] * abs(linear_velocity[0])),
                float(-drag_coefficient * linear_velocity[1] * abs(linear_velocity[1])),
                float(-drag_coefficient * linear_velocity[2] * abs(linear_velocity[2])),
            ),
            Float3(float(position[0]), float(position[1]), float(position[2])),
            "Force",
        )
        # Isaac uses Z-up world coordinates; ArduPilot JSON uses NED and FRD.
        relative_position = tuple(
            float(position[index]) - self._position_origin[index] for index in range(3)
        )
        position_ned = (
            relative_position[0],
            -relative_position[1],
            -relative_position[2],
        )
        velocity_ned = (
            float(linear_velocity[0]),
            -float(linear_velocity[1]),
            -float(linear_velocity[2]),
        )
        quaternion_ned = (qw, qx, -qy, -qz)
        # Rebuild body-to-NED after the world-axis conversion.
        nw, nx, ny, nz = quaternion_ned
        body_to_ned = (
            (1.0 - 2.0 * (ny * ny + nz * nz), 2.0 * (nx * ny - nw * nz), 2.0 * (nx * nz + nw * ny)),
            (2.0 * (nx * ny + nw * nz), 1.0 - 2.0 * (nx * nx + nz * nz), 2.0 * (ny * nz - nw * nx)),
            (2.0 * (nx * nz - nw * ny), 2.0 * (ny * nz + nw * nx), 1.0 - 2.0 * (nx * nx + ny * ny)),
        )
        angular_ned = (
            float(angular_velocity[0]),
            -float(angular_velocity[1]),
            -float(angular_velocity[2]),
        )
        gyro_body = tuple(
            sum(body_to_ned[row][column] * angular_ned[row] for row in range(3))
            for column in range(3)
        )
        if self._previous_state_time is None:
            acceleration_ned = (0.0, 0.0, -9.80665)
        else:
            dt = max(1.0 / 240.0, sim_time - self._previous_state_time)
            acceleration_ned = (
                (velocity_ned[0] - self._previous_velocity_ned[0]) / dt,
                (velocity_ned[1] - self._previous_velocity_ned[1]) / dt,
                (velocity_ned[2] - self._previous_velocity_ned[2]) / dt - 9.80665,
            )
        acceleration_body = tuple(
            sum(body_to_ned[row][column] * acceleration_ned[row] for row in range(3))
            for column in range(3)
        )
        gyro_body = tuple(max(-10.0, min(10.0, value)) for value in gyro_body)
        acceleration_body = tuple(max(-30.0, min(30.0, value)) for value in acceleration_body)
        acceleration_body = tuple(
            0.15 * acceleration_body[index] + 0.85 * self._filtered_acceleration_body[index]
            for index in range(3)
        )
        self._filtered_acceleration_body = acceleration_body
        self._previous_velocity_ned = velocity_ned
        self._previous_state_time = sim_time
        self._ground_truth_snapshot = {
            "position": relative_position,
            "orientation": tuple(float(value) for value in orientation),
            "linear_velocity": tuple(float(value) for value in linear_velocity),
            "angular_velocity": tuple(float(value) for value in angular_velocity),
            "linear_acceleration": (
                acceleration_ned[0],
                -acceleration_ned[1],
                -(acceleration_ned[2] + 9.80665),
            ),
        }
        with self._state_lock:
            # Store an immutable numeric PhysX snapshot so the UDP sender can
            # run safely in parallel with Isaac's main thread.
            self._latest_state = (
                position_ned,
                quaternion_ned,
                velocity_ned,
                gyro_body,
                acceleration_body,
            )
            self._latest_state_payload_tail = self._state_payload_tail(*self._latest_state)
        if self._last_frame is not None and self._last_log_time < 0.0:
            print("ArduPilot SITL connected; Isaac is applying its PWM motor outputs", flush=True)
            self._last_log_time = sim_time

    def ground_truth_snapshot(self):
        """Return the latest clone-local kinematic state for tuning telemetry."""
        return self._ground_truth_snapshot

    def close(self) -> None:
        self._sender_running = False
        if getattr(self, "_sender_thread", None) is not None:
            self._sender_thread.join(timeout=1.0)
            self._sender_thread = None
        if getattr(self, "_socket", None) is not None:
            self._socket.close()
            self._socket = None


class RosSensorPublisher:
    """Publish stable ROS 2 sensor messages while native sensor writers are disabled."""

    def __init__(
        self,
        stage=None,
        range_sensor_path: str | None = None,
        upward_range_sensor_path: str | None = None,
        drone_body_path: str = "/Drone/body",
        front_camera_path: str | None = None,
        lidar_sensor=None,
        enable_rgb_capture: bool = True,
    ) -> None:
        import rclpy
        from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField, Range
        from std_msgs.msg import Header
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

        self._rclpy = rclpy
        self._Header = Header
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._frame = 0
        self._width = _env_int("ISAAC_CAMERA_WIDTH", 320)
        self._height = _env_int("ISAAC_CAMERA_HEIGHT", 240)
        # Schedule by simulation time rather than render-frame count. This
        # remains correct when several 100 Hz physics steps share one frame.
        camera_fps = max(1.0, float(_env_int("ISAAC_CAMERA_FPS", 5)))
        self._camera_period = 1.0 / camera_fps
        self._next_camera_time = 0.0
        self._stage = stage
        self._range_sensor_path = range_sensor_path
        self._upward_range_sensor_path = upward_range_sensor_path
        self._lidar_sensor = lidar_sensor
        self._last_lidar_frame_id = None
        self._lidar_reported = False
        self._drone_body_path = drone_body_path
        self._TransformStamped = TransformStamped
        self._tf_broadcaster = None
        self._static_tf_broadcaster = None
        self._enable_rgb_capture = enable_rgb_capture
        self._camera_source = None
        self._camera_warning_printed = False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node("isaac_room_sensor_bridge")
            self._Image = Image
            self._CameraInfo = CameraInfo
            self._Imu = Imu
            self._Range = Range
            self._PointCloud2 = PointCloud2
            self._PointField = PointField
            self._Quaternion = Quaternion
            self._Vector3 = Vector3
            self._tf_broadcaster = TransformBroadcaster(self._node)
            self._static_tf_broadcaster = StaticTransformBroadcaster(self._node)
            self._publish_static_transforms()
            if self._enable_rgb_capture and front_camera_path:
                try:
                    self._camera_source = RenderCameraSource(front_camera_path, self._width, self._height)
                    print("Native RGB camera capture enabled", flush=True)
                except Exception:
                    print("Native RGB capture unavailable; using visible diagnostic fallback", flush=True)
                    traceback.print_exc()
            self._front_image = self._node.create_publisher(Image, "/camera/front/image_raw", 10)
            self._front_info = self._node.create_publisher(CameraInfo, "/camera/front/camera_info", 10)
            self._imu = self._node.create_publisher(Imu, "/imu/data", 10)
            self._range_down = self._node.create_publisher(Range, "/range/down", 10)
            self._range_up = self._node.create_publisher(Range, "/range/up", 10)
            self._lidar_points = self._node.create_publisher(PointCloud2, "/lidar/points", 10)
            # SLAM receives the exact same unfiltered returns on a separate
            # frame tree. This keeps Isaac ground truth and ICP odometry from
            # competing to parent the same TF frame.
            self._slam_lidar_points = self._node.create_publisher(
                PointCloud2, "/slam/lidar/points", 10
            )
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._spin_ros, name="ros2-sensor-spin", daemon=True)
            self._spin_thread.start()
            print(
                "ROS sensor topics enabled: /camera/front/image_raw "
                "/lidar/points /slam/lidar/points /imu/data /range/down /range/up "
                "/tf /tf_static",
                flush=True,
            )
        except Exception:
            self.close()
            print("ROS sensor publishers unavailable; Isaac Sim will continue without sensor topics", flush=True)
            traceback.print_exc()

    def _spin_ros(self) -> None:
        """Spin quietly through the expected rclpy shutdown transition."""
        try:
            self._executor.spin()
        except Exception as error:
            if self._rclpy.ok():
                print(f"ROS 2 sensor executor stopped unexpectedly: {error}", flush=True)

    def _safe_publish(self, publisher, message) -> bool:
        """Avoid shutdown-time RCLError when Isaac closes the ROS context first."""
        if self._node is None or not self._rclpy.ok():
            return False
        try:
            publisher.publish(message)
            return True
        except Exception as error:
            if self._rclpy.ok():
                print(f"ROS 2 sensor publish failed: {error}", flush=True)
            return False

    def _make_transform(
        self,
        parent: str,
        child: str,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    ):
        transform = self._TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform

    def _publish_static_transforms(self) -> None:
        """Publish the fixed sensor mounts needed by RViz and robot_state_publisher."""
        if self._static_tf_broadcaster is None:
            return
        # The front USD camera orientation is Gf.Quatf(0.5, 0.5, -0.5, -0.5):
        # ROS quaternion ordering is x, y, z, w.
        front_rotation = (0.5, -0.5, -0.5, 0.5)
        # RTAB-Map starts its local base at the first LiDAR pose.  Put the map
        # origin at the actual spawned body pose, not at the warehouse origin.
        # This is a one-time frame calibration only; no live ground-truth pose
        # is ever fed back into SLAM.
        map_translation = (0.0, 0.0, 0.0)
        map_rotation = (0.0, 0.0, 0.0, 1.0)
        if self._stage is not None:
            try:
                from pxr import Usd, UsdGeom

                body = self._stage.GetPrimAtPath(self._drone_body_path)
                if body.IsValid():
                    body_world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    )
                    position = body_world.ExtractTranslation()
                    rotation = body_world.ExtractRotation().GetQuaternion()
                    imaginary = rotation.GetImaginary()
                    map_translation = tuple(float(position[i]) for i in range(3))
                    map_rotation = (
                        float(imaginary[0]),
                        float(imaginary[1]),
                        float(imaginary[2]),
                        float(rotation.GetReal()),
                    )
            except Exception:
                print("Could not align SLAM map origin to the initial drone pose", flush=True)
                traceback.print_exc()
        transforms = [
            self._make_transform("world", "map", map_translation, map_rotation),
            self._make_transform(
                "drone_base", "camera_front_optical_frame", (0.16, 0.0, 0.02), front_rotation
            ),
            self._make_transform("drone_base", "lidar_3d_link", (0.0, 0.0, 0.09)),
            self._make_transform("drone_base", "imu_link", (0.0, 0.0, 0.0)),
            self._make_transform("drone_base", "tof_down_link", (0.0, 0.0, -0.05)),
            self._make_transform("drone_base", "tof_up_link", (0.0, 0.0, 0.12)),
            self._make_transform("slam_base", "slam_lidar_link", (0.0, 0.0, 0.09)),
        ]
        self._static_tf_broadcaster.sendTransform(transforms)
        print(
            "SLAM map origin aligned to initial drone pose: "
            f"xyz=({map_translation[0]:.4f}, {map_translation[1]:.4f}, {map_translation[2]:.4f})",
            flush=True,
        )

    def _publish_dynamic_transform(self, sim_time: float) -> None:
        """Publish world -> drone_base from the live USD body transform."""
        if self._tf_broadcaster is None:
            return
        transform = self._make_transform("world", "drone_base", (0.0, 0.0, 1.5))
        if self._stage is not None:
            try:
                from pxr import Usd, UsdGeom

                prim = self._stage.GetPrimAtPath(self._drone_body_path)
                if prim.IsValid():
                    world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    position = world.ExtractTranslation()
                    rotation = world.ExtractRotation().GetQuaternion()
                    transform.transform.translation.x = float(position[0])
                    transform.transform.translation.y = float(position[1])
                    transform.transform.translation.z = float(position[2])
                    transform.transform.rotation.x = float(rotation.GetImaginary()[0])
                    transform.transform.rotation.y = float(rotation.GetImaginary()[1])
                    transform.transform.rotation.z = float(rotation.GetImaginary()[2])
                    transform.transform.rotation.w = float(rotation.GetReal())
            except Exception:
                pass
        transform.header.stamp = self._header(sim_time, "world").stamp
        self._tf_broadcaster.sendTransform(transform)

    def _header(self, sim_time: float, frame_id: str):
        header = self._Header()
        header.stamp.sec = int(sim_time)
        header.stamp.nanosec = int((sim_time - int(sim_time)) * 1_000_000_000)
        header.frame_id = frame_id
        return header

    def _image(self, sim_time: float, frame_id: str):
        msg = self._Image()
        msg.header = self._header(sim_time, frame_id)
        msg.height = self._height
        msg.width = self._width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = self._width * 3
        if self._camera_source is not None:
            rendered = self._camera_source.get("front")
            if rendered is not None:
                msg.data = rendered
                return msg
        # Keep a visible diagnostic fallback if rendering is unavailable.
        pixels = bytearray(msg.step * msg.height)
        for y in range(self._height):
            for x in range(self._width):
                offset = (y * self._width + x) * 3
                grid = ((x // 32) + (y // 24)) % 2
                pixels[offset:offset + 3] = (40 + 40 * grid, 80, 190 + 40 * grid)
                if abs(x - self._width // 2) < 2 or abs(y - self._height // 2) < 2:
                    pixels[offset:offset + 3] = (255, 255, 255)
        msg.data = bytes(pixels)
        return msg

    def _camera_info(self, sim_time: float, frame_id: str):
        msg = self._CameraInfo()
        msg.header = self._header(sim_time, frame_id)
        msg.height = self._height
        msg.width = self._width
        msg.distortion_model = "plumb_bob"
        msg.k = [300.0, 0.0, self._width / 2.0, 0.0, 300.0, self._height / 2.0, 0.0, 0.0, 1.0]
        msg.p = [300.0, 0.0, self._width / 2.0, 0.0, 0.0, 300.0, self._height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _directional_distance(self, sensor_path: str | None, direction_z: float) -> float:
        """Raycast from a ToF mount, ignoring camera rendering entirely."""
        if self._stage is None or not sensor_path:
            return 12.0
        try:
            from carb import Float3
            import omni.physx
            from pxr import Usd, UsdGeom

            prim = self._stage.GetPrimAtPath(sensor_path)
            if not prim.IsValid():
                return 12.0
            transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            position = transform.ExtractTranslation()
            hit = omni.physx.get_physx_scene_query_interface().raycast_closest(
                Float3(float(position[0]), float(position[1]), float(position[2])),
                Float3(0.0, 0.0, direction_z),
                12.0,
                True,
            )
            if hit.get("hit"):
                return max(0.05, min(12.0, float(hit["distance"])))
            return 12.0
        except Exception:
            return 12.0

    def _point_cloud_message(self, sim_time: float, points, frame_id: str = "lidar_3d_link"):
        """Serialize an Nx3 float array as a sensor-frame PointCloud2 message."""
        message = self._PointCloud2()
        message.header = self._header(sim_time, frame_id)
        message.height = 1
        message.width = int(points.shape[0])
        message.fields = [
            self._PointField(name="x", offset=0, datatype=self._PointField.FLOAT32, count=1),
            self._PointField(name="y", offset=4, datatype=self._PointField.FLOAT32, count=1),
            self._PointField(name="z", offset=8, datatype=self._PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = points.tobytes(order="C")
        return message

    def _publish_lidar(self, sim_time: float) -> None:
        """Publish one native RTX OS1 frame as sensor_msgs/PointCloud2."""
        if self._lidar_sensor is None:
            return
        try:
            import numpy as np
            import isaacsim.sensors.experimental.rtx.generic_model_output as gmo_api
            from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

            data, _ = self._lidar_sensor.get_data("generic-model-output")
            if data is None:
                return
            raw = data.numpy()
            raw_bytes = raw.view(np.uint8).reshape(-1)
            if raw_bytes.size < 4:
                return
            magic = int.from_bytes(raw_bytes[:4].tobytes(), byteorder="little")
            if magic != int(gmo_api.getMagicNumberGMO()):
                # Multi-tick exposes the render buffer between completed OS1
                # scans. Skip those in-progress snapshots quietly.
                return
            gmo = parse_generic_model_output_data(raw)
            point_count = int(gmo.numElements)
            frame_id = int(gmo.frameId)
            if point_count <= 0 or frame_id == self._last_lidar_frame_id:
                return
            self._last_lidar_frame_id = frame_id

            # The OmniLidar was authored for Cartesian output in its own sensor
            # frame, so these arrays map directly to ROS x/y/z fields.
            points = np.column_stack((gmo.x, gmo.y, gmo.z)).astype("<f4", copy=False)
            points = points[np.all(np.isfinite(points), axis=1)]
            if points.size == 0:
                return

            raw_message = self._point_cloud_message(sim_time, points)
            slam_message = self._point_cloud_message(sim_time, points, "slam_lidar_link")
            published = self._safe_publish(self._lidar_points, raw_message)
            self._safe_publish(self._slam_lidar_points, slam_message)
            if published and not self._lidar_reported:
                self._lidar_reported = True
                print(
                    f"3D LiDAR healthy: {raw_message.width} unfiltered points "
                    "published for RViz and SLAM",
                    flush=True,
                )
        except Exception:
            if not self._lidar_reported:
                print("3D LiDAR PointCloud2 conversion failed", flush=True)
                traceback.print_exc()
                self._lidar_reported = True

    def update(self, sim_time: float) -> None:
        if self._node is None or not self._rclpy.ok():
            return
        self._frame += 1
        self._publish_dynamic_transform(sim_time)
        self._publish_lidar(sim_time)
        # Capture the single required visual stream at its simulation-time
        # rate, independent of the interactive viewport frequency.
        if self._enable_rgb_capture and sim_time + 1.0e-9 >= self._next_camera_time:
            self._next_camera_time = sim_time + self._camera_period
            if self._camera_source is not None:
                try:
                    self._camera_source.capture()
                except Exception:
                    if not self._camera_warning_printed:
                        print("Camera render capture failed; using visible diagnostic fallback", flush=True)
                        traceback.print_exc()
                        self._camera_warning_printed = True
            self._safe_publish(self._front_image, self._image(sim_time, "camera_front_optical_frame"))
            self._safe_publish(self._front_info, self._camera_info(sim_time, "camera_front_optical_frame"))
        imu = self._Imu()
        imu.header = self._header(sim_time, "imu_link")
        imu.orientation.w = 1.0
        imu.linear_acceleration.z = 9.81
        self._safe_publish(self._imu, imu)
        if self._frame % 2 == 0:
            for publisher, frame_id, sensor_path, direction_z in (
                (self._range_down, "tof_down_link", self._range_sensor_path, -1.0),
                (self._range_up, "tof_up_link", self._upward_range_sensor_path, 1.0),
            ):
                distance = self._Range()
                distance.header = self._header(sim_time, frame_id)
                distance.radiation_type = self._Range.INFRARED
                distance.field_of_view = 0.01
                distance.min_range = 0.05
                distance.max_range = 12.0
                distance.range = self._directional_distance(sensor_path, direction_z)
                self._safe_publish(publisher, distance)
        if self._frame == 1:
            print("ROS sensor messages publishing", flush=True)

    def close(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()


def build_stage(output_path: Path | None) -> dict[str, str]:
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics
    import omni.usd

    from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage

    create_new_stage()
    # Isaac Sim 6 uses the USD context as the source of the active stage.
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim created no active USD stage")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetFramesPerSecond(60.0)
    physics_scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
    # Resolve rigid-body motion substantially faster than rendering.  This
    # gives the 400 Hz FCU fresh physics at 240 Hz instead of only 60 Hz.
    PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim()).CreateTimeStepsPerSecondAttr().Set(240)

    default_warehouse = Path(__file__).resolve().parents[1] / "assets" / "simple_warehouse" / "warehouse.usd"
    environment_path = Path(
        os.environ.get(
            "ENVIRONMENT_USD_PATH",
            os.environ.get("WAREHOUSE_USD_PATH", str(default_warehouse)),
        )
    ).expanduser()
    if not environment_path.is_file():
        raise FileNotFoundError(
            f"Local environment USD not found: {environment_path}. "
            "Set ENVIRONMENT_USD_PATH to a local USD file."
        )
    # Open and compose the local environment before adding it to the live
    # stage. The root warehouse layer is only ~1.6 MB and references many
    # sibling prop/material files; checking the composed mesh count prevents
    # an apparently valid but visually empty stage from starting.
    environment_stage = Usd.Stage.Open(str(environment_path.resolve()))
    if environment_stage is None:
        raise RuntimeError(f"Could not open local environment USD: {environment_path}")
    environment_mesh_count = sum(
        1 for prim in environment_stage.Traverse() if prim.IsA(UsdGeom.Mesh)
    )
    is_vendored_warehouse = environment_path.resolve() == default_warehouse.resolve()
    minimum_meshes = 100 if is_vendored_warehouse else 1
    if environment_mesh_count < minimum_meshes:
        raise RuntimeError(
            f"Local environment is incomplete ({environment_mesh_count} meshes composed): "
            f"{environment_path}"
        )
    print(
        f"Local environment validated: {environment_path.name} "
        f"({environment_mesh_count} composed meshes)",
        flush=True,
    )
    warehouse = stage.DefinePrim("/Warehouse", "Xform")
    warehouse.GetReferences().AddReference(str(environment_path.resolve()))

    drone_path = Path(os.environ.get("DRONE_USD_PATH", str(Path.home() / "Downloads" / "iris.usd"))).expanduser()
    if not drone_path.is_file():
        raise FileNotFoundError(
            f"Custom drone USD not found: {drone_path}. Set DRONE_USD_PATH to the location of iris.usd."
        )
    add_reference_to_stage(str(drone_path), "/Drone")
    localize_drone_materials(stage)

    # Place the lowest Iris geometry just above the warehouse floor instead of
    # suspending it or adding a project-specific landing pad. Compute this from
    # the referenced asset so a future Iris revision still starts correctly.
    drone = stage.GetPrimAtPath("/Drone")
    if drone.IsValid():
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        )
        minimum_z = float(bbox_cache.ComputeWorldBound(drone).ComputeAlignedRange().GetMin()[2])
        spawn_z = max(0.0, -minimum_z + 0.003)
        UsdGeom.XformCommonAPI(drone).SetTranslate(Gf.Vec3d(0.0, 0.0, spawn_z))
        print(f"Drone ground spawn height: {spawn_z:.3f} m", flush=True)

    # Add a key light for the warehouse if its referenced lighting is sparse.
    dome = UsdGeom.Xform.Define(stage, "/Lighting")
    dome.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 2.8))
    distant = UsdLux.DistantLight.Define(stage, "/Lighting/KeyLight")
    distant.CreateIntensityAttr(2500.0)
    distant.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))
    sensor_info = add_drone_sensors(stage)
    make_sensor_mounts_massless(stage, sensor_info)
    # The downloaded visual USD does not provide flight-grade rigid-body mass
    # properties. Use the standard Iris values from its dynamics model; tiny
    # inferred mesh inertia makes normal motor corrections flip the vehicle.
    body_prim = stage.GetPrimAtPath(sensor_info["parent"])
    if body_prim.IsValid():
        mass_api = UsdPhysics.MassAPI.Apply(body_prim)
        mass_api.CreateMassAttr().Set(1.5)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.029125, 0.029125, 0.055225))
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    if output_path is None:
        return sensor_info
    if not output_path.parent.exists():
        output_path.parent.mkdir(parents=True, exist_ok=True)
    stage.SetEditTarget(stage.GetRootLayer())
    stage.GetRootLayer().Export(str(output_path))
    return sensor_info


def main() -> None:
    args = parse_args()
    experience = None
    if isaac_sim_root := os.environ.get("ISAAC_SIM_ROOT"):
        candidate = Path(isaac_sim_root) / "apps" / (
            "isaacsim.exp.base.python.kit" if args.headless else "isaacsim.exp.full.kit"
        )
        if candidate.is_file():
            experience = str(candidate)
    # Build the RTX-optimized renderer config
    renderer_config = _rtx_renderer_config()
    synchronous_usd_loads = _env_bool("ISAAC_SYNC_USD_LOADS", True)
    extra_args = [
        "--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge",
        "--/renderer/multiGpu/autoEnable=false",
        "--/renderer/multiGpu/enabled=false",
        # Let PhysX catch up with multiple 100 Hz substeps after an expensive
        # RTX frame. The full GUI experience otherwise forces one fixed 10 ms
        # physics step per rendered frame, making RTF track visual FPS / 100.
        "--/app/player/useFixedTimeStepping=false",
        f"--/persistent/simulation/minFrameRate={_env_int('ISAAC_PHYSX_MIN_FRAME_RATE', 15)}",
        "--/rtx/scenedb/maxHistoryTransformCount=16",
    ]
    if not synchronous_usd_loads:
        extra_args.extend(
            [
                "--/rtx/materialDb/syncLoads=false",
                "--/rtx/hydra/materialSyncLoads=false",
                "--/omni/kit/plugin/syncUsdLoads=false",
            ]
        )
    app_config = {
            "headless": args.headless,
            # RTX renderer selection and GPU index
            "renderer": renderer_config["renderer"],
            "renderer_gpu_index": renderer_config["renderer_gpu_index"],
            # Motion BVH can model beam timing on fast-moving sensors, but it
            # substantially increases RTX startup/VRAM cost. Keep the indoor
            # teleoperation default responsive and allow opt-in when needed.
            "enable_motion_bvh": _env_bool("ISAAC_LIDAR_MOTION_BVH", False),
            "create_new_stage": False,
            "extra_args": extra_args,
        }
    if experience is not None:
        app_config["experience"] = experience
    simulation_app = SimulationApp(app_config)
    ros_sensor_publisher = None
    lidar_sensor = None
    hover_controller = None
    flight_control_bridge = None
    physics_callback_id = None
    rtx_diagnostics = None
    try:
        # Let Kit finish creating its stage/window before editing USD.
        simulation_app.update()
        # Apply RTX/DLSS/viewport settings now that Kit is initialized
        rtx_diagnostics = _apply_rtx_settings(simulation_app, args.headless)
        enable_ros_bridge()
        # Extension activation is asynchronous in Kit; allow its node types to register.
        simulation_app.update()
        output_path = Path(__file__).resolve().parents[1] / "stages" / "room_quadcopter.usd"
        if args.no_save:
            sensor_info = build_stage(None)
        else:
            sensor_info = build_stage(output_path)
        print(f"Created local environment + custom drone stage: {output_path}", flush=True)
        import omni.usd
        import omni.timeline
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        timeline = omni.timeline.get_timeline_interface()
        # Copter's attitude loop requires fresh plant feedback at 100 Hz. Kit
        # can run several physics substeps per rendered frame, so keep control
        # in a physics callback rather than the slower viewport loop.
        # GPU physics is opt-in (default cpu): CPU PhysX is rock-solid with the
        # 800 Hz ArduPilot JSON bridge; enable GPU physics only after control
        # rate is validated on the RTX 5080 with ISAAC_ENABLE_GPU_PHYSICS=1.
        physics_device = "gpu" if _env_bool("ISAAC_ENABLE_GPU_PHYSICS", False) else "cpu"
        SimulationManager.setup_simulation(dt=1.0 / 100.0, device=physics_device)
        simulation_app.update()
        # Build the live /clock graph only after Kit has completed one update
        # on the exported stage. This avoids a native OmniGraph race in the
        # Isaac Sim 6.0.1-rc.7 build.
        add_ros_clock_graph()
        simulation_app.update()
        # ROS bridge extensions load asynchronously. Give Kit a few updates
        # to finish registering the ROS middleware before creating publishers.
        for _ in range(5):
            simulation_app.update()
        sensor_stage = omni.usd.get_context().get_stage()
        # LidarSensor owns an auto-triggered render product. Keep it alive for
        # the entire app lifetime so native RTX frames remain available.
        lidar_sensor = create_lidar_runtime(sensor_info["lidar"])
        simulation_app.update()
        if not args.headless:
            # Use the editor's normal Perspective camera, framed on the
            # drone. Sensor cameras continue to render only to their own
            # ROS products and never take over the main viewport.
            try:
                from pxr import Gf, UsdGeom
                from isaacsim.core.rendering_manager import ViewportManager

                ViewportManager.set_camera("/OmniverseKit_Persp")
                camera_prim = sensor_stage.GetPrimAtPath("/OmniverseKit_Persp")
                camera_xform = UsdGeom.Xformable(camera_prim)
                for operation in camera_xform.GetOrderedXformOps():
                    if operation.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        operation.Set(Gf.Vec3d(6.04763, -5.74526, 3.94019))
                    elif operation.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        operation.Set(Gf.Vec3f(68.13778, 0.0, 50.89824))
                print("Viewport set to requested Perspective drone view", flush=True)
            except Exception:
                print("Could not frame the Perspective viewport on the warehouse", flush=True)
                traceback.print_exc()
        # Print RTX diagnostics after sensor publisher initialization so we know
        # what settings actually took effect.
        if rtx_diagnostics:
            print("RTX 5080 Optimization Diagnostics:", flush=True)
            print(f"  Selected GPU index: {rtx_diagnostics['gpu_index']}", flush=True)
            print(f"  RTX optimization enabled: {rtx_diagnostics['enable_rtx']}", flush=True)
            print(f"  Ray tracing enabled: {rtx_diagnostics['enable_ray_tracing']}", flush=True)
            print(f"  DLSS mode: {rtx_diagnostics['dlss_mode']} "
                  f"(0=Perf,1=Balanced,2=Quality,3=Auto)", flush=True)
            print(f"  Viewport resolution: {rtx_diagnostics['viewport_width']}x"
                  f"{rtx_diagnostics['viewport_height']}", flush=True)
            print(f"  Headless mode: {rtx_diagnostics['headless']}", flush=True)
            # Camera and physics diagnostics
            print(f"  Camera resolution: {_env_int('ISAAC_CAMERA_WIDTH', 320)}x"
                  f"{_env_int('ISAAC_CAMERA_HEIGHT', 240)}@"
                  f"{_env_int('ISAAC_CAMERA_FPS', 5)} Hz", flush=True)
            print(
                "  Timing: wall-clock catch-up, physics=100 Hz, "
                f"min frame clamp={_env_int('ISAAC_PHYSX_MIN_FRAME_RATE', 15)} Hz",
                flush=True,
            )
            print(f"  GPU physics enabled: {physics_device == 'gpu'} "
                  f"(device={physics_device})", flush=True)
            print(f"  Shader cache: {os.environ.get('ISAAC_SHADER_CACHE_DIR', 'not set')}", flush=True)
            print(f"  RViz2 auto-launch: {'enabled' if _env_bool('ISAAC_AUTO_RVIZ', False) else 'disabled'}", flush=True)
        ros_sensor_publisher = RosSensorPublisher(
            sensor_stage,
            sensor_info["tof"],
            sensor_info["tof_up"],
            sensor_info["parent"],
            sensor_info["front_camera"],
            lidar_sensor,
            # Capture real low-rate RGB images by default. Set
            # FLIGHT_CONTROL_RGB_CAPTURE=0 only when diagnosing render load.
            enable_rgb_capture=(
                args.flight_control != "ardupilot"
                or os.environ.get("FLIGHT_CONTROL_RGB_CAPTURE", "1") == "1"
            ),
        )
        if args.flight_control == "ardupilot":
            if args.no_hover:
                raise ValueError("--no-hover cannot be combined with --flight-control ardupilot")
            flight_control_bridge = ArduPilotJsonPhysicsBridge(
                sensor_stage,
                sensor_info["parent"],
                args.sitl_json_port,
            )
            physics_time = [0.0]

            def flight_control_physics_step(dt: float, _context) -> None:
                physics_time[0] += float(dt)
                flight_control_bridge.update(physics_time[0])

            physics_callback_id = SimulationManager.register_callback(
                flight_control_physics_step,
                event=SimulationEvent.PHYSICS_PRE_STEP,
            )
            print(
                "Flight control: ArduPilot SITL JSON mode; waiting for SITL motor outputs",
                flush=True,
            )
        elif not args.no_hover:
            try:
                hover_controller = PropellerAltitudeController(
                    sensor_stage,
                    sensor_info["parent"],
                    args.target_height,
                )
                print(
                    f"Four-propeller collective altitude hold enabled at "
                    f"{args.target_height:.2f} m",
                    flush=True,
                )
            except Exception:
                print("Altitude hold unavailable; drone will remain under normal gravity", flush=True)
                traceback.print_exc()
        print(
            "Sensors active: front camera, Ouster OS1-style 3D RTX LiDAR, "
            "IMU, downward/upward ToF "
            "(ROS topics are enabled)",
            flush=True,
        )
        if not args.start_paused:
            app_utils.play()
            timeline.play()

        rtf_wall_start = time.perf_counter()
        rtf_sim_start = SimulationManager.get_simulation_time()
        rtf_last_report = rtf_wall_start

        def report_rtf(current_sim_time: float) -> None:
            nonlocal rtf_last_report
            now = time.perf_counter()
            if now - rtf_last_report < 5.0:
                return
            wall_elapsed = max(1.0e-6, now - rtf_wall_start)
            rtf = max(0.0, current_sim_time - rtf_sim_start) / wall_elapsed
            print(
                f"Performance: RTF={rtf:.3f}, sim={current_sim_time - rtf_sim_start:.1f}s, "
                f"wall={wall_elapsed:.1f}s",
                flush=True,
            )
            rtf_last_report = now

        def pace_real_time(current_sim_time: float) -> None:
            """Prevent catch-up stepping from running persistently above RTF 1."""
            if not _env_bool("ISAAC_REAL_TIME_PACING", True):
                return
            target_wall_time = rtf_wall_start + max(0.0, current_sim_time - rtf_sim_start)
            remaining = target_wall_time - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)

        if args.headless and args.run_seconds > 0.0:
            deadline = time.perf_counter() + args.run_seconds
            sim_time = 0.0
            while simulation_app.is_running() and time.perf_counter() < deadline:
                sim_time += 1.0 / 60.0
                if hover_controller is not None and flight_control_bridge is None and not args.start_paused:
                    hover_controller.update(sim_time)
                simulation_app.update()
                sim_time = SimulationManager.get_simulation_time()
                ros_sensor_publisher.update(sim_time)
                pace_real_time(sim_time)
                report_rtf(sim_time)
        elif not args.headless:
            sim_time = 0.0
            while simulation_app.is_running():
                sim_time += 1.0 / 60.0
                if hover_controller is not None and flight_control_bridge is None and not args.start_paused:
                    hover_controller.update(sim_time)
                simulation_app.update()
                sim_time = SimulationManager.get_simulation_time()
                ros_sensor_publisher.update(sim_time)
                pace_real_time(sim_time)
                report_rtf(sim_time)
    except Exception:
        traceback.print_exc()
        raise
    finally:
        if physics_callback_id is not None:
            SimulationManager.deregister_callback(physics_callback_id)
        if ros_sensor_publisher is not None:
            ros_sensor_publisher.close()
        if flight_control_bridge is not None:
            flight_control_bridge.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
