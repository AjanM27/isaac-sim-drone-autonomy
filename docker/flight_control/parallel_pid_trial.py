#!/usr/bin/env python3
"""Command and record a synchronized batch of ArduPilot tuning candidates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing as mp
import os
import statistics
import time
import traceback
from copy import deepcopy
from pathlib import Path
from threading import BrokenBarrierError

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from teleop import ISAAC_WORLD_ZERO_YAW_NED, Teleop
from parallel_pid_optimizer import aggregate_candidates
from parallel_pid_metrics import (
    add_reference_signals,
    continuous_angle_errors_degrees,
    motion_metrics,
    trajectory_error_metrics,
)


def rms(values) -> float:
    values = list(values)
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else 0.0


def euler_degrees(q) -> tuple[float, float, float]:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return tuple(math.degrees(value) for value in (roll, pitch, math.atan2(siny, cosy)))


def atomic_json(path: Path, value: dict) -> None:
    """Publish progress without exposing a partially written status file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


class CandidateNode(Teleop):
    def __init__(self, index: int, output_dir: Path, candidate: dict) -> None:
        namespace = f"/tuning/env_{index}"
        super().__init__(namespace, f"parallel_pid_candidate_{index}")
        self.index = index
        self.phase = "preflight"
        self.command_velocity = (0.0, 0.0, 0.0)
        self.latest_acceleration = (0.0, 0.0, 0.0)
        self._previous_ground_truth_velocity = None
        self._previous_ground_truth_time = None
        self.latest_ground_truth: Odometry | None = None
        self.samples: list[dict[str, float | str]] = []
        self.output_dir = output_dir
        self.candidate = candidate
        self.started_at = time.monotonic()
        self.phase_started_at = self.started_at
        self.phase_duration = None
        self._last_progress_write = 0.0
        self.takeoff_command_time: float | None = None
        self.create_subscription(
            Odometry,
            f"/tuning/env_{index}/ground_truth",
            self.ground_truth_callback,
            qos_profile_sensor_data,
        )

    def ground_truth_callback(self, message: Odometry) -> None:
        self.latest_ground_truth = message
        p = message.pose.pose.position
        v = message.twist.twist.linear
        w = message.twist.twist.angular
        roll, pitch, yaw = euler_degrees(message.pose.pose.orientation)
        sample_time = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        velocity = (v.x, v.y, v.z)
        if self._previous_ground_truth_time is not None:
            dt = sample_time - self._previous_ground_truth_time
            if dt > 1.0e-6:
                self.latest_acceleration = tuple(
                    (velocity[axis] - self._previous_ground_truth_velocity[axis]) / dt
                    for axis in range(3)
                )
        self._previous_ground_truth_velocity = velocity
        self._previous_ground_truth_time = sample_time
        a = self.latest_acceleration
        estimated = self.local_pose.pose.position if self.local_pose else None
        self.samples.append(
            {
                "time": time.monotonic(),
                "sim_time": sample_time,
                "phase": self.phase,
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "vx": v.x,
                "vy": v.y,
                "vz": v.z,
                "ax": a[0],
                "ay": a[1],
                "az": a[2],
                "roll_deg": roll,
                "pitch_deg": pitch,
                "yaw_deg": yaw,
                "wx": w.x,
                "wy": w.y,
                "wz": w.z,
                "command_vx": self.command_velocity[0],
                "command_vy": self.command_velocity[1],
                "command_vz": self.command_velocity[2],
                "estimated_x": estimated.x if estimated else math.nan,
                "estimated_y": estimated.y if estimated else math.nan,
                "estimated_z": estimated.z if estimated else math.nan,
            }
        )
        if time.monotonic() - self._last_progress_write >= 1.0:
            self.write_progress("running")

    def set_phase(self, phase: str, duration: float | None = None) -> None:
        if phase == self.phase and duration == self.phase_duration:
            return
        self.phase = phase
        self.phase_duration = duration
        self.phase_started_at = time.monotonic()
        print(
            f"[drone {self.index:02d}] {self.candidate['name']}: {phase}"
            + (f" ({duration:.1f}s)" if duration is not None else ""),
            flush=True,
        )
        self.write_progress("running", force=True)

    def write_progress(
        self, status: str, *, force: bool = False, message: str = "", result: dict | None = None
    ) -> None:
        now = time.monotonic()
        if not force and now - self._last_progress_write < 0.9:
            return
        self._last_progress_write = now
        state = self.latest_ground_truth
        position = state.pose.pose.position if state else None
        velocity = state.twist.twist.linear if state else None
        phase_elapsed = max(0.0, now - self.phase_started_at)
        payload = {
            "drone": self.index,
            "pid": os.getpid(),
            "candidate": self.candidate.get("name", "unknown"),
            "candidate_id": self.candidate.get("candidate_id", ""),
            "replica": self.candidate.get("replica", 0),
            "status": status,
            "phase": self.phase,
            "phase_elapsed_s": phase_elapsed,
            "phase_duration_s": self.phase_duration,
            "phase_progress": (
                min(1.0, phase_elapsed / self.phase_duration)
                if self.phase_duration and self.phase_duration > 0.0 else None
            ),
            "elapsed_s": max(0.0, now - self.started_at),
            "sample_count": len(self.samples),
            "position_m": (
                [position.x, position.y, position.z] if position is not None else None
            ),
            "velocity_mps": (
                [velocity.x, velocity.y, velocity.z] if velocity is not None else None
            ),
            "command_velocity_mps": list(self.command_velocity),
            "message": message,
            "updated_at": datetime_now(),
        }
        if result is not None:
            payload["result"] = result
        atomic_json(self.output_dir / f"env_{self.index}_progress.json", payload)


def datetime_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def drain_callbacks(nodes: list[CandidateNode], maximum: int | None = None) -> None:
    """Drain queued ROS work synchronously; never spin an executor concurrently."""
    count = maximum if maximum is not None else max(16, 12 * len(nodes))
    for _ in range(count):
        nodes[0].spin_once(timeout_sec=0.0)


def spin_and_command(
    nodes: list[CandidateNode],
    phase: str,
    duration: float,
    command,
    ideal_targets: dict[int, object] | None = None,
) -> None:
    if ideal_targets is not None:
        for node in nodes:
            velocity = command(node)
            target = ideal_targets[node.index]
            target.pose.position.x += velocity[0] * duration
            target.pose.position.y += velocity[1] * duration
            target.pose.position.z += velocity[2] * duration
    for node in nodes:
        node.set_phase(phase, duration)
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        drain_callbacks(nodes)
        for node in nodes:
            velocity = command(node)
            node.command_velocity = velocity
            node.send_velocity(*velocity)
        time.sleep(0.02)


def spin_velocity_and_yaw(
    nodes: list[CandidateNode],
    phase: str,
    duration: float,
    velocity: tuple[float, float, float],
    yaw: float,
    ideal_targets: dict[int, object] | None = None,
) -> None:
    if ideal_targets is not None:
        for node in nodes:
            target = ideal_targets[node.index]
            target.pose.position.x += velocity[0] * duration
            target.pose.position.y += velocity[1] * duration
            target.pose.position.z += velocity[2] * duration
    for node in nodes:
        node.set_phase(phase, duration)
        node.command_velocity = velocity
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        drain_callbacks(nodes)
        for node in nodes:
            node.send_velocity(*velocity, yaw=yaw)
        time.sleep(0.02)
    drain_callbacks(nodes)


def spin_nodes(nodes: list[CandidateNode], duration: float) -> None:
    """Refresh all subscriptions without changing ArduPilot's active target."""
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        drain_callbacks(nodes)
        time.sleep(0.02)


def spin_position_hold(
    nodes: list[CandidateNode],
    phase: str,
    duration: float,
    targets: dict[int, object],
    yaw: float = ISAAC_WORLD_ZERO_YAW_NED,
) -> None:
    for node in nodes:
        node.set_phase(phase, duration)
        node.command_velocity = (0.0, 0.0, 0.0)
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        drain_callbacks(nodes)
        for node in nodes:
            node.send_position_hold(targets[node.index], yaw=yaw)
        time.sleep(0.02)


def spin_brake_and_hold(
    nodes: list[CandidateNode], phase: str, duration: float,
    ideal_targets: dict[int, object] | None = None,
) -> None:
    """Brake in velocity mode, then converge to the commanded ideal endpoint."""
    for node in nodes:
        node.set_phase(phase, duration)
        node.command_velocity = (0.0, 0.0, 0.0)
    stable_since: dict[int, float | None] = {node.index: None for node in nodes}
    active_targets: dict[int, object] = {}
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        drain_callbacks(nodes)
        now = time.monotonic()
        for node in nodes:
            if node.index in active_targets:
                node.send_position_hold(active_targets[node.index])
                continue
            node.send_active_brake()
            if node.is_nearly_stopped():
                stable_since[node.index] = stable_since[node.index] or now
                if now - float(stable_since[node.index]) >= 0.25:
                    target = (
                        deepcopy(ideal_targets[node.index])
                        if ideal_targets is not None
                        else node.capture_position_hold()
                    )
                    if target is not None:
                        active_targets[node.index] = deepcopy(target)
            else:
                stable_since[node.index] = None
        time.sleep(0.02)


def phase_rows(node: CandidateNode, phase: str, trim_seconds: float = 0.0):
    rows = [row for row in node.samples if row["phase"] == phase]
    if rows and trim_seconds:
        threshold = float(rows[0]["time"]) + trim_seconds
        rows = [row for row in rows if float(row["time"]) >= threshold]
    return rows


def hover_metrics(rows) -> dict[str, float | bool]:
    if len(rows) < 20:
        return {"valid": False, "score": 1.0e9}
    ref_count = min(20, len(rows))
    ref = {
        axis: statistics.fmean(float(row[axis]) for row in rows[:ref_count])
        for axis in ("x", "y", "z")
    }
    dx = [float(row["x"]) - ref["x"] for row in rows]
    dy = [float(row["y"]) - ref["y"] for row in rows]
    dz = [float(row["z"]) - ref["z"] for row in rows]
    radial = [math.hypot(x, y) for x, y in zip(dx, dy)]
    speed_xy = [math.hypot(float(row["vx"]), float(row["vy"])) for row in rows]
    accel_xy = [math.hypot(float(row["ax"]), float(row["ay"])) for row in rows]
    roll = [float(row["roll_deg"]) for row in rows]
    pitch = [float(row["pitch_deg"]) for row in rows]
    yaw = [math.radians(float(row["yaw_deg"])) for row in rows]
    yaw_unwrapped = [yaw[0]]
    for value in yaw[1:]:
        delta = math.atan2(math.sin(value - yaw_unwrapped[-1]), math.cos(value - yaw_unwrapped[-1]))
        yaw_unwrapped.append(yaw_unwrapped[-1] + delta)
    metrics = {
        "valid": True,
        "sample_count": len(rows),
        "xy_rms_m": rms(radial),
        "xy_max_m": max(radial),
        "z_rms_m": rms(dz),
        "z_max_abs_m": max(abs(value) for value in dz),
        "speed_xy_rms_mps": rms(speed_xy),
        "speed_xy_max_mps": max(speed_xy),
        "speed_z_rms_mps": rms(float(row["vz"]) for row in rows),
        "accel_xy_rms_mps2": rms(accel_xy),
        "accel_z_rms_mps2": rms(float(row["az"]) for row in rows),
        "roll_rms_deg": rms(roll),
        "pitch_rms_deg": rms(pitch),
        "yaw_peak_to_peak_deg": math.degrees(max(yaw_unwrapped) - min(yaw_unwrapped)),
        "angular_rate_rms_rps": rms(
            math.sqrt(float(row["wx"]) ** 2 + float(row["wy"]) ** 2 + float(row["wz"]) ** 2)
            for row in rows
        ),
    }
    normalized = (
        3.0 * (metrics["xy_rms_m"] / 0.03) ** 2
        + 4.0 * (metrics["xy_max_m"] / 0.10) ** 2
        + 2.0 * (metrics["z_rms_m"] / 0.02) ** 2
        + 2.0 * (metrics["speed_xy_rms_mps"] / 0.04) ** 2
        + 1.0 * (metrics["accel_xy_rms_mps2"] / 0.35) ** 2
        + 1.0 * (metrics["roll_rms_deg"] / 0.4) ** 2
        + 1.0 * (metrics["pitch_rms_deg"] / 0.4) ** 2
        + 1.5 * (metrics["yaw_peak_to_peak_deg"] / 2.0) ** 2
    ) / 15.5
    metrics["score"] = normalized
    metrics["pass"] = (
        metrics["xy_max_m"] < 0.10
        and metrics["z_max_abs_m"] < 0.10
        and max(max(abs(value) for value in roll), max(abs(value) for value in pitch)) < 12.0
        and metrics["yaw_peak_to_peak_deg"] < 5.0
    )
    return metrics


def stop_metrics(rows) -> dict[str, float]:
    if len(rows) < 2:
        return {"max_displacement_m": math.inf, "settling_time_s": math.inf}
    x0, y0, z0, t0 = (float(rows[0][key]) for key in ("x", "y", "z", "time"))
    displacement = [
        math.sqrt(
            (float(row["x"]) - x0) ** 2
            + (float(row["y"]) - y0) ** 2
            + (float(row["z"]) - z0) ** 2
        )
        for row in rows
    ]
    settled = math.inf
    for index, row in enumerate(rows):
        window = rows[index : index + 20]
        if len(window) >= 10 and max(
            math.sqrt(float(item["vx"]) ** 2 + float(item["vy"]) ** 2 + float(item["vz"]) ** 2)
            for item in window
        ) <= 0.08:
            settled = float(row["time"]) - t0
            break
    z = [float(row["z"]) for row in rows]
    metrics = {
        "max_displacement_m": max(displacement),
        "settling_time_s": settled,
        "max_altitude_rise_m": max(z) - z[0],
        "max_abs_altitude_error_m": max(abs(value - z[0]) for value in z),
    }
    if "position_error_3d" in rows[0]:
        reference = trajectory_error_metrics(rows)
        metrics.update({
            "ideal_position_max_error_m": reference["position_max_error_m"],
            "ideal_position_final_error_m": reference["position_final_error_m"],
            "ideal_position_tail_rmse_m": reference["position_tail_rmse_m"],
            "ideal_xy_final_error_m": reference["xy_final_error_m"],
            "ideal_z_final_error_m": abs(float(reference["z_final_error_m"])),
        })
    return metrics


def dynamic_metrics(motions: dict, stops: dict) -> dict[str, float]:
    valid_motion = [value for value in motions.values() if value.get("valid", False)]
    horizontal_motion = [
        value for phase, value in motions.items()
        if value.get("valid", False)
        and phase.startswith(("move_", "stress_", "high_speed_", "abrupt_", "yaw_move_"))
    ]
    vertical_motion = [
        value for phase, value in motions.items()
        if value.get("valid", False) and phase.startswith("vertical_")
    ]
    high_speed_motion = [
        value for phase, value in motions.items()
        if value.get("valid", False) and phase.startswith("high_speed_")
    ]
    abrupt_motion = [
        value for phase, value in motions.items()
        if value.get("valid", False) and phase.startswith("abrupt_")
    ]
    moving_yaw_motion = [
        value for phase, value in motions.items()
        if value.get("valid", False) and phase.startswith("yaw_move_")
    ]
    standard_stops = [
        value for phase, value in stops.items()
        if phase.removeprefix("stop_").isdigit()
    ]
    reversal_stops = [value for phase, value in stops.items() if phase == "stop_reversals"]
    high_speed_stops = [value for phase, value in stops.items() if phase.startswith("stop_high_speed_")]
    abrupt_stops = [value for phase, value in stops.items() if phase.startswith("stop_abrupt_")]
    vertical_stops = [value for phase, value in stops.items() if phase.startswith("stop_vertical_")]
    horizontal_stops = standard_stops + reversal_stops + high_speed_stops + abrupt_stops
    valid_stops = list(stops.values())
    if not valid_motion:
        return {}
    return {
        "tracking_position_rmse_m": statistics.fmean(float(v["position_rmse_m"]) for v in valid_motion),
        "tracking_velocity_rmse_mps": statistics.fmean(float(v["velocity_rmse_mps"]) for v in valid_motion),
        "steady_velocity_rmse_mps": statistics.fmean(float(v["steady_velocity_rmse_mps"]) for v in valid_motion),
        "cross_axis_rms_mps": statistics.fmean(float(v["cross_axis_rms_mps"]) for v in valid_motion),
        "steady_cross_axis_rms_mps": statistics.fmean(float(v["steady_cross_axis_rms_mps"]) for v in valid_motion),
        "max_altitude_rise_m": max(float(v["max_altitude_rise_m"]) for v in horizontal_motion),
        "max_abs_altitude_error_m": max(float(v["max_altitude_deviation_m"]) for v in horizontal_motion),
        "high_speed_altitude_deviation_m": max(
            (float(v["max_altitude_deviation_m"]) for v in high_speed_motion), default=0.0
        ),
        "abrupt_reversal_altitude_deviation_m": max(
            (float(v["max_altitude_deviation_m"]) for v in abrupt_motion), default=0.0
        ),
        "moving_yaw_altitude_deviation_m": max(
            (float(v["max_altitude_deviation_m"]) for v in moving_yaw_motion), default=0.0
        ),
        "vertical_tracking_rmse_mps": statistics.fmean(
            float(v["steady_velocity_rmse_mps"]) for v in vertical_motion
        ) if vertical_motion else 0.0,
        "velocity_jerk_rms_mps3": statistics.fmean(float(v["velocity_jerk_rms_mps3"]) for v in valid_motion),
        "worst_stop_displacement_m": max((float(v["max_displacement_m"]) for v in valid_stops), default=0.0),
        "worst_stop_settling_time_s": max((float(v["settling_time_s"]) for v in valid_stops), default=0.0),
        "worst_stop_altitude_rise_m": max((float(v["max_altitude_rise_m"]) for v in valid_stops), default=0.0),
        "standard_stop_displacement_m": max(
            (float(v["max_displacement_m"]) for v in standard_stops), default=0.0
        ),
        "reversal_stop_displacement_m": max(
            (float(v["max_displacement_m"]) for v in reversal_stops), default=0.0
        ),
        "high_speed_stop_displacement_m": max(
            (float(v["max_displacement_m"]) for v in high_speed_stops), default=0.0
        ),
        "abrupt_stop_displacement_m": max(
            (float(v["max_displacement_m"]) for v in abrupt_stops), default=0.0
        ),
        "horizontal_stop_altitude_rise_m": max(
            (float(v["max_altitude_rise_m"]) for v in horizontal_stops), default=0.0
        ),
        "vertical_stop_overshoot_m": max(
            (float(v["max_abs_altitude_error_m"]) for v in vertical_stops), default=0.0
        ),
        "worst_ideal_position_final_error_m": max(
            (float(v.get("ideal_position_final_error_m", math.inf)) for v in valid_stops),
            default=math.inf,
        ),
        "worst_ideal_position_tail_rmse_m": max(
            (float(v.get("ideal_position_tail_rmse_m", math.inf)) for v in valid_stops),
            default=math.inf,
        ),
    }


def yaw_metrics(rows, target_world_deg: float) -> dict[str, float | bool]:
    if len(rows) < 10:
        return {"valid": False, "yaw_rmse_deg": math.inf}
    wrapped_yaw = [float(row["yaw_deg"]) for row in rows]
    errors, wrapped_errors = continuous_angle_errors_degrees(wrapped_yaw, target_world_deg)
    initial_sign = math.copysign(1.0, errors[0]) if abs(errors[0]) > 1.0e-6 else 0.0
    overshoot = max(
        (abs(error) for error in errors if initial_sign and error * initial_sign < 0.0),
        default=0.0,
    )
    x0, y0, z0 = (float(rows[0][name]) for name in ("x", "y", "z"))
    tail_count = max(5, int(len(rows) * 0.20))
    settled = math.inf
    for index in range(len(rows)):
        window = rows[index : index + max(5, int(len(rows) * 0.10))]
        if len(window) >= 5 and max(
            abs(wrapped_errors[index + offset]) for offset, _row in enumerate(window)
        ) < 3.0:
            settled = float(rows[index]["time"]) - float(rows[0]["time"])
            break
    return {
        "valid": True,
        "yaw_rmse_deg": rms(errors),
        "yaw_final_error_deg": abs(wrapped_errors[-1]),
        "yaw_max_error_deg": max(abs(value) for value in errors),
        "yaw_overshoot_deg": overshoot,
        "yaw_tail_peak_to_peak_deg": (
            max(wrapped_errors[-tail_count:]) - min(wrapped_errors[-tail_count:])
        ),
        "yaw_rate_peak_dps": max(abs(math.degrees(float(row["wz"]))) for row in rows),
        "yaw_rate_tail_rms_dps": rms(
            [math.degrees(float(row["wz"])) for row in rows[-tail_count:]]
        ),
        "settling_time_s": settled,
        "xy_drift_m": max(
            math.hypot(float(row["x"]) - x0, float(row["y"]) - y0) for row in rows
        ),
        "max_altitude_deviation_m": max(abs(float(row["z"]) - z0) for row in rows),
    }


def write_candidate(node: CandidateNode, output: Path, candidate: dict) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    rows = add_reference_signals(
        [row for row in node.samples if row["phase"] not in ("preflight", "warmup")]
    )
    csv_path = output / f"env_{node.index}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    hover = hover_metrics(phase_rows(node, "hover", trim_seconds=2.0))
    stops = {
        phase: stop_metrics([row for row in rows if row["phase"] == phase])
        for phase in sorted({str(row["phase"]) for row in rows if str(row["phase"]).startswith("stop_")})
    }
    motions = {
        phase: motion_metrics([row for row in rows if row["phase"] == phase])
        for phase in sorted({
            str(row["phase"]) for row in rows
            if str(row["phase"]).startswith(
                ("move_", "stress_", "high_speed_", "abrupt_", "vertical_", "yaw_move_")
            )
        })
    }
    yaw_targets = {
        "yaw_plus": 90.0,
        "yaw_minus": -90.0,
        "yaw_repeat_plus": 90.0,
        "yaw_repeat_minus": -90.0,
        "yaw_half_turn": 180.0,
        "yaw_return": 0.0,
        "yaw_move_plus": 90.0,
        "yaw_move_minus": -90.0,
    }
    yaw = {
        phase: yaw_metrics(phase_rows(node, phase, trim_seconds=0.5), target)
        for phase, target in yaw_targets.items()
        if any(row["phase"] == phase for row in rows)
    }
    dynamic = dynamic_metrics(motions, stops)
    trajectory_rows = [row for row in rows if row["phase"] != "hover"]
    trajectory = trajectory_error_metrics(trajectory_rows)
    trajectory_by_phase = {
        phase: trajectory_error_metrics([row for row in rows if row["phase"] == phase])
        for phase in sorted({str(row["phase"]) for row in trajectory_rows})
    }
    wall_duration = float(rows[-1]["time"]) - float(rows[0]["time"])
    sim_duration = float(rows[-1]["sim_time"]) - float(rows[0]["sim_time"])
    timing = {
        "real_time_factor": sim_duration / wall_duration if wall_duration > 0.0 else 0.0,
        "telemetry_samples_per_wall_second": len(rows) / wall_duration if wall_duration > 0.0 else 0.0,
    }
    motion_penalty = 0.0
    if stops:
        motion_penalty = statistics.fmean(
            min(5.0, value["max_displacement_m"] / 0.4)
            + min(5.0, value["settling_time_s"] / 6.0)
            for value in stops.values()
        )
    tracking_penalty = statistics.fmean(
        min(10.0, float(value["velocity_rmse_mps"]) / 0.50)
        + min(10.0, float(value["steady_velocity_rmse_mps"]) / 0.15)
        + min(10.0, float(value["position_rmse_m"]) / 0.20)
        + min(10.0, float(value["steady_cross_axis_rms_mps"]) / 0.08)
        + min(10.0, float(value["max_altitude_deviation_m"]) / 0.08)
        + min(10.0, float(value["velocity_jerk_rms_mps3"]) / 8.0)
        for value in motions.values()
    ) if motions else 0.0
    yaw_penalty = statistics.fmean(
        min(10.0, float(value["yaw_rmse_deg"]) / 5.0)
        + min(10.0, float(value["yaw_final_error_deg"]) / 3.0)
        + min(10.0, float(value["yaw_tail_peak_to_peak_deg"]) / 2.0)
        + min(10.0, float(value["max_altitude_deviation_m"]) / 0.03)
        for value in yaw.values()
    ) if yaw else 0.0
    aggressive_altitude_values = [
        float(value["max_altitude_deviation_m"])
        for phase, value in motions.items()
        if phase.startswith(("high_speed_", "abrupt_", "yaw_move_"))
    ] + [
        float(value["max_abs_altitude_error_m"])
        for phase, value in stops.items()
        if phase.startswith(("stop_high_speed_", "stop_abrupt_", "stop_yaw_move"))
    ]
    aggressive_altitude_penalty = statistics.fmean(
        min(10.0, value / 0.05) for value in aggressive_altitude_values
    ) if aggressive_altitude_values else 0.0
    convergence_penalty = statistics.fmean(
        min(20.0, float(value.get("ideal_position_tail_rmse_m", math.inf)) / 0.10)
        + min(20.0, float(value.get("ideal_position_final_error_m", math.inf)) / 0.10)
        for value in stops.values()
    ) if stops else 0.0
    trajectory_penalty = (
        min(20.0, float(trajectory.get("position_rmse_m", math.inf)) / 0.25)
        + min(20.0, float(trajectory.get("position_max_error_m", math.inf)) / 1.00)
        + min(20.0, float(trajectory.get("z_max_abs_error_m", math.inf)) / 0.10)
    ) if trajectory_rows else 0.0
    basic_motion_pass = all(
        value.get("valid", False)
        and float(value["steady_velocity_rmse_mps"]) < 0.20
        and float(value["max_altitude_deviation_m"]) < 0.08
        and float(value["steady_cross_axis_rms_mps"]) < 0.12
        and float(value["max_tilt_deg"]) < 35.0
        for phase, value in motions.items() if phase.startswith("move_")
    )
    stress_pass = all(
        value.get("valid", False)
        and float(value["max_altitude_deviation_m"]) < 0.15
        # These are consecutive 2 m/s reversals, not settled cruise legs. The
        # threshold detects instability while allowing a safe baseline to be
        # improved incrementally by the objective and fresh-run comparison.
        and float(value["steady_velocity_rmse_mps"]) < 0.70
        # During the chained stress sequence, an orthogonal command arrives
        # while velocity from the previous 2 m/s leg is still decaying. That
        # residual is deliberately scored, but is not a hard instability
        # unless it remains large. Keep this guard above the repeatable
        # 0.35-0.36 m/s transition residual seen by the stable incumbent.
        and float(value["steady_cross_axis_rms_mps"]) < 0.40
        and float(value["max_tilt_deg"]) < 45.0
        for phase, value in motions.items() if phase.startswith("stress_")
    )
    high_speed_pass = all(
        value.get("valid", False)
        and float(value["max_altitude_deviation_m"]) < 0.05
        and float(value["steady_cross_axis_rms_mps"]) < 0.25
        and float(value["max_tilt_deg"]) < 45.0
        for phase, value in motions.items() if phase.startswith("high_speed_")
    )
    abrupt_reversal_pass = all(
        value.get("valid", False)
        and float(value["max_altitude_deviation_m"]) < 0.05
        and float(value["max_tilt_deg"]) < 50.0
        for phase, value in motions.items() if phase.startswith("abrupt_")
    )
    vertical_pass = all(
        value.get("valid", False)
        and float(value["steady_velocity_rmse_mps"]) < 0.30
        and float(value["steady_cross_axis_rms_mps"]) < 0.12
        for phase, value in motions.items() if phase.startswith("vertical_")
    )
    yaw_pass = all(
        # Broad translation stages guard against a severe yaw failure; the
        # dedicated yaw stage and yaw penalty rank sub-5-degree accuracy.
        value.get("valid", False)
        and float(value["yaw_final_error_deg"]) < 5.0
        and float(value["yaw_tail_peak_to_peak_deg"]) < 3.0
        and float(value["yaw_overshoot_deg"]) < 8.0
        and float(value["max_altitude_deviation_m"]) < 0.05
        and (phase.startswith("yaw_move_") or float(value["xy_drift_m"]) < 0.10)
        for phase, value in yaw.items()
    )
    standard_stop_pass = all(
        value["max_displacement_m"] < 0.75
        for phase, value in stops.items()
        if phase.removeprefix("stop_").isdigit()
    )
    reversal_stop_pass = all(
        value["max_displacement_m"] < 1.50
        for phase, value in stops.items() if phase == "stop_reversals"
    )
    high_speed_stop_pass = all(
        value["max_displacement_m"] < 1.75
        and value["max_abs_altitude_error_m"] < 0.08
        for phase, value in stops.items() if phase.startswith("stop_high_speed_")
    )
    abrupt_stop_pass = all(
        value["max_displacement_m"] < 2.0
        and value["max_abs_altitude_error_m"] < 0.08
        for phase, value in stops.items() if phase.startswith("stop_abrupt_")
    )
    vertical_stop_pass = all(
        value["max_abs_altitude_error_m"] < 0.45
        for phase, value in stops.items() if phase.startswith("stop_vertical_")
    )
    ideal_convergence_pass = all(
        float(value.get("ideal_position_tail_rmse_m", math.inf)) < 0.10
        and float(value.get("ideal_position_final_error_m", math.inf)) < 0.10
        for value in stops.values()
    )
    timing_penalty = max(0.0, 0.98 - timing["real_time_factor"]) * 50.0
    gates = {
        "hover": {
            "pass": bool(hover.get("pass", False)),
            "limits": {"xy_max_m": 0.10, "z_max_abs_m": 0.10, "yaw_peak_to_peak_deg": 5.0},
        },
        "real_time": {
            "pass": timing["real_time_factor"] >= 0.95,
            "value": timing["real_time_factor"],
            "minimum": 0.95,
        },
        "basic_motion": {"pass": basic_motion_pass},
        "stress_motion": {"pass": stress_pass},
        "high_speed_motion": {
            "pass": high_speed_pass,
            "altitude_deviation_limit_m": 0.05,
        },
        "abrupt_reversals": {
            "pass": abrupt_reversal_pass,
            "altitude_deviation_limit_m": 0.05,
        },
        "vertical_motion": {"pass": vertical_pass},
        "yaw": {
            "pass": yaw_pass,
            "final_error_limit_deg": 5.0,
            "tail_peak_to_peak_limit_deg": 3.0,
            "overshoot_limit_deg": 8.0,
            "static_xy_drift_limit_m": 0.10,
            "altitude_deviation_limit_m": 0.05,
        },
        "standard_stops": {"pass": standard_stop_pass, "limit_m": 0.75},
        "reversal_stop": {"pass": reversal_stop_pass, "limit_m": 1.50},
        "high_speed_stops": {"pass": high_speed_stop_pass, "limit_m": 1.75},
        "abrupt_stops": {"pass": abrupt_stop_pass, "limit_m": 2.0},
        "vertical_stops": {"pass": vertical_stop_pass, "limit_m": 0.45},
        "ideal_position_convergence": {
            "pass": ideal_convergence_pass,
            "tail_rmse_limit_m": 0.10,
            "final_error_limit_m": 0.10,
            "definition": "Euclidean norm of Isaac ground truth minus continuous ideal position",
        },
    }
    all_gates_pass = all(value["pass"] for value in gates.values())
    total_score = (
        float(hover["score"])
        + 0.35 * motion_penalty
        + 0.40 * tracking_penalty
        + 0.15 * yaw_penalty
        + 1.25 * aggressive_altitude_penalty
        + 1.50 * convergence_penalty
        + 0.75 * trajectory_penalty
        + timing_penalty
    )
    summary = {
        "environment": node.index,
        "synchronization": {
            "takeoff_command_monotonic_s": node.takeoff_command_time,
        },
        "candidate": candidate,
        "hover": hover,
        "stops": stops,
        "motion": motions,
        "dynamic": dynamic,
        "yaw": yaw,
        "trajectory": trajectory,
        "trajectory_by_phase": trajectory_by_phase,
        "timing": timing,
        "gates": gates,
        "objective": {
            "hover": float(hover["score"]),
            "braking_penalty": motion_penalty,
            "tracking_penalty": tracking_penalty,
            "yaw_penalty": yaw_penalty,
            "aggressive_altitude_penalty": aggressive_altitude_penalty,
            "ideal_position_convergence_penalty": convergence_penalty,
            "continuous_trajectory_penalty": trajectory_penalty,
            "timing_penalty": timing_penalty,
            "total": total_score,
        },
        "score": total_score,
        "pass": all_gates_pass,
    }
    (output / f"env_{node.index}.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n"
    )
    return summary


def wait_at_barrier(barrier, timeout: float = 150.0) -> None:
    """Wait for a fleet synchronization point, tolerating an aborted peer."""
    if barrier is None:
        return
    try:
        barrier.wait(timeout=timeout)
    except BrokenBarrierError:
        pass


def run_candidate_process(
    index: int,
    args,
    candidate: dict,
    takeoff_barrier,
    airborne_barrier,
    finish_barrier,
    errors,
) -> None:
    """Control one drone in an isolated rclpy process and record its trial."""
    node = None
    try:
        rclpy.init()
        node = CandidateNode(index, args.output_dir, candidate)
        node.set_phase("preflight")
        node.prepare_guided(takeoff=False)
        node.set_phase("takeoff_sync")
        # Arm every vehicle first, then release all takeoff requests together.
        if takeoff_barrier is not None:
            wait_at_barrier(takeoff_barrier)
        node.set_phase("takeoff")
        node.takeoff_and_wait()
        node.takeoff_command_time = node.last_takeoff_request_time
        node.set_phase("airborne_sync")
        # Do not start hover/motion until all nine have reached stable altitude.
        if airborne_barrier is not None:
            wait_at_barrier(airborne_barrier, timeout=30.0)
        nodes = [node]
        spin_nodes(nodes, 1.0)
        target = node.capture_position_hold()
        if target is None:
            raise RuntimeError("No local position was available for hover")
        targets = {index: deepcopy(target)}
        spin_position_hold(nodes, "warmup", args.warmup_seconds, targets)
        spin_nodes(nodes, 0.5)
        target = node.capture_position_hold()
        if target is None:
            raise RuntimeError("No local position was available after warmup")
        targets = {index: deepcopy(target)}
        spin_position_hold(nodes, "hover", args.hover_seconds, targets)

        if args.protocol == "demo":
            # A compact, visually legible fleet demonstration. Paired motions
            # return every vehicle near its starting XY/Z position before the
            # final landing, while keeping identical spacing between drones.
            demo_speed = min(0.8, max(0.4, args.speed))
            demo_segments = (
                ("move_0", (0.0, demo_speed, 0.0), 2.5),
                ("move_1", (0.0, -demo_speed, 0.0), 5.0),
                ("move_2", (0.0, demo_speed, 0.0), 2.5),
                ("move_3", (demo_speed, 0.0, 0.0), 2.5),
                ("move_4", (-demo_speed, 0.0, 0.0), 5.0),
                ("move_5", (demo_speed, 0.0, 0.0), 2.5),
            )
            for phase, velocity, duration in demo_segments:
                spin_and_command(
                    nodes, phase, duration,
                    lambda _node, value=velocity: value,
                    targets,
                )
                if phase in ("move_2", "move_5"):
                    spin_brake_and_hold(nodes, f"stop_{phase[-1]}", 4.0, targets)

            vertical_speed = 0.35
            for phase, velocity in (
                ("vertical_up", (0.0, 0.0, vertical_speed)),
                ("vertical_down", (0.0, 0.0, -vertical_speed)),
            ):
                spin_and_command(
                    nodes, phase, 2.0,
                    lambda _node, value=velocity: value,
                    targets,
                )
                spin_brake_and_hold(nodes, f"stop_{phase}", 4.0, targets)
            spin_position_hold(nodes, "final_hover", 4.0, targets)
            node.set_phase("landing_sync")
            # Keep all nine aircraft in formation until every worker has
            # completed every maneuver. Their finally blocks then request LAND
            # together instead of allowing fast workers to descend early.
            wait_at_barrier(finish_barrier, timeout=30.0)

        if args.protocol in ("basic", "motion", "stress"):
            directions = ((args.speed, 0.0, 0.0), (-args.speed, 0.0, 0.0),
                          (0.0, args.speed, 0.0), (0.0, -args.speed, 0.0))
            for direction_index, velocity in enumerate(directions):
                spin_and_command(
                    nodes, f"move_{direction_index}", 3.0,
                    lambda _node, value=velocity: value,
                    targets,
                )
                spin_brake_and_hold(nodes, f"stop_{direction_index}", 6.0, targets)

            # Keep the accumulated commanded endpoint through the yaw tests.
            # Re-capturing the measured pose here would hide residual position
            # error by redefining the ideal trajectory after translation.
            spin_nodes(nodes, 0.5)
            spin_position_hold(
                nodes, "yaw_plus", 8.0, targets,
                ISAAC_WORLD_ZERO_YAW_NED + math.pi / 2.0,
            )
            spin_position_hold(
                nodes, "yaw_minus", 9.0, targets,
                ISAAC_WORLD_ZERO_YAW_NED - math.pi / 2.0,
            )
            spin_position_hold(
                nodes, "yaw_repeat_plus", 9.0, targets,
                ISAAC_WORLD_ZERO_YAW_NED + math.pi / 2.0,
            )
            spin_position_hold(
                nodes, "yaw_repeat_minus", 9.0, targets,
                ISAAC_WORLD_ZERO_YAW_NED - math.pi / 2.0,
            )
            spin_position_hold(
                nodes, "yaw_half_turn", 8.0, targets,
                ISAAC_WORLD_ZERO_YAW_NED + math.pi,
            )
            spin_position_hold(nodes, "yaw_return", 9.0, targets)

        if args.protocol == "stress":
            stress_speed = max(2.0, args.speed * 2.0)
            reversals = (
                (stress_speed, 0.0, 0.0), (-stress_speed, 0.0, 0.0),
                (0.0, stress_speed, 0.0), (0.0, -stress_speed, 0.0),
                (stress_speed * 0.707, stress_speed * 0.707, 0.0),
                (-stress_speed * 0.707, -stress_speed * 0.707, 0.0),
            )
            for phase_index, velocity in enumerate(reversals):
                spin_and_command(
                    nodes, f"stress_{phase_index}", 2.0,
                    lambda _node, value=velocity: value,
                    targets,
                )
            spin_brake_and_hold(nodes, "stop_reversals", 8.0, targets)

            # Sustained 3 m/s cardinal motion checks tracking and altitude
            # coupling independently from the immediate reversal test.
            high_speed = args.high_speed
            high_speed_directions = (
                (high_speed, 0.0, 0.0),
                (-high_speed, 0.0, 0.0),
                (0.0, high_speed, 0.0),
                (0.0, -high_speed, 0.0),
            )
            for phase_index, velocity in enumerate(high_speed_directions):
                spin_and_command(
                    nodes, f"high_speed_{phase_index}", 2.5,
                    lambda _node, value=velocity: value,
                    targets,
                )
                spin_brake_and_hold(
                    nodes, f"stop_high_speed_{phase_index}", 7.0, targets
                )

            # Each command changes directly to its exact opposite without a
            # zero-velocity interval. Altitude must remain within five cm.
            abrupt_axes = (
                (high_speed, 0.0, 0.0),
                (0.0, high_speed, 0.0),
                (high_speed * 0.707, high_speed * 0.707, 0.0),
            )
            for phase_index, outward in enumerate(abrupt_axes):
                reverse = tuple(-value for value in outward)
                spin_and_command(
                    nodes, f"abrupt_{phase_index}_out", 2.0,
                    lambda _node, value=outward: value,
                    targets,
                )
                spin_and_command(
                    nodes, f"abrupt_{phase_index}_reverse", 2.0,
                    lambda _node, value=reverse: value,
                    targets,
                )
                spin_brake_and_hold(nodes, f"stop_abrupt_{phase_index}", 8.0, targets)

            # Rotate while translating to expose yaw/position/throttle coupling.
            spin_velocity_and_yaw(
                nodes, "yaw_move_plus", 8.0, (args.speed, 0.0, 0.0),
                ISAAC_WORLD_ZERO_YAW_NED + math.pi / 2.0,
                targets,
            )
            spin_velocity_and_yaw(
                nodes, "yaw_move_minus", 9.0, (-args.speed, 0.0, 0.0),
                ISAAC_WORLD_ZERO_YAW_NED - math.pi / 2.0,
                targets,
            )
            spin_brake_and_hold(nodes, "stop_yaw_move", 7.0, targets)

            for phase_index, velocity in enumerate(
                ((0.0, 0.0, 0.75), (0.0, 0.0, -0.75))
            ):
                spin_and_command(
                    nodes, f"vertical_{phase_index}", 1.5,
                    lambda _node, value=velocity: value,
                    targets,
                )
                spin_brake_and_hold(nodes, f"stop_vertical_{phase_index}", 5.0, targets)

        summary = write_candidate(node, args.output_dir, candidate)
        node.write_progress(
            "complete",
            force=True,
            result={
                "pass": summary["pass"],
                "score": summary["score"],
                "hover_xy_max_m": summary["hover"].get("xy_max_m"),
                "hover_z_max_m": summary["hover"].get("z_max_abs_m"),
                "real_time_factor": summary["timing"].get("real_time_factor"),
                "standard_stop_m": summary["dynamic"].get("standard_stop_displacement_m"),
                "reversal_stop_m": summary["dynamic"].get("reversal_stop_displacement_m"),
                "high_speed_altitude_m": summary["dynamic"].get(
                    "high_speed_altitude_deviation_m"
                ),
                "abrupt_altitude_m": summary["dynamic"].get(
                    "abrupt_reversal_altitude_deviation_m"
                ),
                "yaw_final_error_deg": max(
                    (value.get("yaw_final_error_deg", 0.0) for value in summary["yaw"].values()),
                    default=0.0,
                ),
            },
        )
    except BaseException as error:
        detail = traceback.format_exc()
        errors.put((index, detail))
        if node is not None:
            node.write_progress("failed", force=True, message=str(error))
        else:
            atomic_json(
                args.output_dir / f"env_{index}_progress.json",
                {
                    "drone": index,
                    "candidate": candidate.get("name", "unknown"),
                    "status": "failed",
                    "phase": "startup",
                    "message": str(error),
                    "updated_at": datetime_now(),
                },
            )
        try:
            if takeoff_barrier is not None:
                takeoff_barrier.abort()
            if airborne_barrier is not None:
                airborne_barrier.abort()
            if finish_barrier is not None:
                finish_barrier.abort()
        except Exception:
            pass
        raise
    finally:
        if node is not None:
            try:
                node.request_land()
                if args.protocol == "demo":
                    node.set_phase("landing", 25.0)
                    landing_deadline = time.monotonic() + 25.0
                    while rclpy.ok() and node.state.armed and time.monotonic() < landing_deadline:
                        node.spin_once(timeout_sec=0.1)
                    # Keep the completed touchdown visible in the recording.
                    spin_nodes([node], 2.0)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def print_batch_summary(report: dict) -> None:
    print("\nPer-drone results:", flush=True)
    print(
        "drone candidate                     pass score   hover_xy hover_z  "
        "stop     reversal fast_z  abrupt_z yaw_err RTF    failed_gates",
        flush=True,
    )
    for item in sorted(report["environment_ranking"], key=lambda value: value["environment"]):
        failed_gates = ",".join(
            name for name, gate in item.get("gates", {}).items() if not gate.get("pass", False)
        ) or "-"
        print(
            f"{item['environment']:>5} {item['candidate']['name'][:29]:<29} "
            f"{str(item['pass']):>4} {item['score']:>7.3f} "
            f"{item['hover'].get('xy_max_m', math.nan):>8.4f} "
            f"{item['hover'].get('z_max_abs_m', math.nan):>7.4f} "
            f"{item['dynamic'].get('standard_stop_displacement_m', math.nan):>8.3f} "
            f"{item['dynamic'].get('reversal_stop_displacement_m', math.nan):>8.3f} "
            f"{item['dynamic'].get('high_speed_altitude_deviation_m', math.nan):>7.3f} "
            f"{item['dynamic'].get('abrupt_reversal_altitude_deviation_m', math.nan):>8.3f} "
            f"{max((value.get('yaw_final_error_deg', 0.0) for value in item.get('yaw', {}).values()), default=math.nan):>7.2f} "
            f"{item['timing'].get('real_time_factor', 0.0):>6.3f} {failed_gates}",
            flush=True,
        )
    print("\nCandidate ranking:", flush=True)
    for rank, item in enumerate(report["ranking"], start=1):
        print(
            f"  {rank}. {item['candidate']['name']}: score={item['score']:.4f}, "
            f"pass={item['pass']}, replicas={item['replica_count']}, "
            f"worst_RTF={item['timing']['real_time_factor_worst']:.3f}, "
            f"worst_hover_xy={item['hover_worst'].get('xy_max_m', math.nan):.4f} m, "
            f"high_speed_z={item['dynamic_worst'].get('high_speed_altitude_deviation_m', math.nan):.4f} m, "
            f"abrupt_z={item['dynamic_worst'].get('abrupt_reversal_altitude_deviation_m', math.nan):.4f} m, "
            f"yaw_error={item.get('yaw_worst', {}).get('final_error_deg', math.nan):.2f} deg",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-count", type=int, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--warmup-seconds", type=float, default=12.0)
    parser.add_argument("--hover-seconds", type=float, default=35.0)
    parser.add_argument(
        "--protocol", choices=("hover", "demo", "basic", "motion", "stress"), default="hover"
    )
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--high-speed", type=float, default=3.0)
    args = parser.parse_args()
    if args.high_speed < max(2.0, args.speed):
        parser.error("high-speed must be at least 2 m/s and not lower than speed")
    manifest = json.loads(args.manifest.read_text())
    if len(manifest["candidates"]) != args.env_count:
        parser.error("manifest candidate count does not match env-count")

    context = mp.get_context("spawn")
    takeoff_barrier = context.Barrier(args.env_count)
    airborne_barrier = context.Barrier(args.env_count)
    finish_barrier = context.Barrier(args.env_count) if args.protocol == "demo" else None
    errors = context.Queue()
    processes = [
        context.Process(
            target=run_candidate_process,
            args=(
                index,
                args,
                manifest["candidates"][index],
                takeoff_barrier,
                airborne_barrier,
                finish_barrier,
                errors,
            ),
            name=f"pid-candidate-{index}",
        )
        for index in range(args.env_count)
    ]
    for process in processes:
        process.start()
    for process in processes:
        process.join(timeout=360.0)
    for process in processes:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5.0)
    bad_indices = [index for index, process in enumerate(processes) if process.exitcode]
    # Isolated retries are useful for independent PID datasets, but they are
    # invalid for a fleet demonstration: a retried drone would take off while
    # its peers are landing. Let run_batch restart the complete demo instead.
    retry_attempts = 0 if args.protocol == "demo" else 2
    for attempt in range(1, retry_attempts + 1):
        if not bad_indices:
            break
        retry_indices = list(bad_indices)
        bad_indices = []
        for index in retry_indices:
            for suffix in ("json", "csv"):
                (args.output_dir / f"env_{index}.{suffix}").unlink(missing_ok=True)
            retry = context.Process(
                target=run_candidate_process,
                args=(index, args, manifest["candidates"][index], None, None, None, errors),
                name=f"pid-candidate-{index}-retry-{attempt}",
            )
            retry.start()
            retry.join(timeout=360.0)
            if retry.is_alive():
                retry.terminate()
                retry.join(timeout=5.0)
            if retry.exitcode:
                bad_indices.append(index)
    failures = []
    while not errors.empty():
        failures.append(errors.get())
    bad_exits = [(f"pid-candidate-{index}", "retries exhausted") for index in bad_indices]
    if bad_exits:
        detail = "\n".join(f"drone {index}:\n{error}" for index, error in failures)
        raise RuntimeError(f"Parallel candidate workers failed: {bad_exits}\n{detail}")

    summaries = [
        json.loads((args.output_dir / f"env_{index}.json").read_text())
        for index in range(args.env_count)
    ]
    ranking = aggregate_candidates(summaries)
    report = {
        "protocol": args.protocol,
        "speed_mps": args.speed,
        "ranking": ranking,
        "environment_ranking": sorted(
            summaries, key=lambda value: (not value["pass"], value["score"])
        ),
    }
    (args.output_dir / "batch_summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    )
    print_batch_summary(report)
    if os.environ.get("PID_TUNING_VERBOSE_JSON") == "1":
        print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
