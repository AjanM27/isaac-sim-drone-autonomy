#!/usr/bin/env python3
"""Dependency-free tracking metrics shared by the PID runner and unit tests."""

from __future__ import annotations

import math
import statistics


def continuous_angle_errors_degrees(
    samples: list[float], target: float
) -> tuple[list[float], list[float]]:
    """Return continuous and wrapped errors without false ±180-degree jumps."""
    if not samples:
        return [], []
    unwrapped = [float(samples[0])]
    for previous, current in zip(samples, samples[1:]):
        delta = math.degrees(math.atan2(
            math.sin(math.radians(float(current) - float(previous))),
            math.cos(math.radians(float(current) - float(previous))),
        ))
        unwrapped.append(unwrapped[-1] + delta)
    target_delta = math.degrees(math.atan2(
        math.sin(math.radians(float(target) - float(samples[0]))),
        math.cos(math.radians(float(target) - float(samples[0]))),
    ))
    continuous_target = unwrapped[0] + target_delta
    continuous = [value - continuous_target for value in unwrapped]
    wrapped = [
        math.degrees(math.atan2(math.sin(math.radians(error)), math.cos(math.radians(error))))
        for error in continuous
    ]
    return continuous, wrapped


def rms(values) -> float:
    values = list(values)
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else 0.0


def low_pass_vectors(times: list[float], vectors: list[tuple[float, ...]], tau: float = 0.08):
    """Filter physics-rate velocity before differentiating it into jerk."""
    if not vectors:
        return []
    filtered = [vectors[0]]
    for index in range(1, len(vectors)):
        dt = max(1.0e-3, times[index] - times[index - 1])
        alpha = dt / (tau + dt)
        filtered.append(tuple(
            previous + alpha * (current - previous)
            for previous, current in zip(filtered[-1], vectors[index])
        ))
    return filtered


def add_reference_signals(rows: list[dict]) -> list[dict]:
    """Add one continuous ideal trajectory and actual-minus-ideal errors.

    The old implementation restarted the ideal position at the measured pose
    on every phase boundary. That made accumulated drift disappear from the
    score. Here the ideal starts once and is integrated from the command that
    was active over each simulation-time interval. A zero-velocity/braking
    command therefore freezes the expected endpoint instead of silently
    accepting wherever the aircraft happened to stop.
    """
    if not rows:
        return rows
    first = rows[0]
    ideal = [-float(first["y"]), float(first["x"]), float(first["z"])]
    previous_time = float(first["sim_time"])
    previous_command = tuple(float(first[f"command_v{axis}"]) for axis in "xyz")
    estimated0 = tuple(float(first[f"estimated_{axis}"]) for axis in "xyz")
    estimate_valid = all(math.isfinite(value) for value in estimated0)
    estimate_offset = tuple(estimated0[i] - ideal[i] for i in range(3))
    for row in rows:
        current_time = float(row["sim_time"])
        dt = max(0.0, current_time - previous_time)
        for index in range(3):
            ideal[index] += previous_command[index] * dt
        actual_p = (-float(row["y"]), float(row["x"]), float(row["z"]))
        actual_v = (-float(row["vy"]), float(row["vx"]), float(row["vz"]))
        for index, axis in enumerate("xyz"):
            error = actual_p[index] - ideal[index]
            row[f"enu_{axis}"] = actual_p[index]
            row[f"enu_v{axis}"] = actual_v[index]
            row[f"ideal_{axis}"] = ideal[index]
            row[f"position_error_{axis}"] = error
            estimate = float(row[f"estimated_{axis}"])
            row[f"aligned_estimated_{axis}"] = (
                estimate - estimate_offset[index]
                if estimate_valid and math.isfinite(estimate)
                else math.nan
            )
        row["position_error_xy"] = math.hypot(
            float(row["position_error_x"]), float(row["position_error_y"])
        )
        row["position_error_3d"] = math.sqrt(
            sum(float(row[f"position_error_{axis}"]) ** 2 for axis in "xyz")
        )
        previous_time = current_time
        previous_command = tuple(float(row[f"command_v{axis}"]) for axis in "xyz")
    return rows


def trajectory_error_metrics(rows: list[dict]) -> dict[str, float | bool]:
    """Summarize actual minus ideal position over a continuous command run."""
    if len(rows) < 2:
        return {"valid": False, "position_rmse_m": math.inf}
    errors = [float(row["position_error_3d"]) for row in rows]
    xy_errors = [float(row["position_error_xy"]) for row in rows]
    tail_count = max(5, int(len(rows) * 0.10))
    metrics: dict[str, float | bool] = {
        "valid": True,
        "position_rmse_m": rms(errors),
        "position_max_error_m": max(errors),
        "position_final_error_m": errors[-1],
        "position_tail_rmse_m": rms(errors[-tail_count:]),
        "xy_rmse_m": rms(xy_errors),
        "xy_max_error_m": max(xy_errors),
        "xy_final_error_m": xy_errors[-1],
    }
    for axis in "xyz":
        values = [float(row[f"position_error_{axis}"]) for row in rows]
        metrics[f"{axis}_rmse_m"] = rms(values)
        metrics[f"{axis}_max_abs_error_m"] = max(abs(value) for value in values)
        metrics[f"{axis}_final_error_m"] = values[-1]
    return metrics


def motion_metrics(rows) -> dict[str, float | bool]:
    if len(rows) < 10:
        return {"valid": False, "velocity_rmse_mps": math.inf}
    velocity_errors = []
    position_errors = []
    cross = []
    axis_velocity_errors = {axis: [] for axis in "xyz"}
    axis_position_errors = {axis: [] for axis in "xyz"}
    z0 = float(rows[0]["z"])
    for row in rows:
        actual = tuple(float(row[f"enu_v{axis}"]) for axis in "xyz")
        actual_position = tuple(float(row[f"enu_{axis}"]) for axis in "xyz")
        ideal_position = tuple(float(row[f"ideal_{axis}"]) for axis in "xyz")
        command = tuple(float(row[f"command_v{axis}"]) for axis in "xyz")
        velocity_errors.append(math.sqrt(sum((a - c) ** 2 for a, c in zip(actual, command))))
        position_errors.append(
            math.sqrt(sum((a - c) ** 2 for a, c in zip(actual_position, ideal_position)))
        )
        for index, axis in enumerate("xyz"):
            axis_velocity_errors[axis].append(actual[index] - command[index])
            axis_position_errors[axis].append(actual_position[index] - ideal_position[index])
        command_norm = math.sqrt(sum(value * value for value in command))
        if command_norm > 1.0e-6:
            unit = tuple(value / command_norm for value in command)
            along = sum(a * u for a, u in zip(actual, unit))
            cross.append(math.sqrt(max(0.0, sum(a * a for a in actual) - along * along)))

    times = [float(row["sim_time"]) for row in rows]
    velocities = [
        tuple(float(row[f"enu_v{axis}"]) for axis in "xyz") for row in rows
    ]
    filtered_velocity = low_pass_vectors(times, velocities)
    acceleration = []
    for index in range(1, len(rows)):
        dt = max(1.0e-3, times[index] - times[index - 1])
        acceleration.append(tuple(
            (filtered_velocity[index][axis] - filtered_velocity[index - 1][axis]) / dt
            for axis in range(3)
        ))
    jerk = []
    for index in range(1, len(acceleration)):
        dt = max(1.0e-3, times[index + 1] - times[index])
        jerk.append(math.sqrt(sum(
            ((acceleration[index][axis] - acceleration[index - 1][axis]) / dt) ** 2
            for axis in range(3)
        )))

    estimated_errors = []
    for row in rows:
        estimate = tuple(float(row[f"aligned_estimated_{axis}"]) for axis in "xyz")
        if all(math.isfinite(value) for value in estimate):
            actual_position = tuple(float(row[f"enu_{axis}"]) for axis in "xyz")
            estimated_errors.append(math.sqrt(sum(
                (estimate[i] - actual_position[i]) ** 2 for i in range(3)
            )))

    steady_start = float(rows[0]["sim_time"]) + 0.75
    steady_rows = [row for row in rows if float(row["sim_time"]) >= steady_start]
    steady_velocity_errors = []
    steady_cross = []
    for row in steady_rows:
        actual = tuple(float(row[f"enu_v{axis}"]) for axis in "xyz")
        command = tuple(float(row[f"command_v{axis}"]) for axis in "xyz")
        steady_velocity_errors.append(math.sqrt(sum((a - c) ** 2 for a, c in zip(actual, command))))
        command_norm = math.sqrt(sum(value * value for value in command))
        if command_norm > 1.0e-6:
            unit = tuple(value / command_norm for value in command)
            along = sum(a * u for a, u in zip(actual, unit))
            steady_cross.append(math.sqrt(max(0.0, sum(a * a for a in actual) - along * along)))

    metrics = {
        "valid": True,
        "position_rmse_m": rms(position_errors),
        "position_max_error_m": max(position_errors),
        "velocity_rmse_mps": rms(velocity_errors),
        "velocity_max_error_mps": max(velocity_errors),
        "steady_velocity_rmse_mps": rms(steady_velocity_errors),
        "cross_axis_rms_mps": rms(cross),
        "steady_cross_axis_rms_mps": rms(steady_cross),
        "max_altitude_deviation_m": max(abs(float(row["z"]) - z0) for row in rows),
        "max_altitude_rise_m": max(float(row["z"]) - z0 for row in rows),
        "max_altitude_drop_m": max(z0 - float(row["z"]) for row in rows),
        "velocity_jerk_rms_mps3": rms(jerk),
        "estimated_position_rmse_m": rms(estimated_errors) if estimated_errors else math.inf,
        "max_tilt_deg": max(
            math.hypot(float(row["roll_deg"]), float(row["pitch_deg"])) for row in rows
        ),
    }
    for axis in "xyz":
        metrics[f"position_{axis}_rmse_m"] = rms(axis_position_errors[axis])
        metrics[f"velocity_{axis}_rmse_mps"] = rms(axis_velocity_errors[axis])
    return metrics
