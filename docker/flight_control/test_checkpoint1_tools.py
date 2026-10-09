#!/usr/bin/env python3
"""Unit tests for the frame alignment and automatic PID search machinery."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_benchmark import initial_pose_align
from adaptive_cmaes import BoundedCMAES
from evaluate_pid_candidate import evaluate
from pid_optimizer import candidate_stream, load_persisted_baseline, passes_regression_guard
from parallel_pid_optimizer import (
    aggregate_candidates,
    acceptable_improvement,
    decode_parameters,
    encode_parameters,
    group_coordinates,
    optimizer_fitness,
    render_parameters,
    replicate_candidates,
)
from parallel_pid_metrics import (
    add_reference_signals,
    continuous_angle_errors_degrees,
    low_pass_vectors,
    motion_metrics,
    trajectory_error_metrics,
)
from pid_tuning_status import render as render_tuning_status
from benchmark_parallel_capacity import capacity_passed


class InitialPoseAlignmentTest(unittest.TestCase):
    def test_first_pose_is_exact_without_fitting_later_samples(self) -> None:
        reference = np.asarray(((5.0, -2.0, 1.5), (6.0, -2.0, 1.5)))
        estimate = np.asarray(((0.2, 0.3, 0.1), (0.2, 2.3, 0.1)))
        aligned, yaw_delta = initial_pose_align(
            reference,
            estimate,
            np.asarray((90.0, 90.0)),
            np.asarray((0.0, 0.0)),
        )
        np.testing.assert_allclose(aligned[0], reference[0], atol=1.0e-12)
        # The estimator moved two metres while truth moved one. Initial-only
        # alignment must retain that scale error instead of fitting it away.
        self.assertAlmostEqual(np.linalg.norm(aligned[1] - aligned[0]), 2.0)
        self.assertEqual(yaw_delta, 90.0)


class PidEvaluatorTest(unittest.TestCase):
    def test_machine_verdict_and_repeatability_are_reported(self) -> None:
        trial = {
            "hover": {
                "xy_rms_m": 0.03,
                "xy_max_m": 0.10,
                "z_rms_m": 0.005,
                "z_peak_to_peak_m": 0.015,
                "yaw_peak_to_peak_deg": 1.0,
                "ground_xy_speed_rms_mps": 0.02,
                "roll_rms_deg": 0.1,
                "pitch_rms_deg": 0.1,
                "estimator_xy_rms_m": 0.02,
            },
            "aggregate": {
                "worst_stop_displacement_m": 0.08,
                "worst_z_rise_after_stop_m": 0.01,
                "worst_z_drop_after_stop_m": -0.01,
            },
            "stops": {"forward": {"settling_time_s": 2.0}},
        }
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for index in range(2):
                path = Path(directory) / f"trial-{index}.json"
                path.write_text(json.dumps(trial))
                paths.append(path)
            result = evaluate(paths)
        self.assertTrue(result["pass"])
        self.assertTrue(result["safe"])
        self.assertEqual(result["repeat_count"], 2)
        self.assertTrue(np.isfinite(result["score"]))
        self.assertTrue(all(value == 0.0 for value in result["metrics_stddev"].values()))
        self.assertLessEqual(result["metrics_worst"]["worst_stop_displacement_m"], 0.10)


class CandidateSearchTest(unittest.TestCase):
    def test_critical_metric_regression_blocks_false_improvement(self) -> None:
        config = {
            "maximum_critical_regression": 0.2,
            "critical_metrics": ["braking"],
        }
        result = {"metrics_mean": {"braking": 0.75}}
        self.assertFalse(passes_regression_guard(result, {"braking": 0.5}, config))
        result["metrics_mean"]["braking"] = 0.59
        self.assertTrue(passes_regression_guard(result, {"braking": 0.5}, config))

    def test_candidates_are_bounded_and_rounded_to_four_decimals(self) -> None:
        config = {
            "precision": 4,
            "groups": [{
                "name": "xy",
                "parameters": {"P": {"minimum": 0.9, "maximum": 1.1, "step": 0.025}},
            }],
        }
        candidates = list(candidate_stream(config, {"P": 1.0}, {"xy"}, set()))
        values = [candidate[3]["P"] for candidate in candidates]
        self.assertIn(0.975, values)
        self.assertIn(1.025, values)
        self.assertTrue(all(0.9 <= value <= 1.1 for value in values))
        self.assertTrue(all(value == round(value, 4) for value in values))

    def test_failed_baseline_is_still_excluded_from_candidate_stream(self) -> None:
        config = {
            "precision": 4,
            "groups": [{
                "name": "xy",
                "parameters": {"P": {"minimum": 0.9, "maximum": 1.1, "step": 0.05}},
            }],
        }
        baseline = {"P": 1.0}
        history = {json.dumps(baseline, sort_keys=True)}
        first = next(candidate_stream(config, baseline, {"xy"}, history))
        self.assertNotEqual(first[3], baseline)

    def test_baseline_comes_from_the_persisted_sitl_parameter_file(self) -> None:
        config = {
            "precision": 4,
            "groups": [{
                "name": "xy",
                "parameters": {
                    "P": {"minimum": 0.9, "maximum": 1.1, "step": 0.05},
                    "I": {"minimum": 0.1, "maximum": 0.5, "step": 0.05},
                },
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            parameter_file = Path(directory) / "controller.parm"
            parameter_file.write_text("P 1.012345\nI 0.25 # retained\nOTHER 9\n")
            baseline = load_persisted_baseline(config, parameter_file)
        self.assertEqual(baseline, {"P": 1.0123, "I": 0.25})

    def test_linked_knob_changes_roll_and_pitch_together(self) -> None:
        config = {
            "precision": 4,
            "groups": [{"name": "rate", "parameters": {
                "RP_P": {"members": ["ROLL_P", "PITCH_P"], "minimum": 0.1,
                         "maximum": 0.2, "step": 0.01},
            }}],
        }
        candidate = next(candidate_stream(
            config, {"ROLL_P": 0.135, "PITCH_P": 0.135}, {"rate"}, set()
        ))[3]
        self.assertEqual(candidate["ROLL_P"], candidate["PITCH_P"])

    def test_adaptive_coordinates_keep_linked_roll_pitch_equal(self) -> None:
        group = {
            "parameters": {
                "RP_P": {
                    "members": ["ROLL_P", "PITCH_P"],
                    "minimum": 0.1,
                    "maximum": 0.2,
                    "step": 0.01,
                }
            }
        }
        coordinates = group_coordinates(group)
        baseline = {"ROLL_P": 0.135, "PITCH_P": 0.135}
        encoded = encode_parameters(baseline, coordinates)
        candidate = decode_parameters(
            [min(1.0, encoded[0] + 0.123456)], baseline, coordinates, 6
        )
        self.assertEqual(candidate["ROLL_P"], candidate["PITCH_P"])

    def test_parallel_promotion_requires_meaningful_passing_improvement(self) -> None:
        control = {"score": 1.0, "pass": True}
        self.assertTrue(acceptable_improvement({"score": 0.98, "pass": True}, control, 0.01))
        self.assertFalse(acceptable_improvement({"score": 0.995, "pass": True}, control, 0.01))
        self.assertFalse(acceptable_improvement({"score": 0.5, "pass": False}, control, 0.01))

    def test_parallel_failed_baseline_can_take_safe_incremental_improvement(self) -> None:
        control = {
            "score": 10.0, "pass": False,
            "hover_worst": {"xy_max_m": 0.05, "z_max_abs_m": 0.01},
            "dynamic_worst": {
                "max_altitude_rise_m": 0.08,
                "standard_stop_displacement_m": 0.60,
                "reversal_stop_displacement_m": 1.25,
            },
        }
        winner = json.loads(json.dumps(control))
        winner["score"] = 9.5
        winner["dynamic_worst"]["reversal_stop_displacement_m"] = 1.15
        self.assertTrue(acceptable_improvement(winner, control, 0.01))
        winner["hover_worst"]["xy_max_m"] = 0.09
        self.assertFalse(acceptable_improvement(winner, control, 0.01))

    def test_parallel_parameter_render_is_six_decimal_and_exact(self) -> None:
        rendered = render_parameters("P 1.0\nI 0.5\n", {"P": 1.23456, "I": 0.25})
        self.assertEqual(rendered, "P               1.234560\nI               0.250000\n")

    def test_parallel_candidates_are_replicated_before_launch(self) -> None:
        expanded = replicate_candidates(
            [{"name": "control", "parameters": {"P": 1.0}}], 3
        )
        self.assertEqual([item["replica"] for item in expanded], [0, 1, 2])
        self.assertEqual(len({item["candidate_id"] for item in expanded}), 1)

    def test_cma_es_adapts_and_converges_without_parameter_steps(self) -> None:
        optimizer = BoundedCMAES([0.85, 0.15], 8, sigma=0.25, seed=42)
        initial_distance = math.dist(optimizer.mean, [0.25, 0.75])
        for _ in range(45):
            points = optimizer.ask()
            fitness = [sum((value - target) ** 2 for value, target in zip(point, (0.25, 0.75)))
                       for point in points]
            optimizer.tell(points, fitness)
        self.assertLess(math.dist(optimizer.mean, [0.25, 0.75]), initial_distance * 0.10)
        self.assertTrue(all(0.0 <= value <= 1.0 for value in optimizer.mean))

    def test_cma_state_restores_adaptive_distribution(self) -> None:
        optimizer = BoundedCMAES([0.4, 0.6], 6, sigma=0.18, seed=7)
        points = optimizer.ask()
        optimizer.tell(points, [sum(value * value for value in point) for point in points])
        restored = BoundedCMAES([0.0, 0.0], 6, seed=99, state=optimizer.state())
        np.testing.assert_allclose(restored.mean, optimizer.mean)
        np.testing.assert_allclose(restored.covariance, optimizer.covariance)
        self.assertAlmostEqual(restored.sigma, optimizer.sigma)
        self.assertEqual(restored.generation, optimizer.generation)

    def test_cma_fitness_penalizes_each_failed_safety_gate(self) -> None:
        self.assertEqual(optimizer_fitness({"score": 2.5, "failed_gates": []}), 2.5)
        self.assertEqual(
            optimizer_fitness({"score": 2.5, "failed_gates": ["yaw", "high_speed_motion"]}),
            12.5,
        )

    def test_parallel_ranking_penalizes_variance_and_requires_all_replicas(self) -> None:
        def result(candidate_id, env, score, passed=True):
            return {
                "environment": env,
                "candidate": {
                    "candidate_id": candidate_id,
                    "name": candidate_id,
                    "parameters": {"P": 1.0},
                },
                "score": score,
                "pass": passed,
                "hover": {"xy_max_m": score, "z_max_abs_m": 0.01},
            }

        ranking = aggregate_candidates([
            result("stable", 0, 1.0), result("stable", 1, 1.0),
            result("noisy", 2, 0.7), result("noisy", 3, 1.1),
        ])
        self.assertEqual(ranking[0]["candidate_id"], "stable")
        self.assertEqual(ranking[0]["replica_count"], 2)
        failed = aggregate_candidates([
            result("candidate", 0, 0.5), result("candidate", 1, 0.5, False)
        ])
        self.assertFalse(failed[0]["pass"])

    def test_dynamic_reference_uses_enu_and_removes_initial_estimator_offset(self) -> None:
        rows = []
        for index in range(3):
            rows.append({
                "phase": "move_0", "sim_time": float(index),
                "x": 0.0, "y": -float(index), "z": 1.5,
                "vx": 0.0, "vy": -1.0, "vz": 0.0,
                "command_vx": 1.0, "command_vy": 0.0, "command_vz": 0.0,
                "estimated_x": 10.0 + index, "estimated_y": -4.0,
                "estimated_z": 2.0,
                "roll_deg": 0.0, "pitch_deg": 0.0,
            })
        enriched = add_reference_signals(rows)
        self.assertEqual(enriched[-1]["enu_x"], 2.0)
        self.assertEqual(enriched[-1]["ideal_x"], 2.0)
        self.assertEqual(enriched[-1]["aligned_estimated_x"], 2.0)

    def test_ideal_position_is_continuous_across_command_boundaries(self) -> None:
        rows = []
        samples = (
            ("move", 0.0, 0.0, 1.0),
            ("move", 1.0, 1.0, 1.0),
            ("stop", 2.0, 2.0, 0.0),
            ("stop", 3.0, 1.5, 0.0),
        )
        for phase, sim_time, enu_x, command_x in samples:
            rows.append({
                "phase": phase, "sim_time": sim_time,
                "x": 0.0, "y": -enu_x, "z": 1.5,
                "vx": 0.0, "vy": -command_x, "vz": 0.0,
                "command_vx": command_x, "command_vy": 0.0, "command_vz": 0.0,
                "estimated_x": enu_x + 10.0, "estimated_y": 0.0,
                "estimated_z": 1.5,
            })
        enriched = add_reference_signals(rows)
        self.assertAlmostEqual(enriched[-1]["ideal_x"], 2.0)
        self.assertAlmostEqual(enriched[-1]["position_error_x"], -0.5)
        metrics = trajectory_error_metrics(enriched)
        self.assertAlmostEqual(metrics["position_final_error_m"], 0.5)
        self.assertAlmostEqual(metrics["x_final_error_m"], -0.5)

    def test_motion_metrics_report_axis_tracking_and_altitude_coupling(self) -> None:
        rows = []
        for index in range(12):
            rows.append({
                "phase": "stress_0", "sim_time": index * 0.1,
                "x": 0.0, "y": -index * 0.1, "z": 1.5 + index * 0.001,
                "vx": 0.0, "vy": -1.0, "vz": 0.01,
                "command_vx": 1.0, "command_vy": 0.0, "command_vz": 0.0,
                "estimated_x": index * 0.1, "estimated_y": 0.0,
                "estimated_z": 1.5 + index * 0.001,
                "roll_deg": 0.0, "pitch_deg": 1.0,
            })
        metrics = motion_metrics(add_reference_signals(rows))
        self.assertAlmostEqual(metrics["velocity_x_rmse_mps"], 0.0)
        self.assertAlmostEqual(metrics["position_x_rmse_m"], 0.0)
        self.assertGreater(metrics["max_altitude_rise_m"], 0.01)
        self.assertLess(metrics["estimated_position_rmse_m"], 1.0e-9)

    def test_low_pass_rejects_single_sample_velocity_noise(self) -> None:
        times = [index * 0.01 for index in range(21)]
        velocities = [(0.0, 0.0, 0.0) for _ in times]
        velocities[10] = (1.0, 0.0, 0.0)
        filtered = low_pass_vectors(times, velocities)
        self.assertLess(filtered[10][0], 0.12)
        self.assertGreater(filtered[11][0], 0.0)

    def test_yaw_error_unwrap_does_not_create_false_360_degree_overshoot(self) -> None:
        continuous, wrapped = continuous_angle_errors_degrees(
            [170.0, 179.0, -175.0, -160.0, -120.0, -90.0], -90.0
        )
        self.assertLess(max(abs(value) for value in continuous), 101.0)
        self.assertAlmostEqual(wrapped[-1], 0.0, places=9)

    def test_live_status_includes_each_drone_and_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "status_test"
            batch = run / "generation_001_hover"
            batch.mkdir(parents=True)
            (batch / "batch_status.json").write_text(json.dumps({
                "status": "running",
                "protocol": "hover",
                "drone_count": 2,
                "candidates": [
                    {"drone": 0, "name": "incumbent", "replica": 0},
                    {"drone": 1, "name": "candidate", "replica": 0},
                ],
            }))
            (batch / "env_0_progress.json").write_text(json.dumps({
                "status": "running", "phase": "hover", "phase_progress": 0.5,
                "position_m": [1.0, 2.0, 1.5], "velocity_mps": [0.0, 0.0, 0.0],
                "sample_count": 100,
            }))
            (batch / "env_1_progress.json").write_text(json.dumps({
                "status": "preflight", "phase": "preflight", "sample_count": 0,
            }))
            (batch / "env_0.json").write_text(json.dumps({
                "score": 0.25,
                "pass": False,
                "hover": {
                    "xy_max_m": 0.0123,
                    "z_max_abs_m": 0.0045,
                    "yaw_peak_to_peak_deg": 1.25,
                },
                "dynamic": {
                    "standard_stop_displacement_m": 0.21,
                    "reversal_stop_displacement_m": 0.42,
                },
                "timing": {"real_time_factor": 0.999},
                "gates": {"hover": {"pass": True}, "yaw": {"pass": False}},
            }))
            text = render_tuning_status(run, None)
        self.assertIn("incumbent", text)
        self.assertIn("candidate", text)
        self.assertIn("50%", text)
        self.assertIn("1.00,2.00,1.50", text)
        self.assertIn("hover xy=0.0123 m", text)
        self.assertIn("failed gates=yaw", text)

    def test_capacity_gate_is_independent_of_controller_gate(self) -> None:
        result = {
            "replica_count": 10,
            "pass": False,
            "timing": {"real_time_factor_worst": 0.999},
        }
        self.assertTrue(capacity_passed(result, 10))
        result["timing"]["real_time_factor_worst"] = 0.90
        self.assertFalse(capacity_passed(result, 10))



if __name__ == "__main__":
    unittest.main()
