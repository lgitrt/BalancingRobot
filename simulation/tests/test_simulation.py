# Author: Luca Obwegs
"""Focused tests for model generation and balancing controllers."""

import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from balancing_robot_sim.analytical import controller_for_config, simulate
from balancing_robot_sim.generate_sdf import generate
from balancing_robot_sim.trajectory import trajectory_from_config


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "config" / "mechanical.json").read_text(encoding="utf-8"))


class SimulationTests(unittest.TestCase):
    def test_lqr_gain_is_finite_and_has_four_states(self):
        controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
        self.assertEqual(controller.gain.shape, (4,))
        self.assertTrue(np.all(np.isfinite(controller.gain)))

    def test_invalid_time_step_is_rejected(self):
        controller = controller_for_config("pid", CONFIG, CONFIG["control"])
        with self.assertRaises(ValueError):
            controller.step(0.0, 0.0, 0.0)

    def test_commanded_motion_can_be_replaced_by_step_estimates(self):
        controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
        estimated_command = controller.step(
            0.0,
            0.0,
            0.002,
            estimated_position_m=0.12,
            estimated_velocity_m_s=0.03,
        )
        baseline = controller_for_config("lqr", CONFIG, CONFIG["control"])
        baseline_command = baseline.step(0.0, 0.0, 0.002)
        self.assertEqual(controller.commanded_position, 0.12)
        self.assertNotEqual(estimated_command[0], baseline_command[0])

    def test_model_generator_produces_well_formed_configured_sdf(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = generate(
                ROOT / "config" / "mechanical.json",
                ROOT / "models" / "balancing_robot" / "model.sdf.in",
                Path(temporary) / "model.sdf",
            )
            model = ET.parse(output).getroot().find("./model")
            self.assertIsNotNone(model)
            self.assertEqual(model.findtext("link[@name='chassis']/inertial/mass"), "0.8")
            self.assertEqual(
                model.findtext("link[@name='chassis']/pose"), "0 0 0.12 0 0 0"
            )
            self.assertEqual(len(model.findall("joint")), 2)
            self.assertEqual(
                model.findtext("link[@name='left_wheel']/collision/pose"),
                "0 0 0 1.57079632679 0 0",
            )
            self.assertIn("0 0 0.04 0 0.0 0", model.findtext("pose"))

    def test_simulator_stops_when_requested_duration_is_reached(self):
        controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
        run = simulate(controller, 0.5, 0.002, 0.0)
        self.assertFalse(run.fallen)
        self.assertAlmostEqual(run.time_s[-1], 0.5, places=3)

    def test_trajectory_alternates_targets_after_arrival_and_dwell(self):
        trajectory = trajectory_from_config(CONFIG["trajectory"])
        first_target = trajectory.target_position_m
        self.assertGreater(trajectory.step(0.0, 0.0, 0.01), 0.0)
        self.assertEqual(
            trajectory.step(first_target, 0.0, 0.01),
            0.0,
        )
        self.assertEqual(trajectory.target_position_m, first_target)
        for _ in range(50):
            self.assertEqual(trajectory.step(first_target, 0.0, 0.01), 0.0)
        self.assertEqual(trajectory.target_position_m, -first_target)

    def test_trajectory_alternates_when_a_waypoint_is_crossed(self):
        trajectory = trajectory_from_config(CONFIG["trajectory"])
        first_target = trajectory.target_position_m
        trajectory.step(0.0, 0.0, 0.01)
        self.assertEqual(trajectory.step(first_target + 0.01, 0.15, 0.01), 0.0)
        self.assertEqual(trajectory.target_position_m, first_target)
        for _ in range(50):
            trajectory.step(first_target + 0.01, 0.0, 0.01)
        self.assertEqual(trajectory.target_position_m, -first_target)

    def test_default_animation_run_visits_both_trajectory_positions(self):
        controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
        run = simulate(
            controller,
            8.0,
            1.0 / float(CONFIG["control"]["loop_hz"]),
            np.deg2rad(5.0),
            trajectory=trajectory_from_config(CONFIG["trajectory"]),
        )
        self.assertFalse(run.fallen)
        self.assertAlmostEqual(run.time_s[-1], 8.0, places=3)
        self.assertGreater(np.count_nonzero(np.diff(run.target_position_m)), 0)
        for target in CONFIG["trajectory"]["positions_m"]:
            self.assertTrue(
                np.any(np.abs(run.position_m - target) < 0.01),
                f"trajectory did not reach target {target} m",
            )

    def test_lqr_balances_example_plant_better_than_pid(self):
        runs = {}
        for kind in ("pid", "lqr"):
            controller = controller_for_config(kind, CONFIG, CONFIG["control"])
            runs[kind] = simulate(controller, 8.0, 0.002, np.deg2rad(5.0))
        pid, lqr = runs["pid"], runs["lqr"]
        self.assertFalse(pid.fallen)
        self.assertFalse(lqr.fallen)
        self.assertLess(np.sqrt(np.mean(lqr.pitch_rad**2)), np.sqrt(np.mean(pid.pitch_rad**2)))
        self.assertLess(np.sqrt(np.mean(lqr.position_m**2)), 0.1)


if __name__ == "__main__":
    unittest.main()
