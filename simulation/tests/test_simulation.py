# Author: Luca Obwegs
"""Focused tests for model generation and balancing controllers."""

import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from balancing_robot_sim.export_firmware_config import export

from balancing_robot_sim.analytical import controller_for_config, simulate
from balancing_robot_sim.generate_sdf import generate
from balancing_robot_sim.trajectory import trajectory_from_config


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "config" / "mechanical.json").read_text(encoding="utf-8"))


class SimulationTests(unittest.TestCase):
    def test_lqr_gain_is_finite_and_has_five_states(self):
        controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
        self.assertEqual(controller.gain.shape, (5,))
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
            self.assertAlmostEqual(
                float(model.findtext("link[@name='chassis']/inertial/mass")),
                CONFIG["robot"]["chassis_mass_kg"],
            )
            self.assertAlmostEqual(
                float(model.findtext("link[@name='chassis']/pose").split()[2]),
                CONFIG["robot"]["chassis_com_height_m"],
            )
            self.assertEqual(len(model.findall("joint")), 2)
            self.assertEqual(
                model.findtext("link[@name='left_wheel']/collision/pose"),
                "0 0 0 1.57079632679 0 0",
            )
            self.assertAlmostEqual(
                float(model.findtext("pose").split()[2]),
                CONFIG["robot"]["wheel_radius_m"],
            )

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

    def test_pid_and_lqr_recover_and_settle(self):
        # 30 s with position hold: catches slowly diverging outer loops that an
        # 8 s run without hold does not show.
        for kind in ("pid", "lqr"):
            controller = controller_for_config(kind, CONFIG, CONFIG["control"])
            run = simulate(controller, 30.0, 0.002, np.deg2rad(5.0))
            self.assertFalse(run.fallen, kind)
            self.assertLess(np.sqrt(np.mean(run.pitch_rad**2)), np.deg2rad(1.5), kind)
            self.assertLess(abs(run.position_m[-1]), 0.01, kind)
            self.assertLess(abs(run.velocity_m_s[-1]), 0.01, kind)

    def test_balance_command_is_independent_of_position(self):
        for kind in ("pid", "lqr"):
            commands = []
            for position in (-100.0, 0.0, 100.0):
                controller = controller_for_config(kind, CONFIG, CONFIG["control"])
                controller.reset(position=position)
                commands.append(controller.step(
                    0.01, 0.02, 0.002,
                    estimated_position_m=position,
                    estimated_velocity_m_s=0.03,
                ))
            self.assertEqual(commands[0], commands[1])
            self.assertEqual(commands[1], commands[2])

    def test_position_hold_correction_is_bounded_and_origin_resets(self):
        for kind in ("pid", "lqr"):
            commands = []
            for position in (0.3, 100.0):
                controller = controller_for_config(kind, CONFIG, CONFIG["control"])
                self.assertTrue(controller.position_hold_enabled)
                controller.reset(position=5.0)
                self.assertEqual(controller.reference_position, 5.0)
                commands.append(controller.step(
                    0.0, 0.0, 0.002, estimated_position_m=5.0 + position,
                    estimated_velocity_m_s=0.0,
                ))
            np.testing.assert_allclose(commands[0], commands[1], atol=1e-12)

    def test_position_hold_returns_example_plant_toward_origin(self):
        positions = []
        for hold in (False, True):
            controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
            controller.position_hold_enabled = hold
            run = simulate(controller, 20.0, 0.002, np.deg2rad(5.0))
            self.assertFalse(run.fallen)
            positions.append(abs(run.position_m[-1]))
        self.assertLess(positions[1], 0.01)
        self.assertLess(positions[1], positions[0] * 0.1)

    def test_lqr_position_hold_learns_boot_pitch_offset(self):
        # Linear plant with 1-tick command delay: the estimated pitch reads
        # low by the boot offset, so the true balance point is at estimate + offset.
        # 3 degrees drives the robot beyond the position clamp; learning must go on.
        def run(offset_deg, hold):
            offset = np.deg2rad(offset_deg)
            controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
            controller.position_hold_enabled = hold
            m = controller.mechanics
            alpha = m.chassis_mass_kg * 9.80665 * m.chassis_com_height_m / m.equivalent_pitch_inertia
            beta = m.chassis_mass_kg * m.chassis_com_height_m / m.equivalent_pitch_inertia
            dt = 0.002
            position = velocity = pitch = rate = acceleration = 0.0
            positions = []
            for _ in range(int(20.0 / dt)):
                _, next_acceleration, _ = controller.step(
                    pitch - offset, rate, dt,
                    estimated_position_m=position, estimated_velocity_m_s=velocity,
                )
                position += velocity * dt
                velocity += acceleration * dt
                pitch_acceleration = alpha * pitch - beta * acceleration
                pitch += rate * dt
                rate += pitch_acceleration * dt
                acceleration = next_acceleration
                positions.append(position)
            return np.array(positions), controller.position_trim

        free_positions, _ = run(1.0, False)
        self.assertGreater(abs(free_positions[-1]), 1.0)
        for offset_deg, max_travel in ((1.0, 0.10), (3.0, 0.40)):
            hold_positions, learned_trim = run(offset_deg, True)
            self.assertLess(np.max(np.abs(hold_positions)), max_travel)
            self.assertLess(abs(hold_positions[-1]), 0.01)
            self.assertAlmostEqual(learned_trim, -np.deg2rad(offset_deg), delta=np.deg2rad(0.02))

    def test_lqr_position_error_is_clamped(self):
        clamp = CONFIG["control"]["position_hold"]["max_position_error_m"]
        commands, trims = [], []
        for position in (clamp, 100.0):
            controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
            commands.append(controller.step(
                0.0, 0.0, 0.002, estimated_position_m=position,
                estimated_velocity_m_s=0.0,
            ))
            trims.append(controller.position_trim)
        np.testing.assert_allclose(commands[0], commands[1], atol=1e-12)
        # Non-minimum phase: a robot ahead of its reference first drives
        # forward so it can lean back toward the reference.
        self.assertAlmostEqual(commands[0][1], -controller.gain[1] * clamp)
        # Trim learning continues beyond the clamp at the bounded rate.
        expected_trim = -controller.gain[0] / controller.gain[3] * clamp * 0.002
        np.testing.assert_allclose(trims, [expected_trim, expected_trim], rtol=1e-12)

    def test_firmware_export_has_lqr_position_feedback(self):
        controller = controller_for_config("lqr", CONFIG, CONFIG["control"])
        with tempfile.TemporaryDirectory() as temporary:
            output = export(ROOT / "config" / "mechanical.json",
                            Path(temporary) / "robot_config_generated.h")
            content = output.read_text(encoding="utf-8")
            self.assertIn(f"#define ROBOT_LQR_K_INTEGRAL {controller.gain[0]:.9g}f", content)
            self.assertIn(f"#define ROBOT_LQR_K_POSITION {controller.gain[1]:.9g}f", content)
            clamp = CONFIG["control"]["position_hold"]["max_position_error_m"]
            self.assertIn(f"#define ROBOT_POSITION_HOLD_MAX_ERROR_M {clamp:.9g}f", content)
            self.assertIn("#define ROBOT_PID_POSITION_KP 0.0f", content)


if __name__ == "__main__":
    unittest.main()
