# Author: Luca Obwegs
"""Deterministic nonlinear model used for controller comparisons and plots."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from balancing_robot_sim.control import BalanceController, ControllerKind


@dataclass
class Run:
    time_s: NDArray[np.float64]
    position_m: NDArray[np.float64]
    estimated_position_m: NDArray[np.float64]
    velocity_m_s: NDArray[np.float64]
    pitch_rad: NDArray[np.float64]
    pitch_rate_rad_s: NDArray[np.float64]
    acceleration_m_s2: NDArray[np.float64]
    step_rate_hz: NDArray[np.float64]
    fallen: bool


def simulate(
    controller: BalanceController,
    duration_s: float,
    dt: float,
    initial_pitch_rad: float,
    initial_forward_speed_m_s: float = 0.0,
) -> Run:
    if duration_s <= 0.0 or dt <= 0.0:
        raise ValueError("duration and dt must be positive")

    mechanics = controller.mechanics
    count = int(np.ceil(duration_s / dt)) + 1
    time = np.arange(count, dtype=float) * dt
    position = np.zeros(count, dtype=float)
    velocity = np.zeros(count, dtype=float)
    pitch = np.zeros(count, dtype=float)
    pitch_rate = np.zeros(count, dtype=float)
    acceleration = np.zeros(count, dtype=float)
    step_rate = np.zeros(count, dtype=float)
    pitch[0] = initial_pitch_rad
    velocity[0] = initial_forward_speed_m_s
    estimated_position = 0.0
    position_estimate = np.zeros(count, dtype=float)
    estimated_velocity = initial_forward_speed_m_s
    step_residual = 0.0
    equivalent_inertia = mechanics.equivalent_pitch_inertia
    mass = mechanics.chassis_mass_kg
    length = mechanics.chassis_com_height_m
    fall_index = count - 1

    for i in range(count - 1):
        requested_velocity, _, _ = controller.step(
            pitch[i], pitch_rate[i], dt
        )
        step_m = mechanics.meters_per_step
        step_hz = requested_velocity / step_m
        step_hz = float(
            np.clip(
                step_hz,
                -min(
                    mechanics.motor_step_rate_limit_hz,
                    mechanics.max_wheel_speed_m_s / step_m,
                ),
                min(
                    mechanics.motor_step_rate_limit_hz,
                    mechanics.max_wheel_speed_m_s / step_m,
                ),
            )
        )
        step_rate[i] = step_hz

        actuator_velocity = step_hz * step_m
        step_residual += actuator_velocity * dt / step_m
        emitted_steps = int(np.trunc(step_residual))
        step_residual -= emitted_steps
        estimated_velocity = emitted_steps * step_m / dt
        estimated_position += emitted_steps * step_m
        position_estimate[i] = estimated_position

        tau = max(mechanics.motor_time_constant_s, dt)
        alpha = 1.0 - np.exp(-dt / tau)
        actual_accel = float(
            np.clip(
                (actuator_velocity - velocity[i]) * alpha / dt,
                -mechanics.max_wheel_accel_m_s2,
                mechanics.max_wheel_accel_m_s2,
            )
        )
        actual_accel *= 1.0 - mechanics.steps_lost_fraction
        accel = actual_accel
        damping = mechanics.pitch_damping_n_m_s
        theta_ddot = (
            mass * 9.80665 * length * np.sin(pitch[i])
            - mass * length * accel * np.cos(pitch[i])
            - damping * pitch_rate[i]
        ) / equivalent_inertia

        position[i + 1] = position[i] + velocity[i] * dt + 0.5 * accel * dt**2
        position_estimate[i + 1] = estimated_position
        velocity[i + 1] = velocity[i] + accel * dt
        pitch_rate[i + 1] = pitch_rate[i] + theta_ddot * dt
        pitch[i + 1] = pitch[i] + pitch_rate[i] * dt + 0.5 * theta_ddot * dt**2
        acceleration[i] = accel

        controller.commanded_position = estimated_position
        controller.commanded_velocity = estimated_velocity

        if abs(pitch[i + 1]) >= mechanics.fall_angle_rad:
            fall_index = i + 1
            break

    if fall_index < count - 1:
        time = time[: fall_index + 1]
        position = position[: fall_index + 1]
        position_estimate = position_estimate[: fall_index + 1]
        velocity = velocity[: fall_index + 1]
        pitch = pitch[: fall_index + 1]
        pitch_rate = pitch_rate[: fall_index + 1]
        acceleration = acceleration[: fall_index + 1]
        step_rate = step_rate[: fall_index + 1]
    else:
        step_rate[-1] = step_rate[-2] if count > 1 else 0.0
        acceleration[-1] = acceleration[-2] if count > 1 else 0.0

    return Run(
        time,
        position,
        position_estimate,
        velocity,
        pitch,
        pitch_rate,
        acceleration,
        step_rate,
        fall_index < count - 1,
    )


def controller_for_config(
    kind: ControllerKind,
    mechanical_config: dict,
    control_config: dict,
) -> BalanceController:
    from balancing_robot_sim.control import (
        ControllerParameters,
        MechanicalParameters,
    )

    robot = mechanical_config["robot"]
    inertia = robot["chassis_inertia_kg_m2"]
    mechanics = MechanicalParameters(
        chassis_mass_kg=float(robot["chassis_mass_kg"]),
        chassis_com_height_m=float(robot["chassis_com_height_m"]),
        chassis_iyy_kg_m2=float(inertia["iyy"]),
        wheel_mass_kg=float(robot["wheel_mass_kg"]),
        wheel_radius_m=float(robot["wheel_radius_m"]),
        wheel_separation_m=float(robot["wheel_separation_m"]),
        wheel_step_angle_deg=float(robot["wheel_step_angle_deg"]),
        microsteps=int(robot["microsteps"]),
        gear_ratio=float(robot["gear_ratio"]),
        max_wheel_speed_m_s=float(robot["max_wheel_speed_m_s"]),
        max_wheel_accel_m_s2=float(robot["max_wheel_accel_m_s2"]),
        motor_time_constant_s=float(robot["motor_time_constant_s"]),
        motor_step_rate_limit_hz=float(robot["motor_step_rate_limit_hz"]),
        pitch_damping_n_m_s=float(robot["pitch_damping_n_m_s"]),
        steps_lost_fraction=float(robot["steps_lost_fraction"]),
        fall_angle_rad=float(robot["fall_angle_rad"]),
    )
    pid = control_config["pid"]
    parameters = ControllerParameters(
        lqr_q=tuple(float(x) for x in control_config["lqr_q"]),
        lqr_r=float(control_config["lqr_r"]),
        pitch_kp=float(pid["pitch_kp"]),
        pitch_ki=float(pid["pitch_ki"]),
        pitch_kd=float(pid["pitch_kd"]),
        velocity_kp=float(pid["velocity_kp"]),
        position_kp=float(pid["position_kp"]),
    )
    return BalanceController(kind, mechanics, parameters)
