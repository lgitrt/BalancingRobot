# Author: Luca Obwegs
"""Shared balance-controller design and discrete controller implementation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np
from scipy.linalg import solve_continuous_are

ControllerKind = Literal["pid", "lqr"]


@dataclass(frozen=True)
class MechanicalParameters:
    chassis_mass_kg: float
    chassis_com_height_m: float
    chassis_iyy_kg_m2: float
    wheel_mass_kg: float
    wheel_radius_m: float
    wheel_separation_m: float
    wheel_step_angle_deg: float
    microsteps: int
    gear_ratio: float
    max_wheel_speed_m_s: float
    max_wheel_accel_m_s2: float
    motor_time_constant_s: float
    motor_step_rate_limit_hz: float
    pitch_damping_n_m_s: float
    steps_lost_fraction: float
    fall_angle_rad: float

    @property
    def equivalent_pitch_inertia(self) -> float:
        return self.chassis_iyy_kg_m2 + self.chassis_mass_kg * self.chassis_com_height_m**2

    @property
    def meters_per_step(self) -> float:
        angle = np.deg2rad(self.wheel_step_angle_deg)
        return 2.0 * np.pi * self.wheel_radius_m * angle / (
            self.microsteps * self.gear_ratio
        )

    @property
    def effective_max_speed_m_s(self) -> float:
        return min(
            self.max_wheel_speed_m_s,
            self.motor_step_rate_limit_hz * self.meters_per_step,
        )


@dataclass(frozen=True)
class ControllerParameters:
    lqr_q: tuple[float, float, float, float, float]
    lqr_r: float
    pitch_kp: float
    pitch_ki: float
    pitch_kd: float
    velocity_kp: float
    position_hold_enabled: bool
    position_hold_kp: float
    position_hold_max_speed: float
    position_hold_max_error_m: float = 0.15
    position_hold_max_trim_rad: float = float(np.deg2rad(5.0))


def lqr_gain(
    mechanics: MechanicalParameters,
    parameters: ControllerParameters,
) -> np.ndarray:
    """Return continuous LQR gains for
    [position-error integral, position error, speed, pitch, pitch rate]."""
    m = mechanics.chassis_mass_kg
    length = mechanics.chassis_com_height_m
    inertia = mechanics.equivalent_pitch_inertia
    g = 9.80665
    a = np.zeros((5, 5), dtype=float)
    a[0, 1] = 1.0
    a[1, 2] = 1.0
    a[3, 4] = 1.0
    a[4, 3] = m * g * length / inertia
    b = np.array([[0.0], [0.0], [1.0], [0.0], [-m * length / inertia]], dtype=float)
    if len(parameters.lqr_q) != 5:
        raise ValueError(
            "lqr_q must contain five weights: position integral, position, "
            "speed, pitch, pitch rate"
        )
    q = np.diag(parameters.lqr_q)
    r = np.array([[parameters.lqr_r]], dtype=float)
    riccati = solve_continuous_are(a, b, q, r)
    return np.linalg.solve(r, b.T @ riccati).reshape(5)


class BalanceController:
    """Convert the estimated robot state to a bounded forward-speed command."""

    def __init__(
        self,
        kind: ControllerKind,
        mechanics: MechanicalParameters,
        parameters: ControllerParameters,
    ) -> None:
        if kind not in ("pid", "lqr"):
            raise ValueError(f"Unsupported controller {kind!r}")
        self.kind = kind
        self.mechanics = mechanics
        self.parameters = parameters
        if (not np.isfinite(parameters.position_hold_kp) or
                parameters.position_hold_kp < 0.0 or
                not np.isfinite(parameters.position_hold_max_speed) or
                parameters.position_hold_max_speed <= 0.0 or
                not np.isfinite(parameters.position_hold_max_error_m) or
                parameters.position_hold_max_error_m <= 0.0 or
                not np.isfinite(parameters.position_hold_max_trim_rad) or
                parameters.position_hold_max_trim_rad < 0.0):
            raise ValueError("Position-hold gains and limits must be finite; limits positive")
        self.gain = lqr_gain(mechanics, parameters) if kind == "lqr" else None
        self.integral_pitch = 0.0
        self.position_trim = 0.0
        self.commanded_position = 0.0
        self.commanded_velocity = 0.0
        self.reference_position = 0.0
        self.position_hold_enabled = parameters.position_hold_enabled

    def reset(self, position: float = 0.0, velocity: float = 0.0) -> None:
        self.integral_pitch = 0.0
        self.position_trim = 0.0
        self.commanded_position = position
        self.commanded_velocity = velocity
        self.reference_position = position

    def step(
        self,
        pitch_rad: float,
        pitch_rate_rad_s: float,
        dt: float,
        target_velocity_m_s: float = 0.0,
        target_turn_rate_rad_s: float = 0.0,
        estimated_position_m: float | None = None,
        estimated_velocity_m_s: float | None = None,
    ) -> tuple[float, float, float]:
        if dt <= 0.0 or not np.isfinite(dt):
            raise ValueError("dt must be finite and positive")

        p = self.parameters
        m = self.mechanics
        if estimated_position_m is None and estimated_velocity_m_s is None:
            self.commanded_position += self.commanded_velocity * dt
        elif estimated_position_m is not None and estimated_velocity_m_s is not None:
            self.commanded_position = estimated_position_m
            self.commanded_velocity = estimated_velocity_m_s
        else:
            raise ValueError("position and velocity estimates must be supplied together")
        self.reference_position += target_velocity_m_s * dt
        position_offset = self.commanded_position - self.reference_position
        lqr_position = self.kind == "lqr" and self.position_hold_enabled
        # LQR holds position with direct position and learned-trim (integral)
        # feedback; PID keeps the bounded position-to-velocity correction.
        correction = float(np.clip(
            -p.position_hold_kp * position_offset,
            -p.position_hold_max_speed,
            p.position_hold_max_speed,
        )) if self.position_hold_enabled and not lqr_position else 0.0
        velocity_error = self.commanded_velocity - (target_velocity_m_s + correction)

        if self.kind == "lqr":
            assert self.gain is not None
            position_error = float(np.clip(
                position_offset,
                -p.position_hold_max_error_m,
                p.position_hold_max_error_m,
            )) if lqr_position else 0.0
            control_state = np.array(
                [
                    position_error,
                    velocity_error,
                    pitch_rad - self.position_trim,
                    pitch_rate_rad_s,
                ],
                dtype=float,
            )
            acceleration = -float(self.gain[1:] @ control_state)
            if lqr_position and abs(acceleration) < m.max_wheel_accel_m_s2:
                self.position_trim = float(np.clip(
                    self.position_trim -
                    self.gain[0] / self.gain[3] * position_error * dt,
                    -p.position_hold_max_trim_rad,
                    p.position_hold_max_trim_rad,
                ))
        else:
            tentative_integral = float(
                np.clip(self.integral_pitch + pitch_rad * dt, -0.5, 0.5)
            )
            # Positive velocity feedback: to slow down the robot first has to
            # accelerate under its centre of mass so that it leans back.
            acceleration = (
                p.pitch_kp * pitch_rad
                + p.pitch_ki * tentative_integral
                + p.pitch_kd * pitch_rate_rad_s
                + p.velocity_kp * velocity_error
            )
            if abs(acceleration) < m.max_wheel_accel_m_s2:
                self.integral_pitch = tentative_integral

        acceleration = float(
            np.clip(
                acceleration,
                -m.max_wheel_accel_m_s2,
                m.max_wheel_accel_m_s2,
            )
        )
        self.commanded_velocity = float(
            np.clip(
                self.commanded_velocity + acceleration * dt,
                -m.effective_max_speed_m_s,
                m.effective_max_speed_m_s,
            )
        )
        max_turn_offset = max(
            0.0, m.effective_max_speed_m_s - abs(self.commanded_velocity)
        )
        wheel_offset = float(
            np.clip(
                target_turn_rate_rad_s * m.wheel_separation_m * 0.5,
                -max_turn_offset,
                max_turn_offset,
            )
        )
        return (
            self.commanded_velocity,
            acceleration,
            wheel_offset,
        )
