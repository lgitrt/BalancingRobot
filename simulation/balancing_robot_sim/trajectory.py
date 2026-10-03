# Author: Luca Obwegs
"""Smooth alternating position setpoints for the balancing robot."""

from __future__ import annotations

import math


class AlternatingPositionTrajectory:
    def __init__(
        self,
        positions_m: list[float] | tuple[float, float],
        max_speed_m_s: float,
        max_acceleration_m_s2: float,
        dwell_s: float,
        position_tolerance_m: float,
        velocity_tolerance_m_s: float,
    ) -> None:
        if len(positions_m) != 2 or not all(math.isfinite(x) for x in positions_m):
            raise ValueError("trajectory requires exactly two finite positions")
        if positions_m[0] == positions_m[1]:
            raise ValueError("trajectory positions must be different")
        if (
            not math.isfinite(max_speed_m_s)
            or not math.isfinite(max_acceleration_m_s2)
            or max_speed_m_s <= 0.0
            or max_acceleration_m_s2 <= 0.0
        ):
            raise ValueError("trajectory speed and acceleration must be finite and positive")
        if (
            not math.isfinite(dwell_s)
            or not math.isfinite(position_tolerance_m)
            or not math.isfinite(velocity_tolerance_m_s)
            or dwell_s < 0.0
            or position_tolerance_m <= 0.0
            or velocity_tolerance_m_s <= 0.0
        ):
            raise ValueError("trajectory dwell must be non-negative and tolerances positive")

        self.positions_m = (float(positions_m[0]), float(positions_m[1]))
        self.max_speed_m_s = max_speed_m_s
        self.max_acceleration_m_s2 = max_acceleration_m_s2
        self.dwell_s = dwell_s
        self.position_tolerance_m = position_tolerance_m
        self.velocity_tolerance_m_s = velocity_tolerance_m_s
        self.target_index = 1
        self._dwell_remaining_s = 0.0
        self._previous_position_m: float | None = None

    @property
    def target_position_m(self) -> float:
        return self.positions_m[self.target_index]

    def step(self, position_m: float, velocity_m_s: float, dt: float) -> float:
        if not all(math.isfinite(value) for value in (position_m, velocity_m_s, dt)):
            raise ValueError("trajectory state and timestep must be finite")
        if dt <= 0.0:
            raise ValueError("trajectory timestep must be positive")

        if self._dwell_remaining_s > 0.0:
            self._dwell_remaining_s -= dt
            if self._dwell_remaining_s <= 0.0:
                self.target_index = 1 - self.target_index
            self._previous_position_m = position_m
            return 0.0

        position_error = self.target_position_m - position_m
        crossed_target = (
            self._previous_position_m is not None
            and (self.target_position_m - self._previous_position_m) * position_error
            <= 0.0
        )
        self._previous_position_m = position_m
        settled_at_target = (
            abs(position_error) <= self.position_tolerance_m
            and abs(velocity_m_s) <= self.velocity_tolerance_m_s
        )
        if settled_at_target or crossed_target:
            self._dwell_remaining_s = self.dwell_s
            if self._dwell_remaining_s == 0.0:
                self.target_index = 1 - self.target_index
            return 0.0

        braking_speed = math.sqrt(
            2.0 * self.max_acceleration_m_s2 * abs(position_error)
        )
        target_speed = min(self.max_speed_m_s, braking_speed)
        return math.copysign(target_speed, position_error)


def trajectory_from_config(config: dict) -> AlternatingPositionTrajectory:
    return AlternatingPositionTrajectory(
        positions_m=config["positions_m"],
        max_speed_m_s=float(config["max_speed_m_s"]),
        max_acceleration_m_s2=float(config["max_acceleration_m_s2"]),
        dwell_s=float(config["dwell_s"]),
        position_tolerance_m=float(config["position_tolerance_m"]),
        velocity_tolerance_m_s=float(config["velocity_tolerance_m_s"]),
    )
