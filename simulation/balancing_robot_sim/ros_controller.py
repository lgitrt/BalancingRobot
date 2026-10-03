# Author: Luca Obwegs
"""ROS 2 controller using the IMU and commanded-step (encoderless) odometry."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

from balancing_robot_sim.analytical import controller_for_config
from balancing_robot_sim.control import BalanceController
from balancing_robot_sim.trajectory import trajectory_from_config


def load_config(path: str) -> dict:
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if config.get("author") != "Luca Obwegs":
        raise ValueError("mechanical config must include its author")
    return config


def quaternion_angle(imu: Imu, axis: str) -> float:
    q = imu.orientation
    if axis == "x":
        return math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x**2 + q.y**2))
    if axis == "y":
        value = 2.0 * (q.w * q.y - q.z * q.x)
        return math.asin(max(-1.0, min(1.0, value)))
    raise ValueError(f"Unsupported pitch_axis {axis!r}; expected 'x' or 'y'")


class BalanceNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_controller")
        self.declare_parameter("config", "")
        self.declare_parameter("controller", "lqr")
        config_path = self.get_parameter("config").get_parameter_value().string_value
        kind = self.get_parameter("controller").get_parameter_value().string_value
        config = load_config(config_path)
        self.robot_config = config["robot"]
        self.control_config = config["control"]
        self.controller: BalanceController = controller_for_config(
            kind, config, self.control_config
        )
        self.mechanics = self.controller.mechanics
        trajectory_config = config.get("trajectory", {})
        self.trajectory = (
            trajectory_from_config(trajectory_config)
            if trajectory_config.get("enabled", False)
            else None
        )
        self.pitch = float(self.robot_config["initial_pitch_rad"])
        self.pitch_rate = 0.0
        self.imu_time: float | None = None
        self.command_time: float | None = None
        self.requested_speed = 0.0
        self.requested_turn = 0.0
        self.estimated_position = 0.0
        self.estimated_velocity = 0.0
        self.step_residual = [0.0, 0.0]
        self.last_tick: float | None = None
        self.fault_latched = False
        self.imu_subscription = self.create_subscription(Imu, "/imu", self.on_imu, 10)
        self.command_subscription = self.create_subscription(
            Twist, "/balance/cmd_vel", self.on_command, 10
        )
        self.publisher = self.create_publisher(
            Twist, "/model/balancing_robot/cmd_vel", 10
        )
        rate = float(self.control_config["loop_hz"])
        self.timer = self.create_timer(1.0 / rate, self.control_tick)
        trajectory_status = (
            "alternating-position trajectory enabled"
            if self.trajectory is not None
            else "trajectory disabled"
        )
        self.get_logger().info(
            f"Running {kind.upper()} control at {rate:.0f} Hz; {trajectory_status}"
        )

    def on_imu(self, msg: Imu) -> None:
        axis = self.robot_config["pitch_axis"]
        sign = float(self.robot_config["pitch_sign"])
        angular_velocity = {
            "x": msg.angular_velocity.x,
            "y": msg.angular_velocity.y,
        }[axis]
        self.pitch = sign * (
            quaternion_angle(msg, axis) - float(self.robot_config["initial_pitch_rad"])
        )
        self.pitch_rate = sign * angular_velocity
        self.imu_time = self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, msg: Twist) -> None:
        self.requested_speed = float(msg.linear.x)
        self.requested_turn = float(msg.angular.z)
        self.command_time = self.get_clock().now().nanoseconds * 1e-9

    def publish_stop(self) -> None:
        msg = Twist()
        self.publisher.publish(msg)

    def control_tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (
            1.0 / float(self.control_config["loop_hz"])
            if self.last_tick is None
            else min(max(now - self.last_tick, 0.0005), 0.02)
        )
        self.last_tick = now
        if self.imu_time is None or now - self.imu_time > 0.1:
            self.publish_stop()
            self.get_logger().error("IMU data stale; publishing zero speed")
            return
        if abs(self.pitch) >= float(self.robot_config["fall_angle_rad"]):
            self.fault_latched = True
            self.get_logger().error("Fall angle exceeded; controller latched stopped")
        if self.fault_latched:
            self.publish_stop()
            return

        timeout = float(self.control_config["command_timeout_s"])
        manual_command_active = (
            self.command_time is not None and now - self.command_time < timeout
        )
        if manual_command_active:
            target_speed = self.requested_speed
            target_turn = self.requested_turn
        elif self.trajectory is not None:
            target_speed = self.trajectory.step(
                self.estimated_position, self.estimated_velocity, dt
            )
            target_turn = 0.0
        else:
            target_speed = 0.0
            target_turn = 0.0
        speed, _, turn_offset = self.controller.step(
            self.pitch,
            self.pitch_rate,
            dt,
            target_speed,
            target_turn,
            self.estimated_position,
            self.estimated_velocity,
        )
        requested_wheel_speeds = (speed - turn_offset, speed + turn_offset)
        max_step_rate = min(
            self.mechanics.motor_step_rate_limit_hz,
            self.mechanics.max_wheel_speed_m_s / self.mechanics.meters_per_step,
        )
        achieved_wheel_speeds = []
        emitted_steps = []
        for index, requested_wheel_speed in enumerate(requested_wheel_speeds):
            step_rate = max(
                -max_step_rate,
                min(max_step_rate, requested_wheel_speed / self.mechanics.meters_per_step),
            )
            self.step_residual[index] += step_rate * dt
            steps = int(self.step_residual[index])
            self.step_residual[index] -= steps
            emitted_steps.append(steps)
            achieved_wheel_speeds.append(step_rate * self.mechanics.meters_per_step)

        distance = sum(emitted_steps) * self.mechanics.meters_per_step * 0.5
        self.estimated_position += distance
        self.estimated_velocity = sum(achieved_wheel_speeds) * 0.5
        command = Twist()
        command.linear.x = sum(achieved_wheel_speeds) * 0.5
        separation = float(self.robot_config["wheel_separation_m"])
        command.angular.z = (
            achieved_wheel_speeds[1] - achieved_wheel_speeds[0]
        ) / separation
        self.publisher.publish(command)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
