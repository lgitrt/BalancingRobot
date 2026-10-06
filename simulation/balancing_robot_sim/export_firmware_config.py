# Author: Luca Obwegs
"""Generate firmware motion limits and LQR gains from the simulation settings."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from balancing_robot_sim.analytical import controller_for_config
from balancing_robot_sim.control import lqr_gain


def c_float(value: float) -> str:
    text = f"{value:.9g}"
    if "." not in text and "e" not in text:
        text += ".0"
    return f"{text}f"


def export(config_path: Path, output_path: Path) -> Path:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    robot = config["robot"]
    controller = controller_for_config("lqr", config, config["control"])
    gain = lqr_gain(controller.mechanics, controller.parameters)
    mech = controller.mechanics
    inertia = mech.equivalent_pitch_inertia
    mass_height = mech.chassis_mass_kg * mech.chassis_com_height_m
    macros = {
        "ROBOT_CONTROL_HZ": c_float(config["control"]["loop_hz"]),
        "ROBOT_POSITION_HOLD_ENABLED": "1U" if config["control"]["position_hold"]["enabled"] else "0U",
        "ROBOT_POSITION_HOLD_KP": c_float(config["control"]["position_hold"]["kp_s_inv"]),
        "ROBOT_POSITION_HOLD_MAX_SPEED_M_S": c_float(config["control"]["position_hold"]["max_correction_speed_m_s"]),
        "ROBOT_POSITION_HOLD_MAX_ERROR_M": c_float(config["control"]["position_hold"]["max_position_error_m"]),
        "ROBOT_POSITION_HOLD_MAX_TRIM_DEG": c_float(config["control"]["position_hold"]["max_learned_trim_deg"]),
        "ROBOT_WHEEL_RADIUS_M": c_float(robot["wheel_radius_m"]),
        "ROBOT_WHEEL_SEPARATION_M": c_float(robot["wheel_separation_m"]),
        "ROBOT_STEP_ANGLE_DEG": c_float(robot["wheel_step_angle_deg"]),
        "ROBOT_MICROSTEPS": c_float(robot["microsteps"]),
        "ROBOT_GEAR_RATIO": c_float(robot["gear_ratio"]),
        "ROBOT_MAX_SPEED_M_S": c_float(robot["max_wheel_speed_m_s"]),
        "ROBOT_MAX_ACCEL_M_S2": c_float(robot["max_wheel_accel_m_s2"]),
        "ROBOT_MAX_STEP_RATE_HZ": c_float(robot["motor_step_rate_limit_hz"]),
        "ROBOT_FALL_ANGLE_RAD": c_float(robot["fall_angle_rad"]),
        "ROBOT_PID_PITCH_KP": c_float(config["control"]["pid"]["pitch_kp"]),
        "ROBOT_PID_PITCH_KI": c_float(config["control"]["pid"]["pitch_ki"]),
        "ROBOT_PID_PITCH_KD": c_float(config["control"]["pid"]["pitch_kd"]),
        "ROBOT_PID_VELOCITY_KP": c_float(config["control"]["pid"]["velocity_kp"]),
        "ROBOT_PID_POSITION_KP": c_float(0.0),
        "ROBOT_LQR_K_INTEGRAL": c_float(gain[0]),
        "ROBOT_LQR_K_POSITION": c_float(gain[1]),
        "ROBOT_LQR_K_VELOCITY": c_float(gain[2]),
        "ROBOT_LQR_K_PITCH": c_float(gain[3]),
        "ROBOT_LQR_K_PITCH_RATE": c_float(gain[4]),
        # Linearised pitch plant theta_ddot = alpha*theta - beta*a - damping*omega
        "ROBOT_PLANT_ALPHA": c_float(mass_height * 9.80665 / inertia),
        "ROBOT_PLANT_BETA": c_float(mass_height / inertia),
        "ROBOT_PLANT_DAMPING": c_float(mech.pitch_damping_n_m_s / inertia),
        "ROBOT_MOTOR_TIME_CONSTANT_S": c_float(robot["motor_time_constant_s"]),
    }
    content = [
        "/* Author: Luca Obwegs */",
        "/* Generated from simulation/config/mechanical.json; do not edit by hand. */",
        "#ifndef ROBOT_CONFIG_GENERATED_H",
        "#define ROBOT_CONFIG_GENERATED_H",
        "",
    ]
    content.extend(f"#define {name} {value}" for name, value in macros.items())
    content.extend(["", "#endif", ""])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(content), encoding="utf-8")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[2]
    parser.add_argument(
        "--config",
        type=Path,
        default=root / "simulation" / "config" / "mechanical.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root
        / "STM32Code"
        / "BalancingRobot"
        / "Core"
        / "Inc"
        / "robot_config_generated.h",
    )
    args = parser.parse_args()
    print(export(args.config, args.output))


if __name__ == "__main__":
    main()
