# Author: Luca Obwegs
"""Generate the Gazebo SDF model using user-editable mechanical parameters."""

from __future__ import annotations

import json
from pathlib import Path
from string import Template


def generate(config_path: Path, template_path: Path, output_path: Path) -> Path:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    robot = config["robot"]
    i = robot["chassis_inertia_kg_m2"]
    values = {
        "chassis_mass": robot["chassis_mass_kg"],
        "com_height": robot["chassis_com_height_m"],
        "initial_z": robot["wheel_radius_m"],
        "joint_z": -robot["chassis_com_height_m"],
        "chassis_ixx": i["ixx"],
        "chassis_iyy": i["iyy"],
        "chassis_izz": i["izz"],
        "chassis_x": robot["chassis_size_m"][0],
        "chassis_y": robot["chassis_size_m"][1],
        "chassis_z": robot["chassis_size_m"][2],
        "wheel_mass": robot["wheel_mass_kg"],
        "wheel_radius": robot["wheel_radius_m"],
        "wheel_width": robot["wheel_width_m"],
        "wheel_separation": robot["wheel_separation_m"],
        "wheel_half_separation": robot["wheel_separation_m"] / 2,
        "wheel_minus_half_separation": -robot["wheel_separation_m"] / 2,
        "wheel_ground_friction": robot["wheel_ground_friction"],
        "wheel_ground_friction_lateral": robot["wheel_ground_friction_lateral"],
        "wheel_axial_inertia": robot["wheel_axial_inertia_kg_m2"],
        "wheel_transverse_inertia": robot["wheel_transverse_inertia_kg_m2"],
        "initial_pitch": robot["initial_pitch_rad"],
        "max_speed": robot["max_wheel_speed_m_s"],
        "max_wheel_rate": robot["max_wheel_speed_m_s"] / robot["wheel_radius_m"],
        "max_accel": robot["max_wheel_accel_m_s2"],
        "imu_rate": config["control"]["loop_hz"],
    }
    sdf_template = Template(template_path.read_text(encoding="utf-8"))
    sdf = sdf_template.substitute(values)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(sdf, encoding="utf-8")
    return output_path


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(generate(args.config, args.template, args.output))
