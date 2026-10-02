# Author: Luca Obwegs
"""Simulate and compare PID and LQR using the configurable analytical model."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Polygon
import numpy as np

from balancing_robot_sim.analytical import Run, controller_for_config, simulate
from balancing_robot_sim.control import lqr_gain

ROBOT: dict = {}


def add_robot_drawing(ax: plt.Axes, run: Run, index: int):
    wheel_radius = float(ROBOT["wheel_radius_m"])
    wheel_x = run.position_m[index]
    pitch = run.pitch_rad[index]
    wheel = Circle((wheel_x, wheel_radius), wheel_radius, color="#20242b")
    com_height = float(ROBOT["chassis_com_height_m"])
    chassis_size = ROBOT["chassis_size_m"]
    length, height = float(chassis_size[0]), float(chassis_size[2])
    corners = np.array(
        [
            [-length / 2, com_height - height / 2],
            [length / 2, com_height - height / 2],
            [length / 2, com_height + height / 2],
            [-length / 2, com_height + height / 2],
        ]
    )
    rotation = np.array(
        [[np.cos(pitch), -np.sin(pitch)], [np.sin(pitch), np.cos(pitch)]]
    )
    rotated = corners @ rotation.T + np.array([wheel_x, wheel_radius])
    body = Polygon(rotated, closed=True, color="#1677c8")
    ax.add_patch(wheel)
    ax.add_patch(body)
    ax.set_xlim(wheel_x - 0.45, wheel_x + 0.45)
    ax.set_ylim(0.0, wheel_radius + com_height + height)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    ax.axhline(0.0, color="#6b7280", lw=1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "mechanical.json",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--initial-pitch-deg", type=float, default=5.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    global ROBOT
    ROBOT = config["robot"]
    control = config["control"]
    dt = 1.0 / float(control["loop_hz"])
    initial_pitch = np.deg2rad(args.initial_pitch_deg)
    results: dict[str, Run] = {}
    for kind in ("pid", "lqr"):
        controller = controller_for_config(kind, config, control)
        controller.reset(velocity=float(ROBOT["initial_forward_speed_m_s"]))
        results[kind.upper()] = simulate(
            controller,
            args.duration,
            dt,
            initial_pitch,
            float(ROBOT["initial_forward_speed_m_s"]),
        )

    output = Path(__file__).parent / "results"
    output.mkdir(parents=True, exist_ok=True)
    colors = {"PID": "#e07a27", "LQR": "#166fba"}
    summary: dict[str, object] = {}
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    series = [
        ("pitch_rad", "Pitch angle (deg)", 180.0 / np.pi),
        ("pitch_rate_rad_s", "Pitch rate (deg/s)", 180.0 / np.pi),
        ("position_m", "Wheel travel (m)", 1.0),
        ("velocity_m_s", "Wheel speed (m/s)", 1.0),
    ]
    for name, run in results.items():
        summary[name] = {
            "pitch_rms_deg": float(np.sqrt(np.mean(np.rad2deg(run.pitch_rad) ** 2))),
            "peak_pitch_deg": float(np.max(np.abs(np.rad2deg(run.pitch_rad)))),
            "position_rms_m": float(np.sqrt(np.mean(run.position_m**2))),
            "step_position_error_final_m": float(
                run.position_m[-1] - run.estimated_position_m[-1]
            ),
            "fallen": run.fallen,
            "simulated_duration_s": float(run.time_s[-1]),
        }
        for ax, (field, label, scale) in zip(axes.flat, series, strict=True):
            if field == "position_m":
                ax.plot(
                    run.time_s, run.position_m, label=f"{name} physical",
                    color=colors[name], lw=2
                )
                ax.plot(
                    run.time_s, run.estimated_position_m, label=f"{name} step estimate",
                    color=colors[name], lw=1.2, ls="--", alpha=0.8
                )
            else:
                ax.plot(
                    run.time_s,
                    getattr(run, field) * scale,
                    label=name,
                    color=colors[name],
                    lw=2,
                )
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.25)
            ax.legend(frameon=False)
    axes.flat[0].set_title("Balance angle")
    axes.flat[1].set_title("Angular velocity")
    axes.flat[2].set_title("Physical travel and step-based estimate")
    axes.flat[3].set_title("Wheel velocity")
    figure.suptitle("Two-wheel inverted pendulum: PID vs LQR", fontsize=16, weight="bold")
    figure.text(0.995, 0.005, "Author: Luca Obwegs", ha="right", fontsize=8, color="#6b7280")
    figure.savefig(
        output / "controller_comparison.png",
        dpi=180,
        metadata={"Author": "Luca Obwegs"},
    )
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    for name, run in results.items():
        axes[0].plot(
            run.time_s, run.acceleration_m_s2, label=name, color=colors[name], lw=1.7
        )
        axes[1].plot(
            run.time_s, run.step_rate_hz, label=name, color=colors[name], lw=1.7
        )
    axes[0].set_ylabel("Commanded wheel acceleration (m/s^2)")
    axes[1].set_ylabel("Step frequency (steps/s)")
    axes[1].set_xlabel("Time (s)")
    axes[0].set_title("Actuator demand and step-rate limit")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(frameon=False)
    figure.text(0.995, 0.005, "Author: Luca Obwegs", ha="right", fontsize=8, color="#6b7280")
    figure.savefig(
        output / "control_effort.png",
        dpi=180,
        metadata={"Author": "Luca Obwegs"},
    )
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(11, 4.6), constrained_layout=True)
    frame_step = max(1, int(round(1.0 / (dt * 20.0))))
    frames = np.arange(
        0, max(len(run.time_s) for run in results.values()), frame_step
    )

    def update(frame):
        for ax, (name, run) in zip(axes, results.items(), strict=True):
            index = min(int(frame), len(run.time_s) - 1)
            ax.clear()
            add_robot_drawing(ax, run, index)
            fallen_label = " - FALLEN" if run.fallen else ""
            ax.set_title(f"{name}: {run.time_s[index]:.2f} s{fallen_label}")
        return []

    animation = FuncAnimation(figure, update, frames=frames, interval=50, blit=False)
    figure.text(0.995, 0.005, "Author: Luca Obwegs", ha="right", fontsize=8, color="#6b7280")
    animation.save(
        output / "balancing_animation.gif",
        writer=PillowWriter(fps=20, metadata={"artist": "Luca Obwegs"}),
    )
    plt.close(figure)

    lqr_controller = controller_for_config("lqr", config, control)
    summary["author"] = "Luca Obwegs"
    summary["design"] = {
        "lqr_gain": lqr_gain(
            lqr_controller.mechanics, lqr_controller.parameters
        ).tolist()
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Saved figures and animation to {output}")


if __name__ == "__main__":
    main()
