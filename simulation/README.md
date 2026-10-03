# Author: Luca Obwegs

# Balancing robot simulation

This ROS 2 Jazzy / Gazebo Harmonic package contains a two-wheel inverted
pendulum model, a shared PID/LQR controller, keyboard teleoperation, and a
separate analytical model for repeatable plots and controller comparisons.
It targets Ubuntu 24.04. Gazebo itself is not available on native Windows.
Gazebo is configured for a real-time factor of 1, and the ROS controller uses
simulation time so its control steps track the simulator clock.

## Install

Install ROS 2 Jazzy, Gazebo Harmonic, `ros-jazzy-ros-gz`, and
`ros-jazzy-teleop-twist-keyboard` using their official Ubuntu instructions.
Then build from the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r simulation/requirements.txt
source /opt/ros/jazzy/setup.bash
cd simulation
colcon build --symlink-install
source install/setup.bash
```

Launch the Gazebo model and controller:

```bash
ros2 launch balancing_robot_sim simulation.launch.py controller:=lqr
```

Choose `controller:=pid` to test PID. Change controllers by restarting the
launch. In a second terminal, source ROS and the workspace, then use:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/balance/cmd_vel
```

`linear.x` is the forward-speed request; `angular.z` produces differential
left/right wheel motion. The balance controller uses the IMU and the
commanded step-speed odometry estimate; Gazebo odometry is bridged for
visualization and comparison only. This mirrors the real robot's lack of
wheel encoders. Consequently, Gazebo's ideal odometry must not be used to
claim equivalent real-world state feedback.

By default, the controller automatically travels between the two positions
configured in `trajectory.positions_m` (`-0.3 m` and `+0.3 m`). It uses a
braking-distance speed profile, configurable speed/acceleration limits, and a
dwell at each endpoint. A recent keyboard command temporarily takes control;
the automatic trajectory resumes after command input times out.
The default launch controller is LQR. With the example gains, PID balances
but may not track this position route; retune it for the measured mechanics
before selecting `controller:=pid` for autonomous movement.

The launch file generates the model SDF from `config/mechanical.json` and
`models/balancing_robot/model.sdf.in`. Edit the physical parameters in that
JSON file before launch, including masses, inertias, wheel dimensions,
wheel separation, motor step angle, microstep setting, and gear ratio.
Also configure contact friction and the analytical pitch damping. The initial
values are illustrative placeholders, not measurements of
Luca's robot. LQR gains are synthesized from those values at runtime.
To synchronize the generated firmware motion limits and PID/LQR gains after
changing the mechanical or controller settings, run from the repository root:

```bash
python simulation/balancing_robot_sim/export_firmware_config.py
```

Then rebuild the STM32CubeIDE project. Sensor-axis and left/right motor
direction signs are installation-specific; set them in
`STM32Code/BalancingRobot/Core/Inc/robot_config.h`.

The board firmware boots disarmed. Connect a serial terminal to the Nucleo
virtual COM port at 115200 baud and send newline-terminated commands:
`arm`, `disarm`, `pid`, `lqr`, `v 0.1` (forward speed in m/s), or
`t 0.5` (turn rate in rad/s). Commands expire after 0.5 seconds, so stale
velocity requests return to zero. Telemetry lines begin with
`T,pitch_e-4,position_e-4,speed_e-4,command_speed_e-4,controller,armed`;
the four measurements are signed integers scaled by 1e-4. Fault lines begin
with `FAULT,`. Verify IMU axis/sign while the drivers remain disabled before
sending `arm`. The software arm is not an emergency stop.

## Controller comparison, plots, and animation

Run from the repository root:

```bash
python simulation/run_comparison.py --config simulation/config/mechanical.json
```

Outputs are written to `simulation/results/`:

* `controller_comparison.png`: pitch, pitch rate, physical/estimated wheel position, and wheel speed.
* `control_effort.png`: commanded acceleration and step frequency.
* `balancing_animation.gif`: side-view animation of both controllers.
* `summary.json`: RMS errors, peak lean, and fall status.

These are outputs of the deterministic nonlinear **analytical model**, not
Gazebo. A fall is declared once the absolute lean exceeds the configured
fall angle. Use `--duration` to change the experiment length.

To run unit tests:

```bash
python -m unittest discover -s simulation/tests -v
```

The IMU pitch axis/sign and motor direction signs depend on how the board and
motors are mounted. Verify and configure those before enabling the real
motors. Start hardware tests with the robot supported and an independent
motor power cutoff. The A4988 has no torque command: firmware can command
step rate, but missed steps are not observable without encoders.
