# Author: Luca Obwegs
# BalancingRobot

A two-wheel self-balancing robot using an STM32G474RE, an
X-NUCLEO-IKS01A1 inertial sensor board, and A4988 stepper drivers. The project
contains a Python controller comparison, a Gazebo Harmonic/ROS 2 simulation,
and CubeMX-compatible STM32 firmware.

## Simulation and controller comparison

See [simulation/README.md](simulation/README.md) for Ubuntu 24.04, ROS 2
Jazzy, and Gazebo Harmonic setup and controls. Mechanical parameters are
editable in `simulation/config/mechanical.json`. The default LQR simulation
uses 1x simulation time and moves smoothly between two configurable positions.
Run
`simulation/run_comparison.py` for deterministic analytical PID/LQR plots, a
1x-speed GIF animation, and numerical metrics. These generated figures are
from the analytical model, not from Gazebo.

The model is an inverted pendulum driven by wheel acceleration and step-rate
limited actuators. Both controllers use IMU pitch/rate and step-command-based
position/speed estimates, matching the robot's lack of wheel encoders. LQR
gains are synthesized from the editable parameters; use
`simulation/balancing_robot_sim/export_firmware_config.py` to update the
firmware's generated motion limits and gains.

## STM32 firmware

Open `STM32Code/BalancingRobot/.project` in STM32CubeIDE. The project keeps
the CubeMX-generated peripheral initialization. Application code is in
`Core/Src/robot_app.c`; set board-specific signs and sensor address in
`Core/Inc/robot_config.h`. The default example mechanical parameters are
placeholders and must be replaced with measured values before balancing.

| Function | Pin / peripheral |
|---|---|
| Right motor STEP | PA8 / TIM1_CH1 |
| Left motor STEP | PB4 / TIM3_CH1 |
| Left motor DIR | PA9 |
| Right motor DIR | PB10 |
| A4988 ENABLE 1 / 2 | PB0 / PA4; active low, disabled at startup |
| IMU I2C | PB8 / PB9 / I2C1 |
| Serial commands and telemetry | PA2 / PA3 / LPUART1 |
| 500 Hz control tick | TIM6 |

Serial control and telemetry are described in the simulation guide. Firmware
starts with both A4988 drivers disabled; use a serial `arm` command only
after checking the IMU sign and supporting the robot. Provide an independent
physical motor power cutoff. The A4988 accepts step/dir commands rather than
torque, and missed steps cannot be detected without encoders.

The X-NUCLEO-IKS01A1 carries an LSM6DS0 IMU. This application uses the STM32
Cube HAL I2C interface and reads the sensor directly; the free
STMicroelectronics X-CUBE-MEMS1 package is available at
[github.com/STMicroelectronics/x-cube-mems1](https://github.com/STMicroelectronics/x-cube-mems1).
The package's examples target newer expansion boards, so it is not copied
into this project. Review ST's package and component licenses before adding
vendor library files.

## Current example comparison

The checked-in plots use illustrative default parameters, a 5-degree initial
lean, an 8-second analytical simulation, and no simulated missed steps.
They are examples, not predictions of the assembled robot.

| Controller | Pitch RMS | Peak pitch | Target changes | Simulated duration | Fall |
|---|---:|---:|---:|---:|---|
| PID | 3.05 deg | 5.01 deg | 0 | 8.0 s | No |
| LQR | 1.48 deg | 5.03 deg | 2 | 8.0 s | No |

With the illustrative PID gains, PID does not reach the first route endpoint
in this run; the default LQR run visits both configured positions. Tune the
PID gains for the measured robot before using PID for autonomous path tracking.

Results are in `simulation/results/`. Run
`python -m unittest discover -s simulation/tests -v` to execute the focused
unit tests.
