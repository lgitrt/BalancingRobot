<!-- Author: Luca Obwegs -->
# Keyboard remote control

A Tkinter desktop app for driving the balancing robot with the keyboard over
UART (the ST-LINK virtual COM port, LPUART1, 115200 baud). It uses the firmware
commands `v`, `t`, `arm`, `stop`, `pid`, `lqr`, `hold`, `estimator` and
`wheelfb`.

![Remote control app](teleop_screenshot.png)

## Start

```powershell
python -m pip install -r teleop\requirements.txt
python teleop\robot_teleop.py
```

1. Select the COM port (ST-LINK ports are listed first; ↻ refreshes the list).
   Keep 115200 baud and press **Connect**.
2. Hold the robot upright and press **Enter** (or **Arm**).
3. Drive with the keyboard. Release all keys to stop moving.
4. Press **Esc** (or **STOP / disarm**) to switch the motors off.

After a reset the firmware is already set up for driving: `lqr`, `hold on`,
`estimator offsetkf` and `wheelfb kf` are the boot defaults, so no other
command is needed before **Arm**.

The port, baud rate and drive settings are saved in
`~/.balancing_robot_teleop.json` and restored at the next start.

## Keys

| Key | Action |
|---|---|
| W / ↑ | drive forward |
| S / ↓ | drive backward |
| A / ← | turn left (counter-clockwise seen from above) |
| D / → | turn right |
| W + A, S + D, ... | drive a curve |
| A or D alone | turn on the spot |
| Shift (hold) | slow mode: 30 % of the max speed and turn rate |
| Space | halt: zero speed and turn immediately, robot keeps balancing |
| Enter | arm |
| Esc | stop / disarm (motors off, the robot falls if not held) |

Driving keys are ignored while the console input field has focus, so you can
type commands. Press Esc there to return to driving, or click the keypad.
All keys are released when the window loses focus.

## How it drives

- Each key sets a target. The speed ramps towards it with the **Acceleration**
  setting, and the turn rate ramps at twice the max turn rate per second. This
  keeps the balancing controller from being kicked by steps.
- **Max speed** (up to the firmware limit of 0.8 m/s) and **Max turn rate**
  (up to 4 rad/s) are set with the sliders. The defaults are 0.25 m/s and
  1.5 rad/s.
- The firmware sets speed and turn back to zero 0.5 s after the last command.
  The app repeats non-zero commands every 0.1 s while a key is held and sends
  `v 0` / `t 0` once when the command reaches zero. If the app or the cable
  dies, the robot stops by itself within 0.5 s.
- A positive `t` speeds up the right wheel. **Invert turn direction** swaps
  A and D if your wheels are wired the other way round.
- With `hold on` the position-hold reference moves with the commanded speed,
  so the robot holds the spot where you release the keys.

## Display

- **Robot state** shows ARMED / DISARMED, the latched `FAULT,<reason>` (for
  example `fall`; reset the board to clear it) or NO TELEMETRY.
- The values and the chart (pitch and velocity over the last 10 s) come from
  the 10 Hz `T` telemetry line, which the firmware sends armed and disarmed.
- The serial console shows all replies. `T` rows are hidden unless
  **Show data rows** is checked. You can type any firmware command in the
  input field.

The firmware host test `STM32Code/tests/test_robot.c` drives the simulated
robot the way the app does (`v`/`t` keep-alives every 0.1 s, then a single
`v 0`/`t 0`) and checks that it follows the moving hold reference, turns by
ω·b, holds the new position after release and stops after a lost link.

## Files

| File | Content |
|---|---|
| `robot_teleop.py` | Tkinter user interface |
| `teleop_core.py` | key mapping, ramping, command sending, telemetry parsing, serial link (no UI) |
| `test_teleop_core.py` | unit tests and a UI smoke test over pyserial's `loop://` port |
| `requirements.txt` | `pyserial` |

## Tests

```powershell
python -m unittest discover -s teleop -p "test_*.py"
```

The tests check the key mapping, slow mode, limits, ramping, the keep-alive
timing, the final zero command and halt. They also cover telemetry and fault
parsing, a serial loopback, and the app itself: it connects to `loop://`,
drives with simulated key presses, reads injected telemetry and a fault, and
ignores driving keys while typing.
