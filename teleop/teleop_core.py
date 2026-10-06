"""Keyboard teleoperation logic for the balancing robot (UI independent).

Author: Luca Obwegs

The firmware accepts `v <m/s>` (forward speed) and `t <rad/s>` (turn rate,
positive = left/counter-clockwise because the right wheel then runs faster).
Both fall back to zero 0.5 s after the last command, so the sender repeats
non-zero commands periodically while a key is held.
"""

from __future__ import annotations

import math
import queue
import threading
from dataclasses import dataclass, field
from typing import Iterable

FIRMWARE_MAX_SPEED_M_S = 0.8
FIRMWARE_MAX_TURN_RAD_S = 4.0
DEFAULT_BAUD = 115200
TELEMETRY_SCALE = 10000.0
SLOW_FACTOR = 0.3

FORWARD_KEYS = {"w", "up"}
BACKWARD_KEYS = {"s", "down"}
LEFT_KEYS = {"a", "left"}
RIGHT_KEYS = {"d", "right"}
SLOW_KEYS = {"shift_l", "shift_r"}
DRIVE_KEYS = FORWARD_KEYS | BACKWARD_KEYS | LEFT_KEYS | RIGHT_KEYS | SLOW_KEYS


def normalize_key(keysym: str) -> str:
    return keysym.lower()


@dataclass
class DriveLimits:
    max_speed_m_s: float = 0.25
    max_turn_rad_s: float = 1.5
    accel_m_s2: float = 0.5
    turn_accel_rad_s2: float = 4.0
    invert_turn: bool = False

    def clamped(self) -> "DriveLimits":
        return DriveLimits(
            max_speed_m_s=min(max(self.max_speed_m_s, 0.0), FIRMWARE_MAX_SPEED_M_S),
            max_turn_rad_s=min(max(self.max_turn_rad_s, 0.0), FIRMWARE_MAX_TURN_RAD_S),
            accel_m_s2=max(self.accel_m_s2, 1e-3),
            turn_accel_rad_s2=max(self.turn_accel_rad_s2, 1e-3),
            invert_turn=self.invert_turn,
        )


def target_from_keys(keys: Iterable[str], limits: DriveLimits) -> tuple[float, float]:
    """Return the (speed, turn rate) target for the currently pressed keys."""
    pressed = {normalize_key(k) for k in keys}
    limits = limits.clamped()
    forward = (1 if pressed & FORWARD_KEYS else 0) - (1 if pressed & BACKWARD_KEYS else 0)
    left = (1 if pressed & LEFT_KEYS else 0) - (1 if pressed & RIGHT_KEYS else 0)
    scale = SLOW_FACTOR if pressed & SLOW_KEYS else 1.0
    turn_sign = -1.0 if limits.invert_turn else 1.0
    return (forward * limits.max_speed_m_s * scale,
            turn_sign * left * limits.max_turn_rad_s * scale)


def ramp(current: float, target: float, max_step: float) -> float:
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


@dataclass
class DriveCommander:
    """Rate-limits the key target and decides which commands to send."""

    limits: DriveLimits = field(default_factory=DriveLimits)
    keepalive_s: float = 0.1
    speed: float = 0.0
    turn: float = 0.0
    _sent: tuple[float, float] | None = None
    _since_send: float = 0.0

    def halt(self) -> list[str]:
        """Zero both commands immediately, bypassing the ramp."""
        self.speed = self.turn = 0.0
        self._sent = (0.0, 0.0)
        self._since_send = 0.0
        return [format_speed(0.0), format_turn(0.0)]

    def reset(self) -> None:
        self.speed = self.turn = 0.0
        self._sent = None
        self._since_send = 0.0

    def step(self, keys: Iterable[str], dt: float) -> list[str]:
        limits = self.limits.clamped()
        target_speed, target_turn = target_from_keys(keys, limits)
        self.speed = ramp(self.speed, target_speed, limits.accel_m_s2 * dt)
        self.turn = ramp(self.turn, target_turn, limits.turn_accel_rad_s2 * dt)
        command = (round(self.speed, 3), round(self.turn, 3))
        self._since_send += dt
        moving = command != (0.0, 0.0)
        changed = command != self._sent
        if changed or (moving and self._since_send >= self.keepalive_s):
            self._sent = command
            self._since_send = 0.0
            return [format_speed(command[0]), format_turn(command[1])]
        return []


def format_speed(value: float) -> str:
    return f"v {value + 0.0:.3f}"


def format_turn(value: float) -> str:
    return f"t {value + 0.0:.3f}"


@dataclass
class Telemetry:
    pitch_deg: float
    position_m: float
    velocity_m_s: float
    command_velocity_m_s: float
    controller: str
    armed: bool


def parse_telemetry(line: str) -> Telemetry | None:
    """Parse the idle `T,pitch,pos,vel,cmd_vel,controller,armed` line."""
    parts = line.strip().split(",")
    if len(parts) != 7 or parts[0] != "T":
        return None
    try:
        pitch, position, velocity, command = (int(p) / TELEMETRY_SCALE for p in parts[1:5])
        controller = int(parts[5])
        armed = int(parts[6])
    except ValueError:
        return None
    return Telemetry(
        pitch_deg=math.degrees(pitch),
        position_m=position,
        velocity_m_s=velocity,
        command_velocity_m_s=command,
        controller="LQR" if controller == 1 else "PID",
        armed=bool(armed),
    )


def parse_fault(line: str) -> str | None:
    line = line.strip()
    return line[6:] if line.startswith("FAULT,") else None


def list_serial_ports() -> list[tuple[str, str]]:
    """Return (device, description) pairs, ST-LINK ports first."""
    from serial.tools import list_ports

    ports = [(p.device, p.description or p.device) for p in list_ports.comports()]
    return sorted(ports, key=lambda p: ("stlink" not in p[1].lower().replace("-", ""), p[0]))


class SerialLink:
    """Line-based serial connection with a background reader thread.

    Received lines and errors are delivered through `events` as
    ("line", text) or ("error", text) tuples so the UI thread can poll them.
    """

    def __init__(self) -> None:
        self.events: "queue.Queue[tuple[str, str]]" = queue.Queue()
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._serial is not None

    def open(self, port: str, baud: int = DEFAULT_BAUD) -> None:
        import serial

        self.close()
        self._serial = serial.serial_for_url(port, baudrate=baud, timeout=0.1,
                                             write_timeout=0.5)
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, args=(self._serial,),
                                        daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def send(self, line: str) -> bool:
        if self._serial is None:
            return False
        try:
            with self._write_lock:
                self._serial.write((line + "\n").encode("ascii"))
            return True
        except Exception as error:
            self.events.put(("error", f"write failed: {error}"))
            return False

    def _read_loop(self, serial_port) -> None:
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = serial_port.read(serial_port.in_waiting or 1)
            except Exception as error:
                if not self._stop.is_set():
                    self.events.put(("error", f"read failed: {error}"))
                return
            if not chunk:
                continue
            buffer += chunk
            *lines, buffer = buffer.replace(b"\r", b"\n").split(b"\n")
            for raw in lines:
                if raw:
                    self.events.put(("line", raw.decode("ascii", errors="replace")))
            if len(buffer) > 4096:
                buffer = b""
