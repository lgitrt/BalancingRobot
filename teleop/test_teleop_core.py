"""Tests for the keyboard remote control.

Author: Luca Obwegs

Run:  python -m unittest discover -s teleop -p "test_*.py"
"""

from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))

import teleop_core as core  # noqa: E402


class TargetTests(unittest.TestCase):
    def test_keys_map_to_speed_and_turn(self):
        limits = core.DriveLimits(max_speed_m_s=0.3, max_turn_rad_s=2.0)
        self.assertEqual(core.target_from_keys([], limits), (0.0, 0.0))
        self.assertEqual(core.target_from_keys(["w"], limits), (0.3, 0.0))
        self.assertEqual(core.target_from_keys(["Down"], limits), (-0.3, 0.0))
        self.assertEqual(core.target_from_keys(["a"], limits), (0.0, 2.0))
        self.assertEqual(core.target_from_keys(["Right", "Up"], limits), (0.3, -2.0))
        self.assertEqual(core.target_from_keys(["w", "s"], limits), (0.0, 0.0))

    def test_slow_mode_and_invert(self):
        limits = core.DriveLimits(max_speed_m_s=0.5, max_turn_rad_s=2.0, invert_turn=True)
        speed, turn = core.target_from_keys(["w", "a", "Shift_L"], limits)
        self.assertAlmostEqual(speed, 0.5 * core.SLOW_FACTOR)
        self.assertAlmostEqual(turn, -2.0 * core.SLOW_FACTOR)

    def test_limits_are_clamped_to_firmware(self):
        limits = core.DriveLimits(max_speed_m_s=5.0, max_turn_rad_s=10.0)
        self.assertEqual(core.target_from_keys(["w", "a"], limits),
                         (core.FIRMWARE_MAX_SPEED_M_S, core.FIRMWARE_MAX_TURN_RAD_S))


class CommanderTests(unittest.TestCase):
    def test_ramp_rate_limits_speed(self):
        commander = core.DriveCommander(core.DriveLimits(max_speed_m_s=0.4, accel_m_s2=1.0))
        self.assertEqual(commander.step(["w"], 0.1), ["v 0.100", "t 0.000"])
        commander.step(["w"], 0.1)
        commander.step(["w"], 0.1)
        commander.step(["w"], 0.1)
        commander.step(["w"], 0.1)
        self.assertAlmostEqual(commander.speed, 0.4)

    def test_keepalive_while_moving_and_silence_when_idle(self):
        commander = core.DriveCommander(core.DriveLimits(max_speed_m_s=0.1, accel_m_s2=10.0))
        self.assertEqual(commander.step([], 0.05), ["v 0.000", "t 0.000"])
        self.assertEqual(commander.step([], 0.05), [])
        self.assertEqual(commander.step([], 1.0), [])
        self.assertEqual(commander.step(["w"], 0.05), ["v 0.100", "t 0.000"])
        self.assertEqual(commander.step(["w"], 0.05), [])
        # Repeated well inside the 0.5 s firmware timeout.
        self.assertEqual(commander.step(["w"], 0.05), ["v 0.100", "t 0.000"])

    def test_release_ramps_down_and_sends_final_zero(self):
        commander = core.DriveCommander(core.DriveLimits(max_speed_m_s=0.2, accel_m_s2=1.0))
        for _ in range(5):
            commander.step(["w"], 0.05)
        sent = []
        for _ in range(10):
            sent += commander.step([], 0.05)
        self.assertEqual(sent[-2:], ["v 0.000", "t 0.000"])
        self.assertEqual(commander.step([], 0.05), [])

    def test_halt_bypasses_ramp(self):
        commander = core.DriveCommander(core.DriveLimits(max_speed_m_s=0.3, accel_m_s2=10.0))
        commander.step(["w", "a"], 0.1)
        self.assertEqual(commander.halt(), ["v 0.000", "t 0.000"])
        self.assertEqual((commander.speed, commander.turn), (0.0, 0.0))
        self.assertEqual(commander.step([], 0.05), [])

    def test_negative_zero_is_formatted_as_zero(self):
        self.assertEqual(core.format_speed(-0.0), "v 0.000")
        self.assertEqual(core.format_turn(round(-0.0004, 3)), "t 0.000")


class ParseTests(unittest.TestCase):
    def test_parse_telemetry(self):
        telemetry = core.parse_telemetry("T,175,-1234,500,600,1,1\r\n")
        self.assertIsNotNone(telemetry)
        self.assertAlmostEqual(telemetry.pitch_deg, 1.0027, places=3)
        self.assertAlmostEqual(telemetry.position_m, -0.1234)
        self.assertAlmostEqual(telemetry.velocity_m_s, 0.05)
        self.assertAlmostEqual(telemetry.command_velocity_m_s, 0.06)
        self.assertEqual(telemetry.controller, "LQR")
        self.assertTrue(telemetry.armed)
        self.assertEqual(core.parse_telemetry("T,0,0,0,0,0,0").controller, "PID")

    def test_parse_rejects_other_lines(self):
        for line in ("J,1,2,3", "T,1,2,3,4,5", "T,a,0,0,0,0,0", "TRAJ,START", ""):
            self.assertIsNone(core.parse_telemetry(line))

    def test_parse_fault(self):
        self.assertEqual(core.parse_fault("FAULT,fall\r\n"), "fall")
        self.assertIsNone(core.parse_fault("T,0,0,0,0,0,0"))


class SerialLinkTests(unittest.TestCase):
    def test_loopback_lines(self):
        link = core.SerialLink()
        link.open("loop://")
        try:
            self.assertTrue(link.send("v 0.100"))
            self.assertTrue(link.send("t -1.000"))
            received = []
            deadline = time.monotonic() + 2.0
            while len(received) < 2 and time.monotonic() < deadline:
                try:
                    received.append(link.events.get(timeout=0.1))
                except Exception:
                    pass
            self.assertEqual(received, [("line", "v 0.100"), ("line", "t -1.000")])
        finally:
            link.close()
        self.assertFalse(link.connected)
        self.assertFalse(link.send("v 0"))


class AppSmokeTests(unittest.TestCase):
    def setUp(self):
        try:
            import robot_teleop
        except Exception as error:  # pragma: no cover - missing tkinter
            self.skipTest(f"tkinter unavailable: {error}")
        self.module = robot_teleop
        self.tmp = tempfile.TemporaryDirectory()
        self.module.SETTINGS_PATH = Path(self.tmp.name) / "settings.json"
        try:
            self.app = robot_teleop.TeleopApp()
        except Exception as error:  # pragma: no cover - no display
            self.skipTest(f"no display: {error}")
        self.app.withdraw()

    def tearDown(self):
        self.app._on_close()
        self.tmp.cleanup()

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.app.update()
            time.sleep(0.01)

    def log_text(self):
        return self.app.log.get("1.0", "end")

    def test_keyboard_drives_over_loopback(self):
        app = self.app
        app.port_var.set("loop://")
        app._toggle_connection()
        self.assertTrue(app.link.connected)
        app.drive_canvas.focus_set()
        self.pump(0.1)
        app._on_key_press(SimpleNamespace(keysym="w"))
        app._on_key_press(SimpleNamespace(keysym="a"))
        self.pump(0.6)
        self.assertGreater(app.commander.speed, 0.0)
        self.assertGreater(app.commander.turn, 0.0)
        # The loopback echoes the sent commands as received lines.
        self.assertRegex(self.log_text(), r"v 0\.\d+")
        self.assertRegex(self.log_text(), r"t [1-9]\.\d+|t 0\.[1-9]\d*")
        app._on_key_release(SimpleNamespace(keysym="w"))
        app._on_key_release(SimpleNamespace(keysym="a"))
        self.pump(0.1)
        self.assertEqual(app.pressed, set())
        app._on_key_press(SimpleNamespace(keysym="space"))
        self.assertEqual((app.commander.speed, app.commander.turn), (0.0, 0.0))
        app.link.events.put(("line", "T,175,0,0,0,1,1"))
        app.link.events.put(("line", "FAULT,fall"))
        self.pump(0.15)
        self.assertEqual(app._telemetry.controller, "LQR")
        self.assertIn("FAULT", str(app.state_label.cget("text")))
        app._toggle_connection()
        self.assertFalse(app.link.connected)
        self.assertTrue(self.module.SETTINGS_PATH.exists())

    def test_keys_ignored_while_typing(self):
        app = self.app
        entry = next(w for w in app.nametowidget(".").winfo_children()[0]
                     .winfo_children()[-1].winfo_children() if w.winfo_class() == "TEntry")
        app.focus_get = lambda: entry
        self.assertIsNone(app._on_key_press(SimpleNamespace(keysym="w")))
        self.assertEqual(app.pressed, set())
        app.focus_get = lambda: app.drive_canvas
        self.assertEqual(app._on_key_press(SimpleNamespace(keysym="w")), "break")
        self.assertEqual(app.pressed, {"w"})


if __name__ == "__main__":
    unittest.main()
