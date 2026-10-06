"""Keyboard remote control for the balancing robot over UART.

Author: Luca Obwegs

Run:  python teleop\\robot_teleop.py

W/S or Up/Down drive, A/D or Left/Right turn, hold Shift for slow mode,
Space = halt (zero speed and turn), Enter = arm, Esc = stop (disarm).
"""

from __future__ import annotations

import json
import queue
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

from teleop_core import (
    DEFAULT_BAUD,
    DRIVE_KEYS,
    FIRMWARE_MAX_SPEED_M_S,
    FIRMWARE_MAX_TURN_RAD_S,
    DriveCommander,
    DriveLimits,
    SerialLink,
    Telemetry,
    list_serial_ports,
    normalize_key,
    parse_fault,
    parse_telemetry,
)

SETTINGS_PATH = Path.home() / ".balancing_robot_teleop.json"
TICK_MS = 50
CHART_SECONDS = 10.0
TELEMETRY_STALE_S = 1.5
LOG_MAX_LINES = 2000
TEXT_INPUT_CLASSES = {"Entry", "TEntry", "TCombobox", "Text", "TSpinbox", "Spinbox"}
BAUD_RATES = ("9600", "57600", "115200", "230400", "460800", "921600")

BG = "#1e2329"
PANEL = "#272d35"
FG = "#e6e9ef"
MUTED = "#8b95a5"
ACCENT = "#3d8bfd"
GREEN = "#2fbf71"
RED = "#e5484d"
ORANGE = "#f5a524"


class TeleopApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Balancing Robot - Remote Control")
        self.configure(bg=BG)
        self.ui_scale = max(1.0, self.winfo_fpixels("1i") / 96.0)
        self.minsize(self._px(1000), self._px(700))

        self.link = SerialLink()
        self.commander = DriveCommander()
        self.pressed: set[str] = set()
        self._pending_release: dict[str, str] = {}
        self._last_tick = time.monotonic()
        self._telemetry: Telemetry | None = None
        self._telemetry_time = 0.0
        self._fault: str | None = None
        self._history: deque[tuple[float, float, float]] = deque()

        settings = self._load_settings()
        self.port_var = tk.StringVar(value=settings.get("port", ""))
        self.baud_var = tk.StringVar(value=str(settings.get("baud", DEFAULT_BAUD)))
        self.speed_var = tk.DoubleVar(value=settings.get("max_speed", 0.25))
        self.turn_var = tk.DoubleVar(value=settings.get("max_turn", 1.5))
        self.accel_var = tk.DoubleVar(value=settings.get("accel", 0.5))
        self.invert_var = tk.BooleanVar(value=settings.get("invert_turn", False))
        self.show_data_var = tk.BooleanVar(value=False)
        self.estimator_var = tk.StringVar(value="offsetkf")
        self.wheelfb_var = tk.StringVar(value="kf")
        self.command_var = tk.StringVar()

        self._setup_style()
        self._build_ui()
        self._refresh_ports()
        self._update_limits()

        self.bind_all("<KeyPress>", self._on_key_press)
        self.bind_all("<KeyRelease>", self._on_key_release)
        self.bind("<FocusOut>", lambda _e: self.after(20, self._check_focus))
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick_id = self.after(TICK_MS, self._tick)

    # ------------------------------------------------------------------ UI
    def _px(self, value: float) -> int:
        return int(round(value * self.ui_scale))

    def _setup_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=FG, fieldbackground=PANEL,
                        bordercolor="#3a414b", lightcolor=PANEL, darkcolor=PANEL,
                        font=("Segoe UI", 10))
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("TLabelframe", background=PANEL, bordercolor="#3a414b")
        style.configure("TLabelframe.Label", background=PANEL, foreground=ACCENT,
                        font=("Segoe UI Semibold", 10))
        style.configure("TLabel", background=PANEL, foreground=FG)
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Value.TLabel", font=("Consolas", 12))
        style.configure("Big.TLabel", font=("Segoe UI Semibold", 16))
        style.configure("Bar.TLabel", background=BG)
        style.configure("TButton", background="#343b45", foreground=FG, padding=(10, 5),
                        borderwidth=0, focuscolor=PANEL)
        style.map("TButton", background=[("active", "#414a56"), ("disabled", "#2b3038")],
                  foreground=[("disabled", MUTED)])
        style.configure("Accent.TButton", background=ACCENT)
        style.map("Accent.TButton", background=[("active", "#5a9cfd")])
        style.configure("Arm.TButton", background=GREEN)
        style.map("Arm.TButton", background=[("active", "#45d185")])
        style.configure("Stop.TButton", background=RED, font=("Segoe UI Semibold", 10))
        style.map("Stop.TButton", background=[("active", "#f06a6e")])
        style.configure("TCheckbutton", background=PANEL, foreground=FG)
        style.map("TCheckbutton", background=[("active", PANEL)])
        style.configure("Horizontal.TScale", background=PANEL, troughcolor="#3a414b")
        style.configure("TCombobox", arrowcolor=FG, foreground=FG)
        style.map("TCombobox", fieldbackground=[("readonly", PANEL)],
                  foreground=[("readonly", FG)])
        self.option_add("*TCombobox*Listbox.background", PANEL)
        self.option_add("*TCombobox*Listbox.foreground", FG)

    def _button(self, parent, text, command, style="TButton", width=None):
        button = ttk.Button(parent, text=text, command=command, style=style,
                            takefocus=False, width=width)
        return button

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=0)
        root.columnconfigure(1, weight=0)
        root.columnconfigure(2, weight=1)
        root.rowconfigure(3, weight=1)

        self._build_connection(root).grid(row=0, column=0, columnspan=3, sticky="ew",
                                          pady=(0, 8))
        self._build_settings(root).grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        self._build_drive(root).grid(row=1, column=1, sticky="nsew", padx=(0, 8))
        self._build_telemetry(root).grid(row=1, column=2, sticky="nsew")
        self._build_robot_commands(root).grid(row=2, column=0, columnspan=3, sticky="ew",
                                              pady=8)
        self._build_console(root).grid(row=3, column=0, columnspan=3, sticky="nsew")

    def _build_connection(self, parent) -> ttk.Frame:
        frame = ttk.LabelFrame(parent, text="Connection", padding=8)
        ttk.Label(frame, text="Port").pack(side="left")
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=38)
        self.port_combo.pack(side="left", padx=(6, 4))
        self._button(frame, "\u21bb", self._refresh_ports, width=3).pack(side="left")
        ttk.Label(frame, text="Baud").pack(side="left", padx=(14, 0))
        ttk.Combobox(frame, textvariable=self.baud_var, values=BAUD_RATES,
                     width=9).pack(side="left", padx=6)
        self.connect_button = self._button(frame, "Connect", self._toggle_connection,
                                           style="Accent.TButton", width=12)
        self.connect_button.pack(side="left", padx=(8, 0))
        self.connection_label = ttk.Label(frame, text="\u25cf  disconnected",
                                          foreground=MUTED)
        self.connection_label.pack(side="left", padx=14)
        return frame

    def _scale_row(self, frame, row, label, variable, low, high, unit, fmt):
        ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=(6, 0))
        value = ttk.Label(frame, style="Value.TLabel", width=11, anchor="e")
        value.grid(row=row, column=1, sticky="e", pady=(6, 0))
        scale = ttk.Scale(frame, from_=low, to=high, variable=variable, length=self._px(230),
                          command=lambda _v: self._update_limits(), takefocus=False)
        scale.grid(row=row + 1, column=0, columnspan=2, sticky="ew")
        variable.trace_add("write", lambda *_: value.configure(
            text=f"{fmt.format(variable.get())} {unit}"))
        value.configure(text=f"{fmt.format(variable.get())} {unit}")

    def _build_settings(self, parent) -> ttk.Frame:
        frame = ttk.LabelFrame(parent, text="Drive settings", padding=10)
        self._scale_row(frame, 0, "Max speed", self.speed_var, 0.05,
                        FIRMWARE_MAX_SPEED_M_S, "m/s", "{:.2f}")
        self._scale_row(frame, 2, "Max turn rate", self.turn_var, 0.2,
                        FIRMWARE_MAX_TURN_RAD_S, "rad/s", "{:.2f}")
        self._scale_row(frame, 4, "Acceleration", self.accel_var, 0.1, 2.0,
                        "m/s\u00b2", "{:.2f}")
        ttk.Checkbutton(frame, text="Invert turn direction", variable=self.invert_var,
                        command=self._update_limits, takefocus=False).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(10, 0))
        help_text = ("W / \u2191   forward     S / \u2193   backward\n"
                     "A / \u2190   turn left   D / \u2192   turn right\n"
                     "Shift   slow mode (30 %)\n"
                     "Space   halt        Enter   arm\n"
                     "Esc     stop / disarm")
        ttk.Label(frame, text=help_text, style="Muted.TLabel", justify="left",
                  font=("Consolas", 9)).grid(row=7, column=0, columnspan=2, sticky="w",
                                             pady=(12, 0))
        return frame

    def _build_drive(self, parent) -> ttk.Frame:
        frame = ttk.LabelFrame(parent, text="Keyboard", padding=10)
        self.drive_canvas = tk.Canvas(frame, width=self._px(240), height=self._px(270),
                                      bg=PANEL, highlightthickness=0, takefocus=True)
        self.drive_canvas.pack()
        self.drive_canvas.bind("<Button-1>", lambda _e: self.drive_canvas.focus_set())
        return frame

    def _build_telemetry(self, parent) -> ttk.Frame:
        frame = ttk.LabelFrame(parent, text="Robot state", padding=10)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(1, weight=1)
        self.state_label = ttk.Label(frame, text="NOT CONNECTED", style="Big.TLabel",
                                     foreground=MUTED)
        self.state_label.grid(row=0, column=0, sticky="nw")
        values = ttk.Frame(frame, style="Panel.TFrame")
        values.grid(row=1, column=0, sticky="nw", pady=(8, 0))
        self.value_labels: dict[str, ttk.Label] = {}
        for row, (key, text) in enumerate((("controller", "Controller"),
                                           ("pitch", "Pitch"),
                                           ("position", "Position"),
                                           ("velocity", "Velocity"),
                                           ("command", "Cmd velocity"),
                                           ("sent", "Sent v / t"))):
            ttk.Label(values, text=text, style="Muted.TLabel").grid(row=row, column=0,
                                                                   sticky="w", pady=2)
            label = ttk.Label(values, text="-", style="Value.TLabel", width=16,
                              anchor="e")
            label.grid(row=row, column=1, sticky="e", padx=(10, 0))
            self.value_labels[key] = label
        self.chart = tk.Canvas(frame, height=self._px(220), width=self._px(320),
                               bg="#20252c", highlightthickness=0)
        self.chart.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(14, 0))
        return frame

    def _build_robot_commands(self, parent) -> ttk.Frame:
        frame = ttk.LabelFrame(parent, text="Robot commands", padding=8)
        top = ttk.Frame(frame, style="Panel.TFrame")
        top.pack(fill="x")
        bottom = ttk.Frame(frame, style="Panel.TFrame")
        bottom.pack(fill="x", pady=(6, 0))
        self.command_buttons: list[ttk.Widget] = []

        def add(row, text, command, style="TButton"):
            button = self._button(row, text, command, style=style)
            button.pack(side="left", padx=3)
            self.command_buttons.append(button)

        add(top, "Arm  (Enter)", self._arm, "Arm.TButton")
        add(top, "STOP / disarm  (Esc)", self._stop, "Stop.TButton")
        add(top, "Halt  (Space)", self._halt)
        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=8)
        add(top, "PID", lambda: self._send("pid"))
        add(top, "LQR", lambda: self._send("lqr"))
        add(top, "Hold on", lambda: self._send("hold on"))
        add(top, "Hold off", lambda: self._send("hold off"))
        ttk.Label(bottom, text="Estimator").pack(side="left", padx=(3, 0))
        ttk.Combobox(bottom, textvariable=self.estimator_var, state="readonly", width=14,
                     values=("complementary", "offsetkf", "kalman")).pack(side="left",
                                                                         padx=4)
        add(bottom, "Set", lambda: self._send(f"estimator {self.estimator_var.get()}"))
        ttk.Label(bottom, text="Wheel feedback").pack(side="left", padx=(16, 0))
        ttk.Combobox(bottom, textvariable=self.wheelfb_var, state="readonly", width=7,
                     values=("kf", "steps")).pack(side="left", padx=4)
        add(bottom, "Set", lambda: self._send(f"wheelfb {self.wheelfb_var.get()}"))
        return frame

    def _build_console(self, parent) -> ttk.Frame:
        frame = ttk.LabelFrame(parent, text="Serial console", padding=8)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        self.log = ScrolledText(frame, height=10, bg="#15191e", fg=FG,
                                insertbackground=FG, relief="flat",
                                font=("Consolas", 9), state="disabled", takefocus=False)
        self.log.grid(row=0, column=0, columnspan=4, sticky="nsew")
        self.log.tag_configure("tx", foreground=ACCENT)
        self.log.tag_configure("error", foreground=RED)
        self.log.tag_configure("info", foreground=MUTED)
        entry = ttk.Entry(frame, textvariable=self.command_var)
        entry.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        entry.bind("<Return>", lambda _e: self._send_manual())
        self._button(frame, "Send", self._send_manual).grid(row=1, column=1, padx=6,
                                                            pady=(6, 0))
        ttk.Checkbutton(frame, text="Show data rows (T, J, B, ...)",
                        variable=self.show_data_var, takefocus=False).grid(
            row=1, column=2, pady=(6, 0))
        self._button(frame, "Clear", self._clear_log).grid(row=1, column=3, padx=(6, 0),
                                                           pady=(6, 0))
        return frame

    # ------------------------------------------------------------ settings
    def _load_settings(self) -> dict:
        try:
            return json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def _save_settings(self) -> None:
        settings = {
            "port": self.port_var.get().split(" ")[0],
            "baud": self.baud_var.get(),
            "max_speed": round(self.speed_var.get(), 3),
            "max_turn": round(self.turn_var.get(), 3),
            "accel": round(self.accel_var.get(), 3),
            "invert_turn": self.invert_var.get(),
        }
        try:
            SETTINGS_PATH.write_text(json.dumps(settings, indent=2), encoding="utf-8")
        except OSError:
            pass

    def _update_limits(self) -> None:
        max_turn = self.turn_var.get()
        self.commander.limits = DriveLimits(
            max_speed_m_s=self.speed_var.get(),
            max_turn_rad_s=max_turn,
            accel_m_s2=self.accel_var.get(),
            turn_accel_rad_s2=max(2.0 * max_turn, 1.0),
            invert_turn=self.invert_var.get(),
        )

    # ---------------------------------------------------------- connection
    def _refresh_ports(self) -> None:
        try:
            ports = list_serial_ports()
        except Exception as error:
            self._log(f"cannot list ports: {error}", "error")
            ports = []
        entries = [f"{device}  ({description})" for device, description in ports]
        self.port_combo.configure(values=entries)
        current = self.port_var.get().split(" ")[0]
        match = next((e for e in entries if e.split(" ")[0] == current), None)
        if match:
            self.port_var.set(match)
        elif entries and not current:
            self.port_var.set(entries[0])

    def _toggle_connection(self) -> None:
        if self.link.connected:
            self._disconnect()
            return
        port = self.port_var.get().split(" ")[0].strip()
        if not port:
            messagebox.showwarning("No port", "Select or type a COM port first.")
            return
        try:
            baud = int(self.baud_var.get())
            self.link.open(port, baud)
        except Exception as error:
            messagebox.showerror("Connection failed", f"Cannot open {port}:\n{error}")
            return
        self.commander.reset()
        self._fault = None
        self._telemetry = None
        self._history.clear()
        self.connect_button.configure(text="Disconnect")
        self.connection_label.configure(text=f"\u25cf  connected to {port} @ {baud}",
                                        foreground=GREEN)
        self._log(f"connected to {port} @ {baud} baud", "info")
        self._save_settings()
        self.drive_canvas.focus_set()

    def _disconnect(self) -> None:
        if self.link.connected:
            for line in self.commander.halt():
                self.link.send(line)
        self.link.close()
        self.pressed.clear()
        self.connect_button.configure(text="Connect")
        self.connection_label.configure(text="\u25cf  disconnected", foreground=MUTED)
        self._log("disconnected", "info")

    def _send(self, line: str, quiet: bool = False) -> None:
        if not self.link.connected:
            self._log("not connected", "error")
            return
        if self.link.send(line) and not quiet:
            self._log(f"> {line}", "tx")

    def _send_manual(self) -> None:
        line = self.command_var.get().strip()
        if line:
            self._send(line)
            self.command_var.set("")

    def _arm(self) -> None:
        self.commander.reset()
        self._send("arm")

    def _stop(self) -> None:
        self.commander.halt()
        self._send("stop")

    def _halt(self) -> None:
        if self.link.connected:
            for line in self.commander.halt():
                self._send(line)

    # ------------------------------------------------------------ keyboard
    def _typing(self) -> bool:
        widget = self.focus_get()
        return widget is not None and widget.winfo_class() in TEXT_INPUT_CLASSES

    def _on_key_press(self, event):
        key = normalize_key(event.keysym)
        if self._typing():
            if key == "escape":
                self.drive_canvas.focus_set()
                return "break"
            return None
        self._pending_release.pop(key, None)
        if key in DRIVE_KEYS:
            self.pressed.add(key)
            return "break"
        if key == "space":
            self._halt()
            return "break"
        if key in ("return", "kp_enter"):
            self._arm()
            return "break"
        if key == "escape":
            self._stop()
            return "break"
        return None

    def _on_key_release(self, event):
        key = normalize_key(event.keysym)
        if key not in self.pressed:
            return None
        # Linux key auto-repeat sends release/press pairs; only act on a real release.
        token = f"{time.monotonic()}"
        self._pending_release[key] = token
        self.after(40, lambda: self._finish_release(key, token))
        return "break"

    def _finish_release(self, key: str, token: str) -> None:
        if self._pending_release.get(key) == token:
            self._pending_release.pop(key, None)
            self.pressed.discard(key)

    def _check_focus(self) -> None:
        if self.focus_get() is None:
            self.pressed.clear()

    # --------------------------------------------------------------- loop
    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last_tick, 0.2)
        self._last_tick = now
        self._process_events(now)
        if self.link.connected:
            for line in self.commander.step(self.pressed, dt):
                self.link.send(line)
        self._draw_drive()
        self._update_state(now)
        self._draw_chart(now)
        self._tick_id = self.after(TICK_MS, self._tick)

    def _process_events(self, now: float) -> None:
        for _ in range(500):
            try:
                kind, text = self.link.events.get_nowait()
            except queue.Empty:
                break
            if kind == "error":
                self._log(text, "error")
                self._disconnect()
                continue
            telemetry = parse_telemetry(text)
            if telemetry is not None:
                self._telemetry = telemetry
                self._telemetry_time = now
                self._history.append((now, telemetry.pitch_deg, telemetry.velocity_m_s))
            fault = parse_fault(text)
            if fault is not None:
                self._fault = fault
                self.commander.halt()
                self.pressed.clear()
            is_data_row = len(text) > 1 and text[1] == "," and text[0].isupper()
            if self.show_data_var.get() or not is_data_row:
                self._log(text, "error" if fault else None)
        while self._history and now - self._history[0][0] > CHART_SECONDS:
            self._history.popleft()

    def _update_state(self, now: float) -> None:
        sent = f"{self.commander.speed:+.2f} / {self.commander.turn:+.2f}"
        self.value_labels["sent"].configure(text=sent)
        if not self.link.connected:
            self.state_label.configure(text="NOT CONNECTED", foreground=MUTED)
            return
        if self._fault:
            self.state_label.configure(text=f"FAULT: {self._fault}  (reset board)",
                                       foreground=RED)
        telemetry = self._telemetry
        stale = telemetry is None or now - self._telemetry_time > TELEMETRY_STALE_S
        if not self._fault:
            if stale:
                self.state_label.configure(text="NO TELEMETRY", foreground=ORANGE)
            elif telemetry.armed:
                self.state_label.configure(text="ARMED - BALANCING", foreground=GREEN)
            else:
                self.state_label.configure(text="DISARMED", foreground=FG)
        if telemetry is None:
            return
        self.value_labels["controller"].configure(text=telemetry.controller)
        self.value_labels["pitch"].configure(text=f"{telemetry.pitch_deg:+.2f} \u00b0")
        self.value_labels["position"].configure(text=f"{telemetry.position_m:+.3f} m")
        self.value_labels["velocity"].configure(text=f"{telemetry.velocity_m_s:+.3f} m/s")
        self.value_labels["command"].configure(
            text=f"{telemetry.command_velocity_m_s:+.3f} m/s")

    # ------------------------------------------------------------ drawing
    def _draw_drive(self) -> None:
        canvas = self.drive_canvas
        canvas.delete("all")
        size, gap = 58, 6
        cx = 120
        keys = {
            "W": (cx - size / 2, 8, {"w", "up"}),
            "A": (cx - size * 1.5 - gap, 8 + size + gap, {"a", "left"}),
            "S": (cx - size / 2, 8 + size + gap, {"s", "down"}),
            "D": (cx + size / 2 + gap, 8 + size + gap, {"d", "right"}),
        }
        for label, (x, y, names) in keys.items():
            active = bool(self.pressed & names)
            canvas.create_rectangle(x, y, x + size, y + size, width=0,
                                    fill=ACCENT if active else "#343b45")
            canvas.create_text(x + size / 2, y + size / 2, text=label, fill=FG,
                               font=("Segoe UI Semibold", 16))
        slow = bool(self.pressed & {"shift_l", "shift_r"})
        canvas.create_text(cx, 150, text="SLOW MODE" if slow else "",
                           fill=ORANGE, font=("Segoe UI Semibold", 9))

        limits = self.commander.limits.clamped()
        self._draw_bar(canvas, 172, "speed", self.commander.speed,
                       limits.max_speed_m_s, "m/s")
        self._draw_bar(canvas, 222, "turn (+ = left)", self.commander.turn,
                       limits.max_turn_rad_s, "rad/s")
        if canvas.focus_get() is not canvas and self._typing():
            canvas.create_text(cx, 262, text="typing in console - press Esc to drive",
                               fill=ORANGE, font=("Segoe UI", 8))
        canvas.scale("all", 0, 0, self.ui_scale, self.ui_scale)

    def _draw_bar(self, canvas, y, label, value, maximum, unit) -> None:
        left, right = 14, 226
        mid = (left + right) / 2
        canvas.create_text(left, y, text=label, anchor="w", fill=MUTED,
                           font=("Segoe UI", 9))
        canvas.create_text(right, y, text=f"{value:+.2f} {unit}", anchor="e", fill=FG,
                           font=("Consolas", 9))
        canvas.create_rectangle(left, y + 10, right, y + 26, width=0, fill="#343b45")
        if maximum > 0:
            fraction = max(-1.0, min(1.0, value / maximum))
            end = mid + fraction * (right - left) / 2
            canvas.create_rectangle(min(mid, end), y + 10, max(mid, end), y + 26,
                                    width=0, fill=GREEN if value >= 0 else ORANGE)
        canvas.create_line(mid, y + 8, mid, y + 28, fill=FG)

    def _draw_chart(self, now: float) -> None:
        chart = self.chart
        chart.delete("all")
        width = max(chart.winfo_width(), 50)
        height = max(chart.winfo_height(), 50)
        pad_left, pad_right, pad_top, pad_bottom = (self._px(v) for v in (42, 46, 22, 20))
        plot_w = width - pad_left - pad_right
        plot_h = height - pad_top - pad_bottom
        chart.create_text(pad_left, pad_top / 2, text="pitch [\u00b0]", fill=ACCENT,
                          anchor="w", font=("Segoe UI", 9))
        chart.create_text(width - pad_right, pad_top / 2, text="velocity [m/s]", fill=GREEN,
                          anchor="e", font=("Segoe UI", 9))
        chart.create_rectangle(pad_left, pad_top, pad_left + plot_w, pad_top + plot_h,
                               outline="#3a414b")
        zero_y = pad_top + plot_h / 2
        chart.create_line(pad_left, zero_y, pad_left + plot_w, zero_y, fill="#3a414b",
                          dash=(3, 3))
        chart.create_text(pad_left + plot_w / 2, height - pad_bottom / 2,
                          text=f"last {CHART_SECONDS:.0f} s", fill=MUTED,
                          font=("Segoe UI", 8))
        history = list(self._history)
        pitch_range = max([5.0] + [abs(p) for _, p, _ in history])
        velocity_range = max([0.2] + [abs(v) for _, _, v in history])
        for value, y in ((pitch_range, pad_top), (-pitch_range, pad_top + plot_h)):
            chart.create_text(pad_left - 4, y, text=f"{value:+.0f}", fill=ACCENT,
                              anchor="e", font=("Consolas", 8))
        for value, y in ((velocity_range, pad_top), (-velocity_range, pad_top + plot_h)):
            chart.create_text(pad_left + plot_w + 4, y, text=f"{value:+.2f}", fill=GREEN,
                              anchor="w", font=("Consolas", 8))
        if len(history) < 2:
            chart.create_text(pad_left + plot_w / 2, zero_y - 14,
                              text="waiting for telemetry (T lines)", fill=MUTED,
                              font=("Segoe UI", 9))
            return

        def x_of(t):
            return pad_left + plot_w * (1.0 - (now - t) / CHART_SECONDS)

        for index, value_range, color in ((1, pitch_range, ACCENT),
                                          (2, velocity_range, GREEN)):
            points = []
            for sample in history:
                points += [x_of(sample[0]),
                           zero_y - sample[index] / value_range * plot_h / 2]
            chart.create_line(*points, fill=color, width=self._px(2))

    # ---------------------------------------------------------------- log
    def _log(self, text: str, tag: str | None = None) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n", tag or ())
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > LOG_MAX_LINES:
            self.log.delete("1.0", f"{lines - LOG_MAX_LINES}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _on_close(self) -> None:
        self.after_cancel(self._tick_id)
        self._save_settings()
        if self.link.connected:
            self._disconnect()
        self.destroy()


def enable_high_dpi() -> None:
    """Render sharp text on scaled Windows displays."""
    try:
        import ctypes

        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass


def main() -> None:
    enable_high_dpi()
    TeleopApp().mainloop()


if __name__ == "__main__":
    main()
