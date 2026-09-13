"""Mouse-slider and optional Xbox control for the UR5 + mirrored RG2."""

import threading

import Sofa.Core
import tkinter as tkinter

from xbox_controller import (
    CONTROL_LABELS,
    PygameXboxBackend,
    SPEED_PRESETS,
    XboxControlState,
)


ARM_LABELS = CONTROL_LABELS[:6]
SELECTED_LABEL_COLOR = "#9ecbff"


def expand_gui_commands(commands):
    """Expand six arm commands and one grip command to eight SOFA DOFs."""
    values = [float(value) for value in commands]
    if len(values) != 7:
        raise ValueError("expected six UR5 commands and one RG2 command")
    return values[:6] + [values[6], values[6]]


class App(threading.Thread):
    def __init__(self, initAngles, armLimits, gripperLimit):
        threading.Thread.__init__(self, daemon=True)
        values = [float(value) for value in initAngles]
        if len(values) == 7:
            self.initial_commands = values
        elif len(values) == 8:
            self.initial_commands = values[:6] + [
                0.5 * (values[6] + values[7])
            ]
        else:
            raise ValueError("initAngles must contain 7 GUI values or 8 SOFA DOFs")
        self.arm_limits = [(float(low), float(high)) for low, high in armLimits]
        self.gripper_limit = tuple(float(value) for value in gripperLimit)
        self.command_limits = self.arm_limits + [self.gripper_limit]
        self.command_values = self.initial_commands.copy()
        self.controller_status = (
            "Xbox: waiting for Animate; mouse sliders are active"
        )
        self.controller_connected = False
        self.selected_control = 0
        self.command_lock = threading.Lock()
        self.ready = threading.Event()
        self.closed = threading.Event()
        self.start()

    def get_commands(self):
        with self.command_lock:
            return self.command_values.copy()

    def _set_command(self, index, value):
        with self.command_lock:
            self.command_values[index] = float(value)

    def adjust_command(self, index, delta):
        """Increment one command and clamp it to the existing slider limits."""
        with self.command_lock:
            lower, upper = self.command_limits[index]
            value = self.command_values[index] + float(delta)
            value = max(lower, min(upper, value))
            self.command_values[index] = value
            return value

    def set_controller_status(self, text, selected_index, connected):
        """Publish plain data for the Tk thread to render on its next tick."""
        with self.command_lock:
            self.controller_status = str(text)
            self.selected_control = int(selected_index)
            self.controller_connected = bool(connected)

    def _refresh_ui(self):
        """Synchronize gamepad-written values without touching Tk cross-thread."""
        if self.closed.is_set():
            return
        with self.command_lock:
            commands = self.command_values.copy()
            status = self.controller_status
            selected = self.selected_control
            connected = self.controller_connected

        for variable, value in zip(self.variables, commands):
            if abs(variable.get() - value) > 1.0e-9:
                variable.set(value)
        if self.status_variable.get() != status:
            self.status_variable.set(status)
        for index, label in enumerate(self.joint_labels):
            color = (
                SELECTED_LABEL_COLOR
                if connected and index == selected
                else self.default_label_color
            )
            if label.cget("background") != color:
                label.configure(background=color)
        self.root.after(50, self._refresh_ui)

    def reset(self):
        with self.command_lock:
            self.command_values[:] = self.initial_commands
        for variable, value in zip(self.variables, self.initial_commands):
            variable.set(value)

    def close(self):
        self.closed.set()
        self.root.destroy()

    def run(self):
        self.root = tkinter.Tk()
        self.root.title("squashSim UR5 + OnRobot RG2")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        tkinter.Label(
            self.root,
            text="UR5 + OnRobot RG2 joint targets (radians)",
        ).grid(row=0, column=0, columnspan=7, pady=(6, 2))

        labels = CONTROL_LABELS
        limits = self.command_limits
        self.variables = []
        self.joint_labels = []
        for column, (label, (lower, upper), initial) in enumerate(
            zip(labels, limits, self.initial_commands)
        ):
            label_widget = tkinter.Label(self.root, text=label)
            label_widget.grid(row=1, column=column)
            self.joint_labels.append(label_widget)
            variable = tkinter.DoubleVar(value=initial)
            self.variables.append(variable)
            tkinter.Scale(
                self.root,
                variable=variable,
                resolution=0.001,
                length=430,
                from_=upper,
                to=lower,
                orient=tkinter.VERTICAL,
                command=lambda value, index=column: self._set_command(
                    index, value
                ),
            ).grid(row=2, column=column, padx=3)

        self.default_label_color = self.joint_labels[0].cget("background")
        self.status_variable = tkinter.StringVar(value=self.controller_status)
        tkinter.Label(
            self.root,
            textvariable=self.status_variable,
            anchor="w",
        ).grid(row=3, column=0, columnspan=7, sticky="ew", padx=6, pady=(5, 0))
        tkinter.Button(self.root, text="Reset", command=self.reset).grid(
            row=4, column=0, columnspan=7, sticky="ew", padx=6, pady=6
        )
        self.ready.set()
        self.root.after(50, self._refresh_ui)
        self.root.mainloop()


class RobotGUI(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        self.articulations = kwargs["articulations_mo"]
        self.control_dt = float(kwargs.get("controlDt", 0.005))
        self.app = App(
            kwargs.get("initAngles", [0.0] * 6 + [1.18, 1.18]),
            kwargs.get("armLimits", [(-6.28319, 6.28319)] * 6),
            kwargs.get("gripperLimit", (0.0, 1.18)),
        )
        self.gamepad = kwargs.get("gamepadBackend")
        if self.gamepad is None:
            self.gamepad = PygameXboxBackend()
        self.gamepad_state = XboxControlState(len(CONTROL_LABELS))
        self._last_gamepad_status = None

    def reset(self):
        if self.app.ready.is_set() and not self.app.closed.is_set():
            self.app.root.after(0, self.app.reset)

    def onAnimateBeginEvent(self, _event):
        if not self.app.ready.is_set() or self.app.closed.is_set():
            return

        sample = self.gamepad.poll()
        update = self.gamepad_state.step(sample, self.control_dt)
        if update.command_delta != 0.0:
            self.app.adjust_command(
                update.selected_index, update.command_delta
            )

        speed_name, speed = SPEED_PRESETS[update.speed_index]
        if sample.connected:
            status = (
                f"Xbox: {CONTROL_LABELS[update.selected_index]} | "
                f"speed: {speed_name} ({speed:.2f} rad/s)"
            )
        else:
            status = f"Xbox: {sample.status}"
        self.app.set_controller_status(
            status, update.selected_index, sample.connected
        )

        if sample.status != self._last_gamepad_status:
            print(f"[squashSim] Xbox input: {sample.status}")
            self._last_gamepad_status = sample.status

        commands = self.app.get_commands()
        # The ArticulatedSystemPlugin requires one input DOF per finger centre.
        # Both receive the same GUI value, producing the RG2's mirrored motion.
        self.robot.getData("angles").value = expand_gui_commands(commands)


def createScene(rootNode):
    from header import addHeader

    addHeader(rootNode)
    return rootNode
