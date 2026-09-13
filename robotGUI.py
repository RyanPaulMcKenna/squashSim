"""Seven-slider controller for the eight-DOF UR5 + mirrored RG2 model."""

import threading

import Sofa.Core
import tkinter as tkinter


ARM_LABELS = ("J1", "J2", "J3", "J4", "J5", "J6")


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
        self.command_values = self.initial_commands.copy()
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

    def reset(self):
        for variable, value in zip(self.variables, self.initial_commands):
            variable.set(value)
        with self.command_lock:
            self.command_values[:] = self.initial_commands

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

        labels = ARM_LABELS + ("RG2",)
        limits = self.arm_limits + [self.gripper_limit]
        self.variables = []
        for column, (label, (lower, upper), initial) in enumerate(
            zip(labels, limits, self.initial_commands)
        ):
            tkinter.Label(self.root, text=label).grid(row=1, column=column)
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

        tkinter.Button(self.root, text="Reset", command=self.reset).grid(
            row=3, column=0, columnspan=7, sticky="ew", padx=6, pady=6
        )
        self.ready.set()
        self.root.mainloop()


class RobotGUI(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.robot = kwargs["robot"]
        self.articulations = kwargs["articulations_mo"]
        self.app = App(
            kwargs.get("initAngles", [0.0] * 6 + [1.18, 1.18]),
            kwargs.get("armLimits", [(-6.28319, 6.28319)] * 6),
            kwargs.get("gripperLimit", (0.0, 1.18)),
        )

    def reset(self):
        if self.app.ready.is_set() and not self.app.closed.is_set():
            self.app.root.after(0, self.app.reset)

    def onAnimateBeginEvent(self, _event):
        if not self.app.ready.is_set() or self.app.closed.is_set():
            return
        commands = self.app.get_commands()
        # The ArticulatedSystemPlugin requires one input DOF per finger centre.
        # Both receive the same GUI value, producing the RG2's mirrored motion.
        self.robot.getData("angles").value = expand_gui_commands(commands)


def createScene(rootNode):
    from header import addHeader

    addHeader(rootNode)
    return rootNode
