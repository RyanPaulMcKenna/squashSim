"""Mouse/Xbox control and synchronous recording for the UR5 + RG2."""

from pathlib import Path
import threading
import time

import numpy as np
import Sofa.Core
import tkinter as tkinter

from camera_controller import orbit_camera
from episode_recorder import EpisodeRecorder, FrameSample
from xbox_controller import (
    CONTROL_LABELS,
    GamepadSample,
    PygameXboxBackend,
    SPEED_PRESETS,
    XboxControlState,
)


ARM_LABELS = CONTROL_LABELS[:6]
SELECTED_LABEL_COLOR = "#9ecbff"
MAX_WALL_CONTROL_DT = 0.05


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
        self.rigid_dofs = kwargs.get("rigid_mo")
        self.object_dofs = kwargs.get("object_mo")
        self.root_node = kwargs.get("root_node")
        self.camera = kwargs.get("camera")
        self.ee_rigid_indices = tuple(kwargs.get("eeRigidIndices", (7, 8)))
        self.contact_listeners = kwargs.get(
            "contactListeners", {"gripper": (), "floor": ()}
        )
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
        self._clock = kwargs.get("clock") or time.monotonic
        self._last_control_wall_time = None
        self._last_gamepad_status = None
        self._last_sample = GamepadSample()
        self._last_command_delta = 0.0
        self._last_commands = expand_gui_commands(self.app.get_commands())
        self._record_stop_requested = False
        self._recording_error_reported = False
        self._camera_error_reported = False

        self.recorder = kwargs.get("episodeRecorder")
        if self.recorder is None and self.root_node is not None:
            self.recorder = EpisodeRecorder(
                project_root=kwargs.get(
                    "projectRoot", Path(__file__).resolve().parent
                ),
                joint_names=kwargs.get(
                    "jointNames",
                    (
                        "J1",
                        "J2",
                        "J3",
                        "J4",
                        "J5",
                        "J6",
                        "RG2-left",
                        "RG2-right",
                    ),
                ),
                control_labels=CONTROL_LABELS,
                object_label=kwargs.get("objectLabel", "deformable object"),
                representative_object_index=kwargs.get(
                    "representativeObjectIndex"
                ),
                configuration=kwargs.get("recorderConfiguration", {}),
            )

    def _wall_control_dt(self):
        """Use elapsed real time so control speed is independent of frame rate."""
        now = float(self._clock())
        if self._last_control_wall_time is None:
            elapsed = self.control_dt
        else:
            elapsed = now - self._last_control_wall_time
        self._last_control_wall_time = now
        if not np.isfinite(elapsed) or elapsed <= 0.0:
            return self.control_dt
        return min(elapsed, MAX_WALL_CONTROL_DT)

    def _recording_status(self):
        if self.recorder is None:
            return "recording unavailable"
        if self.recorder.is_recording:
            if self._record_stop_requested:
                return "REC saving..."
            return f"REC {self.recorder.recorded_duration_s:.1f}s | A: stop"
        if self.recorder.last_export_directory is not None:
            return (
                "A: record | saved "
                f"{self.recorder.last_export_directory.name}"
            )
        return "A: record"

    @staticmethod
    def _mechanical_values(mechanical_object, data_name):
        return np.asarray(
            mechanical_object.getData(data_name).value, dtype=float
        ).copy()

    @staticmethod
    def _contact_count(listeners):
        return sum(int(listener.getNumberOfContacts()) for listener in listeners)

    def _capture_frame(self):
        joint_position = self._mechanical_values(
            self.articulations, "position"
        ).reshape(-1)
        joint_velocity = self._mechanical_values(
            self.articulations, "velocity"
        ).reshape(-1)

        rigid_positions = self._mechanical_values(
            self.rigid_dofs, "position"
        )
        if rigid_positions.ndim == 1:
            rigid_positions = rigid_positions.reshape(-1, 7)
        ee_position = rigid_positions[
            list(self.ee_rigid_indices), :3
        ].mean(axis=0)

        if self.object_dofs is None:
            object_positions = np.empty((0, 3), dtype=float)
        else:
            object_state = self._mechanical_values(
                self.object_dofs, "position"
            )
            if object_state.ndim == 1:
                coordinate_count = 7 if object_state.size % 7 == 0 else 3
                object_state = object_state.reshape(-1, coordinate_count)
            object_positions = object_state[:, :3]

        speed_index = self.gamepad_state.speed_index
        speed = SPEED_PRESETS[speed_index][1]
        sample = self._last_sample
        return FrameSample(
            sim_time_s=float(self.root_node.getTime()),
            simulation_dt_s=float(self.root_node.getDt()),
            joint_position_rad=joint_position,
            joint_velocity_rad_s=joint_velocity,
            commanded_joint_position_rad=np.asarray(
                self._last_commands, dtype=float
            ),
            object_node_position_m=object_positions,
            ee_position_m=ee_position,
            selected_control_index=self.gamepad_state.selected_index,
            speed_index=speed_index,
            command_speed_rad_s=speed,
            command_delta_rad=self._last_command_delta,
            left_x=sample.left_x,
            right_x=sample.right_x,
            right_y=sample.right_y,
            left_trigger=sample.left_trigger,
            right_trigger=sample.right_trigger,
            left_bumper=sample.left_bumper,
            right_bumper=sample.right_bumper,
            a_button=sample.a_button,
            gripper_object_contact_count=self._contact_count(
                self.contact_listeners.get("gripper", ())
            ),
            floor_object_contact_count=self._contact_count(
                self.contact_listeners.get("floor", ())
            ),
        )

    def _finish_recording(self):
        if self.recorder is None or not self.recorder.is_recording:
            self._record_stop_requested = False
            return
        try:
            output_directory = self.recorder.stop_and_export()
            print(f"[squashSim] recording SAVED: {output_directory}")
            print(
                "[squashSim] evidence: episode.npz, samples.csv, "
                "metadata/tables and SVG plots"
            )
        except Exception as error:
            print(f"[squashSim] recording export FAILED: {error}")
        finally:
            self._record_stop_requested = False

    def reset(self):
        if self.app.ready.is_set() and not self.app.closed.is_set():
            self.app.root.after(0, self.app.reset)

    def onAnimateBeginEvent(self, _event):
        if not self.app.ready.is_set() or self.app.closed.is_set():
            return

        sample = self.gamepad.poll()
        wall_dt = self._wall_control_dt()
        update = self.gamepad_state.step(sample, wall_dt)
        self._last_sample = sample
        self._last_command_delta = update.command_delta
        if update.command_delta != 0.0:
            self.app.adjust_command(
                update.selected_index, update.command_delta
            )

        if self.camera is not None and sample.connected:
            try:
                orbit_camera(
                    self.camera, sample.right_x, sample.right_y, wall_dt
                )
            except Exception as error:
                if not self._camera_error_reported:
                    print(f"[squashSim] right-stick camera disabled: {error}")
                    self._camera_error_reported = True

        if update.recording_toggle and self.recorder is not None:
            if self.recorder.is_recording:
                self._record_stop_requested = True
            else:
                output_directory = self.recorder.start(
                    float(self.root_node.getTime())
                )
                self._record_stop_requested = False
                self._recording_error_reported = False
                print(f"[squashSim] recording START: {output_directory}")

        speed_name, speed = SPEED_PRESETS[update.speed_index]
        if sample.connected:
            status = (
                f"Xbox: {CONTROL_LABELS[update.selected_index]} | "
                f"speed: {speed_name} ({speed:.2f} rad/s) | "
                f"{self._recording_status()}"
            )
        else:
            status = f"Xbox: {sample.status} | {self._recording_status()}"
        self.app.set_controller_status(
            status, update.selected_index, sample.connected
        )

        if sample.status != self._last_gamepad_status:
            print(f"[squashSim] Xbox input: {sample.status}")
            self._last_gamepad_status = sample.status

        commands = self.app.get_commands()
        # The ArticulatedSystemPlugin requires one input DOF per finger centre.
        # Both receive the same GUI value, producing the RG2's mirrored motion.
        self._last_commands = expand_gui_commands(commands)
        self.robot.getData("angles").value = self._last_commands

    def onAnimateEndEvent(self, _event):
        if self.recorder is None or not self.recorder.is_recording:
            return
        try:
            self.recorder.record(self._capture_frame())
        except Exception as error:
            if not self._recording_error_reported:
                print(f"[squashSim] recording sample FAILED: {error}")
                self._recording_error_reported = True
        if self._record_stop_requested:
            self._finish_recording()

    def cleanup(self):
        """Preserve an active recording if the SOFA scene is closed."""
        self._finish_recording()


def createScene(rootNode):
    from header import addHeader

    addHeader(rootNode)
    return rootNode
