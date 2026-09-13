"""Controller lifecycle tests without launching SOFA, Tk or SDL."""

import importlib
from pathlib import Path
import sys
import threading
import types
import unittest

import numpy as np


def _load_robot_gui_with_fake_sofa():
    sofa = types.ModuleType("Sofa")
    core = types.ModuleType("Sofa.Core")

    class Controller:
        def __init__(self, *args, **kwargs):
            self.name = kwargs.get("name", type(self).__name__)

    core.Controller = Controller
    sofa.Core = core
    sys.modules["Sofa"] = sofa
    sys.modules["Sofa.Core"] = core
    sys.modules.pop("robotGUI", None)
    return importlib.import_module("robotGUI")


class _Data:
    def __init__(self, value):
        self.value = value


class _MechanicalObject:
    def __init__(self, **values):
        self.values = {name: _Data(value) for name, value in values.items()}

    def getData(self, name):
        return self.values[name]


class _Robot:
    def __init__(self):
        self.angles = _Data([0.0] * 8)

    def getData(self, name):
        self.assert_name = name
        return self.angles


class _Root:
    def __init__(self):
        self.time = 0.0

    def getTime(self):
        return self.time

    def getDt(self):
        return 0.01


class _Backend:
    def __init__(self, sample_type):
        self.samples = iter(
            [
                sample_type(connected=True, status="Connected", a_button=True),
                sample_type(connected=True, status="Connected"),
                sample_type(connected=True, status="Connected", a_button=True),
            ]
        )

    def poll(self):
        return next(self.samples)


class _App:
    def __init__(self, initAngles, armLimits, gripperLimit):
        del armLimits, gripperLimit
        self.commands = list(initAngles[:6]) + [float(initAngles[6])]
        self.ready = threading.Event()
        self.ready.set()
        self.closed = threading.Event()
        self.status = ""

    def get_commands(self):
        return self.commands.copy()

    def adjust_command(self, index, delta):
        self.commands[index] += float(delta)

    def set_controller_status(self, text, selected_index, connected):
        del selected_index, connected
        self.status = text


class _Recorder:
    def __init__(self):
        self.is_recording = False
        self.last_export_directory = None
        self.frames = []
        self.starts = []

    @property
    def recorded_duration_s(self):
        return 0.1

    def start(self, sim_time):
        self.starts.append(sim_time)
        self.is_recording = True
        return Path("recordings/episode_test")

    def record(self, frame):
        self.frames.append(frame)

    def stop_and_export(self):
        self.is_recording = False
        self.last_export_directory = Path("recordings/episode_test")
        return self.last_export_directory


class _ContactListener:
    def __init__(self, count):
        self.count = count

    def getNumberOfContacts(self):
        return self.count


class RobotGUIRecordingTests(unittest.TestCase):
    def test_a_button_records_completed_steps_and_exports_on_second_press(self):
        robot_gui = _load_robot_gui_with_fake_sofa()
        original_app = robot_gui.App
        robot_gui.App = _App
        try:
            root = _Root()
            robot = _Robot()
            articulation = _MechanicalObject(
                position=np.zeros((8, 1)), velocity=np.ones((8, 1))
            )
            rigid_positions = np.zeros((9, 7))
            rigid_positions[:, 6] = 1.0
            rigid_positions[7, :3] = [0.2, 0.4, 0.1]
            rigid_positions[8, :3] = [0.4, 0.4, 0.1]
            rigid = _MechanicalObject(position=rigid_positions)
            object_positions = np.zeros((31, 7))
            object_positions[:, 6] = 1.0
            object_dofs = _MechanicalObject(position=object_positions)
            recorder = _Recorder()
            controller = robot_gui.RobotGUI(
                robot=robot,
                articulations_mo=articulation,
                rigid_mo=rigid,
                object_mo=object_dofs,
                root_node=root,
                episodeRecorder=recorder,
                gamepadBackend=_Backend(robot_gui.GamepadSample),
                contactListeners={
                    "gripper": (_ContactListener(2), _ContactListener(1)),
                    "floor": (_ContactListener(4),),
                },
                initAngles=[0.0] * 6 + [1.18, 1.18],
                armLimits=[(-6.28, 6.28)] * 6,
                gripperLimit=(0.0, 1.18),
                clock=iter([0.0, 0.01, 0.02]).__next__,
            )

            for step in range(3):
                root.time = 0.01 * step
                controller.onAnimateBeginEvent({})
                root.time += 0.01
                controller.onAnimateEndEvent({})

            self.assertEqual(recorder.starts, [0.0])
            self.assertEqual(len(recorder.frames), 3)
            self.assertFalse(recorder.is_recording)
            self.assertEqual(
                recorder.frames[-1].gripper_object_contact_count, 3
            )
            np.testing.assert_allclose(
                recorder.frames[-1].ee_position_m, [0.3, 0.4, 0.1]
            )
            self.assertEqual(robot.angles.value, [0.0] * 6 + [1.18, 1.18])
        finally:
            robot_gui.App = original_app


if __name__ == "__main__":
    unittest.main()
