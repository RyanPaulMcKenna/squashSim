"""Static regression tests for the URDF-derived SOFA articulation graph.

These tests do not require a SOFA installation. Set SQUASHSIM_UR5_RG2_ROOT
to a checkout of AndrejOrsula/ur5_rg2_ign before running them.
"""

import importlib
import os
import sys
import types
import unittest

import numpy as np
from scipy.spatial.transform import Rotation


class FakeData:
    def __init__(self, node, name, value):
        self.node = node
        self.name = name
        self.value = value

    def getLinkPath(self):
        return f"@{self.node.path}.{self.name}"


class FakeObject:
    def __init__(self, node, component_type, **kwargs):
        self.node = node
        self.component_type = component_type
        self.name = kwargs.get("name", component_type)
        self.parameters = kwargs

    def getLinkPath(self):
        return f"@{self.node.path}/{self.name}"


class FakeNode:
    def __init__(self, name="root", parent=None):
        self.name = name
        self.parent = parent
        self.children = []
        self.objects = []
        self.data = {}

    @property
    def path(self):
        if self.parent is None:
            return f"/{self.name}"
        return f"{self.parent.path}/{self.name}"

    def addChild(self, name):
        child = FakeNode(name, self)
        self.children.append(child)
        setattr(self, name, child)
        return child

    def addData(self, name, value, *_args):
        data = FakeData(self, name, value)
        self.data[name] = data
        return data

    def getData(self, name):
        return self.data[name]

    def addObject(self, component, **kwargs):
        if isinstance(component, str):
            obj = FakeObject(self, component, **kwargs)
        else:
            obj = component
            if not hasattr(obj, "name"):
                obj.name = kwargs.get("name", type(obj).__name__)
        self.objects.append(obj)
        setattr(self, obj.name, obj)
        return obj

    def getObject(self, name):
        for obj in self.objects:
            if obj.name == name:
                return obj
        return None

    def walk(self):
        yield self
        for child in self.children:
            yield from child.walk()


def _install_fake_sofa():
    sofa = types.ModuleType("Sofa")
    core = types.ModuleType("Sofa.Core")
    simulation = types.ModuleType("Sofa.Simulation")

    class Controller:
        def __init__(self, *args, **kwargs):
            self.name = kwargs.get("name", type(self).__name__)

    core.Controller = Controller
    simulation.initTextures = lambda _root: None
    sofa.Core = core
    sofa.Simulation = simulation
    sys.modules["Sofa"] = sofa
    sys.modules["Sofa.Core"] = core
    sys.modules["Sofa.Simulation"] = simulation


def _matrix(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def _rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    return Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle).as_matrix()


def _urdf_geometry_to_sofa_world(robot, transform):
    """Convert a raw URDF-mesh-to-world transform into SOFA Y-up world."""
    basis = np.eye(4)
    basis[:3, :3] = robot.URDF_TO_SOFA
    return basis @ transform


class RobotModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if "SQUASHSIM_UR5_RG2_ROOT" not in os.environ:
            raise unittest.SkipTest("SQUASHSIM_UR5_RG2_ROOT is not set")
        _install_fake_sofa()
        for module_name in ("robot", "robotGUI"):
            sys.modules.pop(module_name, None)
        cls.robot = importlib.import_module("robot")
        cls.gui = importlib.import_module("robotGUI")

    def test_gui_command_is_mirrored(self):
        commands = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.75]
        self.assertEqual(
            self.gui.expand_gui_commands(commands),
            commands[:6] + [0.75, 0.75],
        )

    def test_part_rotations_round_trip_through_transform_engine_quaternions(self):
        for link_name in (*self.robot.RIGID_LINK_NAMES, "rg2_hand"):
            rotation, _translation = self.robot._part_transform(link_name)
            quaternion = Rotation.from_matrix(rotation).as_quat()
            reconstructed = Rotation.from_quat(quaternion).as_matrix()
            np.testing.assert_allclose(
                reconstructed, rotation, atol=2e-8, err_msg=link_name
            )

    def test_complete_scene_graph(self):
        root = FakeNode()
        robot_node = self.robot.Robot(root).addRobot()

        self.assertEqual(len(robot_node.getData("angles").value), 8)
        articulations = robot_node.Articulations
        self.assertEqual(len(articulations.dofs.parameters["position"]), 8)
        self.assertEqual(
            articulations.jointMass.parameters["vertexMass"],
            "1 1 1 1 1 1 1 1",
        )
        self.assertEqual(
            len(articulations.Rigid.dofs.parameters["position"]), 9
        )

        center_nodes = articulations.ArticulationCenters.children
        self.assertEqual(len(center_nodes), 8)
        indices = [
            center.children[0].joint.parameters["articulationIndex"]
            for center in center_nodes
        ]
        self.assertEqual(indices, list(range(8)))

        component_types = [
            obj.component_type
            for node in root.walk()
            for obj in node.objects
            if isinstance(obj, FakeObject)
        ]
        self.assertEqual(component_types.count("TriangleCollisionModel"), 10)
        self.assertNotIn("LineCollisionModel", component_types)
        self.assertNotIn("PointCollisionModel", component_types)

        base_state_types = {"MechanicalObject", "OglModel"}
        for node in root.walk():
            states = [
                obj.component_type
                for obj in node.objects
                if isinstance(obj, FakeObject)
                and obj.component_type in base_state_types
            ]
            self.assertLessEqual(
                len(states), 1, f"multiple BaseStates in {node.path}: {states}"
            )

    def test_sofa_articulation_matches_urdf_forward_kinematics(self):
        rng = np.random.default_rng(20260913)
        limits = self.robot.joint_limits()
        for _ in range(20):
            q = np.array([rng.uniform(low, high) for low, high in limits])
            sofa_frames = self._sofa_frames(q)
            urdf_frames = self._urdf_frames(q)

            for link_name, rigid_index in self.robot.RIGID_INDEX.items():
                local_rotation, local_translation = self.robot._part_transform(
                    link_name
                )
                sofa_geometry = sofa_frames[rigid_index] @ _matrix(
                    local_rotation, local_translation
                )
                expected = _urdf_geometry_to_sofa_world(
                    self.robot, urdf_frames[link_name]
                )
                np.testing.assert_allclose(
                    sofa_geometry,
                    expected,
                    atol=2e-8,
                    err_msg=link_name,
                )

            hand_rotation, hand_translation = self.robot._part_transform(
                "rg2_hand"
            )
            sofa_hand = sofa_frames[
                self.robot.RIGID_INDEX["wrist_3_link"]
            ] @ _matrix(hand_rotation, hand_translation)
            np.testing.assert_allclose(
                sofa_hand,
                _urdf_geometry_to_sofa_world(
                    self.robot, urdf_frames["rg2_hand"]
                ),
                atol=2e-8,
            )

    def _sofa_frames(self, q):
        poses = self.robot.initial_rigid_poses()
        frames = []
        for pose in poses:
            frames.append(
                _matrix(
                    Rotation.from_quat(pose[3:]).as_matrix(),
                    np.asarray(pose[:3]),
                )
            )

        for definition in self.robot.articulation_definitions():
            parent = frames[definition["parent_index"]]
            child_rotation = parent[:3, :3] @ _rotation(
                definition["axis"], q[definition["articulation_index"]]
            )
            child_translation = parent[:3, 3] + parent[:3, :3] @ np.asarray(
                definition["position_on_parent"]
            )
            frames[definition["child_index"]] = _matrix(
                child_rotation, child_translation
            )
        return frames

    def _urdf_frames(self, q):
        frames = {"base_link": np.eye(4)}
        for index, name in enumerate(self.robot.ARM_JOINT_NAMES):
            joint = self.robot.JOINTS[name]
            frames[joint.child] = (
                frames[joint.parent]
                @ self.robot._origin_matrix(joint.origin)
                @ _matrix(_rotation(joint.axis, q[index]), np.zeros(3))
            )

        hand_joint = self.robot.HAND_MOUNT
        frames["rg2_hand"] = frames[hand_joint.parent] @ self.robot._origin_matrix(
            hand_joint.origin
        )
        for offset, name in enumerate(self.robot.FINGER_JOINT_NAMES):
            joint = self.robot.JOINTS[name]
            frames[joint.child] = (
                frames[joint.parent]
                @ self.robot._origin_matrix(joint.origin)
                @ _matrix(_rotation(joint.axis, q[6 + offset]), np.zeros(3))
            )

        geometry_frames = {}
        for link_name in self.robot.RIGID_LINK_NAMES:
            collision_origin = self.robot._origin(
                self.robot.LINKS[link_name].find("collision")
            )
            geometry_frames[link_name] = frames[
                link_name
            ] @ self.robot._origin_matrix(collision_origin)
        collision_origin = self.robot._origin(
            self.robot.LINKS["rg2_hand"].find("collision")
        )
        geometry_frames["rg2_hand"] = frames[
            "rg2_hand"
        ] @ self.robot._origin_matrix(collision_origin)
        return geometry_frames


if __name__ == "__main__":
    unittest.main()
