"""Static tests for the BeamFEM cable and launch-time object selector."""

import unittest

import numpy as np

import demo_object
import flexible_cable
from test_deformable_ball import FakeNode


def _rotate_vector(quaternion, vector):
    xyz = quaternion[:3]
    scalar = quaternion[3]
    vector = np.asarray(vector, dtype=float)
    return (
        2.0 * np.dot(xyz, vector) * xyz
        + (scalar * scalar - np.dot(xyz, xyz)) * vector
        + 2.0 * scalar * np.cross(xyz, vector)
    )


class FlexibleCableTests(unittest.TestCase):
    def test_generated_cable_has_exact_length_and_aligned_frames(self):
        geometry = flexible_cable.generate_cable_geometry()
        self.assertEqual(geometry.frames.shape, (31, 7))
        self.assertEqual(geometry.edges.shape, (30, 2))
        np.testing.assert_array_equal(
            geometry.edges,
            np.column_stack((np.arange(30), np.arange(1, 31))),
        )

        centers = geometry.frames[:, :3]
        segment_vectors = np.diff(centers, axis=0)
        segment_lengths = np.linalg.norm(segment_vectors, axis=1)
        np.testing.assert_allclose(
            segment_lengths,
            flexible_cable.CABLE_LENGTH / 30,
            atol=1e-12,
        )
        self.assertAlmostEqual(
            np.linalg.norm(centers[-1] - centers[0]),
            flexible_cable.CABLE_LENGTH,
        )
        self.assertGreater(centers[-1, 1], centers[0, 1])

        quaternions = geometry.frames[:, 3:]
        np.testing.assert_allclose(
            np.linalg.norm(quaternions, axis=1), 1.0, atol=1e-12
        )
        for quaternion in quaternions:
            np.testing.assert_allclose(
                _rotate_vector(quaternion, [1.0, 0.0, 0.0]),
                geometry.direction,
                atol=1e-12,
            )

    def test_scene_graph_has_beam_spheres_and_mapped_tube(self):
        root = FakeNode()
        cable = flexible_cable.add_flexible_cable(root)
        component_types = [
            obj.component_type
            for node in root.walk()
            for obj in node.objects
        ]

        self.assertEqual(component_types.count("BeamFEMForceField"), 1)
        self.assertEqual(component_types.count("BTDLinearSolver"), 1)
        self.assertEqual(component_types.count("SphereCollisionModel"), 1)
        self.assertEqual(component_types.count("TriangleCollisionModel"), 0)
        self.assertEqual(component_types.count("LineCollisionModel"), 0)
        self.assertEqual(component_types.count("PointCollisionModel"), 0)
        self.assertEqual(component_types.count("TubularMapping"), 1)
        self.assertEqual(
            component_types.count("Edge2QuadTopologicalMapping"), 1
        )
        self.assertEqual(component_types.count("IdentityMapping"), 1)

        self.assertEqual(cable.dofs.parameters["template"], "Rigid3d")
        self.assertEqual(len(cable.dofs.parameters["position"]), 31)
        self.assertEqual(len(cable.topology.parameters["lines"]), 30)
        self.assertEqual(cable.mass.parameters["totalMass"], 0.15)
        self.assertEqual(
            cable.elasticity.parameters["youngModulus"],
            flexible_cable.CABLE_YOUNG_MODULUS,
        )
        self.assertEqual(cable.collisionModel.parameters["group"], [3])
        self.assertEqual(
            cable.collisionModel.parameters["radius"],
            flexible_cable.CABLE_RADIUS,
        )
        self.assertTrue(cable.collisionModel.parameters["moving"])
        self.assertTrue(cable.collisionModel.parameters["simulated"])
        self.assertFalse(cable.collisionModel.parameters["selfCollision"])

        base_state_types = {"MechanicalObject", "OglModel"}
        for node in root.walk():
            states = [
                obj.component_type
                for obj in node.objects
                if obj.component_type in base_state_types
            ]
            self.assertLessEqual(
                len(states), 1, f"multiple BaseStates in {node.path}: {states}"
            )

    def test_selector_defaults_to_cable_and_preserves_ball(self):
        self.assertEqual(demo_object.selected_object_name({}), "cable")
        self.assertEqual(
            demo_object.selected_object_name({"SQUASHSIM_OBJECT": " BALL "}),
            "ball",
        )
        self.assertEqual(
            demo_object.selected_object_name({"SQUASHSIM_OBJECT": "none"}),
            "none",
        )
        with self.assertRaisesRegex(ValueError, "choose one of"):
            demo_object.selected_object_name({"SQUASHSIM_OBJECT": "rope"})

        cable_root = FakeNode()
        cable = demo_object.add_demo_object(cable_root, "cable")
        self.assertEqual(cable.name, "FlexibleCable")

        ball_root = FakeNode()
        ball = demo_object.add_demo_object(ball_root, "ball")
        self.assertEqual(ball.name, "DeformableBall")

        empty_root = FakeNode()
        self.assertIsNone(demo_object.add_demo_object(empty_root, "none"))
        self.assertEqual(empty_root.children, [])


if __name__ == "__main__":
    unittest.main()
