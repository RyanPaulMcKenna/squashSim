"""Static tests for the procedural deformable ball; SOFA is not required."""

from collections import Counter
from itertools import combinations
import unittest

import numpy as np

import deformable_ball


class FakeObject:
    def __init__(self, node, component_type, **parameters):
        self.node = node
        self.component_type = component_type
        self.name = parameters.get("name", component_type)
        self.parameters = parameters

    def getLinkPath(self):
        return f"@{self.node.path}/{self.name}"


class FakeNode:
    def __init__(self, name="root", parent=None):
        self.name = name
        self.parent = parent
        self.children = []
        self.objects = []

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

    def addObject(self, component_type, **parameters):
        obj = FakeObject(self, component_type, **parameters)
        self.objects.append(obj)
        setattr(self, obj.name, obj)
        return obj

    def walk(self):
        yield self
        for child in self.children:
            yield from child.walk()


class DeformableBallTests(unittest.TestCase):
    def test_generated_mesh_is_closed_positive_and_football_sized(self):
        mesh = deformable_ball.generate_ball_mesh()
        self.assertEqual(mesh.surface_vertex_count, 162)
        self.assertEqual(mesh.positions.shape, (163, 3))
        self.assertEqual(mesh.surface_triangles.shape, (320, 3))
        self.assertEqual(mesh.tetrahedra.shape, (320, 4))

        center = np.asarray(deformable_ball.BALL_START_CENTER)
        radii = np.linalg.norm(
            mesh.positions[: mesh.surface_vertex_count] - center,
            axis=1,
        )
        np.testing.assert_allclose(radii, deformable_ball.BALL_RADIUS, atol=1e-12)
        np.testing.assert_allclose(mesh.positions[-1], center, atol=1e-12)

        six_volumes = []
        face_counts = Counter()
        for tetrahedron in mesh.tetrahedra:
            points = mesh.positions[tetrahedron]
            six_volumes.append(
                np.dot(
                    points[1] - points[0],
                    np.cross(points[2] - points[0], points[3] - points[0]),
                )
            )
            face_counts.update(
                tuple(sorted(face)) for face in combinations(tetrahedron, 3)
            )
        self.assertGreater(min(six_volumes), 1e-12)
        boundary = {face for face, count in face_counts.items() if count == 1}
        expected = {
            tuple(sorted(face)) for face in mesh.surface_triangles
        }
        self.assertEqual(boundary, expected)

    def test_scene_graph_has_one_real_collision_surface(self):
        root = FakeNode()
        ball = deformable_ball.add_deformable_ball(root)

        component_types = [
            obj.component_type
            for node in root.walk()
            for obj in node.objects
        ]
        self.assertEqual(component_types.count("TetrahedronFEMForceField"), 1)
        self.assertEqual(component_types.count("MeshMatrixMass"), 1)
        self.assertEqual(component_types.count("TriangleCollisionModel"), 1)
        self.assertEqual(
            component_types.count("Tetra2TriangleTopologicalMapping"), 1
        )
        self.assertEqual(component_types.count("IdentityMapping"), 2)
        self.assertNotIn("SphereCollisionModel", component_types)
        self.assertNotIn("LineCollisionModel", component_types)
        self.assertNotIn("PointCollisionModel", component_types)

        self.assertEqual(
            len(ball.topology.parameters["position"]),
            163,
        )
        self.assertEqual(
            len(ball.topology.parameters["tetrahedra"]),
            320,
        )
        self.assertEqual(ball.mass.parameters["totalMass"], 0.12)
        self.assertTrue(ball.mass.parameters["lumping"])
        self.assertEqual(
            ball.elasticity.parameters["youngModulus"],
            deformable_ball.BALL_YOUNG_MODULUS,
        )
        self.assertEqual(
            ball.Surface.collisionModel.parameters["group"],
            [3],
        )
        self.assertTrue(ball.Surface.collisionModel.parameters["moving"])
        self.assertTrue(ball.Surface.collisionModel.parameters["simulated"])
        self.assertFalse(ball.Surface.collisionModel.parameters["selfCollision"])

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


if __name__ == "__main__":
    unittest.main()
