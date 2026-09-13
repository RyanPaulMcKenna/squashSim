"""Small, self-contained deformable ball for the squashSim scene.

The ball is a genuine volumetric elastic body.  Its tetrahedral simulation
mesh is generated deterministically in Python, while SOFA extracts and maps
the deforming boundary surface for both collision and rendering.
"""

from collections import Counter
from dataclasses import dataclass
from itertools import combinations
import math

import numpy as np


BALL_RADIUS = 0.11
BALL_MASS = 0.12
BALL_START_CENTER = (0.55, 0.65, 0.15)
BALL_YOUNG_MODULUS = 10_000.0
BALL_POISSON_RATIO = 0.35
BALL_COLOR = [0.90, 0.16, 0.035, 1.0]
BALL_COLLISION_GROUP = [3]
BALL_PROXIMITY = 0.001
ICOSPHERE_SUBDIVISIONS = 2


@dataclass(frozen=True)
class BallMesh:
    """Tetrahedral volume and its known outer boundary."""

    positions: np.ndarray
    tetrahedra: np.ndarray
    surface_triangles: np.ndarray
    surface_vertex_count: int


def _unit_icosahedron():
    """Return a consistently oriented unit icosahedron."""
    phi = 0.5 * (1.0 + math.sqrt(5.0))
    vertices = np.asarray(
        [
            [-1.0, phi, 0.0],
            [1.0, phi, 0.0],
            [-1.0, -phi, 0.0],
            [1.0, -phi, 0.0],
            [0.0, -1.0, phi],
            [0.0, 1.0, phi],
            [0.0, -1.0, -phi],
            [0.0, 1.0, -phi],
            [phi, 0.0, -1.0],
            [phi, 0.0, 1.0],
            [-phi, 0.0, -1.0],
            [-phi, 0.0, 1.0],
        ],
        dtype=float,
    )
    vertices /= np.linalg.norm(vertices, axis=1)[:, None]
    faces = np.asarray(
        [
            [0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11],
            [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8],
            [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9],
            [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1],
        ],
        dtype=np.int32,
    )
    return vertices, _orient_faces_outward(vertices, faces)


def _orient_faces_outward(vertices, faces):
    """Orient every surface triangle away from the origin."""
    oriented = np.asarray(faces, dtype=np.int32).copy()
    for face in oriented:
        a, b, c = vertices[face]
        normal = np.cross(b - a, c - a)
        if np.dot(normal, a + b + c) < 0.0:
            face[1], face[2] = face[2], face[1]
    return oriented


def _subdivide_unit_sphere(vertices, faces):
    """Split each triangle into four and project new vertices to the sphere."""
    points = [np.asarray(point, dtype=float) for point in vertices]
    midpoint_indices = {}

    def midpoint(first, second):
        key = tuple(sorted((int(first), int(second))))
        if key not in midpoint_indices:
            point = points[key[0]] + points[key[1]]
            length = np.linalg.norm(point)
            if length <= np.finfo(float).eps:
                raise ValueError("icosphere edge produced an invalid midpoint")
            midpoint_indices[key] = len(points)
            points.append(point / length)
        return midpoint_indices[key]

    subdivided = []
    for first, second, third in faces:
        first_second = midpoint(first, second)
        second_third = midpoint(second, third)
        third_first = midpoint(third, first)
        subdivided.extend(
            [
                [first, first_second, third_first],
                [second, second_third, first_second],
                [third, third_first, second_third],
                [first_second, second_third, third_first],
            ]
        )

    result_vertices = np.asarray(points, dtype=float)
    result_faces = _orient_faces_outward(result_vertices, subdivided)
    return result_vertices, result_faces


def _signed_six_times_tetra_volume(positions, tetrahedron):
    first, second, third, fourth = positions[tetrahedron]
    return float(
        np.dot(second - first, np.cross(third - first, fourth - first))
    )


def _validate_mesh(mesh):
    if not np.isfinite(mesh.positions).all():
        raise ValueError("ball mesh contains a non-finite position")
    if mesh.tetrahedra.ndim != 2 or mesh.tetrahedra.shape[1] != 4:
        raise ValueError("ball volume must contain tetrahedra")
    if mesh.surface_triangles.ndim != 2 or mesh.surface_triangles.shape[1] != 3:
        raise ValueError("ball boundary must contain triangles")
    if mesh.tetrahedra.min() < 0 or mesh.tetrahedra.max() >= len(mesh.positions):
        raise ValueError("ball tetrahedron index is out of range")

    volumes = np.asarray(
        [
            _signed_six_times_tetra_volume(mesh.positions, tetrahedron)
            for tetrahedron in mesh.tetrahedra
        ]
    )
    if np.any(volumes <= 1.0e-12):
        raise ValueError("ball mesh contains an inverted or degenerate tetrahedron")

    # A tetrahedral star is valid only when its single-use faces are exactly
    # the known closed icosphere boundary.  This catches missing or duplicated
    # tetrahedra before SOFA ever sees the mesh.
    face_counts = Counter(
        tuple(sorted(face))
        for tetrahedron in mesh.tetrahedra
        for face in combinations(tetrahedron, 3)
    )
    boundary = {face for face, count in face_counts.items() if count == 1}
    expected = {tuple(sorted(face)) for face in mesh.surface_triangles}
    if boundary != expected:
        raise ValueError("tetrahedral ball boundary is not closed")


def generate_ball_mesh(
    radius=BALL_RADIUS,
    center=BALL_START_CENTER,
    subdivisions=ICOSPHERE_SUBDIVISIONS,
):
    """Generate a deterministic star-tetrahedralised icosphere."""
    radius = float(radius)
    center = np.asarray(center, dtype=float)
    subdivisions = int(subdivisions)
    if radius <= 0.0:
        raise ValueError("ball radius must be positive")
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("ball center must contain three finite coordinates")
    if subdivisions < 0:
        raise ValueError("icosphere subdivisions cannot be negative")

    unit_vertices, surface_triangles = _unit_icosahedron()
    for _ in range(subdivisions):
        unit_vertices, surface_triangles = _subdivide_unit_sphere(
            unit_vertices, surface_triangles
        )

    surface_positions = center + radius * unit_vertices
    center_index = len(surface_positions)
    positions = np.vstack((surface_positions, center))

    tetrahedra = []
    for surface_triangle in surface_triangles:
        tetrahedron = [center_index, *surface_triangle.tolist()]
        if _signed_six_times_tetra_volume(positions, tetrahedron) < 0.0:
            tetrahedron[2], tetrahedron[3] = tetrahedron[3], tetrahedron[2]
        tetrahedra.append(tetrahedron)

    mesh = BallMesh(
        positions=positions,
        tetrahedra=np.asarray(tetrahedra, dtype=np.int32),
        surface_triangles=np.asarray(surface_triangles, dtype=np.int32),
        surface_vertex_count=len(surface_positions),
    )
    _validate_mesh(mesh)
    return mesh


def add_deformable_ball(root_node, name="DeformableBall"):
    """Add a free, light FEM ball that collides with robot and floor."""
    mesh = generate_ball_mesh()
    positions = mesh.positions.tolist()
    tetrahedra = mesh.tetrahedra.tolist()

    ball = root_node.addChild(name)
    ball.addObject(
        "EulerImplicitSolver",
        name="odeSolver",
        firstOrder=False,
        rayleighMass=0.02,
        rayleighStiffness=0.005,
    )
    linear_solver = ball.addObject(
        "SparseLDLSolver",
        name="linearSolver",
        template="CompressedRowSparseMatrixMat3x3",
    )
    volume_topology = ball.addObject(
        "TetrahedronSetTopologyContainer",
        name="topology",
        position=positions,
        tetrahedra=tetrahedra,
    )
    ball.addObject("TetrahedronSetTopologyModifier", name="topologyModifier")
    volume_dofs = ball.addObject(
        "MechanicalObject",
        name="dofs",
        template="Vec3d",
        position=positions,
        rest_position=positions,
    )
    ball.addObject(
        "TetrahedronFEMForceField",
        name="elasticity",
        method="large",
        youngModulus=BALL_YOUNG_MODULUS,
        poissonRatio=BALL_POISSON_RATIO,
    )
    ball.addObject(
        "MeshMatrixMass",
        name="mass",
        totalMass=BALL_MASS,
        lumping=True,
    )

    surface = ball.addChild("Surface")
    surface_topology = surface.addObject(
        "TriangleSetTopologyContainer",
        name="topology",
    )
    surface.addObject("TriangleSetTopologyModifier", name="topologyModifier")
    surface.addObject(
        "Tetra2TriangleTopologicalMapping",
        name="surfaceTopology",
        input=volume_topology.getLinkPath(),
        output=surface_topology.getLinkPath(),
        flipNormals=False,
    )
    surface_dofs = surface.addObject(
        "MechanicalObject",
        name="dofs",
        template="Vec3d",
        rest_position="@../dofs.rest_position",
    )
    surface.addObject(
        "TriangleCollisionModel",
        name="collisionModel",
        moving=True,
        simulated=True,
        selfCollision=False,
        proximity=BALL_PROXIMITY,
        group=BALL_COLLISION_GROUP,
    )
    surface.addObject(
        "IdentityMapping",
        name="surfaceMapping",
        input=volume_dofs.getLinkPath(),
        output=surface_dofs.getLinkPath(),
    )

    visual = surface.addChild("Visual")
    visual.addObject(
        "TriangleSetTopologyContainer",
        name="topology",
        src=surface_topology.getLinkPath(),
    )
    visual_model = visual.addObject(
        "OglModel",
        name="model",
        template="Vec3d",
        color=BALL_COLOR,
        updateNormals=True,
    )
    visual.addObject(
        "IdentityMapping",
        name="visualMapping",
        input=surface_dofs.getLinkPath(),
        output=visual_model.getLinkPath(),
    )

    ball.addObject(
        "LinearSolverConstraintCorrection",
        name="constraintCorrection",
        linearSolver=linear_solver.getLinkPath(),
    )

    print(
        "[squashSim] deformable ball: "
        f"diameter={2.0 * BALL_RADIUS:.2f} m, mass={BALL_MASS:.2f} kg, "
        f"{len(mesh.positions)} nodes, {len(mesh.tetrahedra)} tetrahedra"
    )
    print(
        "[squashSim] ball contact: group 3 collides with floor and robot"
    )
    return ball
