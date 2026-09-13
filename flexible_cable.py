"""Free one-metre BeamFEM cable for the squashSim manipulation scene.

The cable's mechanics are a chain of rigid cross-section frames joined by
SOFA beam elements.  One sphere model on those frames provides robust contact,
and a separate mapped tube provides the visible round cable.
"""

from dataclasses import dataclass

import numpy as np


CABLE_LENGTH = 1.0
CABLE_RADIUS = 0.025
CABLE_MASS = 0.15
CABLE_NODE_COUNT = 31
CABLE_YOUNG_MODULUS = 200_000.0
CABLE_POISSON_RATIO = 0.45
CABLE_START = (-0.25, 0.35, 0.35)
CABLE_DIRECTION = (1.0, 0.10, 0.0)
CABLE_COLOR = [0.08, 0.32, 0.78, 1.0]
CABLE_COLLISION_GROUP = [3]
CABLE_PROXIMITY = 0.001
CABLE_VISUAL_SIDES = 12
CABLE_GRASP_INDEX = CABLE_NODE_COUNT // 2


@dataclass(frozen=True)
class CableGeometry:
    """Initial Rigid3 frames and line topology for the straight cable."""

    frames: np.ndarray
    edges: np.ndarray
    direction: np.ndarray


def _quaternion_from_x_axis(direction):
    """Return an xyzw quaternion rotating local +X onto ``direction``."""
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    x_axis = np.asarray([1.0, 0.0, 0.0])
    cosine = float(np.dot(x_axis, direction))

    if cosine > 1.0 - 1.0e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0])
    if cosine < -1.0 + 1.0e-12:
        return np.asarray([0.0, 0.0, 1.0, 0.0])

    quaternion = np.concatenate((np.cross(x_axis, direction), [1.0 + cosine]))
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def generate_cable_geometry(
    length=CABLE_LENGTH,
    node_count=CABLE_NODE_COUNT,
    start=CABLE_START,
    direction=CABLE_DIRECTION,
):
    """Generate equally spaced Rigid3 frames along an exact-length line."""
    length = float(length)
    node_count = int(node_count)
    start = np.asarray(start, dtype=float)
    direction = np.asarray(direction, dtype=float)

    if length <= 0.0:
        raise ValueError("cable length must be positive")
    if node_count < 2:
        raise ValueError("cable requires at least two nodes")
    if start.shape != (3,) or not np.isfinite(start).all():
        raise ValueError("cable start must contain three finite coordinates")
    if direction.shape != (3,) or not np.isfinite(direction).all():
        raise ValueError("cable direction must contain three finite coordinates")
    direction_norm = np.linalg.norm(direction)
    if direction_norm <= np.finfo(float).eps:
        raise ValueError("cable direction cannot be zero")

    unit_direction = direction / direction_norm
    quaternion = _quaternion_from_x_axis(unit_direction)
    distances = np.linspace(0.0, length, node_count)
    centers = start + distances[:, None] * unit_direction
    orientations = np.repeat(quaternion[None, :], node_count, axis=0)
    frames = np.hstack((centers, orientations))
    edges = np.column_stack(
        (
            np.arange(node_count - 1, dtype=np.int32),
            np.arange(1, node_count, dtype=np.int32),
        )
    )

    if not np.isfinite(frames).all():
        raise ValueError("generated cable contains a non-finite frame")
    if not np.allclose(np.linalg.norm(frames[:, 3:], axis=1), 1.0):
        raise ValueError("generated cable contains a non-unit orientation")
    if not np.isclose(
        np.linalg.norm(frames[-1, :3] - frames[0, :3]), length
    ):
        raise ValueError("generated cable does not have the requested length")

    return CableGeometry(frames=frames, edges=edges, direction=unit_direction)


def add_flexible_cable(root_node, name="FlexibleCable"):
    """Add a free BeamFEM cable that collides with the floor and robot."""
    geometry = generate_cable_geometry()
    frames = geometry.frames.tolist()
    edges = geometry.edges.tolist()

    cable = root_node.addChild(name)
    cable.addObject(
        "EulerImplicitSolver",
        name="odeSolver",
        firstOrder=False,
        rayleighMass=0.02,
        rayleighStiffness=0.01,
    )
    linear_solver = cable.addObject(
        "BTDLinearSolver",
        name="linearSolver",
        template="BTDMatrix6d",
        printLog=False,
        verbose=False,
    )
    cable_dofs = cable.addObject(
        "MechanicalObject",
        name="dofs",
        template="Rigid3d",
        position=frames,
        rest_position=frames,
        showObject=False,
    )
    cable_topology = cable.addObject(
        "MeshTopology",
        name="topology",
        lines=edges,
    )
    cable.addObject(
        "UniformMass",
        name="mass",
        totalMass=CABLE_MASS,
    )
    cable.addObject(
        "BeamFEMForceField",
        name="elasticity",
        radius=CABLE_RADIUS,
        radiusInner=0.0,
        youngModulus=CABLE_YOUNG_MODULUS,
        poissonRatio=CABLE_POISSON_RATIO,
    )
    cable.addObject(
        "SphereCollisionModel",
        name="collisionModel",
        radius=CABLE_RADIUS,
        moving=True,
        simulated=True,
        selfCollision=False,
        proximity=CABLE_PROXIMITY,
        group=CABLE_COLLISION_GROUP,
    )

    tube = cable.addChild("Tube")
    tube_topology = tube.addObject(
        "QuadSetTopologyContainer",
        name="topology",
    )
    tube.addObject("QuadSetTopologyModifier", name="topologyModifier")
    tube.addObject(
        "Edge2QuadTopologicalMapping",
        name="tubeTopology",
        input=cable_topology.getLinkPath(),
        output=tube_topology.getLinkPath(),
        nbPointsOnEachCircle=CABLE_VISUAL_SIDES,
        radius=CABLE_RADIUS,
    )
    tube_dofs = tube.addObject(
        "MechanicalObject",
        name="dofs",
        template="Vec3d",
    )
    tube.addObject(
        "TubularMapping",
        name="tubeMapping",
        input=cable_dofs.getLinkPath(),
        output=tube_dofs.getLinkPath(),
        nbPointsOnEachCircle=CABLE_VISUAL_SIDES,
        radius=CABLE_RADIUS,
    )

    visual = tube.addChild("Visual")
    visual_model = visual.addObject(
        "OglModel",
        name="model",
        template="Vec3d",
        color=CABLE_COLOR,
        updateNormals=True,
    )
    visual.addObject(
        "IdentityMapping",
        name="mapping",
        input=tube_dofs.getLinkPath(),
        output=visual_model.getLinkPath(),
    )

    cable.addObject(
        "LinearSolverConstraintCorrection",
        name="constraintCorrection",
        linearSolver=linear_solver.getLinkPath(),
    )

    spacing = CABLE_LENGTH / (CABLE_NODE_COUNT - 1)
    print(
        "[squashSim] flexible cable: "
        f"length={CABLE_LENGTH:.2f} m, diameter={2.0 * CABLE_RADIUS:.3f} m, "
        f"mass={CABLE_MASS:.2f} kg"
    )
    print(
        "[squashSim] cable mechanics: "
        f"{CABLE_NODE_COUNT} BeamFEM frames at {spacing:.4f} m spacing; "
        f"grasp index={CABLE_GRASP_INDEX}"
    )
    print(
        "[squashSim] cable contact: one sphere layer in group 3; "
        "floor/robot collision enabled, self-collision disabled"
    )
    return cable
