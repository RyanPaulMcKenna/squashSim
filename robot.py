"""UR5 + OnRobot RG2 articulated model for SOFA.

The scene follows the original squashSim ArticulatedSystemPlugin layout, but
all link geometry, joint centres, axes and limits come from Andrej Orsula's
``ur5_rg2.urdf``. The two simplified RG2 finger joints remain separate SOFA
articulations and are driven as a mirrored pair by one GUI command.
"""

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import Sofa.Simulation as Sim
from scipy.spatial.transform import Rotation

from robotGUI import RobotGUI


MODEL_ROOT = Path(
    os.environ.get("SQUASHSIM_UR5_RG2_ROOT", "~/ur5_rg2_ign")
).expanduser().resolve()
URDF_PATH = MODEL_ROOT / "urdf" / "ur5_rg2.urdf"
MESH_ROOT = MODEL_ROOT / "ur5_rg2"

ARM_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
FINGER_JOINT_NAMES = ("rg2_finger_joint1", "rg2_finger_joint2")
ACTUATED_JOINT_NAMES = ARM_JOINT_NAMES + FINGER_JOINT_NAMES

# The hand is fixed to wrist_3_link, so it shares rigid body index 6. Each
# finger needs its own rigid output and ArticulationCenter even though the GUI
# mirrors their values.
RIGID_LINK_NAMES = (
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "rg2_leftfinger",
    "rg2_rightfinger",
)
RIGID_INDEX = {name: index for index, name in enumerate(RIGID_LINK_NAMES)}
PART_NAMES = RIGID_LINK_NAMES[:7] + (
    "rg2_hand",
    "rg2_leftfinger",
    "rg2_rightfinger",
)

# Convert the reference model's Z-up coordinates into SOFA's Y-up world with
# a proper right-handed -90 degree rotation around X.
URDF_TO_SOFA = Rotation.from_euler("x", -90.0, degrees=True).as_matrix()

PART_COLORS = {
    "base_link": [0.17, 0.20, 0.23, 1.0],
    "shoulder_link": [0.27, 0.50, 0.70, 1.0],
    "upper_arm_link": [0.72, 0.75, 0.78, 1.0],
    "forearm_link": [0.72, 0.75, 0.78, 1.0],
    "wrist_1_link": [0.27, 0.50, 0.70, 1.0],
    "wrist_2_link": [0.27, 0.50, 0.70, 1.0],
    "wrist_3_link": [0.17, 0.20, 0.23, 1.0],
    "rg2_hand": [0.12, 0.13, 0.15, 1.0],
    "rg2_leftfinger": [0.86, 0.38, 0.08, 1.0],
    "rg2_rightfinger": [0.86, 0.38, 0.08, 1.0],
}

# The reference base collision mesh reaches 3 mm below the URDF base frame.
# Placing the floor here makes the robot sit on it without changing any of the
# already-working articulation transforms.
FLOOR_TOP_Y = -0.003
FLOOR_HALF_EXTENT = 1.2
FLOOR_THICKNESS = 0.03
FLOOR_COLOR = [0.34, 0.37, 0.41, 1.0]


@dataclass(frozen=True)
class Origin:
    translation: np.ndarray
    rotation: np.ndarray


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    origin: Origin
    axis: np.ndarray
    lower: float
    upper: float


@dataclass(frozen=True)
class TriangleMesh:
    positions: np.ndarray
    triangles: np.ndarray


def _numbers(text, default):
    if text is None:
        return np.asarray(default, dtype=float)
    return np.fromstring(text, sep=" ", dtype=float)


def _origin(element):
    origin_element = element.find("origin")
    if origin_element is None:
        return Origin(np.zeros(3), np.eye(3))
    translation = _numbers(origin_element.get("xyz"), [0.0, 0.0, 0.0])
    rpy = _numbers(origin_element.get("rpy"), [0.0, 0.0, 0.0])
    return Origin(translation, Rotation.from_euler("xyz", rpy).as_matrix())


def _load_urdf():
    if not URDF_PATH.is_file():
        raise FileNotFoundError(
            f"UR5+RG2 URDF not found: {URDF_PATH}\n"
            "Clone https://github.com/AndrejOrsula/ur5_rg2_ign.git to "
            "~/ur5_rg2_ign or set SQUASHSIM_UR5_RG2_ROOT."
        )

    root = ET.parse(URDF_PATH).getroot()
    links = {element.get("name"): element for element in root.findall("link")}
    joint_elements = {
        element.get("name"): element for element in root.findall("joint")
    }

    required_links = set(RIGID_LINK_NAMES) | {"rg2_hand"}
    missing_links = sorted(required_links - set(links))
    missing_joints = sorted(
        (set(ACTUATED_JOINT_NAMES) | {"ur5_hand_joint"})
        - set(joint_elements)
    )
    if missing_links or missing_joints:
        raise ValueError(
            "The reference URDF does not contain the expected UR5+RG2 chain. "
            f"Missing links={missing_links}, joints={missing_joints}"
        )

    joints = {}
    for name, element in joint_elements.items():
        limit = element.find("limit")
        joints[name] = JointSpec(
            name=name,
            parent=element.find("parent").get("link"),
            child=element.find("child").get("link"),
            origin=_origin(element),
            axis=_numbers(
                element.find("axis").get("xyz")
                if element.find("axis") is not None
                else None,
                [0.0, 0.0, 0.0],
            ),
            lower=float(limit.get("lower", "0")) if limit is not None else 0.0,
            upper=float(limit.get("upper", "0")) if limit is not None else 0.0,
        )

    # ArticulatedSystemMapping cannot encode a fixed orientation between two
    # movable rigid frames. The reference arm joints all have zero origin
    # rotation, so their URDF transforms map directly to SOFA centres. The
    # rotated fixed hand mount is folded into the RG2 mesh frames below.
    rotated_arm_origins = [
        name
        for name in ARM_JOINT_NAMES
        if not np.allclose(joints[name].origin.rotation, np.eye(3), atol=1e-9)
    ]
    if rotated_arm_origins:
        raise ValueError(
            "Unexpected rotated arm-joint origins in ur5_rg2.urdf: "
            + ", ".join(rotated_arm_origins)
        )

    return links, joints


LINKS, JOINTS = _load_urdf()
HAND_MOUNT = JOINTS["ur5_hand_joint"]


def _mesh_path(link_name, geometry_kind):
    geometry = LINKS[link_name].find(geometry_kind)
    if geometry is None:
        raise ValueError(
            f"Link {link_name!r} has no {geometry_kind} mesh in {URDF_PATH}"
        )
    mesh = geometry.find("./geometry/mesh")
    if mesh is None:
        raise ValueError(
            f"Link {link_name!r} has no {geometry_kind} mesh geometry in "
            f"{URDF_PATH}"
        )

    uri = mesh.get("filename")
    package_prefix = "package://ur5_rg2_ign/"
    if uri.startswith(package_prefix):
        path = MESH_ROOT / uri[len(package_prefix) :]
    else:
        raw_path = Path(uri).expanduser()
        path = raw_path if raw_path.is_absolute() else URDF_PATH.parent / raw_path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(
            f"{geometry_kind.title()} mesh for {link_name!r} not found: {path}"
        )
    return path


VISUAL_MESH_PATHS = {
    name: _mesh_path(name, "visual") for name in PART_NAMES
}
COLLISION_MESH_PATHS = {
    name: _mesh_path(name, "collision") for name in PART_NAMES
}


def _collada_ref(value):
    if value is None or not value.startswith("#"):
        raise ValueError(f"Unsupported COLLADA reference: {value!r}")
    return value[1:]


def _collada_source(source, namespace, path):
    accessor = source.find("c:technique_common/c:accessor", namespace)
    if accessor is None:
        raise ValueError(f"Missing COLLADA accessor in {path}")

    array_id = _collada_ref(accessor.get("source"))
    array = next(
        (child for child in source if child.get("id") == array_id), None
    )
    if array is None or array.text is None:
        raise ValueError(f"Missing COLLADA array {array_id!r} in {path}")

    values = np.fromstring(array.text, sep=" ", dtype=float)
    count = int(accessor.get("count", "0"))
    stride = int(accessor.get("stride", "1"))
    offset = int(accessor.get("offset", "0"))
    end = offset + count * stride
    if count == 0 or stride < 3 or values.size < end:
        raise ValueError(f"Invalid COLLADA accessor {array_id!r} in {path}")
    return values[offset:end].reshape(count, stride)


def _collada_geometry(geometry, namespace, path):
    mesh = geometry.find("c:mesh", namespace)
    if mesh is None:
        raise ValueError(f"COLLADA geometry without a mesh in {path}")

    unsupported = [
        primitive.tag.rsplit("}", 1)[-1]
        for primitive in mesh
        if primitive.tag.rsplit("}", 1)[-1]
        in {"lines", "linestrips", "polygons", "polylist", "trifans", "tristrips"}
    ]
    if unsupported:
        raise ValueError(
            f"Unsupported COLLADA primitives {sorted(set(unsupported))} in {path}"
        )

    sources = {
        source.get("id"): _collada_source(source, namespace, path)
        for source in mesh.findall("c:source", namespace)
    }
    vertices = {}
    for vertex_set in mesh.findall("c:vertices", namespace):
        position_input = next(
            (
                item
                for item in vertex_set.findall("c:input", namespace)
                if item.get("semantic") == "POSITION"
            ),
            None,
        )
        if position_input is None:
            raise ValueError(f"COLLADA vertices without positions in {path}")
        source_id = _collada_ref(position_input.get("source"))
        vertices[vertex_set.get("id")] = sources[source_id][:, :3]

    position_tables = []
    table_offsets = {}
    triangle_tables = []
    for triangle_set in mesh.findall("c:triangles", namespace):
        inputs = triangle_set.findall("c:input", namespace)
        vertex_input = next(
            (
                item
                for item in inputs
                if item.get("semantic") == "VERTEX"
            ),
            None,
        )
        if vertex_input is None:
            raise ValueError(f"COLLADA triangles without vertices in {path}")

        vertex_set_id = _collada_ref(vertex_input.get("source"))
        if vertex_set_id not in table_offsets:
            table_offsets[vertex_set_id] = sum(
                len(table) for table in position_tables
            )
            position_tables.append(vertices[vertex_set_id])

        index_stride = max(
            int(item.get("offset", "0")) for item in inputs
        ) + 1
        index_element = triangle_set.find("c:p", namespace)
        if index_element is None or index_element.text is None:
            raise ValueError(f"COLLADA triangles without indices in {path}")
        indices = np.fromstring(index_element.text, sep=" ", dtype=np.int64)
        triangle_count = int(triangle_set.get("count", "0"))
        expected_size = triangle_count * 3 * index_stride
        if triangle_count == 0 or indices.size != expected_size:
            raise ValueError(f"Invalid COLLADA triangle indices in {path}")

        vertex_offset = int(vertex_input.get("offset", "0"))
        triangles = indices.reshape(-1, index_stride)[:, vertex_offset]
        triangles = triangles.reshape(-1, 3)
        triangle_tables.append(triangles + table_offsets[vertex_set_id])

    if not position_tables or not triangle_tables:
        raise ValueError(f"No triangular geometry found in {path}")
    return TriangleMesh(
        positions=np.concatenate(position_tables),
        triangles=np.concatenate(triangle_tables).astype(np.int32),
    )


def _collada_node_transform(node, path):
    transform = np.eye(4)
    for element in node:
        tag = element.tag.rsplit("}", 1)[-1]
        if tag == "matrix":
            values = np.fromstring(element.text or "", sep=" ", dtype=float)
            if values.size != 16:
                raise ValueError(f"Invalid COLLADA matrix in {path}")
            operation = values.reshape(4, 4)
            if not np.allclose(operation[3], [0.0, 0.0, 0.0, 1.0]):
                raise ValueError(f"Unsupported COLLADA matrix layout in {path}")
        elif tag == "translate":
            values = np.fromstring(element.text or "", sep=" ", dtype=float)
            if values.size != 3:
                raise ValueError(f"Invalid COLLADA translation in {path}")
            operation = np.eye(4)
            operation[:3, 3] = values
        elif tag == "rotate":
            values = np.fromstring(element.text or "", sep=" ", dtype=float)
            if values.size != 4 or np.linalg.norm(values[:3]) == 0.0:
                raise ValueError(f"Invalid COLLADA rotation in {path}")
            axis = values[:3] / np.linalg.norm(values[:3])
            operation = np.eye(4)
            operation[:3, :3] = Rotation.from_rotvec(
                axis * np.deg2rad(values[3])
            ).as_matrix()
        elif tag == "scale":
            values = np.fromstring(element.text or "", sep=" ", dtype=float)
            if values.size != 3:
                raise ValueError(f"Invalid COLLADA scale in {path}")
            operation = np.eye(4)
            operation[np.arange(3), np.arange(3)] = values
        elif tag in {"lookat", "skew"}:
            raise ValueError(f"Unsupported COLLADA transform {tag!r} in {path}")
        else:
            continue
        transform = transform @ operation
    return transform


def _load_collada_visual(path):
    """Read and bake the reference DAE's scene-node transforms."""
    document = ET.parse(path).getroot()
    if "}" not in document.tag:
        raise ValueError(f"COLLADA document has no XML namespace: {path}")
    namespace = {"c": document.tag.split("}", 1)[0].lstrip("{")}

    up_axis = document.find("c:asset/c:up_axis", namespace)
    if up_axis is None or (up_axis.text or "").strip() != "Z_UP":
        raise ValueError(f"Expected a Z_UP COLLADA visual mesh: {path}")
    unit = document.find("c:asset/c:unit", namespace)
    if unit is not None and not np.isclose(float(unit.get("meter", "1")), 1.0):
        raise ValueError(f"Expected metre-based COLLADA coordinates: {path}")

    geometries = {
        geometry.get("id"): _collada_geometry(geometry, namespace, path)
        for geometry in document.findall(
            "c:library_geometries/c:geometry", namespace
        )
    }
    scene_instance = document.find(
        "c:scene/c:instance_visual_scene", namespace
    )
    if scene_instance is None:
        raise ValueError(f"COLLADA document has no visual scene: {path}")
    scene_id = _collada_ref(scene_instance.get("url"))
    visual_scene = next(
        (
            scene
            for scene in document.findall(
                "c:library_visual_scenes/c:visual_scene", namespace
            )
            if scene.get("id") == scene_id
        ),
        None,
    )
    if visual_scene is None:
        raise ValueError(f"COLLADA visual scene {scene_id!r} not found: {path}")

    position_tables = []
    triangle_tables = []

    def visit(node, parent_transform):
        node_transform = parent_transform @ _collada_node_transform(node, path)
        for instance in node.findall("c:instance_geometry", namespace):
            geometry_id = _collada_ref(instance.get("url"))
            if geometry_id not in geometries:
                raise ValueError(
                    f"COLLADA geometry {geometry_id!r} not found: {path}"
                )
            mesh = geometries[geometry_id]
            transformed = (
                node_transform[:3, :3] @ mesh.positions.T
            ).T + node_transform[:3, 3]
            vertex_offset = sum(len(table) for table in position_tables)
            position_tables.append(transformed)
            triangle_tables.append(mesh.triangles + vertex_offset)
        for child in node.findall("c:node", namespace):
            visit(child, node_transform)

    for node in visual_scene.findall("c:node", namespace):
        visit(node, np.eye(4))

    if not position_tables:
        raise ValueError(f"COLLADA visual scene contains no geometry: {path}")
    return TriangleMesh(
        positions=np.concatenate(position_tables),
        triangles=np.concatenate(triangle_tables).astype(np.int32),
    )


_VISUAL_MESHES_BY_PATH = {
    path: _load_collada_visual(path)
    for path in dict.fromkeys(VISUAL_MESH_PATHS.values())
}
VISUAL_MESHES = {
    name: _VISUAL_MESHES_BY_PATH[path]
    for name, path in VISUAL_MESH_PATHS.items()
}


def _as_pose(transform):
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return [*transform[:3, 3].tolist(), *quaternion.tolist()]


def _basis_transform(transform):
    converted = np.eye(4)
    converted[:3, :3] = URDF_TO_SOFA @ transform[:3, :3] @ URDF_TO_SOFA.T
    converted[:3, 3] = URDF_TO_SOFA @ transform[:3, 3]
    return converted


def _origin_matrix(origin):
    transform = np.eye(4)
    transform[:3, :3] = origin.rotation
    transform[:3, 3] = origin.translation
    return transform


def initial_rigid_poses():
    """Return zero-angle poses for the nine SOFA rigid output frames."""
    urdf_world = {"base_link": np.eye(4)}
    for name in ARM_JOINT_NAMES:
        joint = JOINTS[name]
        urdf_world[joint.child] = urdf_world[joint.parent] @ _origin_matrix(
            joint.origin
        )

    poses_by_link = {
        name: _as_pose(_basis_transform(urdf_world[name]))
        for name in RIGID_LINK_NAMES[:7]
    }

    wrist_world = urdf_world["wrist_3_link"]
    mount = _origin_matrix(HAND_MOUNT.origin)
    for name in FINGER_JOINT_NAMES:
        joint = JOINTS[name]
        centre = wrist_world @ mount @ _origin_matrix(joint.origin)
        # The computational finger frame is aligned with the wrist at q=0.
        # Its fixed hand rotation is applied to mesh vertices and axis instead.
        centre[:3, :3] = wrist_world[:3, :3]
        poses_by_link[joint.child] = _as_pose(_basis_transform(centre))

    return [poses_by_link[name] for name in RIGID_LINK_NAMES]


def _part_transform(link_name, geometry_kind="collision"):
    """Transform URDF geometry into its SOFA computational rigid frame."""
    geometry = LINKS[link_name].find(geometry_kind)
    if geometry is None:
        raise ValueError(
            f"Link {link_name!r} has no {geometry_kind} geometry in {URDF_PATH}"
        )
    geometry_origin = _origin(geometry)

    if link_name in RIGID_LINK_NAMES[:7]:
        prefix_rotation = np.eye(3)
        prefix_translation = np.zeros(3)
    elif link_name == "rg2_hand":
        prefix_rotation = HAND_MOUNT.origin.rotation
        prefix_translation = HAND_MOUNT.origin.translation
    else:
        finger_joint = next(
            JOINTS[name]
            for name in FINGER_JOINT_NAMES
            if JOINTS[name].child == link_name
        )
        prefix_rotation = (
            HAND_MOUNT.origin.rotation @ finger_joint.origin.rotation
        )
        # The finger rigid centre already sits at its joint origin.
        prefix_translation = np.zeros(3)

    rotation = URDF_TO_SOFA @ prefix_rotation @ geometry_origin.rotation
    translation = URDF_TO_SOFA @ (
        prefix_translation + prefix_rotation @ geometry_origin.translation
    )
    return rotation, translation


def _add_part(parent, link_name, rigid_index):
    part = parent.addChild(link_name)

    # Group 1 suppresses collision between the robot's own meshes. The base
    # additionally belongs to group 2 so its permanent, bolted contact with the
    # floor is ignored. Moving links share no group with the floor and therefore
    # generate normal contact constraints against it.
    collision_groups = [1, 2] if link_name == "base_link" else [1]

    collision = part.addChild("Collision")
    collision.addObject(
        "MeshSTLLoader",
        name="loader",
        filename=str(COLLISION_MESH_PATHS[link_name]),
    )
    collision.addObject("MeshTopology", name="topology", src="@loader")

    rotation, translation = _part_transform(link_name, "collision")
    quaternion = Rotation.from_matrix(rotation).as_quat()
    collision.addObject(
        "TransformEngine",
        name="urdfTransform",
        input_position="@loader.position",
        quaternion=quaternion.tolist(),
        translation=translation.tolist(),
    )
    collision.addObject(
        "MechanicalObject",
        name="vertices",
        template="Vec3d",
        position="@urdfTransform.output_position",
        rest_position="@urdfTransform.output_position",
    )
    collision.addObject(
        "TriangleCollisionModel",
        name="model",
        moving=True,
        simulated=True,
        selfCollision=False,
        group=collision_groups,
    )
    collision.addObject(
        "RigidMapping",
        name="rigidMapping",
        input="@../../../dofs",
        output="@vertices",
        index=rigid_index,
        globalToLocalCoords=False,
    )

    visual_mesh = VISUAL_MESHES[link_name]
    visual_rotation, visual_translation = _part_transform(
        link_name, "visual"
    )
    visual_positions = (
        visual_rotation @ visual_mesh.positions.T
    ).T + visual_translation
    visual = part.addChild("Visual")
    visual.addObject(
        "OglModel",
        name="model",
        position=visual_positions.tolist(),
        triangles=visual_mesh.triangles.tolist(),
        color=PART_COLORS[link_name],
        updateNormals=True,
    )
    visual.addObject(
        "RigidMapping",
        name="rigidMapping",
        input="@../../../dofs",
        output="@model",
        index=rigid_index,
        globalToLocalCoords=False,
    )
    return part


def _add_articulation_center(
    parent,
    name,
    parent_index,
    child_index,
    position_on_parent,
    axis,
    articulation_index,
):
    center = parent.addChild(name)
    center.addObject(
        "ArticulationCenter",
        name="center",
        parentIndex=parent_index,
        childIndex=child_index,
        posOnParent=np.asarray(position_on_parent, dtype=float).tolist(),
        posOnChild=[0.0, 0.0, 0.0],
        articulationProcess=0,
    )
    articulation = center.addChild("Articulation")
    articulation.addObject(
        "Articulation",
        name="joint",
        translation=False,
        rotation=True,
        rotationAxis=np.asarray(axis, dtype=float).tolist(),
        articulationIndex=articulation_index,
    )
    return center


def add_floor(root_node, name="Floor"):
    """Add a visible static slab whose top meets the UR5 base.

    The floor is group 2. Only the base also belongs to group 2, suppressing
    that permanent contact pair; the moving robot links remain in group 1 and
    collide with this surface.
    """
    half = FLOOR_HALF_EXTENT
    top = FLOOR_TOP_Y
    bottom = top - FLOOR_THICKNESS

    top_vertices = [
        [-half, top, -half],
        [half, top, -half],
        [half, top, half],
        [-half, top, half],
    ]
    # Counter-clockwise when viewed from above, giving an upward-facing normal.
    top_triangles = [[0, 2, 1], [0, 3, 2]]

    floor = root_node.addChild(name)
    collision = floor.addChild("Collision")
    collision.addObject(
        "MechanicalObject",
        name="vertices",
        template="Vec3d",
        position=top_vertices,
    )
    collision.addObject(
        "MeshTopology",
        name="topology",
        triangles=top_triangles,
    )
    collision.addObject(
        "TriangleCollisionModel",
        name="model",
        moving=False,
        simulated=False,
        selfCollision=False,
        group=[2],
    )

    # Render a thin box rather than a zero-thickness quad so the floor remains
    # easy to see from the default oblique camera angle. Keeping OglModel in a
    # child node also preserves SOFA's one-BaseState-per-node requirement.
    slab_vertices = top_vertices + [
        [-half, bottom, -half],
        [half, bottom, -half],
        [half, bottom, half],
        [-half, bottom, half],
    ]
    slab_triangles = [
        [0, 2, 1], [0, 3, 2],       # top
        [4, 5, 6], [4, 6, 7],       # bottom
        [0, 1, 5], [0, 5, 4],       # front
        [1, 2, 6], [1, 6, 5],       # right
        [2, 3, 7], [2, 7, 6],       # back
        [3, 0, 4], [3, 4, 7],       # left
    ]
    visual = floor.addChild("Visual")
    visual.addObject(
        "OglModel",
        name="model",
        position=slab_vertices,
        triangles=slab_triangles,
        color=FLOOR_COLOR,
        updateNormals=True,
    )

    print(
        "[squashSim] floor: "
        f"{2.0 * half:.2f} x {2.0 * half:.2f} m, top y={top:.3f} m"
    )
    return floor


def articulation_definitions():
    """Return all eight centres derived from the reference URDF."""
    definitions = []
    for articulation_index, name in enumerate(ARM_JOINT_NAMES):
        joint = JOINTS[name]
        definitions.append(
            {
                "name": name,
                "parent_index": RIGID_INDEX[joint.parent],
                "child_index": RIGID_INDEX[joint.child],
                "position_on_parent": URDF_TO_SOFA @ joint.origin.translation,
                "axis": URDF_TO_SOFA @ joint.axis,
                "articulation_index": articulation_index,
            }
        )

    mount_rotation = HAND_MOUNT.origin.rotation
    mount_translation = HAND_MOUNT.origin.translation
    for finger_offset, name in enumerate(FINGER_JOINT_NAMES):
        joint = JOINTS[name]
        definitions.append(
            {
                "name": name,
                "parent_index": RIGID_INDEX["wrist_3_link"],
                "child_index": RIGID_INDEX[joint.child],
                "position_on_parent": URDF_TO_SOFA
                @ (mount_translation + mount_rotation @ joint.origin.translation),
                "axis": URDF_TO_SOFA
                @ (mount_rotation @ joint.origin.rotation @ joint.axis),
                "articulation_index": len(ARM_JOINT_NAMES) + finger_offset,
            }
        )
    return definitions


def joint_limits():
    return [
        (JOINTS[name].lower, JOINTS[name].upper)
        for name in ACTUATED_JOINT_NAMES
    ]


def _normalise_initial_angles(initial_angles):
    if initial_angles is None:
        open_angle = JOINTS[FINGER_JOINT_NAMES[0]].upper
        return [0.0] * 6 + [open_angle, open_angle]
    values = [float(value) for value in initial_angles]
    if len(values) == 7:
        values.append(values[-1])
    if len(values) != 8:
        raise ValueError("initAngles must contain 7 GUI values or 8 SOFA DOFs")
    mirrored = 0.5 * (values[6] + values[7])
    values[6] = mirrored
    values[7] = mirrored
    return values


class Robot:
    def __init__(self, node):
        self.node = node

    def addRobot(self, name="Robot", initAngles=None):
        initial_angles = _normalise_initial_angles(initAngles)
        robot_node = self.node.addChild(name)
        robot_node.addData(
            "angles",
            initial_angles,
            None,
            "UR5 joints followed by two mirrored RG2 finger joints, in radians",
            "",
            "vector<float>",
        )
        robot_node.addObject("EulerImplicitSolver")

        articulations = robot_node.addChild("Articulations")
        articulation_dofs = articulations.addObject(
            "MechanicalObject",
            name="dofs",
            template="Vec1",
            position=initial_angles,
            rest_position=robot_node.getData("angles").getLinkPath(),
        )
        articulations.addObject("ArticulatedHierarchyContainer", name="hierarchy")
        articulations.addObject("SparseLDLSolver", name="linearSolver")
        articulations.addObject(
            "UniformMass",
            name="jointMass",
            template="Vec1d",
            # Keep the exact input form already known to work in squashSim.
            vertexMass=" ".join(["1"] * len(ACTUATED_JOINT_NAMES)),
        )
        articulations.addObject(
            "RestShapeSpringsForceField",
            name="jointTargets",
            stiffness=2000.0,
            points=list(range(len(ACTUATED_JOINT_NAMES))),
        )
        articulations.addObject(
            "LinearSolverConstraintCorrection",
            name="constraintCorrection",
            linearSolver="@linearSolver",
        )

        rigid = articulations.addChild("Rigid")
        rigid_dofs = rigid.addObject(
            "MechanicalObject",
            name="dofs",
            template="Rigid3d",
            position=initial_rigid_poses(),
            showObject=False,
            showObjectScale=0.03,
        )
        rigid.addObject(
            "ArticulatedSystemMapping",
            name="articulatedMapping",
            input1=articulation_dofs.getLinkPath(),
            output=rigid_dofs.getLinkPath(),
        )

        parts = rigid.addChild("Parts")
        for link_name in RIGID_LINK_NAMES[:7]:
            _add_part(parts, link_name, RIGID_INDEX[link_name])
        _add_part(parts, "rg2_hand", RIGID_INDEX["wrist_3_link"])
        _add_part(parts, "rg2_leftfinger", RIGID_INDEX["rg2_leftfinger"])
        _add_part(parts, "rg2_rightfinger", RIGID_INDEX["rg2_rightfinger"])

        centers = articulations.addChild("ArticulationCenters")
        for definition in articulation_definitions():
            _add_articulation_center(centers, **definition)

        print(f"[squashSim] URDF: {URDF_PATH}")
        print(
            "[squashSim] articulated UR5+RG2: "
            f"{len(RIGID_LINK_NAMES)} rigid bodies, "
            f"{len(ACTUATED_JOINT_NAMES)} revolute DOFs"
        )
        print("[squashSim] geometry: DAE visuals + STL collisions")
        print("[squashSim] contact: moving robot links collide with floor")
        return robot_node


def createScene(rootNode):
    from header import addHeader

    addHeader(rootNode)
    add_floor(rootNode)
    robot_node = Robot(rootNode).addRobot()
    limits = joint_limits()
    robot_node.addObject(
        RobotGUI(
            name="sliderController",
            robot=robot_node,
            articulations_mo=robot_node.Articulations.getObject("dofs"),
            initAngles=robot_node.getData("angles").value,
            armLimits=limits[:6],
            gripperLimit=limits[6],
        )
    )

    Sim.initTextures(rootNode)
    return rootNode
