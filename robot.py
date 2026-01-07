
import Sofa.Simulation as Sim
import os, math, numpy as np
from scipy.spatial.transform import Rotation as R


# ---------- Py3.12 / NumPy 2.x shims ----------
import collections as _collections, collections.abc as _abc
for _n in ("Mapping","MutableMapping","Sequence","MutableSequence","Set","MutableSet","Iterable"):
    if not hasattr(_collections, _n):
        setattr(_collections, _n, getattr(_abc, _n))
import fractions as _fractions
if not hasattr(_fractions, "gcd"):
    _fractions.gcd = math.gcd
for old, new in {"float": float, "float_": np.float64, "int": int, "bool": bool, "complex": complex}.items():
    if not hasattr(np, old):
        setattr(np, old, new)
# ----------------------------------------------

from urdfpy import URDF
from robotGUI import RobotGUI


URDF_PATH = os.path.expanduser("~/ws_ur/assets/urdf/ur5e/UR5e_calibrated.urdf")
MESH_ROOT = os.path.dirname(URDF_PATH)


def _resolve(p):  # URDF uses relative mesh paths
    return p if os.path.isabs(p) else os.path.normpath(os.path.join(MESH_ROOT, p))

robot = URDF.load(URDF_PATH)
J = robot.joint_map
L = {l.name: l for l in robot.links}


partNames = [
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link"
]

collisionRotations = {
    "base_link_inertia": (0, 90, 90),
    "shoulder_link":     (0, 180, 180),
    "upper_arm_link":    (90, 0, 180),
    "forearm_link":      (90, 0, 180),
    "wrist_1_link":      (90, 0, 0),
    "wrist_2_link":      (-90, 0, 0),
    "wrist_3_link":      (90, 0, 180)
}

collisionTranslations = {
    "base_link_inertia": (0, 0, 0),
    "shoulder_link":     (0, 0.015, 0),
    "upper_arm_link":    (0, 0, 0),
    "forearm_link":      (0, 0, -0.01),
    "wrist_1_link":      (0, 0, 0),
    "wrist_2_link":      (0, 0.01, 0),
    "wrist_3_link":      (0, 0, 0.01)
}

GRIPPER_MESH_ROOT = '/home/ryanm/ur5_rg2_ign/ur5_rg2/meshes/' # dae /visual/rg2, stl /collision/rg2
# ----------------------------------------------

# Visual Meshes
visual_basePath = _resolve(L[partNames[0]].visuals[0].geometry.mesh.filename)
visual_shoulderPath = _resolve(L[partNames[1]].visuals[0].geometry.mesh.filename)
visual_upperarmPath = _resolve(L[partNames[2]].visuals[0].geometry.mesh.filename)
visual_forearmPath = _resolve(L[partNames[3]].visuals[0].geometry.mesh.filename)
visual_wrist1Path = _resolve(L[partNames[4]].visuals[0].geometry.mesh.filename)
visual_wrist2Path = _resolve(L[partNames[5]].visuals[0].geometry.mesh.filename)
visual_wrist3Path = _resolve(L[partNames[6]].visuals[0].geometry.mesh.filename)
rg2HandVisualPath = GRIPPER_MESH_ROOT + 'visual/rg2/hand.dae'
# rg2FingerVisual = GRIPPER_MESH_ROOT.join('/meshes/visual/finger.dae')


# Collision Meshes
collision_basePath = _resolve(L[partNames[0]].collisions[0].geometry.mesh.filename)
collision_shoulderPath = _resolve(L[partNames[1]].collisions[0].geometry.mesh.filename)
collision_upperarmPath = _resolve(L[partNames[2]].collisions[0].geometry.mesh.filename)
collision_forearmPath = _resolve(L[partNames[3]].collisions[0].geometry.mesh.filename)
collision_wrist1Path = _resolve(L[partNames[4]].collisions[0].geometry.mesh.filename)
collision_wrist2Path = _resolve(L[partNames[5]].collisions[0].geometry.mesh.filename)
collision_wrist3Path = _resolve(L[partNames[6]].collisions[0].geometry.mesh.filename)
rg2HandCollisPath = GRIPPER_MESH_ROOT + 'collision/rg2/hand.stl'


def addVisu(node, index, filename, texfilename=None):
    # Make a container node for this part's visuals
    visu = node.addChild(f'Visu{index}')

    # --- geometry branch (holds the state for vertices) ---
    geom = visu.addChild('geom')
    geom.addObject("AssimpLoader", name="loader", filename=filename, scale="0.001")
    geom.addObject("MeshTopology", src="@loader")  

    # Vertices must be a MechanicalObject and must be float (Vec3d) to match OglModel
    geom.addObject("MechanicalObject", name="vertices", template="Vec3d",
                   position="@loader.position", rest_position="@loader.position")

    # Drive these vertices with the k-th rigid pose from /rigid/dofs
    geom.addObject("RigidMapping", input="@../../../../../dofs", output="@vertices",
                   index=str(index), globalToLocalCoords="false")

    # --- rendering branch (no MechanicalObject here) ---
    visual = visu.addChild('visual')
    # if texfilename is None:
    visual.addObject("OglModel", 
                    name="model", 
                    src="@../geom/loader",
                    useNormals="1",
                    srgbTexturing="1"
                    )
    # else:
    #     visual.addObject("OglModel", 
    #                       name="model", 
    #                       src="@../geom/loader", 
    #                       useNormals="1", 
    #                       alphaBlend="1",
    #                       texturename="/home/ryanm/Shoulder_texture.png")
        # Applies the texture to all sub-meshes, so the texture is being applied multiple times in different transformation, making it look like a big mess.
    # Map the (already-driven) vertex positions into the OglModel
    visual.addObject("IdentityMapping", input="@../geom/vertices", output="@.")

    return





def addCollision(node, index, filename):
    # Make a container node for this part's visuals
    collis = node.addChild(f'Collis{index}')

    # --- geometry branch (holds the state for vertices) ---
    geom = collis.addChild('geom')
    geom.addObject("AssimpLoader", name="loader", filename=filename, scale="1")
    geom.addObject("MeshTopology", src="@loader")  

    rx, ry, rz = collisionRotations[partNames[index]]
    tx, ty, tz = collisionTranslations[partNames[index]]

    geom.addObject("TransformEngine", name="tf",
                input_position="@loader.position",
                rotation=f"{rx} {ry} {rz}",
                translation=f"{tx} {ty} {tz}")

    geom.addObject("MechanicalObject", name="vertices", template="Vec3d",
                position="@tf.output_position",      # <- not @tf.position
                rest_position="@tf.output_position")  # <- same here

    # Drive these vertices with the k-th rigid pose from /rigid/dofs
    geom.addObject("RigidMapping", input="@../../../../../dofs", output="@vertices",
                   index=str(index), globalToLocalCoords="false")

    # actual collision models
    geom.addObject("TriangleCollisionModel", moving="1", simulated="1")
    geom.addObject("LineCollisionModel",     moving="1", simulated="1")
    geom.addObject("PointCollisionModel",    moving="1", simulated="1")

    return


def addCenter(node, name,
              parentIndex, childIndex,
              posOnParent, posOnChild,
              articulationProcess,
              isTranslation, isRotation, axis,
              articulationIndex):

    center = node.addChild(name)
    center.addObject('ArticulationCenter', parentIndex=parentIndex, childIndex=childIndex, posOnParent=posOnParent, posOnChild=posOnChild, articulationProcess=articulationProcess)

    articulation = center.addChild('Articulation')
    articulation.addObject('Articulation', translation=isTranslation, rotation=isRotation, rotationAxis=axis, articulationIndex=articulationIndex)

    return center


def addPart(node, name, index, visuFilename, collisFilename, texfilename=None):

    part = node.addChild(name)
    visu = part.addChild('visual')
    collis = part.addChild('collision')
    addVisu(visu, index, visuFilename, texfilename)
    if collisFilename is not None:
        addCollision(collis, index, collisFilename)

    return part

class Robot:

    def __init__(self, node):
        self.node=node


    def addRobot(self, name='Robot'):

        # Robot node
        robotNode = self.node.addChild(name)

        # Then in your Robot.addRobot():

        # Positions of parts

        cfg = {
            "base_link-base_link_inertia":  0.0,
            "shoulder_pan_joint":           0.0,
            "shoulder_lift_joint":          0.0,
            "elbow_joint":                  0.0,
            "wrist_1_joint":                0.0,
            "wrist_2_joint":                0.0,
            "wrist_3_joint":                0.0,
        }

        A = np.array([  [1,0,0],
                        [0,0,1],
                        [0,1,0]], dtype=float)   # rotate -90° about X
        ALIGN4 = np.eye(4); ALIGN4[:3,:3] = A
        fk = robot.link_fk(cfg=cfg)   # dict for ALL links

        positions = []
        for name in partNames:
            link = L[name]
            # Compose link FK with visual origin in URDF frame:
            T_wv_u = fk[link] @ link.visuals[0].origin
            T_wv_s = ALIGN4 @ T_wv_u @ALIGN4.T
            R_s = T_wv_s[:3,:3]; t_s = T_wv_s[:3,3]
            qx, qy, qz, qw = R.from_matrix(R_s).as_quat()
            positions.append([t_s[0], t_s[1], t_s[2], qx, qy, qz, qw])


        # You can change the joint angles here
        initAngles = [0, 0, 0, 0, 0, 0, 0] # add for more dofs to control.

        robotNode.addData('angles', initAngles, None, 'angle of articulations in radian', '', 'vector<float>')
        robotNode.addObject('EulerImplicitSolver')
        #robotNode.addObject('SparseLDLSolver')
        #robotNode.addObject('GenericConstraintCorrection')

        # Articulations node
        articulations = robotNode.addChild('Articulations')
        articulations.addObject('MechanicalObject', name='dofs', rest_position=robotNode.getData('angles').getLinkPath(), template='Vec1', position=initAngles)
        articulations.addObject('ArticulatedHierarchyContainer')
        articulations.addObject('SparseLDLSolver', name='als')                       # local linear solver for joints
        articulations.addObject('UniformMass', template='Vec1d', vertexMass='1 1 1 1 1 1')  # 6 joints
        articulations.addObject('RestShapeSpringsForceField', stiffness=2000, points=list(range(len(initAngles))))
        articulations.addObject('LinearSolverConstraintCorrection', linearSolver='@als')


        # Rigid
        rigid = articulations.addChild('Rigid')
        rigid.addObject('MechanicalObject', name='dofs', template='Rigid3d', showObject=False, showObjectScale=1, position=positions[:])
        rigid.addObject('ArticulatedSystemMapping', input1=articulations.dofs.getLinkPath(), output=rigid.dofs.getLinkPath())
        #masses = [float(L[name].inertial.mass) for name in partNames]
        #rigid.addObject('UniformMass', template='Rigid3d', totalMass=sum(masses))
        #rigid.addObject('UncoupledConstraintCorrection')


        # Visu
        parts = rigid.addChild('Parts')
        addPart(parts, 'Base' , 0, visual_basePath, collision_basePath)
        addPart(parts, 'Part1', 1, visual_shoulderPath, collision_shoulderPath, "/home/ryanm/Shoulder_texture.png")
        addPart(parts, 'Part2', 2, visual_upperarmPath, collision_upperarmPath)
        addPart(parts, 'Part3', 3, visual_forearmPath, collision_forearmPath)
        addPart(parts, 'Part4', 4, visual_wrist1Path, collision_wrist1Path)
        addPart(parts, 'Part5', 5, visual_wrist2Path, collision_wrist2Path)
        addPart(parts, 'Part6', 6, visual_wrist3Path, collision_wrist3Path)
        addPart(parts, 'Part7', 7, rg2HandVisualPath, rg2HandCollisPath)



        # Center of articulations
        centers = articulations.addChild('ArticulationsCenters')
        addCenter(centers, 'CenterBase' , 0, 1, [0, 0.1625, 0], [0, 0, 0], 0, 0, 1, [0, 1, 0], 0)
        addCenter(centers, 'CenterPart1', 1, 2, [0, 0, 0.138], [0, 0, 0], 0, 0, 1, [0, 0, 1], 1)
        addCenter(centers, 'CenterPart2', 2, 3, [0, 0.425, -0.131], [0, 0, 0], 0, 0, 1, [0, 0, 1], 2)
        addCenter(centers, 'CenterPart3', 3, 4, [0, 0.3922, 0], [0, 0, 0], 0, 0, 1, [0, 0, 1], 3)
        addCenter(centers, 'CenterPart4', 4, 5, [0, 0, 0.1333-0.007], [0, 0, 0], 0, 0, 1, [0, 1, 0], 4)
        addCenter(centers, 'CenterPart5', 5, 6, [0, 0.0997, 0], [0, 0, 0], 0, 0, 1, [0, 0, 1], 5)
        addCenter(centers, 'CenterPart5', 6, 7, [0, 0.425, 0], [0, 0, 0], 0, 0, 1, [0, 0, 1], 6) # hand to wist3

        #0 0 -0.0989
        return robotNode


# Test/example scene
def createScene(rootNode):

    from header import addHeader
    # from robotGUI import RobotGUI  # Uncomment this if you want to use the GUI

    addHeader(rootNode)


    # Robot
    robot = Robot(rootNode).addRobot()
    robot.addObject(RobotGUI(robot=robot, articulations_mo=robot.Articulations.getObject('dofs')))  # Uncomment this if you want to use the GUI


    Sim.initTextures(rootNode)

    return