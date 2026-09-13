# squashSim

SOFA articulated-system scene for a UR5 with an OnRobot RG2 gripper.

The implementation keeps the original squashSim structure—`robot.py`, an
`ArticulatedHierarchyContainer`, an `ArticulatedSystemMapping`, and Tk slider
control—but now derives the complete robot from Andrej Orsula's standardized
UR5+RG2 description:

- six UR5 revolute joints;
- the fixed wrist-to-RG2 mount;
- two simplified RG2 finger joints;
- UR5, hand, and finger meshes;
- joint centres, axes, limits, and fixed transforms read from the URDF.

The two finger joints remain separate internally because SOFA requires one
input DOF per articulation centre. One `RG2` slider mirrors the commanded value
to both joints.

## Setup

Clone the reference model into the default location:

```bash
git clone https://github.com/AndrejOrsula/ur5_rg2_ign.git ~/ur5_rg2_ign
```

If it is elsewhere, set:

```bash
export SQUASHSIM_UR5_RG2_ROOT=/absolute/path/to/ur5_rg2_ign
```

The expected files are:

```text
$SQUASHSIM_UR5_RG2_ROOT/urdf/ur5_rg2.urdf
$SQUASHSIM_UR5_RG2_ROOT/ur5_rg2/meshes/visual/...
$SQUASHSIM_UR5_RG2_ROOT/ur5_rg2/meshes/collision/...
```

## Run

From the squashSim repository:

```bash
$RUN_SOFA -l SofaPython3 -l SofaAssimp -l ArticulatedSystemPlugin robot.py \
  2>&1 | tee ur5-rg2.log
```

Press **Animate**, then use the seven sliders:

| Slider | Command |
| --- | --- |
| J1–J6 | UR5 revolute joints, using the URDF limits |
| RG2 | Both simplified finger joints, `0` closed to `1.18` open |

The robot's internal `angles` data has eight entries:

```text
[J1, J2, J3, J4, J5, J6, RG2-left, RG2-right]
```

The two final values are kept equal by the GUI.

The scene also creates a 2.4 m square visible floor. Its top is aligned with
the underside of the reference UR5 base at `y = -0.003 m`, and rigid-body index
zero is held by a `FixedProjectiveConstraint`, representing the base being
bolted to the floor.

## Geometry and collision

The scene keeps rendering and collision geometry separate:

- the reference repository's higher-detail DAE meshes are used only by
  `OglModel` for rendering;
- the reference repository's smaller STL meshes are used only by
  `TriangleCollisionModel` for collision detection.

The RG2 DAE files contain important scene-node transforms and scaling. A small
COLLADA reader in `robot.py` bakes those transforms before giving the vertices
to SOFA. This avoids using the raw, incorrectly scaled RG2 vertices while
requiring no extra Python package. A right-handed transform then converts the
complete model from URDF Z-up coordinates into SOFA Y-up coordinates.

All robot triangle models and the floor share collision group `1`. This
prevents robot self-collision and suppresses the permanent contact pair where
the bolted base touches the floor. A future object assigned to another group
will still be able to collide with both the floor and robot.

The RG2 hand is a fixed part of the wrist rigid body. Its fixed URDF transform
is applied to the hand geometry. The finger centres and axes are transformed
through that same mount, so no manually tuned UR5 dimensions are present.

Reference model: [AndrejOrsula/ur5_rg2_ign](https://github.com/AndrejOrsula/ur5_rg2_ign)

## Static validation

The URDF-to-SOFA transform and scene-graph tests can run without SOFA itself:

```bash
SQUASHSIM_UR5_RG2_ROOT=~/ur5_rg2_ign \
python -m unittest -v tests.test_robot_model
```
