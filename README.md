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

Press **Animate**, then use the seven sliders or an Xbox-compatible controller.
The controller support is optional and imports Pygame only when available:

```bash
python -m pip install pygame
```

| Slider | Command |
| --- | --- |
| J1–J6 | UR5 revolute joints, using the URDF limits |
| RG2 | Both simplified finger joints, `0` closed to `1.18` open |

The robot's internal `angles` data has eight entries:

```text
[J1, J2, J3, J4, J5, J6, RG2-left, RG2-right]
```

The two final values are kept equal by the GUI.

### Xbox controls

| Control | Action |
| --- | --- |
| Left stick left/right | Select J1–J6 or RG2 (one selection per stick flick) |
| Left trigger | Decrease the selected joint target |
| Right trigger | Increase the selected joint target |
| LB / RB | Reduce / increase joint-control sensitivity |
| Right stick | Orbit the SOFA camera around the robot and object |
| A | Start recording; press A again to finish and export |

Joint and camera movement are integrated using elapsed wall time, so controller
sensitivity no longer changes with SOFA's achieved frame rate. The physics
timestep is now `0.01 s` (100 Hz nominal) to reduce the previous slow-motion
effect while retaining an implicit cable solver and constraint contact. Both
settings are adjustable without editing code:

```bash
SQUASHSIM_DT=0.01 SQUASHSIM_FRICTION=0.8 \
  $RUN_SOFA -l SofaPython3 -l SofaAssimp -l ArticulatedSystemPlugin robot.py
```

The default friction coefficient is `0.8`, rather than `0.25`, to represent a
rubbery RG2/cable surface and make a pinch grasp less prone to slipping. This is
currently the scene-wide Coulomb coefficient, so it also increases cable-floor
friction. That is intentional for this narrow demonstrator.

## Episode recording and evidence

Press **A** once immediately before the manipulation and once after the cable is
lifted. Each completed recording is written to a unique directory beneath
`recordings/` (or `$SQUASHSIM_RECORDING_DIR`) and contains:

| File | Contents |
| --- | --- |
| `episode.npz` | Shape-preserving arrays for every captured channel |
| `samples.csv` | Flat table including every deformable-object node position |
| `metadata.json` | Configuration, schema, timing and repository/source identity |
| `configuration.csv` | Small appendix-ready configuration table |
| `recorded_channels.csv` | Small appendix-ready channel-summary table |
| `height_vs_time.svg` | End-effector, object-centre and object-point height |
| `contact_activity.svg` | RG2/object and floor/object contact activity |
| `joint_positions.svg` | Optional joint-position evidence plot |
| `evidence_summary.md` | One-page index containing the tables and plots |

Every row is sampled synchronously at the end of a SOFA animation step. The raw
channels include simulation and wall timestamps, solved joint position and
velocity, commanded joint targets and controller action, every cable frame's
XYZ position, end-effector position, and contact counts/state/events. SOFA's
Python `ContactListener` does not expose solved contact force, so contact count
is used for the interaction plot.

`metadata.json` also reports the measured real-time factor (simulated seconds
per wall-clock second). A value near `1.0` confirms real-time execution; if the
machine still cannot keep up, the evidence makes that explicit rather than
guessing from the viewport.

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

All robot triangle models share collision group `1`, preventing unwanted robot
self-collision while leaving them available to collide with the floor and the
deformable object in their separate groups.

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
