import os


def _positive_environment_float(name, default):
    value = float(os.environ.get(name, default))
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


# 10 ms halves the number of physics solves per simulated second compared with
# the previous 5 ms scene while remaining conservative for the implicit cable
# solver. Both values can be tuned at launch without editing the demonstration.
SIMULATION_TIMESTEP = _positive_environment_float("SQUASHSIM_DT", "0.01")
CONTACT_FRICTION = _positive_environment_float(
    "SQUASHSIM_FRICTION", "0.8"
)
CONFIGURED_SOFA_VERSION = os.environ.get(
    "SQUASHSIM_SOFA_VERSION", "25.06.00"
)


def addHeader(rootNode):

    rootNode.addObject("RequiredPlugin", name="Sofa.Component.StateContainer")              # MechanicalObject
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Mass")                        # UniformMass
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Mapping.NonLinear")           # RigidMapping
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Mapping.Linear")              # IdentityMapping
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Topology.Container.Constant") # MeshTopology
    # Collision
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.AnimationLoop")
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Collision.Detection.Algorithm")     # BruteForceBroadPhase, BVHNarrowPhase, CollisionPipeline
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Collision.Detection.Intersection")  # NewProximityIntersection
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Collision.Geometry")                # TriangleCollisionModel
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Collision.Response.Contact")   

    rootNode.addObject("RequiredPlugin", name="Sofa.GL.Component.Rendering3D")              # OglModel
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Engine.Transform")            # TransformEngine
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.Visual")                      # VisualStyle
    rootNode.addObject("RequiredPlugin", name="ArticulatedSystemPlugin")
    rootNode.addObject("RequiredPlugin", name="SofaAssimp")                                 # load DAE Mesh
    rootNode.addObject("RequiredPlugin", name="Sofa.Component.IO.Mesh")                     # load stl Mesh
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic') # Needed to use components [TriangleSetTopologyContainer] 

    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [GenericConstraintCorrection]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [TetrahedronFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField] 
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Mapping') # Needed to use components [Tetra2TriangleTopologicalMapping]
    rootNode.addObject('RequiredPlugin', name='MultiThreading') # Needed to use components [ParallelBVHNarrowPhase,ParallelBruteForceBroadPhase]

    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1.0e-9)

    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('ParallelBruteForceBroadPhase')
    rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('NewProximityIntersection', alarmDistance='0.003', contactDistance='0.001')
    rootNode.addObject(
        'CollisionResponse',
        name='ContactManager',
        response='FrictionContactConstraint',
        responseParams=f'mu={CONTACT_FRICTION:.6g}',
    )


    rootNode.addObject("VisualStyle", displayFlags="showVisualModels hideMappings")
    rootNode.addObject('BackgroundSetting', color=[1., 1., 1., 1.])
    rootNode.findData('dt').value = SIMULATION_TIMESTEP
    rootNode.gravity = [0,-9.810,0]

    print(
        "[squashSim] simulation: "
        f"dt={SIMULATION_TIMESTEP:.4f} s "
        f"({1.0 / SIMULATION_TIMESTEP:.1f} Hz nominal), "
        f"contact friction mu={CONTACT_FRICTION:.3g}"
    )

    # rootNode.addObject('ContactListener', name='contacts', listening='1')  # logs contacts

# ----------------------------------------------



def createScene(rootNode):

    addHeader(rootNode)
