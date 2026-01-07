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
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField] 
    rootNode.addObject('RequiredPlugin', name='MultiThreading') # Needed to use components [ParallelBVHNarrowPhase,ParallelBruteForceBroadPhase]

    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1.0e-9)

    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('ParallelBruteForceBroadPhase')
    rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('NewProximityIntersection', alarmDistance='0.0015', contactDistance='0.0005')
    rootNode.addObject('CollisionResponse', name='ContactManager', response='FrictionContactConstraint', responseParams='mu=0.25')


    rootNode.addObject("VisualStyle", displayFlags="showVisualModels hideMappings")
    rootNode.addObject('BackgroundSetting', color=[1., 1., 1., 1.])
    rootNode.findData('dt').value=0.01
    rootNode.gravity = [0,-9.810,0]

    # rootNode.addObject('ContactListener', name='contacts', listening='1')  # logs contacts

# ----------------------------------------------



def createScene(rootNode):

    addHeader(rootNode)