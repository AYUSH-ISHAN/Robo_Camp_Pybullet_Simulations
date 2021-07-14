import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)  ## connect to server
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)  ## Flag to enable Deformable bodies

p.setGravity(0, 0, -10)

planeOrn = [0, 0, 0, 1]  # p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0, 0, -2], planeOrn)
#tex = p.loadTexture("uvmap.png")
torus1 = p.loadSoftBody("torus_only.obj",
                        simFileName="torus.vtk",
                        basePosition=[0, 0, 2],
                        baseOrientation=[1, 1.57, 0, 0],
                        scale=0.5,
                        mass=1,
                        useNeoHookean=1,
                        NeoHookeanMu=400,  ### 0.5, 4, 1, 400, 600, 0.001, 1, 0.5, 0.001
                        NeoHookeanLambda=600,
                        NeoHookeanDamping=0.01,
                        useSelfCollision=1,
                        frictionCoeff=0.5,
                        collisionMargin=0.001,
                        repulsionStiffness = 800)

p.changeVisualShape(torus1, -1, rgbaColor=[0, 255, 0, 1],  flags=0)

torus2 = p.loadSoftBody("torus_only.obj",
                        simFileName="torus.vtk",
                        basePosition=[0, 0, 1.36],
                        baseOrientation=[1, 0, 0, 0],
                        scale=0.5,
                        mass=1,
                        useNeoHookean=1,
                        NeoHookeanMu=400,  ### 0.5, 4, 1, 400, 600, 0.001, 1, 0.5, 0.001
                        NeoHookeanLambda=600,
                        NeoHookeanDamping=0.01,
                        useSelfCollision=1,
                        frictionCoeff=0.5,
                        collisionMargin=0.001,
                        repulsionStiffness = 800)

p.changeVisualShape(torus2, -1, rgbaColor=[255, 0, 0, 1], flags=0)

torus3 = p.loadSoftBody("torus_only.obj",
                        simFileName="torus.vtk",
                        basePosition=[0, 0, 2.65],
                        baseOrientation=[1, 0, 0, 0],
                        scale=0.5,
                        mass=1,
                        useNeoHookean=1,
                        NeoHookeanMu=400,  ### 0.5, 4, 1, 400, 600, 0.001, 1, 0.5, 0.001
                        NeoHookeanLambda=600,
                        NeoHookeanDamping=0.01,
                        useSelfCollision=1,
                        frictionCoeff=0.5,
                        collisionMargin=0.001,
                        repulsionStiffness = 800)

p.changeVisualShape(torus3, -1, rgbaColor=[0, 0, 255, 1], flags=0)

torus4 = p.loadSoftBody("torus_only.obj",
                        simFileName="torus.vtk",
                        basePosition=[0, 0, 3.27],
                        baseOrientation=[1,1.57, 0, 0],
                        scale=0.5,
                        mass=1,
                        useNeoHookean=1,
                        NeoHookeanMu=400,  ### 0.5, 4, 1, 400, 600, 0.001, 1, 0.5, 0.001
                        NeoHookeanLambda=600,
                        NeoHookeanDamping=0.01,
                        useSelfCollision=1,
                        frictionCoeff=0.5,
                        collisionMargin=0.001,
                        repulsionStiffness = 800)

p.changeVisualShape(torus4, -1, rgbaColor=[255, 0, 255, 1], flags=0)

p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)  ## Setting a limit for the resolution of
## voxel to increase performance and decrease accuracy


while p.isConnected():
    p.stepSimulation()  ## Run Run Run!!!

    p.setGravity(0, 0, -10)

#textureUniqueId=tex,