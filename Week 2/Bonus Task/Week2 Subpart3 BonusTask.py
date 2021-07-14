import pybullet as p
import pybullet_data
import time

#################    WORKS nice with this combo...scatters after a ling time..

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.setGravity(0,0,-20)   # with and without setting the gravity, something nice happens...
planeID = p.loadURDF("plane.urdf")
# cube_pos = [0,0,1]
# cube_pos1 = [0,0.75,1]

parent1 = p.loadURDF("cube.urdf", [0,0,4.5])         # [0,0,2])
parent2 = p.loadURDF("cube.urdf", [0,0.1,4.5])
parent3 = p.loadURDF("cube.urdf", [0,0.2,4.5])
parent4 = p.loadURDF("cube.urdf", [0,0.3,4.5])
parent5 = p.loadURDF("cube.urdf", [0,0.4,4.5])
child1 = p.loadURDF("sample.urdf", [0,0,0.5])
child2 = p.loadURDF("sample.urdf", [0,0.11,0.5])                  ### 0.075 works little nice..
child3 = p.loadURDF("sample.urdf", [0,0.21,0.5])
child4 = p.loadURDF("sample.urdf", [0,0.31,0.5])
child5 = p.loadURDF("sample.urdf", [0,0.41,0.5])

p.createConstraint(parent1, -1, -1, -1, p.JOINT_FIXED, [0,0,0],[0,0,0], [0,0,4.5])  # by JOINT_POINT2POINT - the cube can rotate on its place..
p.createConstraint(parent2, -1, -1, -1, p.JOINT_FIXED, [0,0,0],[0,0,0], [0,0.1,4.5])  # by JOINT_POINT2POINT - the cube can rotate on its place..
p.createConstraint(parent3, -1, -1, -1, p.JOINT_FIXED, [0,0,0],[0,0,0], [0,0.2,4.5])  # by JOINT_POINT2POINT - the cube can rotate on its place..
p.createConstraint(parent4, -1, -1, -1, p.JOINT_FIXED, [0,0,0],[0,0,0], [0,0.3,4.5])  # by JOINT_POINT2POINT - the cube can rotate on its place..
p.createConstraint(parent5, -1, -1, -1, p.JOINT_FIXED, [0,0,0],[0,0,0], [0,0.4,4.5])  # by JOINT_POINT2POINT - the cube can rotate on its place..
p.createConstraint(parent1, -1, child1, -1, p.JOINT_POINT2POINT, [0,0,-1], [0,0,0], [0,0,4])  # [0,0,1.5]
p.createConstraint(parent2, -1, child2, -1, p.JOINT_POINT2POINT, [0,0,-1], [0,0,0], [0,0,4])
p.createConstraint(parent3, -1, child3, -1, p.JOINT_POINT2POINT, [0,0,-1], [0,0,0], [0,0,4])
p.createConstraint(parent4, -1, child4, -1, p.JOINT_POINT2POINT, [0,0,-1], [0,0,0], [0,0,4])
p.createConstraint(parent5, -1, child5, -1, p.JOINT_POINT2POINT, [0,0,-1], [0,0,0], [0,0,4])

# #print(p.getConstraintInfo(cubeId))   # #
### changing the last two co-ordinate will make the parent or child to move at that co-ordinate - if there is no constraint on the box.
### on applying teh above the constraint - fixes the box at 1 unit above and keeps the distance between the sphere and box fixed and = 1.
flag = True
while True:
    p.stepSimulation()
    time.sleep(1./240.)
    p.setGravity(0, 0, -32)

    if flag:
        for i in range(1000):
            print(i)
            p.applyExternalForce(child5, -1, [0.0, 30.0, 0.0], [0, 0, 0], p.LINK_FRAME)  # [0,25,0] - the ball goes in desired location.
    flag = False


p.disconnect()