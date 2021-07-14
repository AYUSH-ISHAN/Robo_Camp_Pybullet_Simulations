import pybullet as p
import pybullet_data
import os
import math
import time

file_name = "2R_planar_robot.urdf"
p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 0.05], [0, 0, 0, 0.707])

p.setGravity(0, 0, -10)

l1 = 1
l2 = 1

def Forward_kinematics(angle_1, angle_2):

    y = l1*math.cos(angle_1) + l2*math.cos(angle_1 - angle_2)
    z = l1*math.sin(angle_1) + l2*math.sin(angle_1 - angle_2)
    return [0, y, z]

z = 0
y = 0

while(True):

        if z <= 1 and y >= 0:
            initial_cordinate = [0,0,0]
            y = z
            z += 0.001
            color = [1,0,0]
        elif 1 < z <= 2 and y > 0.001:
            initial_cordinate = [0,1,1]
            y = 2 - z
            z += 0.001
            color = [0,1,0]
        elif 1 < z < 2.1 and y <= 0.001:
            initial_cordinate = [0,0,2]
            y = z-2
            z += -0.001
            color = [0,0,1]
        elif 0 < z <= 1 and y <= 0:
            initial_cordinate = [0,-1,1]
            y = -z
            z += -0.001
            color = [1,1,0]
        joint_pos = p.calculateInverseKinematics(robot, 2, [0, y, z])
        #print(joint_pos)
        p.setJointMotorControl2(bodyIndex=robot,
                                jointIndex=0,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[0],
                                force=500)

        p.setJointMotorControl2(bodyIndex=robot,
                                jointIndex=1,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_pos[1],
                                force=500)
        #print(0, y, z)
        p.addUserDebugLine(initial_cordinate, [0, y, z], color, 2)
        p.stepSimulation()
