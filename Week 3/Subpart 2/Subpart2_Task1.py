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

def Inverse_kinematics(target):

    x = target[0]
    y = target[1]
    z = target[2]

    argument = (y**2 + z**2 - l1**2 - l2**2)/(2*l1*l2)

    angle_2 = -math.acos(argument)

    if y == 0 and z == 0:
        second_tan_arg = float("-inf")
    else:
        second_tan_arg = (l2*math.sin(angle_2))/(l1 + l2*math.cos(angle_2))

    if y == 0 and z >= 0:
        angle_1 = math.atan(float('inf')) + math.atan(second_tan_arg)

    elif y == 0 and z < 0:
        angle_1 = math.atan(float('-inf')) + math.atan(second_tan_arg)

    else:
        angle_1 = math.atan(z/y) + math.atan(second_tan_arg)

    #angle_1 = y / l1 + l1*math.cos(angle_2)
    #angle_1 = -(math.pi/2 - angle_1)
    return angle_1, angle_2

z = 0
y = 0

while(True):

        if z <= 1 and y >= 0:
            initial_cordinate = [0,0,0]
            y = z
            z += 0.001
            color = [1, 0, 0]
        elif 1 < z <= 1.2 and y > 0.001:
            initial_cordinate = [0,1,1]
            y = 2 - z
            z += 0.001
            color = [0, 1, 0]
        # elif 1 < z < 2.1 and y <= 0.001:
        #         #     initial_cordinate = [0,0,2]
        #         #     y = z-2
        #         #     z += -0.001
        #         #     color = [0, 0, 1]
        #         # elif 0 < z <= 1 and y <= 0:
        #         #     initial_cordinate = [0,-1,1]
        #         #     y = -z
        #         #     z += -0.001
        #         #     color = [1, 1, 0]

        angle_1, angle_2 = Inverse_kinematics([0, y, z])
        cordinate = Forward_kinematics(angle_1, angle_2)

        x1 = cordinate[0]
        y1 = cordinate[1]
        z1 = cordinate[2]
        print(cordinate)
        print(0,y,z)
        p.setJointMotorControl2(bodyIndex=robot,
                                jointIndex=0,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_1,
                                force=500)

        p.setJointMotorControl2(bodyIndex=robot,
                                jointIndex=1,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=angle_2,
                                force=500)

        p.addUserDebugLine(initial_cordinate, cordinate, color, 2)

        p.stepSimulation()

