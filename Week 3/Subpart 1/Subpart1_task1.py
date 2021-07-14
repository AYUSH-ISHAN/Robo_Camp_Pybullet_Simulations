import pybullet as p
import pybullet_data
import cv2
import numpy as np

p.connect(p.GUI)  # or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# p.loadURDF("4_arm_robot.urdf", [2,0,0])
# p.loadURDF("4_arm_robot.urdf", [0,5,0])
cube = p.loadURDF("cube.urdf", [2,0,0.5])
p.changeVisualShape(cube, -1, rgbaColor=[0, 1, 0, 1])

p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
    print(p.getJointInfo(car, joint))

link_indices = [2,4]

width = 512
height = 512
fov = 60
aspect = width / height
near = 0.02
far = 5


###################     CAMERA DESIGNING  ::::::::::::::::

# def camera_location(location):
#     location = list(location)
#     location[2] = location[2] + 1
#     location = tuple(location)
#     return location
#
# def camera_focus():
#     link_indices_camera = [0]
#     front_right_wheel_state = p.getLinkStates(car, link_indices_camera)
#    #print(front_right_wheel_state)
#     front_right_wheel_position = list(front_right_wheel_state[0][0])
#     link_indices_camera = [6]
#     rear_right_wheel_state = p.getLinkStates(car, link_indices_camera)
#     #print(rear_right_wheel_state)
#     rear_right_wheel_position = list(rear_right_wheel_state[0][0])    # doubtful line
#
#     orientation = [front_right_wheel_position[i] - rear_right_wheel_position[i] + 1 for i in range(3)]
#     orientation = tuple(orientation)
#     return orientation

targetVel = 3
maxForce = 300
Stop_Vel = 0
# p.applyExternalForce(car,[100,0,0],)

maxForces = [maxForce for i in range(4)]

iter = 0
while (1):

    rKey = ord('r')
    aKey = ord('a')
    cKey = ord('c')
    keys = p.getKeyboardEvents()

    for k, v in keys.items():

        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = 3
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -3
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVelocities = [2, 4, 2, 4]

            p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities,
                                        forces=maxForces)
            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVelocities = [0, 0, 0, 0]
            p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities,
                                        forces=maxForces)

            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVelocities = [4, 2, 4, 2]

            p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities,
                                        forces=maxForces)
            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVelocities = [0, 0, 0, 0]
            p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities,
                                        forces=maxForces)

            p.stepSimulation()

        if rKey in keys and (keys[rKey] & p.KEY_IS_DOWN):
            print("The world is going round and round ... !!")
            targetVelocities = [2, -2, 2, -2]
            p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities,
                                        forces=maxForces)

            p.stepSimulation()

        if rKey in keys and (keys[rKey] & p.KEY_WAS_RELEASED):
            targetVelocities = [0, 0, 0, 0]
            p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities,
                                        forces=maxForces)
            p.stepSimulation()

        if aKey in keys and (keys[aKey] & p.KEY_WAS_RELEASED):
            targetVel += 1
            print("target velocity is : ", targetVel)
            p.stepSimulation()

        if cKey in keys and (keys[cKey] & p.KEY_WAS_RELEASED):
            front_wheel = p.getLinkState(car, 2)
            rear_wheel = p.getLinkState(car, 4)
            bumper = p.getLinkState(car, 8)
            print(front_wheel)
            # one more way can be to use the above function listed...
            view_matrix = p.computeViewMatrix(bumper[0],
                                              [bumper[0][0] + 10 * (front_wheel[0][0] - rear_wheel[0][0]), bumper[0][1] + 10 * (front_wheel[0][1] - rear_wheel[0][1]),
                                               bumper[0][2] + 10 * (front_wheel[0][2] - rear_wheel[0][2])], [0, 0, 1])

            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

            images = p.getCameraImage(width,
                                      height,
                                      view_matrix,
                                      projection_matrix,
                                      shadow=True,
                                      renderer=p.ER_BULLET_HARDWARE_OPENGL)

            rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
            #rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
            # print("images = ", images)
            # print("rgb_opengl = ", rgb_opengl)
            cv2.imshow('rgb', rgb_opengl)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            p.stepSimulation()

        iter += 1
        if iter % 100 == 0:
            print(p.getLinkStates(car, link_indices)[0])  # deals with link index = 2.

p.getContactPoints(car)
p.disconnect()