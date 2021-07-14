import pybullet as p
import pybullet_data

# (2, b'front_left_wheel',
# (3, b'front_right_wheel',
# (4, b'rear_left_wheel',
# (5, b'rear_right_wheel'.

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
carpos = [0,0,0.1]

car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)

for joints in range(numJoints):
    print(p.getJointInfo(car, joints))

targetVel = 3
maxForce = 300
Stop_Vel = 0
# p.applyExternalForce(car,[100,0,0],)

maxForces = [maxForce for i in range(4)]

while(1):

    rKey = ord('r')
    aKey = ord('a')
    keys = p.getKeyboardEvents()

    # print(p.KEY_IS_DOWN)
    # print(p.KEY_WAS_RELEASED)
    # print(p.KEY_WAS_TRIGGERED)

    for k,v in keys.items():

        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
                for joints in range(2,6):
                    p.setJointMotorControl2(car, joints, p.VELOCITY_CONTROL, targetVelocity = targetVel, force = maxForce)

                p.stepSimulation()

        if (k ==  p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
                for joints in range(2,6):
                    p.setJointMotorControl2(car, joints, p.VELOCITY_CONTROL, targetVelocity = Stop_Vel, force = maxForce)

                p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
                for joints in range(2, 6):
                    p.setJointMotorControl2(car, joints, p.VELOCITY_CONTROL, targetVelocity= -targetVel, force=maxForce)

                p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
                for joints in range(2, 6):
                    p.setJointMotorControl2(car, joints, p.VELOCITY_CONTROL, targetVelocity=Stop_Vel, force=maxForce)

                p.stepSimulation()

        if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):

                targetVelocities = [2, 4, 2, 4]

                p.setJointMotorControlArray(car, [2,3,4,5], p.VELOCITY_CONTROL, targetVelocities = targetVelocities, forces = maxForces)
                p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):

                targetVelocities = [0, 0, 0, 0]
                p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities = targetVelocities, forces=maxForces)

                p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
                targetVelocities = [4, 2, 4, 2]

                p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities = targetVelocities,
                                            forces=maxForces)
                p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVelocities = [0, 0, 0, 0]
                p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities = targetVelocities,
                                            forces=maxForces)

                p.stepSimulation()

        if rKey in keys and (keys[rKey] & p.KEY_IS_DOWN):

                print("The world is going round and round ... !!")
                targetVelocities = [2, -2, 2, -2]
                p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities, forces=maxForces)

                p.stepSimulation()

        if rKey in keys and (keys[rKey] & p.KEY_WAS_RELEASED):

                targetVelocities = [0, 0, 0, 0]
                p.setJointMotorControlArray(car, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities, forces=maxForces)
                p.stepSimulation()

        if aKey in keys and (keys[aKey] & p.KEY_WAS_RELEASED):
                targetVel += 1
                print("target velocity is : ",targetVel)
                p.stepSimulation()

p.getContactPoints(car)
p.disconnect()









