import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeID = p.loadURDF('plane.urdf')


##########  READ  ME ::::

#######   setting the collision to 0.5 unit higher the ground to see its effect ...  #######
######   Check the collision parameter in the urdf file...



start_orien = p.getQuaternionFromEuler([0,0,0])
robo = p.loadURDF('4_arm_robot.urdf', [0,0,-0.5], start_orien)

for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()


#            ixx="1.2"
#            iyy="5.5"
#            izz="6.0"
#            ixy="0.0"
#            ixz="-0.2"
#            iyz="-0.01"