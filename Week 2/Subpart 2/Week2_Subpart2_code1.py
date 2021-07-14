import pybullet as p
import pybullet_data
import time

print("Enter 1 for String type input : ")
print("Enter 2 for Number type input : ")

n = int(input("Enter the number : "))
command_string = None
command_number = None

if n == 1:
    print("Enter either VELOCITY_CONTROL or TORQUE_CONTROL : ")
    command_string= input("Enter your command in BLOCK LETTERS : ")

if n == 2:
    print("Enter 1 for VELOCITY CONTROL : ")
    print("Enter 2 for TORQUE CONTROL : ")
    command_number = int(input("enter the number : "))

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
# these are the pre required conditions for the task.
ramp = p.loadURDF("wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp, -1, lateralFriction=0.5)

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])

print("\n Number of JOINTS in HUSKY-the car : ", p.getNumJoints(husky))  # the output is 10
print("\n The Joint information of HUSKY : \n")
for i in range(p.getNumJoints(husky)):
    print(p.getJointInfo(husky, i))
print("\n Getting the joints state : \n")
for i in range(p.getNumJoints(husky)):
    print(p.getJointState(husky, i))
print("\n")
print(p.getBaseVelocity(husky))

link_indices = [i for i in range(p.getNumJoints(husky))]


def Torque_control():

    optimal_torque_value = -270

    ## set to -270 for the back_flip
    ## set to -250 for just climbing
    ## set to -260 for a good boy type climb.

    '''
    this function should have the setJointMotorControl in TORQUE_CONTROL configuration
    with forc = optimal_force_value
    '''

    optimal_velocity_value = -50
    optimal_torques_value = [optimal_torque_value for i in range(4)]
    # the below code is for rotating the car..
    # optimal_torques_value = [-optimal_torque_value, optimal_torque_value, -optimal_torque_value, optimal_torque_value]
    # print(optimal_torques_value)
    targetvelocity = optimal_velocity_value
    targetVelocities = [targetvelocity for i in range(4)]
    p.setJointMotorControlArray(bodyUniqueId=husky,
                                jointIndices=[2, 3, 4, 5],
                                controlMode=p.TORQUE_CONTROL,
                                targetVelocities = targetVelocities,
                                forces=optimal_torques_value)


def Velocity_control():
    maxForce = 20

    optimal_velocity_value = -50
    forces = [maxForce for j in range(4)]
    targetvelocity = optimal_velocity_value
    targetVelocities = [targetvelocity for i in range(4)]

    p.setJointMotorControlArray(bodyUniqueId=husky,
                                jointIndices=[2, 3, 4, 5],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocities=targetVelocities,
                                forces=forces)


def checking_control(input):
    if input == 1:
        Velocity_control()
    elif input == 2:
        Torque_control()



iteration = 0
while (1):
    time.sleep(.01)
    '''
    1.Here call either the Torque_control function or Velocity_control 
      function according to the initial user choice and apply the optimal velocity/Torque
      to the motors that you have found by experimentation.
    2.print base state and velocity 100 iteration steps once.
    '''
    p.stepSimulation()
    p.setGravity(0, 0, -10)

    if command_string == "VELOCITY_CONTROL" or command_number == 1:
        Velocity_control()

    elif command_string == "TORQUE_CONTROL" or command_number == 2:
        Torque_control()

    else:
        print("INVALID INPUT !! RUN THE PROGRAM AGAIN !!")

    iteration += 1

    if iteration % 100 == 0:
        if command_string == "VELOCITY_CONTROL" or command_number == 1:
            print("Welcome to the reign of VELOCITY !!")

        elif command_string == "TORQUE_CONTROL" or command_number == 2:
            print("Welcome to the reign of TORQUE !!")

        else:
            print("INVALID INPUT !! RUN THE PROGRAM AGAIN !!")

        print("The base link velocity : ", p.getBaseVelocity(husky))
        print("The base link state : ",p.getLinkStates(husky,link_indices))   ### search about getLinkStates...

p.disconnect()
