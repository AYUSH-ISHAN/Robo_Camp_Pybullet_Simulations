import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p_ID = p.loadURDF("plane.urdf")
sphere_orien = p.getQuaternionFromEuler([0,0,0])

########   here, we started the fbonacci series from 1 ...

def fibonacci(i):
    # here i is the ith term of the fibonacci to be reproduced... indexing from 1..
    first, second = 0, 1
    next = 0
    if i == 1:
         return first
    if i == 2:
        return second
    if i >= 3:
        for j in range(2,i):
            next = first + second
            first = second
            second = next
            #print(next)
    return next

def load_spheres(wave_counter):
    res = fibonacci(wave_counter)
    print(res)
    for i in range(1, res + 1):
    # print("minor loop")
        sphere_id = p.loadURDF("sample.urdf", [i-1, 0, 1])

wave_counter = 1

print(wave_counter)

while True:
    load_spheres(wave_counter+1)
    for j in range(10000):  # time step of 10000 to each wave..  ## reduce it to 100 for heavy rainfall
        p.stepSimulation()
        time.sleep(1. / 240.)


    wave_counter += 1


p.disconnect()