import pybullet as p
import time
import pybullet_data


physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeID = p.loadURDF("plane.urdf")
start_pos_bot1 = [2, 2, 1]
start_pos_bot2 = [0, 0, 1]
# start_orient = p.getQuaternionFromEuler([0,0,0])
bot_id = p.loadURDF("sample.urdf", start_pos_bot1)
bot_id2 = p.loadURDF("dabba.urdf", start_pos_bot2)
value = 0

###  function to get gravity...
def value(i):
    if i > 9.8:
        gravity = 0
        return gravity
    else:
        return i

i = 0
while True:
    i += 0.1
    p.stepSimulation()
    time.sleep(1. / 240.)     ### this factor introduces lag between the readings..
    gravity = value(i)
    i = gravity
    print(gravity)
    p.setGravity(gravity, gravity, 0)

# pos1, ori1 = p.getBasePositionAndOrientation(bot_id)
# pos2, ori2 = p.getBasePositionAndOrientation(bot_id2)
#
# print(pos1, ori1)
# print(pos2, ori2)
# print(bot_id2, bot_id)  #    bot id dor second and first : 2 1
p.disconnect()
