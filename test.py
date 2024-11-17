import pybullet as p
import time
import pybullet_data
import numpy as np
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0)
udf_path = "/home/sman/Work/CMU/Research/LEGO-pybullet/my-robot/robot.urdf"
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(udf_path, startPos, startOrientation)

maxForce = 0
mode = p.POSITION_CONTROL

timestep = 1./240.
A = 30 * np.pi / 180
omega = 1.7 * 2 * np.pi
act_offset = -0.25
waitTime = 5

# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range(10000):
    currTime = i*timestep
    p.stepSimulation()
    # if currTime > waitTime:
    #     act_pos = A*np.sin(omega*(currTime - waitTime)) + act_offset
    #     p.setJointMotorControl2(
    #         robotId, 0, controlMode=mode, targetPosition=act_pos)
    # else:
    #     p.setJointMotorControl2(
    #         robotId, 0, controlMode=mode, targetPosition=act_offset)
    time.sleep(timestep)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos, cubeOrn)
p.disconnect()
