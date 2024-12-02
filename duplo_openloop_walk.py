import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
urdf_path = "/duplo_new/robot.urdf"
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1.2]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)

for i in range(p.getNumJoints(robotId)):
    if p.getJointInfo(robotId, i)[1] == b'hip':
        hipJointId = i
        print(f"hip joint id: {hipJointId}\n")

mode = p.POSITION_CONTROL

timestep = 1./240.
Amp = 30 * np.pi / 180
omega = np.sqrt(9.81/1.15) * 1
print(f"frequency: {omega/(2*np.pi)}")
act_offset = 0
waitTime = 5
runTime = 25
storePeriod = 3

traj = []
joint_des = []
joint_real = []
init_pos = 0

for i in range(round(runTime/timestep)):
    currTime = i*timestep
    state_vec = p.getLinkState(robotId, 0, computeLinkVelocity=1)
    com_x, com_y, com_z = state_vec[0]

    act_pos = Amp*np.sin(omega*(currTime - waitTime)
                         ) if currTime > waitTime else init_pos
    joint_pos = p.getJointState(robotId, hipJointId)[0]

    joint_real.append((currTime, joint_pos))
    joint_des.append((currTime, act_pos))
    traj.append((currTime, com_x, com_y, com_z))
    # wait a bit before starting the actuation

    p.setJointMotorControl2(robotId,
                            hipJointId,
                            controlMode=mode,
                            targetPosition=act_pos)

    p.stepSimulation()

    # every storePeriod seconds, store trajectory and joint data
    if currTime % storePeriod < timestep:
        np.savetxt("./traj_data/duplo_joint.csv", joint_real, delimiter=",")
        np.savetxt("./traj_data/duplo_joint_des.csv", joint_des, delimiter=",")
        np.savetxt("./traj_data/duplo_traj.csv", traj, delimiter=",")

    time.sleep(timestep)

p.disconnect()

# dave trajectory as file
