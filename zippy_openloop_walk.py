import pybullet as p
import time
import pybullet_data
import numpy as np

# physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
physicsClient = p.connect(p.DIRECT)  # or p.DIRECT for non-graphical version

p.resetDebugVisualizerCamera(cameraDistance=1.6,
                             cameraYaw=50,
                             cameraPitch=-28,
                             cameraTargetPosition=[-0.8845049142837524,
                                                   0.7213836908340454,
                                                   -0.6222991943359375])

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
urdf_path = "zippy/robot.urdf"
urdf_path = "zippy_newmass/robot.urdf"
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation)

for i in range(p.getNumJoints(robotId)):
    if p.getJointInfo(robotId, i)[1] == b'hip':
        hipJointId = i
        print(f"hip joint id: {hipJointId}\n")

mode = p.POSITION_CONTROL
# mode = p.VELOCITY_CONTROL

timestep = 1./240.
timestep = 1./3000.
p.setTimeStep(timestep)
pos_amp = 21 * np.pi / 180
# vel_amp = 5.81
omega = 8.3 * 2 * np.pi
omega = 28.7 * 2
hz = omega/(2*np.pi)
vel_amp = pos_amp * omega
print(f"frequency: {hz}")
act_offset = 0
waitTime = 3
runTime = 25
storePeriod = 3

traj = []
joint_des = []
joint_real = []
init_pos = 0
act_perc_offset = 0.5 / 100 * 0
# act_vel_perc = 10/100


for i in range(round(runTime/timestep)):
    currTime = i*timestep
    state_vec = p.getLinkState(robotId, 0, computeLinkVelocity=1)
    com_x, com_y, com_z = state_vec[0]

    # generate a square wave for the hip joint
    if currTime > waitTime:
        wave_val = np.sin(omega*(currTime - waitTime))
        act_pos = (pos_amp if wave_val > 0 else -pos_amp)
        act_vel = (vel_amp if wave_val > 0 else -vel_amp)

    else:
        act_pos = init_pos
        act_vel = 0

    joint_pos = p.getJointState(robotId, hipJointId)[0]

    joint_real.append((currTime, joint_pos))
    joint_des.append((currTime, act_pos))

    # only append every 0.01 seconds
    if currTime % 0.01 < timestep:
        traj.append((currTime, com_x, com_y, com_z))
    # wait a bit before starting the actuation

    p.stepSimulation()

    p.setJointMotorControl2(robotId,
                            hipJointId,
                            controlMode=mode,
                            targetPosition=act_pos,
                            targetVelocity=act_vel,
                            force=0.02)

    # print(p.getDebugVisualizerCamera())

    # every storePeriod seconds, store trajectory and joint data
    if currTime % storePeriod < timestep:
        np.savetxt("./traj_data/zippy_joint.csv", joint_real, delimiter=",")
        np.savetxt("./traj_data/zippy_joint_des.csv", joint_des, delimiter=",")
        np.savetxt("./traj_data/zippy_traj.csv", traj, delimiter=",")

    # time.sleep(4*timestep)
    print(currTime)

p.disconnect()

# dave trajectory as file
