import numpy as np
import matplotlib.pyplot as plt

# read from file and store in a numpy array
j_desired_file_path = "traj_data/zippy_joint_des.csv"
j_actual_file_path = "traj_data/zippy_joint.csv"

# j_desired_file_path = "traj_data/duplo_joint_des.csv"
# j_actual_file_path = "traj_data/duplo_joint.csv"

# j_desired_file_path = "traj_data/mugatu_joint_des.csv"
# j_actual_file_path = "traj_data/mugatu_joint.csv"

j_desired = np.loadtxt(j_desired_file_path, delimiter=",")
j_actual = np.loadtxt(j_actual_file_path, delimiter=",")

jd_t = j_desired[:, 0]
jd_pos = j_desired[:, 1]

ja_t = j_actual[:, 0]
ja_pos = j_actual[:, 1]

# plot both
plt.plot(jd_t, jd_pos, label="Desired Position")
plt.plot(ja_t, ja_pos, label="Actual Position")
plt.xlabel("Time (s)")
plt.ylabel("Joint Position (rad)")
plt.title("Joint Position vs Time")
plt.legend()
plt.show()
