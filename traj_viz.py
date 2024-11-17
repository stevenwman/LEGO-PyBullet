import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# read from file and store in a numpy array
file_path = "traj_data/mugatu_traj.csv"
trajectory = np.loadtxt(file_path, delimiter=",")

# trajectory = np.array(traj)
t = trajectory[:, 0]
x = trajectory[:, 1]
y = trajectory[:, 2]
z = trajectory[:, 3]

# Normalize indices to [0, 1] and map to colormap
colors = cm.viridis(t / max(t))

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the trajectory with color grading
for i in range(len(x) - 1):
    ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colors[i])

# Set equal scaling for axes
max_range = np.array([x.max() - x.min(), y.max() -
                     y.min(), z.max() - z.min()]).max() / 2.0
mid_x = (x.max() + x.min()) * 0.5
mid_y = (y.max() + y.min()) * 0.5
mid_z = (z.max() + z.min()) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# Add labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Trajectory with Equal Axes Scale')

# Add a colorbar to show the time index
sm = plt.cm.ScalarMappable(
    cmap=cm.viridis, norm=plt.Normalize(vmin=t[0], vmax=t[-1]))
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax, pad=0.1)
cbar.set_label('Time Index')

# Show the plot
plt.show()
