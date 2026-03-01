import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

filename = "trajectory.csv"
plot_filename = "trajectory.png"

# Parameters
num_points = 5          # number of star points
outer_radius = 0.6      # radius of outer vertices
inner_radius = 0.1      # radius of inner vertices
inner_height = 0.0     # height of inner vertices
outer_height_1 = 0     # height of outer vertices
outer_height_2 = 0     # height of outer vertices
duration = 15.0         # total duration in seconds
yaw = 0.0               # constant yaw

# Compute angles
angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)

# Generate star vertices
vertices = []
for i in range(num_points):
    # outer vertex
    x_outer = outer_radius * np.cos(angles[i])
    y_outer = outer_radius * np.sin(angles[i])
    vertices.append([x_outer, y_outer, outer_height_1 if i%2==0 else outer_height_2, yaw])
    
    # inner vertex
    x_inner = inner_radius * np.cos(angles[i] + np.pi/num_points)
    y_inner = inner_radius * np.sin(angles[i] + np.pi/num_points)
    vertices.append([x_inner, y_inner, inner_height, yaw])

# Add timestamp
timestamps = np.linspace(0, 1000*duration, len(vertices))
trajectory = [[t, *v] for t, v in zip(timestamps, vertices)]

# Plot star
xs = [v[0] for v in vertices] + [vertices[0][0]]  # close the loop
ys = [v[1] for v in vertices] + [vertices[0][1]]
zs = [v[2] for v in vertices] + [vertices[0][2]]

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs, ys, zs, 'o-', color='orange')
ax.set_title("Star Trajectory (3D)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")

# Enforce identical data scale on x/y/z axes.
xs_arr = np.array(xs)
ys_arr = np.array(ys)
zs_arr = np.array(zs)
center_x = (xs_arr.max() + xs_arr.min()) / 2
center_y = (ys_arr.max() + ys_arr.min()) / 2
center_z = (zs_arr.max() + zs_arr.min()) / 2
half_range = max(
    (xs_arr.max() - xs_arr.min()) / 2,
    (ys_arr.max() - ys_arr.min()) / 2,
    (zs_arr.max() - zs_arr.min()) / 2,
)
ax.set_xlim(center_x - half_range, center_x + half_range)
ax.set_ylim(center_y - half_range, center_y + half_range)
ax.set_zlim(center_z - half_range, center_z + half_range)

ax.set_box_aspect((1, 1, 1))
ax.grid(True)
plt.tight_layout()
plt.savefig(plot_filename, dpi=150)
plt.close(fig)

# Save CSV
with open(filename, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp","x","y","z","yaw"])
    writer.writerows(trajectory)

print("CSV saved as", filename)
print("Plot saved as", plot_filename)
