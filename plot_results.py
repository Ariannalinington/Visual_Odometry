import numpy as np
import matplotlib.pyplot as plt


trajectory = np.genfromtxt(
    "trajectory.csv",
    delimiter=",",
    names=True
)

plt.figure(figsize=(8, 7))
plt.plot(
    trajectory["gt_x"],
    trajectory["gt_y"],
    label="Ground truth"
)
plt.plot(
    trajectory["est_x"],
    trajectory["est_y"],
    linestyle="--",
    label="Visual Odometry"
)
plt.scatter(
    trajectory["gt_x"][0],
    trajectory["gt_y"][0],
    marker="o",
    label="Start"
)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Estimated trajectory vs Ground Truth")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("trajectory_comparison.png", dpi=200)
plt.show()


map_data = np.genfromtxt(
    "map.csv",
    delimiter=",",
    names=True
)

fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")
ax.scatter(
    map_data["gt_x"],
    map_data["gt_y"],
    map_data["gt_z"],
    s=15,
    label="Ground truth"
)
ax.scatter(
    map_data["est_x"],
    map_data["est_y"],
    map_data["est_z"],
    s=15,
    marker="x",
    label="Estimated map"
)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Estimated 3D Map vs Ground Truth")
ax.legend()
plt.tight_layout()
plt.savefig("map_comparison.png", dpi=200)
plt.show()
