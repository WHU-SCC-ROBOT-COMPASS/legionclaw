#!/usr/bin/env python3
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

data = np.loadtxt(sys.argv[1], skiprows=1)
t, x, y, z = data[:,0], data[:,1], data[:,2], data[:,3]
title = sys.argv[2] if len(sys.argv) > 2 else "Trajectory"
prefix = sys.argv[1].replace(".txt", "")

fig, ax = plt.subplots(figsize=(10,8))
ax.plot(x, y, "b-", linewidth=1)
ax.scatter(x[0], y[0], c="green", s=50, label="start")
ax.scatter(x[-1], y[-1], c="red", s=50, label="end")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_title(f"{title} - 2D"); ax.axis("equal"); ax.grid(); ax.legend()
fig.savefig(f"{prefix}_2d.png", dpi=150); plt.close()
print(f"Saved {prefix}_2d.png")

fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111, projection="3d")
ax.plot(x, y, z, "b-", linewidth=1)
ax.scatter(x[0], y[0], z[0], c="green", s=50, label="start")
ax.scatter(x[-1], y[-1], z[-1], c="red", s=50, label="end")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
ax.set_title(f"{title} - 3D"); ax.legend()
fig.savefig(f"{prefix}_3d.png", dpi=150); plt.close()
print(f"Saved {prefix}_3d.png")

fig, axes = plt.subplots(4, 1, figsize=(12,10), sharex=True)
axes[0].plot(t-t[0], x, label="X"); axes[0].set_ylabel("X (m)"); axes[0].grid(); axes[0].legend()
axes[1].plot(t-t[0], y, label="Y"); axes[1].set_ylabel("Y (m)"); axes[1].grid(); axes[1].legend()
axes[2].plot(t-t[0], z, label="Z"); axes[2].set_ylabel("Z (m)"); axes[2].grid(); axes[2].legend()
dist = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)
speed = dist / np.diff(t)
axes[3].plot(t[1:]-t[0], speed, label="Speed", color="orange")
axes[3].set_xlabel("Time (s)"); axes[3].set_ylabel("Speed (m/s)"); axes[3].grid(); axes[3].legend()
fig.suptitle(f"{title} - Stats")
fig.savefig(f"{prefix}_stats.png", dpi=150); plt.close()
print(f"Saved {prefix}_stats.png")
