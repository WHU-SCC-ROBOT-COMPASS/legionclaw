#!/usr/bin/env python3
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def load_pcd(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline()
            if line.startswith(b"DATA"):
                fmt = line.decode().strip().split()[1]
                break
            parts = line.decode().strip().split()
            if len(parts) >= 2:
                hdr[parts[0]] = parts[1]
        n = int(hdr.get("POINTS", 0))
        fields = hdr.get("FIELDS", "").split()
        dt = [(f, "<f4") for f in fields[:4]]
        data = np.frombuffer(f.read(), dtype=np.dtype(dt), count=n)
    has_i = "intensity" in fields or "i" in str(fields).lower()
    return data["x"], data["y"], data["z"], data[fields[3]] if has_i else np.zeros(n)

x, y, z, i = load_pcd(sys.argv[1])
m = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
x, y, z, i = x[m], y[m], z[m], i[m]
print(f"Total: {len(x)} pts")

# Downsample for rendering
if len(x) > 100000:
    idx = np.random.choice(len(x), 100000, replace=False)
    x, y, z, i = x[idx], y[idx], z[idx], i[idx]

xc, yc = np.median(x), np.median(y)
half = max(np.percentile(x,99)-np.percentile(x,1), np.percentile(y,99)-np.percentile(y,1), 20) / 2
rng = [xc-half, xc+half, yc-half, yc+half]

fig, axes = plt.subplots(1,2,figsize=(20,9))
for ax, c, cm, lbl in [(axes[0],z,"viridis","Z"), (axes[1],i,"hot","Intensity")]:
    ax.scatter(x, y, c=c, s=1, cmap=cm, alpha=0.6)
    ax.set_xlim(rng[0],rng[1]); ax.set_ylim(rng[2],rng[3])
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"BEV (color={lbl})"); ax.set_aspect("equal"); ax.grid(alpha=0.3)
plt.tight_layout()
fig.savefig(sys.argv[2], dpi=200, bbox_inches="tight")
print(f"Saved {sys.argv[2]}")
