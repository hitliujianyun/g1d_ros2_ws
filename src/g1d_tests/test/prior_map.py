"""Prior map of a sim world built from its ground truth (map frame = sim world frame)."""
import os

import numpy as np
from g1d_core.grid import build_grid, save_map
from g1d_core.pcd import write_pcd
from g1d_core.sim2d import load_world


def make_prior_map(world_yaml, out, step=0.05):
    boxes = load_world(world_yaml)
    pts = [(x, y, 0.0) for x in np.arange(-3.0, 9.0, 0.1) for y in np.arange(-4.4, 4.4, 0.1)]
    for x0, y0, z0, x1, y1, z1 in boxes:
        zs = np.arange(z0, z1 + 1e-9, step)
        for z in zs:
            pts += [(x, y, z) for x in np.arange(x0, x1 + 1e-9, step) for y in (y0, y1)]
            pts += [(x, y, z) for y in np.arange(y0, y1 + 1e-9, step) for x in (x0, x1)]
        pts += [(x, y, z1) for x in np.arange(x0, x1 + 1e-9, step) for y in np.arange(y0, y1 + 1e-9, step)]
    pts = np.array(pts)
    os.makedirs(out, exist_ok=True)
    write_pcd(os.path.join(out, "map.pcd"), pts)
    save_map(build_grid(pts), out, "map")
    with open(os.path.join(out, "trajectory.csv"), "w", encoding="utf-8") as f:   # search seeds
        f.write("x,y,yaw\n")
        for x in np.arange(-2.5, 8.6, 0.5):
            for y in np.arange(-4.0, 4.01, 0.5):
                f.write(f"{x:.2f},{y:.2f},0.0\n")
