#!/usr/bin/env python3
"""Figure of a run's SLAM map, SLAM trajectory and the true wall outlines (OFFLINE).
Usage: plot_slam.py RUN_DIR WORLD OUT.png     (truth is used only here, for the picture)"""
import json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(__file__))
from evaluate_slam import load_truth_pose, outline_cells, WS

run, world, out = sys.argv[1:4]
rows = [json.loads(l) for l in open(os.path.join(run, 'slam_poses.jsonl')) if '"slam"' in l]
t = np.array([r['stamp'] for r in rows]); slam = np.array([r['slam'] for r in rows])
odom = np.array([r['odom'] for r in rows])
tt, tx, ty, tyaw, tz = load_truth_pose(run)
keep = (t >= tt[0]) & (t <= tt[-1]) & (np.abs(np.interp(t, tt, tz) - 6.0) <= 0.7)
first = np.nonzero(keep)[0][0]; last = np.nonzero(keep)[0]; idx = last[last < first + 5000]
slam = slam[keep]; t = t[keep]; odom = odom[keep]
truth_xy = np.column_stack([np.interp(t, tt, tx), np.interp(t, tt, ty)])
dyaw = np.interp(t[0], tt, tyaw) - slam[0, 2]
c, s = math.cos(-dyaw), math.sin(-dyaw)
def world_to_map(p):                      # inverse of the scorer's first-sample alignment
    rel = p - truth_xy[0]
    return np.column_stack([c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]]) + slam[0, :2]
g = np.load(os.path.join(run, 'slam_map.npz'))
data, res, org = g['data'], float(g['resolution']), g['origin']
spec = json.load(open(os.path.join(WS, 'results/phase7/worlds', f'{world}.json')))
outline = world_to_map(outline_cells(spec['structures'], 6.0))
fig, ax = plt.subplots(figsize=(9, 8))
ext = [org[0], org[0] + data.shape[1] * res, org[1], org[1] + data.shape[0] * res]
img = np.where(data < 0, 0.55, np.where(data > 50, 0.0, 1.0))
ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1)
ax.plot(*outline.T, '.', ms=1.2, color='tab:red', label='true wall outline at 6 m (offline truth)')
ax.plot(slam[:, 0], slam[:, 1], '-', color='tab:blue', lw=1.4, label='SLAM trajectory')
ax.plot(*world_to_map(truth_xy).T, '--', color='tab:green', lw=1.0, label='true trajectory (aligned at the first sample)')
ax.set_aspect('equal'); ax.legend(loc='lower right', fontsize=8)
ax.set_title(f'{os.path.basename(run)}: slam_toolbox map (black = occupied, grey = unknown)')
ax.set_xlabel('map x [m]'); ax.set_ylabel('map y [m]')
fig.tight_layout(); fig.savefig(out, dpi=110); print('saved', out)
