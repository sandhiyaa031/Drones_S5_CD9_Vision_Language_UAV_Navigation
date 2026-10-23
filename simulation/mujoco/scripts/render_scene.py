#!/usr/bin/env python3
"""Render still views of a MuJoCo scene (offscreen) to PNG.
Usage: render_scene.py [scenario] [out_dir]    (MUJOCO_GL=egl|glfw|osmesa)"""
import os, sys
import cv2, mujoco, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sc = sys.argv[1] if len(sys.argv) > 1 else 'urban_C'
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '..', 'results', 'screens')
os.makedirs(out, exist_ok=True)
m = mujoco.MjModel.from_xml_path(os.path.join(HERE, '..', 'worlds', f'mj_{sc}.xml'))
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 600, 960)
views = {'overview': (-50, -45, 48, 0.78, 0.55), 'collapse': (-30, 28, 22, 0.35, 0.50),
         'courtyard': (58, -22, 22, 1.57, 0.60), 'top_site': (8, 52, 52, 0.0, 1.45)}
for name, (x, y, z, yaw, pitch) in views.items():
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    # look direction from yaw (about z) and pitch (down)
    d_xy = np.array([np.cos(yaw), np.sin(yaw)]) * np.cos(pitch)
    look = np.array([d_xy[0], d_xy[1], -np.sin(pitch)])
    dist = 60.0
    cam.lookat[:] = np.array([x, y, z]) + look * dist
    cam.distance = dist
    cam.azimuth = np.degrees(yaw)
    cam.elevation = -np.degrees(pitch)
    r.update_scene(d, cam)
    cv2.imwrite(os.path.join(out, f'{sc}_{name}.png'), r.render()[:, :, ::-1])
    print('saved', name)
