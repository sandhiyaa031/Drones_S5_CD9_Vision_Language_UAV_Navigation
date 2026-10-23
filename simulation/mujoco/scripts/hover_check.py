#!/usr/bin/env python3
"""Smallest closed-loop check: an empty plane, take off, hover at 5 m."""
import os, sys
import mujoco, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from controllers.quadrotor import Quadrotor, CascadedController

xml = f'''<mujoco model="hover_check"><option timestep="0.002" gravity="0 0 -9.81"/>
<worldbody><geom name="ground" type="plane" size="50 50 0.1"/></worldbody>
<include file="{os.path.join(HERE, '..', 'models', 'x500_body.xml')}"/></mujoco>'''
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
q = Quadrotor(m, d); c = CascadedController()
dt_ctrl, n_sub = 0.004, 2
zs = []
for step in range(int(20 / m.opt.timestep)):
    if step % n_sub == 0:
        p, R, v, w = q.state()
        t = d.time
        ref = np.array([0, 0, min(5.0, 0.8 * t)])
        f = c.update(p, R, v, w, ref, np.zeros(3), 0.0, dt_ctrl)
    q.apply(f, m.opt.timestep)
    mujoco.mj_step(m, d)
    zs.append(q.state()[0][2])
zs = np.array(zs)
print('final z', round(zs[-1], 3), 'rms over last 8 s', round(float(np.sqrt(((zs[-4000:] - 5.0) ** 2).mean())), 4),
      'max z', round(float(zs.max()), 3))
