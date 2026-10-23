import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, '..'))
import numpy as np
from controllers.flight_sim import Sim, OccupancyMap
s = Sim('urban_A')
# put the UAV by teleporting for THIS static sensor check only (no flight)
s.d.qpos[s.quad.root:s.quad.root + 3] = [-3.0, 36.0, 6.0]
import mujoco; mujoco.mj_forward(s.m, s.d)
o, a, r = s.lidar()
print('origin', o.round(2), 'rays hit', int((r <= 29).sum()), 'of', len(r), 'min', round(float(r.min()), 2))
mp = OccupancyMap(); mp.update(o, a, r)
data = mp.grid_data(); print('occupied cells', int((data == 100).sum()), 'free', int((data == 0).sum()))
