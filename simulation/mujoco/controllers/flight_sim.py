"""MuJoCo flight simulation core: stepping, logging, collisions, lidar, map.

Everything the vehicle does goes through the four motor forces (see
quadrotor.py). Nothing here moves the vehicle except physics.

Labels (kept in every report):
  * state: the controller uses the simulator's own state (ground-truth pose
    and velocity, no estimator, no noise). It is NOT PX4 and NOT SLAM.
  * map: occupancy grid built from simulated lidar rays with the known pose
    ("mapping with known pose"), NOT SLAM.
"""

from dataclasses import dataclass, field
import math
import os
import time

import mujoco
import numpy as np

from controllers.quadrotor import CascadedController, Gains, Quadrotor

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


@dataclass
class Setpoint:
    p: np.ndarray
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.0
    motors_off: bool = False


class Sim:
    def __init__(self, scenario='urban_C', world_xml=None, gains=None,
                 gui=False, realtime=False, dt_ctrl=0.004):
        path = world_xml or os.path.join(ROOT, 'worlds',
                                         f'mj_{scenario}.xml')
        self.scenario = scenario
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        self.quad = Quadrotor(self.m, self.d)
        self.ctrl = CascadedController(gains)
        self.dt_phys = float(self.m.opt.timestep)
        self.n_sub = max(1, round(dt_ctrl / self.dt_phys))
        self.dt_ctrl = self.n_sub * self.dt_phys
        uav = self.m.body('uav').id
        self.uav_body = uav
        self.uav_geoms = {g for g in range(self.m.ngeom)
                          if self.m.geom_bodyid[g] == uav}
        self.ground = self.m.geom('ground').id
        self.log = {k: [] for k in ('t', 'p', 'v', 'rpy', 'ref', 'thrust')}
        self.collisions = []
        self.ground_contact = False
        self.touchdown_speed = None
        self.tick = 0
        self.last_sp = Setpoint(self.state()[0])
        self.realtime = realtime
        self.viewer = None
        if gui:
            from mujoco import viewer as mjviewer
            self.viewer = mjviewer.launch_passive(self.m, self.d, show_left_ui=False,
                                                  show_right_ui=False)
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = uav
            self.viewer.cam.distance = 11.0
            self.viewer.cam.azimuth = 215.0
            self.viewer.cam.elevation = -28.0
        self._wall0 = time.time()

    # -- state ---------------------------------------------------------------
    @property
    def t(self):
        return float(self.d.time)

    def state(self):
        return self.quad.state()

    def yaw(self):
        rot = self.state()[1]
        return math.atan2(rot[1, 0], rot[0, 0])

    # -- stepping ------------------------------------------------------------
    def control_tick(self, sp: Setpoint):
        p, rot, v, w = self.state()
        if sp.motors_off:
            f = np.zeros(4)
        else:
            f = self.ctrl.update(p, rot, v, w, sp.p, sp.v, sp.yaw,
                                 self.dt_ctrl)
        for _ in range(self.n_sub):
            self.quad.apply(f, self.dt_phys)
            vz_before = self.state()[2][2]
            mujoco.mj_step(self.m, self.d)
            self._contacts(vz_before)
        self.last_sp = sp
        self.tick += 1
        if self.tick % 10 == 0:
            p, rot, v, _ = self.state()
            self.log['t'].append(self.t)
            self.log['p'].append(p.copy())
            self.log['v'].append(v.copy())
            yaw = math.atan2(rot[1, 0], rot[0, 0])
            pitch = -math.asin(max(-1, min(1, rot[2, 0])))
            roll = math.atan2(rot[2, 1], rot[2, 2])
            self.log['rpy'].append((roll, pitch, yaw))
            self.log['ref'].append(np.array(sp.p).copy())
            self.log['thrust'].append(float(self.quad.thrust.sum()))
        if self.viewer is not None and self.tick % 8 == 0:
            self.viewer.sync()
            if self.realtime:
                lag = self.t - (time.time() - self._wall0)
                if lag > 0:
                    time.sleep(min(lag, 0.05))

    def _contacts(self, vz_before):
        d = self.d
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            a, b = g1 in self.uav_geoms, g2 in self.uav_geoms
            if a == b:
                continue
            other = g2 if a else g1
            if other == self.ground:
                if not self.ground_contact:
                    self.ground_contact = True
                    self.touchdown_speed = float(-vz_before)
            else:
                name = self.m.geom(other).name or f'geom{other}'
                if not self.collisions or self.collisions[-1][1] != name:
                    self.collisions.append((self.t, name, float(c.dist)))

    def run(self, provider, until, max_time):
        """provider(sim) -> Setpoint every control tick; until(sim) -> bool."""
        t_end = self.t + max_time
        while self.t < t_end:
            self.control_tick(provider(self))
            if until(self):
                return True
        return False

    def close(self):
        if self.viewer is not None:
            self.viewer.close()

    # -- sensors ---------------------------------------------------------------
    def lidar(self, n=360, max_range=29.0):
        """Horizontal ring of rays from the lidar site. Returns (origin_xy,
        angles, ranges) with ranges = max_range + 1 where nothing is hit."""
        origin = self.d.site_xpos[self.quad.site_lidar].copy()
        ang = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
        vec = np.column_stack([np.cos(ang), np.sin(ang), np.zeros(n)])
        geomid = np.zeros(n, dtype=np.int32)
        dist = np.zeros(n)
        normal = np.zeros(n * 3)
        mujoco.mj_multiRay(self.m, self.d, origin, vec.flatten(), None, 1,
                           self.uav_body, geomid, dist, normal, n,
                           max_range)
        ranges = np.where(dist < 0, max_range + 1.0, dist)
        return origin, ang, ranges


class OccupancyMap:
    """Log-odds grid from lidar rays with a KNOWN pose (not SLAM)."""

    def __init__(self, x0=-70.0, y0=-50.0, x1=130.0, y1=130.0, res=0.25):
        self.res, self.x0, self.y0 = res, x0, y0
        self.nx, self.ny = int((x1 - x0) / res), int((y1 - y0) / res)
        self.logodds = np.zeros((self.ny, self.nx), np.float32)
        self.seen = np.zeros((self.ny, self.nx), bool)
        self.scans = 0

    def update(self, origin, ang, ranges, max_range=29.0):
        res = self.res
        for a, r in zip(ang, ranges):
            hit = r <= max_range
            rr = r if hit else max_range
            steps = np.arange(0.0, rr, res * 0.8)
            xs = origin[0] + np.cos(a) * steps
            ys = origin[1] + np.sin(a) * steps
            ix = ((xs - self.x0) / res).astype(int)
            iy = ((ys - self.y0) / res).astype(int)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            self.logodds[iy[ok], ix[ok]] -= 0.15
            self.seen[iy[ok], ix[ok]] = True
            if hit:
                hx = int((origin[0] + np.cos(a) * r - self.x0) / res)
                hy = int((origin[1] + np.sin(a) * r - self.y0) / res)
                if 0 <= hx < self.nx and 0 <= hy < self.ny:
                    self.logodds[hy, hx] += 0.9
                    self.seen[hy, hx] = True
        np.clip(self.logodds, -4.0, 4.0, out=self.logodds)
        self.scans += 1

    def grid_data(self):
        """slam_toolbox-style data: -1 unknown, 0 free, 100 occupied."""
        data = np.full(self.logodds.shape, -1, np.int8)
        data[self.seen & (self.logodds < 0.3)] = 0
        data[self.logodds >= 0.7] = 100
        return data

    @property
    def origin(self):
        return (self.x0, self.y0)
