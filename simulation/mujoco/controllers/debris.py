"""Falling debris and avoidance for the MuJoCo mission.

Physics: free boxes (the pool debris0..5 in the scene) are released above the
UAV and fall under gravity; they collide with the UAV and the ground.
Perception: a rendered upward DEPTH image only (the same camera geometry as
the Gazebo depth_up sensor: 100 deg hfov, image up = body backward). The
project's own modules then run unchanged, as in the Gazebo ROS nodes:
debris_detection.DebrisDetector -> debris_tracking.DebrisTracker ->
debris_risk.assess (ballistic path of each track) -> debris_avoidance.
AvoidancePlanner, whose proposal is checked with avoidance_interface.
Debris positions are never read from the simulator for perception; the
release log and the true separation are used only to score the run.
"""

import math

import mujoco
import numpy as np

from uav_autonomy import avoidance_interface as ai
from uav_autonomy import debris_avoidance as da
from uav_autonomy import debris_detection as dd
from uav_autonomy import debris_risk as dr
from uav_autonomy import debris_tracking as dt

W, H = 320, 240
HFOV = math.radians(100.0)
FX = (W / 2.0) / math.tan(HFOV / 2.0)
FAR = 35.0


class UpDepth:
    def __init__(self, model, data):
        self.d = data
        ext = float(model.stat.extent)
        model.vis.map.znear = 0.2 / ext          # metres: clip 0.2 .. 35 m like the sensor
        model.vis.map.zfar = FAR / ext
        self.renderer = mujoco.Renderer(model, H, W)
        self.renderer.enable_depth_rendering()

    def frame(self):
        self.renderer.update_scene(self.d, camera='up')
        depth = self.renderer.render().astype(np.float32)
        depth[depth >= FAR - 0.5] = np.nan      # nothing returned
        return depth

    def close(self):
        self.renderer.close()


class DebrisChain:
    """Detector -> tracker -> risk -> avoidance planner, fed by the up camera."""

    def __init__(self, sim, cam, ground_z=0.0, period_s=0.1):
        self.sim, self.cam, self.ground_z, self.period = sim, cam, ground_z, period_s
        self.detector = dd.DebrisDetector(dd.Intrinsics(FX, FX, W / 2.0, H / 2.0, W, H))
        self.tracker = dt.DebrisTracker()
        self.planner = da.AvoidancePlanner()
        self.rp = dr.RiskParams()
        self.last = -1e9
        self.plan = None
        self.log = []                # one row per processed frame with detections
        self.events = []             # planner transitions
        self.active_since = None
        self.n_frames = 0
        self.n_detections = 0

    def step(self, t, pos_ned, q_wxyz, vel_ned, mission_ned):
        """Run one perception + planning cycle if due; return the active
        avoidance setpoint (NED) or None."""
        if t - self.last >= self.period:
            self.last = t
            depth = self.cam.frame()
            res = self.detector.process(depth, t, (pos_ned, q_wxyz))
            self.n_frames += 1
            meas = [dt.Measurement(d.position_local, d.size_m, d.confidence, d.touches_border)
                    for d in res.detections if d.position_local is not None]
            self.n_detections += len(meas)
            tracks = self.tracker.step(t, meas) or []
            threats, rows = [], []
            for tr in tracks:
                if tr.hits < self.rp.potential_min_hits or tr.time_since_update > 0.0:
                    continue
                knots = dr.ballistic_knots(tr.x[:3], tr.x[3:], tr.P, self.rp)
                r = dr.assess(tr.id, tr.stamp, tr.status, knots, pos_ned, vel_ned,
                              float(tr.size_m), self.rp, tentative=(tr.status == dt.TENTATIVE))
                rows.append({'id': tr.id, 'state': dr.STATE_NAMES[r.state],
                             'd_min': round(float(r.d_min), 2), 'tca': round(float(r.tca), 2)})
                if r.state in (dr.POTENTIAL_THREAT, dr.WARNING, dr.CRITICAL):
                    threats.append(da.Threat(int(tr.id), float(tr.stamp), knots,
                                             float(tr.size_m), int(r.state), 0.0))
            before = len(self.planner.transitions)
            self.plan = self.planner.update(t, pos_ned, vel_ned, self.ground_z, mission_ned,
                                            True, threats, 0.0, 0.0, speed_limit=0.0)
            for tr_ in self.planner.transitions[before:]:
                self.events.append({'t': round(t, 2), 'from': da.MODE_NAMES[tr_[1]],
                                    'to': da.MODE_NAMES[tr_[2]], 'reason': tr_[3]})
            if rows:
                self.log.append({'t': round(t, 2), 'tracks': rows,
                                 'mode': da.MODE_NAMES[self.planner.mode]})
        plan = self.plan
        if plan is not None and plan.active and plan.target is not None:
            ok, why = ai.validate_command(plan.target, pos_ned, self.ground_z, 0.0, True)
            if ok:
                return np.asarray(plan.target, float)
        return None


class DebrisReleaser:
    """Release pool boxes above where the UAV will be (aimed, like the Gazebo plan)."""

    def __init__(self, sim, n=3, height=9.0, lead_s=1.4, first_s=25.0, every_s=22.0):
        self.sim, self.n, self.height, self.lead = sim, n, height, lead_s
        self.first, self.every = first_s, every_s
        self.released = []
        m = sim.m
        for i in range(6):                       # debris collides with the UAV too
            g = m.geom(f'debris{i}_geom').id
            m.geom_conaffinity[g] = 3
        self.min_sep = [math.inf] * n

    def update(self, t_since_search, pos_enu, vel_enu):
        k = len(self.released)
        if k < self.n and t_since_search >= self.first + k * self.every:
            m, d = self.sim.m, self.sim.d
            target = np.array([pos_enu[0] + vel_enu[0] * self.lead,
                               pos_enu[1] + vel_enu[1] * self.lead,
                               pos_enu[2] + self.height])
            j = m.joint(f'debris{k}_joint')
            qa, va = int(j.qposadr[0]), int(j.dofadr[0])
            d.qpos[qa:qa + 3] = target
            d.qpos[qa + 3:qa + 7] = [1, 0, 0, 0]
            d.qvel[va:va + 6] = 0.0
            mujoco.mj_forward(m, d)
            self.released.append({'name': f'debris{k}', 'sim_time_s': round(self.sim.t, 2),
                                  'release_enu_m': [round(float(x), 2) for x in target],
                                  'size_m': 0.4, 'mass_kg': 1.5, 'uav_at_release_enu_m': [round(float(x), 2) for x in pos_enu]})
        # scoring only: true separation between each released box and the UAV
        for i in range(len(self.released)):
            b = self.sim.d.body(f'debris{i}').xpos
            sep = float(np.linalg.norm(b - pos_enu))
            self.min_sep[i] = min(self.min_sep[i], sep)
