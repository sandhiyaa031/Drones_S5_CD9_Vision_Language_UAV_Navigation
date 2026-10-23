"""Camera perception glue for the MuJoCo mission.

Renders the downward camera, runs the project's survivor detector
(uav_autonomy.survivor_detection, HSV + shape baseline) on the image and
feeds the project's localiser (uav_autonomy.survivor_localization, EKF).
The vehicle pose handed to the localiser is the simulator's own state
(ground truth, NOT an estimator and NOT SLAM). Detections come from the
rendered image only; survivor positions are never read from the scene.
"""

import math

import cv2
import mujoco
import numpy as np

W, H, FOVY_DEG = 640, 480, 73.0
FY = (H / 2.0) / math.tan(math.radians(FOVY_DEG) / 2.0)
FX = FY                                # square pixels
T_NED_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
T_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def pose_ned_frd(p_enu, rot_enu_flu):
    """MuJoCo world ENU / body FLU -> the localiser's NED position and
    quaternion (w, x, y, z) of NED <- FRD."""
    pos = np.array([p_enu[1], p_enu[0], -p_enu[2]])
    rot = T_NED_ENU @ rot_enu_flu @ T_FLU_FRD
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, rot.flatten())
    return pos, q


class DownCamera:
    def __init__(self, model, data):
        self.m, self.d = model, data
        self.renderer = mujoco.Renderer(model, H, W)

    def frame_bgr(self):
        self.renderer.update_scene(self.d, camera='down')
        rgb = self.renderer.render()
        return np.ascontiguousarray(rgb[:, :, ::-1])

    def close(self):
        self.renderer.close()


class Tracker:
    """Nearest-neighbour pixel tracker: stable track ids across frames."""

    def __init__(self, max_dist=45.0, max_age=8):
        self.tracks = {}      # id -> (u, v, last_frame)
        self.next_id = 1
        self.frame = 0
        self.max_dist, self.max_age = max_dist, max_age

    def update(self, candidates):
        self.frame += 1
        out, used = [], set()
        for c in candidates:
            u, v = c.center
            best, best_d = None, self.max_dist
            for tid, (tu, tv, fr) in self.tracks.items():
                if tid in used or self.frame - fr > self.max_age:
                    continue
                d = math.hypot(u - tu, v - tv)
                if d < best_d:
                    best, best_d = tid, d
            if best is None:
                best = self.next_id
                self.next_id += 1
            used.add(best)
            self.tracks[best] = (u, v, self.frame)
            out.append({
                'track_id': best, 'center_u': u, 'center_v': v,
                'bbox_width': float(c.width), 'bbox_height': float(c.height),
                'touches_border': bool(c.touches_border), 'measured': True,
                'semantic_state': 1, 'confidence': float(c.confidence)})
        return out
