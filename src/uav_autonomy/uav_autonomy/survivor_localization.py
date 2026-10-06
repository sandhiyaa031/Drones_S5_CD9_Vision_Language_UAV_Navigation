"""Survivor 3D localisation from the downward camera (Phase 8).

Pure numpy; no ROS, no simulator ground truth. The ROS wrapper is
survivor_localization_node.py.

Inputs per camera frame
    * the survivor candidates of that frame, in pixels (Phase 7 output)
    * the vehicle's own pose estimate at the image time (PX4 local NED)
    * camera intrinsics and the camera's mounting on the airframe

Method
    The camera is monocular, so one frame gives a bearing ray only. Each
    survivor is a static 3D point p = (north, east, down) estimated by an
    extended Kalman filter whose measurement is the pixel position:

      initialisation  the first ray is intersected with a horizontal plane
                      at an assumed height above the ground; the covariance
                      is small across the ray and large along it
      update          z = (u, v) = project(p), 2x3 Jacobian, innovation gate

    While the vehicle hovers the estimate stays on the first ray and its
    height comes from the assumption. When the vehicle moves, the rays
    intersect and the filter triangulates; the assumption stops mattering.

Frames
    local   PX4 local NED (x north, y east, z down), origin where the
            estimator was initialised
    body    FRD (x forward, y right, z down)
    image   u right, v down; the downward camera is mounted so that image
            right = body right and image up = body forward
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence

import numpy as np

# semantic states of uav_interfaces/SurvivorDetection
STATE_CANDIDATE = 1
STATE_SURVIVOR_CONFIRMED = 2
STATE_UNCERTAIN = 3
STATE_REJECTED = 4

# localisation states (uav_interfaces/SurvivorLocation)
LOC_TENTATIVE = 0     # too few observations to report a position
LOC_COARSE = 1        # position on a single viewing ray; height assumed
LOC_LOCALIZED = 2     # horizontal 1-sigma below the threshold


@dataclass
class CameraModel:
    """Pinhole intrinsics and mounting of the downward camera."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    # camera position in the body frame [m] (FRD); x500_rescue: 0.14 m
    # below base_link
    offset_body: Sequence[float] = (0.0, 0.0, 0.14)


@dataclass
class LocalizerParams:
    """Tunable parameters (all exposed as ROS parameters)."""

    # height of the detected point above the ground when nothing else is
    # known: mid-way between a person lying (0.2 m) and the top of a
    # standing person (1.7 m)
    assumed_height_m: float = 0.9
    assumed_height_sigma_m: float = 0.6
    # measurement noise
    pixel_sigma_fraction: float = 0.15   # of the larger bounding-box side
    pixel_sigma_min: float = 3.0         # [px]
    attitude_sigma_deg: float = 1.5      # vehicle attitude estimate error
    position_sigma_m: float = 0.15       # vehicle position estimate error
    # consecutive frames share their errors (same view, same attitude
    # error): frames closer than this count as one measurement
    correlation_time_s: float = 0.5
    # errors that do not average out however many frames are used: the
    # vehicle's own position error in the local frame and the offset between
    # the box centre and the person's position. Added to the reported
    # covariance, not to the filter.
    systematic_sigma_m: float = 0.2
    gate_chi2: float = 13.8              # 2 dof, 99.9 %
    max_tilt_deg: float = 35.0           # skip frames at larger vehicle tilt
    min_ray_down: float = 0.35           # skip rays flatter than ~20 deg
    max_range_m: float = 60.0
    # entity management
    # a new image track joins an existing entity when its pixel position
    # agrees with that entity's projection (2 dof chi-square, 95 %)
    association_chi2: float = 6.0
    # two reported entities closer than this are the same object
    merge_distance_m: float = 0.5
    min_observations: int = 5            # before a position is reported
    localized_sigma_m: float = 0.5       # horizontal 1-sigma for LOCALIZED
    localized_min_observations: int = 10
    tentative_timeout_s: float = 5.0     # drop unconfirmed blips after this


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix local_from_body of a PX4 attitude quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pixel_ray_body(u: float, v: float, cam: CameraModel) -> np.ndarray:
    """Unit viewing ray of a pixel in the body frame (FRD)."""
    ray = np.array([-(v - cam.cy) / cam.fy, (u - cam.cx) / cam.fx, 1.0])
    return ray / np.linalg.norm(ray)


def camera_pose(position_ned, q_wxyz, cam: CameraModel):
    """Camera position in the local frame and rotation local_from_body."""
    rot = quat_to_matrix(*q_wxyz)
    return np.asarray(position_ned, float) + rot @ np.asarray(
        cam.offset_body, float), rot


def project(point_local, cam_position, rot, cam: CameraModel):
    """Local point -> (u, v) and depth along the optical axis."""
    b = rot.T @ (np.asarray(point_local, float) - cam_position)
    depth = b[2]
    if depth <= 1e-6:
        return None, depth
    return np.array([cam.cx + cam.fx * b[1] / depth,
                     cam.cy - cam.fy * b[0] / depth]), depth


def projection_jacobian(point_local, cam_position, rot,
                        cam: CameraModel) -> np.ndarray:
    """d(u, v) / d(point_local), 2x3."""
    b = rot.T @ (np.asarray(point_local, float) - cam_position)
    x, y, z = b
    d_body = np.array([[0.0, cam.fx / z, -cam.fx * y / (z * z)],
                       [-cam.fy / z, 0.0, cam.fy * x / (z * z)]])
    return d_body @ rot.T


def ray_plane(cam_position, ray_local, plane_down: float):
    """Intersection of a ray with the horizontal plane z = plane_down."""
    if ray_local[2] <= 1e-6:
        return None, None
    s = (plane_down - cam_position[2]) / ray_local[2]
    if s <= 0.0:
        return None, None
    return cam_position + s * ray_local, s


def triangulate(origins, rays):
    """Least-squares intersection of rays (Nx3 origins, Nx3 unit rays).

    Returns (point, condition number of the normal matrix). A large
    condition number means the rays are nearly parallel.
    """
    a = np.zeros((3, 3))
    b = np.zeros(3)
    for o, r in zip(np.atleast_2d(origins), np.atleast_2d(rays)):
        m = np.eye(3) - np.outer(r, r)
        a += m
        b += m @ o
    cond = float(np.linalg.cond(a))
    if not np.isfinite(cond) or cond > 1e9:
        return None, cond
    return np.linalg.solve(a, b), cond


def horizontal_sigma(cov: np.ndarray) -> float:
    """1-sigma of the worst horizontal direction."""
    return float(math.sqrt(max(np.linalg.eigvalsh(cov[:2, :2])[-1], 0.0)))


@dataclass
class Entity:
    """One located survivor candidate."""

    entity_id: int
    position: np.ndarray                 # local NED
    covariance: np.ndarray               # 3x3
    first_stamp: float
    last_stamp: float
    first_camera: np.ndarray
    observations: int = 1
    rejected_updates: int = 0
    baseline_m: float = 0.0              # largest camera displacement used
    min_range_m: float = 0.0
    track_ids: List[int] = field(default_factory=list)
    semantic_state: int = STATE_CANDIDATE
    confirmed: bool = False              # the VLM confirmed it at least once
    vlm_rejected: bool = False           # the VLM rejected it at least once
    last_update_stamp: float = 0.0
    last_track_id: int = 0

    def horizontal_sigma(self) -> float:
        return horizontal_sigma(self.covariance)


class SurvivorLocalizer:
    """Keeps one EKF per survivor candidate."""

    def __init__(self, cam: CameraModel,
                 params: LocalizerParams = LocalizerParams(),
                 ground_down: float = 0.0):
        self.cam = cam
        self.params = params
        self.ground_down = float(ground_down)   # local z of the ground
        self.entities: Dict[int, Entity] = {}
        self._by_track: Dict[int, int] = {}
        self._next_id = 1
        self.skipped = {'no_pose': 0, 'border': 0, 'tilt': 0, 'flat_ray': 0,
                        'gate': 0, 'not_measured': 0}

    # -- measurement model -------------------------------------------------
    def _pixel_variance(self, det: dict, range_m: float) -> float:
        p = self.params
        f = 0.5 * (self.cam.fx + self.cam.fy)
        box = max(float(det.get('bbox_width', 0)),
                  float(det.get('bbox_height', 0)))
        s_px = max(p.pixel_sigma_min, p.pixel_sigma_fraction * box)
        s_att = f * math.radians(p.attitude_sigma_deg)
        s_pos = f * p.position_sigma_m / max(range_m, 0.5)
        return s_px ** 2 + s_att ** 2 + s_pos ** 2

    def _initial(self, cam_position, ray_local, det):
        """First estimate: the ray cut by the assumed-height plane."""
        p = self.params
        point, s = ray_plane(cam_position, ray_local,
                             self.ground_down - p.assumed_height_m)
        if point is None or s > p.max_range_m:
            return None, None
        f = 0.5 * (self.cam.fx + self.cam.fy)
        s_across = s * math.sqrt(self._pixel_variance(det, s)) / f
        s_along = p.assumed_height_sigma_m / ray_local[2]
        rr = np.outer(ray_local, ray_local)
        cov = s_across ** 2 * (np.eye(3) - rr) + s_along ** 2 * rr
        return point, cov

    # -- association -------------------------------------------------------
    def _associate(self, det, cam_position, rot) -> Optional[Entity]:
        """Existing entity whose projection explains this detection."""
        best, best_d = None, None
        z = np.array([det['center_u'], det['center_v']])
        for entity in self.entities.values():
            predicted, _ = project(entity.position, cam_position, rot,
                                   self.cam)
            if predicted is None:
                continue
            h = projection_jacobian(entity.position, cam_position, rot,
                                    self.cam)
            rng = float(np.linalg.norm(entity.position - cam_position))
            s = h @ entity.covariance @ h.T + np.eye(2) * \
                self._pixel_variance(det, rng)
            innovation = z - predicted
            d = float(innovation @ np.linalg.solve(s, innovation))
            if d <= self.params.association_chi2 and (
                    best_d is None or d < best_d):
                best, best_d = entity, d
        return best

    def _merge_duplicates(self):
        """Fuse reported entities that converged onto the same place."""
        p = self.params
        ids = sorted(e.entity_id for e in self.reported())
        for i, a_id in enumerate(ids):
            for b_id in ids[i + 1:]:
                a, b = self.entities.get(a_id), self.entities.get(b_id)
                if a is None or b is None:
                    continue
                if np.linalg.norm(a.position[:2] - b.position[:2]) \
                        > p.merge_distance_m:
                    continue
                ia, ib = np.linalg.inv(a.covariance), \
                    np.linalg.inv(b.covariance)
                cov = np.linalg.inv(ia + ib)
                a.position = cov @ (ia @ a.position + ib @ b.position)
                # the two estimates share their errors: keep the smaller
                # covariance rather than the fused one
                a.covariance = a.covariance if np.trace(a.covariance) \
                    <= np.trace(b.covariance) else b.covariance
                a.observations += b.observations
                a.rejected_updates += b.rejected_updates
                a.baseline_m = max(a.baseline_m, b.baseline_m)
                a.min_range_m = min(a.min_range_m, b.min_range_m)
                a.last_stamp = max(a.last_stamp, b.last_stamp)
                a.confirmed = a.confirmed or b.confirmed
                a.vlm_rejected = a.vlm_rejected or b.vlm_rejected
                if b.last_update_stamp > a.last_update_stamp:
                    a.last_update_stamp = b.last_update_stamp
                    a.last_track_id = b.last_track_id
                    a.semantic_state = b.semantic_state
                a.track_ids += b.track_ids
                for track_id in b.track_ids:
                    self._by_track[track_id] = a.entity_id
                del self.entities[b_id]

    # -- main entry --------------------------------------------------------
    def update(self, stamp: float, detections: Sequence[dict], pose):
        """Process the candidates of one camera frame.

        detections: dicts with track_id, center_u, center_v, bbox_width,
        bbox_height, touches_border, measured, semantic_state.
        pose: (position_ned, q_wxyz) at the image time, or None.
        Returns the entities updated in this frame.
        """
        p = self.params
        updated = []
        if pose is None:
            self.skipped['no_pose'] += len(detections)
            self._prune(stamp)
            return updated
        cam_position, rot = camera_pose(pose[0], pose[1], self.cam)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, rot[2, 2]))))
        for det in detections:
            self._note_semantics(det)
            if not det.get('measured', True):
                self.skipped['not_measured'] += 1
                continue
            if det.get('touches_border', False):
                self.skipped['border'] += 1
                continue
            if tilt > p.max_tilt_deg:
                self.skipped['tilt'] += 1
                continue
            ray = rot @ pixel_ray_body(det['center_u'], det['center_v'],
                                       self.cam)
            if ray[2] < p.min_ray_down:
                self.skipped['flat_ray'] += 1
                continue
            entity = self._entity_for(stamp, det, cam_position, rot, ray)
            if entity is None:
                continue
            if entity.last_update_stamp == stamp and entity.observations > 1:
                continue        # second detection of the same entity
            if entity.observations > 1 or entity.first_stamp != stamp:
                if not self._ekf_update(entity, stamp, det, cam_position,
                                        rot):
                    continue
            entity.last_stamp = stamp
            entity.last_update_stamp = stamp
            entity.last_track_id = int(det['track_id'])
            entity.baseline_m = max(entity.baseline_m, float(
                np.linalg.norm(cam_position - entity.first_camera)))
            rng = float(np.linalg.norm(entity.position - cam_position))
            entity.min_range_m = rng if entity.min_range_m <= 0.0 else min(
                entity.min_range_m, rng)
            updated.append(entity)
        self._merge_duplicates()
        self._prune(stamp)
        return [e for e in updated if e.entity_id in self.entities]

    def _note_semantics(self, det):
        entity_id = self._by_track.get(int(det['track_id']))
        entity = self.entities.get(entity_id) if entity_id else None
        if entity is None:
            return
        state = int(det.get('semantic_state', STATE_CANDIDATE))
        entity.semantic_state = state
        if state == STATE_SURVIVOR_CONFIRMED:
            entity.confirmed = True
        if state == STATE_REJECTED:
            entity.vlm_rejected = True

    def _entity_for(self, stamp, det, cam_position, rot, ray):
        track_id = int(det['track_id'])
        entity_id = self._by_track.get(track_id)
        if entity_id in self.entities:
            return self.entities[entity_id]
        point, cov = self._initial(cam_position, ray, det)
        if point is None:
            self.skipped['flat_ray'] += 1
            return None
        entity = self._associate(det, cam_position, rot)
        if entity is None:
            entity = Entity(
                entity_id=self._next_id, position=point, covariance=cov,
                first_stamp=stamp, last_stamp=stamp,
                first_camera=cam_position.copy(), last_update_stamp=stamp)
            self._next_id += 1
            self.entities[entity.entity_id] = entity
        entity.track_ids.append(track_id)
        self._by_track[track_id] = entity.entity_id
        self._note_semantics(det)
        return entity

    def _ekf_update(self, entity, stamp, det, cam_position, rot) -> bool:
        p = self.params
        predicted, depth = project(entity.position, cam_position, rot,
                                   self.cam)
        if predicted is None:
            self.skipped['gate'] += 1
            entity.rejected_updates += 1
            return False
        h = projection_jacobian(entity.position, cam_position, rot, self.cam)
        rng = float(np.linalg.norm(entity.position - cam_position))
        dt = max(stamp - entity.last_update_stamp, 1e-3)
        inflate = max(1.0, p.correlation_time_s / dt)
        r = np.eye(2) * self._pixel_variance(det, rng)
        innovation = np.array([det['center_u'], det['center_v']]) - predicted
        s_gate = h @ entity.covariance @ h.T + r
        if float(innovation @ np.linalg.solve(s_gate, innovation)) \
                > p.gate_chi2:
            self.skipped['gate'] += 1
            entity.rejected_updates += 1
            return False
        s = h @ entity.covariance @ h.T + r * inflate
        k = entity.covariance @ h.T @ np.linalg.inv(s)
        entity.position = entity.position + k @ innovation
        i_kh = np.eye(3) - k @ h
        entity.covariance = i_kh @ entity.covariance @ i_kh.T \
            + k @ (r * inflate) @ k.T
        entity.observations += 1
        return True

    def _prune(self, stamp):
        p = self.params
        for entity_id in [
                e.entity_id for e in self.entities.values()
                if e.observations < p.min_observations and not e.confirmed
                and stamp - e.last_stamp > p.tentative_timeout_s]:
            del self.entities[entity_id]
            for track_id in [t for t, e in self._by_track.items()
                             if e == entity_id]:
                del self._by_track[track_id]

    # -- output ------------------------------------------------------------
    def reported_covariance(self, entity: Entity) -> np.ndarray:
        """Filter covariance plus the systematic error floor."""
        return entity.covariance + np.eye(3) * \
            self.params.systematic_sigma_m ** 2

    def reported_sigma(self, entity: Entity) -> float:
        """Horizontal 1-sigma that is published."""
        return horizontal_sigma(self.reported_covariance(entity))

    def state_of(self, entity: Entity) -> int:
        p = self.params
        if entity.observations < p.min_observations:
            return LOC_TENTATIVE
        if (entity.observations >= p.localized_min_observations
                and self.reported_sigma(entity) <= p.localized_sigma_m):
            return LOC_LOCALIZED
        return LOC_COARSE

    def reported(self) -> List[Entity]:
        """Entities with enough observations to be published."""
        return [e for e in self.entities.values()
                if e.observations >= self.params.min_observations]
