"""Collision-risk estimation for tracked debris (no ROS imports). Advisory.

For one debris track with predicted states (Phase 4) and the UAV state at the
same timestamp:

    r(h)     = p_debris(t + h) - p_uav(t + h)
    v_rel(h) = v_debris(t + h) - v_uav

UAV motion over the horizon (parameter uav_motion_model):
    constant_velocity      p_uav(t + h) = p_uav + v_uav h          (Phase 5)
    constant_acceleration  p_uav + v_uav h + 0.5 a_uav h^2, with the
                           acceleration held for uav_accel_hold_s and the
                           velocity constant afterwards
    any other trajectory   passed in as sampled (times, positions), e.g. the
                           calibrated vehicle response to its setpoint

Closest approach
    Authoritative: the predicted path is sampled densely between the
    predictor's states (constant acceleration inside each interval, taken
    from the difference of the predicted velocities) and the smallest |r| is
    taken. Nothing is evaluated beyond the last predicted state.
    For comparison: the constant-relative-velocity closed form
        t* = clamp(-(r . v_rel) / |v_rel|^2, 0, horizon),  d = |r + v_rel t*|

Geometry (all distances from the UAV centre to the tracked debris point)
    contact radius = uav_radius + debris_radius_factor * size
    safety radius  = contact radius + margin
    An optional per-axis scale turns the sphere into an axis-aligned
    ellipsoid (the relative vector is divided by the scale before its norm).

Uncertainty
    sigma_r = 1-sigma of the predicted position along the separation at
    closest approach. uncertainty margin = min(sigma_scale * sigma_r,
    cap_base + cap_rate * t_ca); conservative separation = d_min - margin.

States (confirmed tracks)
    CRITICAL  d_min < contact radius, or d_min < safety radius with the
              closest approach sooner than the reaction time
    WARNING   d_min < safety radius
    WATCH     conservative separation < safety radius
    SAFE      otherwise
Tentative tracks are never given those states; a fast tentative object whose
ballistic path enters the safety volume is POTENTIAL_THREAT.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

SAFE, WATCH, WARNING, CRITICAL, POTENTIAL_THREAT, UNKNOWN = range(6)
STATE_NAMES = {SAFE: 'SAFE', WATCH: 'WATCH', WARNING: 'WARNING',
               CRITICAL: 'CRITICAL', POTENTIAL_THREAT: 'POTENTIAL_THREAT',
               UNKNOWN: 'UNKNOWN'}
SEVERITY = {SAFE: 0, WATCH: 1, WARNING: 2, CRITICAL: 3}


@dataclass(frozen=True)
class RiskParams:
    """Risk settings; every value is justified in docs/debris_risk.md."""

    uav_radius_m: float = 0.40
    debris_radius_factor: float = 0.71     # of the track's size estimate
    default_debris_size_m: float = 0.40    # when the track has no size yet
    margin_m: float = 0.30
    reaction_time_s: float = 0.75
    sigma_scale: float = 1.0
    # The predictor's covariance is far too wide beyond about 0.5 s (Phase 4:
    # reported 3D sigma 2.7 m at 1 s against a 95th-percentile error of
    # 0.17 m). The uncertainty margin is therefore capped by the measured
    # error envelope: cap = base + rate * time_to_closest_approach.
    uncertainty_cap_base_m: float = 0.10
    uncertainty_cap_rate_m_s: float = 0.40
    safety_axes_scale: tuple = (1.0, 1.0, 1.0)   # x, y, z (local NED)
    sample_dt_s: float = 0.01
    # tentative tracks
    potential_min_speed_m_s: float = 2.0
    potential_min_hits: int = 2
    gravity_m_s2: float = 9.8
    potential_horizons_s: tuple = (0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0)
    # risk score: separation at which the proximity term reaches zero
    score_outer_margin_m: float = 1.0
    # UAV motion over the horizon (Phase 6.1; docs/debris_risk.md)
    uav_motion_model: str = 'constant_velocity'
    uav_accel_hold_s: float = 0.6          # acceleration assumed to last
    uav_accel_deadband_m_s2: float = 0.8   # below this: treated as zero
    uav_accel_max_m_s2: float = 8.0        # measured values are clipped


@dataclass
class Knot:
    """One state of a predicted path (h = 0 is the present track state)."""

    h: float
    p: np.ndarray
    v: np.ndarray
    cov: Optional[np.ndarray] = None       # 3x3 position covariance


@dataclass
class RiskResult:
    """Risk of one track."""

    track_id: int
    stamp: float
    track_status: int
    state: int
    score: float
    tca: float
    d_min: float
    relative_speed: float
    closest_point: np.ndarray
    uav_closest_point: np.ndarray
    current_separation: float
    horizons: List[float]
    separation_at_horizons: List[float]
    contact_radius: float
    safety_radius: float
    uncertainty_margin: float
    conservative_separation: float
    intersects: bool
    predicted_collision: bool
    horizon: float
    linear_tca: float
    linear_d_min: float
    path: List[np.ndarray] = field(default_factory=list)   # for display


def radii(size_m: float, params: RiskParams):
    """Contact and safety radius for a debris size estimate."""
    size = size_m if size_m and size_m > 0 else params.default_debris_size_m
    contact = params.uav_radius_m + params.debris_radius_factor * size
    return contact, contact + params.margin_m


def _scaled_norm(r: np.ndarray, scale) -> np.ndarray:
    return np.linalg.norm(r / np.asarray(scale, dtype=float), axis=-1)


def linear_closest_approach(r0, v_rel, horizon):
    """Closed form for constant relative velocity, clamped to [0, horizon]."""
    speed2 = float(np.dot(v_rel, v_rel))
    if speed2 < 1e-9:
        return 0.0, float(np.linalg.norm(r0))
    t = min(max(-float(np.dot(r0, v_rel)) / speed2, 0.0), horizon)
    return t, float(np.linalg.norm(np.asarray(r0) + np.asarray(v_rel) * t))


def uav_trajectory(uav_p, uav_v, uav_a, horizon, params: RiskParams,
                   dt: float = 0.02):
    """Sampled UAV path for the constant-acceleration model.

    The measured acceleration (clipped; ignored inside the deadband) is
    held for ``uav_accel_hold_s``; after that the velocity stays constant.
    Returns (times, positions).
    """
    p, v = np.asarray(uav_p, float), np.asarray(uav_v, float)
    a = np.zeros(3) if uav_a is None else np.asarray(uav_a, float).copy()
    norm = float(np.linalg.norm(a))
    if not np.all(np.isfinite(a)) or norm < params.uav_accel_deadband_m_s2:
        a = np.zeros(3)
    elif norm > params.uav_accel_max_m_s2:
        a *= params.uav_accel_max_m_s2 / norm
    times = np.arange(0.0, horizon + dt * 0.5, dt)
    held = np.minimum(times, params.uav_accel_hold_s)[:, None]
    rest = (times[:, None] - held)
    return times, (p + v * times[:, None] + 0.5 * a * held * held
                   + a * held * rest)


def _uav_at(traj, uav_p, uav_v, h):
    """UAV position and velocity at horizons ``h`` (array)."""
    if traj is None:
        return uav_p + uav_v * h[:, None], np.broadcast_to(
            uav_v, (len(h), 3))
    times, pos = traj
    p = np.column_stack([np.interp(h, times, pos[:, k]) for k in range(3)])
    eps = 0.02
    ahead = np.column_stack([np.interp(h + eps, times, pos[:, k])
                             for k in range(3)])
    return p, (ahead - p) / eps


def sampled_closest_approach(knots: Sequence[Knot], uav_p, uav_v,
                             params: RiskParams, uav_traj=None):
    """Densely sample the predicted path; return the closest approach.

    ``uav_traj`` = (times, positions) replaces the constant-velocity UAV.
    """
    uav_p, uav_v = np.asarray(uav_p, float), np.asarray(uav_v, float)
    best = None
    for a, b in zip(knots, knots[1:]):
        span = b.h - a.h
        steps = max(2, int(math.ceil(span / params.sample_dt_s)) + 1)
        tau = np.linspace(0.0, span, steps)[:, None]
        accel = (b.v - a.v) / span
        p_d = a.p + a.v * tau + 0.5 * accel * tau * tau
        v_d = a.v + accel * tau
        h = a.h + tau[:, 0]
        p_u, v_u = _uav_at(uav_traj, uav_p, uav_v, h)
        dist = _scaled_norm(p_d - p_u, params.safety_axes_scale)
        i = int(np.argmin(dist))
        if best is None or dist[i] < best['d']:
            w = tau[i, 0] / span
            cov = None
            if a.cov is not None and b.cov is not None:
                cov = (1 - w) * a.cov + w * b.cov
            best = {'d': float(dist[i]), 't': float(h[i]), 'p_d': p_d[i],
                    'p_u': p_u[i], 'v_rel': v_d[i] - v_u[i], 'cov': cov}
    if best is None:          # a single state: nothing to sample
        k = knots[0]
        r = k.p - uav_p
        best = {'d': float(_scaled_norm(r, params.safety_axes_scale)),
                't': 0.0, 'p_d': k.p, 'p_u': uav_p, 'v_rel': k.v - uav_v,
                'cov': k.cov}
    return best


def classify(d_min, conservative, tca, contact, safety,
             params: RiskParams) -> int:
    """Risk state of a confirmed track from the physical quantities."""
    if d_min < contact:
        return CRITICAL
    if d_min < safety:
        return CRITICAL if tca <= params.reaction_time_s else WARNING
    if conservative < safety:
        return WATCH
    return SAFE


def risk_score(conservative, tca, contact, safety, horizon,
               params: RiskParams) -> float:
    """0..1: proximity (using the conservative separation) times urgency."""
    outer = safety + params.score_outer_margin_m
    proximity = (outer - conservative) / max(1e-6, outer - contact)
    proximity = min(max(proximity, 0.0), 1.0)
    urgency = 1.0 - 0.5 * min(max(tca / max(horizon, 1e-6), 0.0), 1.0)
    return float(proximity * urgency)


def assess(track_id: int, stamp: float, track_status: int,
           knots: Sequence[Knot], uav_p, uav_v, size_m: float,
           params: RiskParams = RiskParams(),
           tentative: bool = False, uav_a=None,
           uav_traj=None) -> RiskResult:
    """Risk of one track from its predicted path and the UAV state.

    UAV motion: ``uav_traj`` (times, positions) if given; else constant
    acceleration from ``uav_a`` when params.uav_motion_model asks for it;
    else constant velocity.
    """
    uav_p, uav_v = np.asarray(uav_p, float), np.asarray(uav_v, float)
    contact, safety = radii(size_m, params)
    horizon = knots[-1].h
    if (uav_traj is None and uav_a is not None
            and params.uav_motion_model == 'constant_acceleration'):
        uav_traj = uav_trajectory(uav_p, uav_v, uav_a, horizon, params)
    ca = sampled_closest_approach(knots, uav_p, uav_v, params, uav_traj)
    r0 = knots[0].p - uav_p
    v0 = knots[0].v - uav_v
    lin_t, lin_d = linear_closest_approach(r0, v0, horizon)
    sigma = 0.0
    if ca['cov'] is not None and ca['d'] > 1e-6:
        u = (ca['p_d'] - ca['p_u']) / np.linalg.norm(ca['p_d'] - ca['p_u'])
        sigma = float(math.sqrt(max(0.0, u @ ca['cov'] @ u)))
    margin = min(params.sigma_scale * sigma,
                 params.uncertainty_cap_base_m
                 + params.uncertainty_cap_rate_m_s * ca['t'])
    conservative = ca['d'] - margin
    if tentative:
        fast = float(np.linalg.norm(knots[0].v)) \
            >= params.potential_min_speed_m_s
        state = POTENTIAL_THREAT if (fast and ca['d'] < safety) else SAFE
    else:
        state = classify(ca['d'], conservative, ca['t'], contact, safety,
                         params)
    p_h, _ = _uav_at(uav_traj, uav_p, uav_v,
                     np.array([k.h for k in knots[1:]]))
    separations = [float(_scaled_norm(k.p - p_h[i],
                                      params.safety_axes_scale))
                   for i, k in enumerate(knots[1:])]
    return RiskResult(
        track_id=track_id, stamp=stamp, track_status=track_status,
        state=state,
        score=risk_score(conservative, ca['t'], contact, safety, horizon,
                         params),
        tca=ca['t'], d_min=ca['d'],
        relative_speed=float(np.linalg.norm(ca['v_rel'])),
        closest_point=np.asarray(ca['p_d']),
        uav_closest_point=np.asarray(ca['p_u']),
        current_separation=float(_scaled_norm(
            r0, params.safety_axes_scale)),
        horizons=[k.h for k in knots[1:]],
        separation_at_horizons=separations,
        contact_radius=contact, safety_radius=safety,
        uncertainty_margin=margin, conservative_separation=conservative,
        intersects=bool(ca['d'] < safety),
        predicted_collision=bool(ca['d'] < contact),
        horizon=horizon, linear_tca=lin_t, linear_d_min=lin_d,
        path=[k.p for k in knots])


def ballistic_knots(p, v, cov6, params: RiskParams) -> List[Knot]:
    """Path of a tentative track: free fall from its present state."""
    p, v = np.asarray(p, float), np.asarray(v, float)
    g = np.array([0.0, 0.0, params.gravity_m_s2])
    pos_cov = None if cov6 is None else np.asarray(cov6)[:3, :3]
    knots = [Knot(0.0, p, v, pos_cov)]
    for h in params.potential_horizons_s:
        cov = None
        if cov6 is not None:
            f = np.eye(6)
            f[0, 3] = f[1, 4] = f[2, 5] = h
            cov = (f @ np.asarray(cov6) @ f.T)[:3, :3]
        knots.append(Knot(h, p + v * h + 0.5 * g * h * h, v + g * h, cov))
    return knots


class UavStateBuffer:
    """UAV position and velocity keyed by PX4 timestamp_sample."""

    def __init__(self, horizon_s: float = 3.0, max_gap_s: float = 0.05,
                 max_extrapolation_s: float = 0.03):
        """Start empty."""
        self.horizon_s = horizon_s
        self.max_gap_s = max_gap_s
        self.max_extrapolation_s = max_extrapolation_s
        self._t: List[float] = []
        self._p: List[np.ndarray] = []
        self._v: List[np.ndarray] = []

    def add(self, t: float, position, velocity):
        """Append a sample (ignored if not newer or not finite)."""
        p, v = np.asarray(position, float), np.asarray(velocity, float)
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v))):
            return
        if self._t and t <= self._t[-1]:
            return
        self._t.append(t)
        self._p.append(p)
        self._v.append(v)
        while self._t and self._t[0] < t - self.horizon_s:
            self._t.pop(0)
            self._p.pop(0)
            self._v.pop(0)

    def at(self, t: float):
        """(position, velocity) at time t, or None if not covered."""
        if not self._t:
            return None
        ts = self._t
        if t <= ts[0]:
            ok = ts[0] - t <= self.max_extrapolation_s
            return (self._p[0], self._v[0]) if ok else None
        if t >= ts[-1]:
            ok = t - ts[-1] <= self.max_extrapolation_s
            return (self._p[-1], self._v[-1]) if ok else None
        i = int(np.searchsorted(ts, t))
        if ts[i] - ts[i - 1] > self.max_gap_s:
            return None
        a = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        return ((1 - a) * self._p[i - 1] + a * self._p[i],
                (1 - a) * self._v[i - 1] + a * self._v[i])


def highest_state(results: Sequence[RiskResult]) -> int:
    """Most severe state among confirmed-track results."""
    worst = SAFE
    for r in results:
        if r.state in SEVERITY and SEVERITY[r.state] > SEVERITY[worst]:
            worst = r.state
    return worst
