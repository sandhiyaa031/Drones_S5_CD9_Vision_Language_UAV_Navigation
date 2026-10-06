"""Debris avoidance planner (no ROS imports).

Given the UAV state, the mission setpoint and the predicted paths of the
tracked debris, choose a position setpoint that keeps the UAV outside every
debris safety volume, or report that none exists. The result is a proposal:
flight_controller validates it and is the only node that commands PX4.

All positions are PX4 local NED (z down). "Altitude" is height above the
takeoff point, altitude = ground_z - z.

Candidate
    A position setpoint: "hold" (the mission setpoint) or the present
    position displaced along a unit direction by a distance.
Response model
    PX4's cascaded position controller, reduced to
        v_cmd  = clamp(kp * (setpoint - p), v_max)
        a_cmd  = clamp(kv * (v_cmd - v), a_max)        (per axis)
        da/dt  = (a_cmd - a) / tau                     (attitude lag)
    integrated with a fixed step. The acceleration starts at the value the
    setpoint in force until now would command. Constants are measured
    (docs/debris_avoidance.md).
Hard constraints (candidate rejected if any fails)
    altitude limits, speed limits, structure exclusion zones, and for EVERY
    relevant track:
        separation(t) >= safety radius + uncertainty margin(t) + model error
    at every sampled time while that debris is above the takeoff plane.
Cost of a feasible candidate
    J = w_risk * risk + w_distance * deviation + w_energy * effort
        + w_smooth * change + w_altitude * altitude_change
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from uav_autonomy.debris_risk import (
    CRITICAL, Knot, POTENTIAL_THREAT, RiskParams, WARNING, radii)

NORMAL_FLIGHT, PREPARE_AVOIDANCE, AVOIDING, CLEARING, RESUME_MISSION, \
    AVOIDANCE_FAILSAFE, DEFENSIVE = range(7)
FRESH, DEGRADED, STALE = range(3)
HEALTH_NAMES = {FRESH: 'FRESH', DEGRADED: 'DEGRADED', STALE: 'STALE'}
MODE_NAMES = {
    NORMAL_FLIGHT: 'NORMAL_FLIGHT', PREPARE_AVOIDANCE: 'PREPARE_AVOIDANCE',
    AVOIDING: 'AVOIDING', CLEARING: 'CLEARING',
    RESUME_MISSION: 'RESUME_MISSION',
    AVOIDANCE_FAILSAFE: 'AVOIDANCE_FAILSAFE', DEFENSIVE: 'DEFENSIVE'}

# Candidate directions in local NED (north, east, down).
_D = math.sqrt(0.5)
DIRECTIONS = {
    'north': (1.0, 0.0, 0.0), 'south': (-1.0, 0.0, 0.0),
    'east': (0.0, 1.0, 0.0), 'west': (0.0, -1.0, 0.0),
    'north_east': (_D, _D, 0.0), 'north_west': (_D, -_D, 0.0),
    'south_east': (-_D, _D, 0.0), 'south_west': (-_D, -_D, 0.0),
    'climb': (0.0, 0.0, -1.0), 'descend': (0.0, 0.0, 1.0)}


@dataclass(frozen=True)
class Box:
    """Axis-aligned region: north, east and altitude intervals [m]."""

    north: Tuple[float, float]
    east: Tuple[float, float]
    altitude: Tuple[float, float]

    def contains(self, n, e, alt):
        return ((self.north[0] <= n) & (n <= self.north[1])
                & (self.east[0] <= e) & (e <= self.east[1])
                & (self.altitude[0] <= alt) & (alt <= self.altitude[1]))


# Known geometry of the collapsed building from the mission plan, relative
# to the takeoff point (the UAV starts inside, under the roof opening).
# Keep-out: the building footprint up to 0.8 m above the highest roof slab
# (debris lands and bounces there). Corridor: the roof opening shrunk by the
# UAV radius plus 0.2 m, through which takeoff and landing take place.
DEFAULT_KEEP_OUT = (Box((-4.5, 4.5), (-4.5, 4.5), (-1.0, 4.0)),)
DEFAULT_CORRIDORS = (Box((-0.4, 0.4), (-0.5, 1.4), (-1.0, 4.0)),)


@dataclass(frozen=True)
class AvoidanceParams:
    """Planner settings; every value is justified in the documentation."""

    # Candidates
    horizontal_distances_m: Tuple[float, ...] = (2.5, 4.0)
    climb_distances_m: Tuple[float, ...] = (2.0,)
    descend_distances_m: Tuple[float, ...] = (1.0,)
    directions: Tuple[str, ...] = tuple(DIRECTIONS)
    # Response model. Phase 6.1 fit: predictions started from every 0.2 s
    # of 12 hover steps and two transit legs (133 start states), with the
    # acceleration state initialised from PX4's measured acceleration
    # (results/phase6_1/model/). The Phase 6 values (0.336, 0.819, 3.373,
    # 6.46) fitted step starts only and were up to 0.8 m off mid-motion.
    tau_xy_s: float = 0.493
    kp_xy: float = 0.827
    kv_xy: float = 3.203
    a_xy_max: float = 11.08       # per axis
    v_xy_max: float = 12.0        # PX4 MPC_XY_VEL_MAX
    tau_z_s: float = 0.02
    kp_z: float = 1.181
    kv_z: float = 12.0
    a_z_max: float = 6.75
    v_up_max: float = 3.0         # PX4 MPC_Z_VEL_MAX_UP
    v_down_max: float = 1.5       # PX4 MPC_Z_VEL_MAX_DN
    # Largest position error of that model up to 1.0 s ahead over those
    # start states was 0.25 m; it is added to the separation every
    # candidate must keep.
    model_error_m: float = 0.25
    model_dt_s: float = 0.02
    horizon_s: float = 2.0
    # Hard limits
    min_altitude_m: float = 1.5
    max_altitude_m: float = 12.0
    max_speed_xy: float = 5.0
    max_speed_z: float = 2.5
    keep_out: Tuple[Box, ...] = DEFAULT_KEEP_OUT
    corridors: Tuple[Box, ...] = DEFAULT_CORRIDORS
    # Cost
    w_risk: float = 10.0
    w_distance: float = 1.0
    w_energy: float = 0.5
    w_smooth: float = 0.5
    w_altitude: float = 0.5
    risk_clear_scale_m: float = 1.5
    # A setpoint being flown is kept while it stays feasible, unless
    # another candidate is cheaper by this much.
    switch_cost_margin: float = 3.0
    # State machine
    # Optional trigger (default off = strictly the risk states): also start
    # a manoeuvre when the planner's own model of the flight to the mission
    # setpoint enters the safety volume of a confirmed track. The risk
    # estimator assumes the UAV keeps its present velocity, which is wrong
    # while it accelerates along a transit leg.
    nominal_path_trigger: bool = False
    precautionary_on_potential: bool = False
    precautionary_distance_m: float = 1.0
    clear_hold_s: float = 0.30
    prepare_timeout_s: float = 0.6
    max_manoeuvre_s: float = 4.0
    # Perception health from the age of the newest depth-derived risk frame
    # (sensor time). Typical age 0.04 s, 95 % below 0.24 s (Phase 6 logs).
    health_fresh_s: float = 0.15          # older: DEGRADED
    health_stale_s: float = 0.50          # older: STALE
    # The last risk frame keeps being used (debris paths advanced in time)
    # for this long; after that nothing is known and the planner is
    # DEFENSIVE.
    risk_hold_s: float = 1.0
    # Caps requested on mission motion (the controller applies them).
    degraded_speed_limit: float = 2.0     # [m/s]
    stale_speed_limit: float = 1.0        # [m/s] STALE and DEFENSIVE
    prepare_speed_limit: float = 2.0      # [m/s] unconfirmed fast object
    odometry_timeout_s: float = 0.5       # arrival clock (liveness)
    odometry_lag_timeout_s: float = 0.2   # behind the newest depth frame
    resume_tolerance_m: float = 0.5
    max_track_age_s: float = 0.35
    risk: RiskParams = field(default_factory=RiskParams)


@dataclass
class Threat:
    """One debris track with its predicted path."""

    track_id: int
    stamp: float                 # time of knots[0]
    knots: Sequence[Knot]
    size: float
    risk_state: int
    time_since_update: float = 0.0


@dataclass
class Candidate:
    name: str
    target: np.ndarray
    feasible: bool = False
    rejected: str = ''
    min_clearance: float = math.inf     # min over tracks of (sep - required)
    min_separation: float = math.inf
    max_speed_xy: float = 0.0
    max_speed_z: float = 0.0
    cost: float = math.inf
    terms: Dict[str, float] = field(default_factory=dict)
    path: Optional[np.ndarray] = None   # (T, 3)


@dataclass
class Plan:
    """Planner output for one cycle."""

    mode: int
    active: bool
    target: Optional[np.ndarray]
    candidate: str = ''
    reason: str = ''
    precautionary: bool = False
    failsafe: bool = False
    threat_ids: Tuple[int, ...] = ()
    predicted_min_separation: float = math.inf
    cost: float = 0.0
    candidates: List[Candidate] = field(default_factory=list)
    transition: Optional[Tuple[int, int, str]] = None
    health: int = FRESH
    speed_limit: float = 0.0            # requested cap on mission motion


def _accel_command(target, p, v, params: AvoidanceParams, v_xy_max=None):
    """Acceleration the position controller asks for (before the lag).

    ``v_xy_max`` (C, 1) is the horizontal speed cap per candidate (the
    flight controller's speed limit on the mission setpoint).
    """
    err = target - p
    v_cmd = np.empty_like(err)
    v_xy = params.kp_xy * err[:, :2]
    speed = np.linalg.norm(v_xy, axis=1, keepdims=True)
    cap = params.v_xy_max if v_xy_max is None else v_xy_max
    v_cmd[:, :2] = v_xy * np.minimum(1.0, cap / np.maximum(speed, 1e-9))
    # z is down: positive velocity is a descent.
    v_cmd[:, 2] = np.clip(params.kp_z * err[:, 2], -params.v_up_max,
                          params.v_down_max)
    a = np.empty_like(err)
    a[:, :2] = np.clip(params.kv_xy * (v_cmd[:, :2] - v[:, :2]),
                       -params.a_xy_max, params.a_xy_max)
    a[:, 2] = np.clip(params.kv_z * (v_cmd[:, 2] - v[:, 2]),
                      -params.a_z_max, params.a_z_max)
    return a


def simulate_response(p0, v0, targets, params: AvoidanceParams,
                      previous_target=None, speed_limits=None,
                      previous_speed_limit=0.0, a0=None):
    """Closed-loop response to position setpoints.

    ``targets`` is (C, 3). ``previous_target`` is the setpoint in force
    until now: the vehicle's present acceleration is taken to be what that
    setpoint commands (zero without one). ``speed_limits`` (C,) caps the
    horizontal speed per candidate (0 = no limit), as the flight controller
    does for a speed-limited mission setpoint. ``a0`` is the measured
    present acceleration; when given it replaces the value derived from
    ``previous_target``. Returns the sample times (T,),
    positions (C, T, 3) and velocities (C, T, 3); sample 0 is the present
    state.
    """
    targets = np.atleast_2d(np.asarray(targets, float))
    count = targets.shape[0]
    steps = int(round(params.horizon_s / params.model_dt_s))
    dt = params.model_dt_s
    p = np.tile(np.asarray(p0, float), (count, 1))
    v = np.tile(np.asarray(v0, float), (count, 1))
    caps = np.full((count, 1), params.v_xy_max)
    if speed_limits is not None:
        limits = np.asarray(speed_limits, float).reshape(-1, 1)
        caps = np.where(limits > 0.0, np.minimum(limits, caps), caps)
    if a0 is not None:
        a = np.tile(np.asarray(a0, float), (count, 1))
    elif previous_target is None:
        a = np.zeros((count, 3))
    else:
        a = _accel_command(
            np.asarray(previous_target, float)[None, :], p, v, params,
            previous_speed_limit if previous_speed_limit > 0.0 else None)
    gain = np.array([min(1.0, dt / params.tau_xy_s)] * 2
                    + [min(1.0, dt / params.tau_z_s)])
    times = np.arange(steps + 1) * dt
    pos = np.empty((count, steps + 1, 3))
    vel = np.empty((count, steps + 1, 3))
    pos[:, 0], vel[:, 0] = p, v
    for k in range(steps):
        a = a + (_accel_command(targets, p, v, params, caps) - a) * gain
        p = p + v * dt + 0.5 * a * dt * dt
        v = v + a * dt
        pos[:, k + 1], vel[:, k + 1] = p, v
    return times, pos, vel


def debris_path(knots: Sequence[Knot], times):
    """Debris position and 3x3 covariance at ``times`` after knots[0].

    Constant acceleration inside each interval (from the difference of the
    predicted velocities), as in the risk estimator. Times outside the
    predicted span are marked invalid; nothing is extrapolated.
    """
    times = np.asarray(times, float)
    h = np.array([k.h for k in knots])
    pos = np.full((times.size, 3), np.nan)
    cov = np.zeros((times.size, 3, 3))
    valid = (times >= h[0] - 1e-9) & (times <= h[-1] + 1e-9)
    if len(knots) < 2:
        return pos, cov, np.zeros(times.size, bool)
    idx = np.clip(np.searchsorted(h, times, side='right') - 1, 0,
                  len(knots) - 2)
    for i in range(len(knots) - 1):
        sel = valid & (idx == i)
        if not sel.any():
            continue
        span = h[i + 1] - h[i]
        acc = (np.asarray(knots[i + 1].v) - np.asarray(knots[i].v)) / span
        dt = (times[sel] - h[i])[:, None]
        pos[sel] = (np.asarray(knots[i].p) + np.asarray(knots[i].v) * dt
                    + 0.5 * acc * dt * dt)
        if knots[i + 1].cov is not None:
            # later knot: conservative
            cov[sel] = np.asarray(knots[i + 1].cov)
    return pos, cov, valid


def in_structure(points, ground_z, params: AvoidanceParams):
    """True where a point (…, 3) lies in a keep-out zone outside corridors."""
    points = np.asarray(points, float)
    n, e, alt = points[..., 0], points[..., 1], ground_z - points[..., 2]
    out = np.zeros(points.shape[:-1], bool)
    for box in params.keep_out:
        out |= box.contains(n, e, alt)
    for box in params.corridors:
        out &= ~box.contains(n, e, alt)
    return out


def make_candidates(position, mission_target, params: AvoidanceParams,
                    extra: Sequence[Tuple[str, Sequence[float]]] = ()):
    """Hold plus displaced setpoints around the present position."""
    position = np.asarray(position, float)
    out = [Candidate('hold', np.asarray(mission_target, float).copy())]
    for name in params.directions:
        direction = np.asarray(DIRECTIONS[name])
        if name == 'climb':
            distances = params.climb_distances_m
        elif name == 'descend':
            distances = params.descend_distances_m
        else:
            distances = params.horizontal_distances_m
        for dist in distances:
            out.append(Candidate(f'{name}_{dist:g}',
                                 position + direction * dist))
    for name, target in extra:
        out.append(Candidate(name, np.asarray(target, float).copy()))
    return out


def evaluate(candidates: List[Candidate], position, velocity, mission_target,
             ground_z, threats: Sequence[Threat], t_uav: float,
             params: AvoidanceParams, previous_target=None, anchor=None,
             accel=None, speed_limit: float = 0.0):
    """Fill in feasibility and cost for every candidate.

    ``accel`` is the measured present acceleration; ``speed_limit`` the cap
    in force on the mission setpoint (applies to "hold" only).
    """
    position = np.asarray(position, float)
    mission_target = np.asarray(mission_target, float)
    targets = np.array([c.target for c in candidates])
    # The setpoint in force now: the active avoidance setpoint, else the
    # mission setpoint.
    in_force = previous_target if previous_target is not None \
        else mission_target
    limits = [speed_limit if c.name == 'hold' else 0.0 for c in candidates]
    times, pos, vel = simulate_response(
        position, velocity, targets, params, in_force, limits,
        speed_limit if previous_target is None else 0.0, accel)
    altitude = ground_z - pos[..., 2]
    speed_xy = np.linalg.norm(vel[..., :2], axis=2).max(axis=1)
    speed_z = np.abs(vel[..., 2]).max(axis=1)
    structure = in_structure(pos, ground_z, params)
    # A vehicle already inside a zone (e.g. climbing out of the building
    # through the opening) must not be refused every option for that alone:
    # only entering a zone, or a target inside one, is rejected.
    inside_now = structure[:, 0]
    enters = (structure & ~inside_now[:, None]).any(axis=1) | in_structure(
        targets, ground_z, params)

    clearance = np.full(len(candidates), np.inf)
    separation = np.full(len(candidates), np.inf)
    for threat in threats:
        _, safety = radii(threat.size, params.risk)
        offset = t_uav - threat.stamp
        d_pos, d_cov, valid = debris_path(threat.knots, times + offset)
        valid &= d_pos[:, 2] <= ground_z          # still above the ground
        if not valid.any():
            continue
        rel = d_pos[None, valid] - pos[:, valid]                 # (C, T, 3)
        sep = np.linalg.norm(rel, axis=2)
        unit = rel / np.maximum(sep[..., None], 1e-9)
        sigma = np.sqrt(np.maximum(np.einsum(
            'cti,tij,ctj->ct', unit, d_cov[valid], unit), 0.0))
        t_ahead = np.maximum(times[valid] + offset, 0.0)
        cap = (params.risk.uncertainty_cap_base_m
               + params.risk.uncertainty_cap_rate_m_s * t_ahead)
        margin = np.minimum(params.risk.sigma_scale * sigma, cap[None, :])
        clearance = np.minimum(clearance, (
            sep - safety - margin - params.model_error_m).min(axis=1))
        separation = np.minimum(separation, sep.min(axis=1))

    anchor = position if anchor is None else np.asarray(anchor, float)
    for i, cand in enumerate(candidates):
        cand.path = pos[i]
        cand.min_clearance = float(clearance[i])
        cand.min_separation = float(separation[i])
        cand.max_speed_xy = float(speed_xy[i])
        cand.max_speed_z = float(speed_z[i])
        cand.rejected = ''
        if (altitude[i].min() < params.min_altitude_m
                and altitude[i].min() < altitude[i, 0] - 1e-6):
            cand.rejected = 'below minimum altitude'
        elif altitude[i].max() > params.max_altitude_m:
            cand.rejected = 'above maximum altitude'
        elif speed_xy[i] > params.max_speed_xy and cand.name != 'hold':
            # (the speed of the mission itself is the controller's matter)
            cand.rejected = 'horizontal speed limit'
        elif speed_z[i] > params.max_speed_z:
            cand.rejected = 'vertical speed limit'
        elif enters[i]:
            cand.rejected = 'structure exclusion zone'
        elif clearance[i] < 0.0:
            cand.rejected = 'safety radius + uncertainty margin'
        cand.feasible = not cand.rejected
        risk = float(np.clip(
            1.0 - clearance[i] / params.risk_clear_scale_m, 0.0, 1.0))
        deviation = _point_segment_distance(cand.target, anchor,
                                            mission_target)
        effort = float(np.linalg.norm(cand.target - position))
        change = (float(np.linalg.norm(cand.target - previous_target))
                  if previous_target is not None else 0.0)
        alt_change = abs(float(cand.target[2] - mission_target[2]))
        cand.terms = {'risk': risk, 'deviation': deviation,
                      'effort': effort, 'change': change,
                      'altitude': alt_change}
        cand.cost = (params.w_risk * risk + params.w_distance * deviation
                     + params.w_energy * effort + params.w_smooth * change
                     + params.w_altitude * alt_change)
    return candidates


def _point_segment_distance(point, a, b):
    ab = b - a
    denom = float(ab @ ab)
    s = 0.0 if denom < 1e-9 else float(np.clip((point - a) @ ab / denom,
                                               0.0, 1.0))
    return float(np.linalg.norm(point - (a + s * ab)))


def select(candidates: Sequence[Candidate]):
    """Lowest-cost feasible candidate, or None."""
    feasible = [c for c in candidates if c.feasible]
    return min(feasible, key=lambda c: c.cost) if feasible else None


def best_effort(candidates: Sequence[Candidate], current: str = '',
                switch_gain_m: float = 0.2):
    """Largest-clearance candidate among those breaking no vehicle limit.

    The candidate already being flown is kept unless another one is better
    by ``switch_gain_m``, so that near-equal options do not alternate; and
    the mission setpoint ("hold") is left only for a candidate that gains
    at least that much: when nothing helps, nothing is commanded.
    """
    usable = [c for c in candidates
              if c.rejected in ('', 'safety radius + uncertainty margin')]
    if not usable:
        return None
    best = max(usable, key=lambda c: c.min_clearance)
    # the kept setpoint is the last candidate of that name (exact target)
    kept = [c for c in usable if c.name == current][-1:]
    if kept and best.min_clearance - kept[0].min_clearance < switch_gain_m:
        return kept[0]
    hold = [c for c in usable if c.name == 'hold']
    if (not kept and hold
            and best.min_clearance - hold[0].min_clearance < switch_gain_m):
        return hold[0]
    return best


class AvoidancePlanner:
    """State machine around the candidate search."""

    def __init__(self, params: AvoidanceParams = AvoidanceParams()):
        self.params = params
        self.mode = NORMAL_FLIGHT
        self.target = None          # active avoidance setpoint
        self.candidate = ''
        self.anchor = None          # UAV position when the manoeuvre began
        self.mode_since = 0.0
        self.clear_since = None
        self.manoeuvre_started = None
        self.prepared = None        # candidate found while preparing
        self.transitions = []       # (t, from, to, reason)
        self.manoeuvres = 0
        self.refused = set()        # candidate names refused by the controller
        self._health = FRESH
        self._limit = 0.0
        self._airborne = False

    # -- helpers ---------------------------------------------------------
    def _go(self, t, mode, reason):
        if mode == self.mode:
            return None
        record = (self.mode, mode, reason)
        self.transitions.append((t, self.mode, mode, reason))
        self.mode, self.mode_since = mode, t
        return record

    def _plan(self, mode_from, t, **kw):
        plan = Plan(mode=self.mode, active=kw.pop('active', False),
                    target=kw.pop('target', None), **kw)
        plan.health = self._health
        limit = self._limit
        if self.mode == PREPARE_AVOIDANCE:
            # Defensive preparation: no manoeuvre, but no fast motion.
            prepare = self.params.prepare_speed_limit
            limit = prepare if limit <= 0.0 else min(limit, prepare)
        plan.speed_limit = 0.0 if not self._airborne else limit
        if self.mode != mode_from:
            plan.transition = self.transitions[-1][1:]
        return plan

    # -- main entry ------------------------------------------------------
    def update(self, t_uav, position, velocity, ground_z, mission_target,
               airborne_mission, threats: Sequence[Threat],
               perception_age_s: float, odometry_age_s: float,
               command_refused: bool = False,
               odometry_lag_s: float = 0.0, accel=None,
               speed_limit: float = 0.0) -> Plan:
        """One planning cycle. ``t_uav`` is the time of the UAV state.

        ``accel`` is the measured acceleration (optional); ``speed_limit``
        the cap the controller has in force on the mission setpoint.
        """
        p = self.params
        start = self.mode
        position = np.asarray(position, float)
        velocity = np.asarray(velocity, float)
        self._airborne = bool(airborne_mission and mission_target is not None)
        if perception_age_s <= p.health_fresh_s:
            self._health, self._limit = FRESH, 0.0
        elif perception_age_s <= p.health_stale_s:
            self._health, self._limit = DEGRADED, p.degraded_speed_limit
        else:
            self._health, self._limit = STALE, p.stale_speed_limit

        if not airborne_mission or mission_target is None:
            self._reset()
            self._go(t_uav, NORMAL_FLIGHT, 'not in an airborne mission state')
            return self._plan(start, t_uav, reason='not airborne')
        mission_target = np.asarray(mission_target, float)

        if (odometry_age_s > p.odometry_timeout_s
                or odometry_lag_s > p.odometry_lag_timeout_s):
            # Without the vehicle state nothing can be planned or checked.
            self._reset()
            self._go(t_uav, AVOIDANCE_FAILSAFE, 'odometry stale')
            return self._plan(start, t_uav, failsafe=True,
                              reason=f'odometry stale '
                                     f'({odometry_age_s:.2f} s, lag '
                                     f'{odometry_lag_s:.2f} s): command '
                                     'released, mission setpoint in force')

        if command_refused and self.mode in (AVOIDING, AVOIDANCE_FAILSAFE):
            self.refused.add(self.candidate)
            self.target = None

        # Stale perception is not "no debris": the last risk frame stays in
        # use (paths advanced in time) for risk_hold_s. Beyond that nothing
        # is known any more.
        if perception_age_s > p.risk_hold_s:
            return self._stale(start, t_uav, perception_age_s)
        if self.mode == DEFENSIVE:
            self._reset()
            self._go(t_uav, NORMAL_FLIGHT, 'perception restored')

        relevant = [th for th in threats
                    if th.time_since_update <= p.max_track_age_s]
        urgent = [th for th in relevant
                  if th.risk_state in (WARNING, CRITICAL)]
        potential = [th for th in relevant
                     if th.risk_state == POTENTIAL_THREAT]

        if not relevant and self.mode == NORMAL_FLIGHT:
            # Nothing tracked: the mission setpoint needs no evaluation.
            return self._plan(start, t_uav, reason='no threat')

        extra = []
        if self.target is not None:
            extra.append((self.candidate or 'current', self.target))
        candidates = [c for c in make_candidates(
            position, mission_target, p, extra) if c.name not in self.refused
            or c.name == 'hold']
        # The kept setpoint appears twice if it has a generated name: keep
        # the explicit one (exact target) and drop regenerated namesakes.
        if extra:
            keep = candidates[-1]
            candidates = [c for c in candidates[:-1]
                          if c.name != keep.name] + [keep]
        evaluate(candidates, position, velocity, mission_target, ground_z,
                 relevant, t_uav, p, self.target,
                 self.anchor if self.anchor is not None else position,
                 accel, speed_limit)
        hold = candidates[0]
        ids = tuple(sorted(th.track_id for th in urgent + potential))

        in_manoeuvre = self.mode in (AVOIDING, AVOIDANCE_FAILSAFE, CLEARING)
        nominal_unsafe = False
        if p.nominal_path_trigger and not urgent and not in_manoeuvre:
            confirmed = [th for th in relevant
                         if th.risk_state != POTENTIAL_THREAT]
            probe = evaluate([Candidate('hold', mission_target.copy())],
                             position, velocity, mission_target, ground_z,
                             confirmed, t_uav, p, self.target, position,
                             accel, speed_limit)[0]
            nominal_unsafe = (probe.rejected
                              == 'safety radius + uncertainty margin')
        if urgent or nominal_unsafe or (in_manoeuvre and not hold.feasible):
            return self._avoid(start, t_uav, position, candidates, urgent,
                               ids, nominal_unsafe)

        if in_manoeuvre:
            return self._clearing(start, t_uav, hold, candidates, ids)

        if self.mode == RESUME_MISSION:
            if np.linalg.norm(position - mission_target) \
                    <= p.resume_tolerance_m:
                self._reset()
                self._go(t_uav, NORMAL_FLIGHT, 'mission setpoint regained')
            return self._plan(start, t_uav, candidates=candidates,
                              reason='returning to the mission setpoint')

        if potential:
            return self._prepare(start, t_uav, position, candidates,
                                 potential, ids)
        if self.mode == PREPARE_AVOIDANCE:
            if t_uav - self.mode_since > p.prepare_timeout_s:
                self._reset()
                self._go(t_uav, NORMAL_FLIGHT, 'potential threat gone')
            return self._plan(start, t_uav, candidates=candidates,
                              reason='prepared, no threat')
        if self.mode == AVOIDANCE_FAILSAFE:
            self._reset()
            self._go(t_uav, NORMAL_FLIGHT, 'inputs valid again')
        return self._plan(start, t_uav, candidates=candidates,
                          reason='no threat')

    # -- branches --------------------------------------------------------
    def _reset(self):
        self.target = None
        self.candidate = ''
        self.anchor = None
        self.clear_since = None
        self.manoeuvre_started = None
        self.prepared = None
        self.refused = set()

    def _stale(self, start, t, age):
        """No usable risk frame for longer than the hold time."""
        p = self.params
        if self.mode in (AVOIDING, AVOIDANCE_FAILSAFE, CLEARING) \
                and self.target is not None:
            # Keep escaping on the last valid plan, but not indefinitely.
            if t - self.manoeuvre_started <= p.max_manoeuvre_s:
                return self._plan(
                    start, t, active=True, target=self.target,
                    candidate=self.candidate, failsafe=True,
                    reason=f'perception stale ({age:.2f} s): holding the '
                           'last avoidance setpoint')
        # DEFENSIVE: nothing is known about the sky. No manoeuvre is
        # invented; the mission continues at a low speed cap until depth
        # frames return. Hovering or landing would not make the vehicle
        # safer against an unseen object and would abandon the mission.
        self._reset()
        self._go(t, DEFENSIVE,
                 f'no depth-derived risk frame for {age:.2f} s')
        return self._plan(start, t, failsafe=True,
                          reason=f'perception stale ({age:.2f} s): '
                                 'defensive, mission motion capped at '
                                 f'{p.stale_speed_limit:.1f} m/s')

    def _avoid(self, start, t, position, candidates, urgent, ids,
               nominal_unsafe=False):
        chosen = select(candidates)
        if self.target is not None and chosen is not None:
            # Commitment: the last candidate is the setpoint being flown.
            kept = candidates[-1]
            if (kept.name == self.candidate and kept.feasible
                    and kept.cost - chosen.cost
                    < self.params.switch_cost_margin):
                chosen = kept
        failsafe = False
        if chosen is None:
            chosen = best_effort(candidates, self.candidate)
            failsafe = True
        if self.manoeuvre_started is None:
            self.manoeuvre_started = t
            self.anchor = position.copy()
            self.manoeuvres += 1
        self.clear_since = None
        if chosen is None:
            self.target, self.candidate = None, ''
            self._go(t, AVOIDANCE_FAILSAFE,
                     'no candidate within vehicle limits')
            return self._plan(start, t, failsafe=True, threat_ids=ids,
                              candidates=candidates,
                              reason='no candidate within vehicle limits: '
                                     'mission setpoint in force')
        if failsafe:
            self._go(t, AVOIDANCE_FAILSAFE,
                     'no candidate keeps the safety radius: best effort')
            reason = ('no feasible candidate; best effort '
                      f'{chosen.name} (clearance '
                      f'{chosen.min_clearance:+.2f} m)')
        else:
            worst = max((th.risk_state for th in urgent), default=WARNING)
            self._go(t, AVOIDING,
                     ('CRITICAL' if worst == CRITICAL else 'WARNING'
                      if urgent else 'mission path unsafe (planner model)'
                      if nominal_unsafe else 'mission setpoint unsafe')
                     + f' -> {chosen.name}')
            reason = f'avoiding with {chosen.name}'
        if chosen.name == 'hold':
            # The mission setpoint itself is the safe choice.
            self.target, self.candidate = None, 'hold'
            return self._plan(start, t, threat_ids=ids, candidate='hold',
                              failsafe=failsafe, candidates=candidates,
                              predicted_min_separation=chosen.min_separation,
                              cost=chosen.cost, reason=reason)
        self.target, self.candidate = chosen.target.copy(), chosen.name
        return self._plan(start, t, active=True, target=self.target,
                          candidate=chosen.name, failsafe=failsafe,
                          threat_ids=ids, candidates=candidates,
                          predicted_min_separation=chosen.min_separation,
                          cost=chosen.cost, reason=reason)

    def _clearing(self, start, t, hold, candidates, ids):
        p = self.params
        self._go(t, CLEARING, 'no urgent threat; mission setpoint is safe')
        if self.clear_since is None:
            self.clear_since = t
        if t - self.clear_since >= p.clear_hold_s or self.target is None:
            self.target, self.candidate = None, ''
            self.clear_since = None
            self.manoeuvre_started = None
            self.refused = set()
            self._go(t, RESUME_MISSION, 'clear: resuming mission')
            return self._plan(start, t, candidates=candidates,
                              reason='resuming mission')
        return self._plan(start, t, active=True, target=self.target,
                          candidate=self.candidate, threat_ids=ids,
                          candidates=candidates,
                          predicted_min_separation=hold.min_separation,
                          reason='clearing')

    def _prepare(self, start, t, position, candidates, potential, ids):
        p = self.params
        self._go(t, PREPARE_AVOIDANCE, 'unconfirmed fast object')
        self.prepared = select(candidates)
        if p.precautionary_on_potential and self.prepared is not None \
                and self.prepared.name != 'hold':
            direction = self.prepared.target - position
            norm = np.linalg.norm(direction)
            if norm > 1e-6:
                target = position + direction / norm * min(
                    norm, p.precautionary_distance_m)
                return self._plan(
                    start, t, active=True, target=target,
                    candidate=self.prepared.name, precautionary=True,
                    threat_ids=ids, candidates=candidates,
                    reason='precautionary step for an unconfirmed object')
        return self._plan(
            start, t, threat_ids=ids, candidates=candidates,
            candidate=self.prepared.name if self.prepared else '',
            reason='prepared for an unconfirmed object; no manoeuvre')
