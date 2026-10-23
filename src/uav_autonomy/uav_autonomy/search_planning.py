"""Search planning for the downward camera (Phase 9).

Pure numpy; no ROS, no simulator ground truth. The ROS wrapper is
search_planner_node.py.

What it decides
    * a coverage path (parallel lanes) over a rectangular search area, sized
      from the camera's footprint on the ground at the flight altitude
    * which part of the area has been seen so far (a grid, filled from the
      vehicle's own pose and the camera geometry)
    * short detours to look at a located survivor candidate from the side:
      the vision-language model confirms oblique views far more often than
      straight-down ones (Phase 7), and a second viewpoint gives the
      localisation a baseline (Phase 8)

What it does not decide
    Altitude, arming, landing, and whether a goal is flown at all: the
    planner proposes horizontal goals; flight_controller validates them.
    Speeds are kept low and constant so that the debris-avoidance envelope
    measured in Phase 6.1 (steady flight, no hard accelerations) holds.

Frames: PX4 local NED; "ne" = (north, east).
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

STATE_IDLE = 0
STATE_SEARCHING = 1
STATE_INSPECTING = 2
STATE_COMPLETE = 3
STATE_NAMES = {0: 'IDLE', 1: 'SEARCHING', 2: 'INSPECTING', 3: 'COMPLETE'}


@dataclass
class Camera:
    """Intrinsics of the downward camera (image right = body right,
    image up = body forward)."""

    fx: float
    fy: float
    width: int
    height: int


@dataclass
class SearchArea:
    """Axis-aligned search rectangle in the local frame [m]."""

    north_min: float
    north_max: float
    east_min: float
    east_max: float

    def contains(self, ne, margin: float = 0.0) -> bool:
        return (self.north_min - margin <= ne[0] <= self.north_max + margin
                and self.east_min - margin <= ne[1]
                <= self.east_max + margin)

    def clamp(self, ne, margin: float = 0.0):
        return (min(max(ne[0], self.north_min - margin),
                    self.north_max + margin),
                min(max(ne[1], self.east_min - margin),
                    self.east_max + margin))


@dataclass
class PlannerParams:
    """Tunable parameters (all exposed as ROS parameters)."""

    target_height_m: float = 0.9      # height of what is looked for
    footprint_margin: float = 0.8     # usable fraction of the image
    lane_overlap: float = 0.5         # neighbouring swaths overlap this much
    waypoint_radius_m: float = 0.6
    search_speed_m_s: float = 2.0     # validated for detection and avoidance
    inspect_speed_m_s: float = 1.5
    cell_m: float = 0.5               # coverage grid
    max_tilt_deg: float = 25.0        # frames at larger tilt do not count
    oblique_deg: float = 15.0         # a view at least this far off nadir
    # inspection of a located candidate
    inspect_enabled: bool = True
    inspect_angle_deg: float = 35.0   # off-nadir viewing angle aimed for
    inspect_min_radius_m: float = 2.0
    inspect_max_radius_m: float = 4.0
    inspect_views: int = 2            # viewpoints, 90 degrees apart
    inspect_dwell_s: float = 4.0      # time at each viewpoint
    inspect_arrival_speed_m_s: float = 0.6
    inspect_timeout_s: float = 25.0   # per candidate
    max_inspections: int = 6
    # a candidate is inspected only once it has this many camera frames and
    # has been known for this long: short-lived duplicate entries disappear
    # and the VLM has had time to reject it
    inspect_min_observations: int = 15
    inspect_delay_s: float = 3.0
    geofence_margin_m: float = 3.0    # inspection viewpoints may leave the
    #                                   area by this much


def footprint_half_extents(height_m: float, cam: Camera,
                           margin: float) -> Tuple[float, float]:
    """Half-size of the usable ground footprint: (forward, right) [m]."""
    return (margin * height_m * (cam.height / 2.0) / cam.fy,
            margin * height_m * (cam.width / 2.0) / cam.fx)


def footprint_ne_half_extents(height_m: float, cam: Camera, margin: float,
                              yaw: float) -> Tuple[float, float]:
    """Half-size (north, east) of a box inside the footprint at this yaw.

    Exact when the heading is within 15 degrees of a compass axis (PX4 keeps
    the heading constant in these missions); otherwise the largest square
    that fits at any heading.
    """
    forward, right = footprint_half_extents(height_m, cam, margin)
    quarter = (yaw + math.pi / 4.0) % (math.pi / 2.0) - math.pi / 4.0
    if abs(quarter) > math.radians(15.0):
        side = min(forward, right) / math.sqrt(2.0)
        return side, side
    shrink = math.cos(quarter) - math.sin(abs(quarter))
    if round((yaw - quarter) / (math.pi / 2.0)) % 2 == 0:
        return forward * shrink, right * shrink   # facing north or south
    return right * shrink, forward * shrink       # facing east or west


def lawnmower(area: SearchArea, half_north: float, half_east: float,
              overlap: float, start_ne) -> List[Tuple[float, float]]:
    """Coverage waypoints: parallel lanes along the longer side of the area.

    half_north / half_east: half-size of the camera footprint on the ground.
    Returns the lane end points in flying order, starting at the lane end
    nearest to start_ne.
    """
    length_n = area.north_max - area.north_min
    length_e = area.east_max - area.east_min
    along_north = length_n >= length_e
    if along_north:
        width, half_across, half_along = length_e, half_east, half_north
        across_min, along_min, along_max = (area.east_min, area.north_min,
                                            area.north_max)
    else:
        width, half_across, half_along = length_n, half_north, half_east
        across_min, along_min, along_max = (area.north_min, area.east_min,
                                            area.east_max)
    spacing = max(2.0 * half_across * (1.0 - overlap), 0.5)
    lanes = max(1, int(math.ceil(width / spacing - 1e-9)))
    across = [across_min + (i + 0.5) * width / lanes for i in range(lanes)]
    inset = min(half_along, (along_max - along_min) / 2.0)
    lo, hi = along_min + inset, along_max - inset
    ends = [lo] if hi - lo < 0.2 else [lo, hi]

    def point(a, b):
        return (b, a) if along_north else (a, b)

    corners = [(point(a, b), i, j) for i, a in enumerate((across[0],
                                                           across[-1]))
               for j, b in enumerate((ends[0], ends[-1]))]
    _, flip_across, flip_along = min(
        corners, key=lambda c: math.hypot(c[0][0] - start_ne[0],
                                          c[0][1] - start_ne[1]))
    if flip_across:
        across = across[::-1]
    order = ends[::-1] if flip_along else ends
    path = []
    for a in across:
        for b in order:
            path.append(point(a, b))
        order = order[::-1]
    return path


def path_length(points: Sequence[Sequence[float]]) -> float:
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(points[:-1], points[1:])))


def quat_to_matrix(w, x, y, z) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q_wxyz) -> float:
    w, x, y, z = q_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def footprint_polygon(position_ned, q_wxyz, cam: Camera, plane_down: float,
                      margin: float, max_tilt_deg: float):
    """Corners (north, east) of the usable image area on a horizontal plane.

    Returns None if the vehicle is tilted too far or a corner ray does not
    reach the plane.
    """
    return footprint_polygon_rot(position_ned, quat_to_matrix(*q_wxyz), cam,
                                 plane_down, margin, max_tilt_deg)


def footprint_polygon_rot(position_ned, rot, cam: Camera, plane_down: float,
                          margin: float, max_tilt_deg: float):
    """footprint_polygon for a rotation matrix local_from_body."""
    if math.degrees(math.acos(max(-1.0, min(1.0, rot[2, 2])))) \
            > max_tilt_deg:
        return None
    half_u = margin * (cam.width / 2.0) / cam.fx
    half_v = margin * (cam.height / 2.0) / cam.fy
    corners = []
    for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        ray = rot @ np.array([-sv * half_v, su * half_u, 1.0])
        if ray[2] <= 1e-3:
            return None
        s = (plane_down - position_ned[2]) / ray[2]
        if s <= 0.0:
            return None
        corners.append((position_ned[0] + s * ray[0],
                        position_ned[1] + s * ray[1]))
    return corners


class CoverageGrid:
    """Which cells of the search area the camera has seen."""

    def __init__(self, area: SearchArea, cell_m: float):
        self.area = area
        self.cell = cell_m
        self.rows = max(1, int(math.ceil(
            (area.north_max - area.north_min) / cell_m)))
        self.cols = max(1, int(math.ceil(
            (area.east_max - area.east_min) / cell_m)))
        north = area.north_min + (np.arange(self.rows) + 0.5) * cell_m
        east = area.east_min + (np.arange(self.cols) + 0.5) * cell_m
        self.north, self.east = np.meshgrid(north, east, indexing='ij')
        self.views = np.zeros((self.rows, self.cols), dtype=np.int32)
        self.oblique = np.zeros((self.rows, self.cols), dtype=bool)

    def update(self, polygon, camera_ne, height_m: float,
               oblique_deg: float):
        """Mark the cells inside a convex footprint polygon."""
        if polygon is None:
            return 0
        inside = np.ones((self.rows, self.cols), dtype=bool)
        sign = 0.0
        for (n0, e0), (n1, e1) in zip(polygon, polygon[1:] + polygon[:1]):
            cross = (n1 - n0) * (self.east - e0) - (e1 - e0) * (
                self.north - n0)
            if sign == 0.0:
                centre_n = sum(p[0] for p in polygon) / len(polygon)
                centre_e = sum(p[1] for p in polygon) / len(polygon)
                sign = math.copysign(1.0, (n1 - n0) * (centre_e - e0)
                                     - (e1 - e0) * (centre_n - n0))
            inside &= (cross * sign) >= 0.0
        self.views[inside] += 1
        distance = np.hypot(self.north - camera_ne[0],
                            self.east - camera_ne[1])
        self.oblique |= inside & (
            distance >= height_m * math.tan(math.radians(oblique_deg)))
        return int(inside.sum())

    def fraction(self) -> float:
        return float((self.views > 0).mean())

    def oblique_fraction(self) -> float:
        return float(self.oblique.mean())


@dataclass
class Goal:
    """What the planner proposes for this cycle."""

    state: int
    target_ne: Tuple[float, float]
    speed_limit: float
    complete: bool
    reason: str
    waypoint_index: int = 0
    waypoint_count: int = 0
    inspecting_id: int = 0


@dataclass
class Inspection:
    """One detour to view a located candidate."""

    entity_id: int
    target_ne: Tuple[float, float]
    viewpoints: List[Tuple[float, float]]
    started: float
    index: int = 0
    dwell_started: Optional[float] = None
    result: str = ''
    ended: Optional[float] = None


def inspection_viewpoints(target_ne, vehicle_ne, radius: float, views: int,
                          area: SearchArea, margin: float):
    """Viewpoints on a circle around the target, 90 degrees apart.

    The first is the point of the circle nearest to the vehicle; the next
    ones turn in whichever direction stays closer to the middle of the area.
    """
    dn, de = vehicle_ne[0] - target_ne[0], vehicle_ne[1] - target_ne[1]
    bearing = math.atan2(de, dn) if math.hypot(dn, de) > 0.3 else 0.0
    centre = ((area.north_min + area.north_max) / 2.0,
              (area.east_min + area.east_max) / 2.0)

    def at(angle):
        return area.clamp((target_ne[0] + radius * math.cos(angle),
                           target_ne[1] + radius * math.sin(angle)), margin)

    def off_centre(direction):
        p = at(bearing + direction * math.pi / 2.0)
        return math.hypot(p[0] - centre[0], p[1] - centre[1])

    direction = 1.0 if off_centre(1.0) <= off_centre(-1.0) else -1.0
    return [at(bearing + direction * k * math.pi / 2.0)
            for k in range(max(1, views))]


class SearchPlanner:
    """Coverage path, coverage map and candidate inspection."""

    def __init__(self, area: SearchArea, cam: Camera,
                 params: PlannerParams = PlannerParams()):
        self.area = area
        self.cam = cam
        self.params = params
        self.state = STATE_IDLE
        self.waypoints: List[Tuple[float, float]] = []
        self.index = 0
        self.grid = CoverageGrid(area, params.cell_m)
        self.height_agl = 0.0
        self.started: Optional[float] = None
        self.finished: Optional[float] = None
        self.inspection: Optional[Inspection] = None
        self.inspections: List[Inspection] = []
        self.inspected: Dict[int, str] = {}
        self.skipped_candidates: Dict[int, str] = {}
        self._first_seen: Dict[int, float] = {}
        self.skipped_targets: List[Tuple[Tuple[float, float], str]] = []
        # set by the node while a VLM request is pending: the inspection then
        # stays at its last viewpoint (up to inspect_timeout_s) so that the
        # answer arrives while the candidate is still in view
        self.vlm_busy = False

    # -- start -------------------------------------------------------------
    def start(self, now: float, position_ne, yaw: float, height_agl: float):
        """Plan the coverage path from the current position."""
        p = self.params
        self.height_agl = float(height_agl)
        view_height = max(height_agl - p.target_height_m, 0.5)
        half_n, half_e = footprint_ne_half_extents(
            view_height, self.cam, p.footprint_margin, yaw)
        self.waypoints = lawnmower(self.area, half_n, half_e,
                                   p.lane_overlap, position_ne)
        self.index = 0
        self.state = STATE_SEARCHING
        self.started = now
        return self.waypoints

    # -- coverage ----------------------------------------------------------
    def observe(self, position_ned, q_wxyz, ground_down: float):
        """Add one camera frame's footprint to the coverage map."""
        p = self.params
        plane = ground_down - p.target_height_m
        polygon = footprint_polygon(position_ned, q_wxyz, self.cam, plane,
                                    p.footprint_margin, p.max_tilt_deg)
        return self.grid.update(polygon, position_ned[:2],
                                plane - position_ned[2], p.oblique_deg)

    # -- inspection --------------------------------------------------------
    def _candidate(self, now, position_ne, locations):
        p = self.params
        for loc in locations:
            self._first_seen.setdefault(int(loc['id']), now)
        if not p.inspect_enabled or len(self.inspections) >= \
                p.max_inspections:
            return None
        best, best_d = None, None
        for loc in locations:
            entity_id = int(loc['id'])
            if entity_id in self.inspected \
                    or entity_id in self.skipped_candidates:
                continue
            if int(loc.get('obs', p.inspect_min_observations)) \
                    < p.inspect_min_observations \
                    or now - self._first_seen[entity_id] < p.inspect_delay_s:
                continue
            ne = (float(loc['ned'][0]), float(loc['ned'][1]))
            if not self.area.contains(ne, p.geofence_margin_m):
                self.skipped_candidates[entity_id] = 'outside the area'
                continue
            if loc.get('rejected') and not loc.get('confirmed'):
                self.skipped_candidates[entity_id] = 'rejected by the VLM'
                continue
            if any(math.hypot(ne[0] - i.target_ne[0], ne[1] - i.target_ne[1])
                   < 1.0 for i in self.inspections):
                self.skipped_candidates[entity_id] = \
                    'same place as an inspected candidate'
                continue
            d = math.hypot(ne[0] - position_ne[0], ne[1] - position_ne[1])
            if best_d is None or d < best_d:
                best, best_d = loc, d
        return best

    def _begin_inspection(self, now, position_ne, loc):
        p = self.params
        ne = (float(loc['ned'][0]), float(loc['ned'][1]))
        height = self.height_agl - float(loc.get('height',
                                                 p.target_height_m))
        radius = min(max(height * math.tan(math.radians(
            p.inspect_angle_deg)), p.inspect_min_radius_m),
            p.inspect_max_radius_m)
        self.inspection = Inspection(
            entity_id=int(loc['id']), target_ne=ne, started=now,
            viewpoints=inspection_viewpoints(
                ne, position_ne, radius, p.inspect_views, self.area,
                p.geofence_margin_m))
        self.inspections.append(self.inspection)
        self.state = STATE_INSPECTING

    def _end_inspection(self, now, result):
        self.inspection.result = result
        self.inspection.ended = now
        self.inspected[self.inspection.entity_id] = result
        self.inspection = None
        self.state = STATE_SEARCHING

    def skip_current(self, now: float, target_ne, reason: str) -> bool:
        """Give up the present target (the navigator found no route).

        target_ne must be the target being flown (guards against a late
        message about an earlier one). Returns True if something was
        skipped."""
        def same(a):
            return math.hypot(a[0] - target_ne[0],
                              a[1] - target_ne[1]) <= 0.5
        if self.state == STATE_INSPECTING and self.inspection is not None:
            ins = self.inspection
            if not same(ins.viewpoints[ins.index]):
                return False
            ins.index += 1
            ins.dwell_started = None
            if ins.index >= len(ins.viewpoints):
                self._end_inspection(now, 'viewpoint unreachable')
            self.skipped_targets.append((tuple(target_ne), reason))
            return True
        if self.state == STATE_SEARCHING and self.index < len(
                self.waypoints) and same(self.waypoints[self.index]):
            self.skipped_targets.append((tuple(target_ne), reason))
            self.index += 1
            return True
        return False

    # -- main entry --------------------------------------------------------
    def update(self, now: float, position_ne, speed: float,
               locations: Sequence[dict] = ()) -> Goal:
        """One planning cycle. locations: Phase 8 entries (id, ned, height,
        confirmed, rejected, state)."""
        p = self.params
        count = len(self.waypoints)
        if self.state == STATE_IDLE:
            return Goal(STATE_IDLE, tuple(position_ne), p.search_speed_m_s,
                        False, 'not started')
        if self.state == STATE_COMPLETE:
            return Goal(STATE_COMPLETE, self.waypoints[-1],
                        p.search_speed_m_s, True, 'coverage path finished',
                        count, count)

        if self.state == STATE_SEARCHING:
            candidate = self._candidate(now, position_ne, locations)
            if candidate is not None:
                self._begin_inspection(now, position_ne, candidate)

        if self.state == STATE_INSPECTING:
            ins = self.inspection
            by_id = {int(loc['id']): loc for loc in locations}
            loc = by_id.get(ins.entity_id)
            if now - ins.started > p.inspect_timeout_s:
                self._end_inspection(now, 'timeout')
            elif loc is not None and loc.get('confirmed') \
                    and ins.dwell_started is not None \
                    and now - ins.dwell_started >= 1.0:
                self._end_inspection(now, 'confirmed')
            else:
                viewpoint = ins.viewpoints[ins.index]
                arrived = math.hypot(
                    viewpoint[0] - position_ne[0],
                    viewpoint[1] - position_ne[1]) <= p.waypoint_radius_m \
                    and speed <= p.inspect_arrival_speed_m_s
                if arrived and ins.dwell_started is None:
                    ins.dwell_started = now
                if ins.dwell_started is not None and \
                        now - ins.dwell_started >= p.inspect_dwell_s:
                    ins.index += 1
                    ins.dwell_started = None
                    if ins.index >= len(ins.viewpoints) and self.vlm_busy \
                            and now - ins.started < p.inspect_timeout_s \
                            and not (loc is not None and loc.get('confirmed')):
                        ins.index = len(ins.viewpoints) - 1
                        ins.dwell_started = now          # keep waiting here
                    elif ins.index >= len(ins.viewpoints):
                        self._end_inspection(
                            now, 'viewed' if loc is None or not loc.get(
                                'confirmed') else 'confirmed')
                if self.state == STATE_INSPECTING:
                    return Goal(
                        STATE_INSPECTING, ins.viewpoints[ins.index],
                        p.inspect_speed_m_s, False,
                        f'inspecting candidate {ins.entity_id}, view '
                        f'{ins.index + 1}/{len(ins.viewpoints)}',
                        self.index, count, ins.entity_id)

        while self.index < count and math.hypot(
                self.waypoints[self.index][0] - position_ne[0],
                self.waypoints[self.index][1] - position_ne[1]) \
                <= p.waypoint_radius_m:
            self.index += 1
        if self.index >= count:
            self.state = STATE_COMPLETE
            self.finished = now
            return Goal(STATE_COMPLETE, self.waypoints[-1],
                        p.search_speed_m_s, True, 'coverage path finished',
                        count, count)
        return Goal(STATE_SEARCHING, self.waypoints[self.index],
                    p.search_speed_m_s, False,
                    f'coverage waypoint {self.index + 1}/{count}',
                    self.index, count)
