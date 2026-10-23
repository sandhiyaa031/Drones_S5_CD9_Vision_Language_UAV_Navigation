"""Map-aware route planning for the village missions.

Pure numpy / standard library; no ROS. The ROS wrapper is navigator_node.py.

Layers (who decides what):

    flight stabilisation      PX4 (unchanged)
    state estimate for flight PX4 EKF2 (unchanged)
    localisation in the map   slam_toolbox (map -> odom correction)
    global route              THIS FILE: A* on the SLAM occupancy grid
    local path following      THIS FILE: a look-ahead point on that route
    debris reflex             avoidance planner (unchanged, overrides)
    where to search           search planner (unchanged)
    whether to fly at all     mission manager (unchanged)

The grid is slam_toolbox's map: one horizontal slice of the world at the
flight altitude. Cells never observed are treated as free but more
expensive, so the vehicle can set out before it has seen its destination and
re-plans as walls appear ("optimistic" planning in an unknown map).
"""

from dataclasses import dataclass
import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

OCCUPIED_THRESHOLD = 50

NAV_IDLE = 'IDLE'
NAV_FOLLOWING = 'FOLLOWING'
NAV_DIRECT = 'DIRECT'                # no map yet: straight to the goal
NAV_UNREACHABLE = 'UNREACHABLE'
NAV_BACKTRACK = 'BACKTRACKING'
NAV_LOCALIZATION_LOST = 'LOCALIZATION_LOST'
NAV_ARRIVED = 'ARRIVED'


@dataclass
class NavParams:
    """Tunable parameters (exposed as ROS parameters)."""

    planning_resolution_m: float = 0.5
    inflation_m: float = 1.75         # keep-out distance from mapped cells
    caution_m: float = 4.0            # cells closer than this cost more
    caution_weight: float = 3.0
    unknown_cost: float = 0.6         # extra cost of a never-observed cell
    lookahead_m: float = 3.5
    replan_period_s: float = 1.0
    goal_tolerance_m: float = 0.6
    grid_margin_m: float = 12.0
    max_expansions: int = 150000
    crumb_spacing_m: float = 1.5      # breadcrumbs of the flown path
    crumb_reached_m: float = 1.0
    goal_snap_m: float = 3.0          # a goal inside an obstacle may move
    #                                   this far to the nearest free cell
    near_obstacle_speed_m_s: float = 1.2
    near_obstacle_m: float = 3.0
    # localisation health
    # SLAM is unusable if its transform or its scan input is older than
    # this (an avoidance climb takes the lidar out of the mapped slice for
    # a few seconds; that must not count as a loss)
    slam_timeout_s: float = 6.0
    # The map frame and the PX4 frame are different frames (the map is
    # anchored to wherever, and facing however, the vehicle was at the
    # first scan), so their coordinates are not compared directly. What
    # must hold is that the relation between them changes slowly: if the
    # SLAM pose moves by more than this against the PX4 pose within
    # jump_window_s, SLAM has jumped (a wrong scan match or loop closure).
    max_jump_m: float = 2.5
    jump_window_s: float = 2.0


class PlanningGrid:
    """Occupancy on a coarser grid, with clearance to the nearest obstacle."""

    def __init__(self, data: np.ndarray, resolution: float, origin_xy,
                 points_xy: Sequence[Sequence[float]], params: NavParams):
        """data: slam_toolbox map (rows = y, cols = x; -1 unknown, 0..100).
        points_xy: points (start, goal) the grid must contain."""
        p = params
        self.res = p.planning_resolution_m
        x0 = origin_xy[0]
        y0 = origin_xy[1]
        x1 = x0 + data.shape[1] * resolution
        y1 = y0 + data.shape[0] * resolution
        xs = [x0, x1] + [q[0] for q in points_xy]
        ys = [y0, y1] + [q[1] for q in points_xy]
        self.x0 = min(xs) - p.grid_margin_m
        self.y0 = min(ys) - p.grid_margin_m
        cols = int(math.ceil((max(xs) + p.grid_margin_m - self.x0)
                             / self.res))
        rows = int(math.ceil((max(ys) + p.grid_margin_m - self.y0)
                             / self.res))
        self.occupied = np.zeros((rows, cols), dtype=bool)
        self.known = np.zeros((rows, cols), dtype=bool)
        r, c = np.nonzero(data >= 0)
        if len(r):
            gx = ((x0 + (c + 0.5) * resolution - self.x0) / self.res).astype(
                int)
            gy = ((y0 + (r + 0.5) * resolution - self.y0) / self.res).astype(
                int)
            self.known[gy, gx] = True
            occ = data[r, c] > OCCUPIED_THRESHOLD
            self.occupied[gy[occ], gx[occ]] = True
        self.clearance = self._clearance()
        self.blocked = self.clearance < p.inflation_m
        cost = np.ones((rows, cols))
        near = self.clearance < p.caution_m
        cost[near] += p.caution_weight * (
            1.0 - (self.clearance[near] - p.inflation_m)
            / max(p.caution_m - p.inflation_m, 1e-6)).clip(0.0, 1.0)
        cost[~self.known] += p.unknown_cost
        self.cost = cost

    def _clearance(self) -> np.ndarray:
        """Distance [m] from every cell to the nearest occupied cell."""
        if not self.occupied.any():
            return np.full(self.occupied.shape, 1e3)
        from scipy.ndimage import distance_transform_edt
        return distance_transform_edt(~self.occupied) * self.res

    def cell(self, xy) -> Tuple[int, int]:
        return (int((xy[1] - self.y0) / self.res),
                int((xy[0] - self.x0) / self.res))

    def point(self, cell) -> Tuple[float, float]:
        return (self.x0 + (cell[1] + 0.5) * self.res,
                self.y0 + (cell[0] + 0.5) * self.res)

    def inside(self, cell) -> bool:
        return 0 <= cell[0] < self.occupied.shape[0] \
            and 0 <= cell[1] < self.occupied.shape[1]

    def nearest_free(self, cell, max_distance_m: float):
        """Closest unblocked cell within max_distance_m, or None."""
        if self.inside(cell) and not self.blocked[cell]:
            return cell
        reach = int(math.ceil(max_distance_m / self.res))
        best, best_d = None, None
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                c = (cell[0] + dr, cell[1] + dc)
                d = math.hypot(dr, dc)
                if d * self.res > max_distance_m or not self.inside(c) \
                        or self.blocked[c]:
                    continue
                if best_d is None or d < best_d:
                    best, best_d = c, d
        return best

    def segment_free(self, a_xy, b_xy) -> bool:
        """True if the straight segment a-b crosses no blocked cell."""
        length = math.hypot(b_xy[0] - a_xy[0], b_xy[1] - a_xy[1])
        steps = max(2, int(length / (self.res * 0.5)))
        for i in range(steps + 1):
            f = i / steps
            c = self.cell((a_xy[0] + f * (b_xy[0] - a_xy[0]),
                           a_xy[1] + f * (b_xy[1] - a_xy[1])))
            if self.inside(c) and self.blocked[c]:
                return False
        return True


NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
              (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def astar(grid: PlanningGrid, start_xy, goal_xy, params: NavParams):
    """Shortest weighted route on the grid.

    Returns (list of (x, y) points or None, note). The start is allowed to
    be inside the keep-out band (the vehicle may already be near a wall):
    it leaves by the cheapest cells. The goal is moved to the nearest free
    cell within goal_snap_m if it lies in the band.
    """
    start = grid.cell(start_xy)
    goal = grid.nearest_free(grid.cell(goal_xy), params.goal_snap_m)
    if goal is None:
        return None, 'goal is inside an obstacle'
    if not grid.inside(start):
        return None, 'start is outside the planning grid'
    rows, cols = grid.occupied.shape
    g = np.full((rows, cols), np.inf)
    parent = {}
    g[start] = 0.0
    heap = [(0.0, start)]
    expansions = 0
    blocked, cost = grid.blocked, grid.cost
    escape = bool(blocked[start])
    while heap:
        f, cur = heapq.heappop(heap)
        if cur == goal:
            break
        if f - math.hypot(cur[0] - goal[0], cur[1] - goal[1]) \
                > g[cur] + 1e-9:
            continue
        expansions += 1
        if expansions > params.max_expansions:
            return None, 'search limit reached'
        for dr, dc, step in NEIGHBOURS:
            nxt = (cur[0] + dr, cur[1] + dc)
            if nxt[0] < 0 or nxt[1] < 0 or nxt[0] >= rows or nxt[1] >= cols:
                continue
            if blocked[nxt]:
                # only allowed while still leaving the band we started in,
                # and never through an occupied cell itself
                if not (escape and blocked[cur]) or grid.occupied[nxt]:
                    continue
                penalty = 20.0
            else:
                penalty = 0.0
            new = g[cur] + step * (cost[nxt] + penalty)
            if new < g[nxt]:
                g[nxt] = new
                parent[nxt] = cur
                heapq.heappush(heap, (new + math.hypot(
                    nxt[0] - goal[0], nxt[1] - goal[1]), nxt))
    if goal != start and goal not in parent:
        return None, 'no route in the present map'
    cells = [goal]
    while cells[-1] != start:
        cells.append(parent[cells[-1]])
    cells.reverse()
    path = [tuple(start_xy)] + [grid.point(c) for c in cells[1:]]
    snapped = math.hypot(path[-1][0] - goal_xy[0],
                         path[-1][1] - goal_xy[1]) > grid.res
    if not snapped:
        path[-1] = tuple(goal_xy)
    return shortcut(grid, path), ('goal moved to the nearest free cell'
                                  if snapped else '')


def shortcut(grid: PlanningGrid, path):
    """Remove intermediate points wherever a straight segment is free."""
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not grid.segment_free(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def path_length(path) -> float:
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(path[:-1], path[1:])))


def lookahead_point(path, position_xy, distance_m):
    """Point `distance_m` further along the path than the vehicle is."""
    if len(path) == 1:
        return path[0]
    best_d, best_s, s_acc = None, 0.0, 0.0
    for a, b in zip(path[:-1], path[1:]):
        seg = (b[0] - a[0], b[1] - a[1])
        length = math.hypot(*seg)
        if length < 1e-9:
            continue
        t = ((position_xy[0] - a[0]) * seg[0]
             + (position_xy[1] - a[1]) * seg[1]) / (length * length)
        t = min(max(t, 0.0), 1.0)
        d = math.hypot(a[0] + t * seg[0] - position_xy[0],
                       a[1] + t * seg[1] - position_xy[1])
        if best_d is None or d < best_d:
            best_d, best_s = d, s_acc + t * length
        s_acc += length
    target = min(best_s + distance_m, s_acc)
    s_acc = 0.0
    for a, b in zip(path[:-1], path[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if s_acc + length >= target - 1e-9 and length > 1e-9:
            f = (target - s_acc) / length
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
        s_acc += length
    return path[-1]


class Pose2D:
    """Planar pose (x, y, yaw) with the two operations navigation needs."""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    def apply(self, xy):
        """Point given in this pose's frame -> parent frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + c * xy[0] - s * xy[1],
                self.y + s * xy[0] + c * xy[1])

    def inverse_apply(self, xy):
        """Point given in the parent frame -> this pose's frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = xy[0] - self.x, xy[1] - self.y
        return (c * dx + s * dy, -s * dx + c * dy)


def px4_to_map(point_enu, pose_px4: Pose2D, pose_map: Pose2D):
    """A point in the PX4 local frame (as east, north) -> the SLAM map.

    The vehicle is one body with two pose estimates; a point is carried
    from one frame to the other through the body frame.
    """
    return pose_map.apply(pose_px4.inverse_apply(point_enu))


def map_to_px4(point_map, pose_px4: Pose2D, pose_map: Pose2D):
    return pose_px4.apply(pose_map.inverse_apply(point_map))


class Navigator:
    """Turns a far goal into the next safe set-point, on the evolving map."""

    def __init__(self, params: NavParams = NavParams()):
        self.params = params
        self.state = NAV_IDLE
        self.path: List[Tuple[float, float]] = []
        self.path_goal: Optional[Tuple[float, float]] = None
        self.last_plan = -1e9
        self.note = ''
        self.plans = 0
        self.replans_blocked = 0
        self.plan_ms: List[float] = []
        self.unreachable_since: Optional[float] = None
        self.crumbs: List[Tuple[float, float]] = []
        self.backtrack_idx: Optional[int] = None
        self._history: List[Tuple[float, Pose2D, Pose2D]] = []
        self.largest_jump_m = 0.0

    def localization_ok(self, now, slam_stamp, pose_px4: Pose2D,
                        pose_map: Optional[Pose2D]) -> Tuple[bool, str]:
        p = self.params
        if pose_map is None or slam_stamp is None:
            return False, 'no SLAM pose yet'
        if now - slam_stamp > p.slam_timeout_s:
            return False, (f'SLAM transform or scan input is '
                           f'{now - slam_stamp:.1f} s old')
        # where would the vehicle be in the map if the relation between the
        # two frames were still what it was jump_window_s ago?
        self._history.append((now, pose_px4, pose_map))
        while len(self._history) > 1 and \
                now - self._history[1][0] >= p.jump_window_s:
            self._history.pop(0)
        then, old_px4, old_map = self._history[0]
        if now - then >= 0.5 * p.jump_window_s:
            expected = px4_to_map((pose_px4.x, pose_px4.y), old_px4, old_map)
            jump = math.hypot(pose_map.x - expected[0],
                              pose_map.y - expected[1])
            self.largest_jump_m = max(self.largest_jump_m, jump)
            if jump > p.max_jump_m:
                return False, (f'the SLAM pose jumped {jump:.1f} m against '
                               f'the PX4 pose within {now - then:.1f} s')
        return True, ''

    def record(self, position_map) -> None:
        """Remember where the vehicle has been (every crumb_spacing_m)."""
        p = self.params
        if self.backtrack_idx is not None:
            return
        if not self.crumbs or math.hypot(
                position_map[0] - self.crumbs[-1][0],
                position_map[1] - self.crumbs[-1][1]) >= p.crumb_spacing_m:
            self.crumbs.append((float(position_map[0]),
                                float(position_map[1])))
            del self.crumbs[:-4000]

    def backtrack(self, position_map):
        """Next set-point when retracing the flown path in reverse, or None
        when the path is used up. The crumbs are places the vehicle has been
        at, so the way back is known to be flyable even when the map no
        longer offers a route (for example from a pocket the inflated map has
        closed behind it)."""
        p = self.params
        if self.backtrack_idx is None:
            self.backtrack_idx = len(self.crumbs) - 1
        while self.backtrack_idx >= 0 and math.hypot(
                self.crumbs[self.backtrack_idx][0] - position_map[0],
                self.crumbs[self.backtrack_idx][1] - position_map[1]) \
                < p.crumb_reached_m:
            self.backtrack_idx -= 1
        if self.backtrack_idx < 0:
            return None
        return self.crumbs[self.backtrack_idx]

    def stop_backtrack(self) -> None:
        self.backtrack_idx = None

    def update(self, now: float, grid: Optional[PlanningGrid],
               position_map, goal_map, clock=None):
        """Returns (set-point in the map frame, speed limit or None, state).

        grid None means no map has arrived yet: fly straight (the vehicle is
        still above its launch pad, and the first scans arrive there).
        """
        p = self.params
        if math.hypot(goal_map[0] - position_map[0],
                      goal_map[1] - position_map[1]) <= p.goal_tolerance_m:
            self.state, self.path = NAV_ARRIVED, [tuple(goal_map)]
            self.unreachable_since = None
            return tuple(goal_map), None, self.state
        if grid is None:
            self.state = NAV_DIRECT
            return tuple(goal_map), None, self.state
        goal_changed = self.path_goal is None or math.hypot(
            goal_map[0] - self.path_goal[0],
            goal_map[1] - self.path_goal[1]) > 0.3
        blocked = bool(self.path) and not all(
            grid.segment_free(a, b) for a, b in zip(
                [tuple(position_map)] + self.path[1:-1], self.path[1:]))
        if blocked and not goal_changed:
            self.replans_blocked += 1
        if goal_changed or blocked or not self.path \
                or now - self.last_plan >= p.replan_period_s:
            t0 = clock() if clock else None
            path, note = astar(grid, position_map, goal_map, p)
            if clock:
                self.plan_ms.append(1000.0 * (clock() - t0))
            self.plans += 1
            self.last_plan = now
            self.path_goal = tuple(goal_map)
            self.note = note
            if path is None:
                self.path = []
                self.state = NAV_UNREACHABLE
                if self.unreachable_since is None:
                    self.unreachable_since = now
                return tuple(position_map), p.near_obstacle_speed_m_s, \
                    self.state
            self.path = path
        # The route may end at the nearest free cell instead of the goal (a
        # goal inside an obstacle's keep-out band). Once the vehicle stands
        # at that end, the goal itself cannot be reached: say so instead of
        # hovering there for the rest of the mission.
        end = self.path[-1]
        if math.hypot(end[0] - goal_map[0],
                      end[1] - goal_map[1]) > p.goal_tolerance_m \
                and math.hypot(end[0] - position_map[0],
                               end[1] - position_map[1]) \
                <= p.goal_tolerance_m:
            self.state = NAV_UNREACHABLE
            self.note = 'at the nearest reachable point to the goal'
            if self.unreachable_since is None:
                self.unreachable_since = now
            return tuple(position_map), p.near_obstacle_speed_m_s, \
                self.state
        self.unreachable_since = None
        self.state = NAV_FOLLOWING
        distance = p.lookahead_m
        carrot = lookahead_point(self.path, position_map, distance)
        while distance > 0.6 and not grid.segment_free(position_map, carrot):
            distance *= 0.6
            carrot = lookahead_point(self.path, position_map, distance)
        here = grid.cell(position_map)
        near = grid.inside(here) and \
            grid.clearance[here] < p.near_obstacle_m
        return carrot, (p.near_obstacle_speed_m_s if near else None), \
            self.state
