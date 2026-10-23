"""Unit tests for SLAM geometry and map-aware navigation (village).

No ROS, no simulator.
"""

import math
import os

import numpy as np
import pytest

from uav_autonomy.navigation import (
    NAV_ARRIVED,
    NAV_DIRECT,
    NAV_FOLLOWING,
    NAV_UNREACHABLE,
    NavParams,
    Navigator,
    PlanningGrid,
    Pose2D,
    astar,
    lookahead_point,
    map_to_px4,
    path_length,
    px4_to_map,
)
from uav_autonomy.slam_geometry import (
    DriftModel,
    enu_flu_rotation,
    level_scan,
    ned_to_enu_xy,
    yaw_enu,
)

SRC = os.path.join(os.path.dirname(__file__), '..', 'uav_autonomy')
P = NavParams()


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def world(walls=(), size=(160, 160), resolution=0.25, origin=(-10.0, -10.0),
          known=True):
    """An occupancy grid; walls are (x0, y0, x1, y1) boxes in metres."""
    data = np.zeros(size, dtype=np.int8) if known else np.full(
        size, -1, dtype=np.int8)
    for x0, y0, x1, y1 in walls:
        c0, c1 = int((x0 - origin[0]) / resolution), int(
            (x1 - origin[0]) / resolution)
        r0, r1 = int((y0 - origin[1]) / resolution), int(
            (y1 - origin[1]) / resolution)
        data[r0:r1 + 1, c0:c1 + 1] = 100
    return data, resolution, origin


def grid_for(walls=(), points=((0.0, 0.0), (20.0, 20.0)), **kw):
    data, resolution, origin = world(walls, **kw)
    return PlanningGrid(data, resolution, origin, points, P)


# -- frames -------------------------------------------------------------------

def test_ned_to_enu_position():
    assert ned_to_enu_xy([3.0, 7.0, -5.0]) == (7.0, 3.0)   # (east, north)


def test_heading_north_in_ned_is_ninety_degrees_in_enu():
    assert yaw_enu(enu_flu_rotation(quat(yaw=0.0))) == pytest.approx(
        math.pi / 2)
    assert yaw_enu(enu_flu_rotation(quat(yaw=math.pi / 2))) == \
        pytest.approx(0.0, abs=1e-9)                        # east


def test_level_attitude_keeps_body_up_pointing_up():
    rot = enu_flu_rotation(quat(yaw=0.7))
    assert rot[2, 2] == pytest.approx(1.0)


# -- scan levelling -----------------------------------------------------------

def scan(value=10.0, n=720):
    return np.full(n, value), -math.pi, 2 * math.pi / n


def test_level_scan_is_identity_when_level():
    ranges, a0, da = scan()
    out, dropped = level_scan(ranges, a0, da, enu_flu_rotation(quat()), 6.0,
                              0.6, 30.0)
    assert dropped == 0 and np.allclose(out, 10.0)


def test_tilted_scan_drops_returns_far_from_the_flight_plane():
    ranges, a0, da = scan(20.0)
    rot = enu_flu_rotation(quat(pitch=math.radians(10)))
    out, dropped = level_scan(ranges, a0, da, rot, 6.0, 0.6, 30.0,
                              max_plane_offset_m=0.8)
    # 20 m * sin(10 deg) = 3.5 m off the plane fore and aft: dropped;
    # the rays to the sides stay in the plane
    assert dropped > 300
    kept = np.isfinite(out)
    assert kept.any() and np.all(out[kept] <= 20.0 + 1e-6)


def test_tilted_scan_shortens_ranges_to_their_horizontal_projection():
    ranges, a0, da = scan(4.0)
    rot = enu_flu_rotation(quat(pitch=math.radians(8)))
    out, _ = level_scan(ranges, a0, da, rot, 6.0, 0.6, 30.0)
    kept = out[np.isfinite(out)]
    assert kept.min() == pytest.approx(4.0 * math.cos(math.radians(8)),
                                       abs=0.02)


def test_ground_returns_are_dropped():
    ranges, a0, da = scan(25.0)
    rot = enu_flu_rotation(quat(roll=math.radians(4)))
    out, dropped = level_scan(ranges, a0, da, rot, 1.8, 0.6, 30.0,
                              max_plane_offset_m=5.0, min_height_m=1.5)
    assert dropped > 0          # 25 m * sin(4 deg) = 1.7 m down: below 1.5 m


def test_invalid_ranges_stay_empty():
    ranges, a0, da = scan()
    ranges[:100] = np.inf
    ranges[100:120] = 0.1
    out, _ = level_scan(ranges, a0, da, enu_flu_rotation(quat()), 6.0, 0.6,
                        30.0)
    assert np.isinf(out[:120]).all() and np.isfinite(out[130:]).all()


# -- drift model --------------------------------------------------------------

def test_no_drift_is_the_identity():
    model = DriftModel()
    for i in range(50):
        assert model.update(i * 0.1, i * 0.2, i * 0.1, 0.3) == \
            pytest.approx((i * 0.2, i * 0.1, 0.3))


def test_scale_error_stretches_the_travelled_distance():
    model = DriftModel(scale_error=0.05)
    out = None
    for i in range(101):
        out = model.update(i * 0.1, i * 0.1, 0.0, 0.0)
    assert out[0] == pytest.approx(10.5) and out[1] == pytest.approx(0.0)


def test_yaw_drift_bends_a_straight_path():
    model = DriftModel(yaw_rate_rad_s=math.radians(1.0))
    out = None
    for i in range(301):
        out = model.update(i * 0.1, i * 0.1, 0.0, 0.0)
    assert out[2] == pytest.approx(math.radians(30.0))
    assert out[1] > 5.0 and out[0] < 30.0


# -- pose arithmetic ----------------------------------------------------------

def test_point_round_trip_between_frames():
    px4, slam = Pose2D(40.0, 30.0, 1.2), Pose2D(43.5, 25.0, 1.09)
    point = (55.0, 41.0)
    assert map_to_px4(px4_to_map(point, px4, slam), px4, slam) == \
        pytest.approx(point)


def test_vehicle_position_maps_onto_itself():
    px4, slam = Pose2D(40.0, 30.0, 1.2), Pose2D(43.5, 25.0, 1.09)
    assert px4_to_map((px4.x, px4.y), px4, slam) == pytest.approx(
        (slam.x, slam.y))


def test_rotated_map_frame_is_handled():
    """A point 10 m ahead of the vehicle is 10 m ahead in both frames."""
    px4, slam = Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, math.radians(-6.4))
    ahead = px4_to_map((10.0, 0.0), px4, slam)
    assert math.hypot(*ahead) == pytest.approx(10.0)
    assert math.degrees(math.atan2(ahead[1], ahead[0])) == \
        pytest.approx(-6.4)


# -- planning grid ------------------------------------------------------------

def test_clearance_and_keep_out_band():
    grid = grid_for([(10.0, 0.0, 10.5, 20.0)])
    assert grid.blocked[grid.cell((10.2, 10.0))]
    assert grid.blocked[grid.cell((11.5, 10.0))]         # inside 1.75 m
    assert not grid.blocked[grid.cell((14.0, 10.0))]
    assert grid.clearance[grid.cell((14.0, 10.0))] == pytest.approx(3.5,
                                                                   abs=0.6)


def test_grid_grows_to_contain_start_and_goal():
    grid = grid_for(points=((0.0, 0.0), (90.0, 70.0)))
    assert grid.inside(grid.cell((90.0, 70.0)))
    assert not grid.known[grid.cell((90.0, 70.0))]       # never observed


def test_segment_check():
    grid = grid_for([(10.0, 0.0, 10.5, 20.0)])
    assert not grid.segment_free((5.0, 10.0), (15.0, 10.0))
    assert grid.segment_free((5.0, 25.0), (15.0, 25.0))


# -- route planning -----------------------------------------------------------

def test_straight_route_in_free_space():
    path, note = astar(grid_for(), (0.0, 0.0), (20.0, 15.0), P)
    assert note == '' and len(path) == 2
    assert path_length(path) == pytest.approx(25.0, abs=0.1)


def test_route_goes_round_a_wall_and_keeps_its_distance():
    grid = grid_for([(10.0, -5.0, 10.5, 12.0)])
    path, _ = astar(grid, (0.0, 0.0), (20.0, 0.0), P)
    # the direct line is 20 m; round the wall's end it is about 27 m
    assert path is not None and 24.0 < path_length(path) < 32.0
    for a, b in zip(path[:-1], path[1:]):
        assert grid.segment_free(a, b)
        for f in np.linspace(0, 1, 20):
            cell = grid.cell((a[0] + f * (b[0] - a[0]),
                              a[1] + f * (b[1] - a[1])))
            assert grid.clearance[cell] >= P.inflation_m - grid.res


def test_route_through_a_gap_wide_enough():
    walls = [(10.0, -5.0, 10.5, 4.0), (10.0, 10.0, 10.5, 25.0)]   # 6 m gap
    path, _ = astar(grid_for(walls), (0.0, 7.0), (20.0, 7.0), P)
    assert path is not None and path_length(path) < 22.0


def test_gap_too_narrow_is_not_used():
    walls = [(10.0, -8.0, 10.5, 6.0), (10.0, 8.5, 10.5, 28.0)]    # 2.5 m gap
    grid = grid_for(walls)
    path, _ = astar(grid, (0.0, 7.0), (20.0, 7.0), P)
    assert path is not None and path_length(path) > 30.0


def test_enclosed_goal_is_unreachable():
    walls = [(14.0, 14.0, 26.0, 14.5), (14.0, 25.5, 26.0, 26.0),
             (14.0, 14.0, 14.5, 26.0), (25.5, 14.0, 26.0, 26.0)]
    path, note = astar(grid_for(walls), (0.0, 0.0), (20.0, 20.0), P)
    assert path is None and 'no route' in note


def test_goal_inside_an_obstacle_moves_to_the_nearest_free_cell():
    grid = grid_for([(18.0, 18.0, 20.0, 20.0)])
    path, note = astar(grid, (0.0, 0.0), (19.0, 20.8), P)
    assert path is not None and 'nearest free cell' in note
    assert not grid.blocked[grid.cell(path[-1])]
    assert math.hypot(path[-1][0] - 19.0, path[-1][1] - 20.8) <= \
        P.goal_snap_m + grid.res


def test_goal_deep_inside_an_obstacle_is_refused():
    grid = grid_for([(10.0, 10.0, 24.0, 24.0)])
    path, note = astar(grid, (0.0, 0.0), (17.0, 17.0), P)
    assert path is None and 'inside an obstacle' in note


def test_start_inside_the_keep_out_band_can_leave():
    grid = grid_for([(10.0, 0.0, 10.5, 20.0)])
    path, _ = astar(grid, (9.0, 10.0), (0.0, 10.0), P)     # 1 m from a wall
    assert path is not None and path[-1] == (0.0, 10.0)


def test_unknown_space_is_crossed_but_costs_more():
    """Optimistic planning: an unobserved map does not stop the vehicle."""
    data, resolution, origin = world(known=False)
    grid = PlanningGrid(data, resolution, origin, [(0, 0), (20, 20)], P)
    path, _ = astar(grid, (0.0, 0.0), (20.0, 20.0), P)
    assert path is not None
    assert grid.cost[grid.cell((5.0, 5.0))] > 1.0


# -- path following -----------------------------------------------------------

def test_lookahead_moves_along_the_path():
    path = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    assert lookahead_point(path, (2.0, 0.3), 3.0) == pytest.approx((5.0,
                                                                    0.0))
    assert lookahead_point(path, (9.0, 0.0), 3.0) == pytest.approx((10.0,
                                                                    2.0))
    assert lookahead_point(path, (10.0, 9.5), 3.0) == pytest.approx((10.0,
                                                                     10.0))


def test_navigator_flies_straight_before_any_map_exists():
    nav = Navigator(P)
    target, limit, state = nav.update(0.0, None, (0.0, 0.0), (30.0, 0.0))
    assert state == NAV_DIRECT and target == (30.0, 0.0)


def test_navigator_follows_and_arrives():
    nav = Navigator(P)
    grid = grid_for([(10.0, -5.0, 10.5, 12.0)])
    position, t, states = np.array([0.0, 0.0]), 0.0, set()
    for _ in range(600):
        target, limit, state = nav.update(t, grid, tuple(position),
                                          (20.0, 0.0))
        states.add(state)
        if state == NAV_ARRIVED:
            break
        step = np.array(target) - position
        d = np.linalg.norm(step)
        position = position + step / max(d, 1e-9) * min(d, 0.2)
        assert not grid.occupied[grid.cell(tuple(position))]
        t += 0.1
    assert NAV_FOLLOWING in states and state == NAV_ARRIVED


def test_navigator_replans_when_a_wall_appears_on_the_route():
    nav = Navigator(P)
    free = grid_for()
    nav.update(0.0, free, (0.0, 0.0), (20.0, 0.0))
    assert len(nav.path) == 2
    blocked = grid_for([(10.0, -5.0, 10.5, 12.0)])
    target, _, state = nav.update(0.2, blocked, (1.0, 0.0), (20.0, 0.0))
    assert state == NAV_FOLLOWING and nav.replans_blocked == 1
    assert len(nav.path) > 2 and blocked.segment_free((1.0, 0.0), target)


def test_navigator_reports_unreachable_and_holds_position():
    nav = Navigator(P)
    walls = [(14.0, 14.0, 26.0, 14.5), (14.0, 25.5, 26.0, 26.0),
             (14.0, 14.0, 14.5, 26.0), (25.5, 14.0, 26.0, 26.0)]
    target, limit, state = nav.update(5.0, grid_for(walls), (0.0, 0.0),
                                      (20.0, 20.0))
    assert state == NAV_UNREACHABLE and target == (0.0, 0.0)
    assert nav.unreachable_since == 5.0


def test_navigator_reports_a_goal_it_can_only_get_near():
    """Seen in flight (village_B r1): the route ended at the nearest free
    cell, the vehicle stood there, and the mission waited until its time
    budget ran out. Standing at the end of such a route is 'unreachable'."""
    nav = Navigator(P)
    grid = grid_for([(18.0, 18.0, 20.0, 20.0)])
    goal = (19.0, 20.8)
    _, _, state = nav.update(1.0, grid, (0.0, 0.0), goal)
    assert state == 'FOLLOWING' and nav.unreachable_since is None
    end = nav.path[-1]
    assert math.hypot(end[0] - goal[0], end[1] - goal[1]) > P.goal_tolerance_m
    target, _, state = nav.update(20.0, grid, end, goal)
    assert state == NAV_UNREACHABLE and target == end
    assert nav.unreachable_since == 20.0
    _, _, state = nav.update(26.0, grid, end, goal)
    assert state == NAV_UNREACHABLE and nav.unreachable_since == 20.0


def test_backtrack_retraces_the_flown_path_in_reverse():
    """Seen in flight (urban_B): the vehicle ended in a pocket the map had
    closed behind it, no route home existed, and the controller's climb hit
    a floor slab. The navigator retraces its own crumbs instead."""
    nav = Navigator(P)
    for x in range(0, 31):
        nav.record((float(x), 0.0))
    for y in range(1, 21):
        nav.record((30.0, float(y)))
    assert len(nav.crumbs) < 51 and nav.crumbs[0] == (0.0, 0.0)
    here = (30.0, 20.0)
    first = nav.backtrack(here)
    assert math.hypot(first[0] - 30.0, first[1] - 18.5) < 1.6   # just behind
    # while backtracking, nothing new is recorded
    nav.record((99.0, 99.0))
    assert (99.0, 99.0) not in nav.crumbs
    # moving along the crumbs walks the index down to the start
    seen = []
    for pos in reversed(nav.crumbs):
        sp = nav.backtrack(pos)
        if sp is not None:
            seen.append(sp)
    assert seen and seen[-1][0] < 3.0 and seen[-1][1] == 0.0
    assert nav.backtrack((0.2, 0.0)) is None          # used up at the start
    nav.stop_backtrack()
    nav.record((50.0, 50.0))
    assert (50.0, 50.0) in nav.crumbs


def test_crumbs_are_spaced():
    nav = Navigator(P)
    for i in range(1000):
        nav.record((i * 0.05, 0.0))                    # 50 m in 5 cm steps
    assert 30 <= len(nav.crumbs) <= 36


def test_navigator_slows_down_near_obstacles():
    nav = Navigator(P)
    grid = grid_for([(10.0, 0.0, 10.5, 20.0)])
    _, limit, _ = nav.update(0.0, grid, (7.6, 10.0), (0.0, 10.0))
    assert limit == P.near_obstacle_speed_m_s
    _, limit, _ = Navigator(P).update(0.0, grid, (0.0, 10.0), (4.0, 10.0))
    assert limit is None


# -- localisation health ------------------------------------------------------

def test_localisation_needs_a_fresh_slam_stamp():
    nav = Navigator(P)
    pose = Pose2D(0, 0, 0)
    assert not nav.localization_ok(10.0, None, pose, None)[0]
    assert not nav.localization_ok(20.0, 10.0, pose, pose)[0]
    assert nav.localization_ok(11.0, 10.0, pose, pose)[0]


def test_rotated_map_frame_far_from_origin_is_not_a_fault():
    """The first village flight failed on exactly this: the map frame was
    6.4 degrees off the PX4 frame, 6 m apart at 55 m. That is a frame
    offset, not a localisation fault."""
    nav = Navigator(P)
    rot = math.radians(-6.4)
    for i in range(200):
        t = i * 0.1
        x = 0.4 * i
        px4 = Pose2D(x, 0.0, 0.0)
        slam = Pose2D(x * math.cos(rot), x * math.sin(rot), rot)
        ok, reason = nav.localization_ok(t, t, px4, slam)
        assert ok, reason
    assert math.hypot(slam.x - px4.x, slam.y - px4.y) > 6.0
    assert nav.largest_jump_m < 0.1


def test_a_sudden_slam_jump_is_a_fault():
    nav = Navigator(P)
    for i in range(40):
        t = i * 0.1
        pose = Pose2D(0.2 * i, 0.0, 0.0)
        assert nav.localization_ok(t, t, pose, pose)[0]
    ok, reason = nav.localization_ok(4.0, 4.0, Pose2D(8.0, 0.0, 0.0),
                                     Pose2D(8.0, 4.0, 0.0))
    assert not ok and 'jumped' in reason


def test_a_loop_closure_sized_correction_is_tolerated():
    nav = Navigator(P)
    for i in range(40):
        t = i * 0.1
        pose = Pose2D(0.2 * i, 0.0, 0.0)
        nav.localization_ok(t, t, pose, pose)
    assert nav.localization_ok(4.0, 4.0, Pose2D(8.0, 0.0, 0.0),
                               Pose2D(8.0, 1.3, 0.0))[0]


# -- audits -------------------------------------------------------------------

@pytest.mark.parametrize('name', ['navigation.py', 'navigator_node.py',
                                  'slam_geometry.py', 'slam_bridge_node.py'])
def test_no_ground_truth_and_no_px4_commands(name):
    text = open(os.path.join(SRC, name)).read()
    text = text.replace('never publishes to /fmu/in/*', '')
    for forbidden in ('/fmu/in/', 'gz.msgs', 'gz.transport', '/world/',
                      'poses.csv', 'pose/info', 'TrajectorySetpoint',
                      'VehicleCommand', 'OffboardControlMode'):
        assert forbidden not in text, forbidden
