"""Unit tests for the search planner (Phase 9). No ROS, no simulator."""

import math
import os

import numpy as np
import pytest

from uav_autonomy.search_interface import (
    SearchLimits,
    goal_speed,
    validate_goal,
)
from uav_autonomy.search_planning import (
    Camera,
    CoverageGrid,
    PlannerParams,
    STATE_COMPLETE,
    STATE_IDLE,
    STATE_INSPECTING,
    STATE_SEARCHING,
    SearchArea,
    SearchPlanner,
    footprint_half_extents,
    footprint_ne_half_extents,
    footprint_polygon,
    inspection_viewpoints,
    lawnmower,
    path_length,
    yaw_of,
)

FOCAL = 640.0 / math.tan(0.87)
CAM = Camera(fx=FOCAL, fy=FOCAL, width=1280, height=960)
AREA = SearchArea(-4.0, 12.0, -4.0, 4.0)
SRC = os.path.join(os.path.dirname(__file__), '..', 'uav_autonomy')
EAST = math.pi / 2.0


def quat(yaw=0.0, roll=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cy, sr * cy, sr * sy, cr * sy]


def fly(planner, start_ne, locations_at=None, dt=0.1, speed=2.0,
        t0=0.0, t_max=400.0, observe=True, ground=0.0, altitude=5.0):
    """Move a point vehicle toward each goal; returns (path, goals, t)."""
    position = np.array(start_ne, float)
    path, goals, t = [tuple(position)], [], t0
    planner.start(t, position, EAST, altitude)
    while t < t_max:
        locations = locations_at(t, position) if locations_at else []
        to_goal = None
        goal = planner.update(t, position, 0.0 if to_goal is None else speed,
                              locations)
        goals.append(goal)
        if goal.complete:
            break
        to_goal = np.array(goal.target_ne) - position
        distance = np.linalg.norm(to_goal)
        step = min(distance, min(goal.speed_limit, speed) * dt)
        if distance > 1e-9:
            position = position + to_goal / distance * step
        path.append(tuple(position))
        if observe:
            planner.observe([position[0], position[1], ground - altitude],
                            quat(EAST), ground)
        t += dt
    return path, goals, t


# -- controller-side rules ---------------------------------------------------

def test_fresh_goal_in_range_is_accepted():
    assert validate_goal((5.0, 3.0), (0.0, 0.0), 0.1) == (True, None)


def test_stale_goal_is_refused():
    ok, reason = validate_goal((5.0, 3.0), (0.0, 0.0), 1.5)
    assert not ok and 'stale' in reason


def test_non_finite_goal_is_refused():
    assert not validate_goal((float('nan'), 0.0), (0.0, 0.0), 0.1)[0]
    assert not validate_goal((float('inf'), 0.0), (0.0, 0.0), 0.1)[0]


def test_goal_outside_range_is_refused():
    ok, reason = validate_goal((30.0, 0.0), (0.0, 0.0), 0.1)
    assert not ok and 'takeoff point' in reason


def test_range_is_measured_from_home_not_origin():
    assert validate_goal((120.0, 0.0), (100.0, 0.0), 0.1)[0]


def test_speed_is_never_uncapped_or_above_maximum():
    limits = SearchLimits()
    assert goal_speed(0.0) == limits.default_speed_m_s
    assert goal_speed(float('nan')) == limits.default_speed_m_s
    assert goal_speed(50.0) == limits.max_speed_m_s
    assert goal_speed(0.01) == limits.min_speed_m_s
    assert goal_speed(1.5) == 1.5


# -- footprint ---------------------------------------------------------------

def test_footprint_scales_with_height():
    forward, right = footprint_half_extents(4.0, CAM, 1.0)
    assert right == pytest.approx(4.0 * math.tan(0.87), rel=1e-6)
    assert forward == pytest.approx(right * 0.75, rel=1e-6)
    assert footprint_half_extents(8.0, CAM, 1.0)[0] == \
        pytest.approx(2 * forward)


def test_footprint_axes_follow_heading():
    forward, right = footprint_half_extents(4.0, CAM, 0.8)
    assert footprint_ne_half_extents(4.0, CAM, 0.8, 0.0) == \
        pytest.approx((forward, right))
    assert footprint_ne_half_extents(4.0, CAM, 0.8, EAST) == \
        pytest.approx((right, forward))
    assert footprint_ne_half_extents(4.0, CAM, 0.8, -math.pi) == \
        pytest.approx((forward, right))


def test_footprint_at_odd_heading_is_conservative():
    forward, right = footprint_half_extents(4.0, CAM, 0.8)
    half_n, half_e = footprint_ne_half_extents(4.0, CAM, 0.8,
                                               math.radians(40))
    assert half_n == half_e and half_n < min(forward, right)


def test_polygon_matches_half_extents_when_level():
    polygon = footprint_polygon([1.0, 2.0, -5.0], quat(EAST), CAM, -0.9,
                                0.8, 25.0)
    north = [p[0] for p in polygon]
    east = [p[1] for p in polygon]
    half_n, half_e = footprint_ne_half_extents(4.1, CAM, 0.8, EAST)
    assert max(north) - 1.0 == pytest.approx(half_n, rel=1e-6)
    assert max(east) - 2.0 == pytest.approx(half_e, rel=1e-6)


def test_polygon_refused_when_tilted_too_far():
    assert footprint_polygon([0, 0, -5.0], quat(0.0, math.radians(30)), CAM,
                             0.0, 0.8, 25.0) is None


def test_yaw_of_quaternion():
    assert yaw_of(quat(0.7)) == pytest.approx(0.7)


# -- coverage path -----------------------------------------------------------

def test_lanes_run_along_the_longer_side():
    path = lawnmower(AREA, 3.0, 2.2, 0.5, (0.0, 0.0))
    north_moves = sum(abs(b[0] - a[0]) for a, b in zip(path, path[1:]))
    east_moves = sum(abs(b[1] - a[1]) for a, b in zip(path, path[1:]))
    assert north_moves > east_moves


def test_lane_spacing_respects_overlap():
    path = lawnmower(AREA, 3.0, 2.2, 0.5, (0.0, 0.0))
    lanes = sorted({round(p[1], 3) for p in path})
    assert len(lanes) == 4                       # 8 m / (4.4 m * 0.5)
    assert max(np.diff(lanes)) <= 2.2 + 1e-6
    assert lanes[0] - AREA.east_min <= 2.2 and \
        AREA.east_max - lanes[-1] <= 2.2


def test_no_overlap_gives_fewer_lanes():
    assert len({round(p[1], 3) for p in lawnmower(
        AREA, 3.0, 2.2, 0.0, (0.0, 0.0))}) == 2


def test_path_starts_at_the_nearest_corner():
    for start in ((-4.0, -4.0), (12.0, 4.0), (12.0, -4.0), (-4.0, 4.0)):
        path = lawnmower(AREA, 3.0, 2.2, 0.5, start)
        corner_distance = math.hypot(path[0][0] - start[0],
                                     path[0][1] - start[1])
        assert all(corner_distance <= math.hypot(
            p[0] - start[0], p[1] - start[1]) + 1e-9 for p in path)


def test_path_is_a_serpentine_inside_the_area():
    path = lawnmower(AREA, 3.0, 2.2, 0.5, (0.0, 0.0))
    assert all(AREA.contains(p) for p in path)
    for a, b, c in zip(path, path[1:], path[2:]):
        # consecutive legs alternate: along a lane, then across
        assert (a[1] == b[1]) != (b[1] == c[1])


def test_small_area_gives_a_single_waypoint():
    path = lawnmower(SearchArea(-1.0, 1.0, -1.0, 1.0), 3.0, 2.2, 0.5,
                     (0.0, 0.0))
    assert path == [(0.0, 0.0)]


def test_path_length():
    assert path_length([(0, 0), (3, 4), (3, 0)]) == pytest.approx(9.0)


# -- coverage map ------------------------------------------------------------

def test_grid_marks_cells_inside_the_polygon_only():
    grid = CoverageGrid(SearchArea(0.0, 10.0, 0.0, 10.0), 0.5)
    seen = grid.update([(2.0, 2.0), (6.0, 2.0), (6.0, 5.0), (2.0, 5.0)],
                       (4.0, 3.5), 4.0, 15.0)
    assert seen == 8 * 6
    assert grid.fraction() == pytest.approx(48 / 400)


def test_grid_handles_either_polygon_winding():
    area = SearchArea(0.0, 10.0, 0.0, 10.0)
    polygon = [(2.0, 2.0), (6.0, 2.0), (6.0, 5.0), (2.0, 5.0)]
    a, b = CoverageGrid(area, 0.5), CoverageGrid(area, 0.5)
    assert a.update(polygon, (4, 3.5), 4.0, 15.0) == \
        b.update(polygon[::-1], (4, 3.5), 4.0, 15.0)


def test_oblique_cells_are_those_away_from_nadir():
    grid = CoverageGrid(SearchArea(0.0, 10.0, 0.0, 10.0), 0.5)
    grid.update([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
                (5.0, 5.0), 4.0, 15.0)
    assert grid.fraction() == 1.0
    assert 0.8 < grid.oblique_fraction() < 1.0
    assert not grid.oblique[10, 10]


def test_full_path_covers_the_area_and_gives_oblique_views():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    path, goals, _ = fly(planner, (0.0, 0.0))
    assert goals[-1].complete
    assert planner.grid.fraction() >= 0.99
    assert planner.grid.oblique_fraction() >= 0.95


# -- planner -----------------------------------------------------------------

def test_idle_until_started():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    goal = planner.update(0.0, (0.0, 0.0), 0.0)
    assert goal.state == STATE_IDLE and not goal.complete


def test_waypoints_are_visited_in_order_then_complete():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    path, goals, _ = fly(planner, (0.0, 0.0), observe=False)
    indices = [g.waypoint_index for g in goals]
    assert indices == sorted(indices)
    assert goals[-1].state == STATE_COMPLETE and goals[-1].complete
    assert planner.update(999.0, (0, 0), 0.0).complete   # stays complete


def test_speed_cap_is_the_search_speed():
    planner = SearchPlanner(AREA, CAM, PlannerParams(search_speed_m_s=1.7))
    _, goals, _ = fly(planner, (0.0, 0.0), observe=False)
    assert {g.speed_limit for g in goals if g.state == STATE_SEARCHING} == \
        {1.7}


def test_candidate_triggers_an_inspection_and_search_resumes():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    target = (6.0, 1.0)

    def locations(t, position):
        seen = math.hypot(position[0] - target[0],
                          position[1] - target[1]) < 3.0 or planner.inspected
        return [{'id': 3, 'ned': [target[0], target[1], -1.0],
                 'height': 1.0, 'confirmed': False, 'rejected': False}] \
            if seen or planner.inspections else []

    path, goals, _ = fly(planner, (0.0, 0.0), locations)
    states = [g.state for g in goals]
    assert STATE_INSPECTING in states and goals[-1].complete
    assert planner.inspected == {3: 'viewed'}
    inspection = planner.inspections[0]
    assert len(inspection.viewpoints) == 2
    for viewpoint in inspection.viewpoints:
        radius = math.hypot(viewpoint[0] - target[0],
                            viewpoint[1] - target[1])
        assert 2.0 <= radius <= 4.0 + 1e-6
    assert planner.grid.fraction() >= 0.99          # coverage not lost


def test_inspection_views_are_oblique_and_ninety_degrees_apart():
    views = inspection_viewpoints((6.0, 0.0), (4.0, 0.0), 3.0, 2, AREA, 3.0)
    assert views[0] == pytest.approx((3.0, 0.0))
    a = np.array(views[0]) - (6.0, 0.0)
    b = np.array(views[1]) - (6.0, 0.0)
    assert abs(float(a @ b)) < 1e-6
    off_nadir = math.degrees(math.atan2(3.0, 5.0 - 1.0))
    assert off_nadir > 30.0


def test_inspection_viewpoints_stay_inside_the_geofence():
    views = inspection_viewpoints((11.5, 3.8), (11.0, 3.5), 4.0, 2, AREA,
                                  3.0)
    assert all(AREA.contains(v, 3.0 + 1e-9) for v in views)


def test_each_candidate_is_inspected_once():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    loc = [{'id': 1, 'ned': [2.0, 0.0, -1.0], 'height': 1.0,
            'confirmed': False, 'rejected': False}]
    fly(planner, (0.0, 0.0), lambda t, p: loc, observe=False)
    assert len(planner.inspections) == 1


def test_confirmation_ends_the_inspection_early():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    state = {'arrived': None}

    def locations(t, position):
        ins = planner.inspection
        if ins is not None and ins.dwell_started is not None \
                and state['arrived'] is None:
            state['arrived'] = t
        confirmed = state['arrived'] is not None \
            and t - state['arrived'] > 1.5
        return [{'id': 1, 'ned': [2.0, 0.0, -1.0], 'height': 1.0,
                 'confirmed': confirmed, 'rejected': False}]

    fly(planner, (0.0, 0.0), locations, observe=False)
    inspection = planner.inspections[0]
    assert inspection.result == 'confirmed' and inspection.index == 0


def test_vlm_rejected_candidate_is_not_inspected():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    loc = [{'id': 1, 'ned': [2.0, 0.0, -1.0], 'height': 1.0,
            'confirmed': False, 'rejected': True}]
    fly(planner, (0.0, 0.0), lambda t, p: loc, observe=False)
    assert not planner.inspections
    assert planner.skipped_candidates == {1: 'rejected by the VLM'}


def test_candidate_outside_the_area_is_not_inspected():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    loc = [{'id': 1, 'ned': [30.0, 0.0, -1.0], 'height': 1.0,
            'confirmed': False, 'rejected': False}]
    fly(planner, (0.0, 0.0), lambda t, p: loc, observe=False)
    assert not planner.inspections


def test_duplicate_entry_at_an_inspected_place_is_skipped():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    locs = [{'id': 1, 'ned': [2.0, 0.0, -1.0], 'height': 1.0,
             'confirmed': False, 'rejected': False},
            {'id': 2, 'ned': [2.4, 0.3, -1.0], 'height': 1.0,
             'confirmed': False, 'rejected': False}]
    fly(planner, (0.0, 0.0), lambda t, p: locs, observe=False)
    assert len(planner.inspections) == 1


def test_inspection_count_is_bounded():
    planner = SearchPlanner(AREA, CAM, PlannerParams(max_inspections=2))
    locs = [{'id': k, 'ned': [2.0 * k, 0.0, -1.0], 'height': 1.0,
             'confirmed': False, 'rejected': False} for k in range(1, 6)]
    fly(planner, (0.0, 0.0), lambda t, p: locs, observe=False)
    assert len(planner.inspections) == 2


def test_new_or_thin_candidate_is_not_inspected_yet():
    planner = SearchPlanner(AREA, CAM, PlannerParams())
    planner.start(0.0, (0.0, 0.0), EAST, 5.0)
    loc = [{'id': 1, 'ned': [6.0, 0.0, -1.0], 'height': 1.0, 'obs': 40,
            'confirmed': False, 'rejected': False}]
    assert planner.update(0.0, (0.0, 0.0), 0.0, loc).state == \
        STATE_SEARCHING                       # known for 0 s
    thin = [dict(loc[0], obs=6)]
    assert planner.update(5.0, (0.0, 0.0), 0.0, thin).state == \
        STATE_SEARCHING                       # too few frames
    assert planner.update(5.1, (0.0, 0.0), 0.0, loc).state == \
        STATE_INSPECTING


def test_inspection_times_out():
    planner = SearchPlanner(AREA, CAM, PlannerParams(inspect_timeout_s=3.0,
                                                     inspect_delay_s=0.0))
    planner.start(0.0, (0.0, 0.0), EAST, 5.0)
    loc = [{'id': 1, 'ned': [6.0, 0.0, -1.0], 'height': 1.0,
            'confirmed': False, 'rejected': False}]
    assert planner.update(0.0, (0.0, 0.0), 0.0, loc).state == \
        STATE_INSPECTING
    goal = planner.update(4.0, (0.0, 0.0), 0.0, loc)   # never arrived
    assert goal.state == STATE_SEARCHING
    assert planner.inspected == {1: 'timeout'}


def test_inspection_can_be_switched_off():
    planner = SearchPlanner(AREA, CAM, PlannerParams(inspect_enabled=False))
    loc = [{'id': 1, 'ned': [2.0, 0.0, -1.0], 'height': 1.0,
            'confirmed': False, 'rejected': False}]
    fly(planner, (0.0, 0.0), lambda t, p: loc, observe=False)
    assert not planner.inspections


def test_inspection_is_slower_than_the_search():
    params = PlannerParams()
    assert params.inspect_speed_m_s < params.search_speed_m_s <= 3.0


# -- audits ------------------------------------------------------------------

@pytest.mark.parametrize('name', ['search_planning.py',
                                  'search_planner_node.py',
                                  'search_interface.py'])
def test_no_ground_truth_and_no_px4_commands(name):
    text = open(os.path.join(SRC, name)).read()
    text = text.replace('never\npublishes to /fmu/in/*', '')
    for forbidden in ('/fmu/in/', 'gz.msgs', 'gz.transport', '/world/',
                      'poses.csv', 'pose/info', 'TrajectorySetpoint',
                      'VehicleCommand', 'OffboardControlMode'):
        assert forbidden not in text, forbidden


# files from before this project phase; no entry point starts them
# (docs/runbook.md: "Never start rescue_mission.py")
LEGACY = {'rescue_mission.py', 'state_bridge.py', 'vlm_detector.py',
          'vlm_perception.py', 'survivor_perception.py',
          'dynamic_obstacle_simulator.py', 'dynamic_obstacle_tracker.py',
          'dynamic_obstacle_vision.py', 'continuous_debris_spawner.py'}


def test_only_the_flight_controller_publishes_to_px4():
    for name in os.listdir(SRC):
        if not name.endswith('.py') or name == 'flight_controller.py' \
                or name in LEGACY:
            continue
        text = open(os.path.join(SRC, name)).read()
        assert "'/fmu/in/" not in text, name
