"""Unit tests for survivor localisation (Phase 8). No ROS, no simulator."""

import math
import os

import numpy as np
import pytest

from uav_autonomy.survivor_localization import (
    CameraModel,
    LOC_COARSE,
    LOC_LOCALIZED,
    LocalizerParams,
    STATE_CANDIDATE,
    STATE_REJECTED,
    STATE_SURVIVOR_CONFIRMED,
    SurvivorLocalizer,
    camera_pose,
    horizontal_sigma,
    pixel_ray_body,
    project,
    projection_jacobian,
    quat_to_matrix,
    ray_plane,
    triangulate,
)

FOCAL = 640.0 / math.tan(0.87)
CAM = CameraModel(fx=FOCAL, fy=FOCAL, cx=640.0, cy=480.0, width=1280,
                  height=960)
SRC = os.path.join(os.path.dirname(__file__), '..', 'uav_autonomy')


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def observe(target, position, q, track_id=1, state=STATE_CANDIDATE,
            noise=None, box=120):
    """Pixel detection of a local point from a vehicle pose."""
    cam_position, rot = camera_pose(position, q, CAM)
    uv, depth = project(target, cam_position, rot, CAM)
    assert uv is not None and depth > 0
    if noise is not None:
        uv = uv + noise
    return {'track_id': track_id, 'center_u': float(uv[0]),
            'center_v': float(uv[1]), 'bbox_width': box, 'bbox_height': box,
            'touches_border': False, 'measured': True,
            'semantic_state': state}


def hover_run(localizer, target, position, n=40, t0=10.0, dt=0.07, **kw):
    q = quat(yaw=0.3)
    for i in range(n):
        localizer.update(t0 + i * dt, [observe(target, position, q, **kw)],
                         (position, q))
    return t0 + n * dt


# -- geometry ---------------------------------------------------------------

def test_centre_pixel_looks_straight_down():
    assert np.allclose(pixel_ray_body(640.0, 480.0, CAM), [0, 0, 1])


def test_image_axes_match_body_axes():
    assert pixel_ray_body(900.0, 480.0, CAM)[1] > 0      # right = body right
    assert pixel_ray_body(640.0, 200.0, CAM)[0] > 0      # up = body forward


def test_level_attitude_is_identity():
    assert np.allclose(quat_to_matrix(1, 0, 0, 0), np.eye(3))


def test_yaw_east_points_body_forward_east():
    rot = quat_to_matrix(*quat(yaw=math.pi / 2))
    assert np.allclose(rot @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_camera_is_below_the_body_origin():
    cam_position, _ = camera_pose([1.0, 2.0, -5.0], quat(), CAM)
    assert np.allclose(cam_position, [1.0, 2.0, -4.86])


def test_project_and_ray_are_inverse():
    position, q = np.array([2.0, -1.0, -6.0]), quat(0.1, -0.05, 1.0)
    cam_position, rot = camera_pose(position, q, CAM)
    target = np.array([3.5, 0.2, -1.0])
    uv, depth = project(target, cam_position, rot, CAM)
    ray = rot @ pixel_ray_body(uv[0], uv[1], CAM)
    direction = (target - cam_position) / np.linalg.norm(
        target - cam_position)
    assert depth > 0
    assert np.allclose(ray, direction, atol=1e-9)


def test_point_behind_camera_is_not_projected():
    cam_position, rot = camera_pose([0, 0, -5.0], quat(), CAM)
    uv, depth = project([0, 0, -9.0], cam_position, rot, CAM)
    assert uv is None and depth < 0


def test_jacobian_matches_finite_differences():
    position, q = np.array([0.5, 0.2, -5.0]), quat(0.05, 0.1, 0.4)
    cam_position, rot = camera_pose(position, q, CAM)
    target = np.array([1.5, -0.5, -1.2])
    jac = projection_jacobian(target, cam_position, rot, CAM)
    numeric = np.zeros((2, 3))
    for k in range(3):
        step = np.zeros(3)
        step[k] = 1e-5
        hi, _ = project(target + step, cam_position, rot, CAM)
        lo, _ = project(target - step, cam_position, rot, CAM)
        numeric[:, k] = (hi - lo) / 2e-5
    assert np.allclose(jac, numeric, rtol=1e-5, atol=1e-5)


def test_ray_plane_intersection():
    point, s = ray_plane(np.array([0, 0, -5.0]), np.array([0.6, 0, 0.8]),
                         -1.0)
    assert np.allclose(point, [3.0, 0, -1.0]) and s == pytest.approx(5.0)


def test_ray_plane_rejects_upward_and_behind():
    assert ray_plane(np.zeros(3), np.array([1.0, 0, 0]), 1.0)[0] is None
    assert ray_plane(np.array([0, 0, 2.0]), np.array([0, 0, 1.0]),
                     1.0)[0] is None


def test_triangulate_two_views():
    target = np.array([2.0, 1.0, -0.5])
    origins = np.array([[0, 0, -5.0], [4.0, 0, -5.0]])
    rays = (target - origins)
    rays /= np.linalg.norm(rays, axis=1)[:, None]
    point, cond = triangulate(origins, rays)
    assert np.allclose(point, target, atol=1e-9) and cond < 100


def test_triangulate_parallel_rays_is_refused():
    origins = np.array([[0, 0, -5.0], [0, 0, -5.0]])
    rays = np.array([[0, 0, 1.0], [0, 0, 1.0]])
    assert triangulate(origins, rays)[0] is None


def test_horizontal_sigma_is_worst_direction():
    assert horizontal_sigma(np.diag([0.04, 0.25, 9.0])) == pytest.approx(0.5)


# -- single-view behaviour --------------------------------------------------

def test_hover_estimate_is_on_the_assumed_height_plane():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    target = np.array([1.2, 0.0, -0.9])       # at the assumed height
    hover_run(loc, target, np.array([0.0, 0.0, -5.0]))
    entity = loc.reported()[0]
    assert np.linalg.norm(entity.position - target) < 0.05


def test_hover_height_error_gives_bounded_horizontal_error():
    """Point at 1.5 m, assumed 0.9 m: error = 0.6 m * tan(off-nadir)."""
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    target = np.array([2.0, 0.0, -1.5])
    hover_run(loc, target, np.array([0.0, 0.0, -5.0]))
    entity = loc.reported()[0]
    error = np.linalg.norm(entity.position[:2] - target[:2])
    expected = 0.6 * 2.0 / (5.0 - 0.14 - 1.5)
    assert error == pytest.approx(expected, abs=0.08)
    assert error < 2.0 * loc.reported_sigma(entity) + 0.2


def test_hover_does_not_claim_a_measured_height():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    hover_run(loc, np.array([1.0, 0.5, -0.9]), np.array([0.0, 0.0, -5.0]))
    entity = loc.reported()[0]
    assert math.sqrt(entity.covariance[2, 2]) > 0.3
    assert entity.baseline_m < 0.01


def test_uncertainty_does_not_collapse_with_many_correlated_frames():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    hover_run(loc, np.array([1.0, 0.5, -0.9]), np.array([0.0, 0.0, -5.0]),
              n=400, dt=0.033)
    assert loc.reported_sigma(loc.reported()[0]) >= \
        LocalizerParams().systematic_sigma_m


# -- multi-view behaviour ---------------------------------------------------

def test_moving_vehicle_recovers_the_true_height():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    target = np.array([6.0, 1.0, -1.6])        # 0.7 m above the assumption
    q = quat(yaw=0.0)
    for i in range(120):
        position = np.array([0.1 * i, 0.0, -5.0])
        if abs(position[0] - target[0]) > 4.5:
            continue
        loc.update(10.0 + 0.2 * i, [observe(target, position, q)],
                   (position, q))
    entity = loc.reported()[0]
    assert np.linalg.norm(entity.position - target) < 0.1
    assert entity.baseline_m > 5.0
    assert loc.state_of(entity) == LOC_LOCALIZED


def test_noisy_moving_run_is_consistent_with_reported_sigma():
    rng = np.random.default_rng(3)
    errors, sigmas = [], []
    for trial in range(20):
        loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
        target = np.array([5.0, rng.uniform(-2, 2), -rng.uniform(0.2, 1.7)])
        for i in range(90):
            position = np.array([1.0 + 0.1 * i, 0.0, -5.0])
            q = quat(rng.normal(0, 0.01), rng.normal(0, 0.01), 0.0)
            noise = rng.normal(0, 8.0, 2)
            true_q = quat()
            det = observe(target, position, true_q, noise=noise)
            loc.update(5.0 + 0.1 * i, [det], (position, q))
        entity = loc.reported()[0]
        errors.append(np.linalg.norm(entity.position[:2] - target[:2]))
        sigmas.append(loc.reported_sigma(entity))
    errors, sigmas = np.array(errors), np.array(sigmas)
    assert np.median(errors) < 0.15
    assert (errors <= 2.0 * sigmas).mean() >= 0.9


def test_climb_gives_a_vertical_baseline():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    target = np.array([1.5, 0.0, -1.5])
    q = quat()
    for i in range(80):
        position = np.array([0.0, 0.0, -2.5 - 0.05 * i])
        loc.update(3.0 + 0.1 * i, [observe(target, position, q)],
                   (position, q))
    entity = loc.reported()[0]
    assert np.linalg.norm(entity.position - target) < 0.15


# -- gating and skipping ----------------------------------------------------

def test_frame_without_pose_is_skipped():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    det = observe(np.array([1.0, 0, -0.9]), np.array([0, 0, -5.0]), quat())
    assert loc.update(1.0, [det], None) == []
    assert loc.skipped['no_pose'] == 1 and not loc.entities


def test_border_and_held_detections_are_not_used():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    position, q = np.array([0, 0, -5.0]), quat()
    det = observe(np.array([1.0, 0, -0.9]), position, q)
    loc.update(1.0, [dict(det, touches_border=True)], (position, q))
    loc.update(1.1, [dict(det, measured=False)], (position, q))
    assert not loc.entities
    assert loc.skipped['border'] == 1 and loc.skipped['not_measured'] == 1


def test_large_tilt_is_skipped():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    position, q = np.array([0, 0, -5.0]), quat(roll=math.radians(40))
    det = {'track_id': 1, 'center_u': 640.0, 'center_v': 480.0,
           'bbox_width': 100, 'bbox_height': 100, 'touches_border': False,
           'measured': True, 'semantic_state': STATE_CANDIDATE}
    loc.update(1.0, [det], (position, q))
    assert loc.skipped['tilt'] == 1 and not loc.entities


def test_outlier_pixel_is_gated():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    target, position = np.array([1.0, 0, -0.9]), np.array([0, 0, -5.0])
    hover_run(loc, target, position, n=20)
    before = loc.reported()[0].position.copy()
    bad = observe(target, position, quat(yaw=0.3),
                  noise=np.array([400.0, 0.0]))
    loc.update(20.0, [bad], (position, quat(yaw=0.3)))
    assert loc.skipped['gate'] == 1
    assert np.allclose(loc.reported()[0].position, before)


def test_not_reported_before_min_observations():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    hover_run(loc, np.array([1.0, 0, -0.9]), np.array([0, 0, -5.0]), n=3)
    assert loc.reported() == []


def test_single_view_state_is_coarse_when_far():
    params = LocalizerParams(localized_sigma_m=0.3)
    loc = SurvivorLocalizer(CAM, params, ground_down=0.0)
    hover_run(loc, np.array([8.0, 0, -0.9]), np.array([0, 0, -12.0]), n=6,
              box=40)
    assert loc.state_of(loc.reported()[0]) == LOC_COARSE


# -- entities ---------------------------------------------------------------

def test_new_track_id_of_same_object_joins_the_entity():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    target, position = np.array([1.0, 0, -0.9]), np.array([0, 0, -5.0])
    t = hover_run(loc, target, position, n=20, track_id=1)
    hover_run(loc, target, position, n=20, t0=t + 2.0, track_id=7)
    assert len(loc.reported()) == 1
    assert loc.reported()[0].track_ids == [1, 7]


def test_objects_one_metre_apart_stay_separate():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    position, q = np.array([0, 0, -5.0]), quat()
    a, b = np.array([1.2, 0.0, -0.9]), np.array([0.35, 0.65, -0.9])
    for i in range(30):
        loc.update(1.0 + 0.07 * i, [observe(a, position, q, track_id=1),
                                    observe(b, position, q, track_id=2)],
                   (position, q))
    assert len(loc.reported()) == 2


def test_confirmation_is_sticky_and_not_shared_with_neighbours():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    position, q = np.array([0, 0, -5.0]), quat()
    a, b = np.array([1.2, 0.0, -0.9]), np.array([0.35, 0.65, -0.9])
    for i in range(30):
        state = STATE_SURVIVOR_CONFIRMED if 10 <= i < 20 else STATE_CANDIDATE
        loc.update(1.0 + 0.07 * i, [
            observe(a, position, q, track_id=1, state=state),
            observe(b, position, q, track_id=2, state=STATE_REJECTED)],
            (position, q))
    by_track = {e.track_ids[0]: e for e in loc.reported()}
    assert by_track[1].confirmed and not by_track[1].vlm_rejected
    assert not by_track[2].confirmed and by_track[2].vlm_rejected


def test_duplicate_entities_are_merged():
    loc = SurvivorLocalizer(CAM, LocalizerParams(association_chi2=0.0),
                            ground_down=0.0)
    target, position = np.array([1.0, 0, -0.9]), np.array([0, 0, -5.0])
    t = hover_run(loc, target, position, n=10, track_id=1)
    hover_run(loc, target, position, n=10, t0=t, track_id=2,
              state=STATE_SURVIVOR_CONFIRMED)
    assert len(loc.reported()) == 1
    entity = loc.reported()[0]
    assert sorted(entity.track_ids) == [1, 2] and entity.confirmed


def test_short_blip_is_dropped_after_timeout():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    position, q = np.array([0, 0, -5.0]), quat()
    loc.update(1.0, [observe(np.array([1.0, 0, -0.9]), position, q)],
               (position, q))
    assert len(loc.entities) == 1
    loc.update(8.0, [], (position, q))
    assert not loc.entities


def test_established_entity_is_kept_when_out_of_view():
    loc = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    position, q = np.array([0, 0, -5.0]), quat()
    hover_run(loc, np.array([1.0, 0, -0.9]), position, n=20)
    loc.update(200.0, [], (position, q))
    assert len(loc.reported()) == 1


def test_ground_level_shifts_the_height_prior():
    low = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.0)
    high = SurvivorLocalizer(CAM, LocalizerParams(), ground_down=0.5)
    for loc in (low, high):
        hover_run(loc, np.array([1.0, 0, -0.9]), np.array([0, 0, -5.0]), n=6)
    assert high.reported()[0].position[2] - low.reported()[0].position[2] \
        == pytest.approx(0.5, abs=0.05)


# -- audits -----------------------------------------------------------------

@pytest.mark.parametrize('name', ['survivor_localization.py',
                                  'survivor_localization_node.py'])
def test_no_ground_truth_and_no_px4_commands(name):
    text = open(os.path.join(SRC, name)).read()
    for forbidden in ('/fmu/in/', 'gz.msgs', 'gz.transport', '/world/',
                      'poses.csv', 'pose/info', 'create_client'):
        assert forbidden not in text.replace(
            'never publishes to /fmu/in/*', '')
    assert 'TrajectorySetpoint' not in text and 'VehicleCommand' not in text


def test_node_only_subscribes_to_own_sensors_and_estimates():
    text = open(os.path.join(SRC, 'survivor_localization_node.py')).read()
    topics = [line.split("'")[1] for line in text.splitlines()
              if "'/" in line and ('camera' in line or 'fmu' in line
                                   or 'perception' in line)
              and 'create' not in line and line.strip().startswith(
                  ("CameraInfo, '", "VehicleOdometry, '",
                   "SurvivorDetectionArray, '"))]
    assert sorted(topics) == ['/camera/camera_info',
                              '/fmu/out/vehicle_odometry',
                              '/perception/survivor/detections']
