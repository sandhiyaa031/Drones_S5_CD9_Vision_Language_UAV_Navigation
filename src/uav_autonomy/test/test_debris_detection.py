"""Unit tests for depth-based debris detection using synthetic scenes."""

import math

import numpy as np
import pytest

from uav_autonomy.debris_detection import (
    DebrisDetector,
    DetectorParams,
    Intrinsics,
    PoseBuffer,
    StaticFilter,
    backproject,
    depth_to_debug_image,
    detect_frame,
    sensor_to_body_frd,
    sensor_to_local,
)

W, H = 640, 480
FX = (W / 2) / math.tan(math.radians(100) / 2)
K = Intrinsics(fx=FX, fy=FX, cx=320.0, cy=240.0, width=W, height=H)
IDENTITY_Q = np.array([1.0, 0.0, 0.0, 0.0])


def sky():
    return np.full((H, W), np.inf, dtype=np.float32)


def add_box(depth, centre_s, size, d_face=None):
    """Render the near face of a box (sensor frame centre, metres)."""
    x, y, z = centre_s
    face = x - size / 2 if d_face is None else d_face
    u0 = K.cx - K.fx * (y + size / 2) / face
    u1 = K.cx - K.fx * (y - size / 2) / face
    v0 = K.cy - K.fy * (z + size / 2) / face
    v1 = K.cy - K.fy * (z - size / 2) / face
    r0, r1 = max(0, int(round(v0))), min(H, int(round(v1)))
    c0, c1 = max(0, int(round(u0))), min(W, int(round(u1)))
    region = depth[r0:r1, c0:c1]
    region[region > face] = face
    return face


def test_open_sky_gives_nothing():
    result = detect_frame(sky(), K)
    assert result.detections == [] and result.valid_pixels == 0


def test_single_box_position_and_size():
    depth = sky()
    face = add_box(depth, (5.2, 1.0, -0.5), 0.4)
    result = detect_frame(depth, K)
    assert len(result.detections) == 1
    det = result.detections[0]
    assert det.position_sensor[0] == pytest.approx(face, abs=1e-4)
    assert det.position_sensor[1] == pytest.approx(1.0, abs=0.03)
    assert det.position_sensor[2] == pytest.approx(-0.5, abs=0.03)
    assert det.size_m == pytest.approx(0.4, abs=0.05)
    assert not det.touches_border
    assert 0.5 < det.confidence <= 1.0


def test_two_separate_boxes():
    depth = sky()
    add_box(depth, (6.0, 1.5, 0.0), 0.4)
    add_box(depth, (4.0, -1.0, 1.0), 0.3)
    result = detect_frame(depth, K)
    ranges = sorted(d.position_sensor[0] for d in result.detections)
    assert ranges == pytest.approx([3.85, 5.8], abs=1e-3)


def test_overlapping_boxes_at_different_depths_are_split():
    depth = sky()
    add_box(depth, (8.0, 0.0, 0.0), 0.6)      # far, large in the image? no:
    add_box(depth, (3.0, 0.05, 0.0), 0.2)     # near box in front of it
    result = detect_frame(depth, K)
    assert len(result.detections) == 2
    assert sorted(round(d.position_sensor[0], 1)
                  for d in result.detections) == [2.9, 7.7]


def test_body_or_rotor_returns_are_rejected():
    depth = sky()
    depth[200:260, 300:360] = 0.3            # closer than min_range
    depth[0:40, 0:40] = -np.inf              # closer than the near clip
    result = detect_frame(depth, K)
    assert result.detections == []
    assert result.near_pixels == 60 * 60 + 40 * 40


def test_large_surface_is_rejected():
    depth = sky()
    depth[:, :300] = 2.4                     # roof slab filling half the view
    result = detect_frame(depth, K)
    assert result.detections == []
    assert result.rejected == {'too_large': 1}


def test_speckle_noise_is_rejected():
    depth = sky()
    depth[100, 100] = 6.0
    depth[300:302, 400:402] = 9.0
    result = detect_frame(depth, K)
    assert result.detections == []
    assert result.rejected.get('too_few_pixels') == 2


def test_beyond_max_range_is_ignored():
    depth = sky()
    add_box(depth, (40.0, 0.0, 0.0), 2.0)
    assert detect_frame(depth, K).detections == []


def test_nan_pixels_do_not_break_detection():
    depth = sky()
    add_box(depth, (5.0, 0.0, 0.0), 0.4)
    depth[10:20, 10:20] = np.nan
    assert len(detect_frame(depth, K).detections) == 1


def test_border_touching_region_is_flagged_and_less_confident():
    depth = sky()
    add_box(depth, (4.0, 0.0, 0.0), 0.4)
    full = detect_frame(depth, K).detections[0]
    depth2 = sky()
    add_box(depth2, (4.0, 4.5, 0.0), 0.4)    # at the left image edge
    cut = detect_frame(depth2, K).detections[0]
    assert cut.touches_border and not full.touches_border
    assert cut.confidence < full.confidence


def test_backprojection_round_trip():
    p = backproject([100.0, 320.0], [50.0, 240.0], [4.0, 7.0], K)
    assert p[1] == pytest.approx([7.0, 0.0, 0.0])
    u = K.cx - K.fx * p[0, 1] / p[0, 0]
    v = K.cy - K.fy * p[0, 2] / p[0, 0]
    assert (u, v) == pytest.approx((100.0, 50.0))


def test_sensor_axes_in_body_frame():
    # optical axis = up; in FRD up is -z, plus the 0.15 m mount offset
    assert sensor_to_body_frd(np.array([5.0, 0, 0])) == pytest.approx(
        [0, 0, -5.15])
    # image left (+Y_s) = body left = -y in FRD
    assert sensor_to_body_frd(np.array([5.0, 1.0, 0])) == pytest.approx(
        [0, -1.0, -5.15])
    # image up (+Z_s) = body backward = -x
    assert sensor_to_body_frd(np.array([5.0, 0, 1.0])) == pytest.approx(
        [-1.0, 0, -5.15])


def test_sensor_to_local_level_and_yawed():
    uav = np.array([10.0, 20.0, -3.0])       # NED, 3 m above the origin
    above = sensor_to_local(np.array([5.0, 0, 0]), uav, IDENTITY_Q)
    assert above == pytest.approx([10.0, 20.0, -8.15])
    # yaw +90 deg (nose east): body forward maps to +east
    half = math.radians(90) / 2
    q = np.array([math.cos(half), 0, 0, math.sin(half)])
    ahead = sensor_to_local(np.array([5.0, 0, -1.0]), uav, q)
    assert ahead == pytest.approx([10.0, 21.0, -8.15], abs=1e-9)


def test_pose_buffer_interpolates_and_refuses_gaps():
    buf = PoseBuffer()
    buf.add(1.00, [0, 0, 0], IDENTITY_Q)
    buf.add(1.01, [1, 0, 0], IDENTITY_Q)
    p, _ = buf.at(1.005)
    assert p == pytest.approx([0.5, 0, 0])
    assert buf.at(1.02)[0] == pytest.approx([1, 0, 0])   # 10 ms ahead: held
    assert buf.at(1.20) is None                          # too far ahead
    assert buf.at(0.5) is None
    buf.add(1.50, [2, 0, 0], IDENTITY_Q)
    assert buf.at(1.25) is None                          # 490 ms gap
    buf.add(1.49, [9, 9, 9], IDENTITY_Q)                 # out of order
    assert buf.latest_time() == 1.50
    buf.add(1.6, [math.nan, 0, 0], IDENTITY_Q)           # invalid: ignored
    assert buf.latest_time() == 1.50


def test_static_filter_flags_stationary_but_not_falling():
    filt = StaticFilter()
    static_flags, falling_flags = [], []
    for i in range(60):                       # 2 s at 30 Hz
        t = i / 30.0
        fall_z = -10.0 + 0.5 * 9.8 * t * t    # NED down positive
        flags = filt.update(t, np.array([[2.0, 1.0, -6.0],
                                         [0.5, 0.5, fall_z]]))
        static_flags.append(flags[0])
        falling_flags.append(flags[1])
    assert not any(falling_flags)
    assert not any(static_flags[:22])         # younger than static_time
    assert all(static_flags[24:])


def test_static_filter_with_dropped_frames():
    filt = StaticFilter()
    flags = []
    for i in range(60):
        if i % 4 == 0:                        # a quarter of frames missing
            continue
        flags.append(filt.update(i / 30.0, np.array([[1.0, 1.0, -5.0]]))[0])
    assert flags[-1]


def test_detector_rejects_static_structure_despite_ego_motion():
    """A fixed object stays static in the local frame while the UAV moves."""
    detector = DebrisDetector(K)
    kept = []
    fixed_local = np.array([1.0, 0.5, -9.0])
    for i in range(60):
        t = i / 30.0
        uav = np.array([0.02 * i, 0.0, -3.0])            # 0.6 m/s north
        # local -> body FRD (level, zero yaw) -> sensor
        frd = fixed_local - uav
        flu = np.array([frd[0], -frd[1], -frd[2]]) - [0, 0, 0.15]
        p_s = (flu[2], flu[1], -flu[0])
        depth = sky()
        add_box(depth, p_s, 0.4, d_face=p_s[0])
        result = detector.process(depth, t, (uav, IDENTITY_Q))
        kept.append(len(result.detections))
        if i == 59:
            assert result.rejected.get('static') == 1
    assert kept[0] == 1 and kept[-1] == 0


def test_detector_keeps_falling_object_and_reports_local_position():
    detector = DebrisDetector(K)
    uav = np.array([0.0, 0.0, -3.0])
    for i in range(20):
        t = i / 30.0
        height = 6.0 - 0.5 * 9.8 * t * t      # sensor-frame range
        if height < 1.0:
            break
        depth = sky()
        add_box(depth, (height, 0.8, 0.0), 0.4, d_face=height)
        result = detector.process(depth, t, (uav, IDENTITY_Q))
        assert len(result.detections) == 1
        local = result.detections[0].position_local
        assert local[2] == pytest.approx(-3.0 - 0.15 - height, abs=1e-3)
        assert local[1] == pytest.approx(-0.8, abs=0.05)


def test_detector_without_pose_still_detects_in_sensor_frame():
    detector = DebrisDetector(K)
    depth = sky()
    add_box(depth, (5.0, 0.0, 0.0), 0.4)
    result = detector.process(depth, 0.0, None)
    assert len(result.detections) == 1
    assert result.detections[0].position_local is None


def test_custom_parameters():
    depth = sky()
    add_box(depth, (5.0, 0.0, 0.0), 0.4)
    strict = DetectorParams(min_size_m=1.0)
    assert detect_frame(depth, K, strict).rejected == {'too_small': 1}


def test_debug_image_shape():
    depth = sky()
    add_box(depth, (5.0, 0.0, 0.0), 0.4)
    image = depth_to_debug_image(depth, detect_frame(depth, K))
    assert image.shape == (H, W, 3) and image.dtype == np.uint8
