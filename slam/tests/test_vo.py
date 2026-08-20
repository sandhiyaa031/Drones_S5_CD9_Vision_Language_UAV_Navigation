"""
Unit tests for Visual Odometry and Feature Tracker.
"""

import numpy as np
import pytest
from gps_denied_nav.vo.feature_tracker import FeatureTracker
from gps_denied_nav.vo.visual_odometer import VisualOdometer


def test_feature_tracker_initialization():
    tracker = FeatureTracker(max_features=200, detector_type="FAST")
    assert tracker.max_features == 200
    assert tracker.detector_type == "FAST"
    assert len(tracker.tracked_features) == 0


def test_feature_tracking_synthetic_motion():
    import cv2
    tracker = FeatureTracker(max_features=150, detector_type="SHI_TOMASI")
    # Create test image with distinct corner markers
    img1 = np.zeros((480, 640), dtype=np.uint8)
    for x in range(100, 550, 60):
        for y in range(100, 380, 60):
            cv2.drawMarker(img1, (x, y), 255, cv2.MARKER_CROSS, 10, 2)

    # First frame initializes keypoints
    curr_pts, prev_pts, ids = tracker.track(img1)
    assert len(tracker.tracked_features) > 0

    # Shift image by 2 pixels (simulate motion)
    img2 = np.roll(img1, shift=2, axis=1)
    curr_pts2, prev_pts2, ids2 = tracker.track(img2)
    assert len(curr_pts2) > 0


def test_visual_odometer_processing():
    K = np.array([[525.0, 0, 320.0], [0, 525.0, 240.0], [0, 0, 1.0]], dtype=np.float64)
    vo = VisualOdometer(camera_matrix=K)

    # Blank image handling
    blank_img = np.zeros((480, 640, 3), dtype=np.uint8)
    success, pos, quat, cov = vo.process_frame(blank_img, timestamp=1.0)

    assert pos.shape == (3, 1)
    assert quat.shape == (4,)
    assert cov.shape == (6, 6)
