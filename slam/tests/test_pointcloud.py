"""
Unit tests for Point Cloud Processing and Ground Plane Segmentation.
"""

import numpy as np
import pytest
from gps_denied_nav.mapping.pointcloud_processor import PointCloudProcessor


def test_voxel_downsampling():
    processor = PointCloudProcessor(voxel_size=0.1)
    # Generate 50 points very close together (within 1 voxel)
    pts = np.random.uniform(0.01, 0.05, (50, 3)).astype(np.float32)
    downsampled = processor.voxel_downsample(pts)

    # Should be condensed to a single representative voxel centroid
    assert len(downsampled) == 1
    assert np.allclose(downsampled[0], np.mean(pts, axis=0), atol=1e-3)


def test_ground_plane_segmentation():
    processor = PointCloudProcessor(ground_distance_threshold=0.1)
    
    # 200 ground points on Z=0
    x_g = np.random.uniform(-5, 5, 200)
    y_g = np.random.uniform(-5, 5, 200)
    z_g = np.random.normal(0.0, 0.02, 200)
    ground_pts = np.column_stack([x_g, y_g, z_g])

    # 50 obstacle points at Z=1.5
    x_o = np.random.uniform(-1, 1, 50)
    y_o = np.random.uniform(-1, 1, 50)
    z_o = np.random.uniform(1.0, 2.0, 50)
    obs_pts = np.column_stack([x_o, y_o, z_o])

    all_pts = np.vstack([ground_pts, obs_pts])
    obstacles, ground, plane = processor.segment_ground_plane(all_pts)

    assert len(ground) >= 150
    assert len(obstacles) <= 100
