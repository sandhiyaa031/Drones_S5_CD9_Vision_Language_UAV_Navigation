"""
Point Cloud Processor for 3D Sparse/Dense Visual Depth Data.
Includes Voxel Grid Downsampling, Statistical Outlier Removal, and Ground Plane Segmentation.
"""

import numpy as np
from typing import Tuple, Optional


class PointCloudProcessor:
    def __init__(
        self,
        voxel_size: float = 0.1,         # 10 cm voxel grid size
        min_points_per_voxel: int = 1,
        sor_k_neighbors: int = 10,       # Statistical Outlier Removal neighbors
        sor_std_ratio: float = 2.0,
        ground_distance_threshold: float = 0.15
    ):
        self.voxel_size = voxel_size
        self.min_points_per_voxel = min_points_per_voxel
        self.sor_k_neighbors = sor_k_neighbors
        self.sor_std_ratio = sor_std_ratio
        self.ground_distance_threshold = ground_distance_threshold

    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        Voxel grid spatial downsampling of 3D point cloud.
        Args:
            points: (N, 3) numpy array
        Returns:
            downsampled: (M, 3) numpy array where M <= N
        """
        if len(points) == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Quantize points into discrete voxel coordinates
        voxel_coords = np.floor(points / self.voxel_size).astype(np.int32)

        # Group points by unique voxel coordinate
        unique_voxels, indices, counts = np.unique(voxel_coords, axis=0, return_inverse=True, return_counts=True)

        downsampled = np.zeros((len(unique_voxels), 3), dtype=np.float32)
        for i in range(len(unique_voxels)):
            if counts[i] >= self.min_points_per_voxel:
                mask = (indices == i)
                downsampled[i] = np.mean(points[mask], axis=0)

        return downsampled

    def filter_statistical_outliers(self, points: np.ndarray) -> np.ndarray:
        """
        Filter out isolated noise points using distance to k-nearest neighbors.
        """
        if len(points) < self.sor_k_neighbors + 1:
            return points

        # Subsample for fast distance estimation if point count is very large
        n_pts = len(points)
        if n_pts > 2000:
            idx = np.random.choice(n_pts, 2000, replace=False)
            sample_pts = points[idx]
        else:
            sample_pts = points

        # Compute pairwise distance to k-nearest neighbors
        from scipy.spatial import cKDTree
        tree = cKDTree(sample_pts)
        distances, _ = tree.query(points, k=min(self.sor_k_neighbors + 1, len(sample_pts)))
        mean_dists = np.mean(distances[:, 1:], axis=1)

        mu = np.mean(mean_dists)
        sigma = np.std(mean_dists)
        valid_mask = mean_dists < (mu + self.sor_std_ratio * sigma)

        return points[valid_mask]

    def segment_ground_plane(self, points: np.ndarray, max_iterations: int = 50) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
        """
        Segment ground plane vs obstacles using RANSAC plane fitting.

        Returns:
            obstacles: (K, 3) Non-ground obstacle points
            ground: (G, 3) Ground plane points
            plane_model: [a, b, c, d] where ax + by + cz + d = 0 (or None)
        """
        if len(points) < 10:
            return points, np.empty((0, 3), dtype=np.float32), None

        best_inliers = []
        best_model = None

        n_pts = len(points)
        for _ in range(max_iterations):
            sample_idx = np.random.choice(n_pts, 3, replace=False)
            p1, p2, p3 = points[sample_idx]

            # Normal vector
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm

            # Check if normal is roughly vertical (|nz| > 0.7 for horizontal floor)
            if abs(normal[2]) < 0.7:
                continue

            d = -np.dot(normal, p1)

            # Distances of all points to plane
            distances = np.abs(np.dot(points, normal) + d)
            inliers = np.where(distances < self.ground_distance_threshold)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_model = np.array([normal[0], normal[1], normal[2], d])

        if len(best_inliers) == 0:
            # Fallback: simple height threshold (lowest 15% as ground)
            z_min = np.min(points[:, 2])
            is_ground = points[:, 2] < (z_min + 0.2)
            return points[~is_ground], points[is_ground], None

        is_inlier = np.zeros(n_pts, dtype=bool)
        is_inlier[best_inliers] = True

        obstacles = points[~is_inlier]
        ground = points[is_inlier]
        return obstacles, ground, best_model
