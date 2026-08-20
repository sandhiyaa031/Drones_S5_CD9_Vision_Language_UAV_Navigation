"""
Robust KLT + FAST/ORB Feature Tracker with Spatial Grid Bucketing.
Provides uniform feature distribution and forward-backward consistency checking.
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class TrackedFeature:
    id: int
    pt: np.ndarray          # Current 2D point (x, y)
    initial_pt: np.ndarray  # Point where it was first observed
    age: int = 1            # Number of frames this feature has been tracked
    landmark_3d: Optional[np.ndarray] = None  # Triangulated 3D point in world/camera frame


class FeatureTracker:
    def __init__(
        self,
        max_features: int = 250,
        min_distance: float = 15.0,
        grid_rows: int = 4,
        grid_cols: int = 4,
        max_features_per_bucket: int = 20,
        detector_type: str = "FAST",  # "FAST", "SHI_TOMASI", "ORB"
        fb_threshold: float = 1.0     # Forward-backward error threshold in pixels
    ):
        self.max_features = max_features
        self.min_distance = min_distance
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.max_features_per_bucket = max_features_per_bucket
        self.detector_type = detector_type.upper()
        self.fb_threshold = fb_threshold

        self.next_feature_id = 0
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_pts: Optional[np.ndarray] = None
        self.tracked_features: Dict[int, TrackedFeature] = {}

        # LK Optical Flow Parameters
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        # Detectors
        self.fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
        self.orb = cv2.ORB_create(nfeatures=max_features * 2)

    def track(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, List[int]]:
        """
        Track features from previous frame to current frame using pyramidal KLT
        with bidirectional (forward-backward) error rejection.

        Args:
            image: Grayscale or BGR image (H, W)

        Returns:
            curr_pts: (N, 2) numpy array of valid tracked points in current frame
            prev_pts: (N, 2) numpy array of corresponding points in previous frame
            feature_ids: List of N tracking IDs
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        # First frame initialization
        if self.prev_gray is None or len(self.tracked_features) == 0:
            self._detect_new_features(gray)
            self.prev_gray = gray
            return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32), []

        # Get existing points and their IDs
        ids = list(self.tracked_features.keys())
        p0 = np.array([self.tracked_features[fid].pt for fid in ids], dtype=np.float32).reshape(-1, 1, 2)

        if len(p0) == 0:
            self._detect_new_features(gray)
            self.prev_gray = gray
            return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32), []

        # Forward Optical Flow (prev -> curr)
        p1, st_fwd, err_fwd = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None, **self.lk_params)

        # Backward Optical Flow (curr -> prev) for consistency check
        p0_back, st_bwd, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, p1, None, **self.lk_params)

        # Compute Forward-Backward geometric error
        fb_error = np.linalg.norm(p0.reshape(-1, 2) - p0_back.reshape(-1, 2), axis=1)

        # Boundary checks
        h, w = gray.shape
        in_bounds = (
            (p1[:, 0, 0] >= 5) & (p1[:, 0, 0] < w - 5) &
            (p1[:, 0, 1] >= 5) & (p1[:, 0, 1] < h - 5)
        )

        valid_mask = (st_fwd.reshape(-1) == 1) & (st_bwd.reshape(-1) == 1) & (fb_error < self.fb_threshold) & in_bounds

        # Update tracked feature dictionary
        new_tracked = {}
        matched_curr = []
        matched_prev = []
        matched_ids = []

        for i, is_valid in enumerate(valid_mask):
            fid = ids[i]
            if is_valid:
                pt_curr = p1[i, 0]
                pt_prev = p0[i, 0]
                tf = self.tracked_features[fid]
                tf.pt = pt_curr
                tf.age += 1
                new_tracked[fid] = tf

                matched_curr.append(pt_curr)
                matched_prev.append(pt_prev)
                matched_ids.append(fid)

        self.tracked_features = new_tracked

        # Replenish features if count dropped below target
        if len(self.tracked_features) < self.max_features:
            self._detect_new_features(gray)

        self.prev_gray = gray

        if len(matched_curr) > 0:
            return np.array(matched_curr, dtype=np.float32), np.array(matched_prev, dtype=np.float32), matched_ids
        else:
            return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32), []

    def _detect_new_features(self, gray: np.ndarray):
        """
        Detect new keypoints with spatial bucketing across a grid to prevent clustering.
        """
        h, w = gray.shape
        cell_h = h // self.grid_rows
        cell_w = w // self.grid_cols

        # Create an occupancy mask to avoid detecting close to existing points
        mask = np.ones((h, w), dtype=np.uint8) * 255
        for tf in self.tracked_features.values():
            cv2.circle(mask, (int(tf.pt[0]), int(tf.pt[1])), int(self.min_distance), 0, -1)

        # Count features per bucket
        bucket_counts = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        for tf in self.tracked_features.values():
            r = min(int(tf.pt[1] // cell_h), self.grid_rows - 1)
            c = min(int(tf.pt[0] // cell_w), self.grid_cols - 1)
            bucket_counts[r, c] += 1

        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                needed = self.max_features_per_bucket - bucket_counts[r, c]
                if needed <= 0:
                    continue

                y1, y2 = r * cell_h, min((r + 1) * cell_h, h)
                x1, x2 = c * cell_w, min((c + 1) * cell_w, w)

                cell_mask = mask[y1:y2, x1:x2]
                cell_gray = gray[y1:y2, x1:x2]

                new_pts = []
                if self.detector_type == "FAST":
                    kps = self.fast.detect(cell_gray, mask=cell_mask)
                    # Sort by response
                    kps = sorted(kps, key=lambda kp: kp.response, reverse=True)[:needed]
                    for kp in kps:
                        new_pts.append((kp.pt[0] + x1, kp.pt[1] + y1))
                elif self.detector_type == "ORB":
                    kps = self.orb.detect(cell_gray, mask=cell_mask)
                    kps = sorted(kps, key=lambda kp: kp.response, reverse=True)[:needed]
                    for kp in kps:
                        new_pts.append((kp.pt[0] + x1, kp.pt[1] + y1))
                else:  # Shi-Tomasi
                    corners = cv2.goodFeaturesToTrack(
                        cell_gray,
                        maxCorners=needed,
                        qualityLevel=0.01,
                        minDistance=self.min_distance,
                        mask=cell_mask
                    )
                    if corners is not None:
                        for pt in corners:
                            new_pts.append((pt[0, 0] + x1, pt[0, 1] + y1))

                for pt in new_pts:
                    pt_arr = np.array(pt, dtype=np.float32)
                    self.tracked_features[self.next_feature_id] = TrackedFeature(
                        id=self.next_feature_id,
                        pt=pt_arr,
                        initial_pt=pt_arr.copy(),
                        age=1
                    )
                    cv2.circle(mask, (int(pt[0]), int(pt[1])), int(self.min_distance), 0, -1)
                    self.next_feature_id += 1

    def draw_tracks(self, image: np.ndarray) -> np.ndarray:
        """
        Draw active feature tracks and status HUD onto an output frame.
        """
        out = image.copy()
        if len(out.shape) == 2:
            out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

        for tf in self.tracked_features.values():
            p_curr = (int(tf.pt[0]), int(tf.pt[1]))
            p_init = (int(tf.initial_pt[0]), int(tf.initial_pt[1]))

            # Color by track age (cyan for new -> green -> bright yellow for long-lived)
            color_val = min(255, tf.age * 8)
            color = (255 - color_val, 255, color_val)

            cv2.line(out, p_init, p_curr, (0, 165, 255), 1)
            cv2.circle(out, p_curr, 3, color, -1)

        # HUD Info
        num_feats = len(self.tracked_features)
        status_text = f"Tracked Features: {num_feats} | Method: KLT+{self.detector_type}"
        cv2.putText(out, status_text, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return out
