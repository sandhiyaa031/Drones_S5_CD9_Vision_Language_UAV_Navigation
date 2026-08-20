"""
Trajectory Benchmark and Evaluation Suite (ATE & RPE Metrics).
Computes Absolute Trajectory Error (ATE) and Relative Pose Error (RPE)
according to standard robotics benchmarks (Sturm et al. TUM/EuRoC benchmark standards).
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Tuple, Optional
from scipy.spatial.transform import Rotation as R_scipy


class TrajectoryEvaluator:
    def __init__(self):
        self.gt_poses = []       # List of (3,) [x, y, z]
        self.est_poses = []      # List of (3,) [x, y, z]
        self.timestamps = []

    def add_sample(self, timestamp: float, gt_pos: np.ndarray, est_pos: np.ndarray):
        """
        Record timestamped Ground Truth and Estimated positions.
        """
        self.timestamps.append(timestamp)
        self.gt_poses.append(np.array(gt_pos).flatten())
        self.est_poses.append(np.array(est_pos).flatten())

    def compute_ate(self, align_se3: bool = False) -> Dict[str, float]:
        """
        Compute Absolute Trajectory Error (ATE) statistics.

        Returns:
            metrics: Dictionary with RMSE, Mean, Median, Max, Std, and Drift Rate %
        """
        if len(self.gt_poses) < 2:
            return {"rmse": 0.0, "mean": 0.0, "median": 0.0, "max": 0.0, "std": 0.0, "drift_percent": 0.0}

        gt = np.array(self.gt_poses)
        est = np.array(self.est_poses)

        if align_se3:
            est_aligned = self._align_trajectories_se3(est, gt)
        else:
            est_aligned = est

        # Translation errors
        errors = np.linalg.norm(gt - est_aligned, axis=1)

        rmse = float(np.sqrt(np.mean(errors ** 2)))
        mean_err = float(np.mean(errors))
        median_err = float(np.median(errors))
        max_err = float(np.max(errors))
        std_err = float(np.std(errors))

        # Total distance traveled by ground truth
        diffs = np.diff(gt, axis=0)
        total_distance = float(np.sum(np.linalg.norm(diffs, axis=1)))
        drift_percent = (rmse / (total_distance + 1e-6)) * 100.0

        return {
            "rmse": rmse,
            "mean": mean_err,
            "median": median_err,
            "max": max_err,
            "std": std_err,
            "total_distance_m": total_distance,
            "drift_percent": drift_percent
        }

    def compute_rpe(self, delta_steps: int = 30) -> Dict[str, float]:
        """
        Compute Relative Pose Error (RPE) over fixed step intervals.
        """
        if len(self.gt_poses) <= delta_steps:
            return {"rpe_trans_rmse": 0.0, "rpe_trans_mean": 0.0}

        gt = np.array(self.gt_poses)
        est = np.array(self.est_poses)

        rpe_errors = []
        for i in range(len(gt) - delta_steps):
            d_gt = gt[i + delta_steps] - gt[i]
            d_est = est[i + delta_steps] - est[i]
            rpe_errors.append(np.linalg.norm(d_gt - d_est))

        rpe_arr = np.array(rpe_errors)
        return {
            "rpe_trans_rmse": float(np.sqrt(np.mean(rpe_arr ** 2))),
            "rpe_trans_mean": float(np.mean(rpe_arr))
        }

    def _align_trajectories_se3(self, est: np.ndarray, gt: np.ndarray) -> np.ndarray:
        """
        Umeyama SE(3) Rigid Body Alignment (estimates R and t).
        """
        mu_est = np.mean(est, axis=0)
        mu_gt = np.mean(gt, axis=0)

        est_centered = est - mu_est
        gt_centered = gt - mu_gt

        H = est_centered.T @ gt_centered
        U, S, Vt = np.linalg.svd(H)
        R_align = Vt.T @ U.T

        if np.linalg.det(R_align) < 0:
            Vt[-1, :] *= -1
            R_align = Vt.T @ U.T

        t_align = mu_gt - R_align @ mu_est
        return (R_align @ est.T).T + t_align

    def plot_evaluation(self, save_path: Optional[str] = None, show: bool = False):
        """
        Generate a multi-panel evaluation plot comparing Estimated vs Ground Truth trajectories.
        """
        if len(self.gt_poses) < 2:
            return

        gt = np.array(self.gt_poses)
        est = np.array(self.est_poses)
        t = np.array(self.timestamps)
        ate = self.compute_ate()

        fig = plt.figure(figsize=(14, 10))

        # 1. 3D Trajectory Plot
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        ax1.plot(gt[:, 0], gt[:, 1], gt[:, 2], 'g-', label='Ground Truth', linewidth=2)
        ax1.plot(est[:, 0], est[:, 1], est[:, 2], 'b--', label='Estimated (VO+EKF)', linewidth=2)
        ax1.scatter([gt[0, 0]], [gt[0, 1]], [gt[0, 2]], color='green', s=60, marker='o', label='Start')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        ax1.set_title('3D Trajectory in GPS-Denied Environment')
        ax1.legend(loc='upper right')

        # 2. 2D Top-Down Path (X-Y Plane)
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(gt[:, 0], gt[:, 1], 'g-', label='Ground Truth', linewidth=2)
        ax2.plot(est[:, 0], est[:, 1], 'b--', label='Estimated', linewidth=2)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Y [m]')
        ax2.set_title('Top-Down Flight Path (X-Y Plane)')
        ax2.grid(True, linestyle=':')
        ax2.legend()

        # 3. Position Error over Time
        errors = np.linalg.norm(gt - est, axis=1)
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(t, errors, 'r-', linewidth=1.5, label='Position Error')
        ax3.axhline(ate['rmse'], color='k', linestyle='--', label=f"ATE RMSE: {ate['rmse']:.3f} m")
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Error [m]')
        ax3.set_title('Absolute Position Error Over Time')
        ax3.grid(True, linestyle=':')
        ax3.legend()

        # 4. Altitude Tracking (Z vs Time)
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(t, gt[:, 2], 'g-', label='Ground Truth Z', linewidth=2)
        ax4.plot(t, est[:, 2], 'b--', label='Estimated Z', linewidth=2)
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Altitude Z [m]')
        ax4.set_title(f"Altitude Tracking | Total Drift: {ate['drift_percent']:.2f}%")
        ax4.grid(True, linestyle=':')
        ax4.legend()

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300)
            print(f"Evaluation plot saved to {save_path}")

        if show:
            plt.show()

        plt.close(fig)
