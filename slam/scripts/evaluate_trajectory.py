"""
Command-line Trajectory Evaluation Tool (TUM / EuRoC / CSV formats).
Computes ATE, RPE, and generates publication-quality comparison figures.
"""

import numpy as np
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from gps_denied_nav.eval.trajectory_evaluator import TrajectoryEvaluator


def load_tum_trajectory(file_path: str):
    """
    Parse TUM format trajectory file: timestamp tx ty tz qx qy qz qw
    """
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#') or len(line.strip()) == 0:
                continue
            parts = line.strip().split()
            if len(parts) >= 4:
                t = float(parts[0])
                p = [float(parts[1]), float(parts[2]), float(parts[3])]
                data.append((t, p))
    return data


def main():
    parser = argparse.ArgumentParser(description="Evaluate SLAM / VO Trajectory against Ground Truth")
    parser.add_argument("--gt", type=str, required=True, help="Ground truth trajectory file (TUM format)")
    parser.add_argument("--est", type=str, required=True, help="Estimated trajectory file (TUM format)")
    parser.add_argument("--output_plot", type=str, default="trajectory_benchmark.png", help="Path to save evaluation plot")
    parser.add_argument("--align", action="store_true", help="Perform SE(3) Umeyama alignment")
    args = parser.parse_args()

    print(f"Loading Ground Truth: {args.gt}")
    gt_data = load_tum_trajectory(args.gt)
    print(f"Loading Estimated:    {args.est}")
    est_data = load_tum_trajectory(args.est)

    evaluator = TrajectoryEvaluator()

    # Match timestamps or nearest neighbors
    gt_times = np.array([x[0] for x in gt_data])
    gt_poses = np.array([x[1] for x in gt_data])

    for t_est, p_est in est_data:
        # Find closest GT sample
        idx = np.argmin(np.abs(gt_times - t_est))
        if np.abs(gt_times[idx] - t_est) < 0.1:  # Within 100ms
            evaluator.add_sample(t_est, gt_poses[idx], p_est)

    ate = evaluator.compute_ate(align_se3=args.align)
    rpe = evaluator.compute_rpe()

    print("\n" + "=" * 50)
    print("        TRAJECTORY BENCHMARK REPORT")
    print("=" * 50)
    print(f"  Samples Analyzed:         {len(evaluator.gt_poses)}")
    print(f"  Total Distance Traveled:  {ate['total_distance_m']:.2f} m")
    print(f"  ATE RMSE:                 {ate['rmse']:.4f} m")
    print(f"  ATE Mean Error:           {ate['mean']:.4f} m")
    print(f"  ATE Max Error:            {ate['max']:.4f} m")
    print(f"  ATE Std Dev:              {ate['std']:.4f} m")
    print(f"  RPE Translation RMSE:     {rpe['rpe_trans_rmse']:.4f} m")
    print(f"  Drift Percentage:         {ate['drift_percent']:.2f} %")
    print("=" * 50)

    evaluator.plot_evaluation(save_path=args.output_plot, show=False)
    print(f"Benchmark plot saved to: {args.output_plot}")


if __name__ == '__main__':
    main()
