"""
Camera Calibration Utility using OpenCV Checkerboard Patterns.
Computes Intrinsics K, Distortion Coefficients D, and exports ROS/ORB-SLAM3 compatible YAML configs.
"""

import cv2
import numpy as np
import glob
import argparse
import yaml
import os


def calibrate_camera(
    image_dir: str,
    pattern_rows: int = 6,
    pattern_cols: int = 9,
    square_size_m: float = 0.025,
    output_yaml: str = "camera_calib.yaml"
):
    print(f"Starting camera calibration using checkerboard ({pattern_cols}x{pattern_rows}, {square_size_m*1000:.1f}mm)...")

    # Termination criteria for sub-pixel corner refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 3D points in real world space (Z = 0)
    objp = np.zeros((pattern_rows * pattern_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_cols, 0:pattern_rows].T.reshape(-1, 2) * square_size_m

    objpoints = []  # 3D point in real world space
    imgpoints = []  # 2D points in image plane

    images = glob.glob(os.path.join(image_dir, "*.jpg")) + glob.glob(os.path.join(image_dir, "*.png"))
    if not images:
        print(f"No calibration images found in directory: {image_dir}")
        return False

    img_shape = None
    successful_frames = 0

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (pattern_cols, pattern_rows), None)

        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            successful_frames += 1

    print(f"Detected valid checkerboard corners in {successful_frames}/{len(images)} images.")
    if successful_frames < 5:
        print("Error: Need at least 5 good frames for accurate calibration.")
        return False

    # Perform camera calibration
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)

    # Compute Reprojection Error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    mean_error = total_error / len(objpoints)

    print("\nCalibration Results:")
    print(f"  Reprojection Error: {mean_error:.4f} pixels (Target: < 0.5 px)")
    print(f"  fx: {K[0,0]:.2f}, fy: {K[1,1]:.2f}")
    print(f"  cx: {K[0,2]:.2f}, cy: {K[1,2]:.2f}")
    print(f"  Distortion (k1, k2, p1, p2, k3): {dist.ravel()[:5]}")

    # Export to ROS YAML format
    calib_data = {
        "image_width": int(img_shape[0]),
        "image_height": int(img_shape[1]),
        "camera_name": "calibrated_uav_camera",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": K.flatten().tolist()
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": dist.flatten()[:5].tolist()
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                K[0,0], 0.0, K[0,2], 0.0,
                0.0, K[1,1], K[1,2], 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
        }
    }

    with open(output_yaml, 'w') as f:
        yaml.dump(calib_data, f, default_flow_style=False)

    print(f"Configuration written to: {output_yaml}")
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Camera Calibration Script")
    parser.add_argument("--dir", type=str, default="./calib_images", help="Directory containing checkerboard images")
    parser.add_argument("--rows", type=int, default=6, help="Checkerboard internal row corners")
    parser.add_argument("--cols", type=int, default=9, help="Checkerboard internal col corners")
    parser.add_argument("--size", type=float, default=0.025, help="Checkerboard square size in meters")
    parser.add_argument("--output", type=str, default="config/camera_calib.yaml", help="Output YAML path")
    args = parser.parse_args()

    calibrate_camera(args.dir, args.rows, args.cols, args.size, args.output)
