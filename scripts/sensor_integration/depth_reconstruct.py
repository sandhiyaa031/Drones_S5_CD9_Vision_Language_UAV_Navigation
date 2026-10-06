#!/usr/bin/env python3
"""Offline check: 3D points from the upward depth camera vs Gazebo truth.

For every recorded depth frame with finite pixels, each finite pixel is
back-projected to a 3D point and transformed to the world frame:

    sensor frame (+X optical axis, +Y image left, +Z image up), depth d axial
        p_s = ( d, -(u - cx) d / fx, -(v - cy) d / fy )
    p_w = R_wm (R_ms p_s + t_ms) + t_wm      (m = model, s = sensor link)

Here the sensor pose comes from Gazebo ground truth so that the result
isolates the depth sensor itself. Each point is compared with the true surface
of the falling box (pose interpolated to the image timestamp). Evaluation only:
nothing in this file is used by perception or control.
"""

import argparse
import csv
import json
import os
from collections import defaultdict

import cv2
import numpy as np

FX = FY = 268.5118688877865
CX, CY = 320.0, 240.0


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp_free_interp(track, t):
    """Position linear, orientation nearest sample."""
    ts = track[:, 0]
    if t < ts[0] or t > ts[-1]:
        return None
    pos = np.array([np.interp(t, ts, track[:, i]) for i in (1, 2, 3)])
    return pos, track[int(np.argmin(np.abs(ts - t))), 4:8]


def point_to_box_distance(points_w, center, quat, size):
    """Unsigned distance from points to the surface of an oriented box."""
    local = (points_w - center) @ quat_to_matrix(*quat)
    half = np.asarray(size) / 2.0
    outside = np.maximum(np.abs(local) - half, 0.0)
    d_out = np.linalg.norm(outside, axis=1)
    d_in = np.min(half - np.abs(local), axis=1)
    return np.where(d_out > 0, d_out, np.maximum(d_in, 0.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dir')
    parser.add_argument('--uav', default='x500_rescue_0')
    args = parser.parse_args()
    series = defaultdict(list)
    with open(os.path.join(args.run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            series[row['name']].append([float(row[k]) for k in (
                't_sim', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')])
    series = {k: np.array(v) for k, v in series.items()}
    uav = series[args.uav]
    link = series['depth_up_link'][0]
    r_ms, t_ms = quat_to_matrix(*link[4:8]), link[1:4]
    releases = {r['name']: r for r in json.load(
        open(os.path.join(args.run_dir, 'releases.json')))}
    frames = []
    with open(os.path.join(args.run_dir, 'depth_frames.csv')) as handle:
        for row in csv.DictReader(handle):
            frames.append((float(row['t_sim']), row['file'],
                           int(row['finite_pixels'])))

    per_frame, no_object_frames = [], {'total': 0, 'with_finite': 0,
                                       'closer_than_1m': 0}
    hover = (uav[:, 3] > 2.8)
    t_hover = (uav[hover, 0].min(), uav[hover, 0].max()) if hover.any() \
        else (0, 0)
    for t, fname, n_finite in frames:
        if not t_hover[0] <= t <= t_hover[1]:
            continue
        active = [n for n, tr in series.items()
                  if n.startswith('p1_debris') and tr[0, 0] <= t <= tr[-1, 0]]
        if n_finite == 0:
            if not active:
                no_object_frames['total'] += 1
            continue
        mm = cv2.imread(os.path.join(args.run_dir, 'frames_raw', 'depth',
                                     fname), cv2.IMREAD_UNCHANGED)
        vs, us = np.nonzero((mm > 0) & (mm < 65535))
        d = mm[vs, us].astype(np.float64) / 1000.0
        if not active:
            no_object_frames['total'] += 1
            no_object_frames['with_finite'] += 1
            no_object_frames['closer_than_1m'] += int((d < 1.0).any())
            continue
        u_pose = slerp_free_interp(uav, t)
        r_wm = quat_to_matrix(*u_pose[1])
        p_s = np.column_stack([d, -(us - CX) * d / FX, -(vs - CY) * d / FY])
        p_w = (p_s @ r_ms.T + t_ms) @ r_wm.T + u_pose[0]
        sensor_w = r_wm @ t_ms + u_pose[0]
        # attribute the pixels to the nearest active object
        best = None
        for name in active:
            pose = slerp_free_interp(series[name], t)
            dist = point_to_box_distance(p_w, pose[0], pose[1],
                                         releases[name]['size'])
            if best is None or np.median(dist) < np.median(best[2]):
                best = (name, pose, dist)
        name, pose, dist = best
        track = series[name]
        i = int(np.argmin(np.abs(track[:, 0] - t)))
        j0, j1 = max(i - 2, 0), min(i + 2, len(track) - 1)
        speed = float(np.linalg.norm(track[j1, 1:4] - track[j0, 1:4]) /
                      (track[j1, 0] - track[j0, 0]))
        # centre estimate from the lowest (nearest) face, box half-height
        # taken from the release record (evaluation only)
        near = d <= d.min() + 0.05
        centre_est = p_w[near].mean(axis=0) + np.array(
            [0, 0, releases[name]['size'][2] / 2.0])
        per_frame.append({
            'object': name, 't_sim': t, 'pixels': int(len(d)),
            'range_m': float(np.linalg.norm(pose[0] - sensor_w)),
            'height_above_sensor_m': float(pose[0][2] - sensor_w[2]),
            'speed_m_s': speed,
            'surface_error_median_m': float(np.median(dist)),
            'surface_error_p95_m': float(np.percentile(dist, 95)),
            'surface_error_max_m': float(dist.max()),
            'centre_error_m': float(np.linalg.norm(centre_est - pose[0])),
            'centre_error_xy_m': float(np.linalg.norm(
                (centre_est - pose[0])[:2])),
            'centre_error_z_m': float((centre_est - pose[0])[2]),
            'touches_image_border': bool(
                us.min() == 0 or vs.min() == 0 or us.max() == 639
                or vs.max() == 479)})

    out_dir = os.path.join(args.run_dir, 'analysis')
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'depth_reconstruction_frames.csv'), 'w',
              newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(per_frame[0].keys()))
        writer.writeheader()
        writer.writerows(per_frame)

    objects = []
    for name in sorted({f['object'] for f in per_frame}):
        rows = [f for f in per_frame if f['object'] == name]
        full = [f for f in rows if not f['touches_image_border']]
        objects.append({
            'object': name, 'label': releases[name].get('label', ''),
            'depth_frames': len(rows),
            'first_seen_height_above_sensor_m':
                round(rows[0]['height_above_sensor_m'], 2),
            'range_span_m': [round(min(f['range_m'] for f in rows), 2),
                             round(max(f['range_m'] for f in rows), 2)],
            'max_speed_m_s': round(max(f['speed_m_s'] for f in rows), 2),
            'min_pixels': min(f['pixels'] for f in rows),
            'surface_error_median_m': round(float(np.median(
                [f['surface_error_median_m'] for f in rows])), 4),
            'surface_error_worst_frame_max_m': round(max(
                f['surface_error_max_m'] for f in rows), 4),
            'centre_error_median_m': round(float(np.median(
                [f['centre_error_m'] for f in full])), 4) if full else None,
            'centre_error_max_m': round(max(
                f['centre_error_m'] for f in full), 4) if full else None})
    all_surface = [f['surface_error_median_m'] for f in per_frame]
    full = [f for f in per_frame if not f['touches_image_border']]
    summary = {
        'objects_with_depth_frames': len(objects),
        'depth_frames_with_object': len(per_frame),
        'surface_error_m': {
            'median_of_frame_medians': float(np.median(all_surface)),
            'p95_of_frame_medians': float(np.percentile(all_surface, 95)),
            'max_of_frame_medians': float(np.max(all_surface)),
            'worst_single_point': float(max(
                f['surface_error_max_m'] for f in per_frame))},
        'centre_error_m_fully_visible_frames': {
            'frames': len(full),
            'median': float(np.median([f['centre_error_m'] for f in full])),
            'p95': float(np.percentile(
                [f['centre_error_m'] for f in full], 95)),
            'max': float(max(f['centre_error_m'] for f in full))},
        'hover_frames_without_any_object': no_object_frames,
        'objects': objects}
    with open(os.path.join(out_dir, 'depth_reconstruction.json'), 'w') as h:
        json.dump(summary, h, indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k != 'objects'},
                     indent=1))
    for o in objects:
        print(o)


if __name__ == '__main__':
    main()
