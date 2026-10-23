#!/usr/bin/env python3
"""OFFLINE scoring of SLAM against simulator ground truth (village).

Inputs from a run folder: slam_poses.jsonl and slam_map.npz (recorded from
slam_toolbox's output during the flight), poses.csv (the simulator's true
vehicle pose) and the world's truth file. Ground truth is used here only.

Reported
    trajectory   position error of the SLAM pose and of the odometry that
                 was handed to SLAM, against the true pose: RMSE, mean, max,
                 error at the end, and per 10 m of path ("drift"); heading
                 error. Both frames are aligned to the truth at the first
                 common sample only (no later correction). A second set
                 of numbers (rigid_fit_*) uses the best rigid fit over the
                 whole flight, which removes a constant frame offset.
    map          precision: share of occupied map cells within 0.5 m of a
                 true obstacle outline at the mapped height
                 recall: share of true outline cells within lidar range of
                 the flown path that the map contains (within 0.5 m)
                 coverage: mapped (known) area
    health       scans forwarded / received, map updates, the number of
                 times slam_toolbox moved its map->odom correction

Usage: evaluate_slam.py [--world NAME] [--altitude 6.0] [--out FILE] RUN...
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
LIDAR_RANGE_M = 29.0
MATCH_M = 0.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_truth_pose(run_dir):
    t, x, y, yaw, z = [], [], [], [], []
    with open(os.path.join(run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            if row['name'] != 'x500_rescue_0':
                continue
            qx, qy, qz, qw = (float(row[k]) for k in ('qx', 'qy', 'qz', 'qw'))
            t.append(float(row['t_sim']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            z.append(float(row['z']))
            yaw.append(math.atan2(2 * (qw * qz + qx * qy),
                                  1 - 2 * (qy * qy + qz * qz)))
    return (np.array(t), np.array(x), np.array(y), np.unwrap(np.array(yaw)),
            np.array(z))


def trajectory_metrics(est, truth_xy, truth_yaw, path):
    """est: N x 3 (x, y, yaw) aligned in time with the truth arrays."""
    # align at the first sample only (translation and heading)
    d_yaw = truth_yaw[0] - est[0, 2]
    c, s = math.cos(d_yaw), math.sin(d_yaw)
    rel = est[:, :2] - est[0, :2]
    aligned = np.column_stack([c * rel[:, 0] - s * rel[:, 1],
                               s * rel[:, 0] + c * rel[:, 1]]) + truth_xy[0]
    err = np.linalg.norm(aligned - truth_xy, axis=1)
    yaw_err = np.degrees(np.abs([wrap(v) for v in (
        est[:, 2] + d_yaw - truth_yaw)]))
    out = {'position_rmse_m': round(float(np.sqrt((err ** 2).mean())), 3),
           'position_mean_m': round(float(err.mean()), 3),
           'position_max_m': round(float(err.max()), 3),
           'position_end_m': round(float(err[-1]), 3),
           'yaw_mean_deg': round(float(yaw_err.mean()), 2),
           'yaw_max_deg': round(float(yaw_err.max()), 2)}
    if path[-1] > 10.0:
        out['end_error_percent_of_path'] = round(
            100.0 * float(err[-1]) / float(path[-1]), 2)
    # second view: best rigid fit over the whole flight (rotation and
    # translation, no scale). It removes a constant frame offset - such as
    # the heading error PX4 still has at the first sample - and leaves only
    # the error that changes during the flight.
    a = est[:, :2] - est[:, :2].mean(axis=0)
    b = truth_xy - truth_xy.mean(axis=0)
    u, _, vt = np.linalg.svd(a.T @ b)
    rot = u @ np.diag([1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
    fit_err = np.linalg.norm(a @ rot - b, axis=1)
    out['rigid_fit_rmse_m'] = round(float(np.sqrt((fit_err ** 2).mean())), 3)
    out['rigid_fit_max_m'] = round(float(fit_err.max()), 3)
    out['rigid_fit_rotation_deg'] = round(math.degrees(
        math.atan2(rot[0, 1], rot[0, 0])) - math.degrees(d_yaw), 2)
    return out, err


def outline_cells(structures, altitude, resolution=0.25):
    """Points on the outlines of structures that cross the mapped height."""
    points = []
    for s in structures:
        if not (s['z'][0] <= altitude <= s['z'][1]):
            continue
        cx, cy = s['centre']
        w, d = s['size']
        yaw = s.get('yaw', 0.0)
        c, sn = math.cos(yaw), math.sin(yaw)
        if s['kind'] in ('tree', 'pole'):
            r = w / 2.0
            if s['kind'] == 'tree':
                # crown: a sphere cut by the horizontal plane
                centre_z = (s['z'][0] + s['z'][1]) / 2.0
                r = math.sqrt(max(r * r - (altitude - centre_z) ** 2, 0.04))
            for a in np.arange(0, 2 * math.pi, resolution / max(r, 0.2)):
                points.append((cx + r * math.cos(a), cy + r * math.sin(a)))
            continue
        for u in np.arange(-w / 2, w / 2 + 1e-6, resolution):
            for v in (-d / 2, d / 2):
                points.append((cx + c * u - sn * v, cy + sn * u + c * v))
        for v in np.arange(-d / 2, d / 2 + 1e-6, resolution):
            for u in (-w / 2, w / 2):
                points.append((cx + c * u - sn * v, cy + sn * u + c * v))
    return np.array(points)


def nearest_distance(a, b):
    """For each point of a, the distance to the nearest point of b."""
    from scipy.spatial import cKDTree
    return cKDTree(b).query(a)[0]


def evaluate(run_dir, world, altitude):
    run_dir = run_dir.rstrip('/')
    rows = []
    for line in open(os.path.join(run_dir, 'slam_poses.jsonl')):
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if 'slam' in row:
            rows.append(row)
    result = {'run': os.path.basename(run_dir)}
    if len(rows) < 20:
        result['error'] = 'no SLAM poses recorded'
        return result
    t = np.array([r['stamp'] for r in rows])
    slam = np.array([r['slam'] for r in rows])
    odom = np.array([r['odom'] for r in rows])
    slam[:, 2], odom[:, 2] = np.unwrap(slam[:, 2]), np.unwrap(odom[:, 2])
    tt, tx, ty, tyaw, tz = load_truth_pose(run_dir)
    keep = (t >= tt[0]) & (t <= tt[-1])
    # score only while the vehicle is in the mapped slice (SLAM is fed then)
    height = np.interp(t, tt, tz)
    in_slice = keep & (np.abs(height - altitude) <= 0.7)
    # The return leg is flown high above the slice, where SLAM receives no
    # scans and only carries the odometry forward. Score the first stay in
    # the slice (the search); the pass through the slice on the way down to
    # land is reported separately.
    idx = np.nonzero(in_slice)[0]
    landing_pass = None
    if len(idx):
        gaps = np.nonzero(np.diff(t[idx]) > 5.0)[0]
        if len(gaps):
            landing_pass = idx[gaps[0] + 1:]
            idx = idx[:gaps[0] + 1]
    keep = np.zeros(len(t), dtype=bool)
    keep[idx] = True
    all_t, all_slam = t, slam
    t, slam, odom = t[keep], slam[keep], odom[keep]
    truth_xy = np.column_stack([np.interp(t, tt, tx), np.interp(t, tt, ty)])
    truth_yaw = np.interp(t, tt, tyaw)
    path = np.concatenate([[0.0], np.cumsum(np.linalg.norm(
        np.diff(truth_xy, axis=0), axis=1))])
    slam_m, slam_err = trajectory_metrics(slam, truth_xy, truth_yaw, path)
    odom_m, odom_err = trajectory_metrics(odom, truth_xy, truth_yaw, path)
    if landing_pass is not None and len(landing_pass):
        # same first-sample alignment as above
        d_yaw = truth_yaw[0] - slam[0, 2]
        c, sn = math.cos(d_yaw), math.sin(d_yaw)
        rel = all_slam[landing_pass, :2] - slam[0, :2]
        est = np.column_stack([c * rel[:, 0] - sn * rel[:, 1],
                               sn * rel[:, 0] + c * rel[:, 1]]) + truth_xy[0]
        true = np.column_stack([np.interp(all_t[landing_pass], tt, tx),
                                np.interp(all_t[landing_pass], tt, ty)])
        slam_m['error_after_high_return_m'] = round(float(
            np.linalg.norm(est - true, axis=1).max()), 3)
    corr = np.array([r['map_odom'] for r in rows])[keep]
    jumps = np.linalg.norm(np.diff(corr[:, :2], axis=0), axis=1)
    result.update({
        'samples': int(len(t)), 'duration_s': round(float(t[-1] - t[0]), 1),
        'path_length_m': round(float(path[-1]), 1),
        'slam': slam_m, 'odometry_given_to_slam': odom_m,
        'largest_map_correction_step_m': round(float(jumps.max()), 3),
        'correction_steps_over_0p3_m': int((jumps > 0.3).sum())})
    info = os.path.join(run_dir, 'slam_map_info.json')
    if os.path.exists(info):
        result['map_info'] = json.load(open(info))
    bridge = os.path.join(run_dir, 'slam_bridge.jsonl')
    if os.path.exists(bridge):
        last = None
        for line in open(bridge):
            last = line
        if last:
            result['bridge'] = json.loads(last)

    spec_path = os.path.join(WS, 'results/phase7/worlds', f'{world}.json')
    grid_path = os.path.join(run_dir, 'slam_map.npz')
    if os.path.exists(spec_path) and os.path.exists(grid_path):
        spec = json.load(open(spec_path))
        grid = np.load(grid_path)
        data, res, origin = grid['data'], float(grid['resolution']), \
            grid['origin']
        rows_i, cols_i = np.nonzero(data > 50)
        occupied = np.column_stack([origin[0] + (cols_i + 0.5) * res,
                                    origin[1] + (rows_i + 0.5) * res])
        # map frame -> world: the same first-sample alignment as the poses
        d_yaw = truth_yaw[0] - slam[0, 2]
        c, s = math.cos(d_yaw), math.sin(d_yaw)
        rel = occupied - slam[0, :2]
        occupied_w = np.column_stack([
            c * rel[:, 0] - s * rel[:, 1],
            s * rel[:, 0] + c * rel[:, 1]]) + truth_xy[0]
        outline = outline_cells(spec['structures'], altitude)
        reach = nearest_distance(outline, truth_xy[::5]) <= LIDAR_RANGE_M
        result['map'] = {
            'resolution_m': res,
            'known_area_m2': round(float((data >= 0).sum()) * res * res, 0),
            'occupied_cells': int(len(occupied)),
            'true_outline_points_in_range': int(reach.sum())}
        if len(occupied) and reach.any():
            d_occ = nearest_distance(occupied_w, outline)
            d_out = nearest_distance(outline[reach], occupied_w)
            result['map'].update({
                'precision': round(float((d_occ <= MATCH_M).mean()), 3),
                'recall_in_range': round(float((d_out <= MATCH_M).mean()),
                                         3),
                'occupied_cell_error_median_m': round(float(
                    np.median(d_occ)), 3),
                'occupied_cell_error_p95_m': round(float(
                    np.percentile(d_occ, 95)), 3)})
            # the same with the map placed by the best rigid fit of the
            # SLAM trajectory to the truth (shape of the map, without the
            # error of anchoring it at one sample)
            mean_s, mean_t = slam[:, :2].mean(axis=0), truth_xy.mean(axis=0)
            u, _, vt = np.linalg.svd((slam[:, :2] - mean_s).T
                                     @ (truth_xy - mean_t))
            rot = u @ np.diag([1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
            occupied_f = (occupied - mean_s) @ rot + mean_t
            f_occ = nearest_distance(occupied_f, outline)
            f_out = nearest_distance(outline[reach], occupied_f)
            result['map'].update({
                'rigid_fit_precision': round(float(
                    (f_occ <= MATCH_M).mean()), 3),
                'rigid_fit_recall_in_range': round(float(
                    (f_out <= MATCH_M).mean()), 3),
                'rigid_fit_cell_error_median_m': round(float(
                    np.median(f_occ)), 3)})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--world', default='village_C')
    parser.add_argument('--altitude', type=float, default=6.0)
    parser.add_argument('--out')
    args = parser.parse_args()
    results = {}
    for run_dir in args.runs:
        r = evaluate(run_dir, args.world, args.altitude)
        results[r['run']] = r
        print(f'== {r["run"]}')
        if 'error' in r:
            print('   ' + r['error'])
            continue
        print(f'   path {r["path_length_m"]} m in {r["duration_s"]} s, '
              f'{r["samples"]} samples')
        for key in ('slam', 'odometry_given_to_slam'):
            m = r[key]
            print(f'   {key:24s} RMSE {m["position_rmse_m"]} m  max '
                  f'{m["position_max_m"]} m  end {m["position_end_m"]} m '
                  f'({m.get("end_error_percent_of_path", "-")} % of path)  '
                  f'yaw mean {m["yaw_mean_deg"]} max {m["yaw_max_deg"]} deg'
                  f'  | rigid fit RMSE {m["rigid_fit_rmse_m"]} m max '
                  f'{m["rigid_fit_max_m"]} m')
        if 'map' in r:
            print(f'   map: {r["map"]}')
        print(f'   bridge {r.get("bridge")}; corrections over 0.3 m: '
              f'{r["correction_steps_over_0p3_m"]} (largest '
              f'{r["largest_map_correction_step_m"]} m)')
    if args.out:
        with open(args.out, 'w') as handle:
            json.dump(results, handle, indent=1)


if __name__ == '__main__':
    main()
