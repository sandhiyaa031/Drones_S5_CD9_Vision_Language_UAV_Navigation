#!/usr/bin/env python3
"""Offline evaluation of the debris detector against Gazebo ground truth.

Inputs in one run directory:
    detections.jsonl   what the detector published (log_detections.py)
    poses.csv          ground-truth poses (record.py), evaluation only
    releases.json      box sizes of the released objects

For every detector frame taken while the UAV hovers, each ground-truth box is
transformed into the true sensor frame. A box "should be detected" when it is
still falling, its centre projects inside the image, and its near face is
between the detector's minimum range and the sensor's far clip. Detections are
matched to boxes by distance to the true box surface.
"""

import argparse
import csv
import json
import math
import os
from collections import defaultdict

import numpy as np

FX = 268.5118688877865
CX, CY, W, H = 320.0, 240.0, 640, 480
MATCH_M = 0.35


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def interp(track, t):
    ts = track[:, 0]
    if t < ts[0] or t > ts[-1]:
        return None
    pos = np.array([np.interp(t, ts, track[:, i]) for i in (1, 2, 3)])
    return pos, track[int(np.argmin(np.abs(ts - t))), 4:8]


def fall_end(track):
    t = track[:, 0]
    vz = np.gradient(track[:, 3], t)
    peak = 0.0
    for i in range(1, len(t)):
        peak = min(peak, vz[i])
        if peak < -2.0 and vz[i] > 0.4 * peak:
            return t[i]
    return t[-1]


def box_surface_distance(p_w, centre, quat, size):
    local = quat_to_matrix(*quat).T @ (p_w - centre)
    half = np.asarray(size) / 2.0
    outside = np.maximum(np.abs(local) - half, 0.0)
    d_out = float(np.linalg.norm(outside))
    return d_out if d_out > 0 else 0.0


def pct(values, scale=1.0, digits=3):
    v = np.asarray(values, dtype=float) * scale
    if not len(v):
        return None
    return {'n': int(len(v)), 'median': round(float(np.median(v)), digits),
            'p95': round(float(np.percentile(v, 95)), digits),
            'max': round(float(v.max()), digits)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dir')
    parser.add_argument('--uav', default='x500_rescue_0')
    parser.add_argument('--min-range', type=float, default=0.5)
    parser.add_argument('--max-range', type=float, default=35.0)
    args = parser.parse_args()

    series = defaultdict(list)
    with open(os.path.join(args.run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            try:
                series[row['name']].append([float(row[k]) for k in (
                    't_sim', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')])
            except (TypeError, ValueError):
                continue        # truncated last line
    series = {k: np.array(v) for k, v in series.items()}
    uav = series[args.uav]
    link = series['depth_up_link'][0]
    base = series['base_link'][0]
    r_ms, t_ms = quat_to_matrix(*link[4:8]), link[1:4]
    releases = {r['name']: r for r in json.load(
        open(os.path.join(args.run_dir, 'releases.json')))}
    debris = {n: tr for n, tr in series.items()
              if n.startswith('p1_debris') and len(tr) > 10
              and n in releases}
    t_fall_end = {n: fall_end(tr) for n, tr in debris.items()}
    # PX4 local origin = base_link position while parked (start of the log)
    origin_w = uav[0, 1:4] + quat_to_matrix(*uav[0, 4:8]) @ base[1:4]

    frames = []
    with open(os.path.join(args.run_dir, 'detections.jsonl')) as handle:
        for line in handle:
            try:
                frames.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    hover = uav[uav[:, 3] > 2.8]
    t0, t1 = hover[0, 0], hover[-1, 0]

    per_obj = defaultdict(lambda: {'should': 0, 'hit': 0, 'first_should': None,
                                   'first_hit': None, 'err_surface': [],
                                   'err_centre': [], 'err_local': [],
                                   'speed': [], 'range': [],
                                   'partial_hits': 0})
    false_pos = {'near_debris_after_impact': 0, 'near_debris_partial': 0,
                 'unexplained': 0}
    unexplained = []
    proc_ms, pose_ok, n_frames, n_det = [], 0, 0, 0
    stamps = []
    simultaneous = []
    for frame in frames:
        d = frame['d']
        t = d['stamp']
        if not t0 <= t <= t1:
            continue
        n_frames += 1
        stamps.append(t)
        proc_ms.append(d['processing_ms'])
        pose_ok += bool(d['pose_available'])
        up = interp(uav, t)
        r_wm = quat_to_matrix(*up[1])
        r_ws = r_wm @ r_ms
        sensor_w = up[0] + r_wm @ t_ms
        truth = {}
        for name, tr in debris.items():
            pose = interp(tr, t)
            if pose is None:
                continue
            p_s = r_ws.T @ (pose[0] - sensor_w)
            size = releases[name]['size']
            half = max(size) / 2
            u = CX - FX * p_s[1] / p_s[0] if p_s[0] > 0 else -1
            v = CY - FX * p_s[2] / p_s[0] if p_s[0] > 0 else -1
            in_image = p_s[0] > 0 and 0 <= u < W and 0 <= v < H
            falling = t <= t_fall_end[name]
            should = (falling and in_image
                      and p_s[0] - half >= args.min_range
                      and np.linalg.norm(p_s) + half <= args.max_range)
            i = int(np.argmin(np.abs(tr[:, 0] - t)))
            j0, j1 = max(i - 2, 0), min(i + 2, len(tr) - 1)
            speed = float(np.linalg.norm(tr[j1, 1:4] - tr[j0, 1:4]) /
                          max(1e-6, tr[j1, 0] - tr[j0, 0]))
            truth[name] = (pose, p_s, should, falling, speed)
            if should:
                o = per_obj[name]
                o['should'] += 1
                if o['first_should'] is None:
                    o['first_should'] = t
        simultaneous.append(sum(1 for v in truth.values() if v[2]))
        used = set()
        for item in d['items']:
            n_det += 1
            p_w = r_ws @ np.array(item['p_sensor']) + sensor_w
            best = None
            for name, (pose, p_s, should, falling, speed) in truth.items():
                dist = box_surface_distance(
                    p_w, pose[0], pose[1], releases[name]['size'])
                if best is None or dist < best[1]:
                    best = (name, dist)
            if best is None or best[1] > MATCH_M:
                false_pos['unexplained'] += 1
                unexplained.append({'t': t, 'p_sensor': item['p_sensor'],
                                    'size_m': item['size_m'],
                                    'pixels': item['pixels'],
                                    'nearest_truth_m': (round(best[1], 2)
                                                        if best else None)})
                continue
            name, dist = best
            pose, p_s, should, falling, speed = truth[name]
            if not falling:
                false_pos['near_debris_after_impact'] += 1
                continue
            o = per_obj[name]
            if not should:
                o['partial_hits'] += 1
                false_pos['near_debris_partial'] += 0
                continue
            if name in used:
                continue
            used.add(name)
            o['hit'] += 1
            if o['first_hit'] is None:
                o['first_hit'] = t
            o['err_surface'].append(dist)
            o['err_centre'].append(float(np.linalg.norm(p_w - pose[0])))
            o['speed'].append(speed)
            o['range'].append(item['range_m'])
            if item['p_local'] is not None:
                n, e, dn = item['p_local']
                est_w = np.array([e, n, -dn]) + origin_w
                o['err_local'].append(box_surface_distance(
                    est_w, pose[0], pose[1], releases[name]['size']))

    objects = []
    for name in sorted(debris):
        o = per_obj[name]
        if o['should'] == 0 and o['hit'] == 0 and o['partial_hits'] == 0:
            objects.append({'object': name,
                            'label': releases[name].get('label', ''),
                            'frames_should_be_detected': 0,
                            'note': 'never inside the field of view while '
                                    'falling during the hover'})
            continue
        objects.append({
            'object': name, 'label': releases[name].get('label', ''),
            'frames_should_be_detected': o['should'],
            'frames_detected': o['hit'],
            'detection_rate': round(o['hit'] / o['should'], 3)
            if o['should'] else None,
            'first_detection_delay_s':
                round(o['first_hit'] - o['first_should'], 3)
                if o['first_hit'] is not None
                and o['first_should'] is not None else None,
            'surface_error_m': pct(o['err_surface']),
            'centre_distance_m': pct(o['err_centre']),
            'local_frame_surface_error_m': pct(o['err_local']),
            'max_speed_m_s': round(max(o['speed']), 2) if o['speed'] else None,
            'range_m': [round(min(o['range']), 2), round(max(o['range']), 2)]
            if o['range'] else None,
            'extra_detections_while_partly_in_view': o['partial_hits']})

    seen = [o for o in objects if o.get('frames_should_be_detected')]
    all_surface = [e for n in debris for e in per_obj[n]['err_surface']]
    all_local = [e for n in debris for e in per_obj[n]['err_local']]
    all_speed = [s for n in debris for s in per_obj[n]['speed']]
    should_total = sum(o['frames_should_be_detected'] for o in seen)
    hit_total = sum(o['frames_detected'] for o in seen)
    dt = np.diff(stamps)
    summary = {
        'run': os.path.basename(os.path.abspath(args.run_dir)),
        'hover_frames_processed': n_frames,
        'detector_frame_rate_hz_sim_time': round(
            (len(stamps) - 1) / (stamps[-1] - stamps[0]), 2),
        'frame_stamp_step_ms': pct(dt, 1e3, 1),
        'pose_available_fraction': round(pose_ok / max(1, n_frames), 4),
        'processing_ms': pct(proc_ms, 1.0, 2),
        'objects_released': len(debris),
        'objects_that_should_be_detected': len(seen),
        'objects_detected_at_least_once': sum(
            1 for o in seen if o['frames_detected'] > 0),
        'max_simultaneous_objects_in_view': int(max(simultaneous))
        if simultaneous else 0,
        'frames_with_2_or_more_in_view': int(sum(
            1 for s in simultaneous if s >= 2)),
        'object_frames_should': should_total,
        'object_frames_detected': hit_total,
        'frame_level_recall': round(hit_total / max(1, should_total), 4),
        'detections_total': n_det,
        'detections_unexplained': false_pos['unexplained'],
        'detections_on_debris_after_first_impact':
            false_pos['near_debris_after_impact'],
        'surface_error_m': pct(all_surface),
        'local_frame_surface_error_m': pct(all_local),
        'surface_error_vs_speed_correlation': round(float(np.corrcoef(
            all_speed, all_surface)[0, 1]), 3) if len(all_surface) > 5
        else None,
        'unexplained_examples': unexplained[:10],
        'objects': objects}
    out_dir = os.path.join(args.run_dir, 'analysis')
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'detection_evaluation.json'), 'w') as h:
        json.dump(summary, h, indent=1)
    print(json.dumps({k: v for k, v in summary.items()
                      if k not in ('objects', 'unexplained_examples')},
                     indent=1))
    for o in objects:
        print(' ', o['object'][-2:], o.get('frames_detected'), '/',
              o['frames_should_be_detected'], 'delay',
              o.get('first_detection_delay_s'), 'surf',
              (o.get('surface_error_m') or {}).get('median'), 'vmax',
              o.get('max_speed_m_s'), o.get('note', ''))
    if math.isnan(summary['frame_level_recall']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
