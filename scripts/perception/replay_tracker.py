#!/usr/bin/env python3
"""Replay recorded detections through the tracker and evaluate it offline.

The tracker receives exactly what the detector published live: for every
frame the sensor timestamp and the local-frame detections. Gazebo ground
truth (poses.csv) is read afterwards, only to score the result.

With --tracks the tracker is not run; a tracks.jsonl logged from the live
tracker node is scored instead (same metrics).
"""

import argparse
import csv
import json
import os
import time
from collections import defaultdict

import numpy as np
from scipy.optimize import linear_sum_assignment

from uav_autonomy.debris_tracking import (
    COASTING,
    CONFIRMED,
    DebrisTracker,
    Measurement,
    TrackerParams,
)

MATCH_M = 0.6


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_truth(run_dir, uav_name, min_altitude=2.8):
    series = defaultdict(list)
    with open(os.path.join(run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            try:
                series[row['name']].append([float(row[k]) for k in (
                    't_sim', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')])
            except (TypeError, ValueError):
                continue
    series = {k: np.array(v) for k, v in series.items()}
    uav = series[uav_name]
    base = series['base_link'][0]
    origin = uav[0, 1:4] + quat_to_matrix(*uav[0, 4:8]) @ base[1:4]
    releases = {r['name']: r for r in json.load(
        open(os.path.join(run_dir, 'releases.json')))}
    debris = {}
    for name, tr in series.items():
        if not name.startswith('p1_debris') or name not in releases:
            continue
        t = tr[:, 0]
        # truth in the PX4 local NED frame (origin = parked base_link)
        ned = np.column_stack([tr[:, 2] - origin[1], tr[:, 1] - origin[0],
                               -(tr[:, 3] - origin[2])])
        vel = np.gradient(ned, t, axis=0)
        vz = vel[:, 2]
        # End of free fall = first impact: the downward speed, which grows
        # monotonically under gravity, suddenly drops. The numerical
        # derivative smears an impact over neighbouring samples, so the fall
        # is taken to end 40 ms before the first drop.
        end = t[-1]
        drops = np.nonzero(vz[1:] < vz[:-1] - 0.3)[0]
        if len(drops):
            end = max(t[0], t[drops[0]] - 0.04)
        debris[name] = {'t': t, 'p': ned, 'v': vel, 'quat': tr[:, 4:8],
                        'fall_end': end, 'size': releases[name]['size']}
    hover = uav[uav[:, 3] > min_altitude]
    return debris, (hover[0, 0], hover[-1, 0]), uav, origin


def truth_at(obj, t):
    if t < obj['t'][0] or t > obj['t'][-1]:
        return None
    p = np.array([np.interp(t, obj['t'], obj['p'][:, i]) for i in range(3)])
    v = np.array([np.interp(t, obj['t'], obj['v'][:, i]) for i in range(3)])
    return p, v


def surface_distance(p, centre, size):
    """Distance from a point to an axis-aligned box of the given size."""
    half = np.asarray(size) / 2.0
    # NED axes: north = world y, east = world x, so swap the first two sizes
    half = np.array([half[1], half[0], half[2]])
    outside = np.maximum(np.abs(p - centre) - half, 0.0)
    return float(np.linalg.norm(outside))


def rmse(values):
    v = np.asarray(values, dtype=float)
    return round(float(np.sqrt(np.mean(v ** 2))), 4) if len(v) else None


def stats(values, digits=3):
    v = np.asarray(values, dtype=float)
    if not len(v):
        return None
    return {'n': int(len(v)), 'mean': round(float(v.mean()), digits),
            'median': round(float(np.median(v)), digits),
            'p95': round(float(np.percentile(np.abs(v), 95)), digits),
            'max_abs': round(float(np.abs(v).max()), digits)}


def run_tracker(run_dir, params):
    """Return per-frame track snapshots and timing from the recorded log."""
    tracker = DebrisTracker(params)
    frames, proc = [], []
    with open(os.path.join(run_dir, 'detections.jsonl')) as handle:
        for line in handle:
            try:
                d = json.loads(line)['d']
            except (json.JSONDecodeError, KeyError):
                continue
            meas = [Measurement(np.array(i['p_local'], dtype=float),
                                i['size_m'], i['confidence'],
                                bool(i.get('touches_border', False)))
                    for i in d['items'] if i['p_local'] is not None]
            started = time.perf_counter()
            tracks = tracker.step(d['stamp'], meas)
            proc.append((time.perf_counter() - started) * 1e3)
            if tracks is None:
                continue
            frames.append({'stamp': d['stamp'], 'n_meas': len(meas),
                           'tracks': [{
                               'id': tr.id, 'p': tr.x[:3].tolist(),
                               'v': tr.x[3:].tolist(),
                               'P': np.diag(tr.P).tolist(),
                               'status': tr.status, 'hits': tr.hits,
                               'tsu': tr.time_since_update, 'age': tr.age,
                               'dt': tr.last_dt, 'nis': tr.last_nis,
                               'innovation': (tr.last_innovation.tolist()
                                              if tr.last_innovation
                                              is not None else None)}
                               for tr in tracks]})
    return frames, proc, tracker


def load_logged_tracks(path):
    frames, proc = [], []
    with open(path) as handle:
        for line in handle:
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            frames.append(d)
            if 'processing_ms' in d:
                proc.append(d['processing_ms'])
    return frames, proc


def evaluate(frames, debris, hover, label):
    live = (CONFIRMED, COASTING)
    per_obj = defaultdict(lambda: {'ids': [], 'frames': 0, 'tracked': 0,
                                   'first_seen': None, 'first_conf': None})
    err_surface, err_centre, err_vel, err_vz, err_vxy = [], [], [], [], []
    speeds, nis, dts, coast_err = [], [], [], []
    coast_vel, young_vel = [], []
    previous_pair = {}
    naive_ids = defaultdict(list)
    track_matched = defaultdict(int)
    track_frames = defaultdict(int)
    gap_cases = []
    prev_stamp = None
    for frame in frames:
        t = frame['stamp']
        gap = t - prev_stamp if prev_stamp is not None else 0.0
        prev_stamp = t
        if not hover[0] <= t <= hover[1]:
            continue
        dts.append(gap)
        truth = {n: truth_at(o, t) for n, o in debris.items()}
        truth = {n: v for n, v in truth.items()
                 if v is not None and t <= debris[n]['fall_end']}
        for tr in frame['tracks']:
            if tr['status'] in live:
                track_frames[tr['id']] += 1
            if tr.get('nis') is not None and tr['status'] in live:
                nis.append(tr['nis'])
        # Truth-to-track correspondence, CLEAR MOT convention: a pairing
        # from the previous frame is kept while it is still within the
        # match distance; the rest is solved by minimum-cost assignment.
        names = list(truth)
        pairs = {}
        by_id = {tr['id']: tr for tr in frame['tracks']}
        taken = set()
        for n in names:
            tid = previous_pair.get(n)
            if tid in by_id and tid not in taken:
                d = float(np.linalg.norm(np.array(by_id[tid]['p'])
                                         - truth[n][0]))
                if d <= MATCH_M:
                    pairs[n] = (d, by_id[tid])
                    taken.add(tid)
        free_n = [n for n in names if n not in pairs]
        free_t = [tr for tr in frame['tracks'] if tr['id'] not in taken]
        if free_n and free_t:
            cost = np.array([[np.linalg.norm(np.array(tr['p']) - truth[n][0])
                              for tr in free_t] for n in free_n])
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] <= MATCH_M:
                    pairs[free_n[i]] = (float(cost[i, j]), free_t[j])
        for n, (_, tr) in pairs.items():
            previous_pair[n] = tr['id']
        # the same without carry-over, for comparison
        if names and frame['tracks']:
            cost = np.array([[np.linalg.norm(np.array(tr['p']) - truth[n][0])
                              for tr in frame['tracks']] for n in names])
            for i, j in zip(*linear_sum_assignment(cost)):
                tr = frame['tracks'][j]
                if cost[i, j] <= MATCH_M and tr['status'] in live:
                    naive_ids[names[i]].append(tr['id'])
        for name, (p, v) in truth.items():
            best = pairs.get(name)
            o = per_obj[name]
            # the object counts once any track (even tentative) is on it
            if best is None:
                if o['first_seen'] is not None:
                    o['frames'] += 1
                continue
            if o['first_seen'] is None:
                o['first_seen'] = t
            o['frames'] += 1
            d, tr = best
            if tr['status'] not in live:
                continue
            if o['first_conf'] is None:
                o['first_conf'] = t
            o['tracked'] += 1
            o['ids'].append(tr['id'])
            track_matched[tr['id']] += 1
            est_p, est_v = np.array(tr['p']), np.array(tr['v'])
            e_surface = surface_distance(est_p, p, debris[name]['size'])
            e_vel = float(np.linalg.norm(est_v - v))
            if tr['status'] == COASTING:
                coast_err.append(e_surface)
                coast_vel.append(e_vel)
                continue
            err_surface.append(e_surface)
            err_centre.append(float(d))
            err_vel.append(e_vel)
            err_vz.append(float(est_v[2] - v[2]))
            err_vxy.append(float(np.linalg.norm((est_v - v)[:2])))
            speeds.append(float(np.linalg.norm(v)))
            if tr['hits'] <= 4:
                young_vel.append(e_vel)
            if gap >= 0.09:
                gap_cases.append({'gap_s': round(gap, 3),
                                  'surface_error_m': round(e_surface, 3),
                                  'velocity_error_m_s': round(e_vel, 3)})

    switches, fragments, continuity = 0, 0, []
    untracked = []
    for name, o in per_obj.items():
        ids = o['ids']
        changes = sum(1 for a, b in zip(ids, ids[1:]) if a != b)
        switches += changes
        fragments += len(set(ids))
        if o['frames']:
            continuity.append(o['tracked'] / o['frames'])
        if not ids:
            untracked.append(name)
    never_seen = [n for n in debris if n not in per_obj
                  and hover[0] <= debris[n]['t'][0] <= hover[1]]
    false_tracks = [tid for tid, n in track_frames.items()
                    if track_matched.get(tid, 0) == 0]
    conf_delay = [o['first_conf'] - o['first_seen'] for o in per_obj.values()
                  if o['first_conf'] is not None]
    worst_gap = max(gap_cases, key=lambda g: g['gap_s']) if gap_cases else None
    return {
        'label': label,
        'note': 'position/velocity errors are for measurement-updated '
                '(CONFIRMED) states; COASTING states are reported separately',
        'frames_in_hover': len(dts),
        'frame_gap_ms': stats(np.array(dts[1:]) * 1e3, 1),
        'objects_with_any_track': len(per_obj),
        'objects_with_confirmed_track': len(per_obj) - len(untracked),
        'objects_never_tracked': untracked,
        'objects_released_in_hover_without_any_track': never_seen,
        'position_rmse_to_true_surface_m': rmse(err_surface),
        'position_rmse_to_true_centre_m': rmse(err_centre),
        'position_error_to_surface_m': stats(err_surface),
        'velocity_rmse_m_s': rmse(err_vel),
        'velocity_error_norm_m_s': stats(err_vel),
        'vertical_velocity_bias_m_s': stats(err_vz),
        'horizontal_velocity_error_m_s': stats(err_vxy),
        'true_speed_range_m_s': [round(min(speeds), 2), round(max(speeds), 2)]
        if speeds else None,
        'id_switches': switches,
        'id_switches_without_carry_over': sum(
            sum(1 for a, b in zip(ids, ids[1:]) if a != b)
            for ids in naive_ids.values()),
        'mean_track_ids_per_object': round(
            fragments / max(1, len(per_obj) - len(untracked)), 3),
        'track_continuity_mean': round(float(np.mean(continuity)), 4)
        if continuity else None,
        'track_continuity_min': round(float(np.min(continuity)), 4)
        if continuity else None,
        'confirmation_delay_s': stats(conf_delay),
        'confirmed_tracks_total': len(track_frames),
        'false_confirmed_tracks': len(false_tracks),
        'nis_mean': round(float(np.mean(nis)), 3) if nis else None,
        'nis_fraction_above_gate': round(float(np.mean(
            np.array(nis) > 16.27)), 4) if nis else None,
        'coasting_surface_error_m': stats(coast_err),
        'coasting_velocity_error_m_s': stats(coast_vel),
        'velocity_error_first_two_confirmed_frames_m_s': stats(young_vel),
        'frames_after_gap_ge_90ms': len(gap_cases),
        'after_gap_surface_error_m': stats(
            [g['surface_error_m'] for g in gap_cases]),
        'after_gap_velocity_error_m_s': stats(
            [g['velocity_error_m_s'] for g in gap_cases]),
        'longest_gap_case': worst_gap,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dirs', nargs='+')
    parser.add_argument('--uav', default='x500_rescue_0')
    parser.add_argument('--model', choices=('cv', 'gravity', 'both'),
                        default='both')
    parser.add_argument('--tracks', action='store_true',
                        help='score tracks.jsonl logged from the live node')
    parser.add_argument('--out-name', default='tracking_evaluation.json')
    parser.add_argument('--min-altitude', type=float, default=2.8,
                        help='evaluate while the true UAV height exceeds '
                             'this (2.8 = hover only)')
    args = parser.parse_args()
    models = {'cv': TrackerParams(gravity_m_s2=0.0, sigma_accel_z=12.0),
              'gravity': TrackerParams()}
    chosen = models if args.model == 'both' else {
        args.model: models[args.model]}
    for run_dir in args.run_dirs:
        debris, hover, _, _ = load_truth(run_dir, args.uav,
                                         args.min_altitude)
        report = {'run': os.path.basename(os.path.abspath(run_dir))}
        if args.tracks:
            frames, proc = load_logged_tracks(
                os.path.join(run_dir, 'tracks.jsonl'))
            report['live'] = evaluate(frames, debris, hover, 'live node')
            report['live']['processing_ms'] = stats(proc, 3)
        else:
            for name, params in chosen.items():
                frames, proc, tracker = run_tracker(run_dir, params)
                result = evaluate(frames, debris, hover, name)
                result['processing_ms'] = stats(proc, 3)
                result['dropped_frames'] = tracker.dropped_frames
                report[name] = result
        out = os.path.join(run_dir, 'analysis')
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, args.out_name), 'w') as handle:
            json.dump(report, handle, indent=1)
        for key, r in report.items():
            if not isinstance(r, dict):
                continue
            print(report['run'], key, json.dumps({k: r[k] for k in (
                'objects_with_confirmed_track', 'objects_never_tracked',
                'position_rmse_to_true_surface_m',
                'position_rmse_to_true_centre_m', 'velocity_rmse_m_s',
                'vertical_velocity_bias_m_s', 'id_switches',
                'track_continuity_mean', 'false_confirmed_tracks',
                'nis_mean', 'longest_gap_case', 'processing_ms')}))


if __name__ == '__main__':
    main()
