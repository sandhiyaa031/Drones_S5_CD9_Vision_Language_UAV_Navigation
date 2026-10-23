#!/usr/bin/env python3
"""OFFLINE scoring of a search flight (Phase 9).

Reads what was recorded during the flight (planner status, goals, located
candidates, PX4 flight log) and scores it against simulator ground truth
(poses.csv, world file). Ground truth is used here only.

Reported per run
    outcome and timing     controller outcome, search duration, path length
    motion                 max speed and horizontal acceleration in SEARCH
    coverage               true coverage of the search area (camera footprint
                           from the simulator pose) vs the planner's estimate
    survivors              located or not, final error, time from search
                           start to first report, largest off-nadir angle at
                           which the camera saw each one
    inspections            which candidate, what it really was, result

Usage: evaluate_search.py [--out FILE] RUN...
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(WS, 'src', 'uav_autonomy'))
sys.path.insert(0, os.path.join(WS, 'scripts', 'survivor'))

import evaluate_localization as loc_eval  # noqa: E402
from uav_autonomy.search_planning import (  # noqa: E402
    Camera, CoverageGrid, PlannerParams, SearchArea, footprint_polygon_rot)

CAM = Camera(fx=loc_eval.FOCAL, fy=loc_eval.FOCAL, width=loc_eval.IMAGE_W,
             height=loc_eval.IMAGE_H)
ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def quat_xyzw_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_jsonl(path):
    rows = []
    if not os.path.exists(path):
        return rows
    for line in open(path):
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue
    return rows


def true_poses(run_dir, offset):
    """Simulator pose of the vehicle in the local frame: t, p_ned, R."""
    t, p, rot = [], [], []
    with open(os.path.join(run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            if row['name'] != 'x500_rescue_0':
                continue
            t.append(float(row['t_sim']))
            x, y, z = (float(row[k]) for k in ('x', 'y', 'z'))
            p.append([y - offset[0], x - offset[1], -z])
            r_enu = quat_xyzw_matrix(*(float(row[k]) for k in (
                'qx', 'qy', 'qz', 'qw')))
            rot.append(ENU_TO_NED @ r_enu @ FLU_TO_FRD)
    return np.array(t), np.array(p), rot


def evaluate(run_dir):
    run_dir = run_dir.rstrip('/')
    info = open(os.path.join(run_dir, 'trial_info.txt')).read()
    outcome = next((tok.split('=', 1)[1] for tok in info.split()
                    if tok.startswith('controller_outcome=')), 'UNKNOWN')
    world, spec = loc_eval.load_world(run_dir)
    status = load_jsonl(os.path.join(run_dir, 'search_status.jsonl'))
    goals = load_jsonl(os.path.join(run_dir, 'search_goals.jsonl'))
    flight = load_jsonl(os.path.join(run_dir, 'flight_status.jsonl'))
    result = {'world': world, 'controller_outcome': outcome,
              'flight_states': [f['state'] for f in flight]}
    if not status:
        result['error'] = 'no planner status recorded'
        return result
    final = status[-1]
    area = SearchArea(*final['area'])
    t_start = next(s['stamp'] for s in status if s['state'] != 'IDLE')
    complete = [g['stamp'] for g in goals if g['complete']]
    t_end = complete[0] if complete else final['stamp']
    result.update({
        'area_local': final['area'], 'home': final['home'],
        'waypoints': final['waypoints'],
        'planned_path_m': round(float(sum(
            math.dist(a, b) for a, b in zip(final['waypoints'][:-1],
                                            final['waypoints'][1:]))), 1),
        'search_complete': bool(complete),
        'search_duration_s': round(t_end - t_start, 1),
        'planner_coverage': final['coverage'],
        'planner_coverage_oblique': final['coverage_oblique']})

    # PX4's own estimate during the search: speed and acceleration
    from pyulog import ULog
    log = ULog(os.path.join(run_dir, 'flight.ulg'),
               ['vehicle_local_position'])
    lp = log.data_list[0].data
    tl = lp['timestamp_sample'] * 1e-6
    mask = (tl >= t_start) & (tl <= t_end)
    speed = np.hypot(lp['vx'], lp['vy'])[mask]
    accel = np.hypot(lp['ax'], lp['ay'])[mask]
    xy = np.column_stack([lp['x'], lp['y']])[mask]
    result.update({
        'flown_path_m': round(float(np.hypot(*np.diff(xy, axis=0).T).sum()),
                              1),
        'max_speed_m_s': round(float(speed.max()), 2),
        'mean_speed_m_s': round(float(speed.mean()), 2),
        'max_horizontal_accel_m_s2': round(float(accel.max()), 2),
        'p95_horizontal_accel_m_s2': round(float(
            np.percentile(accel, 95)), 2),
        'altitude_range_m': [round(float(-lp['z'][mask].max()), 2),
                             round(float(-lp['z'][mask].min()), 2)]})

    # true coverage from the simulator pose
    t, p, q, _ = loc_eval.load_ulog_pose(run_dir)
    offset = loc_eval.world_to_local(run_dir, t, p)
    tg, pg, rg = true_poses(run_dir, offset)
    params = PlannerParams()
    grid = CoverageGrid(area, params.cell_m)
    plane = -params.target_height_m          # true ground is world z = 0
    last = -1.0
    for stamp, position, rot in zip(tg, pg, rg):
        if stamp < t_start or stamp > t_end or stamp - last < 0.1:
            continue
        last = stamp
        polygon = footprint_polygon_rot(position, rot, CAM, plane,
                                        params.footprint_margin,
                                        params.max_tilt_deg)
        grid.update(polygon, position[:2], plane - position[2],
                    params.oblique_deg)
    result.update({'true_coverage': round(grid.fraction(), 4),
                   'true_coverage_oblique': round(grid.oblique_fraction(),
                                                  4)})

    # survivors
    rows, _, _ = loc_eval.load_live(run_dir)
    score = loc_eval.score(run_dir, rows, spec)
    survivors = []
    in_area = 0
    for s in spec['survivors']:
        truth = (s['position'][1] - offset[0], s['position'][0] - offset[1])
        inside = bool(area.contains(truth))
        in_area += inside
        entries = [e for e in score['entities'] if e['match'] == s['name']]
        item = {'name': s['name'], 'truth_ned': [round(v, 2) for v in truth],
                'inside_area': bool(inside), 'located': bool(entries),
                'entries': len(entries)}
        if entries:
            best = min(entries, key=lambda e: e['error_final_m'])
            item.update({
                'final_error_m': best['error_final_m'],
                'sigma_h_m': best['sigma_h'],
                'baseline_m': best['baseline_m'],
                'observations': best['observations'],
                'vlm_confirmed': best['vlm_confirmed'],
                'time_to_first_report_s': round(
                    min(e['first_stamp'] for e in entries) - t_start, 1)})
            # largest off-nadir angle at which it was observed
            ids = {e['id'] for e in entries}
            stamps = [r['stamp'] for r in rows
                      if r['id'] in ids and r.get('updated')]
            angles = []
            for stamp in stamps[::3]:
                i = int(np.clip(np.searchsorted(tg, stamp), 0, len(tg) - 1))
                d = math.hypot(pg[i][0] - truth[0], pg[i][1] - truth[1])
                angles.append(math.degrees(math.atan2(
                    d, -pg[i][2] - params.target_height_m)))
            if angles:
                item['off_nadir_deg'] = [round(min(angles), 1),
                                         round(max(angles), 1)]
        survivors.append(item)
    result.update({
        'survivors': survivors,
        'survivors_in_area': int(in_area),
        'survivors_located': int(sum(s['located'] and s['inside_area']
                                     for s in survivors)),
        'other_entries': [
            {'match': e['match'], 'kind': e['match_kind'],
             'final_error_m': e.get('error_final_m')}
            for e in score['entities'] if e['match_kind'] != 'survivor']})

    # inspections
    by_id = {e['id']: e for e in score['entities']}
    inspections = []
    for ins in final['inspections']:
        entity = by_id.get(ins['id'])
        inspections.append({
            'id': ins['id'], 'result': ins['result'],
            'duration_s': None if ins['ended'] is None else round(
                ins['ended'] - ins['started'], 1),
            'what_it_was': None if entity is None else entity['match'],
            'kind': None if entity is None else entity['match_kind'],
            'views': len(ins['viewpoints'])})
    result.update({
        'inspections': inspections,
        'inspection_time_s': round(sum(
            i['duration_s'] or 0.0 for i in inspections), 1),
        'candidates_skipped': final['skipped']})

    # debris, if the run had any
    releases = os.path.join(run_dir, 'releases.json')
    if os.path.exists(releases):
        avoided = [f for f in flight if f.get('avoidance_active')]
        result['debris'] = {
            'releases': len(json.load(open(releases))),
            'avoidance_manoeuvres': max(
                [f['avoidance_accepted'] for f in flight] or [0]),
            'avoidance_during_search': sum(
                1 for f in avoided if f['state'] == 'SEARCH')}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--out')
    args = parser.parse_args()
    results = {}
    for run_dir in args.runs:
        r = evaluate(run_dir)
        results[os.path.basename(run_dir.rstrip('/'))] = r
        print(f'== {os.path.basename(run_dir.rstrip("/"))} [{r["world"]}] '
              f'outcome {r["controller_outcome"]}')
        if 'error' in r:
            print('   ' + r['error'])
            continue
        print(f'   search {"complete" if r["search_complete"] else "NOT complete"}'
              f' in {r["search_duration_s"]} s; path planned '
              f'{r["planned_path_m"]} m flown {r["flown_path_m"]} m; speed '
              f'max {r["max_speed_m_s"]} mean {r["mean_speed_m_s"]} m/s; '
              f'accel max {r["max_horizontal_accel_m_s2"]} p95 '
              f'{r["p95_horizontal_accel_m_s2"]} m/s2')
        print(f'   coverage true {r["true_coverage"]:.3f} (oblique '
              f'{r["true_coverage_oblique"]:.3f}); planner estimate '
              f'{r["planner_coverage"]:.3f} (oblique '
              f'{r["planner_coverage_oblique"]:.3f})')
        print(f'   survivors located {r["survivors_located"]}/'
              f'{r["survivors_in_area"]}')
        for s in r['survivors']:
            if s['located']:
                print(f'     {s["name"]}: error {s["final_error_m"]} m sigma '
                      f'{s["sigma_h_m"]} m, first report after '
                      f'{s["time_to_first_report_s"]} s, off-nadir '
                      f'{s.get("off_nadir_deg")} deg, baseline '
                      f'{s["baseline_m"]} m, entries {s["entries"]}, '
                      f'confirmed {s["vlm_confirmed"]}')
            else:
                print(f'     {s["name"]}: NOT located (inside area: '
                      f'{s["inside_area"]})')
        for i in r['inspections']:
            print(f'     inspection #{i["id"]} -> {i["what_it_was"]} '
                  f'({i["kind"]}): {i["result"]} in {i["duration_s"]} s')
        if r['other_entries']:
            print('     other located objects: ' + ', '.join(
                f'{e["match"]}' for e in r['other_entries']))
        if 'debris' in r:
            print(f'     debris: {r["debris"]}')
    if args.out:
        with open(args.out, 'w') as handle:
            json.dump(results, handle, indent=1)


if __name__ == '__main__':
    main()
