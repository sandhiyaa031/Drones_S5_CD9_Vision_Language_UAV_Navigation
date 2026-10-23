#!/usr/bin/env python3
"""OFFLINE summary of a set of mission runs (Phases 13 and 14).

For each run folder: controller and mission outcome, why the search ended,
whether PX4's log shows a landing, survivors reported and their position
error, coverage, search time, avoidance manoeuvres, and the smallest
distance between any falling box and the vehicle (simulator ground truth,
used here only). With --stats, also the distribution over the runs.

Usage: summarize_runs.py [--stats] [--out FILE] RUN...
"""

import argparse
import collections
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import evaluate_mission  # noqa: E402

CONTACT_M = 0.40      # UAV centre to box surface (Phase 5 definition)


def debris_distance(run_dir):
    """Smallest centre distance and surface distance of any released box."""
    releases = os.path.join(run_dir, 'releases.json')
    if not os.path.exists(releases):
        return None
    sizes = {r['name']: max(r['size']) for r in json.load(open(releases))}
    poses = collections.defaultdict(list)
    with open(os.path.join(run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            if row['name'] == 'x500_rescue_0' or row['name'] in sizes:
                poses[row['name']].append(
                    [float(row[k]) for k in ('t_sim', 'x', 'y', 'z')])
    uav = np.array(poses.pop('x500_rescue_0'))
    best = None
    for name, rows in poses.items():
        box = np.array(rows)
        at = np.column_stack([np.interp(box[:, 0], uav[:, 0], uav[:, k])
                              for k in (1, 2, 3)])
        # only while the vehicle is airborne
        mask = at[:, 2] > 0.5
        if not mask.any():
            continue
        d = np.linalg.norm(box[mask, 1:] - at[mask], axis=1)
        # box surface: centre distance minus the half-diagonal of a face
        surface = float(d.min()) - 0.71 * sizes[name]
        if best is None or surface < best['surface_m']:
            best = {'centre_m': round(float(d.min()), 2),
                    'surface_m': round(surface, 2)}
    if best is not None:
        best['releases'] = len(sizes)
        best['collision'] = best['surface_m'] <= CONTACT_M
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--stats', action='store_true')
    parser.add_argument('--out')
    args = parser.parse_args()
    rows = []
    for run_dir in args.runs:
        run_dir = run_dir.rstrip('/')
        r = evaluate_mission.evaluate(run_dir)
        info = open(os.path.join(run_dir, 'trial_info.txt')).read()
        kill = next((t.split('=', 1)[1] for t in info.split()
                     if t.startswith('kill=')), 'none@none')
        errors = [e['error_m'] for e in r.get('entries', [])
                  if e['truth_kind'] == 'survivor' and e['classification']
                  in ('SURVIVOR_CONFIRMED', 'UNVERIFIED_CANDIDATE')]
        row = {
            'run': os.path.basename(run_dir), 'fault': kill,
            'controller': r['controller_outcome'],
            'mission': r.get('mission_outcome'),
            'objective_met': r.get('objective_met'),
            'end_reason': r.get('search_end_reason'),
            'refused': r.get('refused_because'),
            'flew': r.get('flew'),
            'landed': None if not r.get('flew') else
            'landed' not in r.get('failed_flight_log_checks', ['landed']),
            'log_checks_failed': r.get('failed_flight_log_checks'),
            'search_s': r.get('search_time_s'),
            'coverage': r.get('coverage_fraction'),
            'survivors_in_area': len(r.get('survivors_in_area', [])),
            'survivors_reported': len(r.get('survivors_reported', [])),
            'survivor_errors_m': errors,
            'non_survivor_candidates': len(
                r.get('candidate_non_survivors', [])),
            'confirmed_non_survivors': len(
                r.get('confirmed_non_survivors', [])),
            'avoidance': r.get('avoidance_manoeuvres'),
            'debris': debris_distance(run_dir),
            'report_written': 'error' not in r}
        rows.append(row)
        d = row['debris']
        print(f'{row["run"]:44s} fault {row["fault"]:22s} ctrl '
              f'{row["controller"]:9s} mission {str(row["mission"]):9s} '
              f'landed {str(row["landed"]):5s} obj {str(row["objective_met"]):5s}'
              f' surv {row["survivors_reported"]}/{row["survivors_in_area"]}'
              f' err {errors} cov {row["coverage"]} t {row["search_s"]} '
              f'avoid {row["avoidance"]} '
              + (f'box min {d["surface_m"]} m' if d else '')
              + f' | {row["end_reason"] or row["refused"] or ""}')
    out = {'runs': rows}
    if args.stats:
        flown = [r for r in rows if r['flew']]
        errs = [e for r in flown for e in r['survivor_errors_m']]
        times = [r['search_s'] for r in flown if r['search_s'] is not None
                 and r['objective_met']]
        boxes = [r['debris'] for r in flown if r['debris']]
        stats = {
            'runs': len(rows), 'flown': len(flown),
            'landed': sum(1 for r in flown if r['landed']),
            'mission_complete': sum(1 for r in rows
                                    if r['mission'] == 'COMPLETE'),
            'objective_met': sum(1 for r in rows if r['objective_met']),
            'failsafe': sum(1 for r in rows
                            if r['controller'] == 'FAILSAFE'),
            'survivors_reported': sum(r['survivors_reported']
                                      for r in flown),
            'survivors_in_area': sum(r['survivors_in_area'] for r in flown),
            'confirmed_non_survivors': sum(r['confirmed_non_survivors']
                                           for r in flown),
            'survivor_error_m': None if not errs else {
                'n': len(errs), 'mean': round(float(np.mean(errs)), 3),
                'std': round(float(np.std(errs)), 3),
                'median': round(float(np.median(errs)), 3),
                'max': round(float(np.max(errs)), 3)},
            'search_time_s': None if not times else {
                'n': len(times), 'mean': round(float(np.mean(times)), 1),
                'std': round(float(np.std(times)), 1),
                'min': min(times), 'max': max(times)},
            'coverage_min': min((r['coverage'] for r in flown
                                 if r['objective_met']), default=None),
            'avoidance_manoeuvres': sum(r['avoidance'] or 0 for r in flown),
            'boxes_released': sum(b['releases'] for b in boxes),
            'box_collisions': sum(1 for b in boxes if b['collision']),
            'box_min_surface_m': min((b['surface_m'] for b in boxes),
                                     default=None)}
        out['stats'] = stats
        print(json.dumps(stats, indent=1))
    if args.out:
        with open(args.out, 'w') as handle:
            json.dump(out, handle, indent=1)


if __name__ == '__main__':
    main()
