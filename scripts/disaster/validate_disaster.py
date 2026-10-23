#!/usr/bin/env python3
"""Offline geometry checks of a generated disaster-zone world.

Reads the truth file (results/phase7/worlds/<scenario>.json), rasterises the
structures that cross the lidar slice (6 m) and plans routes with the
navigator's own planner (keep-out 1.75 m). Checks:
  1. height limits: standing structures <= 15 m, rubble <= 4.5 m
  2. structures that really cross the 6 m slice (the map can see them)
  3. routes the mission needs exist and keep their clearance
  4. the street block: the straight street route is closed and a detour exists
Truth is used here for OFFLINE validation of the world, never online.

Usage: validate_disaster.py [--scenario urban_C] [--out FILE]
"""

import argparse
import json
import math
import os
import sys

import numpy as np

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(WS, 'src', 'uav_autonomy'))
from uav_autonomy.navigation import NavParams, PlanningGrid, astar  # noqa

SLICE = 6.0
RES = 0.25


def raster(structures, x0, y0, nx, ny):
    data = np.zeros((ny, nx), dtype=np.int8)
    for s in structures:
        if not (s['z'][0] - 0.2 <= SLICE <= s['z'][1] + 0.2):
            continue
        cx, cy = s['centre']
        w, d = s['size']
        c, sn = math.cos(s['yaw']), math.sin(s['yaw'])
        for u in np.arange(-w / 2, w / 2 + 1e-6, RES / 2):
            for v in np.arange(-d / 2, d / 2 + 1e-6, RES / 2):
                x, y = cx + c * u - sn * v, cy + sn * u + c * v
                i, j = int((y - y0) / RES), int((x - x0) / RES)
                if 0 <= i < ny and 0 <= j < nx:
                    data[i, j] = 100
    return data


def route(grid, params, a, b):
    path, note = astar(grid, a, b, params)
    if path is None:
        return None, 0.0, 0.0, note
    length = float(np.linalg.norm(np.diff(np.array(path), axis=0),
                                  axis=1).sum())
    clear = min(grid.clearance[grid.cell(p)] for p in path
                if grid.inside(grid.cell(p)))
    return path, length, float(clear), note


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scenario', default='urban_C')
    parser.add_argument('--out')
    args = parser.parse_args()
    spec = json.load(open(os.path.join(
        WS, 'results/phase7/worlds', f'{args.scenario}.json')))
    st = spec['structures']
    report = {'scenario': args.scenario, 'checks': {}}
    standing = [s for s in st if s['kind'] in ('wall', 'structure', 'pole')]
    rubble = [s for s in st if s['kind'] == 'rubble']
    max_stand = max(s['z'][1] for s in standing)
    max_rubble = max(s['z'][1] for s in rubble)
    cross = [s for s in standing if s['z'][0] <= SLICE <= s['z'][1]]
    report['checks']['max_standing_height_m'] = (max_stand, max_stand <= 15.01)
    report['checks']['max_rubble_height_m'] = (max_rubble,
                                               max_rubble <= 4.5 + 1e-6)
    report['checks']['structures_crossing_6m_slice'] = (
        len(cross), len(cross) > 20)
    x0, y0, nx, ny = -60.0, -40.0, int(200 / RES), int(180 / RES)
    data = raster(st, x0, y0, nx, ny)
    params = NavParams()
    grid = PlanningGrid(data, RES, (x0, y0), [(0, 0), (100, 100)], params)
    legs = {
        'base -> collapsed block alley': ((0, 0), (24.0, 60.0)),
        'base -> courtyard centre': ((0, 0), (60.0, 13.0)),
        'base -> gap east of street (B2 front)': ((0, 0), (41.0, 43.0)),
        'base -> survivor spot block': ((0, 0), (17.0, 52.0)),
        'street south of the block -> north of it (detour)': ((33, 60),
                                                              (33.0, 90.0)),
        'street south of the block -> the block itself': ((33, 60),
                                                          (33.0, 76.0)),
        'base -> far east end of avenue': ((0, 0), (85.0, 27.0)),
    }
    report['routes'] = {}
    for name, (a, b) in legs.items():
        path, length, clear, note = route(grid, params, a, b)
        straight = math.hypot(b[0] - a[0], b[1] - a[1])
        report['routes'][name] = {
            'found': path is not None, 'length_m': round(length, 1),
            'straight_m': round(straight, 1),
            'min_clearance_m': round(clear, 2), 'note': note}
    detour = report['routes']['street south of the block -> north of it (detour)']
    report['checks']['street_block_forces_a_detour'] = (
        detour['length_m'] / max(detour['straight_m'], 1) if detour['found']
        else None, detour['found'] and detour['length_m'] >
        1.3 * detour['straight_m'])
    report['checks']['routes_needed_exist'] = (
        None, all(report['routes'][k]['found'] for k in (
            'base -> collapsed block alley', 'base -> courtyard centre',
            'base -> gap east of street (B2 front)',
            'street south of the block -> north of it (detour)')))
    ok = all(v[1] for v in report['checks'].values())
    report['passed'] = bool(ok)
    print(f'== {args.scenario}: {"PASS" if ok else "FAIL"}')
    for k, v in report['checks'].items():
        print(f'   {k}: {v[0]}  {"ok" if v[1] else "FAIL"}')
    for k, v in report['routes'].items():
        print(f'   route {k}: found={v["found"]} len={v["length_m"]} m '
              f'(straight {v["straight_m"]}) clearance>={v["min_clearance_m"]}'
              f' m {v["note"]}')
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        json.dump(report, open(args.out, 'w'), indent=1)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
