#!/usr/bin/env python3
"""OFFLINE timing of the navigator's route planning on recorded SLAM maps.

For each run folder with a slam_map.npz: build the planning grid exactly as
the navigator does and plan routes between random free points of the map.
Reports the time to build the grid and to plan one route, and how many of
the random pairs had a route. No simulator, no ROS.

Usage: benchmark_planner.py [--pairs 200] [--out FILE] RUN...
"""

import argparse
import json
import os
import sys
import time

import numpy as np

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(WS, 'src', 'uav_autonomy'))
from uav_autonomy.navigation import NavParams, PlanningGrid, astar  # noqa: E402


def stats(values):
    v = np.array(values)
    return {'n': int(len(v)), 'mean_ms': round(float(v.mean()), 2),
            'median_ms': round(float(np.median(v)), 2),
            'p95_ms': round(float(np.percentile(v, 95)), 2),
            'max_ms': round(float(v.max()), 2)}


def benchmark(run_dir, pairs, rng):
    grid_file = np.load(os.path.join(run_dir, 'slam_map.npz'))
    data, res = grid_file['data'], float(grid_file['resolution'])
    origin = grid_file['origin']
    params = NavParams()
    rows, cols = np.nonzero(data == 0)              # known free cells
    points = np.column_stack([origin[0] + (cols + 0.5) * res,
                              origin[1] + (rows + 0.5) * res])
    build, plan, length, found = [], [], [], 0
    for _ in range(pairs):
        a, b = points[rng.integers(len(points), size=2)]
        t0 = time.perf_counter()
        grid = PlanningGrid(data, res, origin, [tuple(a), tuple(b)], params)
        build.append(1000.0 * (time.perf_counter() - t0))
        t0 = time.perf_counter()
        path, _ = astar(grid, tuple(a), tuple(b), params)
        plan.append(1000.0 * (time.perf_counter() - t0))
        if path is not None:
            found += 1
            length.append(float(np.linalg.norm(
                np.diff(np.array(path), axis=0), axis=1).sum()))
    return {'map_cells': int(data.size), 'map_shape': list(data.shape),
            'pairs': pairs, 'routes_found': found,
            'route_length_mean_m': round(float(np.mean(length)), 1)
            if length else None,
            'grid_build': stats(build), 'route_plan': stats(plan)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--pairs', type=int, default=200)
    parser.add_argument('--out')
    args = parser.parse_args()
    rng = np.random.default_rng(7)
    results = {}
    for run_dir in args.runs:
        run_dir = run_dir.rstrip('/')
        if not os.path.exists(os.path.join(run_dir, 'slam_map.npz')):
            continue
        r = benchmark(run_dir, args.pairs, rng)
        results[os.path.basename(run_dir)] = r
        print(f'{os.path.basename(run_dir)}: map {r["map_shape"]}, routes '
              f'{r["routes_found"]}/{r["pairs"]}, grid build '
              f'{r["grid_build"]}, plan {r["route_plan"]}')
    if args.out:
        with open(args.out, 'w') as handle:
            json.dump(results, handle, indent=1)


if __name__ == '__main__':
    main()
