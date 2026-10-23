#!/usr/bin/env python3
"""Flight tests of the standalone MuJoCo quadrotor (no ROS, no PX4, no Gazebo).

Scenarios
  hover      take off to 6 m, hold 20 s, land on the pad
  waypoints  take off, fly a 16 m square at 3 m/s, return, land
  nav        take off, fly to a goal around obstacles using the lidar map
             (known pose) and the project's A* navigator, return, land
             --goal east,north  (default 33,90: beyond the blocked street)

The controller is NOT PX4: it is the cascaded controller in
controllers/quadrotor.py, fed with the simulator's own state. The map is built
from simulated lidar rays with the known pose (not SLAM).

Usage: run_flight.py SCENARIO [--scene urban_C] [--gui] [--realtime]
                              [--out DIR] [--goal E,N]
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.abspath(os.path.join(
    ROOT, '..', '..', 'src', 'uav_autonomy')))

from controllers.behaviours import Carrot, goto, land, takeoff  # noqa: E402
from controllers.flight_sim import OccupancyMap, Setpoint, Sim  # noqa: E402

ALT = 6.0


def trajectory(sim):
    return np.array(sim.log['p']), np.array(sim.log['t'])


def landing_metrics(sim, home_xy=(0.0, 0.0)):
    p = sim.state()[0]
    return {'touchdown_speed_m_s': None if sim.touchdown_speed is None
            else round(sim.touchdown_speed, 3),
            'landing_offset_m': round(float(np.hypot(p[0] - home_xy[0],
                                                     p[1] - home_xy[1])), 3),
            'ground_contact': sim.ground_contact}


def scenario_hover(sim, args):
    ok, carrot = takeoff(sim, ALT, hold=1.0)
    t0 = sim.t
    hold_ok, _ = goto(sim, carrot, [carrot.p.copy()], 1.0, tol=10.0,
                      timeout=0.1)       # keep the carrot where it is
    stay = {'end': sim.t + 20.0}
    ok2 = sim.run(lambda s: carrot.toward(s, carrot.p, 1.0),
                  lambda s: s.t >= stay['end'], 25.0)
    p, t = trajectory(sim)
    sel = t >= t0 + 1.0
    z = p[sel, 2]
    xy = p[sel, :2]
    metrics = {
        'takeoff_reached': bool(ok),
        'hold_altitude_rmse_m': round(float(np.sqrt(((z - ALT) ** 2).mean())), 4),
        'hold_altitude_max_dev_m': round(float(np.abs(z - ALT).max()), 4),
        'hold_horizontal_drift_m': round(float(np.linalg.norm(
            xy - xy[0], axis=1).max()), 4)}
    metrics['landed'] = land(sim, carrot, (0.0, 0.0))
    metrics.update(landing_metrics(sim))
    return metrics


def scenario_waypoints(sim, args):
    ok, carrot = takeoff(sim, ALT, hold=1.0)
    corners = [(8, 0), (8, 8), (-8, 8), (-8, -8), (8, -8), (0, 0)]
    wps = [np.array([x, y, ALT]) for x, y in corners]
    start = carrot.p.copy()
    t0 = sim.t
    reached_ok, reached = goto(sim, carrot, wps, 3.0, tol=0.6)
    p, t = trajectory(sim)
    sel = t >= t0
    pts = p[sel]
    legs = [start] + wps
    cross = []
    for q in pts:
        best = 1e9
        for a, b in zip(legs[:-1], legs[1:]):
            ab = b - a
            tt = float(np.clip(np.dot(q - a, ab) / np.dot(ab, ab), 0, 1))
            best = min(best, float(np.linalg.norm(q - (a + tt * ab))))
        cross.append(best)
    metrics = {
        'takeoff_reached': bool(ok), 'all_waypoints_reached': bool(reached_ok),
        'waypoints_reached': len(reached),
        'flight_time_s': round(sim.t - t0, 1),
        'cross_track_rmse_m': round(float(np.sqrt(np.mean(np.square(cross)))), 3),
        'cross_track_max_m': round(float(np.max(cross)), 3),
        'altitude_max_dev_m': round(float(np.abs(pts[:, 2] - ALT).max()), 3),
        'max_speed_m_s': round(float(np.linalg.norm(
            np.array(sim.log['v'])[sel], axis=1).max()), 2)}
    metrics['landed'] = land(sim, carrot, (0.0, 0.0))
    metrics.update(landing_metrics(sim))
    return metrics


def scenario_nav(sim, args):
    from uav_autonomy.navigation import (NAV_ARRIVED, NAV_UNREACHABLE,
                                         NavParams, Navigator, PlanningGrid)
    east, north = args.goal
    goal = np.array([east, north])
    params = NavParams()
    nav = Navigator(params)
    omap = OccupancyMap()
    ok, carrot = takeoff(sim, ALT, hold=1.0)
    state = {'last_scan': -1.0, 'grid': None, 'grid_t': -9.0, 'min_range': 99.0,
             'states': {}, 'carrot_xy': carrot.p[:2].copy(),
             'plan_t': []}
    cruise, near_speed = 2.5, params.near_obstacle_speed_m_s

    def on_tick(s):
        if s.t - state['last_scan'] >= 0.1:
            state['last_scan'] = s.t
            o, a, r = s.lidar()
            omap.update(o, a, r)
            state['min_range'] = min(state['min_range'], float(r.min()))
        pos = s.state()[0][:2]
        if s.t - state.get('nav_t', -1.0) < 0.1:       # navigator rate: 10 Hz
            return
        state['nav_t'] = s.t
        if state['grid'] is None or s.t - state['grid_t'] >= 0.5:
            state['grid_t'] = s.t
            state['grid'] = PlanningGrid(omap.grid_data(), omap.res,
                                         omap.origin, [pos, goal], params)
        t0 = time.perf_counter()
        sp, limit, st = nav.update(s.t, state['grid'], pos, goal,
                                   clock=time.perf_counter)
        state['plan_t'].append(time.perf_counter() - t0)
        state['states'][st] = state['states'].get(st, 0) + 1
        state['carrot_xy'] = np.array(sp, float)
        state['speed'] = near_speed if limit is not None else cruise

    def provider(s):
        on_tick(s)
        tgt = np.array([state['carrot_xy'][0], state['carrot_xy'][1], ALT])
        return carrot.toward(s, tgt, state['speed'])

    def arrived(s):
        return float(np.hypot(*(s.state()[0][:2] - goal))) < 1.0

    t0 = sim.t
    reached = sim.run(provider, arrived, 400.0)
    p, t = trajectory(sim)
    path = np.array(sim.log['p'])
    length = float(np.linalg.norm(np.diff(path[t >= t0][:, :2], axis=0),
                                  axis=1).sum())
    metrics = {
        'takeoff_reached': bool(ok), 'goal_reached': bool(reached),
        'goal_en': [float(east), float(north)],
        'straight_line_m': round(float(np.hypot(east, north)), 1),
        'flown_path_m': round(length, 1),
        'flight_time_s': round(sim.t - t0, 1),
        'collisions': [(round(c[0], 2), c[1]) for c in sim.collisions],
        'min_lidar_range_m': round(state['min_range'], 2),
        'navigator_states': state['states'], 'plans': nav.plans,
        'replans_after_block': nav.replans_blocked,
        'plan_ms_median': round(1000 * float(np.median(state['plan_t'])), 2),
        'plan_ms_p95': round(1000 * float(np.percentile(state['plan_t'], 95)), 2),
        'map_scans': omap.scans}
    # return home along the same map, then land
    home = np.array([0.0, 0.0])
    nav2 = Navigator(params)
    nav.path_goal = None

    def provider_home(s):
        on_tick_home(s)
        tgt = np.array([state['carrot_xy'][0], state['carrot_xy'][1], ALT])
        return carrot.toward(s, tgt, state['speed'])

    def on_tick_home(s):
        if s.t - state['last_scan'] >= 0.1:
            state['last_scan'] = s.t
            o, a, r = s.lidar()
            omap.update(o, a, r)
            state['min_range'] = min(state['min_range'], float(r.min()))
        pos = s.state()[0][:2]
        if s.t - state.get('nav_t', -1.0) < 0.1:
            return
        state['nav_t'] = s.t
        if s.t - state['grid_t'] >= 0.5:
            state['grid_t'] = s.t
            state['grid'] = PlanningGrid(omap.grid_data(), omap.res,
                                         omap.origin, [pos, home], params)
        sp, limit, st = nav2.update(s.t, state['grid'], pos, home)
        state['carrot_xy'] = np.array(sp, float)
        state['speed'] = near_speed if limit is not None else cruise

    back = sim.run(provider_home, lambda s: float(np.hypot(
        *(s.state()[0][:2] - home))) < 1.2, 400.0)
    metrics['returned_home'] = bool(back)
    metrics['collisions'] = [(round(c[0], 2), c[1]) for c in sim.collisions]
    metrics['min_lidar_range_m'] = round(state['min_range'], 2)
    metrics['landed'] = land(sim, carrot, (0.0, 0.0))
    metrics.update(landing_metrics(sim))
    return metrics


SCENARIOS = {'hover': scenario_hover, 'waypoints': scenario_waypoints,
             'nav': scenario_nav}


def save_outputs(sim, scenario, metrics, out_dir, args):
    os.makedirs(out_dir, exist_ok=True)
    p, t = trajectory(sim)
    ref = np.array(sim.log['ref'])
    with open(os.path.join(out_dir, 'trajectory.csv'), 'w') as handle:
        handle.write('t,x,y,z,ref_x,ref_y,ref_z\n')
        for i in range(len(t)):
            handle.write(f'{t[i]:.3f},{p[i,0]:.3f},{p[i,1]:.3f},{p[i,2]:.3f},'
                         f'{ref[i,0]:.3f},{ref[i,1]:.3f},{ref[i,2]:.3f}\n')
    report = {
        'scenario': scenario, 'scene': sim.scenario, 'sim_time_s': round(sim.t, 1),
        'labels': {
            'vehicle_state': 'simulator ground truth (no estimator, no noise)',
            'controller': 'standalone cascaded controller, NOT PX4',
            'map': 'lidar rays with known pose (mapping, NOT SLAM)'},
        'args': {k: v for k, v in vars(args).items()},
        'metrics': metrics, 'collisions': sim.collisions}
    with open(os.path.join(out_dir, 'report.json'), 'w') as handle:
        json.dump(report, handle, indent=1, default=str)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
        ax[0].plot(p[:, 0], p[:, 1], lw=1.2, label='flown')
        ax[0].plot(ref[:, 0], ref[:, 1], lw=0.8, ls='--', label='reference')
        shapes = json.load(open(os.path.join(
            ROOT, 'worlds', 'shapes', f'{sim.scenario}.json')))['shapes']
        for s in shapes:
            if s['kind'] == 'box' and s['pos'][2] - s['dims'][2] / 2 < 6.0 \
                    < s['pos'][2] + s['dims'][2] / 2:
                x, y = s['pos'][:2]
                w, d = s['dims'][:2]
                yaw = s['rpy'][2]
                c, sn = math.cos(yaw), math.sin(yaw)
                pts = [(x + c * a - sn * b, y + sn * a + c * b)
                       for a, b in ((-w/2, -d/2), (w/2, -d/2), (w/2, d/2), (-w/2, d/2))]
                ax[0].fill(*zip(*pts), color='0.45', lw=0)
        ax[0].set_aspect('equal'); ax[0].set_title(
            f'{scenario}: path over the 6 m obstacle slice')
        ax[0].legend(loc='lower left'); ax[0].set_xlabel('east [m]'); ax[0].set_ylabel('north [m]')
        ax[1].plot(t, p[:, 2], label='z'); ax[1].plot(t, ref[:, 2], ls='--', label='z ref')
        ax[1].set_xlabel('sim time [s]'); ax[1].set_ylabel('altitude [m]')
        ax[1].legend(); ax[1].set_title('altitude')
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'flight.png'), dpi=110)
    except Exception as exc:                      # plotting is optional
        print('plot skipped:', exc)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('scenario', choices=sorted(SCENARIOS))
    parser.add_argument('--scene', default='urban_C')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--realtime', action='store_true')
    parser.add_argument('--out')
    parser.add_argument('--goal', default='33,90')
    args = parser.parse_args()
    args.goal = tuple(float(v) for v in args.goal.split(','))
    sim = Sim(args.scene, gui=args.gui, realtime=args.realtime or args.gui)
    wall = time.time()
    try:
        metrics = SCENARIOS[args.scenario](sim, args)
    finally:
        sim.close()
    out = args.out or os.path.join(ROOT, 'results', f'{args.scenario}_{args.scene}_'
                                   f'{time.strftime("%H%M%S")}')
    report = save_outputs(sim, args.scenario, metrics, out, args)
    print(json.dumps(report['metrics'], indent=1, default=str))
    print(f'sim {sim.t:.1f} s in {time.time() - wall:.1f} s wall; saved {out}')
    clean = not sim.collisions
    return 0 if clean else 1


if __name__ == '__main__':
    sys.exit(main())
