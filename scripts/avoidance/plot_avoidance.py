#!/usr/bin/env python3
"""Plot avoidance manoeuvres from the planner's own logged output.

For each manoeuvre in a run: the candidates the planner evaluated at the
moment it decided (their setpoints from avoidance.jsonl, their response
paths recomputed with the planner's model from the logged UAV state), the
selected one, the tracked debris positions and risk states (risk.jsonl), the
UAV's safety volume, the mission setpoint, and the trajectory actually
flown (odometry in avoidance.jsonl). A second panel shows the true
separation from Gazebo ground truth (evaluation only).
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate_avoidance as ev  # noqa: E402

from uav_autonomy.debris_avoidance import (  # noqa: E402
    AvoidanceParams, simulate_response)
from uav_autonomy.debris_risk import RiskParams, radii  # noqa: E402

COLOURS = {'SAFE': '#2a9d3a', 'WATCH': '#e0b020', 'WARNING': '#f08020',
           'CRITICAL': '#d02828', 'POTENTIAL_THREAT': '#9050c0',
           'UNKNOWN': '#808080'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run')
    parser.add_argument('--out-dir')
    parser.add_argument('--max', type=int, default=4)
    args = parser.parse_args()
    run = args.run.rstrip('/')
    out_dir = args.out_dir or os.path.join(os.path.dirname(run), '..',
                                           'plots')
    os.makedirs(out_dir, exist_ok=True)
    records = [r for r in ev.jsonl(os.path.join(run, 'avoidance.jsonl'))
               if 't_uav' in r]
    risk = ev.jsonl(os.path.join(run, 'risk.jsonl'))
    res = ev.evaluate(run)
    boxes, _, (t_u, uav_ned) = ev.truth(run)
    import replay_tracker as rt
    debris = rt.load_truth(run, 'x500_rescue_0', 1.0)[0]
    params = AvoidanceParams()
    _, safety = radii(0.4, RiskParams())
    mans = res['manoeuvre_list'][:args.max]
    if not mans:
        print('no manoeuvres in', run)
        return
    fig, axes = plt.subplots(len(mans), 3, figsize=(17, 5.2 * len(mans)),
                             squeeze=False)
    for row, m in enumerate(mans):
        t0 = m['t_start']
        t1 = (m['t_normal'] or m['t_resume'] or t0 + 3.0)
        start = next(r for r in records if r['t_uav'] >= t0
                     and r['candidates'])
        flown = np.array([r['uav_p'] for r in records
                          if t0 - 0.5 <= r['t_uav'] <= t1 + 0.3])
        cands = start['candidates']
        _, paths, _ = simulate_response(
            start['uav_p'], start['uav_v'], [c['target'] for c in cands],
            params)
        tracked = [(row_['stamp'], r) for row_ in risk
                   if t0 - 1.0 <= row_['stamp'] <= t1 for r in row_['risks']]
        for col, (ix, iy, xl, yl) in enumerate((
                (1, 0, 'east [m]', 'north [m]'),
                (1, 2, 'east [m]', 'altitude [m]'))):
            ax = axes[row][col]
            sy = -1.0 if iy == 2 else 1.0
            for c, path in zip(cands, paths):
                chosen = c['name'] == start['candidate']
                ax.plot(path[:, ix], sy * path[:, iy],
                        color='#1f5fd0' if chosen else
                        '#2a9d3a' if c['ok'] else '#d02828',
                        lw=2.6 if chosen else 0.9,
                        alpha=1.0 if chosen else 0.55, zorder=3 if chosen
                        else 2)
            ax.plot(flown[:, ix], sy * flown[:, iy], 'k-', lw=1.8,
                    label='flown (odometry)', zorder=4)
            ax.add_patch(plt.Circle(
                (start['uav_p'][ix], sy * start['uav_p'][iy]), safety,
                color='#4090d0', alpha=0.15, label='safety volume'))
            ax.plot(start['mission'][ix], sy * start['mission'][iy], 'w*',
                    mec='k', ms=15, label='mission setpoint', zorder=5)
            for _, r in tracked:
                ax.plot(r['p'][ix], sy * r['p'][iy], 'o', ms=4,
                        color=COLOURS.get(r['state'], '#808080'), zorder=1)
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.grid(alpha=0.3)
            if col == 0:
                ax.set_aspect('equal')
                ax.set_xlim(start['uav_p'][1] - 5.5, start['uav_p'][1] + 5.5)
                ax.set_ylim(start['uav_p'][0] - 5.5, start['uav_p'][0] + 5.5)
                ax.set_title(
                    f"t={t0:.2f} s  {m['first_mode']}  selected "
                    f"{start['candidate']}  "
                    f"({m['feasible_at_start']}/{m['candidates_at_start']}"
                    ' feasible)', fontsize=10)
            else:
                ax.set_title('side view: blue selected, green feasible, '
                             'red rejected; dots = tracked debris by risk '
                             'state', fontsize=9)
        ax = axes[row][2]
        for b in boxes:
            obj = debris[b['name']]
            sel = (obj['t'] >= t0 - 1.5) & (obj['t'] <= t1 + 0.5)
            if not sel.any():
                continue
            u = np.column_stack([np.interp(obj['t'][sel], t_u, uav_ned[:, i])
                                 for i in range(3)])
            ax.plot(obj['t'][sel] - t0,
                    np.linalg.norm(obj['p'][sel] - u, axis=1),
                    label=f"box {b['index']} true centre distance")
        ax.axhline(safety, color='#f08020', ls='--', label='safety radius')
        ax.axhline(safety - 0.30, color='#d02828', ls='--',
                   label='contact radius')
        ax.axvline(0.0, color='#1f5fd0', lw=1, label='manoeuvre start')
        if m['t_resume']:
            ax.axvline(m['t_resume'] - t0, color='k', lw=1, ls=':',
                       label='resume mission')
        ax.set_ylim(0, 8)
        ax.set_xlabel('time since manoeuvre start [s]')
        ax.set_ylabel('true separation [m] (ground truth)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='upper right')
        axes[row][0].legend(fontsize=7, loc='upper right')
    fig.suptitle(f"{os.path.basename(os.path.dirname(run))}/"
                 f"{os.path.basename(run)}: collisions "
                 f"{res['collisions_whole_life']}/{res['releases']}, min "
                 f"surface separation {res['min_surface_separation_m']} m",
                 fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, f"{os.path.basename(os.path.dirname(run))}"
                                 f"_{os.path.basename(run)}.png")
    fig.savefig(path, dpi=80)
    print(path)


if __name__ == '__main__':
    main()
