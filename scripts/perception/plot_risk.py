#!/usr/bin/env python3
"""Representative risk traces drawn from the live risk node's own output.

For one object per panel: the node's predicted minimum separation over time,
coloured by the risk state it published, with the contact and safety radii.
The dashed line is the true separation from Gazebo (evaluation only).
"""

import argparse
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate_risk as er  # noqa: E402
import replay_tracker as rt  # noqa: E402

INK, MUTED, SURFACE, GRID = '#0b0b0b', '#898781', '#fcfcfb', '#e6e5e0'
STATE = {'SAFE': ('#0ca30c', 'o'), 'WATCH': ('#fab219', 's'),
         'WARNING': ('#ec835a', '^'), 'CRITICAL': ('#d03b3b', 'D'),
         'POTENTIAL_THREAT': ('#4a3aa7', 'P')}


def panel(ax, run_dir, name, title, min_altitude=2.8):
    truth, _ = er.truth_outcomes(run_dir, min_altitude, 'whole_life')
    tr = truth[name]
    obj = tr['obj']
    rows = []
    for frame in er.logged(run_dir):
        st = rt.truth_at(obj, frame['stamp'])
        if st is None:
            continue
        for r in frame['risks']:
            if np.linalg.norm(np.array(r['p']) - st[0]) <= 0.6:
                rows.append((frame['stamp'] - tr['t_ca'], r,
                             frame['uav_p']))
    rows = [x for x in rows if x[0] <= 0.15]
    for state, (colour, marker) in STATE.items():
        pts = [(t, r['d_min']) for t, r, _ in rows if r['state'] == state]
        if pts:
            ax.scatter(*zip(*pts), s=34, color=colour, marker=marker,
                       edgecolor=SURFACE, linewidth=0.8, zorder=3,
                       label=state.replace('_', ' ').title())
    # true separation between UAV centre and box centre (evaluation only)
    ts = obj['t'][(obj['t'] >= tr['t_release'])
                  & (obj['t'] <= tr['t_ca'] + 0.15)]
    if rows:
        uav = np.array(rows[len(rows) // 2][2])
        sep = [np.linalg.norm(rt.truth_at(obj, t)[0] - uav) for t in ts]
        ax.plot(ts - tr['t_ca'], sep, color=MUTED, linewidth=1.3,
                linestyle=(0, (4, 3)), label='true separation now')
        r = rows[-1][1]
        ax.axhline(r['safety_r'], color=INK, linewidth=1)
        ax.axhline(r['contact_r'], color=INK, linewidth=1,
                   linestyle=(0, (1, 2)))
        ax.text(ax.get_xlim()[0], r['safety_r'] + 0.05, ' safety radius',
                color=INK, fontsize=8, va='bottom')
        ax.text(ax.get_xlim()[0], r['contact_r'] - 0.05, ' contact radius',
                color=INK, fontsize=8, va='top')
    ax.set_title(f"{title}\ntrue closest surface distance "
                 f"{tr['min_surface_m']:.2f} m", color=INK, fontsize=10,
                 loc='left')
    ax.set_ylim(0, 3.2)
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.set_xlabel('time relative to true closest approach (s)', color=INK)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--results', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    cases = [('live_A_safe', 'p1_debris_00', 'Safe pass'),
             ('live_B_near_miss', 'p1_debris_02', 'Near miss'),
             ('live_C_direct_hit', 'p1_debris_02', 'Direct hit'),
             ('live_C_lateral_hit', 'p1_debris_01',
              'Lateral hit (only partly seen)')]
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.4), facecolor=SURFACE,
                             sharey=True)
    for ax, (run, name, title) in zip(axes, cases):
        panel(ax, os.path.join(args.results, run), name, title)
    axes[0].set_ylabel('predicted minimum separation (m)', color=INK)
    handles, labels = [], []
    for ax in axes:
        for h, lab in zip(*ax.get_legend_handles_labels()):
            if lab not in labels:
                handles.append(h)
                labels.append(lab)
    fig.legend(handles, labels, loc='lower center', ncol=len(labels),
               frameon=False, labelcolor=INK)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(args.out, dpi=140)
    print('written', args.out)


if __name__ == '__main__':
    main()
