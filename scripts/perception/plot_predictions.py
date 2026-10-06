#!/usr/bin/env python3
"""Plots for Phase 4 from logged predictions and evaluation results.

1. error_vs_horizon.png  - position RMSE against horizon for the three models
2. prediction_view.png   - the predictor's actual output for one track:
   current position, ID, speed, velocity direction, predicted path to 2 s
   with one marker per horizon, and where the tracker later saw the object.
"""

import argparse
import json
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

INK, MUTED, SURFACE, GRID = '#0b0b0b', '#898781', '#fcfcfb', '#e6e5e0'
SERIES = {'ballistic': '#2a78d6', 'ca': '#eb6834', 'cv': '#1baf7a'}
LABELS = {'ballistic': 'ballistic (gravity)', 'cv': 'constant velocity',
          'ca': 'constant acceleration (estimated)'}


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def error_vs_horizon(evaluation, out):
    data = json.load(open(evaluation))
    fig, ax = plt.subplots(figsize=(7.6, 4.4), facecolor=SURFACE)
    style(ax)
    for model in ('cv', 'ca', 'ballistic'):
        rows = data[model]['horizons']
        h = [r['horizon_s'] for r in rows]
        e = [r['position_rmse_surface_m'] for r in rows]
        ax.plot(h, e, color=SERIES[model], linewidth=2, marker='o',
                markersize=6, markeredgecolor=SURFACE, markeredgewidth=1.5,
                label=LABELS[model])
        ax.annotate(f'{e[-1]:.2f} m', (h[-1], e[-1]), xytext=(6, 0),
                    textcoords='offset points', va='center', color=INK,
                    fontsize=9)
    ax.set_yscale('log')
    ax.set_xlim(0, 2.3)
    ax.set_xlabel('prediction horizon (s)', color=INK)
    ax.set_ylabel('position RMSE to true surface (m), log scale', color=INK)
    ax.set_title('Prediction error grows with horizon; only the gravity '
                 'model stays below 0.5 m', color=INK, fontsize=11,
                 loc='left')
    ax.legend(frameon=False, labelcolor=INK, loc='upper left')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def prediction_view(run_dir, out, track_id=None):
    frames = [json.loads(line) for line in open(
        os.path.join(run_dir, 'predictions.jsonl'))]
    per_track = defaultdict(list)
    for f in frames:
        for p in f['predictions']:
            per_track[p['id']].append((f['stamp'], p))
    if track_id is None:      # the track predicted for the longest time
        track_id = max(per_track, key=lambda k: len(per_track[k]))
    series = per_track[track_id]
    t0 = series[0][0]
    fig, ax = plt.subplots(figsize=(7.6, 4.6), facecolor=SURFACE)
    style(ax)
    seen_t = [t - t0 for t, _ in series]
    seen_h = [-p['state_p'][2] for _, p in series]
    ax.plot(seen_t, seen_h, color=MUTED, linewidth=1.5,
            label='tracked position (later frames)')
    picks = [series[0], series[len(series) // 3]]
    for t, p in picks:
        hs = [pt['h'] for pt in p['points']]
        zs = [-pt['p'][2] for pt in p['points']]
        ax.plot([t - t0] + [t - t0 + h for h in hs],
                [-p['state_p'][2]] + zs, color=SERIES['ballistic'],
                linewidth=2, linestyle=(0, (4, 3)))
        sizes = [90 - 32 * h for h in hs]
        ax.scatter([t - t0 + h for h in hs], zs, s=sizes,
                   color=SERIES['ballistic'], edgecolor=SURFACE,
                   linewidth=1.5, zorder=3,
                   label='predicted (marker shrinks with horizon)'
                   if t == picks[0][0] else None)
        ax.scatter([t - t0], [-p['state_p'][2]], s=110, color='#eb6834',
                   edgecolor=SURFACE, linewidth=1.5, zorder=4,
                   label='current track state' if t == picks[0][0] else None)
        speed = float(np.linalg.norm(p['state_v']))
        ax.annotate(f"#{p['id']}  {speed:.1f} m/s down",
                    (t - t0, -p['state_p'][2]), xytext=(8, 6),
                    textcoords='offset points', color=INK, fontsize=9)
        ax.annotate('', xy=(t - t0, -p['state_p'][2] - 0.25 * p['state_v'][2]),
                    xytext=(t - t0, -p['state_p'][2]),
                    arrowprops={'arrowstyle': '->', 'color': '#eb6834',
                                'linewidth': 2})
    ax.axhline(0, color=MUTED, linewidth=1, linestyle=(0, (2, 3)))
    ax.text(seen_t[-1], 0.4, 'takeoff height', color=MUTED, fontsize=9,
            ha='right')
    ax.set_xlabel('time since the track was first predicted (s)', color=INK)
    ax.set_ylabel('height above takeoff point (m)', color=INK)
    ax.set_title(f'Predictor output for track #{track_id}: 2 s ahead at two '
                 'moments', color=INK, fontsize=11, loc='left')
    ax.legend(frameon=False, labelcolor=INK, loc='upper right')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return track_id


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--evaluation', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--out-dir', required=True)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    error_vs_horizon(args.evaluation,
                     os.path.join(args.out_dir, 'error_vs_horizon.png'))
    tid = prediction_view(args.run_dir,
                          os.path.join(args.out_dir, 'prediction_view.png'))
    print('plots written; track', tid)


if __name__ == '__main__':
    main()
