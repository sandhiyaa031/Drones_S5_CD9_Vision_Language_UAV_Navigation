#!/usr/bin/env python3
"""Combine sensor-study runs into tables and plots (evaluation only)."""

import argparse
import csv
import json
import os
import re
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BLUE, INK, MUTED, SURFACE = '#2a78d6', '#0b0b0b', '#898781', '#fcfcfb'


def hovering_objects(run_dir):
    """Objects released and fallen while the UAV held 2.8 to 3.4 m."""
    uav, first, last = [], {}, {}
    with open(os.path.join(run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            t = float(row['t_sim'])
            if row['name'].startswith('x500'):
                uav.append((t, float(row['z'])))
            elif row['name'].startswith('p1_debris'):
                first.setdefault(row['name'], t)
                last[row['name']] = t
    uav = np.array(uav)
    keep = set()
    for name, t0 in first.items():
        z = uav[(uav[:, 0] >= t0) & (uav[:, 0] <= t0 + 1.6), 1]
        if len(z) and z.min() > 2.8 and z.max() < 3.4:
            keep.add(name)
    return keep


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)
    ax.grid(True, color='#e6e5e0', linewidth=0.8)
    ax.set_axisbelow(True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows, frames = [], []
    for run in args.runs:
        keep = hovering_objects(run)
        tag = os.path.basename(run.rstrip('/'))
        for s in json.load(open(os.path.join(run, 'analysis',
                                             'summary.json'))):
            if s['object'] in keep:
                rows.append(dict(s, run=tag))
        with open(os.path.join(run, 'analysis',
                               'real_camera_frames.csv')) as handle:
            for r in csv.DictReader(handle):
                if r['object'] in keep:
                    frames.append(dict(r, run=tag))
    real = [r for r in rows if r['camera'].startswith('REAL')]

    # Table 1: real camera, per object
    with open(os.path.join(args.out, 'table_real_camera.csv'), 'w',
              newline='') as handle:
        cols = ['run', 'object', 'label', 'closest_approach_m',
                't_closest_after_release_s', 'first_view_after_release_s',
                'frames_in_view_before_closest_approach',
                'frames_in_view_during_fall', 'visible_fall_duration_s',
                'warning_time_s', 'pixel_step_median', 'pixel_step_max',
                'size_px_min', 'size_px_max', 'image_diff_response_mean']
        writer = csv.DictWriter(handle, fieldnames=cols,
                                extrasaction='ignore')
        writer.writeheader()
        writer.writerows(real)

    # Table 2: per camera configuration (tilted mounts pooled over azimuth)
    groups = defaultdict(list)
    for r in rows:
        groups[re.sub(r' az \d+$', ' (any yaw)', r['camera'])].append(r)
    table = []
    for name, items in groups.items():
        warn = np.array([i['warning_time_s'] for i in items])
        before = np.array(
            [i['frames_in_view_before_closest_approach'] for i in items])
        first_range = [i['range_at_first_view_m'] for i in items
                       if i['range_at_first_view_m'] is not None
                       and i['frames_in_view_before_closest_approach'] > 0]
        table.append({
            'camera': name, 'object_views': len(items),
            'seen_before_closest_approach_pct':
                round(100 * float(np.mean(before >= 1)), 0),
            'at_least_5_frames_before_pct':
                round(100 * float(np.mean(before >= 5)), 0),
            'median_frames_before': float(np.median(before)),
            'median_warning_s': round(float(np.median(warn)), 2),
            'p25_warning_s': round(float(np.percentile(warn, 25)), 2),
            'median_range_at_first_view_m':
                round(float(np.median(first_range)), 1) if first_range
                else None})
    with open(os.path.join(args.out, 'table_camera_options.csv'), 'w',
              newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(table[0].keys()))
        writer.writeheader()
        writer.writerows(table)
    json.dump({'objects_analysed': len(real), 'cameras': table},
              open(os.path.join(args.out, 'summary.json'), 'w'), indent=1)

    # Plot 1: when does the real camera see the debris?
    t_ca = {(r['run'], r['object']): r for r in real}
    xs, ys = [], []
    for f in frames:
        key = (f['run'], f['object'])
        if f['in_view'] == '1' and f['phase'] == 'fall':
            xs.append(float(f['t_sim']) - t_ca[key]['t_release_sim']
                      - t_ca[key]['t_closest_after_release_s'])
            ys.append(float(f['debris_z']) - float(f['uav_z']) - 0.34)
    fig, ax = plt.subplots(figsize=(7.5, 4.2), facecolor=SURFACE)
    style(ax)
    ax.scatter(xs, ys, s=28, color=BLUE, edgecolor=SURFACE, linewidth=1)
    ax.axvline(0, color=INK, linewidth=1)
    ax.axhline(0, color=MUTED, linewidth=1, linestyle=(0, (4, 3)))
    ax.set_xlim(-1.2, 0.6)
    ax.text(-0.02, ax.get_ylim()[1] * 0.9 if ax.get_ylim()[1] > 0 else 0.3,
            'closest approach', ha='right', color=INK, fontsize=9)
    ax.text(-1.18, 0.08, 'camera height', color=MUTED, fontsize=9)
    ax.set_xlabel('time relative to closest approach to the UAV (s)',
                  color=INK)
    ax.set_ylabel('debris height relative to camera (m)', color=INK)
    ax.set_title('Downward camera: every frame containing falling debris\n'
                 f'({len(real)} objects, UAV hovering at 3 m)', color=INK,
                 fontsize=11, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'real_camera_visibility.png'), dpi=150)
    plt.close(fig)

    # Plot 2: warning time by camera option
    order = sorted(table, key=lambda r: r['median_warning_s'])
    fig, ax = plt.subplots(figsize=(8.5, 0.45 * len(order) + 1.4),
                           facecolor=SURFACE)
    style(ax)
    ax.grid(axis='y', visible=False)
    values = [r['median_warning_s'] for r in order]
    ax.barh([r['camera'] for r in order], values, color=BLUE, height=0.6)
    for i, r in enumerate(order):
        ax.text(values[i] + 0.01, i,
                f"{values[i]:.2f} s  ({r['seen_before_closest_approach_pct']:.0f}% seen)",
                va='center', color=INK, fontsize=8.5)
    ax.set_xlim(0, max(values) * 1.45 + 0.1)
    ax.set_xlabel('median warning time before closest approach (s)',
                  color=INK)
    ax.set_title('Warning time by camera mounting', color=INK,
                 fontsize=11, loc='left')
    fig.text(0.01, 0.005, 'REAL = measured frames. virtual = geometry '
             'applied to the recorded debris trajectories, not rendered.',
             color=MUTED, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'warning_time_by_camera.png'), dpi=150)
    plt.close(fig)

    # Plot 3: per-frame pixel displacement, real camera
    steps = []
    by_obj = defaultdict(list)
    for f in frames:
        if f['in_view'] == '1' and f['phase'] == 'fall':
            by_obj[(f['run'], f['object'])].append(
                (float(f['t_sim']), float(f['u']), float(f['v'])))
    for pts in by_obj.values():
        for a, b in zip(pts, pts[1:]):
            if b[0] - a[0] < 0.06:
                steps.append(float(np.hypot(b[1] - a[1], b[2] - a[2])))
    fig, ax = plt.subplots(figsize=(7.5, 3.8), facecolor=SURFACE)
    style(ax)
    ax.hist(steps, bins=np.arange(0, 260, 20), color=BLUE, edgecolor=SURFACE,
            linewidth=2)
    ax.set_xlabel('debris image displacement between consecutive frames '
                  '(pixels, 1280x960 at 30 Hz)', color=INK)
    ax.set_ylabel('frame pairs', color=INK)
    ax.set_title(f'Downward camera: per-frame debris motion '
                 f'(median {np.median(steps):.0f} px)', color=INK,
                 fontsize=11, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'pixel_step_histogram.png'), dpi=150)
    plt.close(fig)
    print(json.dumps({'objects': len(real), 'pixel_step_median':
                      float(np.median(steps)), 'pixel_step_p90':
                      float(np.percentile(steps, 90))}))
    for r in sorted(table, key=lambda r: -r['median_warning_s']):
        print(r)


if __name__ == '__main__':
    main()
