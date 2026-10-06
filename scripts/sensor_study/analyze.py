#!/usr/bin/env python3
"""Sensor-study analysis: when and where is falling debris observable?

Inputs (one run directory written by record.py + spawn_study.py):
  poses.csv, frames.csv, frames_raw/*.jpg, releases.json

Ground truth is used here, offline, to evaluate observability. For every
debris object it computes, for the real onboard camera and for a set of
hypothetical ("virtual") camera mountings at the same UAV pose:
  * the frames in which the object is inside the field of view
  * time of closest approach to the UAV and of crossing the UAV's altitude
  * warning time = closest approach - first frame in view
  * pixel position, per-frame displacement and apparent size

Real-camera results are cross-checked against the recorded images (frame
differencing at the predicted pixel). Virtual-camera results are geometry
only: no image was rendered for them.
"""

import argparse
import csv
import json
import math
import os
from collections import defaultdict

import cv2
import numpy as np

G = 9.8  # world gravity in the SDF


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Camera:
    """Pinhole camera, Gazebo sensor convention: +X optical axis, +Y left,
    +Z up.  u = cx - fx*Y/X,  v = cy - fy*Z/X."""

    def __init__(self, name, width, height, hfov, rate_hz=30.0,
                 max_range=None, kind='virtual'):
        self.name = name
        self.width, self.height = width, height
        self.fx = self.fy = (width / 2.0) / math.tan(hfov / 2.0)
        self.cx, self.cy = width / 2.0, height / 2.0
        self.rate_hz = rate_hz
        self.max_range = max_range
        self.kind = kind

    def project(self, r_wc, t_wc, p_w):
        p = r_wc.T @ (np.asarray(p_w) - t_wc)
        if p[0] <= 0.1:
            return None
        u = self.cx - self.fx * p[1] / p[0]
        v = self.cy - self.fy * p[2] / p[0]
        rng = float(np.linalg.norm(p))
        inside = (0 <= u < self.width and 0 <= v < self.height and
                  (self.max_range is None or rng <= self.max_range))
        return u, v, rng, inside


def look_direction(azimuth_deg, elevation_deg):
    """World rotation for a camera looking along (azimuth, elevation)."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    x = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
                  math.sin(el)])
    up = np.array([0.0, 0.0, 1.0])
    if abs(x[2]) > 0.999:  # straight up/down: pick a horizontal 'left'
        y = np.array([-math.sin(az), math.cos(az), 0.0])
    else:
        y = np.cross(up, x)
        y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def load_run(run_dir):
    series = defaultdict(list)
    with open(os.path.join(run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            series[row['name']].append(
                [float(row[k]) for k in
                 ('t_sim', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')])
    series = {k: np.array(v) for k, v in series.items()}
    frames = []
    with open(os.path.join(run_dir, 'frames.csv')) as handle:
        for row in csv.DictReader(handle):
            frames.append((float(row['t_sim']), row['file']))
    releases = json.load(open(os.path.join(run_dir, 'releases.json')))
    return series, frames, releases


def interp_pose(track, t):
    """Linear position / nearest orientation at time t (None if outside)."""
    ts = track[:, 0]
    if t < ts[0] or t > ts[-1]:
        return None
    pos = np.array([np.interp(t, ts, track[:, i]) for i in (1, 2, 3)])
    quat = track[int(np.argmin(np.abs(ts - t))), 4:8]
    return pos, quat


def fall_phase(track):
    """(t_release, t_impact): impact = first sharp loss of downward speed."""
    t = track[:, 0]
    vz = np.gradient(track[:, 3], t)
    peak = 0.0
    for i in range(1, len(t)):
        peak = min(peak, vz[i])
        if peak < -2.0 and vz[i] > 0.4 * peak:
            return t[0], t[i]
    return t[0], t[-1]


def analyse_object(name, track, uav, cam_local, frames, camera, release,
                   pose_fn):
    """Observability of one object for one camera mounting."""
    t_rel, t_imp = fall_phase(track)
    ts = track[:, 0]
    # closest approach and altitude crossing during the first fall
    sel = (ts >= t_rel) & (ts <= t_imp)
    dist = []
    for row in track[sel]:
        u = interp_pose(uav, row[0])
        dist.append(np.linalg.norm(row[1:4] - u[0]) if u else np.inf)
    dist = np.array(dist)
    t_ca = float(ts[sel][int(np.argmin(dist))])
    d_ca = float(dist.min())
    uav_z = float(np.interp(t_ca, uav[:, 0], uav[:, 3]))
    above = track[sel][:, 3] > uav_z
    t_cross = None
    if above.any() and not above.all():
        t_cross = float(ts[sel][int(np.argmin(above))])
    rows = []
    for t, fname in frames:
        if t < t_rel - 0.2 or t > t_rel + 3.0:
            continue
        d = interp_pose(track, t)
        u = interp_pose(uav, t)
        if d is None or u is None:
            continue
        r_wc, t_wc = pose_fn(u, cam_local)
        proj = camera.project(r_wc, t_wc, d[0])
        size = max(release['size'])
        rows.append({
            't': t, 'file': fname, 'phase': 'fall' if t <= t_imp else 'after',
            'in_view': bool(proj and proj[3]),
            'u': proj[0] if proj else None, 'v': proj[1] if proj else None,
            'range_m': proj[2] if proj else None,
            'size_px': camera.fx * size / proj[2] if proj else None,
            'z': float(d[0][2]), 'uav_z': float(u[0][2])})
    seen = [r for r in rows if r['in_view']]
    seen_fall = [r for r in seen if r['phase'] == 'fall']
    before_ca = [r for r in seen_fall if r['t'] <= t_ca]
    steps = [math.hypot(b['u'] - a['u'], b['v'] - a['v'])
             for a, b in zip(seen_fall, seen_fall[1:])
             if b['t'] - a['t'] < 0.06]
    summary = {
        'object': name, 'label': release.get('label', ''),
        'camera': camera.name,
        't_release_sim': round(float(t_rel), 4),
        'fall_duration_s': round(t_imp - t_rel, 3),
        'closest_approach_m': round(d_ca, 2),
        't_closest_after_release_s': round(t_ca - t_rel, 3),
        't_cross_uav_altitude_after_release_s':
            round(t_cross - t_rel, 3) if t_cross else None,
        'frames_in_view_during_fall': len(seen_fall),
        'frames_in_view_before_closest_approach': len(before_ca),
        'frames_in_view_total': len(seen),
        'first_view_after_release_s':
            round(seen_fall[0]['t'] - t_rel, 3) if seen_fall else None,
        'warning_time_s':
            round(t_ca - before_ca[0]['t'], 3) if before_ca else 0.0,
        'visible_fall_duration_s':
            round(seen_fall[-1]['t'] - seen_fall[0]['t'], 3)
            if seen_fall else 0.0,
        'pixel_step_median': round(float(np.median(steps)), 1)
            if steps else None,
        'pixel_step_max': round(float(np.max(steps)), 1) if steps else None,
        'size_px_min': round(min(r['size_px'] for r in seen_fall), 1)
            if seen_fall else None,
        'size_px_max': round(max(r['size_px'] for r in seen_fall), 1)
            if seen_fall else None,
        'range_at_first_view_m': round(seen_fall[0]['range_m'], 2)
            if seen_fall else None,
    }
    return summary, rows


def real_pose_fn(uav_pose, cam_local):
    """World pose of the real camera: model pose composed with link pose."""
    pos, quat = uav_pose
    r_wm = quat_to_matrix(*quat)
    r_mc = quat_to_matrix(*cam_local[4:8])
    return r_wm @ r_mc, pos + r_wm @ cam_local[1:4]


def virtual_pose_fn(azimuth, elevation, altitude=None):
    def fn(uav_pose, _cam_local):
        pos = uav_pose[0].copy()
        if altitude is not None:
            pos[2] = altitude
        return look_direction(azimuth, elevation), pos
    return fn


def image_check(run_dir, rows, camera):
    """Mean frame-difference response at the predicted pixel (real cam)."""
    responses = []
    previous_name, previous = None, None
    for r in rows:
        if not r['in_view'] or r['phase'] != 'fall':
            continue
        path = os.path.join(run_dir, 'frames_raw', r['file'])
        index = sorted(os.listdir(os.path.join(run_dir, 'frames_raw')))
        i = index.index(r['file'])
        if i == 0:
            continue
        current = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if previous_name != index[i - 1]:
            previous = cv2.imread(os.path.join(
                run_dir, 'frames_raw', index[i - 1]), cv2.IMREAD_GRAYSCALE)
        diff = cv2.absdiff(current, previous)
        half = int(max(12, r['size_px'] * 0.6))
        u, v = int(r['u']), int(r['v'])
        box = diff[max(0, v - half):v + half, max(0, u - half):u + half]
        responses.append(float(box.mean()) if box.size else 0.0)
        r['diff_response'] = responses[-1]
        r['diff_background'] = float(np.median(diff))
        previous_name, previous = r['file'], current
    return responses


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dir')
    parser.add_argument('--uav', default='x500_mono_cam_down_0')
    args = parser.parse_args()
    series, frames, releases = load_run(args.run_dir)
    uav = series[args.uav]
    cam_local = series['camera_link'][0]
    out_dir = os.path.join(args.run_dir, 'analysis')
    os.makedirs(out_dir, exist_ok=True)

    mono = dict(width=1280, height=960, hfov=1.74)
    depth = dict(width=640, height=480, hfov=1.274, max_range=19.1)
    real = Camera('REAL downward mono 1280x960', kind='real', **mono)
    cameras = [(real, real_pose_fn)]
    for alt in (3.3, 5.3, 8.3, 12.3):
        cameras.append((Camera(f'virtual downward mono at {alt:.1f} m',
                               **mono), virtual_pose_fn(0, -90, alt)))
    cameras.append((Camera('virtual upward mono (zenith)', **mono),
                    virtual_pose_fn(0, 90)))
    cameras.append((Camera('virtual upward depth 640x480 (zenith)', **depth),
                    virtual_pose_fn(0, 90)))
    for el in (30, 45, 60):
        for az in (0, 90, 180, 270):
            cameras.append((Camera(
                f'virtual depth 640x480 tilt {el} deg az {az}', **depth),
                virtual_pose_fn(az, el)))

    summaries, real_rows = [], {}
    for release in releases:
        name = release['name']
        if name not in series or len(series[name]) < 20:
            continue
        for camera, pose_fn in cameras:
            summary, rows = analyse_object(
                name, series[name], uav, cam_local, frames, camera, release,
                pose_fn)
            if camera.kind == 'real':
                responses = image_check(args.run_dir, rows, camera)
                summary['image_diff_response_mean'] = (
                    round(float(np.mean(responses)), 1) if responses
                    else None)
                real_rows[name] = rows
            summaries.append(summary)

    with open(os.path.join(out_dir, 'summary.json'), 'w') as handle:
        json.dump(summaries, handle, indent=1)
    keys = list(summaries[0].keys()) + ['image_diff_response_mean']
    keys = list(dict.fromkeys(keys))
    with open(os.path.join(out_dir, 'summary.csv'), 'w', newline='') as h:
        writer = csv.DictWriter(h, fieldnames=keys)
        writer.writeheader()
        writer.writerows(summaries)
    with open(os.path.join(out_dir, 'real_camera_frames.csv'), 'w',
              newline='') as h:
        writer = csv.writer(h)
        writer.writerow(['object', 't_sim', 'file', 'phase', 'in_view', 'u',
                         'v', 'range_m', 'size_px', 'debris_z', 'uav_z',
                         'diff_response', 'diff_background'])
        for name, rows in real_rows.items():
            for r in rows:
                writer.writerow([name, f"{r['t']:.3f}", r['file'], r['phase'],
                                 int(r['in_view'])] + [
                    '' if r.get(k) is None else round(r[k], 2)
                    for k in ('u', 'v', 'range_m', 'size_px', 'z', 'uav_z',
                              'diff_response', 'diff_background')])

    # annotated frames + video of the real camera
    annotate(args.run_dir, out_dir, real_rows)
    print(f'{len(summaries)} summaries -> {out_dir}')


def annotate(run_dir, out_dir, real_rows):
    by_file = defaultdict(list)
    for name, rows in real_rows.items():
        for r in rows:
            if r['u'] is not None:
                by_file[r['file']].append((name, r))
    writer = cv2.VideoWriter(
        os.path.join(out_dir, 'real_camera_annotated.mp4'),
        cv2.VideoWriter_fourcc(*'mp4v'), 30.0, (640, 480))
    saved = defaultdict(int)
    for fname in sorted(by_file):
        image = cv2.imread(os.path.join(run_dir, 'frames_raw', fname))
        any_view = False
        for name, r in by_file[fname]:
            if not r['in_view']:
                continue
            any_view = True
            half = int(max(10, r['size_px'] * 0.7))
            u, v = int(r['u']), int(r['v'])
            color = (0, 255, 0) if r['phase'] == 'fall' else (0, 200, 255)
            cv2.rectangle(image, (u - half, v - half), (u + half, v + half),
                          color, 2)
            cv2.putText(
                image, f"GT {name[-2:]} z={r['z']:.1f}m {r['phase']}",
                (max(0, u - half), max(20, v - half - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(image, f't_sim={fname[:-4]} s  (boxes = ground truth '
                    'projection, evaluation only)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        writer.write(cv2.resize(image, (640, 480)))
        if any_view:
            for name, r in by_file[fname]:
                if r['in_view'] and r['phase'] == 'fall' and saved[name] < 3:
                    cv2.imwrite(os.path.join(
                        out_dir, f'{name}_fall_{saved[name]}.jpg'),
                        cv2.resize(image, (960, 720)),
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                    saved[name] += 1
    writer.release()


if __name__ == '__main__':
    main()
