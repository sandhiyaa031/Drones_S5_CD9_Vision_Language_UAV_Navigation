#!/usr/bin/env python3
"""OFFLINE scoring of survivor localisation (Phase 8).

Two sources of estimates for a recorded flight:

  live     <run>/locations.jsonl, written by scripts/perception/
           log_locations.py from the node's output during the flight
  replay   (--replay) the same library the node uses, fed with the recorded
           Phase 7 pixel detections (<run>/survivor_msgs.jsonl) and PX4's own
           pose estimate from the flight log (<run>/flight.ulg)

Ground truth (simulator poses, world file) is used here only, to score:
the true survivor positions are converted into the PX4 local frame with the
vehicle's simulator pose at the start of the log.

Usage: evaluate_localization.py [--replay] [--world NAME] [--out FILE] RUN...
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

from uav_autonomy.survivor_localization import (  # noqa: E402
    CameraModel, LocalizerParams, SurvivorLocalizer)

IMAGE_W, IMAGE_H = 1280, 960
FOCAL = (IMAGE_W / 2.0) / math.tan(1.74 / 2.0)     # mono_cam model
BODY_HEIGHT_ON_GROUND = 0.23
MATCH_RADIUS_M = 2.5
SURVIVOR_BIAS_M = 0.3


def load_world(run_dir, world=None):
    """Survivor and object positions (world frame) of the run's world."""
    if world is None:
        world = 'base'
        info = os.path.join(run_dir, 'trial_info.txt')
        if os.path.exists(info):
            for token in open(info).read().split():
                if token.startswith('world=') and token != \
                        'world=collapsed_building_rescue':
                    world = token.split('=', 1)[1]
    spec = json.load(open(os.path.join(
        WS, 'results/phase7/worlds', f'{world}.json')))
    return world, spec


def load_ulog_pose(run_dir):
    """PX4's own estimate: t, position NED, quaternion wxyz, speed."""
    from pyulog import ULog
    log = ULog(os.path.join(run_dir, 'flight.ulg'),
               ['vehicle_local_position', 'vehicle_attitude'])
    data = {d.name: d.data for d in log.data_list}
    lp, att = data['vehicle_local_position'], data['vehicle_attitude']
    t = lp['timestamp_sample'] * 1e-6
    ta = att['timestamp_sample'] * 1e-6
    p = np.column_stack([lp['x'], lp['y'], lp['z']])
    q = np.column_stack([np.interp(t, ta, att[f'q[{i}]']) for i in range(4)])
    q /= np.linalg.norm(q, axis=1)[:, None]
    speed = np.sqrt(lp['vx'] ** 2 + lp['vy'] ** 2 + lp['vz'] ** 2)
    return t, p, q, speed


def load_gazebo_pose(run_dir):
    t, p = [], []
    with open(os.path.join(run_dir, 'poses.csv')) as handle:
        for row in csv.DictReader(handle):
            if row['name'] == 'x500_rescue_0':
                t.append(float(row['t_sim']))
                p.append([float(row[k]) for k in ('x', 'y', 'z')])
    return np.array(t), np.array(p)


def world_to_local(run_dir, t, p):
    """Offset so that local NED = (y_w, x_w) - offset (evaluation only).

    Uses the first second in which both the simulator pose and PX4's
    estimate exist; the vehicle is then on the ground or hovering.
    """
    tg, pg = load_gazebo_pose(run_dir)
    t0 = max(t[0], tg[0])
    mask = (tg >= t0) & (tg <= t0 + 1.0)
    north = np.interp(tg[mask], t, p[:, 0])
    east = np.interp(tg[mask], t, p[:, 1])
    return np.array([np.median(pg[mask, 1] - north),
                     np.median(pg[mask, 0] - east)])


def pose_at(stamp, t, p, q):
    if stamp < t[0] or stamp > t[-1]:
        return None
    i = int(np.clip(np.searchsorted(t, stamp), 1, len(t) - 1))
    a = (stamp - t[i - 1]) / max(t[i] - t[i - 1], 1e-9)
    position = (1 - a) * p[i - 1] + a * p[i]
    q0, q1 = q[i - 1], q[i]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    quat = (1 - a) * q0 + a * q1
    return position, quat / np.linalg.norm(quat)


def replay(run_dir, params):
    """Run the localiser on the recorded detections; return history rows."""
    t, p, q, speed = load_ulog_pose(run_dir)
    landed = speed[:250] < 0.15
    ground = float(np.median(p[:250, 2][landed])) + BODY_HEIGHT_ON_GROUND \
        if landed.any() else BODY_HEIGHT_ON_GROUND
    cam = CameraModel(fx=FOCAL, fy=FOCAL, cx=IMAGE_W / 2.0, cy=IMAGE_H / 2.0,
                      width=IMAGE_W, height=IMAGE_H)
    localizer = SurvivorLocalizer(cam, params, ground)
    rows = []
    with open(os.path.join(run_dir, 'survivor_msgs.jsonl')) as handle:
        for line in handle:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            detections = []
            for d in msg['detections']:
                x, y, w, h = d['bbox']
                border = x <= 1 or y <= 1 or x + w >= IMAGE_W - 1 \
                    or y + h >= IMAGE_H - 1
                detections.append({
                    'track_id': d['id'], 'center_u': d['center'][0],
                    'center_v': d['center'][1], 'bbox_width': w,
                    'bbox_height': h,
                    'touches_border': d.get('touches_border', border),
                    'measured': d.get('measured', True),
                    'semantic_state': d['semantic_state']})
            updated = {e.entity_id for e in localizer.update(
                msg['stamp'], detections, pose_at(msg['stamp'], t, p, q))}
            for entity in localizer.reported():
                rows.append({
                    'stamp': msg['stamp'], 'id': entity.entity_id,
                    'ned': entity.position.tolist(),
                    'sigma_h': localizer.reported_sigma(entity),
                    'state': localizer.state_of(entity),
                    'obs': entity.observations,
                    'baseline': entity.baseline_m,
                    'height': ground - float(entity.position[2]),
                    'confirmed': entity.confirmed,
                    'rejected': entity.vlm_rejected,
                    'updated': entity.entity_id in updated})
    return rows, localizer.skipped, ground


def load_live(run_dir):
    rows, skipped, ground = [], {}, None
    with open(os.path.join(run_dir, 'locations.jsonl')) as handle:
        for line in handle:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            ground = msg.get('ground_down', ground)
            for loc in msg['locations']:
                rows.append(dict(loc, stamp=msg['stamp']))
    status = os.path.join(run_dir, 'localization_status.jsonl')
    if os.path.exists(status):
        last = None
        for line in open(status):
            last = line
        if last:
            try:
                skipped = json.loads(last).get('skipped', {})
            except ValueError:
                pass
    return rows, skipped, ground


def score(run_dir, rows, world_spec):
    """Match each located entity to the nearest true object and score it."""
    t, p, _, _ = load_ulog_pose(run_dir)
    offset = world_to_local(run_dir, t, p)
    truths = []
    for s in world_spec['survivors']:
        truths.append({'name': s['name'], 'kind': 'survivor', 'ned': [
            s['position'][1] - offset[0], s['position'][0] - offset[1]]})
    for o in world_spec['objects']:
        truths.append({'name': o['name'], 'kind': o.get('kind', 'object'),
                       'ned': [o['position'][1] - offset[0],
                               o['position'][0] - offset[1]]})
    by_id = {}
    for row in rows:
        by_id.setdefault(row['id'], []).append(row)
    entities = []
    for entity_id, history in sorted(by_id.items()):
        final = history[-1]
        best, best_d = None, None
        for truth in truths:
            d = math.hypot(final['ned'][0] - truth['ned'][0],
                           final['ned'][1] - truth['ned'][1])
            # objects placed against a survivor (rubble, occluder) must not
            # take the match from the survivor the detection came from
            rank = d if truth['kind'] == 'survivor' else d + SURVIVOR_BIAS_M
            if best_d is None or rank < best_rank:
                best, best_d, best_rank = truth, d, rank
        matched = best is not None and best_d <= MATCH_RADIUS_M
        item = {
            'id': entity_id, 'final_ned': [round(v, 3) for v in final['ned']],
            'sigma_h': round(final['sigma_h'], 3), 'state': final['state'],
            'observations': final['obs'],
            'baseline_m': round(final['baseline'], 2),
            'height_m': round(final.get('height', float('nan')), 2),
            'vlm_confirmed': bool(final['confirmed']),
            'first_stamp': history[0]['stamp'],
            'last_stamp': final['stamp'],
            'match': best['name'] if matched else None,
            'match_kind': best['kind'] if matched else None}
        if matched:
            truth = np.array(best['ned'])
            errors = np.array([math.hypot(r['ned'][0] - truth[0],
                                          r['ned'][1] - truth[1])
                               for r in history])
            sigmas = np.array([r['sigma_h'] for r in history])
            item.update({
                'truth_ned': [round(v, 3) for v in best['ned']],
                'error_first_m': round(float(errors[0]), 3),
                'error_final_m': round(float(errors[-1]), 3),
                'error_max_m': round(float(errors.max()), 3),
                'error_median_m': round(float(np.median(errors)), 3),
                'error_over_sigma_final': round(float(
                    errors[-1] / max(sigmas[-1], 1e-6)), 2),
                'fraction_within_2sigma': round(float(
                    (errors <= 2.0 * sigmas).mean()), 3)})
        entities.append(item)
    found = {e['match'] for e in entities if e['match_kind'] == 'survivor'}
    return {
        'local_origin_offset_world': [round(float(v), 3) for v in offset],
        'true_survivors': [t['name'] for t in truths
                           if t['kind'] == 'survivor'],
        'survivors_located': sorted(found),
        'duplicate_entities': sum(
            1 for e in entities if e['match_kind'] == 'survivor')
        - len(found),
        'entities': entities}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--replay', action='store_true')
    parser.add_argument('--world')
    parser.add_argument('--out')
    parser.add_argument('--set', action='append', default=[],
                        metavar='NAME=VALUE', help='override a parameter')
    args = parser.parse_args()
    params = LocalizerParams()
    for item in args.set:
        name, value = item.split('=', 1)
        setattr(params, name, type(getattr(params, name))(float(value)))
    results = {}
    for run_dir in args.runs:
        run_dir = run_dir.rstrip('/')
        world, spec = load_world(run_dir, args.world)
        if args.replay:
            rows, skipped, ground = replay(run_dir, params)
        else:
            rows, skipped, ground = load_live(run_dir)
        result = score(run_dir, rows, spec)
        result.update({'world': world, 'source': 'replay' if args.replay
                       else 'live', 'skipped': skipped,
                       'ground_down': ground})
        results[os.path.basename(run_dir)] = result
        print(f'== {os.path.basename(run_dir)} [{world}] '
              f'survivors located {len(result["survivors_located"])}/'
              f'{len(result["true_survivors"])} '
              f'duplicates {result["duplicate_entities"]} skipped {skipped}')
        for e in result['entities']:
            if e['match']:
                print(f'   #{e["id"]} {e["match_kind"]:10s} {e["match"]:18s}'
                      f' err first {e["error_first_m"]:.2f} final '
                      f'{e["error_final_m"]:.2f} max {e["error_max_m"]:.2f} m'
                      f' sigma {e["sigma_h"]:.2f} ratio '
                      f'{e["error_over_sigma_final"]:.1f} in2s '
                      f'{e["fraction_within_2sigma"]:.2f} obs '
                      f'{e["observations"]} base {e["baseline_m"]:.1f} m '
                      f'h {e["height_m"]:.2f} state {e["state"]} '
                      f'conf {int(e["vlm_confirmed"])}')
            else:
                print(f'   #{e["id"]} UNMATCHED at {e["final_ned"]} obs '
                      f'{e["observations"]}')
    if args.out:
        with open(args.out, 'w') as handle:
            json.dump(results, handle, indent=1)


if __name__ == '__main__':
    main()
