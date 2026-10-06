#!/usr/bin/env python3
"""Verify a PX4 SITL flight from its ULog, independently of ROS node output.

Checks armed, Offboard, takeoff, altitude hold, landing, disarm, absence of
collision impacts and (optional) waypoint arrival using only what PX4 itself logged. Prints a JSON report and
exits 0 only if every applicable check passed.

Altitude is measured relative to the first logged local position (the vehicle
starts on the ground). PX4 local frame is NED: north, east, down.
"""

import argparse
import json
import sys

import numpy as np
from pyulog import ULog

ARMED = 2            # vehicle_status.arming_state ARMING_STATE_ARMED
DISARMED = 1
NAV_OFFBOARD = 14    # vehicle_status.nav_state NAVIGATION_STATE_OFFBOARD


def longest_run_seconds(mask, t):
    """Longest contiguous time span over which mask is True."""
    best = 0.0
    start = None
    for i, ok in enumerate(mask):
        if ok and start is None:
            start = i
        if start is not None and (not ok or i == len(mask) - 1):
            end = i if ok else i - 1
            if end > start and t[end] - t[start] > best:
                best = float(t[end] - t[start])
                best_span = (start, end)
            start = None
    return (best, best_span) if best > 0 else (0.0, None)


def parse_waypoints(text):
    points = []
    for item in text.split(';'):
        item = item.strip()
        if item:
            north, east = (float(v) for v in item.split(','))
            points.append((north, east))
    return points


def analyse(path, args):
    log = ULog(path, message_name_filter_list=[
        'vehicle_local_position', 'vehicle_status', 'vehicle_land_detected'])
    data = {d.name: d.data for d in log.data_list}
    for name in ('vehicle_local_position', 'vehicle_status'):
        if name not in data:
            return {'file': path, 'passed': False,
                    'error': f'{name} missing from log'}
    pos = data['vehicle_local_position']
    status = data['vehicle_status']
    t = pos['timestamp'] / 1e6
    st = status['timestamp'] / 1e6
    north = pos['x'] - pos['x'][0]
    east = pos['y'] - pos['y'][0]
    alt = -(pos['z'] - pos['z'][0])

    checks = {}
    metrics = {}

    armed_mask = status['arming_state'] == ARMED
    checks['armed'] = bool(armed_mask.any())
    if not checks['armed']:
        return {'file': path, 'passed': False, 'checks': checks,
                'metrics': {'max_altitude_m': float(alt.max())}}
    t_arm = float(st[np.argmax(armed_mask)])
    after_arm = st >= t_arm
    disarm_after = after_arm & (status['arming_state'] == DISARMED)
    checks['disarmed_after_flight'] = bool(
        disarm_after.any() and status['arming_state'][-1] == DISARMED)
    t_disarm = (float(st[np.argmax(disarm_after)])
                if disarm_after.any() else float(t[-1]))
    checks['offboard_while_armed'] = bool(
        (status['nav_state'][armed_mask] == NAV_OFFBOARD).any())
    metrics['armed_duration_s'] = round(t_disarm - t_arm, 2)

    in_band = np.abs(alt - args.target_alt) <= args.tolerance
    flight = (t >= t_arm) & (t <= t_disarm)
    reached = in_band & flight
    checks['takeoff_reached_target'] = bool(reached.any())
    metrics['max_altitude_m'] = round(float(alt[flight].max()), 3)
    if reached.any():
        metrics['takeoff_time_s'] = round(
            float(t[np.argmax(reached)]) - t_arm, 2)

    hold_s, span = longest_run_seconds(in_band & flight, t)
    metrics['longest_time_in_altitude_band_s'] = round(hold_s, 2)
    if span is not None:
        seg = slice(span[0], span[1] + 1)
        metrics['hold_altitude_rmse_m'] = round(float(np.sqrt(np.mean(
            (alt[seg] - args.target_alt) ** 2))), 4)
        metrics['hold_altitude_min_m'] = round(float(alt[seg].min()), 3)
        metrics['hold_altitude_max_m'] = round(float(alt[seg].max()), 3)
        if args.mode == 'hover':
            metrics['hold_max_horizontal_drift_m'] = round(float(np.max(
                np.hypot(north[seg] - north[span[0]],
                         east[seg] - east[span[0]]))), 3)
    if args.mode == 'hover':
        checks['altitude_hold'] = bool(hold_s >= args.hold_seconds)

    if args.mode == 'waypoint':
        results = []
        for wn, we in parse_waypoints(args.waypoints):
            dist = np.hypot(north - wn, east - we)
            ok = flight & in_band
            closest = float(dist[ok].min()) if ok.any() else float('inf')
            results.append({'north': wn, 'east': we,
                            'closest_approach_m': round(closest, 3),
                            'reached': bool(closest <= args.radius)})
        metrics['waypoints'] = results
        checks['waypoints_reached'] = bool(
            results and all(r['reached'] for r in results))

    end_alt = float(np.mean(alt[-20:]))
    end_offset = float(np.hypot(np.mean(north[-20:]), np.mean(east[-20:])))
    metrics['final_altitude_m'] = round(end_alt, 3)
    metrics['landing_horizontal_offset_m'] = round(end_offset, 3)
    landed_flag = None
    if 'vehicle_land_detected' in data:
        land = data['vehicle_land_detected']
        landed_flag = bool(land['landed'][-1])
    metrics['px4_landed_flag_at_end'] = landed_flag
    checks['landed'] = bool(
        abs(end_alt) <= args.landing_alt_tolerance
        and landed_flag is not False)

    # A flyaway or simulator fault shows up as an implausible excursion.
    max_range = float(np.max(np.hypot(north[flight], east[flight])))
    metrics['max_horizontal_range_m'] = round(max_range, 3)
    checks['no_excursion'] = bool(
        max_range <= args.max_range
        and metrics['max_altitude_m'] <= args.target_alt + 1.0)

    # A collision appears as a horizontal acceleration far beyond anything
    # the multicopter can produce itself (a few m/s^2 in these missions).
    accel_h = np.hypot(pos['ax'], pos['ay'])
    # Only counted while airborne, so touchdown bumps are not collisions.
    airborne = flight & (alt > args.airborne_alt)
    metrics['max_airborne_horizontal_accel_m_s2'] = round(
        float(accel_h[airborne].max()) if airborne.any() else 0.0, 2)
    hits = airborne & (accel_h > args.impact_accel)
    checks['no_impact'] = not bool(hits.any())
    if hits.any():
        first = int(np.argmax(hits))
        metrics['first_impact'] = {
            'time_after_arm_s': round(float(t[first]) - t_arm, 2),
            'north_m': round(float(north[first]), 2),
            'east_m': round(float(east[first]), 2),
            'altitude_m': round(float(alt[first]), 2)}

    return {'file': path, 'mode': args.mode,
            'passed': bool(all(checks.values())),
            'checks': checks, 'metrics': metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('ulog', nargs='+')
    parser.add_argument('--mode', choices=('hover', 'waypoint'),
                        default='hover')
    parser.add_argument('--target-alt', type=float, default=3.0)
    parser.add_argument('--tolerance', type=float, default=0.35,
                        help='altitude band half-width in metres')
    parser.add_argument('--hold-seconds', type=float, default=30.0)
    parser.add_argument('--waypoints', default='',
                        help='NED offsets from start: "n,e;n,e;..."')
    parser.add_argument('--radius', type=float, default=0.5)
    parser.add_argument('--landing-alt-tolerance', type=float, default=0.3)
    parser.add_argument('--max-range', type=float, default=15.0)
    parser.add_argument('--impact-accel', type=float, default=30.0,
                        help='horizontal acceleration [m/s^2] treated as '
                             'a collision')
    parser.add_argument('--airborne-alt', type=float, default=0.5)
    parser.add_argument('--output', help='also write the JSON report here')
    args = parser.parse_args()
    if args.mode == 'waypoint' and not args.waypoints:
        parser.error('--mode waypoint requires --waypoints')

    reports = [analyse(path, args) for path in args.ulog]
    text = json.dumps(reports if len(reports) > 1 else reports[0], indent=2)
    print(text)
    if args.output:
        with open(args.output, 'w') as handle:
            handle.write(text + '\n')
    sys.exit(0 if all(r['passed'] for r in reports) else 1)


if __name__ == '__main__':
    main()
