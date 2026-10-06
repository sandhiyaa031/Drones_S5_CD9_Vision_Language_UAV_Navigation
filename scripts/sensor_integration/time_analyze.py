#!/usr/bin/env python3
"""Analyse a time_record.py capture: is there one usable common time base?

Method. PX4's sensor_combined gyro samples originate from Gazebo's IMU. Each
PX4 sample is matched to the Gazebo IMU sample with the same gyro values
(Gazebo IMU is FLU, PX4 is FRD: y and z change sign). For a matched pair

    offset_i = PX4 timestamp_i - Gazebo stamp_i

is the true clock offset for that sample, free of transport delay. (PX4
publishes the mean of two consecutive Gazebo IMU samples; the match is made
against that mean and referenced to the later sample.) A constant
offset means a common time base exists; its spread is the timing error.
"""

import argparse
import json

import numpy as np


def pct(values):
    v = np.abs(np.asarray(values))
    return {'p50_ms': round(float(np.percentile(v, 50)) * 1e3, 3),
            'p95_ms': round(float(np.percentile(v, 95)) * 1e3, 3),
            'p99_ms': round(float(np.percentile(v, 99)) * 1e3, 3),
            'max_ms': round(float(v.max()) * 1e3, 3)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('capture')
    parser.add_argument('--output')
    parser.add_argument('--match-tolerance', type=float, default=1e-6)
    args = parser.parse_args()
    d = np.load(args.capture)
    gz, px, odom, depth, clock = (d[k] for k in (
        'gz_imu', 'px4_imu', 'odom', 'depth', 'clock'))
    report = {'samples': {k: int(len(d[k])) for k in d.files}}

    # real-time factor over the capture
    report['real_time_factor'] = round(float(
        (clock[-1, 1] - clock[0, 1]) / (clock[-1, 0] - clock[0, 0])), 4)

    # --- match PX4 gyro samples to Gazebo IMU samples by value -----------
    # Measured: each PX4 sensor_combined gyro value equals the mean of two
    # consecutive Gazebo IMU samples (to float32 precision). The pair is
    # referenced to the stamp of its later sample.
    frd = np.column_stack([gz[:, 1], -gz[:, 2], -gz[:, 3]])
    gz_frd = 0.5 * (frd[1:] + frd[:-1])
    gz = gz[1:]
    order = np.argsort(gz_frd[:, 0])
    sorted_x = gz_frd[order, 0]
    offsets, t_sim, arrival_sim = [], [], []
    ambiguous = 0
    for row in px:
        lo = np.searchsorted(sorted_x, row[1] - args.match_tolerance)
        hi = np.searchsorted(sorted_x, row[1] + args.match_tolerance)
        cand = order[lo:hi]
        if len(cand) == 0:
            continue
        ok = cand[(np.abs(gz_frd[cand, 1] - row[2]) < args.match_tolerance) &
                  (np.abs(gz_frd[cand, 2] - row[3]) < args.match_tolerance)]
        if len(ok) != 1:
            ambiguous += len(ok) > 1
            continue
        j = ok[0]
        offsets.append(row[0] - gz[j, 0])
        t_sim.append(gz[j, 0])
        arrival_sim.append(row[5] - gz[j, 0])
    offsets, t_sim = np.array(offsets), np.array(t_sim)
    report['imu_value_match'] = {
        'px4_samples': int(len(px)), 'matched': int(len(offsets)),
        'matched_fraction': round(len(offsets) / max(1, len(px)), 4),
        'ambiguous': int(ambiguous)}
    if len(offsets) < 100:
        report['conclusion'] = 'too few matched samples'
        print(json.dumps(report, indent=1))
        return

    span = float(t_sim[-1] - t_sim[0])
    slope, intercept = np.polyfit(t_sim, offsets, 1)
    k = float(np.median(offsets))
    report['clock_offset_px4_minus_gazebo'] = {
        'median_s': round(k, 6),
        'first_10s_median_s': round(float(np.median(
            offsets[t_sim < t_sim[0] + 10])), 6),
        'last_10s_median_s': round(float(np.median(
            offsets[t_sim > t_sim[-1] - 10])), 6),
        'drift_s_per_s': float(slope),
        'drift_ms_over_capture': round(float(slope) * span * 1e3, 3),
        'sim_time_span_s': round(span, 1)}
    # timing error if a single constant offset (the median) is assumed
    report['timing_error_with_constant_offset'] = pct(offsets - k)
    # timing error after removing a linear drift (best possible linear map)
    report['timing_error_after_linear_fit'] = pct(
        offsets - (slope * t_sim + intercept))
    report['px4_sample_arrival_delay_in_sim_time'] = pct(arrival_sim)

    # --- odometry: same PX4 clock, so the same offset applies ------------
    if len(odom):
        ts = odom[:, 1]
        report['odometry'] = {
            'rate_hz': round(float((len(ts) - 1) / (ts[-1] - ts[0])), 2),
            'timestamp_minus_timestamp_sample': pct(odom[:, 0] - ts),
            'max_gap_ms': round(float(np.diff(ts).max()) * 1e3, 2),
            'wall_arrival_minus_stamp': pct(odom[:, 2] - ts)}
        # depth frame to pose: for every depth frame, the PX4 time that the
        # constant-offset model assigns, the error of that assignment, and
        # the distance to the nearest odometry sample.
        if len(depth):
            est_px4_time = depth[:, 0] + k
            true_offset = np.interp(depth[:, 0], t_sim, offsets)
            base_err = true_offset - k
            idx = np.clip(np.searchsorted(ts, est_px4_time), 1, len(ts) - 1)
            nearest = np.minimum(np.abs(ts[idx] - est_px4_time),
                                 np.abs(ts[idx - 1] - est_px4_time))
            inside = (depth[:, 0] >= t_sim[0]) & (depth[:, 0] <= t_sim[-1])
            report['depth_frame_to_px4_pose'] = {
                'depth_frames': int(inside.sum()),
                'time_base_error': pct(base_err[inside]),
                'nearest_odometry_sample_distance': pct(nearest[inside]),
                'depth_stamp_behind_clock_at_arrival': pct(
                    depth[inside, 2] - depth[inside, 0])}
    text = json.dumps(report, indent=1)
    if args.output:
        with open(args.output, 'w') as handle:
            handle.write(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
