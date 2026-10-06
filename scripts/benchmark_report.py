#!/usr/bin/env python3
"""Turn a benchmark_two_camera.sh run into benchmark.json with pass/fail."""

import glob
import json
import os
import re
import sys


def main():
    run = sys.argv[1]
    measures = [json.load(open(p)) for p in sorted(
        glob.glob(os.path.join(run, 'measure_*.json')),
        key=lambda p: int(re.findall(r'measure_(\d+)', p)[0]))]
    renderer = ''
    ogre = os.path.join(run, 'ogre2.log')
    if os.path.exists(ogre):
        hits = re.findall(r'GL_RENDERER = (.*)', open(ogre,
                                                      errors='ignore').read())
        renderer = hits[-1].strip() if hits else ''
    gazebo_log = open(os.path.join(run, 'gazebo.log'),
                      errors='ignore').read()
    timing = {}
    path = os.path.join(run, 'time_analysis.json')
    if os.path.exists(path):
        timing = json.load(open(path))

    def series(key, sub='rate_hz'):
        out = []
        for m in measures:
            v = m.get(key)
            out.append(v.get(sub) if isinstance(v, dict) else v)
        return out

    rss = series('gazebo_server_rss_mib')
    gpu_mem = [m['gpu_last'].get('memory_mib') if m.get('gpu_last') else None
               for m in measures]
    report = {
        'windows': [m['label'] for m in measures],
        'renderer': renderer,
        'real_time_factor': series('real_time_factor'),
        'gazebo_depth_hz': series('gazebo_depth_up'),
        'gazebo_mono_hz': series('gazebo_camera'),
        'ros_depth_hz': series('ros_depth_up_image_raw'),
        'ros_mono_hz': series('ros_camera_image_raw'),
        'gpu_util_percent_mean': series('gpu_util_percent_mean'),
        'gpu_util_percent_max': series('gpu_util_percent_max'),
        'gpu_memory_mib': gpu_mem,
        'gazebo_server_rss_mib': rss,
        'timing': {k: timing.get(k) for k in (
            'real_time_factor', 'imu_value_match',
            'clock_offset_px4_minus_gazebo',
            'timing_error_with_constant_offset', 'depth_frame_to_px4_pose')},
    }
    alive = all(v is not None for v in rss)
    growth = (rss[-1] - rss[0]) if alive and rss else None
    te = (timing.get('timing_error_with_constant_offset') or {})
    checks = {
        'rendered_on_nvidia': 'NVIDIA' in renderer,
        'real_time_factor_ge_0.95': bool(measures) and all(
            v is not None and v >= 0.95 for v in report['real_time_factor']),
        'gazebo_depth_ge_28hz': bool(measures) and all(
            v is not None and v >= 28.0 for v in report['gazebo_depth_hz']),
        'gazebo_mono_ge_28hz': bool(measures) and all(
            v is not None and v >= 28.0 for v in report['gazebo_mono_hz']),
        'gazebo_alive_until_end': alive and 'Segmentation' not in gazebo_log,
        'memory_growth_le_50mib': growth is not None and growth <= 50.0,
        'timestamp_error_p99_le_1ms': te.get('p99_ms') is not None
        and te['p99_ms'] <= 1.0,
    }
    report['gazebo_memory_growth_mib'] = growth
    report['checks'] = checks
    report['passed'] = all(checks.values())
    text = json.dumps(report, indent=1)
    with open(os.path.join(run, 'benchmark.json'), 'w') as handle:
        handle.write(text + '\n')
    print(text)
    sys.exit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
