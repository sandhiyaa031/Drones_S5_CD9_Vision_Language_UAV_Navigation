#!/usr/bin/env python3
"""Static validation of the upward depth camera against known targets.

Run with the UAV parked in the depth_validation world. Uses Gazebo transport
only. Ground-truth geometry (target heights, sensor height) is used here to
evaluate the sensor; nothing is published to perception or control.

Tests, each from real rendered depth frames:
  1. camera_info intrinsics
  2. open sky: what value encodes "nothing in range"; is the UAV in view?
  3. flat target closer than the near clip and beyond the far clip
  4. flat targets at known distances: axial vs radial depth, accuracy
  5. two offset boxes: optical-frame orientation
"""

import argparse
import json
import math
import os
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
import numpy as np

STATIC_BOX = '''<sdf version="1.9"><model name="{name}"><static>true</static>
<link name="link"><collision name="c"><geometry><box><size>{sx} {sy} {sz}
</size></box></geometry></collision><visual name="v"><geometry><box><size>
{sx} {sy} {sz}</size></box></geometry></visual></link></model></sdf>'''


class Rig:
    def __init__(self, world, uav):
        self.world, self.uav = world, uav
        self.node = Node()
        self.lock = threading.Lock()
        self.frames = []
        self.info = None
        self.pose = {}
        self.pixel_format = None
        self.node.subscribe(Image, '/uav/depth_up/image', self._on_image)
        self.node.subscribe(CameraInfo, '/uav/depth_up/camera_info',
                            self._on_info)
        self.node.subscribe(Pose_V, f'/world/{world}/pose/info',
                            self._on_pose)

    def _on_image(self, msg):
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width).copy()
        with self.lock:
            self.pixel_format = msg.pixel_format_type
            self.step = msg.step
            self.frames.append(
                (msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, depth))
            del self.frames[:-40]

    def _on_info(self, msg):
        self.info = msg

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name in (self.uav, 'depth_up_link', 'base_link'):
                self.pose[p.name] = (p.position.x, p.position.y, p.position.z)

    def grab(self, count=10, settle=1.0):
        """Return `count` fresh frames taken after a settling delay."""
        time.sleep(settle)
        with self.lock:
            self.frames.clear()
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            with self.lock:
                if len(self.frames) >= count:
                    return [f[1] for f in self.frames[:count]]
            time.sleep(0.05)
        raise SystemExit('depth frames stopped arriving')

    def spawn(self, name, size, center):
        req = EntityFactory()
        req.sdf = STATIC_BOX.format(name=name, sx=size[0], sy=size[1],
                                    sz=size[2])
        req.name = name
        req.pose.position.x, req.pose.position.y, req.pose.position.z = center
        req.pose.orientation.w = 1.0
        ok, rep = self.node.request(f'/world/{self.world}/create', req,
                                    EntityFactory, Boolean, 5000)
        return bool(ok and rep.data)

    def remove(self, name):
        ent = Entity()
        ent.name, ent.type = name, Entity.MODEL
        self.node.request(f'/world/{self.world}/remove', ent, Entity,
                          Boolean, 5000)
        time.sleep(0.5)


def describe(depth):
    finite = np.isfinite(depth)
    return {
        'finite_fraction': round(float(finite.mean()), 5),
        'pos_inf_fraction': round(float(np.isposinf(depth).mean()), 5),
        'neg_inf_fraction': round(float(np.isneginf(depth).mean()), 5),
        'nan_fraction': round(float(np.isnan(depth).mean()), 5),
        'finite_min': float(depth[finite].min()) if finite.any() else None,
        'finite_max': float(depth[finite].max()) if finite.any() else None}


def patch(depth, row, col, half=10):
    return depth[max(0, row - half):row + half + 1,
                 max(0, col - half):col + half + 1]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--world', default='depth_validation')
    parser.add_argument('--uav', default='x500_rescue_0')
    parser.add_argument('--distances', default='2,5,10,15')
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rig = Rig(args.world, args.uav)
    deadline = time.monotonic() + 30
    while (rig.info is None or len(rig.pose) < 2 or not rig.frames):
        if time.monotonic() > deadline:
            raise SystemExit('no depth frames / camera_info / poses')
        time.sleep(0.1)
    report = {}
    print('streams up', flush=True)

    # 1. intrinsics -------------------------------------------------------
    k = list(rig.info.intrinsics.k)
    p = list(rig.info.projection.p)
    width, height = rig.info.width, rig.info.height
    fx_expected = (width / 2) / math.tan(1.7453293 / 2)
    report['camera_info'] = {
        'width': width, 'height': height, 'k': k, 'p': p,
        'distortion_k': list(rig.info.distortion.k),
        'frame_id': [d.value[0] for d in rig.info.header.data
                     if d.key == 'frame_id'],
        'fx_expected_from_hfov': round(fx_expected, 3),
        'image_pixel_format_type': int(rig.pixel_format),
        'image_step_bytes': int(rig.step)}
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]

    # sensor height from ground truth (model frame link pose + model pose)
    model_z = rig.pose[args.uav][2]
    sensor_z = model_z + rig.pose['depth_up_link'][2]
    report['truth'] = {'model_z': model_z,
                       'depth_up_link_in_model': rig.pose['depth_up_link'],
                       'base_link_in_model': rig.pose.get('base_link'),
                       'sensor_world_z': sensor_z}

    # 2. open sky ---------------------------------------------------------
    sky = rig.grab(10)
    report['open_sky'] = describe(np.stack(sky))
    np.save(os.path.join(args.out_dir, 'sky.npy'), sky[0])

    print('sky done', report['open_sky'], flush=True)
    # 3. out-of-range targets --------------------------------------------
    plate = (80.0, 80.0, 0.2)
    for label, dist in (('closer_than_near_clip_0.1m', 0.1),
                        ('beyond_far_clip_25m', 25.0)):
        rig.spawn('target', plate, (0, 0, sensor_z + dist + 0.1))
        report[label] = describe(np.stack(rig.grab(5)))
        print(label, report[label], flush=True)
        rig.remove('target')

    # 4. flat targets at known axial distances ---------------------------
    rows = np.arange(height)[:, None] - cy
    cols = np.arange(width)[None, :] - cx
    ray_scale = np.sqrt(1 + (cols / fx) ** 2 + (rows / fy) ** 2)
    corners = {'top_left': (10, 10), 'top_right': (10, width - 11),
               'bottom_left': (height - 11, 10),
               'bottom_right': (height - 11, width - 11)}
    report['flat_targets'] = []
    for dist in (float(d) for d in args.distances.split(',')):
        ok = rig.spawn('target', plate, (0, 0, sensor_z + dist + 0.1))
        frames = np.stack(rig.grab(10))
        rig.remove('target')
        np.save(os.path.join(args.out_dir, f'target_{dist:g}m.npy'),
                frames[0])
        valid = np.isfinite(frames)
        axial_err = frames - dist
        radial_err = frames - dist * ray_scale[None]
        a, r = axial_err[valid], radial_err[valid]
        entry = {
            'true_axial_distance_m': dist, 'spawned': ok,
            'invalid_pixel_rate': round(float(1 - valid.mean()), 6),
            'axial_model': {
                'median_abs_error_m': float(np.median(np.abs(a))),
                'mean_error_m': float(a.mean()),
                'mean_abs_error_m': float(np.abs(a).mean()),
                'max_abs_error_m': float(np.abs(a).max())},
            'radial_model': {
                'median_abs_error_m': float(np.median(np.abs(r))),
                'max_abs_error_m': float(np.abs(r).max())},
            'center_value_m': float(np.median(
                patch(frames[0], int(cy), int(cx)))),
            'center_error_m': float(np.median(
                patch(frames[0], int(cy), int(cx))) - dist),
            'corner_values_m': {}, 'corner_errors_axial_m': {},
            'frame_to_frame_std_m': float(np.nanmax(np.std(
                np.where(valid, frames, np.nan), axis=0)))}
        for name, (row, col) in corners.items():
            value = float(np.median(patch(frames[0], row, col)))
            entry['corner_values_m'][name] = value
            entry['corner_errors_axial_m'][name] = value - dist
        entry['corner_ray_length_m'] = float(dist * ray_scale[10, 10])
        report['flat_targets'].append(entry)

    # 5. orientation: small box toward world +x, large box toward world +y
    z = sensor_z + 5.0 + 0.25
    rig.spawn('box_plus_x_small', (0.5, 0.5, 0.5), (1.5, 0.0, z))
    rig.spawn('box_plus_y_large', (1.0, 1.0, 0.5), (0.0, 1.5, z))
    frame = rig.grab(3)[0]
    np.save(os.path.join(args.out_dir, 'orientation.npy'), frame)
    rig.remove('box_plus_x_small')
    rig.remove('box_plus_y_large')
    finite = np.isfinite(frame)
    vs, us = np.nonzero(finite)
    left = us < cx - 20
    blobs = {}
    for label, mask in (('blob_left_of_center', left),
                        ('blob_not_left', ~left)):
        if mask.any():
            blobs[label] = {'u': float(us[mask].mean()),
                            'v': float(vs[mask].mean()),
                            'pixels': int(mask.sum()),
                            'depth_m': float(np.median(
                                frame[vs[mask], us[mask]]))}
    # prediction with: sensor X = world +Z, sensor Y = world +Y,
    # sensor Z = world -X (UAV yaw is zero at spawn)
    predicted = {
        'box_plus_x_small': {'u': cx - fx * 0.0 / 5.0,
                             'v': cy - fy * (-1.5) / 5.0, 'pixels_approx':
                             (fx * 0.5 / 5.0) ** 2},
        'box_plus_y_large': {'u': cx - fx * 1.5 / 5.0,
                             'v': cy - fy * 0.0 / 5.0, 'pixels_approx':
                             (fx * 1.0 / 5.0) ** 2}}
    report['orientation'] = {'measured_blobs': blobs, 'predicted': predicted}

    with open(os.path.join(args.out_dir, 'static_depth_report.json'),
              'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1), flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
