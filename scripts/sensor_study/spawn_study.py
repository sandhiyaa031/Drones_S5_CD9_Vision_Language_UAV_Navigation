#!/usr/bin/env python3
"""Sensor-study debris releaser (study tool, not the mission spawner).

Releases a scripted list of physical boxes one at a time once the UAV is
hovering. Each box is created through Gazebo's entity factory and, if an
initial velocity is requested, given it by a one-shot wrench that Gazebo
applies for exactly one physics step:  F = m * dv / step.  The box is never
teleported or moved after release; gravity and contacts do the rest.

The plan and the actual release parameters are written to releases.json.

Scenario options per release (Phase 6 scenarios; this is the scenario
generator, so it may read the UAV's simulator pose to aim a release):
    "relative": true   position is an offset from the UAV at release time:
                       [east, north, height above the UAV]
    "lead_s": s        with "relative": aim at where the UAV will be in s
                       seconds at its present velocity
    "ahead_m": d       with "relative": shift the release d metres along
                       the UAV's present direction of travel
    "arm_below": v     with "wait_speed": first wait for the speed to drop
                       below v (so the trigger fires at the start of a leg)
    "after_s": s       with "wait_speed": wait s more seconds after the
                       trigger and aim from the state at that moment
    "wait_speed": v    release once the UAV's horizontal speed exceeds v
                       (skipped if that does not happen within 40 s)
"""

import argparse
import json
import os
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from uav_autonomy.continuous_debris_spawner import make_sdf


class FixedColor:
    """Stand-in RNG so make_sdf uses one known debris colour."""

    def __init__(self, index):
        self.index = index

    def choice(self, options):
        return options[self.index % len(options)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--plan', required=True, help='JSON list of releases')
    parser.add_argument('--out', required=True)
    parser.add_argument('--world', default=os.environ.get(
        'WORLD', 'collapsed_building_rescue'))
    parser.add_argument('--uav', default=os.environ.get(
        'UAV_NAME', 'x500_mono_cam_down_0'))
    parser.add_argument('--start-altitude', type=float, default=2.85)
    parser.add_argument('--physics-step', type=float, default=0.004)
    parser.add_argument('--dwell', type=float, default=3.2,
                        help='seconds each box stays before removal')
    parser.add_argument('--wait-timeout', type=float, default=60.0)
    parser.add_argument('--settle', type=float, default=1.5,
                        help='seconds to wait after the start altitude')
    parser.add_argument('--max-duration', type=float, default=23.0,
                        help='stop releasing after this many seconds (the '
                             'hover lasts 30 s)')
    args = parser.parse_args()
    plan = json.load(open(args.plan))

    node = Node()
    uav = {'z': None, 't': None, 'history': []}

    def on_pose(msg):
        for p in msg.pose:
            if p.name == args.uav:
                uav['x'], uav['y'] = p.position.x, p.position.y
                uav['z'] = p.position.z
                uav['t'] = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                uav['history'].append(
                    (uav['t'], p.position.x, p.position.y, p.position.z))
                del uav['history'][:-400]

    pose_topic = f'/world/{args.world}/pose/info'

    def uav_state(wait_speed=None, arm_below=None, timeout=40.0):
        """UAV position and velocity from fresh simulator poses."""
        uav['history'].clear()
        node.subscribe(Pose_V, pose_topic, on_pose)
        end = time.monotonic() + timeout
        state = None
        armed = arm_below is None
        while time.monotonic() < end:
            time.sleep(0.01)
            hist = list(uav['history'])
            if len(hist) < 2 or hist[-1][0] - hist[0][0] < 0.12:
                continue
            new = hist[-1]
            old = next(h for h in reversed(hist) if new[0] - h[0] >= 0.10)
            dt = new[0] - old[0]
            vel = [(new[i] - old[i]) / dt for i in (1, 2, 3)]
            state = (new[0], list(new[1:]), vel)
            speed = (vel[0] ** 2 + vel[1] ** 2) ** 0.5
            if not armed:
                # wait for the UAV to be (nearly) stopped first, so that
                # the trigger fires at the start of a leg
                armed = speed < arm_below
                continue
            if wait_speed is None or speed >= wait_speed:
                break
        node.unsubscribe(pose_topic)
        time.sleep(0.05)
        return state

    node.subscribe(Pose_V, f'/world/{args.world}/pose/info', on_pose)
    wrench_pub = node.advertise(f'/world/{args.world}/wrench', EntityWrench)

    deadline = time.monotonic() + args.wait_timeout
    while time.monotonic() < deadline:
        if uav['z'] is not None and uav['z'] >= args.start_altitude:
            break
        time.sleep(0.05)
    else:
        raise SystemExit('UAV never reached the start altitude')
    time.sleep(args.settle)  # let the hover settle
    # Stop listening before issuing service requests: a pose callback that
    # arrives while a request is waiting for its reply contends for the
    # Python interpreter lock and makes requests time out.
    node.unsubscribe(f'/world/{args.world}/pose/info')
    started = time.monotonic()
    print(f"UAV hovering at z={uav['z']:.2f}; releasing {len(plan)} boxes",
          flush=True)

    log = []
    pending = []   # (wall time to remove, name)

    def remove_due(force=False):
        now = time.monotonic()
        for due, name in list(pending):
            if force or now >= due:
                remove = Entity()
                remove.name = name
                remove.type = Entity.MODEL
                node.request(f'/world/{args.world}/remove', remove, Entity,
                             Boolean, 3000)
                pending.remove((due, name))

    for index, item in enumerate(plan):
        if time.monotonic() - started > args.max_duration:
            print('release window over; stopping releases', flush=True)
            break
        name = f'p1_debris_{index:02d}'
        size = item.get('size', [0.4, 0.4, 0.4])
        mass = float(item.get('mass', 1.2))
        velocity = item.get('velocity', [0.0, 0.0, 0.0])
        request = EntityFactory()
        request.sdf = make_sdf(name, size, mass, FixedColor(index))
        request.name = name
        request.allow_renaming = False
        position = [float(v) for v in item['position']]
        uav_at_release = None
        if item.get('relative') or 'wait_speed' in item:
            uav_at_release = uav_state(item.get('wait_speed'),
                                       item.get('arm_below'))
        if item.get('after_s') and uav_at_release is not None:
            # let the cruise settle, then take the state again
            time.sleep(float(item['after_s']))
            uav_at_release = uav_state()
        if 'wait_speed' in item and (uav_at_release is None or (
                uav_at_release[2][0] ** 2 + uav_at_release[2][1] ** 2)
                ** 0.5 < item['wait_speed']):
            print(f'skipped {name}: speed trigger never met', flush=True)
            continue
        if item.get('relative') and uav_at_release is not None:
            _, p_uav, v_uav = uav_at_release
            lead = float(item.get('lead_s', 0.0))
            speed = max((v_uav[0] ** 2 + v_uav[1] ** 2) ** 0.5, 1e-6)
            ahead = float(item.get('ahead_m', 0.0))
            position = [
                p_uav[0] + v_uav[0] * (lead + ahead / speed) + position[0],
                p_uav[1] + v_uav[1] * (lead + ahead / speed) + position[1],
                p_uav[2] + position[2]]
        request.pose.position.x = position[0]
        request.pose.position.y = position[1]
        request.pose.position.z = position[2]
        request.pose.orientation.w = 1.0
        ok, reply = node.request(
            f'/world/{args.world}/create', request, EntityFactory, Boolean,
            3000)
        created = bool(ok and reply.data)
        t_request = (uav_at_release[0] if uav_at_release is not None
                     else uav['t'])
        if created and any(abs(v) > 1e-9 for v in velocity):
            time.sleep(0.03)  # entity must exist before the wrench arrives
            wrench = EntityWrench()
            wrench.entity.name = name
            wrench.entity.type = Entity.MODEL
            wrench.wrench.force.x = mass * velocity[0] / args.physics_step
            wrench.wrench.force.y = mass * velocity[1] / args.physics_step
            wrench.wrench.force.z = mass * velocity[2] / args.physics_step
            wrench_pub.publish(wrench)
        log.append({'name': name, 'created': created,
                    'sim_time_at_request': t_request, 'size': size,
                    'mass': mass, 'position': position,
                    'plan_position': item['position'],
                    'relative': bool(item.get('relative')),
                    'uav_at_release': uav_at_release,
                    'requested_velocity': velocity,
                    'label': item.get('label', '')})
        with open(args.out, 'w') as handle:
            json.dump(log, handle, indent=2)
        print(f"released {name} {item.get('label', '')} ok={created}",
              flush=True)
        pending.append((time.monotonic() + args.dwell, name))
        # "gap" = seconds until the next release (default: the dwell time,
        # i.e. one object at a time). A shorter gap gives several objects in
        # the air together.
        wait_until = time.monotonic() + float(item.get('gap', args.dwell))
        while time.monotonic() < wait_until:
            remove_due()
            time.sleep(0.02)
    while pending:
        remove_due()
        time.sleep(0.05)
    with open(args.out, 'w') as handle:
        json.dump(log, handle, indent=2)
    print('done', flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
