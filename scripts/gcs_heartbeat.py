#!/usr/bin/env python3
"""Headless MAVLink ground-control-station heartbeat for PX4 SITL.

PX4's X500 airframe sets NAV_DLL_ACT=2, so the commander requires a GCS link
before it will arm (rcAndDataLinkCheck.cpp). This script provides that link by
sending a standard MAV_TYPE_GCS HEARTBEAT at 1 Hz to PX4's GCS MAVLink port.

It sends nothing else: no commands, no setpoints, no parameter writes. It does
not publish to /fmu/in/*. NAV_DLL_ACT is left untouched, so if this process
stops during flight PX4 performs its configured data-link-loss action.
"""

import argparse
import signal
import time

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument(
        '--port', type=int, default=18570,
        help='PX4 GCS MAVLink UDP port (18570 + instance)')
    parser.add_argument('--rate-hz', type=float, default=1.0)
    parser.add_argument('--system-id', type=int, default=255)
    parser.add_argument('--component-id', type=int, default=190)
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error('--rate-hz must be positive')

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    link = mavutil.mavlink_connection(
        f'udpout:{args.host}:{args.port}',
        source_system=args.system_id,
        source_component=args.component_id,
    )
    print(
        f'GCS heartbeat -> udp {args.host}:{args.port} at {args.rate_hz} Hz '
        f'(system={args.system_id} component={args.component_id})',
        flush=True)
    while running:
        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        time.sleep(1.0 / args.rate_hz)
    print('GCS heartbeat stopped', flush=True)


if __name__ == '__main__':
    main()
