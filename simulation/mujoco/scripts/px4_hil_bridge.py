#!/usr/bin/env python3
"""MuJoCo <-> PX4 SITL bridge over PX4's MAVLink simulator (HIL) interface.

PX4 (airframe 10016_none_iris, `simulator_mavlink start -c 4560`) is the TCP
CLIENT; this program is the TCP SERVER on port 4560 and plays the simulator:

  PX4 -> here   HIL_ACTUATOR_CONTROLS   the four motor outputs (and "armed")
  here -> PX4   HIL_SENSOR              IMU, magnetometer, barometer
                HIL_GPS                 GNSS fix (10 Hz)

Lockstep: PX4 advances only when it receives a HIL_SENSOR (id 0); every
HIL_ACTUATOR_CONTROLS it answers with is applied to MuJoCo for the next 4 ms
step, then the next HIL_SENSOR is sent. Nothing moves the vehicle except the
four rotor forces and reaction torques MuJoCo applies (no teleporting, no
fabricated sensor "success"); the sensors are computed from MuJoCo's state.

Frames: MuJoCo world ENU, body FLU; PX4 body FRD (x, -y, -z).
Home / spherical reference: the Gazebo world's (47.397971, 8.546163, 0 m).

Usage: px4_hil_bridge.py [--scene urban_A] [--port 4560] [--duration 120]
                         [--out DIR] [--gui]
Start this first, then PX4 (scripts/run_px4_mujoco_trial.sh does both).
"""

import argparse
import math
import os
import socket
import sys
import time

os.environ.setdefault('MAVLINK20', '1')
import numpy as np                                    # noqa: E402
from pymavlink import mavutil                         # noqa: E402

import mujoco                                         # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, ROOT)
from controllers.quadrotor import Quadrotor, quat_to_rot  # noqa: E402

LAT0, LON0, ALT0 = 47.397971057728974, 8.546163739800146, 0.0
# Earth field at the reference position as PX4's world model expects it
# (EKF reference: 0.4824 gauss, inclination 63.16 deg; declination 3.37 deg,
# measured from the EKF heading error with the Gazebo world's field); ENU,
# gauss: east, north, up
MAG_ENU_GAUSS = np.array([0.01274, 0.21620, -0.43100])
T_MAX_HIL = 7.36                                # N, hover at 50 %
STALL_S = 8.0                                   # wall s without a PX4 reply = stall
DT = 0.004                                      # PX4 lockstep step, 250 Hz
SENS_ALL = 7 | 56 | 448 | 6656                  # accel gyro mag baro
MODE_ARMED = 128


class FileWriter:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        self.sock.sendall(data)


def hil_scene(scene):
    """The urban scene with the iris-like HIL vehicle instead of the x500."""
    src = os.path.join(ROOT, 'worlds', f'mj_{scene}.xml')
    text = open(src).read().replace('x500_body.xml', 'iris_hil_body.xml')
    out = os.path.join(ROOT, 'worlds', f'_hil_{scene}.xml')
    open(out, 'w').write(text)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--scene', default='urban_A')
    ap.add_argument('--port', type=int, default=4560)
    ap.add_argument('--duration', type=float, default=120.0,
                    help='simulated seconds to run after PX4 connects')
    ap.add_argument('--out', default=os.path.join(ROOT, 'results', 'px4_hil'))
    ap.add_argument('--gui', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    m = mujoco.MjModel.from_xml_path(hil_scene(args.scene))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    quad = Quadrotor(m, d)
    n_sub = max(1, round(DT / m.opt.timestep))
    dt_phys = DT / n_sub
    acc_adr = m.sensor_adr[m.sensor('imu_acc').id]
    gyro_adr = m.sensor_adr[m.sensor('imu_gyro').id]
    viewer = None
    if args.gui:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(m, d)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = m.body('uav').id
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 12, 215, -25

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', args.port))
    server.listen(1)
    print(f'[bridge] waiting for PX4 on tcp {args.port}', flush=True)
    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f'[bridge] PX4 connected from {addr}', flush=True)
    mav = mavutil.mavlink.MAVLink(FileWriter(conn), srcSystem=1, srcComponent=1)
    mav.robust_parsing = True
    parser = mavutil.mavlink.MAVLink(None)
    parser.robust_parsing = True

    def p_ned_home():
        return d.qpos[quad.root:quad.root + 3].copy()

    def send_sensors(t_us, send_gps):
        p, rot, v, w = quad.state()
        acc_flu = d.sensordata[acc_adr:acc_adr + 3]
        gyr_flu = d.sensordata[gyro_adr:gyro_adr + 3]
        # small Gaussian sensor noise (typical MEMS values). Without it the
        # simulated IMU/magnetometer repeat identical values at rest and PX4's
        # data validator flags them STALE after ~100 samples, which stops
        # vehicle_magnetometer and leaves the EKF without a heading.
        acc = tuple(np.array([acc_flu[0], -acc_flu[1], -acc_flu[2]])
                    + rng.normal(0, 0.02, 3))                  # FRD, m/s2
        gyr = tuple(np.array([gyr_flu[0], -gyr_flu[1], -gyr_flu[2]])
                    + rng.normal(0, 0.0007, 3))                # rad/s
        mag_flu = rot.T @ MAG_ENU_GAUSS
        mag = tuple(np.array([mag_flu[0], -mag_flu[1], -mag_flu[2]])
                    + rng.normal(0, 0.0008, 3))                # gauss
        alt = ALT0 + p[2]
        pressure = 1013.25 * (1.0 - 2.25577e-5 * alt) ** 5.25588 \
            + rng.normal(0, 0.012)                              # hPa
        mav.hil_sensor_send(t_us, *acc, *gyr, *mag, pressure, 0.0, alt,
                            25.0, SENS_ALL, 0)
        if send_gps:
            lat = LAT0 + p[1] / 111319.5
            lon = LON0 + p[0] / (111319.5 * math.cos(math.radians(LAT0)))
            mav.hil_gps_send(
                t_us, 3, int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
                80, 100, int(math.hypot(v[0], v[1]) * 100),
                int(v[1] * 100), int(v[0] * 100), int(-v[2] * 100),
                65535, 12, 0, 0)

    rng = np.random.default_rng(1)
    log = open(os.path.join(args.out, 'poses.csv'), 'w')
    log.write('t_sim,x,y,z,qw,qx,qy,qz,c0,c1,c2,c3,armed\n')
    t_us = 0
    next_gps = 0
    steps = 0
    armed = False
    controls = np.zeros(4)
    started = False
    stalled = False
    last_reply = time.time()
    conn.settimeout(0.004)   # start-up phase: real-time pace
    send_sensors(t_us, True)
    wall0 = time.time()
    t_end = args.duration
    while d.time < t_end:
        # wait for PX4's reply to the last sensor message
        got = False
        while not got:
            try:
                data = conn.recv(65536)
            except socket.timeout:
                if started and time.time() - last_reply > STALL_S:
                    print(f'[bridge] STALL: PX4 sent no actuator controls for '
                          f'{STALL_S:.0f} s at sim {d.time:.1f} s (lockstep '
                          'hang, unresolved intermittent; see '
                          'MULTI_SIM_PROGRESS.md)', flush=True)
                    stalled = True
                    t_end = -1
                    break
                if not started:
                    # PX4 sends nothing until its estimator and control chain
                    # have run: keep the clock moving with sensor messages
                    for _ in range(n_sub):
                        quad.apply(np.zeros(4), dt_phys)
                        mujoco.mj_step(m, d)
                    t_us = int(round(d.time * 1e6))
                    send_sensors(t_us, t_us >= next_gps)
                    if t_us >= next_gps:
                        next_gps = t_us + 100000
                continue
            if not data:
                print('[bridge] PX4 closed the connection', flush=True)
                t_end = -1
                break
            for byte in data:
                msg = parser.parse_char(bytes([byte]))
                if msg is not None and msg.get_type() == 'HIL_ACTUATOR_CONTROLS':
                    armed = bool(msg.mode & MODE_ARMED)
                    controls = np.clip(np.array(msg.controls[:4]), 0.0, 1.0) \
                        if armed else np.zeros(4)
                    got = True
                    started = True
                    last_reply = time.time()
        if t_end < 0:
            break
        for _ in range(n_sub):
            quad.apply(controls * T_MAX_HIL, dt_phys)
            mujoco.mj_step(m, d)
        t_us = int(round(d.time * 1e6))
        gps_due = t_us >= next_gps
        if gps_due:
            next_gps = t_us + 100000
        send_sensors(t_us, gps_due)
        steps += 1
        if steps % 10 == 0:
            p, rot, v, w = quad.state()
            q = d.qpos[quad.root + 3:quad.root + 7]
            log.write(f'{d.time:.3f},{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},'
                      f'{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f},'
                      f'{controls[0]:.3f},{controls[1]:.3f},{controls[2]:.3f},'
                      f'{controls[3]:.3f},{int(armed)}\n')
        if viewer is not None and steps % 8 == 0:
            viewer.sync()
        if steps % 2500 == 0:
            p = quad.state()[0]
            print(f'[bridge] sim {d.time:6.1f} s  wall {time.time() - wall0:6.1f} s  '
                  f'pos {p.round(2)}  armed {armed}', flush=True)
    log.close()
    if stalled:
        sys.exit(2)
    print(f'[bridge] done: {d.time:.1f} sim s in {time.time() - wall0:.1f} wall s',
          flush=True)
    if viewer is not None:
        viewer.close()


if __name__ == '__main__':
    main()
