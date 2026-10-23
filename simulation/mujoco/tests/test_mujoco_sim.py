"""Headless tests of the standalone MuJoCo simulation.

Run with the isolated environment:  .venv/bin/python -m pytest tests -q
(no ROS, no Gazebo, no PX4 needed; MUJOCO_GL=egl or glfw for the camera test)
"""

import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.abspath(os.path.join(
    ROOT, '..', '..', 'src', 'uav_autonomy')))

import mujoco  # noqa: E402

from controllers.behaviours import Carrot, goto, land, takeoff  # noqa: E402
from controllers.flight_sim import OccupancyMap, Setpoint, Sim  # noqa: E402

SCENES = ['urban_A', 'urban_B', 'urban_B2', 'urban_C', 'urban_D', 'urban_F']


@pytest.mark.parametrize('scene', SCENES)
def test_scene_compiles_with_vehicle_and_cameras(scene):
    m = mujoco.MjModel.from_xml_path(os.path.join(
        ROOT, 'worlds', f'mj_{scene}.xml'))
    assert m.ngeom > 1000 and m.nu == 4
    assert m.camera('down').id >= 0 and m.site('lidar').id >= 0


def test_no_ros_or_gazebo_modules_are_needed():
    for name in ('rclpy', 'gz', 'px4_msgs'):
        assert name not in sys.modules, f'{name} was imported'


def test_without_motor_thrust_the_vehicle_falls_freely():
    """The vehicle is moved by physics only: with the motors off it falls
    with g (nothing holds it up or moves it)."""
    s = Sim('urban_A')
    r = s.quad.root
    s.d.qpos[r + 2] = 20.0                      # start in the air (test setup)
    mujoco.mj_forward(s.m, s.d)
    t0, z0 = s.t, s.state()[0][2]
    s.run(lambda sim: Setpoint(sim.state()[0], motors_off=True),
          lambda sim: sim.t - t0 >= 1.0, 1.5)
    drop = z0 - s.state()[0][2]
    assert drop == pytest.approx(0.5 * 9.81 * 1.0 ** 2, rel=0.05)


def test_takeoff_hover_and_landing():
    s = Sim('urban_A')
    ok, carrot = takeoff(s, 6.0, hold=1.0)
    assert ok
    n0 = len(s.log['t'])
    s.run(lambda sim: carrot.toward(sim, carrot.p, 1.0),
          lambda sim: len(sim.log['t']) - n0 >= 250, 20.0)    # ~10 s
    z = np.array(s.log['p'])[n0:, 2]
    assert np.sqrt(((z - 6.0) ** 2).mean()) < 0.05
    assert land(s, carrot, (0.0, 0.0))
    assert s.ground_contact and s.touchdown_speed < 0.6
    assert not s.collisions
    assert math.hypot(*s.state()[0][:2]) < 0.5


def test_waypoint_arrival():
    s = Sim('urban_A')
    ok, carrot = takeoff(s, 6.0, hold=0.5)
    wps = [np.array([8.0, 0.0, 6.0]), np.array([8.0, 8.0, 6.0])]
    ok, reached = goto(s, carrot, wps, 3.0, tol=0.6, timeout=30.0)
    assert ok and len(reached) == 2
    assert abs(s.state()[0][2] - 6.0) < 0.2


def test_lidar_sees_a_facade_at_the_flight_slice():
    s = Sim('urban_A')
    s.d.qpos[s.quad.root:s.quad.root + 3] = [-3.0, 36.0, 6.0]  # sensor check
    mujoco.mj_forward(s.m, s.d)
    origin, ang, ranges = s.lidar()
    assert (ranges <= 29.0).sum() > 50
    omap = OccupancyMap()
    omap.update(origin, ang, ranges)
    data = omap.grid_data()
    assert (data == 100).sum() > 30 and (data == 0).sum() > 5000


def test_navigation_around_obstacles_has_no_collision():
    """Base -> the alley west of the street block (about 65 m, buildings
    between): the project's A* on the lidar map, no contact with the scene."""
    from uav_autonomy.navigation import NavParams, Navigator, PlanningGrid
    s = Sim('urban_A')
    ok, carrot = takeoff(s, 6.0, hold=0.5)
    params, nav, omap = NavParams(), Navigator(NavParams()), OccupancyMap()
    goal = np.array([24.0, 60.0])
    st = {'t': -1.0, 'grid': None, 'gt': -9.0, 'c': carrot.p[:2].copy(), 'v': 2.5}

    def provider(sim):
        if sim.t - st['t'] >= 0.1:
            st['t'] = sim.t
            o, a, r = sim.lidar()
            omap.update(o, a, r)
        pos = sim.state()[0][:2]
        if st['grid'] is None or sim.t - st['gt'] >= 0.5:
            st['gt'] = sim.t
            st['grid'] = PlanningGrid(omap.grid_data(), omap.res, omap.origin,
                                      [pos, goal], params)
        c, limit, _ = nav.update(sim.t, st['grid'], pos, goal)
        st['c'] = np.array(c)
        st['v'] = 1.2 if limit is not None else 2.5
        return carrot.toward(sim, np.array([c[0], c[1], 6.0]), st['v'])
    reached = s.run(provider, lambda sim: np.hypot(
        *(sim.state()[0][:2] - goal)) < 1.0, 120.0)
    assert reached
    assert not s.collisions, s.collisions
