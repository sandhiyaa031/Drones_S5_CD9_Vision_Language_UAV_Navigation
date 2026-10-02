#!/usr/bin/env python3
"""Spawn a finite, reproducible set of physical falling debris in Gazebo."""

import argparse
import json
import math
import random
import re
import subprocess
import time


def run_command(command, timeout):
    return subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def gazebo_boolean_succeeded(result):
    output = f"{result.stdout}\n{result.stderr}".lower()
    return result.returncode == 0 and re.search(r"\bdata:\s*true\b", output)


def wait_for_world(world, timeout=60.0):
    service = f"/world/{world}/create"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = run_command(["gz", "service", "-l"], timeout=5.0)
            if result.returncode == 0 and service in result.stdout.splitlines():
                return
        except (OSError, subprocess.TimeoutExpired):
            pass
        time.sleep(0.5)
    raise RuntimeError(f"Gazebo create service {service} was not discovered")


def make_sdf(name, size, mass, rng):
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    colors = [
        (0.48, 0.34, 0.20),
        (0.34, 0.35, 0.36),
        (0.58, 0.50, 0.38),
    ]
    color = rng.choice(colors)
    return f'''<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <allow_auto_disable>false</allow_auto_disable>
    <link name="link">
      <inertial>
        <mass>{mass:.6f}</mass>
        <inertia>
          <ixx>{ixx:.8f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.8f}</iyy><iyz>0</iyz><izz>{izz:.8f}</izz>
        </inertia>
      </inertial>
      <gravity>true</gravity>
      <enable_wind>false</enable_wind>
      <velocity_decay><linear>0.015</linear><angular>0.08</angular></velocity_decay>
      <collision name="collision">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.65</mu2></ode></friction><bounce><restitution_coefficient>0.08</restitution_coefficient></bounce></surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <ambient>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1</ambient>
          <diffuse>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1</diffuse>
          <specular>0.12 0.12 0.12 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''


def service_boolean(world, service_name, request_type, request, timeout_ms=2000):
    result = run_command(
        [
            "gz", "service", "-s", f"/world/{world}/{service_name}",
            "--reqtype", request_type,
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(timeout_ms),
            "--req", request,
        ],
        timeout=timeout_ms / 1000.0 + 2.0,
    )
    return gazebo_boolean_succeeded(result), result


def spawn_model(args, index, rng):
    name = f"p1_debris_{index:02d}"
    sx = rng.uniform(args.size_min, args.size_max)
    sy = rng.uniform(args.size_min, args.size_max)
    sz = rng.uniform(args.size_min, args.size_max)
    mass = rng.uniform(args.mass_min, args.mass_max)
    x = rng.uniform(args.x_min, args.x_max)
    y = rng.uniform(args.y_min, args.y_max)
    z = rng.uniform(args.height_min, args.height_max)
    vx = rng.uniform(args.vx_min, args.vx_max)
    vy = rng.uniform(args.vy_min, args.vy_max)
    vz = rng.uniform(args.vz_min, args.vz_max)
    roll, pitch, yaw = (rng.uniform(-0.5, 0.5) for _ in range(3))
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sin_yaw = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sin_yaw
    qy = cr * sp * cy + sr * cp * sin_yaw
    qz = cr * cp * sin_yaw - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sin_yaw
    sdf = make_sdf(name, (sx, sy, sz), mass, rng)

    request = (
        f"sdf: {json.dumps(sdf)} "
        f"name: {json.dumps(name)} "
        f"pose: {{position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}} "
        f"orientation: {{x: {qx:.7f} y: {qy:.7f} z: {qz:.7f} w: {qw:.7f}}}}} "
        "allow_renaming: false"
    )
    success, result = service_boolean(
        args.world, "create", "gz.msgs.EntityFactory", request
    )
    if not success:
        print(f"[SPAWN_FAILED] {name}: {result.stdout.strip()} {result.stderr.strip()}", flush=True)
        return None

    # The world loads ApplyLinkWrench. Apply and then clear a short force pulse;
    # F = m * delta-v / duration gives the requested velocity change physically.
    fx = mass * vx / args.impulse_duration
    fy = mass * vy / args.impulse_duration
    fz = mass * vz / args.impulse_duration
    wrench = (
        f"entity: {{name: {json.dumps(name)} type: MODEL}} "
        f"wrench: {{force: {{x: {fx:.5f} y: {fy:.5f} z: {fz:.5f}}}}}"
    )
    persistent_topic = f"/world/{args.world}/wrench/persistent"
    clear_topic = f"/world/{args.world}/wrench/clear"
    impulse_result = None
    impulse_error = ""
    try:
        for _ in range(3):
            impulse_result = run_command(
                ["gz", "topic", "-t", persistent_topic,
                 "-m", "gz.msgs.EntityWrench", "-p", wrench],
                timeout=3.0,
            )
            if impulse_result.returncode != 0:
                impulse_error = f"{impulse_result.stderr.strip()}"
                break
            time.sleep(0.02)
        if impulse_result is not None and impulse_result.returncode == 0:
            time.sleep(args.impulse_duration)
    except (OSError, subprocess.TimeoutExpired) as exc:
        impulse_result = None
        impulse_error = str(exc)
    finally:
        clear_request = f"name: {json.dumps(name)} type: MODEL"
        for _ in range(3):
            try:
                clear_result = run_command(
                    ["gz", "topic", "-t", clear_topic,
                     "-m", "gz.msgs.Entity", "-p", clear_request],
                    timeout=3.0,
                )
                if clear_result.returncode != 0:
                    impulse_error += f" clear={clear_result.stderr.strip()}"
            except (OSError, subprocess.TimeoutExpired) as exc:
                impulse_error += f" clear={exc}"
            time.sleep(0.02)

    if impulse_result is None or impulse_result.returncode != 0:
        removed, _ = service_boolean(
            args.world, "remove", "gz.msgs.Entity",
            f"name: {json.dumps(name)} type: MODEL",
        )
        print(f"[IMPULSE_FAILED] {name}; cleanup={removed}: {impulse_error}", flush=True)
        return None

    print(
        f"[SPAWNED] {name} pos=({x:.2f},{y:.2f},{z:.2f}) "
        f"size=({sx:.2f},{sy:.2f},{sz:.2f}) mass={mass:.2f}kg "
        f"initial_velocity=({vx:.2f},{vy:.2f},{vz:.2f})m/s "
        f"lifetime={args.lifetime:.1f}s impulse=APPLIED "
        f"duration={args.impulse_duration:.3f}s",
        flush=True,
    )
    return {"name": name, "expires_at": time.monotonic() + args.lifetime}


def remove_model(args, model):
    success, result = service_boolean(
        args.world,
        "remove",
        "gz.msgs.Entity",
        f"name: {json.dumps(model['name'])} type: MODEL",
    )
    if success:
        print(f"[REMOVED] {model['name']} lifetime={args.lifetime:.1f}s", flush=True)
    else:
        print(
            f"[REMOVE_FAILED] {model['name']}: "
            f"{result.stdout.strip()} {result.stderr.strip()}",
            flush=True,
        )
    return success


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a finite seeded falling-debris scenario in Gazebo."
    )
    parser.add_argument("--world", default="collapsed_building_complex")
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--interval-min", type=float, default=1.0)
    parser.add_argument("--interval-max", type=float, default=2.0)
    parser.add_argument("--height-min", type=float, default=7.0)
    parser.add_argument("--height-max", type=float, default=9.0)
    # This is the visible opening just to the side of the stationary X500.
    parser.add_argument("--x-min", type=float, default=1.75)
    parser.add_argument("--x-max", type=float, default=1.95)
    parser.add_argument("--y-min", type=float, default=-0.40)
    parser.add_argument("--y-max", type=float, default=0.40)
    parser.add_argument("--mass-min", type=float, default=0.8)
    parser.add_argument("--mass-max", type=float, default=2.0)
    parser.add_argument("--size-min", type=float, default=0.30)
    parser.add_argument("--size-max", type=float, default=0.48)
    parser.add_argument("--vx-min", type=float, default=-0.45)
    parser.add_argument("--vx-max", type=float, default=0.45)
    parser.add_argument("--vy-min", type=float, default=-0.45)
    parser.add_argument("--vy-max", type=float, default=0.45)
    parser.add_argument("--vz-min", type=float, default=-1.8)
    parser.add_argument("--vz-max", type=float, default=-1.0)
    parser.add_argument("--lifetime", type=float, default=12.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--impulse-duration", type=float, default=0.08)
    args = parser.parse_args()

    if args.count < 1:
        parser.error("--count must be at least 1")
    for low_name, high_name in (
        ("interval_min", "interval_max"), ("height_min", "height_max"),
        ("x_min", "x_max"), ("y_min", "y_max"),
        ("mass_min", "mass_max"), ("size_min", "size_max"),
        ("vx_min", "vx_max"), ("vy_min", "vy_max"),
        ("vz_min", "vz_max"),
    ):
        if getattr(args, low_name) > getattr(args, high_name):
            parser.error(f"--{low_name.replace('_', '-')} must be <= --{high_name.replace('_', '-')}")
    if args.interval_min < 0 or args.lifetime <= 0 or args.impulse_duration <= 0:
        parser.error("intervals must be nonnegative and lifetime/impulse-duration must be positive")
    return args


def main():
    args = parse_args()
    rng = random.Random(args.seed)
    print(
        f"[SCENARIO] world={args.world} count={args.count} "
        f"interval=[{args.interval_min},{args.interval_max}]s "
        f"height=[{args.height_min},{args.height_max}]m "
        f"lifetime={args.lifetime}s seed={args.seed} "
        f"impulse_duration={args.impulse_duration}s",
        flush=True,
    )
    wait_for_world(args.world)
    print(f"[WORLD_READY] /world/{args.world}/create", flush=True)

    active = []
    spawned = 0
    start = time.monotonic()
    try:
        for index in range(args.count):
            if index:
                delay = rng.uniform(args.interval_min, args.interval_max)
                print(f"[NEXT_RELEASE] in {delay:.2f}s", flush=True)
                time.sleep(delay)
            model = spawn_model(args, index, rng)
            if model is not None:
                active.append(model)
                spawned += 1
            now = time.monotonic()
            for expired in [item for item in active if item["expires_at"] <= now]:
                remove_model(args, expired)
                active.remove(expired)

        while active:
            now = time.monotonic()
            for expired in [item for item in active if item["expires_at"] <= now]:
                remove_model(args, expired)
                active.remove(expired)
            if active:
                time.sleep(min(0.1, max(0.01, min(item["expires_at"] for item in active) - time.monotonic())))
    except KeyboardInterrupt:
        print("[INTERRUPTED] cleaning up active debris", flush=True)
        for model in list(active):
            remove_model(args, model)
    finally:
        duration = time.monotonic() - start
        print(
            f"[SCENARIO_COMPLETE] requested={args.count} spawned={spawned} "
            f"active={len(active)} duration={duration:.1f}s seed={args.seed}",
            flush=True,
        )
    if spawned != args.count or active:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
