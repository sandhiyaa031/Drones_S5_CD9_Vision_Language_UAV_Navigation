"""Rules the flight controller applies to an avoidance command (no ROS).

The avoidance planner proposes a position setpoint; flight_controller is the
only PX4 command authority and accepts the proposal only if every rule here
holds. Anything else is refused and the mission setpoint stays in force.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class AvoidanceLimits:
    """Limits enforced by the controller (docs/debris_avoidance.md)."""

    command_timeout_s: float = 0.5        # older commands are ignored
    max_step_m: float = 6.0               # target distance from the vehicle
    min_altitude_m: float = 1.5           # above the takeoff point
    max_altitude_m: float = 15.0
    recovered_horizontal_m: float = 0.5   # back on the mission setpoint
    recovered_vertical_m: float = 0.35


def validate_command(target: Sequence[float], position: Sequence[float],
                     ground_z: float, age_s: float, state_allows: bool,
                     limits: AvoidanceLimits = AvoidanceLimits()
                     ) -> Tuple[bool, Optional[str]]:
    """Return (accepted, reason-if-refused) for one avoidance command.

    Positions are PX4 local NED; ``ground_z`` is the down coordinate of the
    takeoff point, so altitude above takeoff is ``ground_z - z``.
    """
    if not state_allows:
        return False, 'flight state does not allow avoidance'
    if age_s > limits.command_timeout_s or age_s < 0:
        return False, f'command is stale ({age_s:.2f} s)'
    if len(target) != 3 or not all(math.isfinite(float(v)) for v in target):
        return False, 'target is not finite'
    step = math.dist([float(v) for v in target],
                     [float(v) for v in position])
    if step > limits.max_step_m:
        return False, (f'target is {step:.2f} m from the vehicle '
                       f'(limit {limits.max_step_m:.2f} m)')
    altitude = ground_z - float(target[2])
    if altitude < limits.min_altitude_m:
        return False, (f'target altitude {altitude:.2f} m is below '
                       f'{limits.min_altitude_m:.2f} m')
    if altitude > limits.max_altitude_m:
        return False, (f'target altitude {altitude:.2f} m is above '
                       f'{limits.max_altitude_m:.2f} m')
    return True, None


def recovered(position: Sequence[float], mission_target: Sequence[float],
              limits: AvoidanceLimits = AvoidanceLimits()) -> bool:
    """True once the vehicle is back on the mission setpoint."""
    horizontal = math.hypot(float(position[0]) - float(mission_target[0]),
                            float(position[1]) - float(mission_target[1]))
    vertical = abs(float(position[2]) - float(mission_target[2]))
    return (horizontal <= limits.recovered_horizontal_m
            and vertical <= limits.recovered_vertical_m)


def limit_setpoint(position: Sequence[float], target: Sequence[float],
                   speed_limit: float, px4_xy_p: float = 0.95):
    """Horizontal setpoint that caps the speed PX4 will command.

    PX4's position controller asks for velocity = MPC_XY_P * position error,
    so a setpoint no further than speed_limit / MPC_XY_P from the vehicle
    caps the horizontal speed at speed_limit. Returns (x, y).
    """
    x, y = float(target[0]), float(target[1])
    if speed_limit <= 0.0:
        return x, y
    dx, dy = x - float(position[0]), y - float(position[1])
    dist = math.hypot(dx, dy)
    reach = speed_limit / px4_xy_p
    if dist <= reach:
        return x, y
    return (float(position[0]) + dx * reach / dist,
            float(position[1]) + dy * reach / dist)
