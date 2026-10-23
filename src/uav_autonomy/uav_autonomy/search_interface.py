"""Rules the flight controller applies to a search goal (no ROS).

The search planner proposes a horizontal goal; flight_controller is the only
PX4 command authority and uses the goal as its mission setpoint only if every
rule here holds. Altitude is never taken from the planner.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class SearchLimits:
    """Limits enforced by the controller (docs/search_planner.md)."""

    goal_timeout_s: float = 1.0        # older goals are ignored
    max_range_m: float = 25.0          # goal distance from the takeoff point
    default_speed_m_s: float = 2.0     # cap used when the goal names none
    max_speed_m_s: float = 3.0         # a goal can never raise the cap above
    min_speed_m_s: float = 0.5
    lost_planner_s: float = 5.0        # no valid goal for this long: go home
    max_search_s: float = 300.0        # time budget of the SEARCH state


def validate_goal(target_ne: Sequence[float], home_ne: Sequence[float],
                  age_s: float, limits: SearchLimits = SearchLimits()
                  ) -> Tuple[bool, Optional[str]]:
    """Return (accepted, reason-if-refused) for one search goal."""
    if age_s > limits.goal_timeout_s or age_s < 0:
        return False, f'goal is stale ({age_s:.2f} s)'
    if len(target_ne) != 2 or not all(
            math.isfinite(float(v)) for v in target_ne):
        return False, 'goal is not finite'
    distance = math.hypot(float(target_ne[0]) - float(home_ne[0]),
                          float(target_ne[1]) - float(home_ne[1]))
    if distance > limits.max_range_m:
        return False, (f'goal is {distance:.1f} m from the takeoff point '
                       f'(limit {limits.max_range_m:.1f} m)')
    return True, None


def goal_speed(requested: float,
               limits: SearchLimits = SearchLimits()) -> float:
    """Speed cap for a goal: never uncapped, never above the maximum."""
    if not math.isfinite(requested) or requested <= 0.0:
        return limits.default_speed_m_s
    return min(max(float(requested), limits.min_speed_m_s),
               limits.max_speed_m_s)
