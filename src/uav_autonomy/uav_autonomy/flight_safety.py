"""In-flight safety checks on PX4 local-position telemetry (no ROS imports).

The monitor looks at successive ``VehicleLocalPosition`` samples and reports
the first violation of any limit below. It never commands anything; the flight
controller decides the response. Thresholds and the measurements behind them
are documented in docs/flight_safety.md.

All quantities are in PX4's local NED frame, SI units.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class SafetyLimits:
    """Limits; defaults are justified in docs/flight_safety.md."""

    # Total acceleration (gravity already removed by PX4) treated as impact.
    impact_accel_m_s2: float = 30.0
    # Position change between two samples not explained by reported velocity.
    position_jump_m: float = 0.5
    # Velocity change between two samples beyond impact_accel * dt.
    velocity_jump_m_s: float = 3.0
    # Flight envelope for this mission profile.
    max_horizontal_speed_m_s: float = 6.5
    max_vertical_speed_m_s: float = 3.0
    # Checks apply only above this height over the takeoff point.
    airborne_altitude_m: float = 0.5
    # Sample gaps longer than this are left to the stale-telemetry check.
    max_sample_gap_s: float = 1.0


@dataclass(frozen=True)
class PositionSample:
    """The fields of VehicleLocalPosition used by the monitor."""

    t: float  # seconds, PX4 timestamp
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float
    xy_reset_counter: int = 0
    z_reset_counter: int = 0
    vxy_reset_counter: int = 0
    vz_reset_counter: int = 0

    @classmethod
    def from_msg(cls, msg):
        """Build a sample from a px4_msgs VehicleLocalPosition message."""
        return cls(
            t=float(msg.timestamp) * 1e-6,
            x=float(msg.x), y=float(msg.y), z=float(msg.z),
            vx=float(msg.vx), vy=float(msg.vy), vz=float(msg.vz),
            ax=float(msg.ax), ay=float(msg.ay), az=float(msg.az),
            xy_reset_counter=int(msg.xy_reset_counter),
            z_reset_counter=int(msg.z_reset_counter),
            vxy_reset_counter=int(msg.vxy_reset_counter),
            vz_reset_counter=int(msg.vz_reset_counter),
        )


_RESET_COUNTERS = (
    'xy_reset_counter', 'z_reset_counter',
    'vxy_reset_counter', 'vz_reset_counter')


class FlightSafetyMonitor:
    """Latch the first safety violation seen in a stream of samples."""

    def __init__(self, limits: Optional[SafetyLimits] = None):
        """Create a monitor with the given (or default) limits."""
        self.limits = limits or SafetyLimits()
        self.reset()

    def reset(self):
        """Forget history and any latched violation."""
        self.violation: Optional[str] = None
        self._previous: Optional[PositionSample] = None
        self._previous_airborne = False

    def update(self, sample: PositionSample,
               altitude_m: float) -> Optional[str]:
        """Check one sample; return the latched violation text or None."""
        if self.violation is not None:
            return self.violation
        airborne = altitude_m > self.limits.airborne_altitude_m
        reason = None
        values = (sample.x, sample.y, sample.z, sample.vx, sample.vy,
                  sample.vz, sample.ax, sample.ay, sample.az)
        if not all(math.isfinite(v) for v in values):
            # A non-finite estimate is never acceptable, airborne or not.
            reason = 'estimator failure: non-finite local position sample'
        elif airborne:
            reason = (self._check_sample(sample)
                      or self._check_continuity(sample))
        self._previous = sample
        self._previous_airborne = airborne
        self.violation = reason
        return reason

    def _check_sample(self, s: PositionSample) -> Optional[str]:
        lim = self.limits
        accel = math.sqrt(s.ax ** 2 + s.ay ** 2 + s.az ** 2)
        if accel > lim.impact_accel_m_s2:
            return (f'impact: acceleration {accel:.1f} m/s^2 exceeds '
                    f'{lim.impact_accel_m_s2:.1f} m/s^2')
        speed_h = math.hypot(s.vx, s.vy)
        if speed_h > lim.max_horizontal_speed_m_s:
            return (f'envelope: horizontal speed {speed_h:.2f} m/s exceeds '
                    f'{lim.max_horizontal_speed_m_s:.2f} m/s')
        if abs(s.vz) > lim.max_vertical_speed_m_s:
            return (f'envelope: vertical speed {abs(s.vz):.2f} m/s exceeds '
                    f'{lim.max_vertical_speed_m_s:.2f} m/s')
        return None

    def _check_continuity(self, s: PositionSample) -> Optional[str]:
        p = self._previous
        if p is None or not self._previous_airborne:
            return None
        for name in _RESET_COUNTERS:
            if getattr(s, name) != getattr(p, name):
                return f'estimator discontinuity: PX4 {name} changed'
        dt = s.t - p.t
        if dt <= 0.0 or dt > self.limits.max_sample_gap_s:
            return None
        lim = self.limits
        # Largest position/velocity change that real motion below the impact
        # threshold could produce over dt.
        slack_p = 0.5 * lim.impact_accel_m_s2 * dt * dt
        slack_v = lim.impact_accel_m_s2 * dt
        expected = [0.5 * (a + b) * dt for a, b in (
            (p.vx, s.vx), (p.vy, s.vy), (p.vz, s.vz))]
        actual = (s.x - p.x, s.y - p.y, s.z - p.z)
        residual = math.sqrt(sum(
            (a - e) ** 2 for a, e in zip(actual, expected)))
        if residual > lim.position_jump_m + slack_p:
            return (f'estimator discontinuity: position moved '
                    f'{residual:.2f} m more than velocity explains in '
                    f'{dt * 1000:.0f} ms')
        dv = math.sqrt((s.vx - p.vx) ** 2 + (s.vy - p.vy) ** 2 +
                       (s.vz - p.vz) ** 2)
        if dv > lim.velocity_jump_m_s + slack_v:
            return (f'impact or estimator discontinuity: velocity changed '
                    f'{dv:.2f} m/s in {dt * 1000:.0f} ms')
        return None


def altitude_excursion(altitude_m: float, target_altitude_m: float,
                       limit_m: float) -> Optional[str]:
    """Return a violation if altitude left the allowed band, else None."""
    if not math.isfinite(altitude_m):
        return 'altitude is not finite'
    error = altitude_m - target_altitude_m
    if abs(error) > limit_m:
        return (f'altitude excursion: {altitude_m:.2f} m is {error:+.2f} m '
                f'from the {target_altitude_m:.2f} m target '
                f'(limit {limit_m:.2f} m)')
    return None
