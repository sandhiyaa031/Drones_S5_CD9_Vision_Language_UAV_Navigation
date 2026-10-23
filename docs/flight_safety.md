# In-flight safety monitor

Code: `uav_autonomy/flight_safety.py` (pure Python) used by
`flight_controller.py`. Tests: `test/test_flight_safety.py`.

## What it does

From takeoff until PX4 reports landed, every `VehicleLocalPosition` sample
(about 50 Hz) is checked. The first violation is latched and the controller
calls its existing `_fail()` path: state `FAILSAFE`, PX4 `NAV_LAND` requested
(AUTO LAND). The monitor can only cause a failure. It cannot arm, change a
setpoint, mark a waypoint reached, or report success. A waypoint counts only if
the controller is still outside `FAILSAFE` when the arrival test runs, and the
safety check runs first in every control tick.

The arm, takeoff, hold and landing logic is unchanged (the diff to
`flight_controller.py` is 39 added lines, none removed). No PX4 parameter or
preflight check is touched. `flight_controller.py` is still the only publisher
to `/fmu/in/*`.

## Definitions and thresholds

Checks apply only while more than 0.5 m above the takeoff point, so touchdown
bumps and pre-takeoff estimator alignment are ignored. `dt` is the PX4
timestamp difference between consecutive samples.

| Check | Violation when | Default |
|---|---|---|
| Abnormal acceleration (impact) | `sqrt(ax^2+ay^2+az^2)` exceeds the limit on any sample (PX4 has already removed gravity) | 30 m/s^2 |
| Estimator reset | PX4's `xy`, `z`, `vxy` or `vz` reset counter changes | any change |
| Position discontinuity | position change differs from `mean velocity * dt` by more than `limit + 0.5 * 30 * dt^2` | 0.5 m |
| Velocity discontinuity | velocity change exceeds `limit + 30 * dt` | 3.0 m/s |
| Speed envelope | horizontal or vertical speed exceeds the limit | 6.5 m/s, 3.0 m/s |
| Non-finite estimate | any position, velocity or acceleration is NaN/inf (also on the ground) | n/a |
| Altitude excursion | during waypoint legs and return, altitude differs from the 3 m target by more than the limit | 1.0 m |

Stale telemetry was already covered and is unchanged: position or status older
than 1.0 s, or `xy_valid`/`z_valid` false, fails the flight. Sample gaps longer
than 1.0 s are skipped by the discontinuity checks for that reason.

The heading reset counter is deliberately not watched: PX4 may realign yaw in
flight without anything being wrong.

## Why these numbers (measured on this vehicle and simulator)

Airborne values taken from PX4 logs (125 Hz):

| Flight | max accel | max horiz. speed | max vert. speed | max position residual |
|---|---|---|---|---|
| Hover, 10 flights | 2.4 m/s^2 | 0.05 m/s | 1.7 m/s | < 0.001 m |
| Waypoint, empty world, no contact | 7.4 m/s^2 | 4.3 m/s | 1.7 m/s | 0.004 m |
| Waypoint, old world (clipped something) | 99 m/s^2 | 4.9 m/s | 1.7 m/s | 0.016 m |
| Roof collision, rescue world | 1250 m/s^2 | 7.4 m/s | 4.0 m/s | 0.014 m |
| Roof collision, 2 Oct P1 run | 1197 m/s^2 | 8.8 m/s | 7.5 m/s | 0.040 m |

* **30 m/s^2** is four times the largest acceleration seen in a contact-free
  flight and one third of the weakest recorded impact. It is about 3 g, which
  this multicopter cannot produce by itself in these missions.
* **0.5 m position jump** is more than ten times the largest residual in any
  log. Moving 0.5 m in one 20 ms sample would need 25 m/s.
* **3.0 m/s velocity jump** in one sample corresponds to 150 m/s^2, far above
  the impact limit; it exists to catch an impact whose acceleration spike fell
  between two samples.
* **6.5 m/s / 3.0 m/s** sit between the fastest clean flight (4.9, 1.7) and
  the collisions (7.4 to 8.8, 4.0 to 7.5). They are specific to the present
  missions (legs up to 7 m). A later mission that legitimately flies faster
  must raise them through `SafetyLimits` and re-justify the value.
* **1.0 m altitude excursion**: clean waypoint legs stayed within 0.35 m.

In both recorded collisions the estimator did not reset or jump; the vehicle
was physically thrown. The reset and position-jump checks are therefore
covered by unit tests only, not by a recorded event.

## Verification

* 26 unit tests pass, including replays of recorded logs at 62, 42, 25 and
  12 Hz: the roof collision is caught 8.5 to 9.5 s after arming (the impact
  is at 8.66 s) and five clean hover logs raise nothing.
* Live, `results/safety/hover_with_monitor`: hover flight passes, no trigger.
* Live, `results/safety/waypoint_with_monitor`: impact at 8.6 s, PX4 in
  AUTO LAND at 8.7 s.
* Live, `results/safety/waypoint_with_monitor_long`: caught as a velocity
  jump at 8.5 s, AUTO LAND at 8.5 s.

## Limits (not solved here)

* Detection is after contact. It does not prevent a collision.
* **A crashed or physically obstructed vehicle may remain armed.** After a
  crash the vehicle can come to rest on rubble or a roof slab. In the two live
  tests PX4 never declared "landed" (22 s and 78 s of observation) and stayed
  armed. The response chain is deliberately left as
  `impact / safety failure -> FAILSAFE -> PX4 AUTO LAND`; the controller does
  not send a disarm in `FAILSAFE`, and no automatic force-disarm is added
  (decision of 2026-10-03). Post-crash recovery and disarm is a future
  safety-design item.
* Collisions too gentle to exceed the limits (slow scraping) are not detected.
