# Phase 5: debris collision-risk estimation (advisory)

Code: `uav_autonomy/debris_risk.py` (pure Python core),
`uav_autonomy/debris_risk_node.py` (ROS node), messages in `uav_interfaces`.
Tests: `test/test_debris_risk.py` (32). Results: `results/phase5/`.

The node estimates risk. It issues no command of any kind.

## Interfaces

| | Topic | Type |
|---|---|---|
| in | `/perception/debris/tracks` | `uav_interfaces/DebrisTrackArray` |
| in | `/perception/debris/predictions` | `uav_interfaces/DebrisPredictionArray` |
| in | `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` |
| out | `/perception/debris/risk` | `uav_interfaces/DebrisRiskArray` |
| out | `/perception/debris/risk_markers` | `visualization_msgs/MarkerArray` |
| out | `/perception/debris/risk_debug` | `std_msgs/String` JSON (logging) |

One output per prediction frame, stamped with the track state time. The UAV
position and velocity are interpolated to that time from odometry
(`timestamp_sample`). If no UAV state covers the time (gap over 50 ms, more
than 30 ms extrapolation), every track is reported `UNKNOWN`.

`DebrisRisk`: `track_id`, `stamp`, `risk_state`, `risk_score`,
`time_to_closest_approach`, `minimum_predicted_separation`, `relative_speed`,
`closest_point`, `uav_closest_point`, `current_separation`,
`separation_at_horizons[]`, `contact_radius`, `safety_radius`,
`uncertainty_margin`, `conservative_separation`,
`safety_volume_intersected`, `predicted_collision`, `prediction_horizon`, and
the closed-form values `linear_*` for comparison.

## Equations

Relative motion, all in PX4 local NED, `h` = time after the track state:

    p_uav(h) = p_uav + v_uav h                 (UAV keeps its present velocity)
    r(h)     = p_debris(h) - p_uav(h)
    v_rel(h) = v_debris(h) - v_uav

Closest approach (authoritative): between consecutive Phase 4 states the
debris path is `p_k + v_k tau + 0.5 a_k tau^2` with
`a_k = (v_{k+1} - v_k) / (h_{k+1} - h_k)`. `|r|` is evaluated every 10 ms from
the present state to the last predicted state (2.0 s) and the minimum taken.
Nothing beyond the last state is evaluated.

Closed form (reported for comparison only), constant relative velocity:

    t* = clamp(-(r . v_rel) / |v_rel|^2, 0, horizon),   d = |r + v_rel t*|

It is wrong for falling debris: its minimum-separation error has a 95th
percentile of 3.8 m and a maximum of 24.7 m, against 0.46 m for the sampled
value (offline, 5452 samples).

Uncertainty: `sigma_r = sqrt(u^T P_pos u)`, `u` the unit separation vector at
closest approach, `P_pos` the predicted position covariance there.

    uncertainty_margin      = min(sigma_scale * sigma_r, 0.10 + 0.40 * t_ca)
    conservative_separation = d_min - uncertainty_margin

## Geometry and the definition of collision

All separations are from the **UAV centre** to the **tracked debris point**
(centroid of the visible surface, Phase 3).

| Quantity | Value | Why |
|---|---|---|
| UAV radius | 0.40 m | X500: motor at 0.246 m from the centre plus 0.14 m propeller radius = 0.386 m, rounded up |
| Debris radius | 0.71 x track size | half-diagonal of a square face of side `size`: the furthest the box extends sideways from the tracked point. 0.28 m for a 0.4 m box |
| Contact radius | UAV radius + debris radius | 0.68 m for a 0.4 m box |
| Margin | 0.30 m | covers the measured errors: prediction error to the true surface (95th percentile 0.17 m at 1 s), plus the local-frame detection error (95th percentile up to 0.16 m) |
| Safety radius | contact radius + margin | 0.98 m for a 0.4 m box |
| Reaction time | 0.75 s | time for the UAV to move one safety radius at the 6 m/s^2 it has shown (0.58 s) plus 5 frames of detection and one control step |

`predicted_collision` = `d_min < contact radius`.
`safety_volume_intersected` = `d_min < safety radius`.

Visible-surface bias: the tracked point is on the box face nearest the
camera (the underside), about half a box height below the centre. The
debris radius is taken sideways from that point; the vertical offset does not
change the horizontal miss distance of a near-vertical fall, and the margin
covers the rest. No correction to the box centre is made.

`safety_axes_scale` (default 1, 1, 1) divides the relative vector per axis
before its norm, turning the sphere into an axis-aligned ellipsoid.

## Risk states and thresholds

| State | Condition (confirmed tracks) |
|---|---|
| CRITICAL | `d_min < contact radius`, or `d_min < safety radius` with closest approach within the reaction time (0.75 s) |
| WARNING | `d_min < safety radius`, closest approach later than the reaction time |
| WATCH | nominal path stays outside, but `conservative_separation < safety radius` |
| SAFE | otherwise |
| UNKNOWN | no UAV state at that time |

`risk_score = proximity x urgency`, 0 to 1:
`proximity = clamp((safety + 1.0 - conservative_separation) / (safety + 1.0 - contact), 0, 1)`,
`urgency = 1 - 0.5 * clamp(t_ca / horizon, 0, 1)`. The physical quantities are
published next to it; the score is a convenience.

### Uncertainty is capped, and why

The predictor's covariance is not calibrated. Measured in Phase 4 data
(error to the true centre divided by the reported 3D sigma, 99th percentile):
2.4 at 0.1 s, 1.3 at 0.25 s, 0.55 at 0.5 s, 0.44 at 1 s, 0.14 at 2 s. The
reported sigma is 2.7 m at 1 s and 10.7 m at 2 s while the 95th-percentile
error to the true surface is 0.17 m and 0.86 m. Used as is, every object
within several metres would be WATCH. The margin is therefore the smaller of
the covariance term and an envelope fitted to those measured errors,
`0.10 + 0.40 * t_ca` metres (0.5 m at 1 s, 0.9 m at 2 s).

## Tentative tracks

A TENTATIVE track has no Phase 4 prediction and is never given WATCH,
WARNING or CRITICAL. If it has at least 2 measurements, one in the current
frame, its state is extrapolated in free fall over the same horizons:

* speed at least 2 m/s **and** path enters the safety volume ->
  `POTENTIAL_THREAT`;
* otherwise `SAFE`.

`POTENTIAL_THREAT` does not raise `highest_state`; it sets
`potential_threat_present`. The track itself stays tentative.

## Results

Truth for scoring (offline only): the smallest distance between the UAV
centre and the true box surface. COLLISION <= 0.40 m, NEAR_MISS <= 0.70 m,
SAFE otherwise. This is geometric; of the boxes classed COLLISION only some
actually struck the airframe. "Predicted positive" = the highest state
before the true closest approach was WARNING or CRITICAL.

Two scopes: **free fall** (truth up to the box's first impact with anything)
and **whole life** (including after it bounced off structure or another box).

### Offline: 16 recorded hover flights, 281 objects

Free fall:

| Truth \\ highest state | none | SAFE | WATCH | POTENTIAL | WARNING | CRITICAL |
|---|---|---|---|---|---|---|
| SAFE (216) | 3 | 66 | 104 | 4 | 4 | 35 |
| NEAR_MISS (55) | 3 | 0 | 0 | 0 | 0 | 52 |
| COLLISION (10) | 0 | 0 | 0 | 0 | 0 | 10 |

Precision 0.61, recall 0.954, false-positive rate 0.181, false-negative rate
0.046. All 10 collisions reached CRITICAL with `predicted_collision` set.

Whole life:

| Truth \\ highest state | none | SAFE | WATCH | POTENTIAL | WARNING | CRITICAL |
|---|---|---|---|---|---|---|
| SAFE (195) | 3 | 58 | 97 | 4 | 4 | 29 |
| NEAR_MISS (68) | 3 | 5 | 5 | 0 | 0 | 55 |
| COLLISION (18) | 0 | 3 | 2 | 0 | 0 | 13 |

Precision 0.67, recall 0.79, false-positive rate 0.169, false-negative rate
0.209.

Warning lead time (first WARNING or CRITICAL to true closest approach), free
fall: median 0.80 s, 5th percentile 0.45 s, minimum 0.43 s, maximum 2.04 s
(62 objects). Collisions only: median 1.11 s, minimum 0.78 s.

Share of true intrusions already alerting, by time before closest approach:
3 of 5 tracked at 1.5 s, 22 of 27 at 1.0 s, 39 of 41 at 0.75 s, 52 of 55 at
0.5 s, 60 of 60 at 0.25 s. Most objects are not yet tracked 1 s ahead: they
have not been released or are not yet confirmed.

Minimum-separation error (predictions made at least 0.1 s ahead, 5452):
against the true centre distance median -0.04 m, 5th to 95th percentile
-0.11 to +0.46 m; against the true surface distance median +0.22 m (the
tracked point is not the nearest point of the box). Time of closest approach:
median -0.016 s, 5th to 95th percentile -0.30 to +0.014 s.

Alerts not attributable to any released object: 0. Alerts after the true
closest approach: 11 frames. Downgrades before closest approach: 216 over 281
objects, almost all `WATCH -> SAFE` as the uncertainty margin shrinks with
time to closest approach.

### False negatives

* Free fall, 3: the same lateral throw in three runs. It approaches below the
  camera's field of view and is never detected.
* Whole life, 18: 15 are boxes whose closest approach came **after** they
  struck structure (the roof-opening edge) or another box and were deflected
  toward the UAV. Before the impact their predicted separation was correct
  (1.0 to 1.8 m, SAFE or WATCH). A free-fall model cannot predict a bounce,
  and the bounce happens at or below the camera. The other 3 are the lateral
  throw above.

### False positives

39 in free fall. 37 of them passed at a true surface distance between 0.70
and 0.90 m: just outside the near-miss threshold, inside the conservative
safety radius measured from the tracked point. The remaining 2 (0.91 m and
1.15 m) were adjacent pairs whose merged detection pulled a track sideways.

### Live Gazebo scenarios (risk node running, scored afterwards)

| Scenario | Objects | Result |
|---|---|---|
| A safe pass | 6 at 1.5 to 1.8 m | all SAFE throughout; predicted `d_min` 1.5 to 1.65 m against true centre distance 1.60 to 1.81 m |
| B near miss | 6 at 0.83 to 0.88 m (true surface 0.54 to 0.68 m) | all WARNING then CRITICAL; lead time 0.74 to 1.00 s |
| C direct hit | 1 dropped onto the UAV (true surface 0.05 m) | CRITICAL, `predicted_collision`, lead time 0.91 s; the box struck the UAV and the safety monitor landed it |
| C lateral hit | 1 thrown sideways at the UAV (true surface 0.17 m) | POTENTIAL_THREAT 0.65 s ahead, never WARNING: it was only ever seen cut by the image border, so its track stayed tentative. It struck the UAV |
| D multiple (seed 55, 26 boxes) | 4 near misses in free fall | 4 of 4 CRITICAL; 5 of 22 safe boxes flagged (surface 0.71 to 0.89 m); 1 near miss after a bounce missed |
| E UAV climbing (10 boxes) | 2 intrusions | both CRITICAL, lead 0.63 and 0.99 s; 8 safe boxes SAFE or WATCH |

Live totals over A to D (43 objects, whole life): precision 0.81, recall
0.87; collisions 3, of which 2 CRITICAL and 1 POTENTIAL_THREAT.

Processing per frame: median 0.32 to 0.40 ms, maximum 8.1 ms. Output rate 22
to 26 Hz (one per depth frame delivered). UAV state available for 100 % of
frames.

## Limits

* **Deflections are not predicted.** A box that bounces off structure or
  another box toward the UAV is missed (15 of 86 true intrusions offline).
* **Objects only seen at the image edge reach POTENTIAL_THREAT at best**, and
  objects approaching below the field of view are not seen at all.
* **Conservative by construction**: about 18 % of safe objects are flagged,
  nearly all passing 0.7 to 0.9 m from the airframe.
* **Lead time is short**: median 0.8 s, 5 % under 0.45 s, against a 0.75 s
  reaction estimate. Objects released less than about 5 m above the UAV give
  less warning than an avoidance manoeuvre needs.
* The UAV is assumed to keep its present velocity for 2 s; a planned
  manoeuvre is not known to this node.
* The safety volume is a sphere around the UAV centre; the airframe is flat,
  so vertical clearance is over-estimated as a threat.
* Uncertainty is an empirical envelope from simulation data, not a calibrated
  probability. There is no probability of collision.
* Thresholds were derived from simulation with noise-free depth.


## Phase 6.1: UAV motion model (optional)

Parameter `uav_motion_model` of the risk node:

| Value | UAV path over the horizon |
|---|---|
| `constant_velocity` (default, unchanged) | p + v t |
| `constant_acceleration` | p + v t + ½ a t², a = PX4 `vehicle_local_position` acceleration (mean of 0.06 s), ignored below `uav_accel_deadband_m_s2` 0.8, clipped at `uav_accel_max_m_s2` 8, held for `uav_accel_hold_s` 0.6 s and constant velocity afterwards |
| `response` | the calibrated vehicle response (docs/debris_avoidance.md) to the setpoint the flight controller commands, including its speed cap |

Classification rules, radii and margins are unchanged. Measured on the
4.4 m/s accelerating transit (replay of six flights and five live flights):
neither option gives an earlier warning than constant velocity, because the
limiting error there is in the debris measurement, not in the UAV motion
(`results/phase6_1/README.md`, section A). The default therefore stays
`constant_velocity`.
