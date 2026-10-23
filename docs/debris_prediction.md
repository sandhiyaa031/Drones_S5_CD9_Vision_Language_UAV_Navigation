# Phase 4: debris trajectory prediction

Code: `uav_autonomy/debris_prediction.py` (pure Python core),
`uav_autonomy/debris_predictor.py` (ROS node), messages in `uav_interfaces`.
Tests: `test/test_debris_prediction.py` (24). Results: `results/phase4/`.

Prediction only: no separation, time-to-collision, risk or control.

## Interfaces

| | Topic | Type |
|---|---|---|
| in | `/perception/debris/tracks` | `uav_interfaces/DebrisTrackArray` |
| out | `/perception/debris/predictions` | `uav_interfaces/DebrisPredictionArray` |
| out | `/perception/debris/prediction_markers` | `visualization_msgs/MarkerArray` |
| out | `/perception/debris/predictions_debug` | `std_msgs/String` JSON (logging) |

`DebrisPrediction`: `track_id`, `stamp` (time of the track state),
`track_status`, `time_since_update`, `model`, `acceleration`, and `states[]`.
`PredictedState`: `horizon`, `stamp` (= track stamp + horizon), `position`,
`velocity`, `covariance` (6x6), `below_takeoff_plane`.

Frame: PX4 local NED (down is +z). The predicted point is the same point the
tracker follows, the centroid of the visible surface, not the box centre.

## Method

    p(t + h) = p + v h + 0.5 a h^2        v(t + h) = v + a h
    P(h) = F(h) P F(h)^T + Q(h)

for h = 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0 s, starting from the track state
and covariance at the track's own timestamp. `Q` is the tracker's
white-noise-acceleration covariance.

| Model | Acceleration | sigma_a for `Q` |
|---|---|---|
| constant velocity | 0 | 10 m/s^2 |
| constant acceleration | least-squares slope of the track's velocity over the last 0.4 s (at least 5 samples spanning 0.12 s), plus its variance | 3 m/s^2 |
| ballistic | (0, 0, 9.8) m/s^2, known gravity | 3 m/s^2 |

`auto` uses gravity unless a long enough velocity history differs from it by
more than 4 m/s^2, in which case it uses the estimated acceleration.

**Validity.** A prediction is published only for a CONFIRMED track, or a
COASTING one whose last measurement is at most 0.35 s old. Tentative, stale
and dead tracks get none; a track that is absent from the array has no valid
prediction. Per-track history is dropped when the track disappears.

`below_takeoff_plane` marks predicted points lower than the takeoff point.
The predictor has no map; it does not know where rubble, roof or ground are,
so a prediction continues through them.

## Model comparison (offline replay, same recorded detections for all)

Six flights recorded in this phase (high releases from 20 to 33 m, heights
and speeds, lateral throws, seeds 11 and 55). Position RMSE to the true box
surface, scored only while the true object is still in free fall:

| Horizon | n | constant velocity | constant acceleration | ballistic | auto |
|---|---|---|---|---|---|
| 0.1 s | 1958 | 0.027 m | 0.030 m | 0.028 m | 0.030 m |
| 0.25 s | 1702 | 0.057 m | 0.114 m | 0.036 m | 0.110 m |
| 0.5 s | 1283 | 0.841 m | 0.614 m | 0.057 m | 0.525 m |
| 0.75 s | 831 | 2.34 m | 1.42 m | 0.064 m | 1.11 m |
| 1.0 s | 511 | 4.44 m | 2.36 m | 0.110 m | 1.76 m |
| 1.5 s | 213 | 10.5 m | 4.68 m | 0.160 m | 3.39 m |
| 2.0 s | 41 | 19.1 m | 10.8 m | 0.414 m | 3.47 m |

Vertical velocity bias (predicted minus true, + = down):

| Horizon | constant velocity | constant acceleration | ballistic |
|---|---|---|---|
| 0.5 s | -4.91 m/s | +0.42 m/s | -0.01 m/s |
| 1.0 s | -9.79 m/s | +0.31 m/s | +0.01 m/s |
| 2.0 s | -19.57 m/s | -2.65 m/s | +0.03 m/s |

* Constant velocity is wrong by exactly `g h` in velocity and `0.5 g h^2`
  in position: it ignores an acceleration that is known.
* Constant acceleration has almost no bias but a large spread: the
  acceleration is the slope of a short, noisy velocity history, and its
  error is multiplied by `h^2`.
* `auto` switched to the estimated acceleration in 306 of 2348 predictions.
  The true acceleration of every scored object was 9.80 m/s^2, so each of
  those switches was a mistake; `auto` is worse than ballistic everywhere.
* The same ranking holds on the ten earlier hover recordings
  (`offline_recorded_hover.json`).

**Choice.** The ballistic model is the default. In this scenario the debris
is in free fall and gravity is known; the data gives no case in which a
measured acceleration predicts better. `auto`, `ca` and `cv` stay selectable
through the `model` parameter but are not used.

## Live results (predictor node, ballistic)

Six hover flights, 2346 predictions scored afterwards against ground truth.

| Horizon | n scored | not scored: truth had already hit something | RMSE to true surface | RMSE to true centre | Velocity RMSE | Vertical velocity bias |
|---|---|---|---|---|---|---|
| 0.1 s | 1956 | 152 | 0.028 m | 0.231 m | 0.22 m/s | -0.03 m/s |
| 0.25 s | 1700 | 408 | 0.036 m | 0.236 m | 0.22 m/s | -0.02 m/s |
| 0.5 s | 1281 | 827 | 0.056 m | 0.251 m | 0.21 m/s | -0.01 m/s |
| 0.75 s | 831 | 1277 | 0.064 m | 0.274 m | 0.19 m/s | +0.00 m/s |
| 1.0 s | 511 | 1597 | 0.110 m | 0.324 m | 0.20 m/s | +0.01 m/s |
| 1.5 s | 213 | 1895 | 0.160 m | 0.403 m | 0.18 m/s | +0.01 m/s |
| 2.0 s | 41 | 2067 | 0.414 m | 0.672 m | 0.30 m/s | +0.03 m/s |

UAV climbing (one flight, 276 predictions): RMSE to surface 0.004 m at 0.1 s,
0.005 m at 0.5 s, 0.025 m at 1.0 s; vertical velocity bias within 0.02 m/s.

Error against object speed at prediction time (RMSE to surface):

| Horizon | under 4 m/s | 4 to 8 m/s | over 8 m/s |
|---|---|---|---|
| 0.5 s | 0.054 m | 0.058 m | 0.052 m |
| 1.0 s | 0.174 m | 0.071 m | 0.011 m |
| 1.5 s | 0.243 m | 0.046 m | 0.013 m |
| 2.0 s | 0.424 m (n = 39) | 0.000 m (n = 2) | no data |

Slow objects are the ones just released: far away (20 to 33 m, 12 to 25
pixels) and tracked for only a few frames, so their velocity is least certain
and the error of a long extrapolation is largest.

Error against the gap before the frame the prediction was made from:

| Horizon | under 50 ms | 50 to 150 ms | over 150 ms |
|---|---|---|---|
| 0.1 s | 0.020 m (n = 1674) | 0.058 m (249) | 0.019 m (33) |
| 0.25 s | 0.033 m (1465) | 0.048 m (210) | 0.042 m (25) |
| 0.5 s | 0.055 m (1129) | 0.015 m (143) | 0.250 m (9) |
| 1.0 s | 0.113 m (465) | 0.082 m (45) | 0.000 m (1) |

Predictions made from coasting states (no measurement in that frame):
0.042 m at 0.1 s (n = 117), 0.138 m at 0.25 s (28), 0.336 m at 0.5 s (7).

Uncertainty: the true point was within 2 sigma of the prediction in 82 % of
cases at 0.1 s and 100 % from 0.25 s on. The reported covariance is
conservative at longer horizons (as in Phase 3) and somewhat optimistic at
0.1 s, where the 0.2 m surface-to-centre offset dominates.

Processing per frame: median 0.12 ms, 95th percentile 2.2 ms, maximum 6.3 ms
(frame interval 33 ms).

## Limits and failure cases

* **Most long-horizon predictions cannot be checked and are not physically
  meaningful**: of 2108 predictions at 2.0 s only 41 could be scored; in the
  rest the object had already hit the ground, the roof or the UAV before the
  horizon. The predictor has no map and predicts straight through impact.
* Valid only for free fall. No drag, no bounce, no sliding; after an impact
  the track is wrong until it dies.
* Inherits the tracker's limits: the point is the visible surface (about
  0.2 m from the centre); objects seen only at the image border are not
  confirmed and so are not predicted; two touching objects can share a
  merged measurement.
* Newly confirmed tracks of distant objects have the largest long-horizon
  error (0.42 m at 2 s).
* A coasting track is predicted for at most 0.35 s after its last
  measurement; that is when the object passes below the UAV, so predictions
  of the lower part of the fall rely on the last few measurements.
* In run `live_high_release` a 0.5 m box struck the hovering UAV. Nothing
  uses these predictions to avoid debris yet.
* Simulation only, noise-free depth.
