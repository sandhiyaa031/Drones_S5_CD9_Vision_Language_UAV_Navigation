# Mathematical methods

Every equation below is tagged:

* **[implemented]** — used by the running code; the source file is named.
* **[implemented, not default]** — in the code and selectable, but not the
  configuration that was validated or flown.
* **[evaluation only]** — used offline to score results against simulator
  ground truth; never runs in flight.

Nothing here describes mathematics that is not in the code.

---

## 1. Coordinate frames

| Frame | Axes | Used by |
|---|---|---|
| Gazebo world | x east, y north, z up (ENU) | simulator, ground-truth logs |
| PX4 local | north, east, down (NED), origin at the takeoff point | all estimates, all planning |
| Body, PX4 | forward, right, down (FRD) | PX4 attitude quaternion |
| Body, Gazebo | forward, left, up (FLU) | sensor mounting |
| Sensor | +X optical axis, +Y image left, +Z image up | both cameras |

**[implemented]** World ↔ local NED, relative to the takeoff point:

    n = y_w - y_0        e = x_w - x_0        d = -(z_w - z_0)

Height above the takeoff point is `altitude = d_ground - d`.

Camera mounting (`models/x500_rescue`):

| Camera | Optical axis | Image left | Image up |
|---|---|---|---|
| Upward depth (`depth_up_link`) | body up | body left | body backward (−X) |
| Downward mono (`camera_link`) | body down | body left | body forward (+X) |

---

## 2. Camera model

**[implemented]** `debris_detection.py`. Pinhole model without distortion.
For a point `(X, Y, Z)` in the sensor frame (X along the optical axis):

    u = c_x - f_x * Y / X          v = c_y - f_y * Z / X

    f_x = f_y = (W / 2) / tan(HFOV / 2)

| Camera | W × H | HFOV | f | c_x, c_y |
|---|---|---|---|---|
| Upward depth | 640 × 480 | 100° | 268.512 px | 320, 240 |
| Downward mono | 1280 × 960 | 1.74 rad (99.7°) | 539.9 px | 640, 480 |

The depth image stores the **axial** depth `d = X` in metres (verified with
flat targets: the same value at the centre and the corners). `+inf` means
nothing in range, `−inf` means closer than the near clip.

**[implemented]** Back-projection of pixel `(u, v)` with depth `d`:

    p_s = ( d,  -(u - c_x) d / f_x,  -(v - c_y) d / f_y )

---

## 3. From a depth pixel to a point in the PX4 local frame

**[implemented]** `debris_detection.py`

1. Sensor to body (FLU), with the sensor 0.15 m above `base_link`:

       p_FLU = ( -p_s.z,  p_s.y,  p_s.x ) + (0, 0, 0.15)

2. FLU to FRD: `p_FRD = (x, -y, -z)`.
3. Body to local with PX4's attitude quaternion `q` and position `p_uav`,
   both taken **at the image timestamp**:

       p_NED = R(q) p_FRD + p_uav

### Odometry interpolation

**[implemented]** PX4 odometry arrives at 100 Hz; the nearest sample can be
8–12 ms from a depth frame, which at 10 m/s is 0.1 m. For an image stamped
`t` between odometry samples `t_0 <= t <= t_1`:

    a = (t - t_0) / (t_1 - t_0)
    p(t) = (1 - a) p_0 + a p_1
    q(t) = normalise( (1 - a) q_0 + a q_1 )        (sign of q_1 aligned to q_0)

A pose is used only if the bracketing samples are at most 50 ms apart
(30 ms extrapolation at the ends). Stamps are
`vehicle_odometry.timestamp_sample`. With `UXRCE_DDS_SYNCT=0` PX4 time
equals simulation time exactly (measured error 0 ms), so no offset term is
needed in simulation.

---

## 4. Debris detection

**[implemented]** `debris_detection.py`

1. Valid pixels: finite depth in [0.5 m, 35 m].
2. 8-connected components; a component is split where its sorted depths jump
   by more than 0.30 m.
3. Each pixel is back-projected (section 2); the detection is the centroid of
   the region's 3D points, with its extents.
4. Gates: fewer than 12 pixels, lateral extent under 0.08 m, or over 2.5 m
   are rejected.
5. Static filter in the local frame: a detection is dropped if an earlier
   detection at least 0.75 s old lies within 0.20 m and 60 % of the frames
   since then also had one there.
6. Confidence (a heuristic, not a probability):

       c = 0.30 + 0.35 min(1, pixels / 150) + 0.20 (1 - sigma_depth / 0.30) + 0.15 [not at image border]

---

## 5. Tracking

**[implemented]** `debris_tracking.py`

### State and motion

    x = [ p_x, p_y, p_z, v_x, v_y, v_z ]^T          (local NED, z down)

Gravity is a known input, not an estimated state:

    p' = p + v dt + (1/2) g dt^2 e_z        v' = v + g dt e_z        g = 9.8 m/s^2

    x' = F x + B g,     F = [[I, dt I], [0, I]],     B = [ (1/2) dt^2 e_z ; dt e_z ]

Process noise, white-noise acceleration, per axis with sigma_a = 3 m/s^2:

    Q_axis = sigma_a^2 [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]

`dt` is the difference of the sensor timestamps of consecutive processed
frames; no frame rate is assumed.

### Measurement

    z = H x + w,     H = [ I_3  0 ],     R = sigma_m^2 I_3

sigma_m = 0.10 m; 0.30 m when the object is cut by the image border; half
the region size when two objects are merged into one region. `z` is the
centroid of the **visible surface**, not the box centre.

### Kalman filter

    predict:  x^- = F x + B g                P^- = F P F^T + Q
    update:   y = z - H x^-                  S = H P^- H^T + R
              K = P^- H^T S^-1               x = x^- + K y
              P = (I - K H) P^-

Initial velocity 0 with standard deviation 8 m/s.

### Association

**[implemented]** Mahalanobis gating and Hungarian assignment:

    cost(i, j) = y_ij^T S_ij^-1 y_ij

A pair is allowed only if `cost < 16.27` (chi-square, 3 degrees of freedom,
99.9 %) **and** `|y_ij| < 25 m/s * time_since_update + 0.6 m`. The minimum
total cost assignment is found with
`scipy.optimize.linear_sum_assignment`. Unassigned detections start
tentative tracks.

### Life cycle

TENTATIVE → CONFIRMED after 3 fully visible measurements spanning at least
0.10 m. CONFIRMED → COASTING on a frame without a measurement. Deleted after
6 missed frames or 1.0 s (tentative: 2 missed frames).

**[implemented, not default]** `gravity_m_s2 = 0` gives a plain
constant-velocity filter. It was measured to under-estimate the fall speed
by about 1.1 m/s (median) and is not used.

---

## 6. Trajectory prediction

**[implemented]** `debris_prediction.py`. From the track state at its own
timestamp, for horizons h = 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0 s:

    p(t + h) = p + v h + (1/2) a h^2        v(t + h) = v + a h
    P(h) = F(h) P F(h)^T + Q(h)

| Model | Acceleration `a` | Status |
|---|---|---|
| Ballistic | (0, 0, 9.8) m/s², known gravity | **[implemented]**, default, validated |
| Constant velocity | 0 | **[implemented, not default]** |
| Constant acceleration | least-squares slope of the track's velocity over the last 0.4 s | **[implemented, not default]** |
| Auto | gravity unless the estimated acceleration differs by more than 4 m/s² | **[implemented, not default]** |

Why ballistic: a constant-velocity prediction of a body in free fall is
wrong by exactly `g h` in velocity and `(1/2) g h^2` in position (4.9 m at
1 s). Measured position RMSE at 1.0 s: constant velocity 4.44 m, constant
acceleration 2.36 m, ballistic 0.110 m.

Drag, bounce and sliding are not modelled. The predictor has no map.

---

## 7. Collision risk

**[implemented]** `debris_risk.py`

### Relative motion and closest approach

    r(h) = p_debris(t + h) - p_uav(t + h)

Default UAV motion: constant velocity, `p_uav(t + h) = p_uav + v_uav h`.
The predicted debris path is sampled every 0.01 s between the predictor's
states (constant acceleration inside each interval) and

    d_min = min over h of |r(h)|          t_ca = the h where it occurs

Nothing is evaluated beyond the last predicted state (2 s).

**[implemented, comparison only]** The constant-relative-velocity closed
form is computed and published for comparison, not used for the state:

    t* = clamp( -(r_0 . v_rel) / |v_rel|^2, 0, horizon )        d = | r_0 + v_rel t* |

**[implemented, not default]** (Phase 6.1) Two other UAV motion models:

    constant acceleration:  p_uav + v_uav h + (1/2) a_uav h_a^2 + a_uav h_a (h - h_a),   h_a = min(h, 0.6 s)

with `a_uav` from PX4 (ignored below 0.8 m/s², clipped at 8 m/s²), and the
vehicle response model of section 8. Measured: neither gives an earlier
warning than constant velocity in the hard-acceleration case, so constant
velocity remains the default.

### Geometry

    debris radius  = 0.71 * size          (half-diagonal of a square face)
    contact radius = 0.40 m + debris radius                (0.68 m for a 0.4 m box)
    safety radius  = contact radius + 0.30 m               (0.98 m for a 0.4 m box)

0.40 m is the X500's motor arm (0.246 m) plus propeller radius (0.14 m),
rounded up. The 0.30 m margin covers measured prediction and localisation
error.

### Uncertainty margin

    sigma_r = sqrt( u^T P_pos u ),     u = r / |r| at closest approach
    margin  = min( sigma_r,  0.10 + 0.40 t_ca )     [m]
    conservative separation = d_min - margin

The cap `0.10 + 0.40 t` is an envelope fitted to measured prediction error,
because the filter's covariance is far too wide at long horizons.

### Risk states

| State | Condition |
|---|---|
| CRITICAL | `d_min < contact`, or `d_min < safety` with `t_ca <= 0.75 s` |
| WARNING | `d_min < safety`, `t_ca > 0.75 s` |
| WATCH | `conservative separation < safety` |
| SAFE | otherwise |
| POTENTIAL_THREAT | unconfirmed track, speed ≥ 2 m/s, free-fall path enters the safety radius |

    risk_score = clamp( (safety + 1 - d_cons) / (safety + 1 - contact), 0, 1 ) * ( 1 - 0.5 clamp(t_ca / horizon, 0, 1) )

The score is a convenience; decisions use the states.

---

## 8. Avoidance planning

**[implemented]** `debris_avoidance.py`

### Vehicle response model

A candidate is a position setpoint. Its consequence is simulated with a
reduced model of PX4's cascaded position controller, integrated at 20 ms
over 2 s:

    v_cmd = clamp( k_p (setpoint - p), v_max )
    a_cmd = clamp( k_v (v_cmd - v), a_max )          per axis
    da/dt = (a_cmd - a) / tau                         attitude lag

| | tau | k_p | k_v | a_max |
|---|---|---|---|---|
| Horizontal | 0.493 s | 0.827 1/s | 3.203 1/s | 11.08 m/s² |
| Vertical | 0.02 s | 1.181 1/s | 12.0 1/s | 6.75 m/s² |

The acceleration state starts from PX4's measured acceleration. Constants
were fitted to recorded flights (133 start states); the largest position
error up to 1 s ahead was 0.25 m, which is added to every clearance
requirement as `model_error`.

### Candidates

Hold (the mission setpoint); eight horizontal directions at 2.5 m and 4.0 m;
climb 2.0 m; descend 1.0 m: 19 in all, plus the setpoint currently flown.

### Hard constraints

A candidate is rejected if anywhere on its simulated trajectory:

    altitude outside [1.5 m, 12 m]
    horizontal speed > 5.0 m/s   or   vertical speed > 2.5 m/s
    the path enters a structure keep-out box outside the takeoff corridor
    for ANY tracked object:  |r(t)| < safety radius + margin(t) + model_error

### Cost of a feasible candidate

    J = w_risk * risk + w_dist * deviation + w_energy * effort + w_smooth * change + w_alt * altitude_change

| Term | Definition | Weight |
|---|---|---|
| risk | `clip(1 - clearance / 1.5 m, 0, 1)`, clearance = smallest (separation − required) | 10 |
| deviation | distance of the setpoint from the nominal path [m] | 1.0 |
| effort | distance of the setpoint from the present position [m] | 0.5 |
| change | distance from the setpoint currently flown [m] | 0.5 |
| altitude_change | \|setpoint altitude − mission altitude\| [m] | 0.5 |

The lowest-cost feasible candidate is flown. A setpoint being flown is kept
while feasible unless another is cheaper by 3.0. If nothing is feasible the
candidate with the largest clearance is flown ("best effort"), but only if
it gains at least 0.2 m over holding.

### Speed cap

**[implemented]** `avoidance_interface.py`. PX4 commands velocity
`MPC_XY_P × position error` (`MPC_XY_P = 0.95`), so a horizontal speed cap
`v_lim` is applied by keeping the setpoint no further than

    reach = v_lim / 0.95

from the vehicle.

### Perception health

**[implemented]** From the age `tau_p` of the newest depth-derived frame:
FRESH `tau_p <= 0.15 s`, DEGRADED `<= 0.50 s`, STALE beyond. The last risk
frame keeps being used, each debris path advanced by `tau_p`, for 1.0 s.

---

## 9. Flight safety monitor

**[implemented]** `flight_safety.py`, checked on every PX4 position sample
above 0.5 m:

    impact:                 sqrt(a_x^2 + a_y^2 + a_z^2) > 30 m/s^2
    position discontinuity: | dp - v_mean dt | > 0.5 m + (1/2)(30) dt^2
    velocity discontinuity: | dv | > 3.0 m/s + 30 dt
    speed envelope:         horizontal > 6.5 m/s  or  vertical > 3.0 m/s
    estimator reset counter changed, or any non-finite value

Any violation latches FAILSAFE and requests PX4 AUTO LAND.

---

## 10. Survivor perception

**[implemented]** `survivor_detection.py`, `survivor_confirmation.py`. This is
image-plane processing only.

### Candidate detector (simulation baseline)

Colour masks in OpenCV HSV (H 0–179, S and V 0–255):

| Cue | Hue | Saturation | Value |
|---|---|---|---|
| Clothing (orange) | 5–22 | ≥ 170 | ≥ 45 |
| Head (skin tone) | 8–28 | 35–125 | ≥ 70 |
| Legs (blue) | 100–128 | ≥ 120 | ≥ 30 |

For each connected clothing region (regions closer than 0.35 × the smaller
one's long side are merged):

    head   = a skin blob touching the clothing, blob area / clothing area in [0.06, 2.5], blob fill >= 0.45
    shape  = (long side / short side of the minimum-area rectangle >= 1.6) AND (clothing area / rectangle area <= 0.88)
    legs   = a blue blob touching the clothing, area ratio in [0.03, 1.5]

    confidence = 0.30 + 0.35 head + 0.20 shape + 0.15 legs

`confidence >= 0.55` is a strong candidate; `>= 0.30` a weak one.

### Temporal confirmation

A candidate is reported only when detected in at least 5 of the last 8
frames. Frame-to-frame association: centre distance below 0.75 × the larger
box side, and box area within a factor 3.

### Semantic state

SURVIVOR_CONFIRMED requires a stable track **and** a VLM answer "confirm"
with confidence ≥ 0.6. The VLM is a learned model, not an equation.

### Camera-ray geometry

**Not implemented online.** The node publishes pixel coordinates only; no
ray is cast and no 3D position is estimated. That is Phase 8.

**[evaluation only]** `scripts/survivor/truth.py` projects the known manikin
position into the image with the pinhole model of section 2 and the recorded
simulator pose, and tests occlusion by intersecting the camera-to-point
segment with the world's boxes. This is used only to label frames for
scoring.

---

## 11. Evaluation metrics

**[evaluation only]**

    precision = TP / (TP + FP)        recall = TP / (TP + FN)        F1 = 2 P R / (P + R)
    RMSE = sqrt( mean( e^2 ) )

* Debris position error: distance from the estimate to the true box surface
  (0 when it lies on it), and to the true box centre.
* Collision (truth): distance from the UAV centre to the true box surface
  ≤ 0.40 m. Near miss: ≤ 0.70 m.
* Warning lead time: first WARNING or CRITICAL frame to the true closest
  approach (Phase 5), or manoeuvre start to the box reaching the UAV's
  altitude (Phase 6).
* ID switches: CLEAR MOT convention (a truth-to-track pairing is kept while
  it stays within 0.6 m).
