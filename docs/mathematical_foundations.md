# Mathematical foundations — derivations and worked numbers

The main [README](../README.md) §8 gives each model in compact form. This file adds the derivations and small
numerical examples you need to answer follow-up questions. It is organised as the README is: A frames →
N metrics.

**Labels used throughout**

* **[Implemented]** — the equation is what the code does; the source file is named.
* **[Background]** — textbook theory included to explain an implemented model, *not* claimed as project code.
* **[Illustrative]** — numbers invented (or computed by calling the project's own functions on invented inputs)
  to make an equation concrete. They are **not measured results**.

Unit conventions: metres, seconds, radians unless stated. Vectors are bold, columns. PX4 local frame is NED
(x north, y east, z **down**), so "higher" means "more negative z".

---

## A. Frames, rotations and transforms

| Frame | Axes | Where used |
|---|---|---|
| Gazebo world | x east, y north, z up (ENU) | simulator, offline scoring |
| PX4 local | north, east, down (NED), origin at takeoff | every online estimate, plan and setpoint |
| Body, PX4 | forward, right, down (FRD) | PX4 attitude quaternion |
| Body, Gazebo | forward, left, up (FLU) | sensor mounting in the SDF |
| Camera | project convention: +X optical axis, +Y image left, +Z image up | `debris_detection.py` |
| `odom` / `base_level` / `map` | ENU; `base_level` = position + yaw only | `slam_bridge`, `slam_toolbox` |

**[Implemented]** World ↔ local, relative to the takeoff point $(x_0,y_0,z_0)$:

$$n = y_w - y_0,\qquad e = x_w - x_0,\qquad d = -(z_w - z_0)$$

**[Background → Implemented]** A point in frame $c$ expressed in frame $w$:

$$\mathbf p_w = \mathbf R_{wc}\,\mathbf p_c + \mathbf t_{wc}
\;\Longleftrightarrow\;
\begin{bmatrix}\mathbf p_w\\1\end{bmatrix}=
\underbrace{\begin{bmatrix}\mathbf R_{wc}&\mathbf t_{wc}\\ \mathbf 0^\top&1\end{bmatrix}}_{\mathbf T_{wc}}
\begin{bmatrix}\mathbf p_c\\1\end{bmatrix}$$

Subscript $wc$ reads "from $c$ to $w$": $\mathbf R_{wc}$ rotates vectors written in $c$ into $w$, and
$\mathbf t_{wc}$ is the origin of $c$ written in $w$. Chains compose left to right,
$\mathbf T_{wb}\mathbf T_{bc}=\mathbf T_{wc}$. $\mathbf R$ is orthonormal ($\mathbf R^\top\mathbf R=\mathbf I$,
$\det=+1$), so $\mathbf R^{-1}=\mathbf R^\top$.

**Code convention.** `survivor_localization.quat_to_matrix(w,x,y,z)` returns $\mathbf R_{\text{local,body}}$
(local-from-body) from PX4's quaternion; `camera_pose` returns the camera position
$\mathbf c=\mathbf p_{uav}+\mathbf R\,\mathbf o_{body}$ with $\mathbf o_{body}=(0,0,0.14)$ m (camera 0.14 m below
`base_link`). For the depth camera, `debris_detection.py` goes sensor → body (FLU, +0.15 m up) → FRD →
local:

$$\mathbf p_{FLU}=(-p_{s,z},\;p_{s,y},\;p_{s,x})+(0,0,0.15),\quad
\mathbf p_{FRD}=(x,-y,-z),\quad
\mathbf p_{NED}=\mathbf R(\mathbf q)\,\mathbf p_{FRD}+\mathbf p_{uav}$$

**Pose at the image time.** Odometry arrives at 100 Hz; an image stamped $t$ lies between samples $t_0\le t\le t_1$:

$$a=\tfrac{t-t_0}{t_1-t_0},\quad \mathbf p(t)=(1-a)\mathbf p_0+a\,\mathbf p_1,\quad
\mathbf q(t)=\operatorname{normalise}\big((1-a)\mathbf q_0+a\,\mathbf q_1\big)$$

(normalised linear interpolation of quaternions, sign of $\mathbf q_1$ aligned to $\mathbf q_0$; used only if
$t_1-t_0\le 50$ ms). At 10 m/s a 10 ms timestamp error is already 0.1 m — the reason for the interpolation and for
running PX4 with `UXRCE_DDS_SYNCT=0` (PX4 time = simulation time, measured error 0 ms; a *simulation* time base, not
a hardware clock-sync method).

---

## B. Camera model and back-projection

**[Implemented]** (`debris_detection.py`; `survivor_localization.py` uses the equivalent image convention
$u$ right, $v$ down). Pinhole camera, no distortion. Focal length from the horizontal field of view:

$$f_x=f_y=\frac{W/2}{\tan(\mathrm{HFOV}/2)}$$

| Camera | $W\times H$ | HFOV | $f$ (px) | $(c_x,c_y)$ |
|---|---|---|---|---|
| Upward depth | 640×480 | 100° | 268.512 | (320, 240) |
| Downward RGB | 1280×960 | 1.74 rad (99.7°) | 539.9 | (640, 480) |

For the depth camera (project convention, $X$ along the optical axis) projection and back-projection with *axial*
depth $d=X$:

$$u=c_x-f_x\frac{Y}{X},\quad v=c_y-f_y\frac{Z}{X}
\qquad\Longleftrightarrow\qquad
\mathbf p_s=\Big(d,\;-\frac{(u-c_x)d}{f_x},\;-\frac{(v-c_y)d}{f_y}\Big)$$

**[Illustrative]** Pixel $(400,240)$ at depth 8.0 m: $\mathbf p_s=(8.0,\,-2.38,\,0.0)$ m — 2.38 m to the
image-right side of the optical axis, level with it.

---

## C. State estimation and odometry

**What the project does *not* implement.** The vehicle's position/velocity/attitude estimate comes from
**PX4's EKF2**, fusing the simulated IMU, barometer, magnetometer and GNSS. The project reads it
(`vehicle_odometry`, `vehicle_local_position`) and never re-estimates it. EKF2 is an extended Kalman filter whose
state includes position, velocity, attitude and sensor biases.

**[Background]** An EKF alternates, for state $\mathbf x$ and covariance $\mathbf P$:

$$\text{predict: }\;\hat{\mathbf x}^-=f(\hat{\mathbf x},\mathbf u),\;\;
\mathbf P^-=\mathbf F\mathbf P\mathbf F^\top+\mathbf Q$$
$$\text{update: }\;\mathbf y=\mathbf z-h(\hat{\mathbf x}^-),\;\;
\mathbf S=\mathbf H\mathbf P^-\mathbf H^\top+\mathbf R,\;\;
\mathbf K=\mathbf P^-\mathbf H^\top\mathbf S^{-1},\;\;
\hat{\mathbf x}=\hat{\mathbf x}^-+\mathbf K\mathbf y,\;\;
\mathbf P=(\mathbf I-\mathbf K\mathbf H)\mathbf P^-$$

$\mathbf Q$ = process noise (the model is imperfect), $\mathbf R$ = measurement noise, $\mathbf H=\partial h/\partial\mathbf x$
(the Jacobian; for a linear filter, the measurement matrix). Odometry is the *incremental* estimate of motion;
"position" in `vehicle_odometry` is that estimate integrated and corrected by GNSS/baro inside EKF2. Uncertainty
enters through sensor noise, timestamp error and model error. The project's own estimators built on this are the
**debris Kalman filter** (§F; linear) and the **survivor EKF** (§D–E; nonlinear measurement, static state).

---

## D. Survivor localisation geometry

**Problem.** One camera frame gives a *direction*, not a distance. Detection ($u,v$) is therefore turned into a ray,
and the 3-D point is found by (1) intersecting the ray with an assumed-height plane, then (2) refining with an EKF
as the vehicle moves.

**[Implemented]** (`survivor_localization.py`)

1. **Ray in the body frame** (downward camera: image right = body right, image up = body forward, optical axis down):
$$\mathbf r_b=\frac{(\,-(v-c_y)/f_y,\;(u-c_x)/f_x,\;1\,)}{\lVert\cdot\rVert},\qquad
\mathbf r=\mathbf R\,\mathbf r_b,\qquad \mathbf c=\mathbf p_{uav}+\mathbf R(0,0,0.14)^\top$$
2. **Ray–plane intersection** with the horizontal plane $z=z_{\text{plane}}$ (NED; ground level minus the assumed
height 0.9 m):
$$\mathbf p=\mathbf c+s\,\mathbf r,\qquad s=\frac{z_{\text{plane}}-c_z}{r_z}\quad(\text{valid only if }r_z>0,\ s>0)$$
3. **Hover bias.** If the true height differs from the assumed height by $\Delta h$, the horizontal error is
$\Delta h\cdot\tan\theta$, with $\theta$ the angle between the ray and straight down. The first estimate is
therefore coarse (large covariance *along* the ray).
4. **Multi-view triangulation** (least squares, used for the merge check and as the idea behind the EKF). The point
minimising $\sum_i\lVert(\mathbf I-\mathbf r_i\mathbf r_i^\top)(\mathbf p-\mathbf c_i)\rVert^2$ — the summed squared
distance to every ray — satisfies the normal equations
$$\Big[\sum_i(\mathbf I-\mathbf r_i\mathbf r_i^\top)\Big]\mathbf p=\sum_i(\mathbf I-\mathbf r_i\mathbf r_i^\top)\mathbf c_i$$
whose condition number reports how parallel the rays are (`triangulate()`; > 10⁹ → rejected).
5. **EKF update.** State = the static point $\mathbf p$ (no motion model, so predict is the identity). Measurement
$\mathbf z=(u,v)$,
$$h(\mathbf p)=\Big(c_x+f_x\frac{y_b}{z_b},\;c_y-f_y\frac{x_b}{z_b}\Big),\quad
\mathbf b=\mathbf R^\top(\mathbf p-\mathbf c)=(x_b,y_b,z_b)$$
with the $2\times3$ Jacobian $\mathbf H=\dfrac{\partial h}{\partial\mathbf b}\mathbf R^\top$
(`projection_jacobian`), then the standard update above. Frames are skipped if the box is cut by the image border,
the tilt exceeds 35°, the ray is flatter than ≈20° below the horizon, or the innovation fails the 99.9 %
$\chi^2_2$ gate (13.8).

**[Illustrative, computed with the project functions]** Survivor at $(6,3)$ m, 0.4 m above ground; camera at
5 m altitude, level. From $N=4$ m the pixel is $(640,\,237.9)$, ray 24.2° off nadir. A single-view estimate with the
*assumed* 0.9 m height lands at $(5.78,3.00)$, error $0.5\tan24.2^\circ\approx0.22$ m. Triangulating the views from
$N=4$ m and $N=8$ m (rays from opposite sides, condition number 6.0) returns $(6.00,3.00,-0.40)$ — the height
assumption no longer matters. This is why the search planner adds an inspection detour ("a baseline").

---

## E. Uncertainty

**[Implemented]**

* **Measurement noise** per pixel observation:
$$\sigma_{px}^2=\underbrace{\max(3,\,0.15\,b)^2}_{\text{box-size noise}}+
\underbrace{(f\,\sigma_{att})^2}_{\sigma_{att}=1.5^\circ}+
\underbrace{\Big(\frac{f\,\sigma_{pos}}{\rho}\Big)^2}_{\sigma_{pos}=0.15\text{ m},\ \rho=\text{range}}$$
($b$ = longer bounding-box side in px). Frames closer than 0.5 s share their errors and are down-weighted.
* **Reported covariance** = filter covariance + a 0.2 m systematic floor (vehicle position error and
centre-of-box ≠ centre-of-person do not average out over frames).
* **Reported horizontal 1σ** = $\sqrt{\lambda_{\max}(\operatorname{Cov}(\mathbf x)_{[1:2,1:2]})}$ — the worst
horizontal direction (`horizontal_sigma`).
* States: COARSE (one viewing direction), LOCALIZED (σ ≤ 0.5 m and ≥ 10 frames), not published below 5 frames.

**Interpretation.** A reported 1σ is the filter's *belief* about its own error, built from assumed noise levels. The
measured *error* is the distance to simulator truth, computed offline only. They agree when the assumptions hold
(8 of 8 live Phase 8 instances were inside 2σ except an occluded duplicate at 2.2σ) and disagree under partial
occlusion, which biases the visible centre in a way the covariance does not model. The 1σ is published to the mission
report; a candidate with fewer than 15 frames is labelled `LOW_CONFIDENCE`.

---

## F. Debris tracking

**[Implemented]** (`debris_tracking.py`). State $\mathbf x=[\mathbf p;\mathbf v]\in\mathbb R^6$ in local NED.
Gravity is a *known input*, not an estimated state:

$$\mathbf x_{k+1}=\mathbf F\mathbf x_k+\mathbf B g,\quad
\mathbf F=\begin{bmatrix}\mathbf I&\Delta t\,\mathbf I\\\mathbf 0&\mathbf I\end{bmatrix},\quad
\mathbf B=\begin{bmatrix}\tfrac12\Delta t^2\mathbf e_z\\\Delta t\,\mathbf e_z\end{bmatrix},\ g=9.8\ \mathrm{m/s^2}$$

White-noise-acceleration process noise per axis, $\sigma_a=3$ m/s²:
$\mathbf Q=\sigma_a^2\begin{bmatrix}\Delta t^4/4&\Delta t^3/2\\\Delta t^3/2&\Delta t^2\end{bmatrix}$.
$\Delta t$ is the difference of the *sensor timestamps* of consecutive frames — no frame rate is assumed.
Measurement $\mathbf z=\mathbf H\mathbf x+\mathbf w$, $\mathbf H=[\mathbf I_3\ \mathbf 0]$,
$\mathbf R=\sigma_m^2\mathbf I$ with $\sigma_m=0.10$ m (0.30 m if the object is cut by the image border). $\mathbf z$ is
the centroid of the *visible surface*, not the box centre.

**Association.** For track $i$ and detection $j$: residual $\mathbf y_{ij}=\mathbf z_j-\mathbf H\hat{\mathbf x}^-_i$,
innovation covariance $\mathbf S_{ij}=\mathbf H\mathbf P^-_i\mathbf H^\top+\mathbf R_j$ and

$$d_M^2=\mathbf y_{ij}^\top\mathbf S_{ij}^{-1}\mathbf y_{ij}\ \le 16.27\ \ (\chi^2_{3},\,99.9\,\%)\ \ \text{and}\ \ \lVert\mathbf y_{ij}\rVert<25\,t_{\text{since update}}+0.6\ \text{m}$$

This **is** the implemented cost (`cost = yᵀS⁻¹y`); the minimum-total-cost assignment uses
`scipy.optimize.linear_sum_assignment` (the Hungarian algorithm). Life cycle: TENTATIVE → CONFIRMED after 3
fully visible measurements spanning ≥ 0.10 m; COASTING when a frame has none; deleted after 6 missed frames or 1.0 s.

**[Illustrative]** A track at $z=-12$ m falling at 6 m/s predicted over $\Delta t=0.04$ s:
$z'=-12+6(0.04)+\tfrac12 9.8(0.04)^2=-11.752$ m, $v_z'=6+9.8(0.04)=6.392$ m/s. A detection with residual
$\mathbf y=(0.12,-0.05,0.20)$ m and $\mathbf S=0.04\,\mathbf I$ gives $d_M^2=1.42<16.27$ → accepted.
(Why the gate value: $\chi^2_3$ 99.9 % quantile = 16.27.)

*Why gravity in the filter:* without it the filter under-estimates fall speed by about 1.1 m/s (median, measured);
with it the vertical velocity bias is −0.04…+0.03 m/s.

---

## G. Trajectory prediction

**[Implemented]** (`debris_prediction.py`). From the track state at its own timestamp, for horizons
$h\in\{0.1,0.25,0.5,0.75,1.0,1.5,2.0\}$ s:

$$\mathbf p(t+h)=\mathbf p+\mathbf v\,h+\tfrac12\mathbf a\,h^2,\qquad \mathbf v(t+h)=\mathbf v+\mathbf a\,h,\qquad
\mathbf a=(0,0,9.8)\ \text{(ballistic, default)}$$
$$\mathbf P(h)=\mathbf F(h)\mathbf P\mathbf F(h)^\top+\mathbf Q(h)$$

The constant-velocity ($\mathbf a=0$) and estimated-constant-acceleration variants are implemented but **not
default**. Drag, bounce and sliding are not modelled and the predictor has no map (it predicts *through*
structure).

**Error growth.** A velocity error $\delta v$ gives a position error $\delta v\,h$, and a wrong acceleration
$\delta a$ gives $\tfrac12\delta a\,h^2$. Using constant velocity on a free-falling body makes $\delta a=g$:
$\tfrac12 g h^2=4.9$ m at $h=1$ s. Measured RMSE to the true surface (6 flights, 2346 predictions):

| Horizon (s) | 0.1 | 0.5 | 1.0 | 1.5 | 2.0 |
|---|---|---|---|---|---|
| Ballistic (default) RMSE (m) | 0.028 | 0.056 | 0.110 | 0.160 | 0.414 |
| Constant velocity (m) | – | – | 4.44 | – | 19.06 |
| Constant acceleration (est.) (m) | – | – | 2.36 | – | 10.81 |

Only 41 of 2108 predictions at 2.0 s could be scored (the object had usually already hit something), so the 2 s
figure rests on little data.

---

## H. Collision risk

**[Implemented]** (`debris_risk.py`). With $\mathbf r(h)=\mathbf p_{deb}(t+h)-\mathbf p_{uav}(t+h)$ and the UAV assumed
to keep its present velocity ($\mathbf p_{uav}(t+h)=\mathbf p_{uav}+\mathbf v_{uav}h$), the predicted path is sampled
every 0.01 s and

$$d_{\min}=\min_h\lVert\mathbf r(h)\rVert,\qquad t_{ca}=\arg\min_h\lVert\mathbf r(h)\rVert$$

**Closed form (constant relative velocity) — computed and published for comparison, not used for the state.**
Minimising $\lVert\mathbf r_0+\mathbf v_{rel}\tau\rVert^2$ over $\tau$ gives
$\tau^*=-\dfrac{\mathbf r_0\cdot\mathbf v_{rel}}{\mathbf v_{rel}\cdot\mathbf v_{rel}}$, clamped to $[0,\text{horizon}]$,
then $d=\lVert\mathbf r_0+\mathbf v_{rel}\tau^*\rVert$. It is valid only while the relative velocity is constant —
exactly what gravity breaks (see example).

**Radii.** Debris radius $=0.71\,\text{size}$ (half-diagonal of a square face); contact radius
$=0.40+0.71\,\text{size}$; safety radius $=\text{contact}+0.30$ m. A 0.4 m box: $0.684$ m and $0.984$ m
(computed by `radii()`); 0.40 m is the X500 arm (0.246 m) plus propeller radius (0.14 m), rounded up.

**Uncertainty margin.** $\sigma_r=\sqrt{\mathbf u^\top\mathbf P_{pos}\mathbf u}$ with $\mathbf u=\mathbf r/\lVert\mathbf r\rVert$
at closest approach; $\text{margin}=\min(\sigma_r,\;0.10+0.40\,t_{ca})$ — the cap is an envelope fitted to measured
error because the filter covariance is far too wide at long horizons. $\text{conservative separation}=d_{\min}-\text{margin}$.

**States.**

| State | Condition |
|---|---|
| CRITICAL | $d_{\min}<$ contact, **or** $d_{\min}<$ safety and $t_{ca}\le0.75$ s |
| WARNING | $d_{\min}<$ safety, $t_{ca}>0.75$ s |
| WATCH | conservative separation < safety |
| SAFE | otherwise |
| POTENTIAL_THREAT | unconfirmed track, speed ≥ 2 m/s, free-fall path enters the safety radius |

**[Illustrative]** UAV hovering at the origin; a 0.4 m box 12 m above it, 0.3 m north of the vertical, falling at
6 m/s. $\mathbf r_0=(0.3,0,-12)$, $\mathbf v_{rel}=(0,0,6)$, $\mathbf a=(0,0,9.8)$.

* Ballistic: $-12+6\tau+4.9\tau^2=0\Rightarrow\tau=1.068$ s; $d_{\min}=0.30$ m $<0.684$ → **CRITICAL**, lead time ≈ 1.07 s.
* Closed form (no gravity): `linear_closest_approach` returns $t^*=2.0$ s, $d=0.30$ m — the *timing* is wrong by 0.93 s,
  and at $t=1.068$ s it places the box 5.6 m above the UAV instead of at it. Hence the ballistic model.
* Context: the avoidance planner needs ≈ 0.9 s of warning (measured), so a 1.07 s lead is inside the envelope but with
  little to spare; the measured median lead in the Phase 6 hover tests was 1.30 s.

---

## I. Avoidance planning

**[Implemented]** (`debris_avoidance.py`). The planner does not command the UAV; it publishes a position setpoint
*proposal* (`AvoidanceCommand`). Each of 19 candidate setpoints (hold; 8 horizontal directions at 2.5 m and 4.0 m;
climb 2 m; descend 1 m) is rolled out through a reduced model of PX4's cascaded position controller, integrated at
20 ms over 2 s, per axis:

$$\mathbf v_{cmd}=\operatorname{clamp}\big(k_p(\mathbf p_{sp}-\mathbf p),\,v_{\max}\big),\quad
\mathbf a_{cmd}=\operatorname{clamp}\big(k_v(\mathbf v_{cmd}-\mathbf v),\,a_{\max}\big),\quad
\dot{\mathbf a}=\frac{\mathbf a_{cmd}-\mathbf a}{\tau}$$

| | $\tau$ | $k_p$ | $k_v$ | $a_{\max}$ |
|---|---|---|---|---|
| Horizontal | 0.493 s | 0.827 s⁻¹ | 3.203 s⁻¹ | 11.08 m/s² |
| Vertical | 0.02 s | 1.181 s⁻¹ | 12.0 s⁻¹ | 6.75 m/s² |

Constants were *fitted* to 133 recorded start states (largest position error up to 1 s ahead: 0.25 m, added to every
clearance requirement as `model_error`). **Hard constraints** (rejected anywhere on the rollout): altitude outside
[1.5, 12] m; horizontal speed > 5 m/s or vertical > 2.5 m/s; entering a structure keep-out box; separation to *any*
track < safety radius + margin(t) + model error. **Cost** of a feasible candidate:

$$J=10\,\text{risk}+1.0\,\text{deviation}+0.5\,\text{effort}+0.5\,\text{change}+0.5\,\text{altitude}$$

with $\text{risk}=\operatorname{clip}(1-\text{clearance}/1.5\ \text{m},0,1)$. Lowest feasible $J$ wins; a setpoint in force
is kept unless another is cheaper by 3.0. If nothing is feasible the largest-clearance candidate is flown only if it
gains ≥ 0.2 m over holding (logged "best effort").

**Why late detection is dangerous.** The vehicle responds with lag: the response is not instantaneous even for a
maximal command. **[Illustrative, from `simulate_response`]** a 4 m north-west setpoint commanded at $t=0$ from hover
moves the UAV only 0.35 m by 0.5 s, 1.73 m by 1.0 s and 1.94 m at 1.07 s (speed ≈ 3.4 m/s); it arrives at 4.0 m by
2 s. With 1.07 s to impact there is only ≈ 1.9 m of lateral displacement available — it explains the measured ≈ 0.9 s
minimum warning.

**Speed cap.** PX4 commands velocity $\approx\text{MPC\_XY\_P}\times\text{position error}$ with
$\text{MPC\_XY\_P}=0.95\ \text{s}^{-1}$ (default, unchanged). The controller therefore limits speed by keeping the
setpoint no more than $\text{reach}=v_{lim}/0.95$ ahead of the vehicle: a 2 m/s cap ⇒ 2.1 m; a 4 m error would
otherwise ask for 3.8 m/s (`avoidance_interface.limit_setpoint`).

---

## J. SLAM and navigation

| Term | Meaning | In this project |
|---|---|---|
| Odometry | motion estimated between times | PX4 EKF2 `vehicle_odometry` (input to SLAM) |
| Localisation | pose in a *given* map | `slam_toolbox` TF `map→odom` |
| Mapping | building an environment representation | occupancy grid, 0.25 m |
| SLAM | jointly estimating pose and map | `slam_toolbox` (2-D pose graph with scan matching, loop closure) |
| Path planning | choosing a route | `navigation.py`: A* |
| Trajectory control | following the route | `flight_controller` position setpoints + PX4 loops |

**[Implemented]** The map is a **2-D slice at the 6 m flight altitude** from a 720-ray planar lidar; it is not 3-D
SLAM. `slam_bridge` levels each scan with the attitude, drops returns that leave the flight plane by > 0.8 m and
forwards scans only within ±0.7 m of 6 m. **Planning grid:** 0.5 m cells, obstacles inflated by 1.75 m (keep-out),
extra cost up to 4 m clearance, unknown cells passable at extra cost 0.6. **A\*** minimises
$\sum\text{cost}(\text{cell})\times\text{step length}$ on that grid; a 3.5 m look-ahead point on the route is sent as the
goal; the route is re-planned every 1 s and whenever it becomes blocked. Because the map frame and the PX4 frame differ
(by 5–7° in the village flights), goals are converted between frames through the two *current* pose estimates, so
a rotated map does not matter. **Not used:** simulator ground truth (offline scoring only); SLAM output to
*stabilise* the vehicle (PX4 does that with simulated GNSS — this is **not** GPS-denied flight).

Limits visible in the evidence: SLAM RMSE 0.08–0.57 m over 17 village flights, but 0.50–2.80 m in the urban world
(repetitive facades), where it is *less* accurate than the PX4 odometry it is given. A 2-D slice cannot see overhead
slabs: two early urban flights hit slabs during a climb-first return (§M).

---

## K. Coverage planning

**[Implemented]** (`search_planning.py`). For a camera with focal lengths $f_x,f_y$ and image $W\times H$ at height
$h$ above the plane of interest (flight altitude minus the 0.9 m target height), the usable ground footprint has
half-extents (forward, right) with a usable-image margin $m=0.8$:

$$\big(\ell_f,\ \ell_r\big)=m\,h\Big(\frac{H/2}{f_y},\ \frac{W/2}{f_x}\Big)$$

Parallel lanes run along the longer side; with neighbouring-swath overlap $\omega=0.5$ and $\ell_\perp$ the
across-lane half extent:

$$\text{spacing}=2\ell_\perp(1-\omega),\qquad n_{lanes}=\Big\lceil\frac{\text{width}}{\text{spacing}}\Big\rceil$$

Every ground point is therefore imaged from two lanes — from above and from the side. A 0.5 m grid records which cells
the footprint polygon (from the actual pose and attitude) has covered, and which were also seen ≥ 15° off nadir:

$$\text{coverage}=\frac{\#\{\text{cells seen}\}}{\#\{\text{cells in the area}\}},\qquad
L_{path}=\sum_k\lVert\mathbf w_{k+1}-\mathbf w_k\rVert$$

**[Illustrative, computed by calling `footprint_half_extents` and `lawnmower`]** Altitude 5 m ⇒ $h=4.1$ m,
$(\ell_f,\ell_r)=(2.92,\,3.89)$ m. For a 12 m (north) × 8 m (east) area: lane spacing 3.89 m, 3 lanes at east
$=1.33,\,4.00,\,6.67$ m, six waypoints, path length 23.8 m ≈ 12 s at the 2.0 m/s search speed (plus turns).

**Caveats.** Coverage is *geometric*: a cell inside the footprint counts as seen even if a wall or slab hides it, so
100 % coverage does not mean an interior was seen. Cells reported unreachable are skipped; an early return (time
budget, battery, lost perception, first confirmed survivor) ends with coverage < 1, which the report records (e.g. 88 %
at a 35 s budget, 48 % in the "first found" mission).

---

## L. Control

**Physics [Background, checked against the simulated airframe].** In NED a multirotor of mass $m$ obeys

$$m\ddot{\mathbf p}=m g\,\mathbf e_z-T\,\mathbf R\,\mathbf e_z,\qquad T=\sum_{i=1}^4k_f\omega_i^2$$

where thrust $T$ acts along the body $-z$ axis and $\mathbf R$ is the body-to-local rotation. Hover: $\mathbf R=\mathbf I$,
$T=mg$. Tilting by $\theta$ at constant altitude gives horizontal acceleration $a_h=g\tan\theta$, which is why horizontal
motion is *slower to command* than vertical (attitude must change first — the 0.493 s lag $\tau$ above).
**[Illustrative, from the PX4 `x500_base` SDF]** $m\approx2.0+4(0.016)=2.06$ kg (without the project's cameras),
$k_f=8.54858\times10^{-6}$, $\omega_{\max}=1000$ rad/s ⇒ $T_{\max}=4\cdot8.55=34.2$ N vs weight 20.2 N:
thrust-to-weight ≈ 1.69; hover needs ≈ 769 rad/s per rotor (59 % of $T_{\max}$).

**Who controls what.**

| Layer | Runs in | What it does |
|---|---|---|
| Planning/proposals | project nodes | goals, avoidance setpoints (≤ 20 Hz) |
| `flight_controller` | project node, 20 Hz timer | validates, caps speed, publishes **position** setpoints `OffboardControlMode.position=true`; owns the mission state machine |
| PX4 position loop | PX4 | $\mathbf v_{sp}=K_p(\mathbf p_{sp}-\mathbf p)$, $K_p=\text{MPC\_XY\_P}=0.95$ |
| PX4 velocity → attitude → rate loops | PX4 | PID on velocity gives a thrust/attitude setpoint; P on attitude; PID on body rate |
| Control allocation, motors | PX4 / simulator | thrust & torque → rotor speeds |

**The application code is a supervisory, set-point-based controller. It does not implement PID.** It issues position
setpoints open-loop with respect to the *dynamics*, and closes a loop only *logically*: state transitions test PX4
feedback (altitude within 0.25 m of target, distance to waypoint < 0.5 m, arming/mode flags, telemetry age). The only
numeric feedback gain in project code is the speed-cap geometry (÷ 0.95). The PID law below is **[Background]** and
describes PX4's inner loops:

$$u(t)=K_P\,e(t)+K_I\!\int_0^t e(\lambda)\,d\lambda+K_D\frac{de(t)}{dt}$$

$e$ = setpoint − measurement. $K_P$ pushes in proportion to the present error; $K_I$ accumulates error to remove
steady offsets (e.g. a constant wind or payload); $K_D$ reacts to the error's rate of change and damps overshoot.
Saturation (e.g. the 11.08 m/s² acceleration limit in the fitted model, 12 m/s horizontal speed limit) and delays
(lag $\tau$, 20 Hz setpoint rate, status at ≈ 2 Hz of simulation time) mean a commanded setpoint is **not** the resulting
motion — the reason the planner rolls candidates out through a response model before proposing them.

**Measured result** (Phase 0, 5 hover flights): altitude RMSE 0.070–0.096 m, drift 0.042–0.059 m, landing offset
0.015–0.059 m — properties of PX4's controllers in simulation, not of project code.

---

## M. Safety logic

**[Implemented]** `flight_safety.py`, evaluated on every PX4 position sample above 0.5 m altitude:

$$\begin{aligned}
\text{impact:}&\ \lVert\mathbf a\rVert>30\ \mathrm{m/s^2}\\
\text{position jump:}&\ \lVert\Delta\mathbf p-\bar{\mathbf v}\Delta t\rVert>0.5+\tfrac12(30)\Delta t^2\ \mathrm{m}\\
\text{velocity jump:}&\ \lVert\Delta\mathbf v\rVert>3.0+30\,\Delta t\ \mathrm{m/s}\\
\text{speed envelope:}&\ v_{xy}>6.5\ \text{or}\ |v_z|>3.0\ \mathrm{m/s}
\end{aligned}$$

plus an estimator-reset counter change or any non-finite value, plus altitude excursion > 1.0 m in cruise states.
A violation latches `FAILSAFE` and requests PX4 `NAV_LAND`; stale telemetry (position > `position_timeout_s`,
default 1.0 s; status > `status_timeout_s`, default 1.0 s, 3.0 s in the urban runs) does the same. **Why a fallback:**
every layer above can be wrong (stale perception, a SLAM jump, a mis-matched scan); the fallback needs no perception
and is the only layer that is guaranteed to act. Evidence it matters: a stock waypoint route hit the roof 8.66 s after
arming; the monitor later caught the same impact live at 8.6 s and landed at 8.7 s.

The state-machine diagram is in the README (§8 M).

---

## N. Evaluation metrics

All **offline**, using simulator ground truth (scripts named in the README); none runs in flight.

| Group | Metric | Definition |
|---|---|---|
| Detector | precision, recall, F1 | $P=\frac{TP}{TP+FP}$, $R=\frac{TP}{TP+FN}$, $F_1=\frac{2PR}{P+R}$ |
| Debris tracking | position RMSE | $\sqrt{\operatorname{mean}(e^2)}$, $e$ = distance to the true box *surface* (also to centre) |
| | ID switches | CLEAR-MOT convention (pairing kept while within 0.6 m) |
| Prediction | RMSE vs horizon | as above, per horizon |
| Risk | precision/recall of threat flag; lead time | lead = first WARNING/CRITICAL frame → true closest approach |
| Localisation | horizontal error | $\lVert\hat{\mathbf p}_{ne}-\mathbf p^{true}_{ne}\rVert$ (m) at first report and final |
| Mission | completion, coverage, search time, objective met | from `mission_report.json`; `objective_met` = landed safely **and** the search ended because the area was covered or a survivor was confirmed |
| Safety | box collision | box surface within 0.40 m of the vehicle centre (near miss ≤ 0.70 m); closest box distance; avoidance manoeuvres; failsafes; PX4-log checks (`verify_ulog.py`) |

**Small samples.** 12 completions in 12 flights give a **one-sided 95 % lower bound** on the completion rate of
$0.05^{1/12}\approx0.78$ — "12/12" is not "100 %".
