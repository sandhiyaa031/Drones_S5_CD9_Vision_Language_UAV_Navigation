# Phase 6: autonomous debris avoidance

Status: implemented and flown live in Gazebo. Results, including the cases
it does not solve, are in `results/phase6/README.md`.

## 1. Architecture

```
depth camera ─► detector ─► tracker ─► predictor ─► risk ─┐
                                                          ▼
/fmu/out/vehicle_odometry ───────────────────────► debris_avoidance
/uav/flight_status (from flight_controller) ─────►   (planner)
                                                          │ /perception/debris/avoidance_command
                                                          ▼   (uav_interfaces/AvoidanceCommand: a proposal)
                                                   flight_controller ──► /fmu/in/*  ──► PX4
                                                   (validates; sole PX4 authority;
                                                    flight_safety monitor stays active)
```

* `flight_controller` is still the only node that publishes to `/fmu/in/*`.
  The planner publishes a high-level proposal and nothing else.
* The planner never reads Gazebo ground truth. Its inputs are the risk,
  prediction and track topics, PX4 odometry and the controller's status.
* PX4 safety parameters, preflight checks, arming, takeoff and landing
  logic are untouched. `flight_safety.py` is unchanged and remains active
  during manoeuvres (impact, estimator-jump and speed checks).

| File | Role |
|---|---|
| `uav_autonomy/debris_avoidance.py` | planner core, no ROS: candidates, response model, constraints, cost, state machine |
| `uav_autonomy/debris_avoidance_node.py` | ROS wrapper: inputs, command, markers, JSON debug |
| `uav_autonomy/avoidance_interface.py` | rules the controller applies to a command, no ROS |
| `uav_autonomy/flight_controller.py` | additive: command subscription, validation, setpoint override, status |
| `uav_interfaces/msg/AvoidanceCommand.msg`, `FlightStatus.msg` | interface |

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/perception/debris/avoidance_command` | `uav_interfaces/AvoidanceCommand` | planner → controller, ~50 Hz |
| `/uav/flight_status` | `uav_interfaces/FlightStatus` | controller → planner, 20 Hz |
| `/perception/debris/avoidance_markers` | `visualization_msgs/MarkerArray` | safety volume, nominal setpoint, every candidate path, selection |
| `/perception/debris/avoidance_debug` | `std_msgs/String` (JSON) | one record per planning cycle |

## 2. What changed in `flight_controller.py`

Additive only; with no command on the topic the controller behaves exactly
as before (the Phase 0–5 flights and tests are unchanged).

* Subscribes to the command; each 50 ms tick it is validated
  (`avoidance_interface.validate_command`):

  | Rule | Value | Why |
  |---|---|---|
  | flight state | TAKEOFF (once climbing), WAYPOINT_1/2, RETURN_HOME, HOLD | never while arming, landing or failed |
  | command age | ≤ 0.5 s (arrival clock) | **command timeout**: planner silent → mission setpoint back in force |
  | target finite | – | reject NaN/inf |
  | target distance from vehicle | ≤ 6.0 m | bounds the step PX4 is asked to make (4 m candidates + drift) |
  | target altitude above takeoff | 1.5 m … 15 m | never into the ground; ceiling |

* While a command is accepted the position setpoint sent to PX4 is the
  avoidance target. **The mission target (`target_x/y/z`) is never
  modified**, so when the command ends the vehicle returns to it and the
  mission continues from where it was.
* While avoiding or recovering, the mission step is paused: the HOLD timer
  and the waypoint/takeoff timeouts do not run and the HOLD altitude
  tolerance and cruise altitude-excursion checks are not evaluated (the
  displacement is intentional). Arming/Offboard loss still fails safe.
  Recovery ends when the vehicle is within 0.5 m horizontally and 0.35 m
  vertically of the mission setpoint.
* A refused command is logged once with the reason and counted in
  `FlightStatus.avoidance_rejected`; the planner then drops that candidate
  and picks another.
* Publishes `FlightStatus` (state, mission target, commanded setpoint,
  takeoff reference, avoidance flags and counters).
* New optional parameters, defaults = previous constants:
  `target_altitude` (3.0), `hold_seconds` (30), `waypoint_1_north/east`,
  `waypoint_2_north/east`, and `arrival_max_speed` (0 = off; when set, a
  waypoint only counts as reached below that horizontal speed).

## 3. Planner

### 3.1 Risk states → action

| Highest risk state | Planner |
|---|---|
| SAFE, WATCH | NORMAL_FLIGHT; mission continues |
| POTENTIAL_THREAT (unconfirmed fast object) | PREPARE_AVOIDANCE: candidates are evaluated and the best is held ready; **no manoeuvre**. An optional precautionary step (`precautionary_on_potential`, default **off**) moves at most 1.0 m towards the prepared candidate and is flagged `precautionary` |
| WARNING | AVOIDING (if the mission setpoint is not a feasible candidate) |
| CRITICAL | AVOIDING immediately |

### 3.2 Candidates

Position setpoints, regenerated every cycle around the present position:
`hold` (the mission setpoint), eight horizontal directions (N, S, E, W and
diagonals) at 2.5 m and 4.0 m, `climb` 2.0 m, `descend` 1.0 m — 19 in all —
plus the setpoint currently being flown. Lists and distances are parameters.

The setpoint is deliberately further away than the displacement needed: PX4's
position controller commands velocity in proportion to the position error, so
a far setpoint gives a brisk start, and the manoeuvre is ended (mission
setpoint restored) as soon as the threat has passed.

### 3.3 Vehicle response model

Each candidate is evaluated on the trajectory the vehicle would actually fly,
from a reduced model of PX4's cascaded position controller:

```
v_cmd = clamp(kp (setpoint − p), v_max)
a_cmd = clamp(kv (v_cmd − v), a_max)      per axis
da/dt = (a_cmd − a) / tau                 attitude lag
```

integrated at 20 ms over 2.0 s. The acceleration starts from what the
setpoint in force until now commands, so a vehicle already cruising or
returning is modelled correctly.

Constants are **measured**, not assumed: `scripts/avoidance/step_command.py`
flew 12 setpoint steps (4 m, 2.5 m, diagonal, climb, descend, and the returns
started while still moving) at a 5 m hover, and
`scripts/avoidance/fit_response.py` fitted the model
(`results/phase6/calibration_step/`).

| Parameter | Value | |
|---|---|---|
| `tau_xy_s`, `kp_xy`, `kv_xy`, `a_xy_max` | 0.336 s, 0.819 1/s, 3.373 1/s, 6.46 m/s² | horizontal |
| `tau_z_s`, `kp_z`, `kv_z`, `a_z_max` | 0.02 s, 1.181 1/s, 12.0 1/s, 6.75 m/s² | vertical |
| `v_xy_max`, `v_up_max`, `v_down_max` | 12, 3.0, 1.5 m/s | PX4 defaults (MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP/DN) |
| `model_error_m` | 0.17 m | largest model error over 1.5 s across the 12 steps (0.165 m) |

Measured response to a 4 m step from hover: 0.13 m after 0.4 s, 0.44 m after
0.6 s, 0.94 m after 0.8 s, 1.56 m after 1.0 s, 3.04 m after 1.5 s. This is the
physical limit of the manoeuvre: clearing a ~1.2 m requirement takes ~0.9 s.

The first version of the model (dead time, then constant acceleration limit)
matched steps from rest but was up to 1.2 m wrong for a moving vehicle; in the
transit scenario this let the planner accept "carry on" too early
(`results/phase6/superseded/`). It was replaced by the model above.

### 3.4 Hard constraints

A candidate is rejected if, anywhere on its simulated trajectory:

| Constraint | Value | Source |
|---|---|---|
| altitude above takeoff | 1.5 m … 12 m | ground clearance; depth sensor ceiling |
| horizontal speed | ≤ 5.0 m/s | below the flight-safety limit (6.5 m/s) |
| vertical speed | ≤ 2.5 m/s | below the flight-safety limit (3.0 m/s) |
| structure exclusion | see 3.5 | known building geometry |
| separation from **every** relevant track | ≥ safety radius + uncertainty margin + model error | Phase 5 radii and margin; 3.3 |

"Relevant" = every track in the current risk frame (all risk states, also
SAFE ones: a manoeuvre must not fly into another object) updated within
0.35 s, plus unconfirmed objects flagged POTENTIAL_THREAT (ballistic path).
Debris paths are the Phase 4 predictions (constant acceleration between
predicted states, nothing extrapolated beyond 2 s); a path below the takeoff
plane is ignored. Safety radius and uncertainty margin are exactly Phase 5's
(`contact = 0.40 + 0.71·size`, `safety = contact + 0.30`,
`margin = min(σ_r, 0.10 + 0.40·t)`). Detector, tracker, predictor and risk
mathematics are unchanged.

### 3.5 Structure exclusion

Bounce and deflection off the structure are not predicted (by decision); the
UAV is instead kept away from where debris lands. Zones are axis-aligned
boxes in the takeoff frame, taken from the **mission's building plan** (the
same dimensions as the world file, entered as configuration; nothing is read
from the simulator at run time):

| Zone | North | East | Altitude |
|---|---|---|---|
| keep-out: building footprint up to 0.8 m above the highest roof slab (3.2 m) | −4.5 … 4.5 | −4.5 … 4.5 | ≤ 4.0 m |
| corridor: roof opening shrunk by UAV radius + 0.2 m | −0.4 … 0.4 | −0.5 … 1.4 | ≤ 4.0 m |

A candidate that enters a keep-out zone outside a corridor, or whose setpoint
lies in one, is rejected. A vehicle already inside (climbing out through the
opening) is not refused all options for that alone.

### 3.6 Cost

Among feasible candidates the lowest

```
J = w_risk·risk + w_distance·deviation + w_energy·effort
    + w_smooth·change + w_altitude·altitude_change
```

| Term | Definition | Weight |
|---|---|---|
| risk | `clip(1 − clearance / 1.5 m, 0, 1)`, clearance = smallest (separation − required) over all tracks | 10 |
| deviation | distance of the setpoint from the nominal path (segment from where the manoeuvre began to the mission setpoint) [m] | 1.0 |
| effort | distance of the setpoint from the present position [m] | 0.5 |
| change | distance from the setpoint currently flown [m] (hysteresis) | 0.5 |
| altitude_change | \|setpoint altitude − mission altitude\| [m] | 0.5 |

The weights were set once from their meaning — a full unit of risk outweighs
any displacement the planner can command (≤ 4 m ≈ 6 cost units), and lateral
motion is preferred to altitude changes at equal clearance — and were not
tuned on the scenarios. One consequence, visible in the results: with risk
weighted this heavily the planner usually picks a 4 m setpoint over a 2.5 m
one, so deviations are ~3 m where ~1.5 m would have been enough.

### 3.7 State machine

```
NORMAL_FLIGHT ──POTENTIAL_THREAT──► PREPARE_AVOIDANCE ──0.6 s without threat──► NORMAL_FLIGHT
      │  WARNING/CRITICAL                 │ WARNING/CRITICAL
      ▼                                   ▼
   AVOIDING ◄──── feasible again ──── AVOIDANCE_FAILSAFE
      │  no WARNING/CRITICAL and the mission setpoint is a feasible candidate
      ▼
   CLEARING ──0.3 s──► RESUME_MISSION ──within 0.5 m of mission setpoint──► NORMAL_FLIGHT
      ▲ new threat: back to AVOIDING from CLEARING or RESUME_MISSION
```

Every transition is logged with sensor time and reason
(`debris_avoidance.log`, `avoidance.jsonl`). Returning is itself checked: the
manoeuvre continues while flying back to the mission setpoint would violate
a constraint.

### 3.8 Fallbacks

| Condition | Behaviour |
|---|---|
| no candidate satisfies the safety constraint | AVOIDANCE_FAILSAFE, `failsafe = true`: fly the candidate with the largest clearance among those that break no altitude, speed or structure limit (best effort; kept unless another is better by 0.2 m). The mission setpoint is left only for a candidate that gains at least 0.2 m: when nothing helps, nothing is commanded |
| not even that (boxed in) | AVOIDANCE_FAILSAFE, no command: mission setpoint stays in force |
| perception stale (> 0.6 s since the last risk frame, sensor time) | in a manoeuvre: keep the last avoidance setpoint, at most 4 s, then resume; otherwise no manoeuvre is possible; flagged `failsafe` |
| odometry stale (> 0.5 s arrival gap, or > 0.2 s behind the newest depth frame) | command released (mission setpoint in force); the controller's own 1.0 s telemetry guard lands the vehicle if it persists |
| command timeout (planner silent > 0.5 s) | controller drops the command, logs it, flies the mission setpoint |
| controller refuses a command | planner drops that candidate and re-plans |
| collision despite everything | unchanged from before: `flight_safety` impact detection → FAILSAFE → PX4 AUTO LAND |

### 3.9 Other parameters

| Parameter | Value | Meaning |
|---|---|---|
| `horizon_s`, `model_dt_s` | 2.0 s, 0.02 s | planning horizon (= prediction horizon), integration step |
| `risk_clear_scale_m` | 1.5 m | clearance at which the risk term reaches zero |
| `clear_hold_s` | 0.30 s | time the situation must stay clear before resuming |
| `prepare_timeout_s` | 0.6 s | PREPARE → NORMAL without a threat |
| `max_manoeuvre_s` | 4.0 s | longest time on stale perception |
| `resume_tolerance_m` | 0.5 m | mission setpoint regained |
| `max_track_age_s` | 0.35 s | older tracks are not planned against (Phase 4 limit) |
| `rate_hz` | 30 | planning timer (plus one cycle per risk frame) |
| `switch_cost_margin` | 3.0 | a setpoint being flown is kept while feasible unless another candidate is cheaper by this much (commitment; without it the planner alternated between the 2.5 m and 4 m setpoints of one direction) |
| `nominal_path_trigger` | false | optional, see 3.10 |
| `enabled` | true | false = baseline: plans and logs, never commands |

### 3.10 Optional mission-path trigger (default off)

The Phase 5 risk estimator assumes the UAV keeps its present velocity. On a
transit leg the UAV is accelerating, so an object aimed at where it *will*
be is rated SAFE until the UAV has sped up, and the WARNING arrives late.
With `nominal_path_trigger:=true` the planner also starts a manoeuvre when
its own model of the flight to the mission setpoint enters the safety volume
of a **confirmed** track — the WARNING condition evaluated on the planned
motion instead of a constant-velocity one. Unconfirmed objects still only
prepare. It is off by default because it lets a manoeuvre start while the
risk state is SAFE or WATCH, which departs from the specified mapping; it is
evaluated separately in the results.

## 3.11 Known limits (measured, `results/phase6/README.md`)

* Needs about 0.9 s of lead time. Every collision in the final flights had
  ≤ 0.70 s; every threat with ≥ 0.94 s was avoided.
* Laterally thrown debris with under ~1 s of flight is confirmed too late
  (collision 4/4 for the 0.93 s Phase 5 case).
* Transit at 4.4 m/s is not solved: the risk estimator's constant-velocity
  assumption delays the alert while the UAV accelerates, the response model
  is calibrated to 3 m/s, and PX4's position estimate leads the truth by
  ~0.09 s × velocity.
* A depth-stream gap just after a release removes the lead time.
* Inside the roof opening there is no escape: the planner reports
  AVOIDANCE_FAILSAFE and commands nothing.
* Deviations are ~3 m where ~1.5 m would do (cost weights not tuned).
* A crashed vehicle is landed by PX4 AUTO LAND and may stay armed
  (unchanged, by earlier decision).

## 3.12 Phase 6.1 additions

* **Response model**: acceleration state initialised from PX4's measured
  acceleration (`vehicle_local_position` ax/ay/az, mean of the last 0.06 s);
  constants refitted on 133 start states including mid-motion ones:
  `tau_xy_s` 0.493, `kp_xy` 0.827, `kv_xy` 3.203, `a_xy_max` 11.08,
  `model_error_m` 0.25 (vertical unchanged). The values in 3.3 are the
  Phase 6 ones. Fit: `scripts/avoidance/fit_response_states.py`.
* **Perception health** from the age of the newest depth-derived risk frame
  in sensor time: FRESH ≤ `health_fresh_s` 0.15 s, DEGRADED ≤
  `health_stale_s` 0.50 s, else STALE. Stale is not "no debris": the last
  risk frame stays in use, each debris path advanced in time, for
  `risk_hold_s` 1.0 s. Beyond that the planner enters **DEFENSIVE**: no
  manoeuvre is invented (a manoeuvre in progress keeps its setpoint up to
  `max_manoeuvre_s`), the mission continues under a speed cap, and the state
  ends when depth frames return. It does not hover or land: neither makes the
  vehicle safer against an object it cannot see.
* **Speed caps** requested by the planner on mission motion:
  `degraded_speed_limit` 2.0 m/s, `stale_speed_limit` 1.0 m/s (STALE and
  DEFENSIVE), `prepare_speed_limit` 2.0 m/s (POTENTIAL_THREAT). 2.0 m/s is
  inside the search-speed range that passed every test; 1.0 m/s stops within
  ~0.2 m. Avoidance setpoints are never capped. The controller applies the
  smallest of the requested cap and its own `transit_speed_limit` parameter
  by keeping the setpoint sent to PX4 no further than cap / MPC_XY_P (0.95)
  from the vehicle (`avoidance_interface.limit_setpoint`); a requested cap is
  never below 0.5 m/s. `FlightStatus.speed_limit` reports the cap in force and
  the planner models the capped mission setpoint.
* **POTENTIAL_THREAT** still never triggers a manoeuvre. PREPARE_AVOIDANCE
  now also requests the cap (defensive preparation).
* The mission setpoint ("hold") is no longer rejected by the avoidance speed
  constraint; mission speed is the controller's matter.
* Risk node option `uav_motion_model` (see `docs/debris_risk.md`).
* Measured outcome and remaining limits: `results/phase6_1/README.md`.

## 4. Time and frames

All positions are PX4 local NED. Debris states carry the depth-frame time;
the UAV state carries `vehicle_odometry.timestamp_sample`; with
`UXRCE_DDS_SYNCT=0` both are simulation time. The planner advances each
debris path by (UAV state time − track time) before comparing. Perception age
is measured in that time base. Only the two liveness checks (odometry
arrival, command timeout) use the arrival clock, because they detect a
process that stopped talking.

## 5. Running

```bash
# one scenario, with and without avoidance (identical releases)
scripts/avoidance/run_scenario.sh C_direct baseline
scripts/avoidance/run_scenario.sh C_direct avoid
# everything
scripts/avoidance/run_all.sh
# scoring against ground truth, comparison table, plots
PYTHONPATH=src/uav_autonomy python3 scripts/avoidance/evaluate_avoidance.py results/phase6/avoid/C_direct --baseline-root results/phase6/baseline
PYTHONPATH=src/uav_autonomy python3 scripts/avoidance/compare.py
PYTHONPATH=src/uav_autonomy python3 scripts/avoidance/plot_avoidance.py results/phase6/avoid/C_direct
# any flight with the planner
PERCEPTION=1 AVOIDANCE=on NVIDIA_OFFLOAD=1 CAMERA_BRIDGE=1 \
  CONTROLLER_ARGS="-p target_altitude:=5.0" VERIFY_ARGS="--target-alt 5.0" \
  scripts/run_flight_trial.sh results/phase6/my_run hover
```

`AVOIDANCE=baseline` starts the planner disabled. Scenario releases are made
by `scripts/sensor_study/spawn_study.py`, which (as the scenario generator
only) may read the UAV's simulator pose to aim a box at it.

## 6. Tests

`test/test_debris_avoidance.py` (67 tests): response model against the
measured steps, frames and timestamps, structure exclusion, candidate
rejection and selection, cost terms, no/one/multiple/crossing threats, moving
UAV, near miss, POTENTIAL/WARNING/CRITICAL handling, full cycle and return to
path, repeated threats, new threat while resuming, no feasible candidate,
boxed in, stale perception, stale odometry, refused command, and the
controller-side rules including the command timeout.
