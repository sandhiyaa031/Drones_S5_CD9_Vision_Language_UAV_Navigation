# Mission manager (Phase 10)

Status: implemented, unit-tested (53 tests) and flown in seven live Gazebo
runs on 2026-10-06. Evidence: `results/phase10/` (one folder per run under
`live/`, each with `mission_report.json`; score files beside them).

All runs were on the Intel GPU **without the VLM** (the NVIDIA driver was not
available). No entry was ever VLM-confirmed, so every survivor in the
reports is an "unverified candidate", and the `first_confirmed` stop policy
is covered by unit tests only.

## What it does

One node ties the existing functions into a mission:

```
preflight checks ─► START ─► takeoff ─► search ─► return ─► land ─► report
       │                                  │
       └─► REFUSED (nothing flies)        └─► ends early on: time budget, battery,
                                              perception lost, debris protection lost,
                                              first confirmed survivor (policy)
```

It decides three things only: whether the flight may start, when the search
ends, and what goes into the report. Flying, avoiding debris, searching and
locating stay with the nodes that already do them.

| | |
|---|---|
| Logic | `src/uav_autonomy/uav_autonomy/mission_logic.py` (no ROS) |
| Node | `mission_manager` (`mission_manager_node.py`) |
| Controller rules | `mission_interface.py`; `mission_managed` mode in `flight_controller.py` |
| Messages | `uav_interfaces/MissionCommand`, `MissionStatus` |
| Mission files | `missions/*.json` |
| Inputs | `/uav/flight_status`, `/perception/survivor/detections`, `/perception/survivor/locations`, `/planning/search/goal`, `/perception/debris/avoidance_command`, `/fmu/out/battery_status_v1`, `/fmu/out/vehicle_local_position_v1` |
| Outputs | `/mission/command`, `/mission/status` (2 Hz), `/mission/report` (JSON, once), report file |
| Tests | `test/test_mission_logic.py` |
| Run / score | `scripts/mission/run_mission.sh`, `scripts/mission/evaluate_mission.py` (the only place ground truth is used) |

## Mission file

```json
{
 "name": "search_yard",
 "search_area": {"north": [-4, 12], "east": [-4, 4]},
 "altitude_m": 5.0,
 "time_budget_s": 240,
 "stop_policy": "area_covered"
}
```

Optional: `stop_policy: "first_confirmed"`, `require_vlm`,
`require_debris_protection`, `battery_return_fraction` (default 0.25),
`perception_timeout_s` (3), `report_min_observations` (15). The area is in
metres relative to the takeoff point. A mission with an unknown field, an
area further than 25 m from takeoff, an altitude outside 2–15 m or a budget
outside 10–300 s is refused. The run script passes the same area and
altitude to the planner and the controller; the mission manager does not
yet send them itself.

## Authority

`flight_controller` remains the only node that commands PX4. With
`mission_managed:=true` (off by default) it

* waits on the ground, without arming, until a valid `START` arrives;
* accepts `RETURN_HOME` in SEARCH/HOLD/waypoint states and `LAND_NOW` also
  while returning; never in TAKEOFF, LANDING or FAILSAFE;
* ignores commands older than 2 s and acts once per sequence number.

The safety monitor, PX4's own failsafes and debris avoidance are unchanged
and take precedence: a controller FAILSAFE ends the mission as FAILSAFE and
the manager sends nothing further.

## Preflight checks

All must hold for 2 s; otherwise the flight is refused after 30 s and the
report says why.

| Check | Passes when |
|---|---|
| mission | the mission file is valid |
| flight_controller | status fresh and waiting on the ground |
| survivor_perception | detection stream younger than 2 s |
| survivor_localization | location stream younger than 2 s |
| search_planner | a publisher exists on the goal topic |
| vlm (if required) | the perception node reports the VLM available |
| debris_avoidance (if required) | avoidance command stream younger than 2 s |
| battery (if known) | remaining ≥ return level + 0.10 |

## Report

`mission_report.json`: outcome, `objective_met` (landed safely **and** the
search ended because the area was covered or a survivor was confirmed), why
the search ended, phase timeline, duration, coverage, inspections,
avoidance manoeuvres, lowest battery level, and one entry per located
candidate with position (local frame, offset from takeoff,
latitude/longitude), horizontal 1σ and one of four labels:

| Label | Meaning |
|---|---|
| `SURVIVOR_CONFIRMED` | confirmed by the VLM at least once |
| `UNVERIFIED_CANDIDATE` | detected and located, no VLM verdict |
| `REJECTED_BY_VLM` | the VLM judged it not a survivor |
| `LOW_CONFIDENCE` | fewer than 15 camera frames |

Without a VLM the report says so and confirms nothing.

## Results

World: two survivors and several orange distractor objects; area 16 × 8 m.

| Run | What was tested | Result |
|---|---|---|
| `search_yard_multi` | nominal mission | complete, objective met; search 122.5 s, coverage 100 %; both survivors in the report (0.16 m and 0.03 m off), plus 4 distractor entries, all "unverified candidate" |
| `search_yard_protected_multi` | debris protection required, 3 boxes dropped during the search | complete, objective met; 2 avoidance manoeuvres; both survivors reported (0.02 m, 0.06 m) |
| `search_yard_short_budget_multi` | 35 s time budget | RETURN_HOME sent at 35.5 s, landed; coverage 88 %; one survivor reported, the other only as low-confidence (10 frames) |
| `search_yard_multi_perception_loss` | survivor perception stopped mid-search | RETURN_HOME 3.4 s after the last frame, landed; one survivor reported |
| `search_yard_battery_multi` | return level set to 0.85 | RETURN_HOME 3 s into the search (battery 0.85), landed |
| `search_yard_vlm_required_multi` | mission requires a VLM, none available | **refused**; the vehicle never armed |
| `search_yard_multi_no_planner` | search planner not started | **refused**; the vehicle never armed |

* Every flight that took off landed and passed the PX4-log checks, except
  the battery run, whose "altitude hold ≥ 15 s" check fails because the
  mission deliberately left altitude after 3 s.
* No entry was labelled confirmed in any run; no survivor that was seen for
  at least 15 frames is missing from a report.

## Failures and limits

* **Distractors appear in the report** as unverified candidates (4 of 6
  entries in the nominal run). Only a VLM verdict can separate them; that
  was not available. A ghost entry of the roof box (1.34 m off, 18 frames,
  1σ 0.51 m) also appeared once.
* **Not validated with a live VLM**: `SURVIVOR_CONFIRMED`,
  `REJECTED_BY_VLM` and the `first_confirmed` policy are unit-tested only.
* **The battery case uses PX4's simulated battery**, which drains to 50 %
  within the flight; the threshold was raised to 0.85 to trigger it. PX4
  parameters were not changed.
* **`COMPLETE` means landed safely**, not that the job was done; read
  `objective_met` and `search_end_reason`.
* The mission is one rectangle at one altitude; no multi-area missions, no
  resume after an early return, no operator abort command (the message
  exists: `LAND_NOW`, exercised by unit tests only).
* If the flight controller's status stops arriving in flight, the manager
  can neither command nor conclude; it keeps its phase. The controller's
  own guards (telemetry, planner timeout) still land the vehicle.
* The mission manager process is a single point for the report: if it dies
  the vehicle still completes its flight (the controller ends the search by
  its own rules) but no report is written. Not tested live.
* Simulation only.

## Reproduce

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 scripts/survivor/make_worlds.py && scripts/mission/run_mission.sh missions/search_yard.json multi && python3 scripts/mission/evaluate_mission.py results/phase10/live/search_yard_multi
```

With falling debris and required protection:

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && DEBRIS=1 LIGHT=1 scripts/mission/run_mission.sh missions/search_yard_protected.json multi
```

Fault injection: `KILL_PERCEPTION_AT=40` (stop survivor perception after
40 s), `NO_PLANNER=1`. Add `HEADLESS=0` to watch in Gazebo. Unit tests:

```bash
cd ~/Desktop/drones/ros2_ws/src/uav_autonomy && python3 -m pytest test/test_mission_logic.py -q
```
