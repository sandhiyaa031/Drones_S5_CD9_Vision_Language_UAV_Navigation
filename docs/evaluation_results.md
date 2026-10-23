# Evaluation results — detail and provenance

Everything here is **simulation only** (PX4 SITL + Gazebo Harmonic, or MuJoCo where stated). Nothing has flown on
real hardware. The main [README](../README.md) §11 summarises this file.

## How to read the evidence column

| Tag | Meaning |
|---|---|
| **[raw]** | The run folders / result JSON named are present in this checkout and were re-read while writing this report. |
| **[doc]** | The number is quoted from a project document (`results/final/*.md`, `docs/*.md`). The run folders it came from are **not present** in this checkout, so it was not re-derived. |
| **[run now]** | Re-executed during the audit (2026-10-23). |

Run folders under `results/**` are mostly git-ignored for size (`.ulg`, frames, `poses.csv`, `.jsonl`); the
mission reports, `verification.json` and score JSON are what remain.

---

## 1. Unit tests — [run now]

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
cd src/uav_autonomy && python3 -m pytest test -q
```

**552 passed, 6 skipped, 2 failed.** The two failures are the style-lint tests (`test_flake8`: 281 findings;
`test_pep257`); both were failing before this work. Without sourcing the workspace (`install/setup.bash`) nine
`test_survivor_perception.py` tests error with `No module named 'uav_interfaces'` — an environment issue, not a
code defect. `test_copyright.py` also needs the ROS environment.

| Test file | Tests | Covers |
|---|---|---|
| `test_survivor_perception.py` | 92 | detector, temporal confirmation, VLM worker, node (incl. "no ground-truth input", "callback never waits for the VLM") |
| `test_debris_avoidance.py` | 75 | planner, constraints, state machine, controller rules |
| `test_mission_language.py` | 61 | interpreter checks, refusals |
| `test_mission_logic.py` | 57 | preflight, stop policies, report |
| `test_navigation.py` | 43 | A*, inflation, unreachable, breadcrumbs, localisation loss |
| `test_search_planning.py` | 43 | footprint, lanes, coverage, inspection |
| `test_debris_tracking.py` | 42 | Kalman filter, association, life cycle |
| `test_survivor_localization.py` | 38 | ray, EKF, merge/association |
| `test_debris_risk.py` | 35 | closest approach, states |
| `test_flight_safety.py` | 26 | impact/jump/envelope checks, log replays |
| `test_debris_prediction.py` / `test_debris_detection.py` | 24 / 21 | predictor, depth detector |

`simulation/mujoco/tests`: 12 tests (project record, 2026-10-20/21) — not re-run for this report.

---

## 2. Debris chain and survivor perception, Phases 0–7 — [doc] (+ figures [raw])

World `collapsed_building_rescue`, NVIDIA GPU, real-time factor 0.9986 (source: `results/final/MASTER_RESULTS.md`,
status date 2026-10-06). The raw run folders (`results/phase0…phase5`, `phase6_1`, `safety`, `sensor_study`,
`time_sync`, `demo/*.mp4`) are **absent**; the figures in `results/final/figures/` are present.

| Stage | Result |
|---|---|
| Hover (5 flights) | all PX4-log checks pass; altitude RMSE 0.070–0.096 m; landing offset 0.015–0.059 m |
| Depth detection (4 flights, 86 boxes) | 86/86 detected; frame recall 98.7 % (1713/1736); 0 unexplained of 1852; surface error p95 ≤ 0.160 m |
| Tracking (7 flights) | 0 ID switches; velocity RMSE 0.07–0.43 m/s; confirmation delay median 0.07–0.18 s |
| Prediction (6 flights, 2346 predictions) | RMSE 0.028 m @0.1 s … 0.110 m @1.0 s … 0.414 m @2.0 s (constant velocity 4.44 m @1 s) |
| Risk (281 objects offline) | free-fall precision 0.61, recall 0.954; 10/10 collisions CRITICAL; lead time median 0.80 s |
| Avoidance (14 scenarios, with/without) | runs with a collision 11/14 → 5/14; 15/15 direct hits on a hovering UAV avoided (min separation 0.91 m); cruise 2.9–4.4 m/s: 0 collisions in 8 flights |
| Survivor candidate detector (11 scenarios) | TP 2016, FN 0, FP 773 → precision 0.72, recall 1.00 (all FP from one designed distractor cluster) |
| VLM, held-out 175 crops | 0 of 100 non-survivors confirmed; 24 of 45 clear survivors confirmed (oblique 24/32, straight-down 0/13) |

Failure cases stated in the same sources: avoidance needs ≈ 0.9 s of warning; hard-acceleration transit unsolved;
lateral throws with ≤ 1 s of flight not avoided; no escape manoeuvre inside the roof opening; a box struck the UAV in
one Phase 2 run before avoidance existed; one direct hit was lost to a 0.76 s depth gap.

---

## 3. Localisation, search, mission manager, language — [raw]

| Phase | Evidence folder | Result |
|---|---|---|
| 8 localisation | `results/phase8/` (8 live flights) | 8/8 survivor instances located; final error 0.04–0.14 m (flyby), 0.14–0.34 m (hover), 0.46–0.57 m (rubble/occluded); one duplicate entry (occluded) |
| 9 search | `results/phase9/` (7 flights + a first version) | 5 completed with true coverage 100 %, survivors located 0.02–0.22 m; **1 FAILSAFE** (`multi_debris`, machine overload, kept); 1 flight with the planner absent landed safely as designed; `multi_first_version` (duplicate entries) kept as evidence |
| 10 mission manager | `results/phase10/` (7 runs) | nominal and protected runs complete; time budget, perception loss and battery trigger RETURN_HOME; VLM-required and no-planner missions **refused**, vehicle never armed |
| 11 language | `results/phase11/` | 50 instructions (20 dev, 30 held-out): rules 24/30 and model 23/30 held-out correct; **0 unsafe acceptances**; model 17 s per instruction on CPU |

---

## 4. Integrated missions, repeats and faults — [raw] (`results/phase11/live`, `phase13`, `phase14`, `phase16`)

Small Gazebo world (one building plus yard), 2026-10-10. Phases 12–14 ran **without** a live VLM.

| Set | Flights | Outcome |
|---|---|---|
| Phase 12 integrated | 1 | complete; coverage 100 %; 1 avoidance; closest box 0.77 m; survivor 0.05 m |
| Phase 13 whole site | 8 | 8/8 complete, landed, 0 failsafes; survivors 16/16 reported; error mean 0.157 / median 0.085 / max 0.75 m (18 entries); boxes 24 released, **0 collisions**, 15 manoeuvres, closest 0.75 m |
| Phase 13 building | 4 | 4/4 complete; survivors 4/4, error mean 0.107, max 0.15 m; 12 boxes, 0 collisions, 9 manoeuvres, closest 0.85 m |
| Phase 14 faults | 8 fault types | every flight landed; 3 gaps found and fixed (dead localiser, dead depth detector, dead flight controller → no report), **1 open: no report if the mission manager itself dies** |
| Phase 16 live VLM (NVIDIA GPU, kernel 7.0.0-30) | 7 + 1 demo | 9 survivor instances `SURVIVOR_CONFIRMED` (0.02–0.20 m), **0 non-survivors confirmed**, 14 distractor entries `REJECTED_BY_VLM`; first-found policy stopped at 48 % coverage as specified; no failsafe |

Caveats recorded with the results: 12/12 completions → 95 % one-sided lower bound ≈ 78 %; only **two distinct
manikin placements** and one manikin appearance; the VLM still rejects individual crops of true survivors (a
survivor entry counts as confirmed if *any* crop was confirmed, so the result rests on several views); two flights
contain a duplicate low-confidence entry; "0 collisions in 36 boxes" is weaker than the hover tests because boxes were
released above the vehicle at release time; `objective_met` is true for the distractor-only mission (it means
"landed and ended for the planned reason", not "found someone"); operator confirmation was automatic (`CONFIRM=yes`);
the eighth (GUI) flight had one box pass 0.50 m from the vehicle centre (surface ≈ 0.25 m, inside the project's
collision definition; no impact measured, acceleration peak 5.4 m/s²).

---

## 5. Village world, SLAM and map-aware navigation — [raw] (`results/village/`)

17 flights in `summary_missions.json`: **15 COMPLETE, 1 FAILSAFE, 1 REFUSED**. The FAILSAFE
(`…false_localization_alarm`, coverage 0.29) and the REFUSED run (`…refused_param_type`, a parameter-type bug) are
kept as evidence. Fault flights (navigator or SLAM killed at 75 s) landed.

| Quantity | Result |
|---|---|
| SLAM position RMSE vs truth (offline, 17 flights) | 0.08–0.57 m (first-sample alignment) |
| SLAM with deliberately degraded odometry (3 %/6° per min; 8 %/15° per min) | SLAM rigid-fit RMSE 0.16 / 0.21 m; odometry given to SLAM 1.10 / 6.68 m |
| Live VLM flights (NVIDIA GPU) | B: 1 confirmed; D: 1 confirmed, 2 rejected, 1 unverified; F (look-alikes only): 3 rejected, 0 confirmed; C: 3 confirmed, 4 rejected, 1 low-confidence |
| Village survivor errors (completed flights) | 0.10–0.42 m |

The doc `docs/village_system.md` cites a "section 9" listing failed flights that is **missing** from the file; the
failures above come from `summary_missions.json`.

---

## 6. Urban disaster zone (Gazebo) — [raw] (`results/disaster_gazebo/`, `results/faculty/`, `results/audit/`)

Intel graphics, real-time factor ≈ 0.5–0.9; **no live VLM** (NVIDIA module absent on kernel 7.0.0-34). Exterior
search areas only.

| Set | Result |
|---|---|
| Final batch `run_disaster_batch.sh ext` (`summary_ext.json`) | **10/10** COMPLETE, landed, 0 failsafes, PX4-log checks all pass; objective met in 7 (the other 3: the headline mission hit its 420 s budget at coverage 0.795, 2 fault flights end early by design) |
| Survivor entries in that batch | 10 error values, **median 0.205 m**, max 1.07 m (the headline mission reported 1 survivor at 0.08 m and one entry 1.07 m off; one survivor of two was not found) |
| Debris | headline 2 manoeuvres, closest box 1.34 m; debris flight (`xdebris`): 3 manoeuvres, closest box 1.88 m, no impact |
| SLAM vs truth | RMSE 0.50–2.80 m — worse than the village and, in the faculty run, worse than the odometry it is given (rigid-fit 0.69 m vs ≈ 0.20 m) |
| Look-alikes | reported as `UNVERIFIED_CANDIDATE` (no VLM) |
| **All 37 flights with a report** (`summary_missions.json`) | **26 COMPLETE, 11 FAILSAFE** (8 stale-telemetry before `status_timeout_s` was added, 2 impacts in the climb-first return before `map_return`, 1 during a concurrent MuJoCo job). Failures are kept. |
| Faculty flight (`results/faculty/gazebo/…faculty_gz`) | complete, landed 0.14 m from start; survivor reported at (54.95 N, 16.94 E) vs truth (55.0, 17.0): 0.10 m; `UNVERIFIED_CANDIDATE`; SLAM RMSE 1.33 m (0.69 m rigid-fit) |

**Not summarised by any project document — flagged:** `results/faculty_vlm/` holds urban-world attempts made with the
NVIDIA driver unavailable (the logs read say "VLM on the CPU … ~1 min per crop"). From the mission reports and
`survivor.jsonl`:

* Five B/F flights ended COMPLETE; four with coverage 1.0 and objective met, one (`vlm3`) with coverage 0 (the
  controller ended the search; no goal).
* The VLM was available (`VLM_AVAILABLE` in every frame) and answered: `vlm1` 3 reject; `vlm2` 2 reject + 1 confirm;
  `vlm4` 1 confirm; `F1` (look-alikes only) 3 reject. Yet **no entry in those mission reports carries
  `SURVIVOR_CONFIRMED`** (all `UNVERIFIED_CANDIDATE`). Why a logged "confirm" did not reach the report was not
  investigated here.
* The `urban_B_final_vlm` and `urban_E_final_vlm_debris…` series: 1 COMPLETE (time budget) and **6 FAILSAFE**
  (coverage 0–0.79).

These runs therefore support neither a claim of live VLM confirmation in the urban world nor of reliability with a
CPU VLM. Live VLM confirmation *in reports* is demonstrated in the small (Phase 16) and village worlds only.

---

## 7. MuJoCo — [doc] (`MULTI_SIM_COMPARISON.md`, `MULTI_SIM_PROGRESS.md`); not equivalent to Gazebo

| Item | Result | Architecture used |
|---|---|---|
| 6 missions (`results/batch`) | 6/6 complete, 0 collisions, survivors 0.21–0.46 m | own cascaded controller, **simulator ground-truth state**, lidar map with **known pose** (not SLAM), no VLM, no debris avoidance, no mission manager/failsafes |
| Flight tests | hover RMSE 9.4 mm; waypoint cross-track RMSE 0.53 m; blocked-street detour 111 m, 0 collisions | same |
| PX4-in-the-loop | 18 hover/waypoint flights complete with all PX4-log checks passing; 1 lockstep hang unresolved | iris-like model, no camera/lidar in the loop |

These numbers must not be quoted as results of the Gazebo system.

---

## 8. Consolidated limitations (evidence-backed)

1. Simulation only; depth has no sensor noise model; GNSS is available (not GPS-denied flight).
2. Avoidance envelope ≈ 0.9 s warning; hard acceleration, lateral throws < 1 s, roof-opening interior fail.
3. Survivor detector is specific to the simulated orange-clothed manikin; VLM rejects straight-down views.
4. SLAM is 2-D and unreliable in the repetitive urban facades; interiors are out of scope (two impacts).
5. Duplicate / look-alike entries persist without a VLM.
6. No report if the mission manager dies; a dropped second task in an instruction is caught only by the read-back.
7. Live VLM confirmation in the urban-world reports was not demonstrated; the NVIDIA module requires booting kernel 7.0.0-30 (the running kernel 7.0.0-34 has none).
