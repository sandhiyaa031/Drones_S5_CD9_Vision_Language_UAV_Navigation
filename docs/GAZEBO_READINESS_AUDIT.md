# Gazebo readiness audit for the faculty demonstration

Audit date: 2026-10-20. Machine: i7-14650HX, 15 GB RAM, RTX 4050 6 GB,
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.11, PX4 SITL (unmodified,
commit d983b64, airframe `gz_x500`). Paths are relative to
`~/Desktop/drones/ros2_ws/` unless they start with `~`.

"Flown today" means a live PX4 + Gazebo flight run during this audit, with
the result in `results/audit/`. Everything else cites earlier evidence on disk.

## Overall status: **PARTIALLY READY**

| System | Status for a live demo | Why |
|---|---|---|
| **Small-world integrated rescue mission** (one collapsed building, sentence → search → debris avoidance → survivor localisation → return → land) | **READY** | 12/12 repeated missions (Phase 13), 8/8 live-VLM missions (Phase 16), and **flown again today** on Intel graphics: complete, 2/2 survivors located, log checks passed |
| **Village + SLAM system** (124 m × 128 m village, lidar SLAM, map-based navigation) | **NOT RELIABLE YET** | Works in most flights (16 of 19 mission flights completed and landed at base), but **3 of 19 ended in a "telemetry stale" failsafe**, including **the only rehearsal of the visual demo** (today, Gazebo window + RViz) |
| **GPU / live VLM today** | **BLOCKED until a reboot** | Current kernel `7.0.0-34-generic` has no NVIDIA module package; `nvidia-smi` fails. Kernel `7.0.0-30-generic` has the module (`linux-modules-nvidia-595-open-7.0.0-30-generic`) and was the validated configuration |

## 1. Checklist results

| # | Item | Result | Evidence |
|---|---|---|---|
| 1 | Gazebo Harmonic launches and loads the intended world | **Yes.** Small world and village world both loaded today | `results/audit/live/*/gazebo.log` |
| 2 | PX4 SITL ↔ Gazebo ↔ ROS 2 (XRCE-DDS) | **Yes.** Every flight today got PX4 telemetry; preflight passed | `results/audit/live/*/xrce_agent.log`, `mission_manager.log` ("preflight checks passed") |
| 3 | Arm, take off, hover, waypoints, land | **Yes.** 5/5 hover flights pass all log checks (Phase 0); waypoint flights (Phase 1); every completed mission today landed at base, verified from the PX4 log | `results/final/phase0_flight_foundation/`, `results/audit/live/*/verification.json` |
| 4 | Camera and depth sensors publish usable data | **Yes.** Downward RGB feeds the survivor detector, upward depth feeds the debris detector. On Intel today: lidar 5.4 Hz (8.1 Hz with NVIDIA). Phase 1 measured 30 Hz cameras with NVIDIA | today's flights; `results/village/checks/sim_noshadow.json` |
| 5 | Debris chain wired into the mission | **Yes, bounded.** Detector → tracker → predictor → risk → avoidance proposal → flight controller, running inside missions. Today, small world: 2 avoidances, closest box 1.06 m, no impact. Needs ≈ 0.9 s of warning. In the Phase 16 GUI demo flight one box passed 0.50 m from the centre ("no feasible candidate; best effort") | `results/audit/live/search_whole_site_multi_intel_small_world/`, `docs/debris_avoidance.md`, `docs/system_evaluation.md` |
| 6 | Survivor detection, VLM confirmation, 3D localisation | **Detection + localisation: yes**, flown today, errors 0.05–0.17 m. **VLM confirmation: yes, but only with the NVIDIA GPU** (Phase 16, gemma3:4b via Ollama); not possible today. The detector is a colour/shape baseline for the simulated manikin, not a general person detector | `results/phase16/summary.json`, `results/audit/live/` |
| 7 | Natural-language mission → planner → manager | **Yes** with the rule interpreter (flown today via `run_instruction.sh`). The LLM interpreter was evaluated in Phase 11 (CPU); not run today | `results/audit/live/intel_small_world_interpretation/`, `results/phase11/` |
| 8 | Coverage, completion, failsafes, return, landing | **Yes in the small world**: Phase 14 fault tests (8 flights, 6 different components killed): all 8 landed; killing the XRCE agent correctly gave a failsafe. **Village**: SLAM kill and navigator kill both returned and landed. See risk R1 for the unplanned failsafes | `results/phase14/summary.json`, `results/village/live/*kill*` |
| 9 | Size of the environment | **Both exist.** The original world is one collapsed building (≈ 20 m). The village world (`worlds/village/`, 6 scenarios) has streets, 23 buildings, trees, walls, a large collapsed multi-storey building and a damaged house | `scripts/village/make_village.py`, `docs/village_system.md` |
| 10 | Genuine SLAM | **Yes, in the village system only.** 2D lidar pose-graph SLAM (`slam_toolbox`) at the 6 m flight slice, with loop closure. Scored offline against ground truth: search-phase RMSE 0.11–0.57 m. SLAM is used for the map and route planning, **not** for flight stabilisation (PX4 EKF2 with simulated GNSS flies the drone). Survivor localisation is not SLAM | `results/village/summary_slam.json`, `scripts/slam/evaluate_slam.py` |
| 11 | Live vs unit-tested vs unverified | See section 3 | — |
| 12 | Phase 16 live-VLM reports | **All 8 flights complete, all landed, objective met.** Distractor-only mission: 4 candidates, **all 4 rejected by the VLM**, no false survivor. Over the 7 batch flights, 9 of 10 survivors were reported; the one miss is the "first found" mission, which by design stops after the first confirmation. One flight failed the altitude-hold log check | `results/phase16/summary.json`, `results/phase16/live/*/mission_report.json` |

Unit tests run today: **550 passed, 6 skipped**; the only failures are the
two style-lint tests (`test_flake8`: 271 style findings, `test_pep257`).
Run with the workspace sourced (`source install/setup.bash`); without it the
`px4_msgs` imports fail.

## 2. What actually works (with evidence)

Small world (Phases 0–16):

- Flight foundation: `results/final/phase0_flight_foundation/`.
- Debris chain: `results/phase2`–`phase6/`, `docs/debris_*.md`.
- Survivor chain + VLM: `results/phase7/`, `results/phase16/`.
- Repeats: 12/12 complete (`results/phase13/summary_building.json`,
  `summary_site.json`).
- Faults: `results/phase14/summary.json`.
- **Today**: `results/audit/live/search_whole_site_multi_intel_small_world/`.
  Complete, coverage 100 %, 2/2 survivors located (0.11 m, 0.05 m; reported
  unverified because no VLM was available), 2 avoidances, closest box 1.06 m,
  landed, all PX4-log checks passed.

Village + SLAM (built after Phase 16):

- 19 mission flights (`results/village/live/`, `results/audit/`):
  16 complete and landed at base; 3 telemetry-stale failsafes.
- Survivors: every survivor in the searched area was reported, 0.07–0.42 m
  off. With the VLM: 3/3 confirmed in the three-survivor world, look-alikes
  all rejected, no false survivor in the look-alike-only world. **In the
  occlusion world only 1 of 2 was confirmed**: the second was reported
  unverified, and a duplicate track on that person was rejected.
- SLAM: search-phase RMSE 0.11–0.57 m over 80–400 m paths. With
  deliberately degraded odometry, SLAM held 0.16 m and 0.21 m where the
  odometry was off by 1.1 m and 6.7 m. Map: 61–81 % of mapped wall cells lie
  within 0.5 m of a true wall.
- Fault handling: SLAM killed → "LOCALIZATION LOST" after 6 s → climb, return,
  land. Navigator killed → controller has no valid goal → return, land.
- Visualisation (checked live today): RViz opens with `config/village.rviz`;
  `/map`, `/uav/scan`, `/planning/navigator/path`, `/viz/flown_path`,
  `/viz/mission` (base, search area, mission text, candidates) all publish.

## 3. Tested live / tested offline / unverified

| Live in Gazebo (repeated) | Unit- or replay-tested only | Unverified |
|---|---|---|
| Takeoff, hover, waypoints, landing | Navigation A* and frame maths (41 tests) | VLM in the village visual demo (not yet run with the GPU) |
| Debris chain inside missions | Mission language rules, grounding (unit tests) | Village with the Gazebo window + RViz completing a mission (**0 of 1 rehearsals**) |
| Survivor detection, localisation, VLM (small world, village) | Route planning time: median 23–31 ms, p95 ≤ 135 ms (offline on recorded maps, `results/village/planner_benchmark.json`) | LLM interpreter in a village mission |
| NL instruction → mission (rules) | SLAM scoring (offline vs truth, by design) | Debris avoidance checked against the SLAM map (not implemented) |
| Fault injection (small world 7 kills, village 2 kills) | | Real hardware (never claimed) |
| SLAM + navigator in 19 village flights | | |

## 4. Implemented but not demonstrated end-to-end

- The **village visual demo** (`scripts/village/run_demo.sh`). Its one
  rehearsal today ended in a failsafe after 293 s of search (on Intel graphics,
  since the NVIDIA GPU was unavailable).
- **Post-fix repeat batch** (`run_village_batch.sh v2`): 3 of 8 flights
  completed (A, B r1, all within budget); 1 failsafe (B r2); the 4th was
  interrupted when the previous session ended
  (`search_collapsed_building_village_B2_v2_r1` has no report); the rest
  never ran.
- `docs/village_system.md` is written except for its results section.

## 5. Bugs, missing pieces, risks

| ID | Severity | Finding | Evidence |
|---|---|---|---|
| R1 | **High** | **Intermittent "PX4 position/status telemetry became stale" failsafes in the village world**: 3 of 19 mission flights, plus 1 earlier check flight. No unplanned failsafe in the 29 small-world mission flights (Phases 13, 14, 16 and today). The controller treats telemetry older than 1.0 s (wall clock) as lost. PX4 runs in lockstep with Gazebo, so a simulator stall over 1 s triggers it. The village (gpu_lidar + 3 cameras, saturated single server core) and the GUI add load. That cause is a **hypothesis, not proven**: the wall-clock telemetry gaps were not logged | `results/audit/demo/…/flight_controller.log`, `results/audit/live/search_collapsed_building_village_B_intel_headless/`, `results/village/live/search_collapsed_building_village_B_v2_r2/` |
| R2 | **High (today)** | NVIDIA driver not loaded: kernel 7.0.0-34 has no `linux-modules-nvidia-595-open-7.0.0-34` package. Consequences: Intel rendering (slower sensors) and **no live VLM** (a CPU VLM answers in 44–82 s and starves the simulator) | `nvidia-smi`, `dpkg -l` |
| R3 | Medium | After a failsafe the trial script tears down before PX4 finishes its AUTO LAND, so "landed" cannot be verified. In a real flight PX4 would land where it is, inside the village | `verification.json` of the 3 failsafe runs |
| R4 | Medium | Village searches are long: 4–8 min of simulated time per mission at real-time factor 0.69–0.83 (NVIDIA), slower on Intel. The 480 s "headline" mission is too long for a live slot | `results/village/summary_missions.json` |
| R5 | Medium | Debris avoidance unchanged (≈ 0.9 s envelope); its sidestep is not checked against the SLAM map. One "no feasible candidate" event in today's demo flight | `results/audit/demo/…/debris_avoidance.log` |
| R6 | Fixed | The navigator hovered at the nearest free point to a blocked waypoint and never reported it unreachable, using up the time budget. Fixed and regression-tested; the 3 later normal village flights all finished their search within budget | `test/test_navigation.py`, `results/village/live/*_v2_*` |
| R7 | Low | `results/final/PHASE_STATUS.md` and `results/final/GAZEBO_DEMO.md` are dated 2026-10-06 and say Phases 8–15 were "not started"; they are out of date and would mislead a reader | those files |
| R8 | Low | Style lint fails (flake8 271, pep257) | `pytest test` |
| R9 | Info | The search planner runs one rectangle per mission ("village" = disaster district). SLAM is a 2D slice at 6 m, not 3D | `missions/places_village.json` |
| R10 | Info | Modified source files are not committed (git shows them as modified or untracked); the user commits | `git status` |

## 6. Shortest path to a reliable faculty demonstration

1. **Get the NVIDIA GPU back** (owner action, needs a reboot): reboot, open
   GRUB "Advanced options", pick `7.0.0-30-generic`. Then check
   `nvidia-smi`. Do not uninstall anything.
2. **Make the small-world mission the live demo** (proven repeatedly), with
   the VLM and the Gazebo window. Rehearse the exact command 3 times on the
   NVIDIA kernel.
3. **Show the village/SLAM system with the shorter mission** ("collapsed
   building", about 3.5 min of search), not the 480 s headline. Rehearse
   3 times with `run_demo.sh`. If any rehearsal fails, show RViz from a
   headless run (`HEADLESS=1 VIZ=1`) instead of the Gazebo window, plus the
   recorded results.
4. **Diagnose R1 before trusting the live village demo.** Log the wall-clock
   age of PX4 telemetry when the controller declares it stale, and Gazebo's
   real-time factor around that moment. Only then decide on a fix (e.g. less
   rendering load). The 1.0 s safety timeout must not simply be relaxed.
5. Refresh `results/final/PHASE_STATUS.md` and `GAZEBO_DEMO.md` (R7) so the
   documents shown to faculty match the current state.

## 7. Commands (run from `~/Desktop/drones/ros2_ws`)

Machine check:

```bash
uname -r && nvidia-smi
```

Environment for every command:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

Ollama server for the VLM (NVIDIA only; leave running in its own terminal):

```bash
~/ollama/bin/ollama serve
```

**Demo A — small world, full chain** (flown today headless on Intel graphics;
with the VLM in Phase 16). Opens the Gazebo window:

```bash
INTERPRETER=rules HEADLESS=0 LIGHT=1 scripts/mission/run_instruction.sh "Search the whole site for survivors, only report confirmed ones, and avoid falling debris." multi demo_site
```

Without the GPU, drop "only report confirmed ones" (otherwise the mission is
correctly refused: no VLM available):

```bash
INTERPRETER=rules HEADLESS=0 LIGHT=1 scripts/mission/run_instruction.sh "Search the whole site for survivors and avoid falling debris." multi demo_site
```

**Demo B — village, SLAM, navigation, RViz** (shorter mission; rehearse first):

```bash
scripts/village/run_demo.sh "Search the collapsed building for survivors and avoid falling debris." village_B demo_village
```

The full headline instruction (8+ minutes; failed its one rehearsal today):

```bash
scripts/village/run_demo.sh
```

Switch the Gazebo camera while it flies:

```bash
scripts/village/view.sh site
```

Score the village runs afterwards (offline; ground truth used here only):

```bash
scripts/village/evaluate_village.sh
```

Unit tests:

```bash
cd src/uav_autonomy && python3 -m pytest test -q
```


---

# Update, 2026-10-20 (evening): urban disaster zone and MuJoCo

Everything above is the audit as written that morning and is left unchanged.
Since then the following was built and measured (details and evidence:
`MULTI_SIM_PROGRESS.md`, `GAZEBO_DEMO_GUIDE.md`, `MULTI_SIM_COMPARISON.md`).

## Overall status after the update: **PARTIALLY READY, improved**

| System | Status | Why |
|---|---|---|
| Small-world mission (unchanged) | READY | still available; not re-flown after these changes except the 2026-10-20 audit flight |
| **Urban disaster zone, exterior searches, Intel graphics, headless or with the GUI** | **READY for a rehearsed demonstration** | final configuration: 10 of 10 batch flights completed, landed and passed every PX4-log check; the GUI demo with RViz and debris did too |
| Same, with the live VLM | NOT AVAILABLE | NVIDIA driver not loaded on kernel 7.0.0-34 (reboot owner action); candidates stay unverified |
| Village world | unchanged | its 3 stale-telemetry failsafes (R1) are explained below |
| **MuJoCo** | standalone: works (GUI + headless, 12 tests); **PX4 in the loop: demonstrated for hover and 3 waypoints** | `simulation/mujoco/README.md` |

## R1 is now understood (measured)

PX4 publishes `vehicle_status` at about 2 Hz of *simulation* time. At a
real-time factor near 0.5 (heavy world on Intel graphics) two messages are about
1 s apart in wall time, which equals the controller's 1.0 s freshness limit, so
the controller declared "telemetry stale" without any link loss (a separate
monitor saw `/clock` and the position stream healthy: gaps below 0.35 s while
status gaps were 0.83-1.14 s). Fixes: the simulation was made cheaper (static
batching of visuals, one collision body: real-time factor 0.6 to about 0.9 with
sensors, probe) and the controller got a parameter `status_timeout_s` (default
1.0 s = unchanged; 3.0 s for urban runs; position telemetry keeps 1.0 s).
Result: 0 stale-telemetry failsafes in the final configuration. The village
world's earlier failsafes (R1 above) are probably the same effect (real-time
factor 0.79-0.84 at takeoff); not re-tested there.

## New findings

| ID | Severity | Finding |
|---|---|---|
| R11 | High (fixed for exteriors) | A 2D lidar slice cannot see overhead or slabs just below its plane. Two urban flights ended in impacts during the 18 m return climb after entering the collapsed block through a facade opening; a MuJoCo mission collided with a drooping floor tile inside it. Fixes: map-routed return (`map_return`, climb only near home), breadcrumb backtracking when no route home exists, and **exterior-only search areas; interiors of standing structures are out of scope** |
| R12 | Medium | SLAM in the urban world: error 0.5-2.8 m RMSE (village: 0.1-0.6 m) because of repetitive facades and clutter; with the village settings a false match rotated the map by about 25 degrees (kept as evidence). Stricter acceptance settings help; a narrower search window made slam_toolbox crash (also kept) |
| R13 | Info | Dust/haze not simulated on purpose (particle effects would appear in depth and lidar data) |
| R14 | Process | I ran a MuJoCo job during a Gazebo flight and spoiled it; the flight was re-flown alone. Rule: no heavy job next to a Gazebo flight |

## Urban world quick facts

- Generator `scripts/disaster/make_disaster_zone.py`: procedural meshes and
  textures, 6 scenarios, 39k triangles, no external assets
  (`DISASTER_ASSET_LICENSES.md`); validation `validate_disaster.py` (heights,
  6 m slice, routes, detour): PASS for all 6.
- Commands: see `GAZEBO_DEMO_GUIDE.md` (`scripts/disaster/run_demo.sh` for the
  one-command GUI demo).
- Test counts: 552 unit tests pass (5 skipped; two style-lint tests fail).


---

# Final evidence audit, 2026-10-21

Re-measured from the working tree (not from the earlier narrative). Evidence:
`results/audit/gazebo/`, `simulation/mujoco/results/audit/`, details in
`MULTI_SIM_PROGRESS.md` (Stage 9).

| Check | Result |
|---|---|
| Urban world loads and renders | YES: 9 stills inspected (`results/audit/gazebo/screenshots/`) |
| GUI demo (Gazebo window + RViz + debris), Intel graphics | COMPLETE, landed, PX4 checks pass, survivor 0.20 m, 3 avoidance manoeuvres |
| Strongest mission: headline sentence, whole exterior zone, debris | COMPLETE, landed, PX4 checks pass, coverage 0.893, **1 of 2 survivors located (0.01 m)**, 3 candidates are look-alikes (no VLM), 3 manoeuvres, closest box 1.9 m |
| Old small-world mission (fallback) | COMPLETE, 2 of 2 survivors (0.09, 0.03 m), 2 manoeuvres, landed, PX4 checks pass |
| Live VLM | **UNVERIFIED / unavailable**: NVIDIA driver not loaded on kernel 7.0.0-34 |
| ROS 2 regression suite | 552 passed, 6 skipped, **2 failed** (style-lint tests flake8 280 findings, pep257; failing before this work) |
| Gazebo with MuJoCo closed / MuJoCo with Gazebo closed | YES, and each simulator's launcher refuses to start while the other's PX4 stack is running (`tests/check_separation.sh`, both directions pass) |

Findings of the audit itself
- A refused MuJoCo PX4 launch killed a running Gazebo flight (its cleanup ran before the
  refusal check). Fixed; the flight that was spoiled is documented in
  `results/audit/gazebo/spoiled_by_script_bug/NOTE.txt`.
- One of 19 PX4-in-the-loop MuJoCo trials hung in lockstep: unresolved (watchdog added).
- The headline mission still finds only part of the survivors within its time budget and
  cannot reject look-alikes without the VLM: for a demonstration, use the single-area
  missions (`collapsed block`, `courtyard`: 5 of 5 survivors found in the final batch) and
  the headline sentence only with the VLM.
- Visual fidelity: dark, grey, damaged concrete structures with rubble and beams, but not
  photoreal (regular window grid, small rubble piles); dust/haze not simulated by design.
