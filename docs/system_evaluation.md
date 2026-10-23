# Integrated mission, evaluation and failure testing (Phases 12–15)

Status: flown on 2026-10-10. Evidence: `results/phase11/live/` (Phase 12),
`results/phase13/`, `results/phase14/`.

Phases 12–14 ran on the Intel GPU **without a VLM** (the NVIDIA driver was
not available then): no entry was VLM-confirmed in those flights. Phase 16,
at the end of this document, repeats the integrated mission **with the VLM
live** on the NVIDIA GPU. Simulation only.

## Phase 12 — the end-to-end integrated mission

One instruction, one flight, every function running:

```
"Search the collapsed building for survivors and avoid falling debris."
  → interpreted by the language model on the ground (28 s) → read-back
  → mission manager: preflight checks → START
  → takeoff → lane search → side-view inspection of the candidate
  → 3 boxes dropped: debris detected, tracked, predicted, classified, avoided
  → survivor located → return → landing → mission report
```

Result (`results/phase11/live/search_building_base_collapsed_building`):
complete, objective met; search 41 s; coverage 100 %; 1 avoidance manoeuvre;
closest box 0.77 m (surface to vehicle centre); survivor reported 0.05 m
from its true position, 1σ 0.20 m; all PX4-log checks passed.

**Not part of it:** live VLM confirmation. That needs the NVIDIA GPU.

## Phase 13 — repeated integrated missions

The same mission flown repeatedly, boxes dropped during every search.
Missions were produced from instructions by the deterministic interpreter.

| | Whole site, 2 survivors + distractors | Building, 1 survivor |
|---|---|---|
| Flights | 8 | 4 |
| Completed, objective met | 8 | 4 |
| Landed (PX4 log) | 8 | 4 |
| Failsafes | 0 | 0 |
| Survivors reported / present | 16 / 16 | 4 / 4 |
| Survivor position error: mean / median / max | 0.157 / 0.085 / 0.75 m (18 entries) | 0.107 / 0.11 / 0.15 m |
| Search time: mean ± std (min–max) | 124.4 ± 9.4 s (102–137) | 34.8 ± 0.8 s (34–35.5) |
| Coverage | 100 % in every flight | 100 % in every flight |
| Boxes released / collisions | 24 / **0** | 12 / **0** |
| Avoidance manoeuvres | 15 | 9 |
| Closest box (surface to vehicle centre) | 0.75 m | 0.85 m |
| Entries labelled confirmed | 0 | 0 |

* In 2 of the 8 whole-site flights one survivor appears **twice** in the
  report; the second entries are 0.68 m and 0.75 m off. Without those two
  the largest error is 0.21 m.
* Collision = box surface within 0.40 m of the vehicle centre (the Phase 5
  definition). The boxes are released above the vehicle's position at
  release time; while the vehicle is moving some would have missed anyway,
  so "0 collisions in 36" is weaker evidence than the Phase 6 hover tests.
* Twelve flights is still a small sample: with 12 successes in 12, the
  95 % lower confidence bound on the completion rate is about 78 %.

## Phase 14 — failure testing

One process stopped 40 s after the controller started (mid-search) in each
flight. World: two survivors + distractors.

| Process stopped | What happened | Landed | Report |
|---|---|---|---|
| Search planner | Controller refused stale goals, returned home after 5 s without a goal | yes | written; "flight controller ended the search"; objective not met |
| Camera bridge (both cameras) | Mission manager: "survivor perception lost for 3.1 s" → RETURN_HOME | yes | written; objective not met |
| Avoidance planner (protection required) | Mission manager: "debris avoidance lost" → RETURN_HOME | yes | written; objective not met |
| PX4 bridge (XRCE agent) | Controller: telemetry stale → FAILSAFE; PX4's own failsafe: return and land | yes | written; outcome FAILSAFE |
| Survivor localisation | **Gap found:** the mission continued and reported "objective met" with a dead localiser. **Fixed** (watchdog on the location stream). Re-flown: "survivor localisation lost for 3.1 s" → RETURN_HOME | yes | written; objective not met |
| Debris detector (protection required) | **Gap found:** the avoidance planner stayed alive without depth data, so the mission manager believed protection was present and flew on unprotected. **Fixed** (it now watches the planner's perception-health flag). Re-flown: "debris perception stale for 3.0 s" → RETURN_HOME | yes | written; objective not met |
| Flight controller | PX4's offboard-loss failsafe: return-to-launch and land, 0.02–0.03 m from the takeoff point. **Gap found:** no report was written (the mission manager waited for ever). **Fixed** (it concludes after 10 s without controller status). Re-flown: report written, outcome FAILSAFE | yes | written after the fix |
| Mission manager | The controller finished the search by its own rules, returned and landed | yes | **none** (by design the report comes from the mission manager; still open) |

Every flight landed. Three gaps were found by these tests and closed; each
has a unit test. The first, un-fixed runs are kept beside the re-flown ones
(`…_v2`).

Things these tests showed that are worth knowing:

* When the flight controller dies, PX4's return-to-launch **climbs to about
  16 m** before returning (PX4's default return altitude). That is outside
  the altitude band the mission otherwise keeps; no PX4 parameter was
  changed.
* The first attempt to stop the mission manager did nothing: the process
  name was mistyped in the test. Found from the evidence (a report existed),
  corrected and re-flown.

Not tested: two faults at once; faults during takeoff or landing; a node
that keeps running but produces wrong data; network delay; sensor faults.

## Phase 16 — integrated missions with the VLM live

Flown on 2026-10-10 after booting kernel 7.0.0-30 (NVIDIA RTX 4050; 3.8 GB
of 6.1 GB used with the model loaded). Seven missions, each from a
natural-language instruction interpreted by the language model
(1.95–2.80 s per instruction on the GPU), confirmation given
automatically. Evidence: `results/phase16/` (`live/<run>/mission_report.json`,
`survivor.jsonl` with every VLM request and answer, `summary.json`,
`ollama.log`).

All seven planned missions ran, including the distractor-only one.

| Run (`results/phase16/live/…`) | World | Outcome | Coverage | Survivors confirmed / in the searched area | Rejected by VLM | Low-confidence entries | Avoidance | Battery min | Landed |
|---|---|---|---|---|---|---|---|---|---|
| `search_whole_site_multi_site_confirmed_1` | 2 survivors + distractors, 3 boxes dropped | complete, objective met | 100 % | **2 / 2** (0.13 m, 0.05 m) | 3 distractors | 1 (second entry of the outside survivor, 0.70 m off) | 2 | 0.50 | yes |
| `search_whole_site_multi_site_confirmed_2` | same | complete, objective met | 100 % | **2 / 2** (0.13 m, 0.02 m) | 3 distractors | 0 | 2 | 0.50 | yes |
| `search_whole_site_multi_site_confirmed_3` | same | complete, objective met | 100 % | **2 / 2** (0.14 m, 0.02 m) | 3 distractors | 1 (second entry of the cylinder, 0.84 m off) | 2 | 0.50 | yes |
| `search_whole_site_multi_site_first_found` | same world, no boxes; "come back as soon as you find one" | complete, objective met (per its policy) | **48 %** | **1 / 2** (0.20 m); the second survivor was never searched for | 1 distractor | 1 (7 frames, matches nothing) | 0 | 0.65 | yes |
| `search_building_base_building_confirmed` | building, 1 survivor | complete, objective met | 100 % | **1 / 1** (0.04 m) | 0 | 0 | 0 | 0.50 | yes |
| `search_north_yard_outdoor_yard_confirmed` | 1 survivor outside | complete, objective met | 100 % | **1 / 1** (0.09 m) | 0 | 0 | 0 | 0.50 | yes |
| `search_building_distractors_distractors_only` | orange objects only, **no survivor** | complete, objective met | 100 % | 0 / 0 — **nothing confirmed** | 4 distractors | 0 | 0 | 0.50 | yes |

What the evidence supports:

* **Confirmation against ground truth.** Every entry labelled
  `SURVIVOR_CONFIRMED` (9 entries in 6 flights) lies on a true survivor,
  0.02–0.20 m from its true position. **No non-survivor was confirmed in
  any flight**, including the distractor-only mission (4 objects located,
  all 4 `REJECTED_BY_VLM`).
* **Counts, stated carefully.** In the six missions that covered their whole
  area, 9 of 9 survivor instances were confirmed. Counting the first-found
  mission as well, 9 of the 10 survivor instances inside a mission area were
  confirmed; the tenth was not missed by perception — the search ended
  before reaching it, as the instruction asked. These are *per-flight
  instances*: the worlds contain only **two distinct manikin placements**
  (inside the building, and outside to the north), each confirmed in several
  flights. This is not evidence about ten different survivors.
* **Distractors.** 14 distractor entries across the flights, all labelled
  `REJECTED_BY_VLM` (plus 3 low-confidence entries, below).
* **First-confirmed policy.** Behaved as specified: the survivor under the
  takeoff point was confirmed during the climb; RETURN_HOME was sent 1.5 s
  after the search began ("a survivor is confirmed and located"); the
  vehicle landed with 48 % coverage and one survivor unreported. That is
  the policy working, and also its cost: the instruction "come back as soon
  as you find one" trades the second survivor for speed. The flight-log
  check "hold altitude for 15 s" fails for this run only because the flight
  left altitude early by design.
* **Search time with a VLM is shorter.** Whole-site search took 41.5–55.5 s
  here against 102–137 s without a VLM (Phase 13): rejected candidates are
  not inspected and an inspection ends as soon as its candidate is
  confirmed (2–3 inspections per flight instead of 4–5).
* **Debris.** 3 boxes dropped in each of the three protected missions; 2
  avoidance manoeuvres each; closest box 1.02–1.33 m (surface to vehicle
  centre); no collision.
* **VLM activity.** 109 requests in total; answer latency median 2.5–2.6 s,
  maximum 3.17 s; one request superseded by a newer crop (recorded as an
  error, by design); no failed request.
* **No failsafe** in any of the seven flights (in Phase 7, 2 of 11 flights
  failed from GPU contention; that did not recur here, in 7 flights).

Problems and limits found in the evidence:

* **The VLM still rejects true survivors in individual crops.** Example:
  in the building mission the survivor's crops were answered "confirm" 4
  times and "reject" 3 times; in the yard mission 1 "confirm" and 3
  "reject". An entry counts as confirmed if **any** crop was confirmed
  (the label is sticky), and in one flight the same image track was
  confirmed and later rejected. So the 9-of-9 result rests on the search
  giving several views, not on the VLM being reliable per image. A
  rejection of a distractor is likewise not final in principle; no
  distractor was ever confirmed here.
* **Duplicate entries persist**: 2 of 7 flights contain a second,
  low-confidence entry for an object already in the report (0.70 m and
  0.84 m off), and one flight has a 7-frame entry matching nothing. They are
  labelled `LOW_CONFIDENCE`, not confirmed.
* **Seven flights, two manikin placements, one manikin appearance.** No
  occluded or rubble-covered survivor was flown with the VLM in the
  integrated mission. The detector still only knows this manikin.
* **`objective_met` is true for the distractor-only and first-found
  missions** because it means "landed and the search ended for the planned
  reason", not "a survivor was found". Read the entries.
* The operator confirmation of each interpreted mission was automatic.
* Evidence that does not exist: flight logs are present, but camera frames
  were not recorded (`LIGHT=1`), so the crops the VLM saw are not on disk.

### An eighth flight: the demonstration run with the Gazebo window open

`results/phase16/live/search_whole_site_multi_final_sitl` — the same
whole-site, confirmed-only, debris-protected instruction, flown once more
with the simulator's graphical window open. Outcome: complete, objective
met, coverage 100 %, **both survivors confirmed** (0.17 m and 0.03 m),
3 distractors rejected, 3 low-confidence duplicate entries, search 72 s,
landed, all flight-log checks passed.

**One box came too close in this flight.** The second box passed **0.50 m
from the vehicle's centre** while falling at 13 m/s. By the project's own
scoring rule (box surface within 0.40 m of the centre) that is inside the
collision range (surface distance about 0.25 m). No impact was measured
(largest acceleration in that window 5.4 m/s²; the monitor's threshold is
30 m/s²) and the mission continued, but this is **not** a clean avoidance:
the planner logged "no feasible candidate; best effort" for it, and made 7
manoeuvres for 3 boxes. The seven batch flights above (no window) had a
smallest box distance of 1.02 m. One flight cannot say whether the open
window (extra rendering load) caused the difference.

## Phase 15 — demonstration and final status

One command runs the whole system from an instruction (asks for
confirmation; add `HEADLESS=0` to watch in Gazebo):

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && HEADLESS=0 LIGHT=1 scripts/mission/run_instruction.sh "Search the collapsed building for survivors and avoid falling debris." base demo
```

Without a language model (instant interpretation by keyword rules):

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && INTERPRETER=rules HEADLESS=0 LIGHT=1 scripts/mission/run_instruction.sh "Search the whole site for survivors and avoid falling debris." multi demo_site
```

Reproduce the repeat and fault batches (about 70 minutes):

```bash
cd ~/Desktop/drones/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && scripts/mission/run_phase13_14.sh all
```

### Where the project stands

| Phase | Status |
|---|---|
| 0–5 | done (flight, world and sensors, debris detection, tracking, prediction, risk) |
| 6, 6.1 | bounded: avoidance works with about 0.9 s of warning |
| 7 | simulation baseline: candidate detector + VLM, validated separately with the GPU |
| 8–10 | done without a live VLM (localisation, search, mission manager) |
| 11 | done: instruction → validated mission; language model on the CPU |
| 12 | done without a live VLM: one instruction, one integrated flight |
| 13 | done: 12 of 12 repeated integrated missions completed |
| 14 | done: 8 fault types; all landed; 3 gaps found and fixed; 1 open (no report if the mission manager dies) |
| 15 | this document, the demo commands, and the study guide |
| 16 | done: 7 integrated missions with the VLM live; 9 survivor instances confirmed, 0 false confirmations; first-found policy flown |

### What is still open

1. Live VLM in the integrated mission is now flown (Phase 16), but only in
   7 flights with two manikin placements; the VLM still rejects individual
   crops of true survivors.
2. Avoidance outside its envelope: hard acceleration, lateral throws under
   about 1 s, no escape inside the roof opening.
3. The survivor detector knows only the simulated manikin.
4. Duplicate report entries (2 of 8 whole-site flights) and distractors
   reported as unverified candidates.
5. No report when the mission manager itself dies.
6. A dropped second task or contradiction in an instruction is caught only
   by the operator's read-back.
7. Simulation only: perfect depth, no wind, GNSS available.
