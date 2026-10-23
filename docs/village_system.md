# Village system: SLAM, map-aware navigation and village-scale missions

Simulation only (PX4 SITL + Gazebo). Nothing here has flown on real hardware.
Every number in this document comes from the files under `results/village/`;
failed and aborted flights are kept there and are listed in section 9.

## 1. What was there before, and what this stage adds

Audit of the code before this stage (phases 0–16):

| Part | State before | Evidence |
|---|---|---|
| Flight control (`flight_controller.py`, sole publisher on `/fmu/in/*`) | working | phases 1–2 flight logs |
| Falling-debris chain (depth detector → tracker → predictor → risk → avoidance proposal) | working, 0.9 s reaction envelope | `docs/debris_*.md` |
| Survivor perception (candidate detector, confirmation, two-step VLM) | working | `docs/survivor_perception.md` |
| Survivor localisation (camera ray + EKF) | working; this is *object* localisation, not SLAM | `docs/survivor_localization.md` |
| Search planner (lawnmower + inspection) | working, straight lines only, no obstacles in its world | `docs/search_planner.md` |
| Mission manager, natural-language missions | working | `docs/mission_manager.md`, `docs/mission_language.md` |
| World | one building on an empty plane, 20 m across | `worlds/` |
| SLAM, obstacle map, route planning | **absent** | — |

Added in this stage:

1. A village-scale world generator with six scenario worlds.
2. A planar lidar on the vehicle and a real SLAM stack (`slam_toolbox`) fed by
   it, scored against simulator truth offline.
3. A navigator that plans routes on the SLAM map and replans as the map grows.
4. A climb-first safe return in the flight controller.
5. Missions over the village from one human sentence, including "prioritize".
6. RViz visualisation built only from live topics.
7. Scripts for one-command demo, batch evaluation and scoring.

## 2. Architecture

```
gz gpu_lidar (720 rays, 10 Hz, 0.6–30 m)        PX4 EKF2 odometry
        │ /uav/scan_raw                                 │
        ▼                                               ▼
   slam_bridge ── levelled scan /uav/scan ──►  slam_toolbox (async, 2D pose graph)
        │  TF odom→base_level                           │ /map, TF map→odom
        ▼                                               ▼
                         navigator  (A* on the inflated SLAM map)
 search_planner ─ /planning/search/goal_raw ─►  │
                                                ▼ /planning/search/goal
 debris chain ── avoidance proposal ──►  flight_controller ──► /fmu/in/*  (only node)
 survivor perception → localisation → mission_manager (preflight gate, report)
```

Rules that still hold:

- Only `flight_controller.py` publishes to PX4. The navigator publishes a goal
  on the same topic the search planner used; the controller validates it as
  before (range, altitude, freshness).
- No simulator ground truth is read online. `poses.csv` is used by the scoring
  scripts only.
- PX4 source and safety parameters are unchanged.

### What SLAM is used for — and what it is not

- **Used:** building the obstacle map, and placing the vehicle in that map for
  route planning. Goals are converted between the PX4 frame and the map frame
  through the two current pose estimates, so a rotated or shifted map frame
  does not matter.
- **Not used:** flight stabilisation. PX4's EKF2 still flies the vehicle with
  simulated GNSS. This is *not* GPS-denied flight. SLAM's input odometry is
  the PX4 estimate, so with a healthy PX4 estimate SLAM cannot be more
  accurate than it; the SLAM benchmark therefore also feeds deliberately
  degraded odometry (section 6).
- Survivor localisation (camera ray + EKF) is a separate component and is not
  SLAM.

### Frames

| Frame | Convention | Source |
|---|---|---|
| Gazebo world | ENU, base pad at the origin | simulator |
| PX4 local | NED, origin at takeoff; (n, e, d) = (world y, world x, −z) | PX4 |
| `odom` | ENU copy of the PX4 local frame | `slam_bridge` |
| `base_level` | vehicle position, yaw only (roll/pitch removed) | `slam_bridge` |
| `map` | ENU, fixed by SLAM at its first scan | `slam_toolbox` |

The map frame is rotated about 5–7° against the PX4 frame in these flights,
because PX4's heading estimate is still settling when the first scan arrives.
That is harmless for navigation (see above) and is why the scorer reports a
second, rigid-fit alignment.

### The lidar slice

The lidar is planar. The vehicle searches at a fixed 6 m, which is above the
broken walls (≤ 5.2 m) and below the standing houses (7.5–9.5 m), the core
tower (12 m) and the chapel tower (16.2 m). `slam_bridge` levels each scan
with the attitude, drops returns that leave the flight plane by more than
0.8 m, and forwards scans only within ±0.7 m of 6 m. The map is therefore a
2D slice at flight height, not a 3D map.

### Navigator

- Planning grid 0.5 m, built from the 0.25 m SLAM map; obstacles inflated by
  1.75 m (keep-out), extra cost up to 4 m; unknown space is passable but
  costlier.
- A* with path shortcutting; a look-ahead point 3.5 m along the route is sent
  as the goal; replanning every 1 s and whenever the route becomes blocked.
- A goal inside a keep-out band is moved to the nearest free cell within 3 m.
  If the vehicle reaches that cell and the goal is still out of tolerance, or
  no route exists for 4 s, the target is reported as unreachable and the
  search planner skips it.
- Return to base is routed on the map until the home radius, then handed to
  the controller.
- Localisation check: the SLAM transform and scan input must be fresh (6 s),
  and the SLAM pose must not jump more than 2.5 m against PX4 within 2 s. On
  failure the navigator logs `LOCALIZATION LOST` and stops publishing goals;
  the controller's existing rule (no valid goal) then returns home.

### Safe return

`return_altitude` (18 m in the village): when the return starts more than 3 m
from home, the controller first climbs (ramped, to stay inside the vertical
speed limit), then travels, then lands. 18 m is above every structure in the
village. This path does not depend on SLAM, so it is the fallback when SLAM
or the navigator fails.

## 3. The village world

Generator: `scripts/village/make_village.py` (writes the worlds and their
truth files). 124 m × 128 m: base pad, streets, 23 building-scale structures,
24 trees, 7 walls, 2 poles; a collapsed multi-storey building in the centre
(standing core tower, broken walls, floor slabs, rubble) and a partly
collapsed house with a courtyard to its east.

| World | Content | Brief scenario |
|---|---|---|
| `village_A` | no survivors, no look-alikes | A reconnaissance |
| `village_B` | one survivor in an exposed room | B one survivor |
| `village_B2` | one survivor in a different room | B (position not reused) |
| `village_C` | three survivors in two buildings, five look-alikes | C |
| `village_D` | two survivors partly under leaning slabs, one look-alike | D occlusion |
| `village_F` | five look-alikes, no survivors | F false-positive control |
| any + `DEBRIS=1` | boxes released above the vehicle | E falling debris |
| fault injection (`KILL_AT`, `KILL_COMM`) | SLAM or navigator killed in flight | G safe termination |
| market square, degraded odometry | area with no tuning; SLAM under drift | H SLAM evaluation |

Each world exists twice: `worlds/village/` (shadows, for the visual demo) and
`worlds/village_fast/` (no shadows, for headless evaluation). Real-time factor
on this machine: 0.69 with shadows, 0.83 without
(`results/village/checks/sim_*.json`).

Survivor positions are in the truth files (`results/phase7/worlds/`), which
only the scoring scripts read. The mission knows named areas
(`missions/places_village.json`), never survivor coordinates.

## 4. Missions from a sentence

`missions/places_village.json` names four areas (collapsed building, damaged
house, disaster district, market square). "Village" means the disaster
district — both damaged buildings — not all 124 m × 128 m. The headline
sentence

> Search the village for survivors, prioritize the collapsed building, avoid
> falling debris, report survivor locations, and return to base.

is interpreted by the rule interpreter as: area = disaster district, start
nearest the collapsed building, debris protection on, report, return. A
mission is one rectangle; "prioritize" changes where the lawnmower starts.

## 5. Visualisation

`config/village.rviz` shows the SLAM map, the live scan, the planned route,
the flown path in both frames, mission markers and the camera image. All of
it is subscribed from the running system; `mission_viz_node.py` only
republishes existing topics as paths and markers.

## 6. Commands

```bash
# visual demo: Gazebo GUI + RViz + headline mission (VLM if GPU and Ollama are up)
scripts/village/run_demo.sh

# headless evaluation batch, then scoring
scripts/village/run_village_batch.sh all
scripts/village/run_village_batch.sh v2
scripts/village/evaluate_village.sh

# one mission from a sentence
PLACES=missions/places_village.json scripts/mission/run_instruction.sh "Search the collapsed building for survivors." village_B mytag

# Gazebo camera presets
scripts/village/view.sh overview
```

Performance profiles:

| Profile | How | Notes |
|---|---|---|
| Visual demo | `run_demo.sh` (`HEADLESS=0 VIZ=1`, shadows) | slowest |
| Headless evaluation | `run_village_batch.sh` (`village_fast`, no GUI) | about 6 min per flight |
| SLAM benchmark | `run_village_batch.sh slam` + `scripts/slam/evaluate_slam.py` | degraded odometry |
| Live VLM | `run_village_batch.sh vlm` | needs the NVIDIA GPU and Ollama with `gemma3:4b` |
