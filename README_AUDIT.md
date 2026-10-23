# README audit

Audit of the repository for the project report ([README.md](README.md)), 2026-10-23. Nothing in the implementation was
modified; no commit or push was made.

## What was done

* Read the code (all node and logic modules, `flight_controller.py` in full, scripts, configs, message definitions), the
  project's own documents (`docs/`, `results/final/`, `MULTI_SIM_PROGRESS.md`), result JSON files, figures, and the
  base-paper PDF.
* Re-derived the interface tables from `create_publisher` / `create_subscription` calls; spot-checked constants in the
  tracker, risk, avoidance, localisation, search and safety code against the math in the project documents.
* Ran the unit tests: **552 passed, 6 skipped, 2 failed** (flake8 and pep257 style tests) with ROS and the workspace
  sourced. Without sourcing `install/setup.bash`, nine `test_survivor_perception.py` tests error on a missing
  `uav_interfaces` module (environment, not a code fault).
* Recomputed the toy-example numbers by calling `lawnmower`, `footprint_half_extents`, `project`, `ray_plane`,
  `triangulate`, `radii`, `linear_closest_approach` and `simulate_response`.
* Checked all local links/images, all external URLs (HTTP 200), and both Mermaid diagrams (parsed and rendered with
  Mermaid 10). LaTeX could not be rendered locally; it uses only common MathJax commands.
* Did **not** re-fly any simulation, rebuild the workspace, or re-run the evaluation scorers.

## Placement decisions

* The Git repository root is `ros2_ws/` (remote `sandhiyaa031/Drones_S5_CD9_Vision_Language_UAV_Navigation`), so the
  report is `ros2_ws/README.md` — GitHub renders it and all relative image links resolve. A short pointer file was
  created at `~/Desktop/drones/README.md`.
* The previous README (concept stage) was **preserved** unchanged as
  [docs/README_original_concept.md](docs/README_original_concept.md); the team table and logo were taken from it.
* The architecture figure was **copied** from `~/Desktop/drones/deliverables/` into `docs/figures/` (the originals remain).

## Information you must supply

* Faculty guide (name, designation, e-mail), department, campus, academic year, course — marked `TODO(team)` in §1.
* Confirm the team names, roll numbers and e-mails (copied from the old README).
* A decision on whether to include real screenshots/videos: the repository has **no GIFs or videos**.

## Could not be verified

* **Phases 0–5, 6.1 and sensor-study evidence.** `results/phase0…phase5`, `phase6_1`, `safety`, `sensor_study`,
  `sensor_integration`, `time_sync`, `demo/*.mp4` are not on disk (`results/phase6` and `phase7` hold only `plans/` and
  `worlds/`). Their numbers in the report are quoted from `results/final/MASTER_RESULTS.md` and tagged as such; only the
  figures in `results/final/figures/` are present.
* Real-world behaviour (nothing flown), QGroundControl use (no script or log uses it), the NVIDIA/VLM path on today's
  kernel (`7.0.0-34`, `nvidia-smi` fails), a clean-machine installation, the `colcon build` step.
* Why the urban VLM runs in `results/faculty_vlm/` logged "confirm" events but no `SURVIVOR_CONFIRMED` report label.
* Thrust/mass values are from PX4's `x500_base` SDF; the project's added cameras' mass is not included.

## Claims removed, corrected or qualified

* "GPS-denied" (old README title): **not implemented**; the system uses PX4 EKF2 with simulated GNSS. The report says so.
* ORB-SLAM3, Grounding DINO, semantic memory, RRT/RRT\*, QGroundControl-based monitoring: concept-stage items from the old
  README; not built; not claimed.
* "Only `flight_controller` publishes to `/fmu/in/*`": true for all executables registered in `setup.py`; the legacy,
  unreferenced prototype `rescue_mission.py` (plus `.backup`) also publishes there. Qualified in §6.
* Test count "315" (in `results/final/*`) is outdated: 552 now.
* `results/final/README.md`, `PHASE_STATUS.md`, `ARCHITECTURE.md` still say Phases 8–15 are "not started" and omit the
  navigator, SLAM, mission manager, village and urban worlds. They are out of date (dated 2026-10-06); the report uses the
  later per-phase documents and result files instead.
* MuJoCo: results are reported separately and labelled as not comparable (ground-truth state, no VLM/SLAM/avoidance/failsafes).
* `docs/village_system.md` refers to a "section 9" listing failures that does not exist; failures were taken from
  `results/village/summary_missions.json`. Docs that cite `results/demo/*.mp4` refer to missing files.
* Live VLM confirmation in the urban world is **not** claimed.

## Repository state that affects the report

* Many files are modified or untracked in Git (80 entries): `src/uav_interfaces/`, `missions/`, `docs/`, all
  later nodes (`navigation.py`, `mission_*`, `search_*`, `slam_*`, …), most scripts, village/urban worlds and results.
  A fresh clone of the current GitHub history **would not build** (`uav_interfaces` is untracked). Commit before sharing.
* `results/` is 8.1 GB locally; raw logs are git-ignored. Decide what to commit; figures used by the report that are
  untracked and will not render on GitHub until committed: `results/disaster_gazebo/screenshots/…`,
  `results/faculty/slam_map_vs_truth.png` (the `results/final/figures/*` images are tracked).
* `PROJECT_CONTEXT.md`, the long study guide and `MULTI_SIM_PROGRESS.md` (original) sit in `~/Desktop/drones/`, **outside**
  the repository; the base-paper PDF (35 MB) is also outside it.
* Two style-lint tests fail (281 flake8 findings).

## Recommended manual checks

1. Open `README.md` on GitHub (after committing) and check the LaTeX, the two Mermaid diagrams and all images.
2. Fill the `TODO(team)` fields; confirm team data.
3. On a clean machine, follow §10 once and note any missing dependency.
4. Boot kernel `7.0.0-30` and re-run one live-VLM mission before the review, if you intend to show VLM confirmation.
5. Decide whether to refresh or archive the stale `results/final/*` documents.
6. Optionally record a short screen capture of one mission — none exists in the repository.
