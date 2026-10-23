# Limitations and failures

Every item was observed in a test or flight; the evidence is named. For each:
what happened, how it was detected, root cause, whether it was fixed, the
current mitigation, and whether it remains a limitation. Paths are relative
to `ros2_ws/`.

Scope note: everything in this project is simulation (PX4 SITL + Gazebo).
The repository's title mentions GPS-denied navigation; the system as built
uses PX4's EKF2 with a simulated GNSS receiver. **GPS-denied navigation is
not implemented.**

---

## A. Flight foundation and simulation

### A1. Stock waypoint route collides with the roof
1. **What happened**: the controller's hard-coded waypoint route at 3 m flew
   into a roof slab 8.66 s after arming; the controller still reported
   "FLIGHT COMPLETE".
2. **Detected by**: the offline PX4-log check (`verify_ulog.py`), not by the
   controller. `results/phase0/final_config/waypoint_01`.
3. **Root cause**: the world is only collision-free straight up through the
   roof opening; the route and the controller knew nothing about structure.
4. **Fixed**: partly. A flight safety monitor was added: it detected the same
   collision live (impact at 8.6 s, AUTO LAND at 8.7 s). The route itself
   was not changed.
5. **Mitigation**: fly at 5 m (above the roof) with configurable waypoints;
   safety monitor always active.
6. **Remains**: detection is after contact; nothing plans around structure
   (that is the search planner, Phase 9).

### A2. A crashed vehicle can stay armed
1. PX4 never declared "landed" after a crash onto rubble (22 s and 78 s
   observed) and stayed armed.
2. Live tests in `results/safety/`.
3. The vehicle rests tilted or obstructed; PX4's land detector does not
   trigger.
4. **Not fixed, by decision** (2026-10-03): no automatic force-disarm.
5. Response is `impact → FAILSAFE → PX4 AUTO LAND`.
6. **Remains**: post-crash disarm is a future safety-design item.

### A3. Simulator performance depends on the NVIDIA driver
1. After a reboot onto kernel 7.0.0-34 the NVIDIA module no longer loads;
   Gazebo renders on the Intel GPU at real-time factor 0.90–0.94 and
   27–28 Hz, below the 0.95 / 28 Hz targets.
2. Benchmark refuses to run; `results/sensor_integration/v07`, `v08`.
3. Prebuilt NVIDIA modules exist only for kernel 7.0.0-30.
4. Not fixable from inside the project (system change, owner's decision).
5. Boot kernel 7.0.0-30. The demos still run on the Intel GPU (checked
   2026-10-06, `results/project_progress/demo_check/`).
6. **Remains** an environment dependency. The machine was in this state
   again on 2026-10-06.

### A4. "30 s hold" log check fails when the simulator runs slow
1. `verify_ulog.py` reports 26–29.8 s in the altitude band instead of 30 s.
2. Several runs (e.g. `results/time_sync/`, Phase 6 `A_none`).
3. The controller holds 30 s of wall-clock time; below real-time factor 1
   less simulated time passes.
4. Not changed (the threshold was deliberately not relaxed).
5. Reported whenever it occurs.
6. **Remains** a known artefact of the check.

### A5. Earlier simulator problems (fixed)
* Real-time factor 0.41 in the original world (a non-static model
  interpenetrating static geometry) → new clean world.
* Gazebo server out of memory after ~100 s (PX4's `GstCameraSystem` plugin,
  107 MiB/s) → project `gz_server.config` without it.
* PX4 timestamps on ROS drifting from simulation time by seconds →
  `UXRCE_DDS_SYNCT=0` per run (0 ms error). Simulation-only; **not** a clock
  synchronisation solution for real hardware.

---

## B. Debris detection (Phase 2)

### B1. Merged objects
1. Two boxes overlapping in the image at nearly the same range are reported
   as one detection; all missed object-frames examined (23 of 1736) were
   this case; one object was first detected 0.33 s late.
2. Offline scoring against ground truth, `results/phase2/`.
3. Connected components split only on a depth jump > 0.30 m.
4. Not fixed.
5. The tracker inflates measurement noise for merged regions and holds
   tracks for up to 0.6 s.
6. **Remains.**

### B2. Partial detections at the image border
1. 113 detections were of boxes only partly in view; their position is the
   centroid of the visible part, biased toward the image centre.
2. Same evaluation.
3. Geometry of the field of view.
4. Flagged (`touches_border`), not corrected.
5. Tracker uses 0.30 m measurement noise for them and does not confirm a
   track from border-only views.
6. **Remains**, and it is the cause of C2 and E2 below.

### B3. Field of view and depth-stream gaps
1. Objects approaching below the 100° upward cone are never seen (3 lateral
   throws). The ROS depth stream delivers ~24 of 30 frames per second with
   gaps up to 0.26–0.53 s (0.76 s seen once).
2. Detection evaluation; stream timing logs.
3. Zenith-only camera; bridge/rendering stalls, especially when an object is
   created in the simulator.
4. Not fixed.
5. Tracker tolerates gaps; Phase 6.1 added a perception-health state.
6. **Remains.** See E5.

### B4. Noise-free sensor; position is the visible surface
The simulated depth has no noise model (errors at float precision), so all
thresholds are untested against real sensor noise. The detection point is
the visible face, about 0.16 m from the box centre. Both **remain**.

---

## C. Tracking (Phase 3)

### C1. Duplicate tracks from merged objects
Three duplicate confirmed tracks appeared in two runs when boxes fell within
~0.5 m of each other (`docs/debris_tracking.md`). IDs were kept (0 ID
switches). Not fixed. **Remains.**

### C2. Objects seen only at the image edge are never confirmed
4 of 8 objects in the lateral run stayed TENTATIVE. By design (confirmation
needs 3 full views); the consequence is that they reach only
POTENTIAL_THREAT in Phase 5. **Remains.**

### C3. Nothing valid after impact
A track keeps falling ballistically for up to 6 frames after the real object
has hit something: velocity errors of 6–15 m/s in those frames. No bounce or
contact model. Tracks are deleted after 1.0 s without a measurement.
**Remains.**

### C4. Covariance is not calibrated
Innovation consistency (NIS) mean 0.03–0.13 against an expected 3: the
stated uncertainty is far larger than the actual error. Handled downstream
by capping the uncertainty margin with a measured envelope. **Remains.**

---

## D. Prediction and risk (Phases 4, 5)

### D1. Free fall only; no map
1. Of 2108 predictions at the 2.0 s horizon only 41 could be scored: in the
   rest the object had already hit the ground, roof or UAV. The predictor
   predicts straight through structure.
2. `docs/debris_prediction.md`.
3. No drag, bounce or map by design.
4. Not fixed.
5. Risk ignores predictions below the takeoff plane; the planner keeps the
   UAV out of structure zones.
6. **Remains.**

### D2. Deflections off structure are not predicted
15 of 18 whole-life false negatives offline were boxes that bounced off the
roof edge or another box toward the UAV (`docs/debris_risk.md`). Decision:
handle with structural exclusion zones in the planner, not in the predictor.
**Remains.**

### D3. Conservative false alarms
About 18 % of safe objects are flagged; 37 of 39 passed 0.70–0.90 m from the
airframe. By construction (safety radius measured from the tracked surface
point). **Remains**, accepted.

### D4. Short warning time
Median lead 0.80 s, 5th percentile 0.45 s. Objects released less than ~5 m
above the UAV give less warning than a manoeuvre needs. **Remains**; it is
the root of E1.

---

## E. Avoidance (Phases 6, 6.1)

### E1. Warning-window limit
1. Every collision with avoidance active had a lead time ≤ 0.70 s; every
   threat with ≥ 0.92 s was avoided (44 hover manoeuvres).
2. Phase 6 and 6.1 scoring, `results/phase6_1/lead_vs_outcome.json`.
3. Physics of the vehicle: a 4 m setpoint step moves it 0.94 m in 0.8 s and
   1.56 m in 1.0 s; about 1.2 m is needed.
4. Not fixable by the planner.
5. Best-effort fallback (AVOIDANCE_FAILSAFE); safety monitor lands the
   vehicle after an impact.
6. **Remains**: minimum warning ≈ 0.9 s.

### E2. Short lateral throws
Thrown debris with 1.0 s of flight: collision 3 of 3; 0.93 s: collision in
all 5 attempts. It enters from the side, is confirmed late (C2), lead
0.3–0.65 s. Defensive preparation on POTENTIAL_THREAT made no measurable
difference. **Remains.**

### E3. Hard-acceleration transit
1. On a 6 m setpoint step (≈ 6 m/s², peak 4.4 m/s) a box aimed ahead of the
   UAV hit it in 3 of 5 flights (Phase 6) and 1 of 2 (Phase 6.1).
2. Scoring; frame-by-frame diagnosis against ground truth.
3. **Root cause**: while the UAV accelerates hard, the *measurement* of the
   debris drifts: PX4's position estimate leads the truth by up to 0.45 m
   and the sensor-to-object vector is off by up to 0.46 m at 10 m range
   (attitude estimate). A vertically falling box gets a false sideways
   velocity of ~1.1–1.4 m/s, so the alert comes ~0.75 s before impact.
4. **Not fixed.** Acceleration-aware UAV motion models were added and
   measured: no earlier warning.
5. Steady cruise is safe up to 4.4 m/s (0 collisions in 8 flights);
   search-speed legs pass (12 of 12).
6. **Remains**: avoid hard acceleration during mission legs. Not measured:
   threats during the acceleration phase of a speed-capped leg.

### E4. Estimator diagnostic conclusion
1. Hypothesis: PX4 relabels each simulated GPS fix as 110 ms old
   (`SENS_GPS0_DELAY`), although its actual latency is 0 ± 5 ms.
2. Source inspection plus one paired flight (110 ms vs 0 ms), parameters
   restored and verified. `results/phase6_1_estimator_diagnostic/`.
3. Confirmed for the **position** part only: lead 86.6 → 10.0 ms, position
   error −70 %. Tilt error (3.4° peak) and sensor-to-object vector error
   (0.45 vs 0.46 m) unchanged.
4. Warning lead time unchanged (0.764 vs 0.788 s); collision in both arms.
5. The investigation was stopped there, as agreed. PX4 configuration left at
   its default.
6. **Remains**: the cause of the attitude/vector error under hard
   acceleration is not identified.

### E5. Depth gaps (stale perception)
A 0.76 s natural gap just after a release cost one direct hit (Phase 6,
seed 22). Injected gaps: 100/250 ms fine, 500 ms no collision but margin
down to 0.64 m, 750 ms collision. A gap after the threat has been seen is
harmless. Phase 6.1 added FRESH/DEGRADED/STALE health, speed caps and a
DEFENSIVE state. **Remains**: a gap > ~0.5 s that hides a new object cannot
be compensated.

### E6. No-escape structural cases
Inside the roof opening every sideways candidate is excluded by structure and
neither climbing nor descending clears the box. The planner reports
AVOIDANCE_FAILSAFE and commands nothing; the box hits; the safety monitor
lands the vehicle. One earlier version climbed toward the debris for a
predicted 14 mm gain (fixed: leave the mission setpoint only for ≥ 0.2 m
gain). **Remains** by design: reported, not hidden.

### E7. Other avoidance findings
* First response model was up to 1.2 m wrong for a moving UAV (fixed, refit;
  now ≤ 0.25 m at 1 s).
* Candidate setpoints alternated during a manoeuvre (fixed: commitment).
* Deviations are ~3 m where ~1.5 m would do (cost weights not tuned).
  **Remains.**
* The stock waypoint mission requested landing while still moving and landed
  on the roof (worked around with `arrival_max_speed`). Mission logic is
  Phase 10.

---

## F. Survivor perception and VLM (Phase 7)

### F1. Colour/shape baseline
1. The detector finds the manikin by orange clothing, a skin-coloured head
   and blue legs. All 773 false positives of the final run are one designed
   distractor: an orange slab with a skin-coloured sphere touching it, next
   to an orange box.
2. Distractor scenarios, `results/phase7/summary_vlm.json`.
3. It is a colour/shape detector by construction.
4. An earlier version fused separate nearby objects into one "person"
   (495 of 512 frames): fixed (cues must touch the clothing region). The
   touching case is not fixable by colour.
5. The VLM rejects it; the published state is UNCERTAIN, not confirmed.
6. **Remains.** Not a general person detector; real-world generalisation is
   future work.

### F2. VLM viewpoint limitation and false rejections
1. On held-out crops the VLM (`gemma3:4b`) confirmed 24 of 45 clearly
   visible survivors and rejected 20. Seen obliquely: 24 of 32 confirmed;
   seen from straight above: 0 of 13. It describes the top view as "a
   rocket", "a drone with orange wings", "an egg". Live: 4 of 8 survivor
   instances confirmed.
2. `results/phase7/vlm_eval_heldout_gemma3_4b.json`, live flights.
3. A small model and a blocky manikin; from above the figure does not look
   human. Not image size: oblique crops are confirmed down to 30 px.
4. Not fixed.
5. A VLM rejection of a strong candidate is reported as UNCERTAIN (with
   `VLM_REJECTED` kept), never silently dropped; nothing is confirmed
   without a VLM confirmation.
6. **Remains.** A search pattern would need an oblique look to confirm.

### F3. Prompt sensitivity of the small VLM
Four single-prompt designs failed on development crops (rejected everything,
or confirmed everything including bare floor). Replaced by two requests:
neutral description, then text-only decision. One questionable confirmation
remained in the final live run (a close-up crop labelled as the orange
cylinder during landing). `docs/survivor_perception.md`. **Remains** a
property of the model.

### F4. GPU contention
1. In the first VLM batch 2 of 11 flights ended in a controller failsafe
   ("telemetry became stale").
2. Controller log.
3. The VLM and Gazebo share one 6 GB GPU; an inference burst stalled the
   simulator for more than the controller's 1 s telemetry guard.
4. The guard was **not** weakened.
5. The next 22 GPU-VLM flights completed.
6. **Remains** a test-bench risk on this laptop.

### F5. CPU-only VLM latency
44 s per verification in one run, 82 s in another; in the final run it did
not finish verifying the survivor within a 75 s hover. **Remains**: the CPU
variant is not usable for confirmation during short passes.

### F6. Other Phase 7 findings
* A confirmed track hopped onto a nearby orange cylinder for 10 frames
  (fixed: association also requires a consistent box size).
* The no-VLM evidence batch was flown before two later changes (association
  size check, rejection rule); neither touches the detector or the no-VLM
  path.
* Only one debris crop in the held-out VLM set; lighting varied only as
  shadow versus direct sun.

---

## G. Process failures (for completeness)

* Two PX4 logs of flights that never armed were deleted together with their
  `failed_start_*` folders during an approved cleanup on 2026-10-04,
  although ULogs were meant to be kept. They cannot be recovered. Recorded
  in `results/phase5/README.md`.
* The two perception chains (debris avoidance; survivor + VLM) have never
  been flown together in one flight.

---

## H. Not implemented (Phases 8–15)

Survivor 3D localisation, search planner, mission state machine,
natural-language mission interpretation, end-to-end mission, system
evaluation, failure testing, final demonstration. None of these exists; no
claim is made about them.
