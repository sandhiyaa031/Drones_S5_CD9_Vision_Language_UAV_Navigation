# Phase 6 and 6.1 — Autonomous debris avoidance

**Status: BOUNDED / PARTIAL (closed as a bounded engineering result)**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

A planner proposes an escape setpoint when an object is a threat; the flight controller validates and executes it and then resumes the mission. Only the flight controller commands PX4. Phase 6.1 added perception-health handling, speed caps and optional UAV motion models, and measured the envelope. An estimator diagnostic tested one hypothesis for the remaining failure.

## Key results

* Phase 6 (14 scenarios, each flown without and with avoidance): runs with a collision 11/14 → 5/14; missions completed 3/14 → 10/14
* 15 of 15 direct hits on a hovering UAV avoided, smallest separation 0.91 m; 0 of 22 non-threats hit; 1 unnecessary manoeuvre of 25
* warning lead 1.30 s median; start latency 0.068 s; planner 8.8 ms per cycle
* Phase 6.1: steady cruise at 2.9–4.4 m/s, 0 collisions in 8 flights; seeded runs 0 collisions in 24 releases
* depth gap hiding a new object: fine to 250 ms, marginal at 500 ms (0.64 m), collision at 750 ms
* minimum warning needed ≈ 0.9 s
* estimator diagnostic: GPS-delay setting changes the position lead (86.6 → 10.0 ms) but not the warning time (0.764 vs 0.788 s) or the outcome; configuration left at the PX4 default
* 75 unit tests

## Failures and limits

* hard-acceleration transit is **not solved** (collision in 1 of 2 flights in 6.1, 3 of 5 in 6)
* lateral throws with about 1 s of flight or less are not avoided
* no escape manoeuvre exists inside the roof opening (reported as no-escape, by design)
* acceleration-aware UAV motion models gave no earlier warning (negative result)
* deviations are larger than strictly necessary

## Source

* [`debris_avoidance.py`](../../../src/uav_autonomy/uav_autonomy/debris_avoidance.py)
* [`debris_avoidance_node.py`](../../../src/uav_autonomy/uav_autonomy/debris_avoidance_node.py)
* [`avoidance_interface.py`](../../../src/uav_autonomy/uav_autonomy/avoidance_interface.py)
* [`flight_controller.py`](../../../src/uav_autonomy/uav_autonomy/flight_controller.py)
* [`test_debris_avoidance.py`](../../../src/uav_autonomy/test/test_debris_avoidance.py)
* [`scripts/avoidance/`](../../../scripts/avoidance/)

## Evidence

* [`docs/debris_avoidance.md`](../../../docs/debris_avoidance.md)
* [`results/phase6/README.md`](../../phase6/README.md)
* [`results/phase6/comparison.md`](../../phase6/comparison.md)
* [`results/phase6/comparison.json`](../../phase6/comparison.json)
* [`results/phase6/plots/`](../../phase6/plots/)
* [`results/phase6_1/README.md`](../../phase6_1/README.md)
* [`results/phase6_1/lead_vs_outcome.json`](../../phase6_1/lead_vs_outcome.json)
* [`results/phase6_1_estimator_diagnostic/README.md`](../../phase6_1_estimator_diagnostic/README.md)
* [`demo check of 2026-10-06`](../demo_check/avoidance_direct_intel_gpu/)

## Figures

* [09_avoidance_direct_threats.png](../figures/09_avoidance_direct_threats.png)
* [10_avoidance_multiple_threats.png](../figures/10_avoidance_multiple_threats.png)
* [11_avoidance_failure_hard_acceleration_transit.png](../figures/11_avoidance_failure_hard_acceleration_transit.png)
* [12_avoidance_failure_lateral_throw.png](../figures/12_avoidance_failure_lateral_throw.png)
* [13_avoidance_cruise_4p4_m_s.png](../figures/13_avoidance_cruise_4p4_m_s.png)
* [14_avoidance_with_500ms_depth_gap.png](../figures/14_avoidance_with_500ms_depth_gap.png)
* [19_demo_check_2026-10-06_avoidance.png](../figures/19_demo_check_2026-10-06_avoidance.png)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
