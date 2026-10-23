# Phase 0 — Flight foundation

**Status: DONE**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

The UAV arms, takes off to 3 m, holds for 30 s and lands by itself from a cold start, with one command. Every flight is checked against the PX4 flight log, not only against the controller's own messages. A flight safety monitor lands the vehicle on impact, estimator jump, speed violation or stale telemetry.

## Key results

* 5 of 5 hover flights pass every log check (final configuration)
* takeoff 6.69–7.27 s; hover altitude RMSE 0.070–0.096 m; drift 0.042–0.059 m; landing offset 0.015–0.059 m
* the stock waypoint route hit the roof 8.66 s after arming (kept as a failure); with the monitor the impact is detected at 8.6 s and PX4 AUTO LAND starts at 8.7 s
* 26 unit tests

## Failures and limits

* the monitor reacts after contact, it does not prevent it
* a crashed vehicle may stay armed (no automatic disarm, by decision)

## Source

* [`flight_controller.py`](../../../src/uav_autonomy/uav_autonomy/flight_controller.py)
* [`flight_safety.py`](../../../src/uav_autonomy/uav_autonomy/flight_safety.py)
* [`test_flight_safety.py`](../../../src/uav_autonomy/test/test_flight_safety.py)
* [`scripts/run_flight_trial.sh`](../../../scripts/run_flight_trial.sh)
* [`scripts/verify_ulog.py`](../../../scripts/verify_ulog.py)

## Evidence

* [`docs/runbook.md`](../../../docs/runbook.md)
* [`docs/flight_safety.md`](../../../docs/flight_safety.md)
* [`results/phase0/final_config/`](../../phase0/final_config/)
* [`results/phase0/final_config/hover_01/verification.json`](../../phase0/final_config/hover_01/verification.json)
* [`results/phase0/final_config/waypoint_01/ (roof collision)`](../../phase0/final_config/waypoint_01/)
* [`results/safety/`](../../safety/)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
