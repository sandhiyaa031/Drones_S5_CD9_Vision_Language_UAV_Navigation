# Phase 5 — Collision-risk estimation

**Status: DONE (advisory output)**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

For each tracked object the system computes the closest approach to the UAV, the time to it, and a state SAFE / WATCH / WARNING / CRITICAL (plus POTENTIAL_THREAT for unconfirmed fast objects).

## Key results

* offline, 281 objects, free fall: precision 0.61, recall 0.954; 10 of 10 true collisions CRITICAL
* whole life: precision 0.67, recall 0.79
* live scenarios A–D: precision 0.81, recall 0.87
* warning lead time median 0.80 s, 5th percentile 0.45 s
* 35 unit tests

## Failures and limits

* 15 of the 18 whole-life misses were boxes deflected by structure or another box (not predictable with a free-fall model)
* the lateral hit reached only POTENTIAL_THREAT and struck the UAV
* about 18 % of safe objects are flagged (conservative)
* sphere safety volume; uncertainty is an empirical envelope

## Source

* [`debris_risk.py`](../../../src/uav_autonomy/uav_autonomy/debris_risk.py)
* [`debris_risk_node.py`](../../../src/uav_autonomy/uav_autonomy/debris_risk_node.py)
* [`test_debris_risk.py`](../../../src/uav_autonomy/test/test_debris_risk.py)

## Evidence

* [`docs/debris_risk.md`](../../../docs/debris_risk.md)
* [`results/phase5/README.md`](../../phase5/README.md)
* [`results/phase5/offline_recorded_hover.json`](../../phase5/offline_recorded_hover.json)
* [`results/phase5/live_all_hover.json`](../../phase5/live_all_hover.json)

## Figures

* [08_collision_risk_traces.png](../figures/08_collision_risk_traces.png)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
