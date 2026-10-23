# Phase 4 — Trajectory prediction

**Status: DONE**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

For every confirmed track the system predicts the position up to 2 s ahead with a ballistic model and propagates the uncertainty. Three models were compared on the same recorded data.

## Key results

* RMSE to the true surface: 0.028 m at 0.1 s, 0.056 m at 0.5 s, 0.110 m at 1.0 s, 0.160 m at 1.5 s, 0.414 m at 2.0 s
* constant velocity at 1.0 s: 4.44 m; constant acceleration: 2.36 m
* six hover flights, 2346 predictions scored
* 0.12 ms per frame
* 24 unit tests

## Failures and limits

* only 41 of 2108 predictions at 2.0 s could be scored: the predictor has no map and predicts through structure
* free fall only (no drag, no bounce)

## Source

* [`debris_prediction.py`](../../../src/uav_autonomy/uav_autonomy/debris_prediction.py)
* [`debris_predictor.py`](../../../src/uav_autonomy/uav_autonomy/debris_predictor.py)
* [`test_debris_prediction.py`](../../../src/uav_autonomy/test/test_debris_prediction.py)

## Evidence

* [`docs/debris_prediction.md`](../../../docs/debris_prediction.md)
* [`results/phase4/README.md`](../../phase4/README.md)
* [`results/phase4/live_hover_evaluation.json`](../../phase4/live_hover_evaluation.json)
* [`results/phase4/offline_models_on_phase4_recordings.json`](../../phase4/offline_models_on_phase4_recordings.json)

## Figures

* [06_trajectory_prediction_view.png](../figures/06_trajectory_prediction_view.png)
* [07_prediction_error_vs_horizon.png](../figures/07_prediction_error_vs_horizon.png)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
