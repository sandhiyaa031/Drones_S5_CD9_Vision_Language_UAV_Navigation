# Phase 3 — Debris tracking

**Status: DONE**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

Each falling object gets a persistent identity and a velocity estimate from a Kalman filter that knows about gravity. Detections are assigned to tracks with a Mahalanobis gate and the Hungarian method.

## Key results

* 0 ID switches in 7 live runs
* position RMSE to the true surface 0.000–0.073 m; velocity RMSE 0.07–0.43 m/s
* vertical velocity bias −0.04 to +0.03 m/s (without gravity in the model: −1.1 m/s)
* confirmation delay median 0.07–0.18 s; at most 2.7 ms per frame
* 42 unit tests

## Failures and limits

* 3 duplicate confirmed tracks from merged objects
* objects seen only at the image border stay tentative (4 of 8 lateral throws confirmed)
* after an impact the coasting track is wrong for a few frames
* no drag or bounce model

## Source

* [`debris_tracking.py`](../../../src/uav_autonomy/uav_autonomy/debris_tracking.py)
* [`debris_tracker.py`](../../../src/uav_autonomy/uav_autonomy/debris_tracker.py)
* [`test_debris_tracking.py`](../../../src/uav_autonomy/test/test_debris_tracking.py)

## Evidence

* [`docs/debris_tracking.md`](../../../docs/debris_tracking.md)
* [`results/phase3/README.md`](../../phase3/README.md)
* [`results/phase3/summary_live.json`](../../phase3/summary_live.json)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
