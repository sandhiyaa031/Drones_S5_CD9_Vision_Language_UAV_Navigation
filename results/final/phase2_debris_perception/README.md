# Phase 2 — 3D debris perception

**Status: DONE**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

Falling objects are detected in the upward depth image and reported as 3D positions in the PX4 local frame, using the vehicle pose interpolated at the image time. Static structure is filtered out. No simulator ground truth is used online.

## Key results

* 86 of 86 objects detected at least once
* frame-level recall 98.7 % (1713 of 1736)
* 0 of 1852 detections without a true object within 0.35 m
* local-frame surface error p95 ≤ 0.160 m, max 0.244 m
* 2.5–7.7 ms per frame, about 24 Hz
* 21 unit tests

## Failures and limits

* overlapping boxes at the same range merge into one detection
* 113 detections of boxes cut by the image border have a biased position (flagged)
* the reported point is the visible surface, not the centre (≈ 0.16 m)
* depth stream gaps up to 0.53 s were observed
* in one run a box struck the hovering UAV — nothing avoided debris at this phase

## Source

* [`debris_detection.py`](../../../src/uav_autonomy/uav_autonomy/debris_detection.py)
* [`debris_depth_detector.py`](../../../src/uav_autonomy/uav_autonomy/debris_depth_detector.py)
* [`test_debris_detection.py`](../../../src/uav_autonomy/test/test_debris_detection.py)

## Evidence

* [`docs/debris_perception.md`](../../../docs/debris_perception.md)
* [`results/phase2/README.md`](../../phase2/README.md)
* [`results/phase2/summary.json`](../../phase2/summary.json)
* [`results/demo/detector_view_seed11.mp4`](../../demo/detector_view_seed11.mp4)

## Figures

* [05_debris_detection_debug_view.jpg](../figures/05_debris_detection_debug_view.jpg)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
