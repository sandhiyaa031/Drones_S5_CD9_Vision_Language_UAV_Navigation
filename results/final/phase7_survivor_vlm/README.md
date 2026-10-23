# Phase 7 — Survivor perception and VLM verification

**Status: ACCEPTED SIMULATION BASELINE**

This page is a summary with links. The original evidence stays where it was produced; nothing was moved or copied except the figures in [`../figures/`](../figures/).

## What was built

A deterministic colour/shape detector proposes survivor candidates in the downward camera image, a temporal filter requires 5 of 8 frames, and a local vision-language model (gemma3:4b through Ollama) decides whether the crop shows a person. Without a reachable VLM nothing is ever reported as confirmed. Simulation survivor-perception baseline validated; real-world generalization is future work.

## Key results

* detector, 11 live flights: TP 2016, FN 0, FP 773 → precision 0.7228, recall 1.0000, F1 0.8391; all false positives on one designed distractor cluster
* VLM on 175 held-out crops: non-survivors 98/100 rejected, 0 confirmed; clear survivors 24/45 confirmed (oblique view 24/32, straight-down 0/13)
* live with the VLM: 4 of 8 survivor instances SURVIVOR_CONFIRMED, 2.5–4.2 s after entering view; 1146 confirmed frames on a survivor, 0 on a non-survivor
* VLM latency p50 2.36 s (GPU); camera callback p50 2.1–3.9 ms (never waits for the VLM)
* without a VLM: 0 confirmations in 11 flights, status VLM_UNAVAILABLE
* 92 unit tests

## Failures and limits

* the detector is tuned to the simulated manikin's colours; it is not a general person detector
* the VLM rejects survivors seen from straight above
* an orange object touching a skin-coloured object scores as a strong candidate
* 2 of 11 flights in the first GPU batch failed safe from GPU contention; CPU-only VLM takes 44–82 s
* no 3D survivor position is produced (that is Phase 8, not started)

## Source

* [`survivor_detection.py`](../../../src/uav_autonomy/uav_autonomy/survivor_detection.py)
* [`survivor_confirmation.py`](../../../src/uav_autonomy/uav_autonomy/survivor_confirmation.py)
* [`vlm_client.py`](../../../src/uav_autonomy/uav_autonomy/vlm_client.py)
* [`survivor_perception_node.py`](../../../src/uav_autonomy/uav_autonomy/survivor_perception_node.py)
* [`test_survivor_perception.py`](../../../src/uav_autonomy/test/test_survivor_perception.py)
* [`scripts/survivor/`](../../../scripts/survivor/)

## Evidence

* [`docs/survivor_perception.md`](../../../docs/survivor_perception.md)
* [`results/phase7/README.md`](../../phase7/README.md)
* [`results/phase7/summary_vlm.md`](../../phase7/summary_vlm.md)
* [`results/phase7/summary_no_vlm.md`](../../phase7/summary_no_vlm.md)
* [`results/phase7/vlm_eval_heldout_gemma3_4b.json`](../../phase7/vlm_eval_heldout_gemma3_4b.json)
* [`results/phase7/size_study_detector.json`](../../phase7/size_study_detector.json)
* [`demo check of 2026-10-06 (no VLM)`](../demo_check/survivor_clear_intel_gpu_no_vlm/)

## Figures

* [15_survivor_confirmed_by_vlm.jpg](../figures/15_survivor_confirmed_by_vlm.jpg)
* [16_survivor_distractor_scene.jpg](../figures/16_survivor_distractor_scene.jpg)
* [17_survivor_outdoor_pass.jpg](../figures/17_survivor_outdoor_pass.jpg)
* [18_vlm_dataset_samples.jpg](../figures/18_vlm_dataset_samples.jpg)
* [20_demo_check_2026-10-06_survivor_candidate.jpg](../figures/20_demo_check_2026-10-06_survivor_candidate.jpg)

Full detail: [`../MASTER_RESULTS.md`](../MASTER_RESULTS.md) · numbers with sources: [`../METRICS_SUMMARY.md`](../METRICS_SUMMARY.md) · equations: [`../MATHEMATICAL_METHODS.md`](../MATHEMATICAL_METHODS.md)
