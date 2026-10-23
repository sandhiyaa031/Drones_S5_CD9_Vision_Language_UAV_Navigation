# Phase 7: survivor perception and VLM verification

**Simulation survivor-perception baseline validated; real-world
generalization is future work.** The candidate detector is built for the
appearance of the Gazebo manikin and says nothing about real people.
Results: `results/phase7/README.md`.

Scope: image-plane detection and semantic verification only. No 3D
localisation, no search planning, no mission logic, no PX4 commands.

## 1. Pipeline

```
/camera/image_raw (rgb8, 1280x960) ──► candidate detector        every frame, ~5 ms
/camera/camera_info                    (deterministic, colour + shape cues)
                                            │ candidates of this frame
                                            ▼
                                   temporal confirmation         stable tracks only
                                            │ crop of a stable track (event-based)
                                            ▼
                                   VLM worker thread ──HTTP──► OpenAI-compatible endpoint
                                            │ validated answer, or error / unavailable
                                            ▼
                    /perception/survivor/detections   uav_interfaces/SurvivorDetectionArray
                    /perception/survivor/debug_image  sensor_msgs/Image
                    /perception/survivor/status       std_msgs/String (JSON, logging only)
```

| File | Role |
|---|---|
| `uav_autonomy/survivor_detection.py` | candidate detector, image decoding (no ROS) |
| `uav_autonomy/survivor_confirmation.py` | tracks, temporal confirmation, semantic state, VLM request policy (no ROS) |
| `uav_autonomy/vlm_client.py` | request building, strict answer validation, background worker, endpoint status (no ROS) |
| `uav_autonomy/survivor_perception_node.py` | ROS node |
| `uav_interfaces/msg/SurvivorDetection.msg`, `SurvivorDetectionArray.msg` | interface |

## 2. Audit of the code that existed before this phase

| File | What it did | Problems | Now |
|---|---|---|---|
| `vlm_perception.py` | red HSV threshold on every 10th frame, logs "VLM ALERT: Survivor detected", publishes `std_msgs/String "SURVIVOR_FOUND"` | subscribes to `/camera` (the stream is `/camera/image_raw`); **there is no VLM in it**: a colour threshold is reported as a VLM alert; needs `cv_bridge`; no timestamp, no position; never an entry point | kept, unreferenced |
| `vlm_detector.py` | sends the whole camera frame to `VLM_ENDPOINT`, publishes `PoseStamped` | the HTTP call is made **inside the image callback** (blocks up to 10 s); sends full frames, not crops; payload format `{image, prompt}` matches no real API; `rgb8` frames are JPEG-encoded as if BGR (colours swapped); normalised image coordinates are published as a 3D position with a made-up `z = -2.0`; stamp is the node clock, not the frame time; never an entry point | kept, unreferenced |
| `survivor_perception.py` (earlier attempt) | orange HSV blobs as candidates, crops to a VLM from a worker thread, JSON on `std_msgs/String` | every orange blob is a candidate; no temporal confirmation; same non-standard payload; no availability state; `String` as the interface; needs `cv_bridge` | **reused ideas**: worker thread + queue, crop with padding, candidates and verification kept as separate outputs. Replaced by the new node; the file is kept, unreferenced |

The `survivor_perception` executable now starts the new node. None of the
three old files was validated against a real VLM, and none is claimed to
work.

## 3. Candidate detector (simulation baseline)

The manikin has orange clothing, a skin-coloured head and blue legs. In the
camera image shadowed clothing is dark brown-orange but stays saturated
(HSV ≈ 13, 212, 101); lit clothing is (13, 200, 223); the head is (17, 62,
100–193); the spawner's debris boxes are tan (19, 43, 171) and brown — less
saturated than the clothing.

Per frame, on a half-resolution copy (boxes are reported in full-resolution
pixels):

1. clothing mask: H 5–22, S ≥ 170, V ≥ 45; open/close 3×3;
2. connected regions; regions whose boxes are closer than half the smaller
   one's long side are merged (an occluder can cut a body in two);
3. cues for each region, looked for in its box grown by 45 %:
   * **head**: a skin-coloured blob (H 8–28, S 35–125, V ≥ 70) that touches
     the clothing, fills ≥ 45 % of its own box, with area 0.06–2.5 × the
     clothing area;
   * **shape**: minimum-area rectangle of the clothing has aspect ≥ 1.6, or
     the clothing fills ≤ 88 % of it (arms spread from the torso);
   * **legs**: blue (H 100–128, S ≥ 120) ≥ 3 % of the clothing area;
4. `confidence = 0.30 + 0.35·head + 0.20·shape + 0.15·legs`.

| | |
|---|---|
| ≥ 0.55 "strong" | person-like cue set (clothing plus head, or plus shape and legs) |
| 0.30–0.55 "weak" | an orange object of plausible size: possibly a partly hidden survivor, possibly just an orange object |
| minimum clothing area | 60 px² (full resolution) |

An orange box is therefore only a weak candidate, and skin-coloured or tan
objects alone are nothing. A known hard case remains by construction: an
orange object with a skin-coloured compact object touching it scores as
strong. That is what the VLM stage is for.

Image decoding: `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, with row padding;
any other encoding is refused (logged, counted, no output). No `cv_bridge`.

## 4. Temporal confirmation

| Parameter | Value | Meaning |
|---|---|---|
| `confirm_hits` / `confirm_window` | 5 / 8 frames | a candidate becomes a stable track when detected in 5 of the last 8 frames |
| `hold_s` | 1.0 s | a stable track survives this long without a detection, flagged `measured = false`, then it is dropped |
| `match_distance` | 0.75 × larger box side | frame-to-frame association by centre distance |
| `match_area_ratio` | 3.0 | a candidate continues a track only if its box area is within ×3 of the track's (added after a confirmed track hopped onto a small orange object next to the survivor for 0.5 s in a live run) |
| tentative drop | 4 misses in a row | a candidate that cannot reach 5 of 8 is discarded |

A track that is not stable is never published: a single-frame false candidate
produces no output. A candidate seen every other frame (4 of 8) is not
confirmed; one seen in 3 of 4 frames is. Frames whose stamp is not newer than
the previous frame are ignored. All ages and delays use the camera stamps.

## 5. VLM verification

* The worker thread sends **one crop at a time** (box padded by 25 %, long
  side scaled into 224–448 px, JPEG 90) to `<endpoint>/chat/completions` in
  the OpenAI-compatible format, temperature 0, JSON response format.
  Endpoint and model: parameters `vlm_endpoint`, `vlm_model` (defaults from
  `VLM_ENDPOINT`, `VLM_MODEL`); an API key, if any, only from the
  environment variable `VLM_API_KEY` (never logged, never a parameter).
* One verification is **two requests to the same model**:
  1. the crop with a neutral question — "describe the main object … and its
     visibility" — whose prompt mentions neither people nor the manikin's
     appearance (a unit test enforces this);
  2. the model's description, as text only: does it describe a person or a
     human-like figure? → `target`, `decision`, `confidence`.
* The combined answer is validated strictly (names, types, value sets):
  `{"target": str, "confidence": 0..1, "visibility": "clear|partial|poor",
  "decision": "confirm|reject|uncertain"}`; `confidence` is the model's
  confidence in its decision. Anything else is an **error**, not an answer.
  The model's description is logged with every answer.
* The camera callback only copies the crop into a one-slot queue; it never
  waits. Answers are applied on a later frame. A crop still waiting when a
  newer one arrives is replaced and reported as superseded.

Why two requests (measured with `gemma3:4b`, development crops only):

| Prompt design | Result on a development slice |
|---|---|
| one request: "is this a human or human-shaped manikin… confirm/reject" | rejected every crop, survivors included (while naming them "manikin") |
| one request with a description of what the manikin looks like | confirmed every crop, including bare floor and shadows (it repeated the description back) |
| one request, neutral wording, describe-then-decide in one JSON | accurate descriptions, but "reject" for every crop |
| one request, direct yes/no question | "yes" for all shadow crops |
| **two requests** (neutral description, then text-only decision) | separates survivors from non-survivors; adopted |

The second-step prompt was revised once more after an evaluation showed it
"confirming" descriptions such as "soccer ball" and "drone"; the crops used
for that became development data and the reported evaluation uses a later,
separate set (`results/phase7/README.md`).

Request policy (event-based, never per frame):

| Event | Request |
|---|---|
| track becomes stable | first verification |
| error, or answer "uncertain" | retry after `retry_interval_s` 2.0 s, at most `max_retries` 5 in a row |
| answered track | again every `reverify_interval_s` 10 s (0 = never), or at once when its box area has grown ×`reverify_area_gain` 2.0 (a better view) |
| always | at most one request in flight; ≥ `min_request_interval_s` 0.5 s apart; not for boxes under `min_crop_px` 16; not for a track that is not in view |

## 6. Semantic state

Per detection (`semantic_state`) and for the frame (`state`, the most
significant one present):

| State | When |
|---|---|
| `NO_TARGET` | no stable track (or only rejected ones) |
| `CANDIDATE` | stable track with a strong cue set, no usable VLM answer |
| `UNCERTAIN` | stable track with a weak cue set and no VLM answer; or VLM answered "uncertain"; or "confirm" below `vlm_confirm_confidence` 0.6; or one rejection after a confirmation; or **the VLM rejected a strong cue set** (detector and VLM disagree) |
| `SURVIVOR_CONFIRMED` | stable track **and** latest VLM answer "confirm" with confidence ≥ 0.6 |
| `REJECTED` (per detection) | VLM answered "reject" for a weak cue set (twice in a row if it had confirmed before) |

Why a rejected strong candidate is UNCERTAIN and not dropped: measured on the
held-out crops, the VLM rejects 44 % of clearly visible survivors (all of
those seen from straight above or from more than 4.5 m) while it never
confirmed a non-survivor. A "confirm" is therefore trustworthy, a "reject" is
not strong enough to erase a complete person-like cue set. The
`verification_status` still says `VLM_REJECTED`; nothing is ever confirmed
without a VLM confirmation.

`verification_status`: `NOT_REQUESTED`, `PENDING`, `VLM_CONFIRMED`,
`VLM_REJECTED`, `VLM_UNCERTAIN`, `VLM_UNAVAILABLE`, `VLM_ERROR`.

## 7. When the VLM is not there

| Endpoint status (`vlm_status`) | Meaning | Behaviour |
|---|---|---|
| `VLM_AVAILABLE` | the endpoint answered the probe or the last request | normal |
| `VLM_UNAVAILABLE` | no endpoint/model configured, or it cannot be reached (re-probed every 10 s) | candidates are still detected, followed and published as `CANDIDATE` / `UNCERTAIN` with `verification_status = VLM_UNAVAILABLE`, `vlm_confidence = -1`; no request is sent; **nothing is ever `SURVIVOR_CONFIRMED`** |
| `VLM_ERROR` | reachable, but the last request timed out, returned an HTTP error or a malformed answer | that track gets `VLM_ERROR` and is retried under the policy above; no answer is invented |

The detector alone is deterministic and needs nothing but the camera.

### VLM and simulator on one GPU

On this laptop the VLM and Gazebo's rendering share one 6 GB GPU. In the
first VLM batch, two of eleven flights ended in a flight-controller failsafe
("telemetry became stale"): an inference burst stalled the simulator for
more than the controller's 1 s telemetry guard. The guard was not changed.
A CPU-only variant of the same model (`gemma3-4b-cpu`, 8 threads) avoids GPU
contention at the cost of ~35 s per verification. This is a test-bench
constraint: on a real system the VLM does not share a GPU with a simulator.

## 8. Interface

`SurvivorDetectionArray`: header (camera frame stamp and frame id), `state`,
`vlm_status`, `vlm_model`, `processing_ms`, `detections[]`.
`SurvivorDetection`: header, `track_id`, bounding box and centre in pixels,
image size, `candidate_confidence`, `head_cue`, `legs_cue`,
`touches_border`, `hits`, `age`, `measured`, `semantic_state`,
`verification_status`, `vlm_confidence`, `vlm_visibility`, `vlm_label`,
`vlm_stamp` (camera time of the verified crop), `vlm_latency`,
`vlm_requests`, `source`. No 3D position is published in this phase.

The debug image is the camera frame at half size with this node's own boxes,
states, cue scores and VLM answers drawn on it. No ground truth is drawn.

## 9. No ground truth online

The node subscribes to `/camera/image_raw` and `/camera/camera_info` and to
nothing else. It imports no Gazebo transport module and does not read the
world file, entity poses, or the positions written by the test tooling.
`test_survivor_perception.py::test_online_code_has_no_ground_truth_input`
checks this (imports, topic names, file reads). Ground truth is used only by
the scoring scripts in `scripts/survivor/` (`truth.py`,
`evaluate_survivor.py`, `build_vlm_dataset.py`).

## 10. Running

```bash
# local VLM (installed in the user's home, not system-wide)
OLLAMA_KEEP_ALIVE=60m ~/ollama/bin/ollama serve
export VLM_ENDPOINT=http://127.0.0.1:11434/v1 VLM_MODEL=gemma3:4b

scripts/survivor/make_worlds.py                 # test worlds (once)
scripts/survivor/run_scenario.sh A_clear        # one live scenario
NO_VLM=1 scripts/survivor/run_scenario.sh A_clear novlm   # without a VLM
PYTHONPATH=src/uav_autonomy python3 scripts/survivor/evaluate_survivor.py results/phase7/live/A_clear
PYTHONPATH=src/uav_autonomy python3 scripts/survivor/build_vlm_dataset.py results/phase7/live/*_novlm
PYTHONPATH=src/uav_autonomy python3 scripts/survivor/evaluate_vlm.py --output results/phase7/vlm_eval.json
```

In any flight: `SURVIVOR=1 CAMERA_BRIDGE=1 scripts/run_flight_trial.sh ...`.
