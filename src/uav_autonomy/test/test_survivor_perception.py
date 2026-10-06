"""Unit tests: survivor candidate detector, temporal confirmation, VLM client.

The VLM tests talk to a small HTTP server started inside the test. It is a
protocol stand-in used to exercise the client's success, error, timeout and
malformed-answer paths; it is not a VLM and proves nothing about one.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import pytest

from uav_autonomy.survivor_confirmation import (
    CANDIDATE, ConfirmationParams, NO_TARGET, NOT_REQUESTED, PENDING,
    REJECTED, SURVIVOR_CONFIRMED, SurvivorMonitor, UNCERTAIN,
    VLM_CONFIRMED, VLM_ERROR, VLM_REJECTED, VLM_UNAVAILABLE, VLM_UNCERTAIN,
    VlmAnswer, semantic_state)
from uav_autonomy.survivor_detection import (
    Candidate, DetectorParams, detect, to_bgr)
from uav_autonomy.vlm_client import (
    DECIDE_PROMPT, DESCRIBE_PROMPT, VLM_AVAILABLE, VlmConfig,
    VlmFormatError, VlmWorker, build_decision_request, build_request,
    crop_region, encode_crop, parse_answer, parse_description, request_once)
from uav_autonomy import vlm_client

# colours as the camera renders them (BGR)
ORANGE = (17, 53, 101)          # shadowed clothing
ORANGE_SUN = (46, 125, 222)
SKIN = (146, 173, 193)
BLUE = (110, 40, 10)
GREY = (120, 120, 120)
TAN_DEBRIS = (142, 160, 171)
BROWN_DEBRIS = (51, 87, 122)


def scene(width=640, height=480, colour=GREY):
    return np.full((height, width, 3), colour, np.uint8)


def draw_manikin(img, cx, cy, size=100, head=True, legs=True,
                 clothing=ORANGE):
    """Top-down manikin: arms bar + torso, head disc, legs below."""
    arm, torso = int(size * 0.5), int(size * 0.16)
    cv2.rectangle(img, (cx - arm, cy - torso), (cx + arm, cy + torso),
                  clothing, -1)
    cv2.rectangle(img, (cx - torso, cy), (cx + torso, cy + int(size * 0.3)),
                  clothing, -1)
    if legs:
        cv2.rectangle(img, (cx - torso, cy + int(size * 0.3)),
                      (cx + torso, cy + int(size * 0.45)), BLUE, -1)
    if head:
        cv2.circle(img, (cx, cy - int(size * 0.12)), int(size * 0.16), SKIN,
                   -1)
    return img


def cand(x=100, y=100, w=80, h=60, conf=1.0, head=True):
    return Candidate(x, y, w, h, conf, head, True, True, False, w * h * 0.5)


# -- candidate detector ------------------------------------------------------

def test_valid_survivor_candidate():
    found = detect(draw_manikin(scene(), 320, 240), DetectorParams(scale=1.0))
    assert len(found) == 1
    c = found[0]
    assert c.head and c.shape and c.legs and c.confidence == 1.0
    assert not c.touches_border
    u, v = c.center
    assert abs(u - 320) < 15 and abs(v - 250) < 25
    assert 90 <= c.width <= 110


def test_boxes_are_in_full_resolution_pixels_at_reduced_scale():
    img = draw_manikin(scene(1280, 960), 800, 500, size=200)
    full = detect(img, DetectorParams(scale=1.0))[0]
    half = detect(img, DetectorParams(scale=0.5))[0]
    assert abs(half.x - full.x) <= 4 and abs(half.width - full.width) <= 6
    assert abs(half.center[0] - 800) < 20


def test_no_candidate_in_plain_scene():
    img = scene()
    cv2.rectangle(img, (0, 0), (300, 200), (75, 75, 75), -1)     # roof
    cv2.rectangle(img, (300, 200), (640, 480), (198, 198, 198), -1)
    assert detect(img) == []


def test_debris_boxes_are_not_candidates():
    img = scene()
    cv2.rectangle(img, (100, 100), (160, 160), TAN_DEBRIS, -1)
    cv2.rectangle(img, (300, 200), (370, 270), BROWN_DEBRIS, -1)
    cv2.rectangle(img, (450, 300), (500, 350), (92, 90, 87), -1)
    assert detect(img, DetectorParams(scale=1.0)) == []


def test_orange_box_is_only_a_weak_candidate():
    # False candidate: an orange object without head, arms or legs.
    img = scene()
    cv2.rectangle(img, (250, 180), (330, 260), ORANGE_SUN, -1)
    found = detect(img, DetectorParams(scale=1.0))
    assert len(found) == 1
    c = found[0]
    assert not c.head and not c.legs and not c.shape
    assert c.confidence == pytest.approx(0.30)
    assert c.confidence < DetectorParams().strong_confidence


def test_skin_coloured_object_alone_is_ignored():
    img = scene()
    cv2.circle(img, (320, 240), 40, SKIN, -1)
    assert detect(img, DetectorParams(scale=1.0)) == []


def test_partial_candidate_without_head():
    # Head and one arm hidden by a grey slab: clothing and legs remain.
    img = draw_manikin(scene(), 320, 240)
    cv2.rectangle(img, (230, 150), (330, 235), (75, 75, 75), -1)
    found = detect(img, DetectorParams(scale=1.0))
    assert len(found) == 1
    assert not found[0].head
    assert 0.30 <= found[0].confidence < 1.0


def test_candidate_cut_by_the_image_edge_is_flagged():
    found = detect(draw_manikin(scene(), 320, 12), DetectorParams(scale=1.0))
    assert found and found[0].touches_border


def test_body_cut_in_two_by_an_occluder_is_one_candidate():
    img = draw_manikin(scene(), 320, 240, size=160)
    cv2.rectangle(img, (345, 150), (355, 330), (75, 75, 75), -1)
    assert len(detect(img, DetectorParams(scale=1.0))) == 1


def test_separate_orange_objects_are_not_fused():
    img = scene()
    cv2.rectangle(img, (200, 200), (260, 260), ORANGE_SUN, -1)
    cv2.rectangle(img, (300, 200), (360, 260), ORANGE_SUN, -1)
    assert len(detect(img, DetectorParams(scale=1.0))) == 2


def test_cues_must_be_attached_to_the_clothing():
    # An orange box with a blue box and a skin-coloured ball NEAR it (not
    # touching) is still just an orange box.
    img = scene()
    cv2.rectangle(img, (280, 200), (360, 280), ORANGE_SUN, -1)
    cv2.rectangle(img, (380, 215), (420, 255), BLUE, -1)
    cv2.circle(img, (245, 240), 18, SKIN, -1)
    found = detect(img, DetectorParams(scale=1.0))
    assert len(found) == 1
    assert not found[0].head and not found[0].legs
    assert found[0].confidence < DetectorParams().strong_confidence


def test_plank_and_obliquely_seen_box_have_no_shape_cue():
    img = scene()
    cv2.rectangle(img, (100, 100), (400, 140), ORANGE_SUN, -1)     # plank
    hexagon = np.array([[450, 300], [500, 280], [550, 300], [550, 360],
                        [500, 380], [450, 360]])
    cv2.fillPoly(img, [hexagon], ORANGE)                           # box
    found = detect(img, DetectorParams(scale=1.0))
    assert len(found) == 2 and not any(c.shape for c in found)
    assert all(c.confidence == pytest.approx(0.30) for c in found)


def test_orange_object_touching_a_skin_coloured_ball_is_a_known_hard_case():
    # Documented limitation of the colour baseline: this cue set scores as
    # a strong candidate. Rejecting it is the VLM's job.
    img = scene()
    cv2.rectangle(img, (280, 220), (380, 260), ORANGE_SUN, -1)
    cv2.circle(img, (265, 240), 18, SKIN, -1)
    found = detect(img, DetectorParams(scale=1.0))
    assert found[0].head
    assert found[0].confidence >= DetectorParams().strong_confidence


def test_sunlit_and_shadowed_clothing_both_detected():
    for clothing in (ORANGE, ORANGE_SUN):
        found = detect(draw_manikin(scene(), 320, 240, clothing=clothing),
                       DetectorParams(scale=1.0))
        assert found and found[0].confidence == 1.0


def test_small_target_below_minimum_area_is_ignored():
    img = draw_manikin(scene(), 320, 240, size=8)
    assert detect(img, DetectorParams(scale=1.0)) == []


def test_two_survivors_give_two_candidates():
    img = draw_manikin(scene(), 150, 150)
    draw_manikin(img, 480, 330)
    assert len(detect(img, DetectorParams(scale=1.0))) == 2


# -- image encoding ------------------------------------------------------------

def test_rgb8_and_bgr8_are_decoded_to_the_same_bgr_image():
    bgr = draw_manikin(scene(64, 48), 32, 24, size=30)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    assert np.array_equal(to_bgr(bgr.tobytes(), 48, 64, 'bgr8'), bgr)
    assert np.array_equal(to_bgr(rgb.tobytes(), 48, 64, 'rgb8'), bgr)


def test_rgb8_camera_frame_detects_like_bgr():
    # A channel swap would turn orange into blue and miss the survivor.
    bgr = draw_manikin(scene(), 320, 240)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    decoded = to_bgr(rgb.tobytes(), 480, 640, 'rgb8')
    assert detect(decoded, DetectorParams(scale=1.0))[0].confidence == 1.0
    assert detect(rgb, DetectorParams(scale=1.0)) == [] or detect(
        rgb, DetectorParams(scale=1.0))[0].confidence < 1.0


def test_row_padding_is_handled():
    bgr = scene(10, 4)
    padded = np.zeros((4, 36), np.uint8)
    padded[:, :30] = bgr.reshape(4, 30)
    assert np.array_equal(to_bgr(padded.tobytes(), 4, 10, 'bgr8', 36), bgr)


@pytest.mark.parametrize('encoding', ['32FC1', '16UC1', 'yuv422', ''])
def test_unsupported_encoding_is_refused(encoding):
    with pytest.raises(ValueError):
        to_bgr(bytes(640 * 480 * 3), 480, 640, encoding)


def test_short_image_buffer_is_refused():
    with pytest.raises(ValueError):
        to_bgr(bytes(100), 480, 640, 'rgb8')


# -- temporal confirmation -----------------------------------------------------

P = ConfirmationParams()


def feed(monitor, stamps, candidates_at):
    out = []
    for t in stamps:
        out = monitor.update(t, candidates_at(t))
    return out


def frames(n, t0=10.0, dt=1 / 30):
    return [t0 + i * dt for i in range(n)]


def test_single_frame_candidate_is_never_reported():
    m = SurvivorMonitor()
    ts = frames(30)
    stable = feed(m, ts, lambda t: [cand()] if t == ts[3] else [])
    assert stable == [] and m.frame_state() == NO_TARGET
    assert m.tracks == []


def test_persistent_candidate_becomes_stable_after_confirm_hits():
    m = SurvivorMonitor()
    ts = frames(10)
    for i, t in enumerate(ts):
        stable = m.update(t, [cand()])
        assert bool(stable) == (i + 1 >= P.confirm_hits)
    assert m.frame_state() == CANDIDATE
    assert stable[0].stable_stamp == pytest.approx(ts[P.confirm_hits - 1])


def test_intermittent_candidate_with_enough_hits_is_confirmed():
    # seen in 3 of every 4 frames: 6 of the last 8
    m = SurvivorMonitor()
    stable = feed(m, frames(20),
                  lambda t: [] if round((t - 10) * 30) % 4 == 3 else [cand()])
    assert len(stable) == 1


def test_intermittent_candidate_seen_every_other_frame_is_not_confirmed():
    m = SurvivorMonitor()
    stable = feed(m, frames(40),
                  lambda t: [cand()] if round((t - 10) * 30) % 2 == 0 else [])
    assert stable == []


def test_candidate_disappearance():
    m = SurvivorMonitor()
    feed(m, frames(10), lambda t: [cand()])
    # held (not measured) for up to hold_s, then dropped
    held = m.update(10.5, [])
    assert len(held) == 1 and not held[0].measured
    assert m.update(10.0 + 9 / 30 + P.hold_s + 0.05, []) == []
    assert m.frame_state() == NO_TARGET


def test_moving_candidate_keeps_its_track_id():
    m = SurvivorMonitor()
    stable = feed(m, frames(30),
                  lambda t: [cand(x=100 + int((t - 10) * 300))])
    assert len(stable) == 1 and stable[0].track_id == 1


def test_duplicate_candidates_do_not_create_duplicate_tracks():
    # The same object reported twice in a frame (overlapping boxes) and in
    # every following frame must not multiply.
    m = SurvivorMonitor()
    stable = feed(m, frames(20), lambda t: [cand(), cand(x=104, y=102)])
    assert len(stable) <= 2
    ids = {t.track_id for t in m.tracks}
    assert len(ids) == len(m.tracks) and len(m.tracks) <= 2


def test_confirmed_track_does_not_hop_to_a_smaller_nearby_object():
    # Seen live: the survivor left the view and a small orange object next
    # to it inherited the survivor's confirmed track for half a second.
    m = SurvivorMonitor()
    feed(m, frames(10), lambda t: [cand(x=300, y=200, w=176, h=104)])
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('confirm'), t.area)
    survivor_id = t.track_id
    stable = feed(m, frames(10, t0=10.4),
                  lambda t: [cand(x=330, y=240, w=42, h=36, conf=0.30,
                                  head=False)])
    states = {tr.track_id: semantic_state(tr, P) for tr in stable}
    other = [i for i in states if i != survivor_id]
    assert other and states[other[0]] == UNCERTAIN       # a new, weak track
    assert not any(tr.track_id == survivor_id and tr.measured
                   for tr in stable)


def test_track_follows_gradual_size_change():
    m = SurvivorMonitor()
    stable = feed(m, frames(40),
                  lambda t: [cand(w=int(80 + (t - 10) * 150),
                                  h=int(60 + (t - 10) * 110))])
    assert len(stable) == 1 and stable[0].track_id == 1


def test_two_separate_candidates_are_two_tracks():
    m = SurvivorMonitor()
    stable = feed(m, frames(10), lambda t: [cand(), cand(x=500, y=600)])
    assert {t.track_id for t in stable} == {1, 2}


def test_weak_stable_candidate_is_uncertain_not_candidate():
    m = SurvivorMonitor()
    stable = feed(m, frames(10), lambda t: [cand(conf=0.30, head=False)])
    assert semantic_state(stable[0], P) == UNCERTAIN
    assert m.frame_state() == UNCERTAIN


def test_out_of_order_and_repeated_timestamps_are_ignored():
    m = SurvivorMonitor()
    feed(m, frames(6), lambda t: [cand()])
    hits = m.tracks[0].hits
    m.update(10.0, [cand()])                 # older than the last frame
    m.update(10.0 + 5 / 30, [cand()])        # same stamp again
    assert m.tracks[0].hits == hits
    assert m.dropped_out_of_order == 2


def test_age_uses_camera_time_not_arrival_time():
    m = SurvivorMonitor()
    stable = feed(m, frames(10, t0=123.4), lambda t: [cand()])
    assert stable[0].first_stamp == pytest.approx(123.4)
    assert stable[0].last_stamp == pytest.approx(123.4 + 9 / 30)


# -- semantic state with VLM answers ------------------------------------------

def stable_monitor(conf=1.0):
    m = SurvivorMonitor()
    feed(m, frames(10), lambda t: [cand(conf=conf, head=conf > 0.5)])
    return m


def answer(decision, confidence=0.9, visibility='clear'):
    return VlmAnswer(decision, confidence, visibility, 'manikin', 10.2, 1.5,
                     'test-model')


def test_request_only_after_stability_and_one_at_a_time():
    m = SurvivorMonitor()
    m.update(10.0, [cand()])
    assert m.next_request(0.0, True) is None           # not stable yet
    feed(m, frames(10, t0=10.1), lambda t: [cand()])
    track = m.next_request(1.0, True)
    assert track is not None and track.verification == PENDING
    assert track.requests == 1
    assert m.next_request(5.0, True) is None           # one in flight


def test_vlm_confirmed():
    m = stable_monitor()
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('confirm', 0.92), t.area)
    assert t.verification == VLM_CONFIRMED
    assert semantic_state(t, P) == SURVIVOR_CONFIRMED
    assert m.frame_state() == SURVIVOR_CONFIRMED
    assert t.confirmed_stamp is not None


def test_vlm_low_confidence_confirmation_is_uncertain():
    m = stable_monitor()
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('confirm', 0.4), t.area)
    assert semantic_state(t, P) == UNCERTAIN


def test_vlm_rejected_weak_candidate():
    m = stable_monitor(conf=0.30)
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('reject', 0.9), t.area)
    assert t.verification == VLM_REJECTED
    assert semantic_state(t, P) == REJECTED
    assert m.frame_state() == NO_TARGET       # a rejected object is no target


def test_vlm_rejection_of_a_strong_candidate_is_a_disagreement():
    # Full person-like cue set, VLM says no: reported as UNCERTAIN with
    # verification status VLM_REJECTED, never silently dropped and never
    # confirmed.
    m = stable_monitor(conf=1.0)
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('reject', 0.95), t.area)
    assert t.verification == VLM_REJECTED
    assert semantic_state(t, P) == UNCERTAIN
    assert m.frame_state() == UNCERTAIN


def test_vlm_uncertain_and_retry():
    m = stable_monitor()
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('uncertain', 0.5, 'poor'), t.area)
    assert t.verification == VLM_UNCERTAIN
    assert semantic_state(t, P) == UNCERTAIN
    assert m.next_request(1.5, True) is None                 # too soon
    assert m.next_request(1.0 + P.retry_interval_s + 0.1, True) is t


def test_retries_stop_after_max_retries():
    m = stable_monitor()
    wall = 1.0
    for _ in range(P.max_retries):
        t = m.next_request(wall, True)
        assert t is not None
        m.apply_failure(t.track_id, unavailable=False)
        wall += P.retry_interval_s + 0.1
    assert m.next_request(wall, True) is None
    assert t.verification == VLM_ERROR
    assert semantic_state(t, P) == CANDIDATE                 # not confirmed


def test_one_rejection_after_a_confirmation_is_uncertain_two_reject():
    m = stable_monitor()
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('confirm'), t.area)
    t.verification = NOT_REQUESTED
    m.apply_answer(t.track_id, answer('reject'), t.area)
    assert semantic_state(t, P) == UNCERTAIN
    m.apply_answer(t.track_id, answer('reject'), t.area)
    assert semantic_state(t, P) == UNCERTAIN       # strong cue set
    weak = stable_monitor(conf=0.30)
    w = weak.next_request(1.0, True)
    weak.apply_answer(w.track_id, answer('confirm'), w.area)
    weak.apply_answer(w.track_id, answer('reject'), w.area)
    assert semantic_state(w, P) == UNCERTAIN
    weak.apply_answer(w.track_id, answer('reject'), w.area)
    assert semantic_state(w, P) == REJECTED


def test_reverification_is_rate_limited_and_event_based():
    m = stable_monitor()
    t = m.next_request(1.0, True)
    m.apply_answer(t.track_id, answer('confirm'), t.area)
    assert m.next_request(3.0, True) is None                 # nothing new
    # the object got much bigger in the image: verify again
    for stamp in frames(5, t0=11.0):
        m.update(stamp, [cand(w=120, h=90)])
    assert m.next_request(3.5, True) is t
    m.apply_answer(t.track_id, answer('confirm'), t.area)
    # otherwise only after the re-verification interval
    assert m.next_request(4.5, True) is None
    assert m.next_request(3.5 + P.reverify_interval_s - 0.5, True) is None
    assert m.next_request(3.5 + P.reverify_interval_s + 0.1, True) is t


def test_no_request_for_a_candidate_that_is_not_in_view():
    m = stable_monitor()
    m.update(10.6, [])                        # held, not measured
    assert m.next_request(1.0, True) is None


def test_tiny_crops_are_not_sent():
    m = SurvivorMonitor()
    feed(m, frames(10), lambda t: [cand(w=10, h=8)])
    assert m.next_request(1.0, True) is None


def test_vlm_unavailable_is_stated_and_never_confirmed():
    m = stable_monitor()
    assert m.next_request(1.0, vlm_usable=False) is None
    t = m.tracks[0]
    assert t.verification == VLM_UNAVAILABLE
    assert semantic_state(t, P) == CANDIDATE
    assert m.frame_state() == CANDIDATE
    assert t.answers == [] and t.requests == 0


def test_answer_for_a_vanished_track_is_ignored():
    m = stable_monitor()
    assert m.apply_answer(999, answer('confirm'), 1.0) is None
    assert m.apply_failure(999, False) is None


# -- VLM answer validation -------------------------------------------------------

GOOD = ('{"target": "human-shaped manikin", "confidence": 0.92, '
        '"visibility": "clear", "decision": "confirm"}')


def test_parse_valid_answer():
    assert parse_answer(GOOD) == {
        'target': 'human-shaped manikin', 'confidence': 0.92,
        'visibility': 'clear', 'decision': 'confirm'}


def test_parse_answer_inside_code_fence_or_text():
    assert parse_answer('```json\n' + GOOD + '\n```')['decision'] == 'confirm'
    assert parse_answer('Here it is: ' + GOOD)['confidence'] == 0.92


@pytest.mark.parametrize('text', [
    '', 'yes, that is a person', '{"decision": "confirm"}',
    '{"target": "x", "confidence": 1.4, "visibility": "clear", '
    '"decision": "confirm"}',
    '{"target": "x", "confidence": "high", "visibility": "clear", '
    '"decision": "confirm"}',
    '{"target": "x", "confidence": 0.9, "visibility": "great", '
    '"decision": "confirm"}',
    '{"target": "x", "confidence": 0.9, "visibility": "clear", '
    '"decision": "yes"}',
    '{"target": "", "confidence": 0.9, "visibility": "clear", '
    '"decision": "confirm"}',
    '{"target": "x", "confidence": true, "visibility": "clear", '
    '"decision": "confirm"}',
    '[1, 2, 3]', '{"target": "x", "confidence": 0.9,'])
def test_malformed_answers_are_refused(text):
    with pytest.raises(VlmFormatError):
        parse_answer(text)


def test_request_is_openai_compatible_and_carries_one_jpeg_crop():
    config = VlmConfig(endpoint='http://x/v1', model='m')
    jpeg = encode_crop(draw_manikin(scene(), 320, 240), (270, 200, 100, 90),
                       config)
    assert jpeg[:2] == b'\xff\xd8'                         # JPEG
    req = build_request(config, jpeg)
    assert req['model'] == 'm' and req['temperature'] == 0
    content = req['messages'][0]['content']
    assert content[0] == {'type': 'text', 'text': DESCRIBE_PROMPT}
    assert content[1]['image_url']['url'].startswith(
        'data:image/jpeg;base64,/9j/')


def test_description_prompt_does_not_mention_people():
    # A leading prompt made the model "see" a person in every crop.
    lowered = DESCRIBE_PROMPT.lower()
    for word in ('person', 'human', 'survivor', 'manikin', 'doll', 'head',
                 'body', 'orange', 'rescue'):
        assert word not in lowered


def test_decision_request_is_text_only_and_quotes_the_description():
    config = VlmConfig(endpoint='http://x/v1', model='m')
    req = build_decision_request(config, 'A grey "box" on the floor.')
    content = req['messages'][0]['content']
    assert isinstance(content, str) and 'image' not in content.lower()
    assert "A grey 'box' on the floor." in content
    assert content == DECIDE_PROMPT % "A grey 'box' on the floor."


def test_parse_description():
    assert parse_description(
        '{"description": "A red cube.", "visibility": "clear"}') == {
            'description': 'A red cube.', 'visibility': 'clear'}
    for bad in ('{"description": "", "visibility": "clear"}',
                '{"description": "A red cube.", "visibility": "sharp"}',
                '{"visibility": "clear"}', 'A red cube.'):
        with pytest.raises(VlmFormatError):
            parse_description(bad)


def test_crop_is_padded_clipped_and_sized_for_the_model():
    config = VlmConfig()
    assert crop_region((480, 640, 3), (600, 440, 60, 60), 0.25) == \
        (585, 425, 640, 480)
    img = draw_manikin(scene(1280, 960), 640, 480, size=600)
    big = cv2.imdecode(np.frombuffer(encode_crop(
        img, (300, 300, 680, 400), config), np.uint8), cv2.IMREAD_COLOR)
    assert max(big.shape[:2]) == config.max_side_px
    small = cv2.imdecode(np.frombuffer(encode_crop(
        img, (630, 470, 20, 20), config), np.uint8), cv2.IMREAD_COLOR)
    assert max(small.shape[:2]) == config.min_side_px


def test_crop_keeps_bgr_colour_order():
    img = scene(200, 200)
    cv2.rectangle(img, (60, 60), (140, 140), ORANGE_SUN, -1)
    decoded = cv2.imdecode(np.frombuffer(encode_crop(
        img, (60, 60, 80, 80), VlmConfig(pad_fraction=0.0)), np.uint8),
        cv2.IMREAD_COLOR)
    b, g, r = decoded[decoded.shape[0] // 2, decoded.shape[1] // 2]
    assert r > g > b                                       # still orange


# -- VLM client against a protocol stand-in ----------------------------------

class StandIn:
    """HTTP server speaking the two endpoints the client uses."""

    def __init__(self):
        self.reply = GOOD
        self.description = ('{"description": "A stylised human figure seen '
                            'from above.", "visibility": "clear"}')
        self.delay = 0.0
        self.status = 200
        self.requests = []          # image requests (one per verification)
        self.text_requests = []     # the follow-up classification requests
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _send(self, code, body):
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                self._send(200, {'data': [{'id': 'stand-in'}]})

            def do_POST(self):
                length = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(length))
                with_image = not isinstance(
                    body['messages'][0]['content'], str)
                if with_image:          # request 1 of a verification
                    outer.requests.append(body)
                    time.sleep(outer.delay)
                else:
                    outer.text_requests.append(body)
                if outer.status != 200:
                    self._send(outer.status, {'error': 'x'})
                    return
                self._send(200, {'choices': [{'message': {'content': (
                    outer.description if with_image else outer.reply)}}]})

        self.server = HTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       daemon=True)
        self.thread.start()

    @property
    def url(self):
        return f'http://127.0.0.1:{self.server.server_address[1]}/v1'

    def close(self):
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def stand_in():
    server = StandIn()
    yield server
    server.close()


JPEG = encode_crop(draw_manikin(scene(), 320, 240), (270, 200, 100, 90))


def wait_for(worker, n=1, timeout=5.0):
    out, end = [], time.monotonic() + timeout
    while len(out) < n and time.monotonic() < end:
        out += worker.poll()
        time.sleep(0.01)
    return out


def test_request_returns_validated_answer_and_latency(stand_in):
    stand_in.delay = 0.1
    out = request_once(VlmConfig(endpoint=stand_in.url, model='stand-in'),
                       JPEG, job_id=7, context={'track_id': 3})
    assert out.kind == 'answer' and out.decision == 'confirm'
    assert out.confidence == 0.92 and out.visibility == 'clear'
    assert out.description == 'A stylised human figure seen from above.'
    assert out.job_id == 7 and out.context == {'track_id': 3}
    assert out.latency_s >= 0.1
    sent = stand_in.requests[0]
    assert sent['model'] == 'stand-in'
    assert sent['messages'][0]['content'][1]['type'] == 'image_url'
    # the second request carries the description, not the image
    follow_up = stand_in.text_requests[0]['messages'][0]['content']
    assert 'A stylised human figure seen from above.' in follow_up


def test_visibility_comes_from_the_description_step(stand_in):
    stand_in.description = ('{"description": "Part of a figure.", '
                            '"visibility": "partial"}')
    out = request_once(VlmConfig(endpoint=stand_in.url, model='m'), JPEG)
    assert out.kind == 'answer' and out.visibility == 'partial'


def test_malformed_description_is_an_error(stand_in):
    stand_in.description = 'It looks like a person to me.'
    out = request_once(VlmConfig(endpoint=stand_in.url, model='m'), JPEG)
    assert out.kind == 'error' and 'malformed' in out.error
    assert stand_in.text_requests == []       # no second request


def test_rejection_and_uncertain_answers_pass_through(stand_in):
    config = VlmConfig(endpoint=stand_in.url, model='stand-in')
    stand_in.reply = ('{"target": "box", "confidence": 0.03, '
                      '"visibility": "clear", "decision": "reject"}')
    assert request_once(config, JPEG).decision == 'reject'
    stand_in.description = ('{"description": "A blurred dark shape.", '
                            '"visibility": "poor"}')
    stand_in.reply = ('{"target": "unclear", "confidence": 0.5, '
                      '"decision": "uncertain"}')
    out = request_once(config, JPEG)
    assert out.decision == 'uncertain' and out.visibility == 'poor'


def test_malformed_reply_is_an_error_not_an_answer(stand_in):
    stand_in.reply = 'I think this might be a person.'
    out = request_once(VlmConfig(endpoint=stand_in.url, model='m'), JPEG)
    assert out.kind == 'error' and 'malformed' in out.error
    assert out.decision == '' and out.confidence == -1.0
    assert out.raw.endswith('I think this might be a person.')


def test_timeout_is_an_error(stand_in):
    stand_in.delay = 1.0
    out = request_once(VlmConfig(endpoint=stand_in.url, model='m',
                                 timeout_s=0.2), JPEG)
    assert out.kind == 'error' and 'timeout' in out.error
    assert out.latency_s < 0.9


def test_http_error_is_an_error(stand_in):
    stand_in.status = 500
    out = request_once(VlmConfig(endpoint=stand_in.url, model='m'), JPEG)
    assert out.kind == 'error' and '500' in out.error


def test_no_endpoint_configured_is_unavailable():
    out = request_once(VlmConfig(), JPEG)
    assert out.kind == 'unavailable' and out.decision == ''


def test_unreachable_endpoint_is_unavailable():
    out = request_once(VlmConfig(endpoint='http://127.0.0.1:9/v1',
                                 model='m', timeout_s=1.0), JPEG)
    assert out.kind == 'unavailable'


def test_api_key_is_sent_as_bearer_but_not_in_the_body(stand_in):
    seen = {}
    original = vlm_client.urllib.request.urlopen

    def spy(request, timeout=None):
        seen['auth'] = request.get_header('Authorization')
        return original(request, timeout=timeout)
    vlm_client.urllib.request.urlopen = spy
    try:
        request_once(VlmConfig(endpoint=stand_in.url, model='m',
                               api_key='secret-token'), JPEG)
    finally:
        vlm_client.urllib.request.urlopen = original
    assert seen['auth'] == 'Bearer secret-token'
    assert 'secret-token' not in json.dumps(stand_in.requests[0])


def test_worker_is_asynchronous(stand_in):
    stand_in.delay = 0.5
    worker = VlmWorker(VlmConfig(endpoint=stand_in.url, model='m'))
    try:
        img = draw_manikin(scene(), 320, 240)
        started = time.monotonic()
        job = worker.submit(img, (270, 200, 100, 90), {'track_id': 1})
        assert time.monotonic() - started < 0.05         # did not wait
        assert worker.poll() == []                       # not ready yet
        assert worker.busy
        out = wait_for(worker)
        assert out[0].job_id == job and out[0].kind == 'answer'
        assert out[0].context == {'track_id': 1}
        assert 0.5 <= out[0].latency_s < 2.0
        assert worker.status == VLM_AVAILABLE and not worker.busy
    finally:
        worker.stop()


def test_worker_copies_the_crop_at_submission(stand_in):
    # The camera buffer is reused; the queued crop must not change with it.
    stand_in.delay = 0.2
    worker = VlmWorker(VlmConfig(endpoint=stand_in.url, model='m'))
    try:
        img = draw_manikin(scene(), 320, 240)
        worker.submit(img, (270, 200, 100, 90))
        img[:] = 0
        wait_for(worker)
        url = stand_in.requests[0]['messages'][0]['content'][1][
            'image_url']['url']
        import base64
        sent = cv2.imdecode(np.frombuffer(base64.b64decode(
            url.split(',', 1)[1]), np.uint8), cv2.IMREAD_COLOR)
        assert sent.mean() > 50
    finally:
        worker.stop()


def test_worker_replaces_a_waiting_job_and_reports_it(stand_in):
    stand_in.delay = 0.4
    worker = VlmWorker(VlmConfig(endpoint=stand_in.url, model='m'))
    try:
        img = draw_manikin(scene(), 320, 240)
        first = worker.submit(img, (270, 200, 100, 90), {'track_id': 1})
        time.sleep(0.1)                                  # first is running
        second = worker.submit(img, (270, 200, 100, 90), {'track_id': 2})
        third = worker.submit(img, (270, 200, 100, 90), {'track_id': 3})
        out = {o.job_id: o for o in wait_for(worker, 3, timeout=6.0)}
        assert out[first].kind == 'answer' and out[third].kind == 'answer'
        assert out[second].kind == 'error'
        assert 'superseded' in out[second].error
        assert len(stand_in.requests) == 2               # second never sent
    finally:
        worker.stop()


def test_worker_status_without_endpoint_and_after_errors(stand_in):
    idle = VlmWorker(VlmConfig())
    try:
        time.sleep(0.2)
        assert idle.status == vlm_client.VLM_UNAVAILABLE and not idle.usable
    finally:
        idle.stop()
    stand_in.reply = 'not json'
    worker = VlmWorker(VlmConfig(endpoint=stand_in.url, model='m'))
    try:
        end = time.monotonic() + 3
        while not worker.usable and time.monotonic() < end:
            time.sleep(0.02)
        assert worker.status == VLM_AVAILABLE            # probe succeeded
        worker.submit(draw_manikin(scene(), 320, 240), (270, 200, 100, 90))
        assert wait_for(worker)[0].kind == 'error'
        assert worker.status == vlm_client.VLM_ERROR and worker.usable
    finally:
        worker.stop()


def test_monitor_and_worker_end_to_end(stand_in):
    """Stable candidate -> crop -> stand-in answer -> SURVIVOR_CONFIRMED."""
    worker = VlmWorker(VlmConfig(endpoint=stand_in.url, model='stand-in'))
    monitor = SurvivorMonitor()
    img = draw_manikin(scene(), 320, 240)
    try:
        end = time.monotonic() + 3
        while not worker.usable and time.monotonic() < end:
            time.sleep(0.02)
        state, stamp = NO_TARGET, 10.0
        deadline = time.monotonic() + 5
        while state != SURVIVOR_CONFIRMED and time.monotonic() < deadline:
            stamp += 1 / 30
            found = detect(img, DetectorParams(scale=1.0))
            for out in worker.poll():
                monitor.apply_answer(out.context['track_id'], VlmAnswer(
                    out.decision, out.confidence, out.visibility,
                    out.target, out.context['stamp'], out.latency_s),
                    out.context['area'])
            monitor.update(stamp, found)
            track = monitor.next_request(time.monotonic(), worker.usable)
            if track is not None:
                c = track.candidate
                worker.submit(img, (c.x, c.y, c.width, c.height),
                              {'track_id': track.track_id, 'stamp': stamp,
                               'area': track.area})
            state = monitor.frame_state()
            time.sleep(0.005)
        assert state == SURVIVOR_CONFIRMED
        assert len(stand_in.requests) == 1               # one crop, not a stream
    finally:
        worker.stop()


# -- the ROS node: frames in, messages out (no simulator) --------------------

@pytest.fixture(scope='module')
def ros():
    import rclpy
    rclpy.init()
    yield
    rclpy.try_shutdown()


def make_node(monkeypatch, endpoint='', model=''):
    from uav_autonomy.survivor_perception_node import SurvivorPerceptionNode
    monkeypatch.setenv('VLM_ENDPOINT', endpoint)
    monkeypatch.setenv('VLM_MODEL', model)
    node = SurvivorPerceptionNode()
    sent = {'arrays': [], 'debug': [], 'status': []}
    node.pub.publish = sent['arrays'].append
    node.pub_debug.publish = sent['debug'].append
    node.pub_status.publish = sent['status'].append
    return node, sent


def image_msg(bgr, stamp, encoding='rgb8', frame_id='camera_link'):
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    msg.header.frame_id = frame_id
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = encoding
    msg.step = bgr.shape[1] * 3
    data = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if encoding == 'rgb8' \
        else bgr
    msg.data = data.tobytes()
    return msg


def test_node_starts_with_all_parameters_declared(ros, monkeypatch):
    node, _ = make_node(monkeypatch)
    try:
        assert node.get_parameter('detector.strong_confidence').value == \
            DetectorParams().strong_confidence
        assert node.get_parameter('confirmation.confirm_hits').value == \
            ConfirmationParams().confirm_hits
        assert node.get_parameter('vlm_endpoint').value == ''
    finally:
        node.destroy_node()


def test_node_output_carries_camera_stamp_and_frame(ros, monkeypatch):
    node, sent = make_node(monkeypatch)
    try:
        img = draw_manikin(scene(), 320, 240)
        for i in range(8):
            node.on_image(image_msg(img, 50.0 + i / 30))
        last = sent['arrays'][-1]
        assert len(sent['arrays']) == 8                  # one per frame
        stamp = last.header.stamp.sec + last.header.stamp.nanosec * 1e-9
        assert stamp == pytest.approx(50.0 + 7 / 30, abs=1e-6)
        assert last.header.frame_id == 'camera_link'
        det = last.detections[0]
        assert det.header.frame_id == 'camera_link'
        assert det.header.stamp == last.header.stamp
        assert (det.image_width, det.image_height) == (640, 480)
        assert abs(det.center_u - 320) < 15
        assert det.bbox_width > 0 and det.bbox_height > 0
        assert det.source == 'hsv_shape_baseline'
        assert det.age == pytest.approx(7 / 30, abs=1e-3)
    finally:
        node.destroy_node()


def test_node_uses_camera_info_frame_when_the_image_has_none(ros,
                                                             monkeypatch):
    from sensor_msgs.msg import CameraInfo
    node, sent = make_node(monkeypatch)
    try:
        info = CameraInfo()
        info.header.frame_id = 'x500_rescue_0/camera_link/camera'
        info.width, info.height = 640, 480
        node.on_info(info)
        node.on_image(image_msg(scene(), 60.0, frame_id=''))
        assert sent['arrays'][-1].header.frame_id == \
            'x500_rescue_0/camera_link/camera'
    finally:
        node.destroy_node()


def test_node_ignores_frames_without_timestamp_or_with_bad_encoding(
        ros, monkeypatch):
    node, sent = make_node(monkeypatch)
    try:
        img = draw_manikin(scene(), 320, 240)
        node.on_image(image_msg(img, 0.0))
        bad = image_msg(img, 70.0)
        bad.encoding = '32FC1'
        node.on_image(bad)
        assert sent['arrays'] == [] and node.bad_frames == 2
    finally:
        node.destroy_node()


def test_node_without_vlm_reports_candidate_and_unavailable(ros,
                                                            monkeypatch):
    from uav_interfaces.msg import SurvivorDetection, SurvivorDetectionArray
    node, sent = make_node(monkeypatch)
    try:
        img = draw_manikin(scene(), 320, 240)
        for i in range(20):
            node.on_image(image_msg(img, 80.0 + i / 30))
        assert [a.state for a in sent['arrays'][:4]] == [0, 0, 0, 0]
        last = sent['arrays'][-1]
        assert last.state == SurvivorDetection.STATE_CANDIDATE
        assert last.vlm_status == SurvivorDetectionArray.VLM_UNAVAILABLE
        det = last.detections[0]
        assert det.semantic_state == SurvivorDetection.STATE_CANDIDATE
        assert det.verification_status == \
            SurvivorDetection.VERIFY_VLM_UNAVAILABLE
        assert det.vlm_confidence == -1.0 and det.vlm_label == ''
        assert det.vlm_requests == 0
    finally:
        node.destroy_node()


def test_node_camera_callback_never_waits_for_the_vlm(ros, monkeypatch,
                                                      stand_in):
    from uav_interfaces.msg import SurvivorDetection
    stand_in.delay = 0.8
    node, sent = make_node(monkeypatch, stand_in.url, 'stand-in')
    try:
        end = time.monotonic() + 3
        while not node.worker.usable and time.monotonic() < end:
            time.sleep(0.02)
        img = draw_manikin(scene(), 320, 240)
        slowest, stamp, confirmed_after = 0.0, 90.0, None
        started = time.monotonic()
        while time.monotonic() - started < 3.0:
            stamp += 1 / 30
            t0 = time.monotonic()
            node.on_image(image_msg(img, stamp))
            slowest = max(slowest, time.monotonic() - t0)
            dets = sent['arrays'][-1].detections
            if dets and dets[0].semantic_state == \
                    SurvivorDetection.STATE_SURVIVOR_CONFIRMED:
                confirmed_after = time.monotonic() - started
                break
            time.sleep(0.01)
        assert slowest < 0.25                    # the VLM took 0.8 s
        assert confirmed_after is not None and confirmed_after >= 0.8
        det = sent['arrays'][-1].detections[0]
        assert det.verification_status == \
            SurvivorDetection.VERIFY_VLM_CONFIRMED
        assert det.vlm_confidence == pytest.approx(0.92)
        assert det.vlm_latency >= 0.8
        assert det.source == 'hsv_shape_baseline+vlm:stand-in'
        assert len(stand_in.requests) == 1       # one crop for many frames
        # while waiting, the state was CANDIDATE with a pending request
        pending = [a for a in sent['arrays'] if a.detections
                   and a.detections[0].verification_status
                   == SurvivorDetection.VERIFY_PENDING]
        assert pending and all(
            a.detections[0].semantic_state
            == SurvivorDetection.STATE_CANDIDATE for a in pending)
    finally:
        node.destroy_node()


def test_node_debug_image_is_bgr8_and_reduced(ros, monkeypatch):
    node, sent = make_node(monkeypatch)
    try:
        img = draw_manikin(scene(), 320, 240)
        for i in range(9):
            node.on_image(image_msg(img, 100.0 + i / 30))
        assert len(sent['debug']) == 3           # every third frame
        dbg = sent['debug'][-1]
        assert dbg.encoding == 'bgr8'
        assert (dbg.width, dbg.height) == (320, 240)
        assert len(dbg.data) == 320 * 240 * 3
        assert dbg.header.stamp == sent['arrays'][-1].header.stamp
    finally:
        node.destroy_node()


# -- audit: the online code has no ground-truth input ------------------------

def test_online_code_has_no_ground_truth_input(ros, monkeypatch):
    import inspect
    import re
    from uav_autonomy import (
        survivor_confirmation, survivor_detection, survivor_perception_node,
        vlm_client)
    forbidden = (r'\bimport gz\b', r'\bfrom gz\b', r'gz\.transport',
                 r'pose/info', r'\.sdf\b', r'survivor_target',
                 r'poses\.csv', r'world_specs', r'results/phase7',
                 r'ground_truth', r'groundtruth')
    for module in (survivor_detection, survivor_confirmation, vlm_client,
                   survivor_perception_node):
        code = '\n'.join(
            line for line in inspect.getsource(module).splitlines()
            if not line.strip().startswith('#'))
        # the module docstrings may say that no ground truth is used
        code = re.sub(r'"""[\s\S]*?"""', '', code)
        for pattern in forbidden:
            assert not re.search(pattern, code), (module.__name__, pattern)
        # reads no files (urllib's urlopen is the HTTP call to the VLM)
        assert not re.search(r'(?<![\w.])open\(', code), module.__name__
    node, _ = make_node(monkeypatch)
    try:
        topics = sorted(s.topic_name for s in node.subscriptions)
        assert topics == ['/camera/camera_info', '/camera/image_raw']
        published = sorted(p.topic_name for p in node.publishers
                           if 'survivor' in p.topic_name)
        assert published == ['/perception/survivor/debug_image',
                             '/perception/survivor/detections',
                             '/perception/survivor/status']
        assert not any('/fmu/' in p.topic_name for p in node.publishers)
    finally:
        node.destroy_node()
