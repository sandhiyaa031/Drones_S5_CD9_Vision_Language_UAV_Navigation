"""Temporal confirmation and semantic state of survivor candidates (no ROS).

A candidate in one frame means nothing. Candidates are followed from frame
to frame; a followed candidate ("track") is only reported once it is stable,
and only called a survivor once the VLM has confirmed a crop of it.

Stability
    A track is STABLE when it was detected in at least ``confirm_hits`` of
    the last ``confirm_window`` camera frames. A stable track survives up to
    ``hold_s`` seconds without a detection (intermittent view, brief
    occlusion) and is then dropped. A track that never becomes stable is
    never reported: a single-frame false candidate produces no output.

Semantic state of a stable track
    no usable VLM answer    strong cue set -> CANDIDATE, weak -> UNCERTAIN
    VLM "confirm", confidence >= vlm_confirm_confidence
                            -> SURVIVOR_CONFIRMED
    VLM "confirm" below that, or "uncertain"   -> UNCERTAIN
    VLM "reject"            weak cue set   -> REJECTED
                            strong cue set -> UNCERTAIN: the detector found
                               a complete person-like cue set and the VLM
                               disagrees. The VLM is not reliable enough to
                               erase that (measured: it rejects about a
                               third of clear survivor crops, and most
                               crops taken from straight above), so the
                               disagreement is reported, not hidden.
                            after a confirmation -> UNCERTAIN until a second
                               rejection in a row
    The VLM being unavailable or failing never produces a confirmation.

Verification requests (event-based, one at a time)
    first request   when a track becomes stable
    retry           after an error or an "uncertain" answer, not sooner than
                    ``retry_interval_s``, at most ``max_retries`` in a row
    re-verification of an answered track every ``reverify_interval_s``
                    (0 = never), or at once if its box area has grown by
                    ``reverify_area_gain`` since the verified crop
    never           more often than ``min_request_interval_s`` overall, nor
                    for a box whose short side is below ``min_crop_px``
"""

from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

from uav_autonomy.survivor_detection import Candidate

# semantic states (values match uav_interfaces/SurvivorDetection)
NO_TARGET, CANDIDATE, SURVIVOR_CONFIRMED, UNCERTAIN, REJECTED = range(5)
STATE_NAMES = {NO_TARGET: 'NO_TARGET', CANDIDATE: 'CANDIDATE',
               SURVIVOR_CONFIRMED: 'SURVIVOR_CONFIRMED',
               UNCERTAIN: 'UNCERTAIN', REJECTED: 'REJECTED'}
# verification status
(NOT_REQUESTED, PENDING, VLM_CONFIRMED, VLM_REJECTED, VLM_UNCERTAIN,
 VLM_UNAVAILABLE, VLM_ERROR) = range(7)
VERIFY_NAMES = {NOT_REQUESTED: 'NOT_REQUESTED', PENDING: 'PENDING',
                VLM_CONFIRMED: 'VLM_CONFIRMED', VLM_REJECTED: 'VLM_REJECTED',
                VLM_UNCERTAIN: 'VLM_UNCERTAIN',
                VLM_UNAVAILABLE: 'VLM_UNAVAILABLE', VLM_ERROR: 'VLM_ERROR'}
# array-level priority: what the whole frame is reported as
_PRIORITY = {SURVIVOR_CONFIRMED: 3, CANDIDATE: 2, UNCERTAIN: 1, REJECTED: 0}


@dataclass(frozen=True)
class ConfirmationParams:
    """Thresholds (docs/survivor_perception.md)."""

    confirm_hits: int = 5
    confirm_window: int = 8
    hold_s: float = 1.0
    match_distance: float = 0.75       # of the larger box side
    # A candidate continues a track only if its box area is within this
    # factor of the track's: a confirmed survivor's track must not hop onto
    # a different, nearby object when the survivor leaves the view.
    match_area_ratio: float = 3.0
    strong_confidence: float = 0.55
    vlm_confirm_confidence: float = 0.6
    min_request_interval_s: float = 0.5
    retry_interval_s: float = 2.0
    max_retries: int = 5
    reverify_interval_s: float = 10.0
    reverify_area_gain: float = 2.0
    min_crop_px: int = 16


@dataclass
class VlmAnswer:
    """One valid VLM answer for a track."""

    decision: str                      # confirm | reject | uncertain
    confidence: float
    visibility: str                    # clear | partial | poor
    label: str
    frame_stamp: float                 # camera time of the verified crop
    latency_s: float
    model: str = ''


@dataclass
class Track:
    """A candidate followed over frames."""

    track_id: int
    candidate: Candidate
    first_stamp: float
    last_stamp: float
    hits: int = 1
    window: deque = field(default_factory=deque)
    confidences: deque = field(default_factory=deque)
    stable: bool = False
    stable_stamp: Optional[float] = None
    measured: bool = True
    answers: List[VlmAnswer] = field(default_factory=list)
    verification: int = NOT_REQUESTED
    requests: int = 0
    failures_in_row: int = 0
    last_request_wall: Optional[float] = None
    verified_area: float = 0.0
    confirmed_stamp: Optional[float] = None

    strong_threshold: float = 0.55

    @property
    def strong(self):
        """Median cue score over the window reaches the strong threshold."""
        if not self.confidences:
            return False
        ordered = sorted(self.confidences)
        return ordered[len(ordered) // 2] >= self.strong_threshold

    @property
    def area(self):
        return float(self.candidate.width * self.candidate.height)


def semantic_state(track: Track, params: ConfirmationParams) -> int:
    """State of a stable track from its cue strength and VLM answers."""
    if track.answers:
        last = track.answers[-1]
        if last.decision == 'confirm':
            return (SURVIVOR_CONFIRMED
                    if last.confidence >= params.vlm_confirm_confidence
                    else UNCERTAIN)
        if last.decision == 'uncertain':
            return UNCERTAIN
        previous = track.answers[-2] if len(track.answers) > 1 else None
        if previous is not None and previous.decision == 'confirm':
            return UNCERTAIN
        # a rejection: final for a weak cue set, a disagreement for a
        # strong one
        return UNCERTAIN if track.strong else REJECTED
    return CANDIDATE if track.strong else UNCERTAIN


class SurvivorMonitor:
    """Follow candidates, decide stability, state and VLM requests."""

    def __init__(self, params: ConfirmationParams = ConfirmationParams()):
        self.params = params
        self.tracks: List[Track] = []
        self._next_id = 1
        self._last_request_wall: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self.dropped_out_of_order = 0

    # -- per camera frame --------------------------------------------------
    def update(self, stamp: float, candidates: List[Candidate]) -> List[Track]:
        """Associate this frame's candidates; return the stable tracks.

        ``stamp`` is the camera frame time. A frame older than the previous
        one is ignored (the previous output is returned).
        """
        p = self.params
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.dropped_out_of_order += 1
            return [t for t in self.tracks if t.stable]
        self._last_stamp = stamp
        unmatched = list(candidates)
        for track in sorted(self.tracks, key=lambda t: -t.hits):
            best, best_d = None, None
            cu, cv = track.candidate.center
            limit = p.match_distance * max(track.candidate.width,
                                           track.candidate.height)
            for cand in unmatched:
                u, v = cand.center
                d = ((u - cu) ** 2 + (v - cv) ** 2) ** 0.5
                gate = max(limit, p.match_distance
                           * max(cand.width, cand.height))
                ratio = (cand.width * cand.height) / max(track.area, 1.0)
                if not (1.0 / p.match_area_ratio <= ratio
                        <= p.match_area_ratio):
                    continue
                if d <= gate and (best_d is None or d < best_d):
                    best, best_d = cand, d
            if best is not None:
                unmatched.remove(best)
                track.candidate = best
                track.last_stamp = stamp
                track.hits += 1
                track.measured = True
                track.window.append(True)
                track.confidences.append(best.confidence)
            else:
                track.measured = False
                track.window.append(False)
            while len(track.window) > p.confirm_window:
                track.window.popleft()
            while len(track.confidences) > p.confirm_window:
                track.confidences.popleft()
            if not track.stable and sum(track.window) >= p.confirm_hits:
                track.stable = True
                track.stable_stamp = stamp
        for cand in unmatched:
            track = Track(self._next_id, cand, stamp, stamp)
            track.strong_threshold = p.strong_confidence
            track.window.append(True)
            track.confidences.append(cand.confidence)
            if p.confirm_hits <= 1:
                track.stable, track.stable_stamp = True, stamp
            self.tracks.append(track)
            self._next_id += 1
        # drop: stable tracks after hold_s unseen; tentative ones once the
        # run of misses is long enough that the window cannot fill
        give_up = p.confirm_window - p.confirm_hits + 1
        kept = []
        for track in self.tracks:
            if track.stable:
                if stamp - track.last_stamp <= p.hold_s:
                    kept.append(track)
                continue
            misses = 0
            for seen in reversed(track.window):
                if seen:
                    break
                misses += 1
            if misses < give_up:
                kept.append(track)
        self.tracks = kept
        return [t for t in self.tracks if t.stable]

    def frame_state(self) -> int:
        """Most significant state among stable tracks (NO_TARGET if none)."""
        best, rank = NO_TARGET, -1
        for track in self.tracks:
            if not track.stable:
                continue
            state = semantic_state(track, self.params)
            if _PRIORITY[state] > rank:
                best, rank = state, _PRIORITY[state]
        return NO_TARGET if best == REJECTED else best

    # -- VLM -----------------------------------------------------------------
    def next_request(self, wall: float, vlm_usable: bool) -> Optional[Track]:
        """Track whose crop should go to the VLM now, or None.

        ``vlm_usable`` False (no endpoint): stable tracks are marked
        VLM_UNAVAILABLE and nothing is requested.
        """
        p = self.params
        stable = [t for t in self.tracks if t.stable]
        if not vlm_usable:
            for track in stable:
                if track.verification in (NOT_REQUESTED, PENDING, VLM_ERROR):
                    track.verification = VLM_UNAVAILABLE
            return None
        if any(t.verification == PENDING for t in self.tracks):
            return None
        if (self._last_request_wall is not None
                and wall - self._last_request_wall < p.min_request_interval_s):
            return None
        chosen, chosen_rank = None, None
        for track in stable:
            if not track.measured or track.candidate.short_side \
                    < p.min_crop_px:
                continue
            since = (None if track.last_request_wall is None
                     else wall - track.last_request_wall)
            if track.verification in (NOT_REQUESTED, VLM_UNAVAILABLE):
                rank = 0                               # never verified
            elif track.verification == VLM_ERROR or (
                    track.answers
                    and track.answers[-1].decision == 'uncertain'):
                if (track.failures_in_row >= p.max_retries
                        or since < p.retry_interval_s):
                    continue
                rank = 1
            else:
                grown = (track.verified_area > 0 and track.area
                         >= p.reverify_area_gain * track.verified_area)
                due = (p.reverify_interval_s > 0
                       and since >= p.reverify_interval_s)
                if not (due or (grown and since >= p.retry_interval_s)):
                    continue
                rank = 2
            if chosen_rank is None or rank < chosen_rank:
                chosen, chosen_rank = track, rank
        if chosen is not None:
            chosen.verification = PENDING
            chosen.requests += 1
            chosen.last_request_wall = wall
            self._last_request_wall = wall
        return chosen

    def _find(self, track_id) -> Optional[Track]:
        return next((t for t in self.tracks if t.track_id == track_id), None)

    def apply_answer(self, track_id: int, answer: VlmAnswer,
                     verified_area: float) -> Optional[Track]:
        """Record a valid VLM answer. Unknown tracks are ignored."""
        track = self._find(track_id)
        if track is None:
            return None
        track.answers.append(answer)
        track.verified_area = verified_area
        track.verification = {'confirm': VLM_CONFIRMED,
                              'reject': VLM_REJECTED,
                              'uncertain': VLM_UNCERTAIN}[answer.decision]
        track.failures_in_row = (track.failures_in_row + 1
                                 if answer.decision == 'uncertain' else 0)
        if (semantic_state(track, self.params) == SURVIVOR_CONFIRMED
                and track.confirmed_stamp is None):
            track.confirmed_stamp = self._last_stamp
        return track

    def apply_failure(self, track_id: int, unavailable: bool):
        """The request failed: no answer is recorded, nothing is invented."""
        track = self._find(track_id)
        if track is None:
            return None
        track.verification = VLM_UNAVAILABLE if unavailable else VLM_ERROR
        track.failures_in_row += 1
        return track
