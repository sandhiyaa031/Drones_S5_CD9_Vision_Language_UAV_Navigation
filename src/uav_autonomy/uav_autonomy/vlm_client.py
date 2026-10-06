"""Asynchronous VLM verification of image crops (no ROS).

A worker thread sends one crop at a time to an OpenAI-compatible
``/chat/completions`` endpoint (Ollama, llama.cpp, vLLM, LM Studio and the
hosted APIs speak it) and returns a strictly validated answer:

    {"target": "<what the object is>",
     "confidence": 0.0 .. 1.0,          how sure the model is of its decision
     "visibility": "clear" | "partial" | "poor",
     "decision": "confirm" | "reject" | "uncertain"}

One verification is two requests to the same model:
    1. the crop, with a neutral question: describe the main object and say
       how visible it is. Nothing in this prompt mentions people.
    2. the description (text only): does it describe a person or a
       human-like figure?
A single prompt that both shows the image and asks "is this a person" was
tried first and rejected on measurements: the small local model either
confirmed every crop or rejected every crop, depending on the wording
(docs/survivor_perception.md).

Nothing is ever fabricated. Every request ends in exactly one outcome:
    answer        a valid structured answer
    error         timeout, HTTP error, or an answer that does not validate
    unavailable   no endpoint configured, or it cannot be reached

Endpoint status
    VLM_UNAVAILABLE  no endpoint configured, or the last attempt could not
                     connect (re-probed every ``probe_interval_s``)
    VLM_AVAILABLE    the endpoint answered the probe / the last request
    VLM_ERROR        reachable, but the last request failed

The caller never blocks: ``submit`` returns at once (a job waiting in the
queue is replaced by a newer one), results are collected with ``poll``.
"""

import base64
import json
import queue
import socket
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Optional

import cv2

VLM_AVAILABLE, VLM_UNAVAILABLE, VLM_ERROR = range(3)
STATUS_NAMES = {VLM_AVAILABLE: 'VLM_AVAILABLE',
                VLM_UNAVAILABLE: 'VLM_UNAVAILABLE', VLM_ERROR: 'VLM_ERROR'}

DECISIONS = ('confirm', 'reject', 'uncertain')
VISIBILITIES = ('clear', 'partial', 'poor')

DESCRIBE_PROMPT = (
    'This image was taken by a camera looking straight down at the ground '
    'from a few metres above. Describe the main object in it. Reply with '
    'one JSON object: {"description": "<one short sentence: what the main '
    'object is, with its shape and colours>", "visibility": "clear" or '
    '"partial" or "poor"}')

DECIDE_PROMPT = (
    'A drone camera looking down saw an object. The object was described '
    'like this:\n"%s"\n\n'
    'Decide whether the described object is a person or a human-like '
    'figure.\n'
    '- "confirm": the description says it is a person, a human or humanoid '
    'figure, a doll, a dummy, a manikin, or a character that has a head and '
    'a body or wears clothes.\n'
    '- "reject": the description says it is a thing: a box, crate, block, '
    'cube, slab, ball, sphere, egg, rock, vehicle, rocket, drone, tool, '
    'container, building part, wall, window, floor or shadow.\n'
    '- "uncertain": the description is too vague to tell.\n'
    'Examples: "a person lying on the ground" -> confirm; "a cartoon '
    'character wearing a shirt and trousers" -> confirm; "a mannequin with '
    'a head and arms" -> confirm; "a soccer ball" -> reject; "a small black '
    'drone" -> reject; "a cardboard box" -> reject; "a toy rocket" -> '
    'reject; "a grey shape" -> uncertain.\n'
    'Reply with one JSON object: {"target": "<two-word name of the '
    'object>", "decision": "confirm" or "reject" or "uncertain", '
    '"confidence": <number 0 to 1: how sure you are of your decision>}')


class VlmFormatError(ValueError):
    """The model's reply is not the required structured answer."""


@dataclass(frozen=True)
class VlmConfig:
    """Endpoint settings."""

    endpoint: str = ''              # base URL, e.g. http://127.0.0.1:11434/v1
    model: str = ''
    api_key: str = ''               # optional; never logged
    timeout_s: float = 30.0
    probe_interval_s: float = 10.0
    max_side_px: int = 448          # crop is resized so its long side is this
    min_side_px: int = 224          # small crops are enlarged to this
    jpeg_quality: int = 90
    pad_fraction: float = 0.25      # context around the bounding box
    max_tokens: int = 150


@dataclass
class VlmOutcome:
    """Result of one request."""

    job_id: int
    kind: str                       # 'answer' | 'error' | 'unavailable'
    latency_s: float
    target: str = ''
    confidence: float = -1.0
    visibility: str = ''
    decision: str = ''
    description: str = ''           # the model's own description of the crop
    error: str = ''
    raw: str = ''
    model: str = ''
    context: Optional[dict] = None  # whatever the caller attached to the job


def crop_region(image_shape, bbox, pad_fraction):
    """Padded crop rectangle (x0, y0, x1, y1), clipped to the image."""
    height, width = image_shape[:2]
    x, y, w, h = bbox
    pad = int(round(pad_fraction * max(w, h)))
    x0, y0 = max(0, int(x) - pad), max(0, int(y) - pad)
    x1, y1 = min(width, int(x + w) + pad), min(height, int(y + h) + pad)
    if x1 <= x0 or y1 <= y0:
        raise ValueError(f'empty crop for bbox {bbox}')
    return x0, y0, x1, y1


def encode_crop(bgr, bbox, config: VlmConfig = VlmConfig()) -> bytes:
    """JPEG of the padded crop, resized into the model's useful range.

    ``bgr`` must be a BGR image (as OpenCV expects for JPEG encoding).
    """
    x0, y0, x1, y1 = crop_region(bgr.shape, bbox, config.pad_fraction)
    crop = bgr[y0:y1, x0:x1]
    long_side = max(crop.shape[:2])
    scale = 1.0
    if long_side > config.max_side_px:
        scale = config.max_side_px / long_side
    elif long_side < config.min_side_px:
        scale = config.min_side_px / long_side
    if scale != 1.0:
        crop = cv2.resize(
            crop, (max(1, int(round(crop.shape[1] * scale))),
                   max(1, int(round(crop.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC)
    ok, encoded = cv2.imencode('.jpg', crop,
                               [cv2.IMWRITE_JPEG_QUALITY, config.jpeg_quality])
    if not ok:
        raise ValueError('could not encode the crop as JPEG')
    return encoded.tobytes()


def _chat(config: VlmConfig, content) -> dict:
    return {'model': config.model, 'temperature': 0,
            'max_tokens': config.max_tokens,
            'response_format': {'type': 'json_object'},
            'messages': [{'role': 'user', 'content': content}]}


def build_request(config: VlmConfig, jpeg: bytes) -> dict:
    """Request 1: the crop and the neutral description question."""
    data = base64.b64encode(jpeg).decode('ascii')
    return _chat(config, [
        {'type': 'text', 'text': DESCRIBE_PROMPT},
        {'type': 'image_url',
         'image_url': {'url': f'data:image/jpeg;base64,{data}'}}])


def build_decision_request(config: VlmConfig, description: str) -> dict:
    """Request 2: text only, classify the description."""
    return _chat(config, DECIDE_PROMPT % description.replace('"', "'"))


def _json_object(text: str) -> dict:
    if not isinstance(text, str) or not text.strip():
        raise VlmFormatError('empty reply')
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end <= start:
        raise VlmFormatError('no JSON object in the reply')
    try:
        doc = json.loads(text[start:end + 1])
    except json.JSONDecodeError as exc:
        raise VlmFormatError(f'invalid JSON: {exc}') from exc
    if not isinstance(doc, dict):
        raise VlmFormatError('reply is not a JSON object')
    return doc


def parse_description(text: str) -> dict:
    """Validate the reply to request 1."""
    doc = _json_object(text)
    description, visibility = doc.get('description'), doc.get('visibility')
    if not isinstance(description, str) or len(description.strip()) < 3:
        raise VlmFormatError('description must be a sentence')
    if visibility not in VISIBILITIES:
        raise VlmFormatError(f'visibility must be one of {VISIBILITIES}')
    return {'description': description.strip()[:300],
            'visibility': visibility}


def parse_answer(text: str) -> dict:
    """Validate the model's reply; raise VlmFormatError if it is not valid.

    Accepts the JSON object alone or wrapped in a code fence / surrounding
    text. Field names, types and value sets are checked strictly.
    """
    doc = _json_object(text)
    decision = doc.get('decision')
    visibility = doc.get('visibility')
    confidence = doc.get('confidence')
    target = doc.get('target')
    if decision not in DECISIONS:
        raise VlmFormatError(f'decision must be one of {DECISIONS}')
    if visibility not in VISIBILITIES:
        raise VlmFormatError(f'visibility must be one of {VISIBILITIES}')
    if isinstance(confidence, bool) or not isinstance(confidence,
                                                      (int, float)):
        raise VlmFormatError('confidence must be a number')
    if not 0.0 <= float(confidence) <= 1.0:
        raise VlmFormatError('confidence must be within 0..1')
    if not isinstance(target, str) or not target.strip():
        raise VlmFormatError('target must be a non-empty string')
    return {'target': target.strip()[:80], 'confidence': float(confidence),
            'visibility': visibility, 'decision': decision}


def _reply_text(body: dict) -> str:
    try:
        return body['choices'][0]['message']['content']
    except (KeyError, IndexError, TypeError) as exc:
        raise VlmFormatError(f'unexpected response shape: {exc}') from exc


def _is_connection_failure(exc) -> bool:
    if isinstance(exc, urllib.error.HTTPError):
        return False
    reason = getattr(exc, 'reason', exc)
    return isinstance(reason, (ConnectionError, socket.gaierror)) or (
        isinstance(reason, OSError) and not isinstance(reason, TimeoutError)
        and not isinstance(reason, socket.timeout))


def request_once(config: VlmConfig, jpeg: bytes, job_id: int = 0,
                 context: Optional[dict] = None) -> VlmOutcome:
    """Blocking: one crop -> one outcome. Used by the worker and offline."""
    started = time.monotonic()

    def outcome(kind, **fields):
        return VlmOutcome(job_id=job_id, kind=kind,
                          latency_s=time.monotonic() - started,
                          model=config.model, context=context, **fields)

    if not config.endpoint or not config.model:
        return outcome('unavailable', error='no endpoint or model configured')
    headers = {'Content-Type': 'application/json'}
    if config.api_key:
        headers['Authorization'] = f'Bearer {config.api_key}'
    url = config.endpoint.rstrip('/') + '/chat/completions'

    def post(body):
        request = urllib.request.Request(
            url, data=json.dumps(body).encode('utf-8'), headers=headers,
            method='POST')
        with urllib.request.urlopen(request,
                                    timeout=config.timeout_s) as response:
            return _reply_text(json.loads(response.read().decode('utf-8')))

    raw = ''
    try:
        raw = post(build_request(config, jpeg))
        seen = parse_description(raw)
        second = post(build_decision_request(config, seen['description']))
        raw = raw + ' || ' + second
        doc = _json_object(second)
        doc['visibility'] = seen['visibility']
        return outcome('answer', raw=raw,
                       description=seen['description'],
                       **parse_answer(json.dumps(doc)))
    except VlmFormatError as exc:
        return outcome('error', error=f'malformed answer: {exc}', raw=raw)
    except (TimeoutError, socket.timeout):
        return outcome('error', error=f'timeout after {config.timeout_s} s')
    except urllib.error.HTTPError as exc:
        return outcome('error', error=f'HTTP {exc.code}')
    except (urllib.error.URLError, OSError) as exc:
        if 'timed out' in str(exc):
            return outcome('error',
                           error=f'timeout after {config.timeout_s} s')
        if _is_connection_failure(exc):
            return outcome('unavailable', error=f'cannot connect: {exc}')
        return outcome('error', error=str(exc))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        return outcome('error', error=f'response is not JSON: {exc}')


def probe(config: VlmConfig) -> bool:
    """True if the endpoint answers a model listing."""
    if not config.endpoint or not config.model:
        return False
    headers = {}
    if config.api_key:
        headers['Authorization'] = f'Bearer {config.api_key}'
    try:
        request = urllib.request.Request(
            config.endpoint.rstrip('/') + '/models', headers=headers)
        with urllib.request.urlopen(request, timeout=3.0) as response:
            return 200 <= response.status < 300
    except (urllib.error.URLError, OSError, ValueError):
        return False


class VlmWorker:
    """One background thread; one request at a time; never blocks callers."""

    def __init__(self, config: VlmConfig, requester=request_once,
                 prober=probe):
        self.config = config
        self._requester = requester
        self._prober = prober
        self._jobs = queue.Queue(maxsize=1)
        self._results = queue.Queue()
        self._status = VLM_UNAVAILABLE
        self._busy = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._next_id = 1
        self.requests = 0
        self.replaced = 0
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='vlm_worker')
        self._thread.start()

    # -- caller side -----------------------------------------------------
    @property
    def status(self) -> int:
        with self._lock:
            return self._status

    @property
    def usable(self) -> bool:
        """An endpoint is configured and not known to be unreachable."""
        return self.status != VLM_UNAVAILABLE

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy or not self._jobs.empty()

    def submit(self, bgr, bbox, context: Optional[dict] = None) -> int:
        """Queue a crop of ``bgr`` (copied here); returns the job id.

        Never blocks. A job still waiting is replaced by this one and
        reported as an error outcome ("superseded") so that its track does
        not stay pending.
        """
        x0, y0, x1, y1 = crop_region(bgr.shape, bbox,
                                     self.config.pad_fraction)
        job = {'id': self._next_id, 'crop': bgr[y0:y1, x0:x1].copy(),
               'context': context}
        self._next_id += 1
        try:
            self._jobs.put_nowait(job)
        except queue.Full:
            try:
                old = self._jobs.get_nowait()
                self.replaced += 1
                self._results.put(VlmOutcome(
                    job_id=old['id'], kind='error', latency_s=0.0,
                    error='superseded by a newer crop',
                    context=old['context']))
            except queue.Empty:
                pass
            self._jobs.put_nowait(job)
        return job['id']

    def poll(self):
        """Completed outcomes since the last call (non-blocking)."""
        out = []
        while True:
            try:
                out.append(self._results.get_nowait())
            except queue.Empty:
                return out

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    # -- worker thread ---------------------------------------------------
    def _set_status(self, status):
        with self._lock:
            self._status = status

    def _run(self):
        last_probe = None
        while not self._stop.is_set():
            now = time.monotonic()
            if self.status == VLM_UNAVAILABLE and (
                    last_probe is None
                    or now - last_probe >= self.config.probe_interval_s):
                last_probe = now
                if self._prober(self.config):
                    self._set_status(VLM_AVAILABLE)
            try:
                job = self._jobs.get(timeout=0.05)
            except queue.Empty:
                continue
            with self._lock:
                self._busy = True
            try:
                crop = job['crop']
                jpeg = encode_crop(crop, (0, 0, crop.shape[1], crop.shape[0]),
                                   VlmConfig(
                                       max_side_px=self.config.max_side_px,
                                       min_side_px=self.config.min_side_px,
                                       jpeg_quality=self.config.jpeg_quality,
                                       pad_fraction=0.0))
                self.requests += 1
                result = self._requester(self.config, jpeg, job['id'],
                                         job['context'])
            except Exception as exc:    # never let the worker die
                result = VlmOutcome(job_id=job['id'], kind='error',
                                    latency_s=0.0, error=f'worker: {exc}',
                                    context=job['context'])
            self._set_status({'answer': VLM_AVAILABLE, 'error': VLM_ERROR,
                              'unavailable': VLM_UNAVAILABLE}[result.kind])
            if result.kind == 'unavailable':
                last_probe = time.monotonic()
            self._results.put(result)
            with self._lock:
                self._busy = False
