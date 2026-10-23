"""Natural-language mission interpretation (Phase 11).

Pure Python (standard library only); no ROS, no simulator. It runs BEFORE a
flight, on the ground: an instruction in plain language becomes a mission
file, or a refusal with reasons.

    instruction ─► interpreter ─► structured fields ─► checks ─► MissionSpec
                   (language model                     (deterministic,
                    or keyword rules)                   this file)

The rule of the whole project applies to language too: the interpreter only
PROPOSES. What it proposes is a handful of fields from a fixed vocabulary;
it can never output coordinates, and every value is checked here before a
mission exists:

  * the task must be a survivor search (the only mission the system flies)
  * the place must be one of the named places in the catalogue
  * every number must literally appear in the instruction (a model that
    invents a value is caught)
  * nothing is clamped: a value outside the limits refuses the mission
  * anything unclear refuses the mission and says what to clarify
  * the operator reads the interpreted mission back before it is flown
"""

from dataclasses import dataclass, field
import difflib
import json
import math
import re
import time
from typing import Dict, List, Optional, Tuple
import urllib.error
import urllib.request

from uav_autonomy.mission_logic import (
    MissionSpec, STOP_AREA_COVERED, STOP_FIRST_CONFIRMED)

TASK_SEARCH = 'search_survivors'
TASK_OTHER = 'other'
STATUS_ACCEPTED = 'ACCEPTED'
STATUS_REFUSED = 'REFUSED'
STATUS_ERROR = 'ERROR'            # the interpreter itself failed

FIELDS = ('task', 'place', 'altitude_m', 'time_limit_s', 'stop_when',
          'avoid_debris', 'confirmed_only', 'battery_return_percent',
          'unclear', 'priority_place')

NUMBER_WORDS = {
    'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
    'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10, 'eleven': 11,
    'twelve': 12, 'fifteen': 15, 'twenty': 20, 'thirty': 30, 'forty': 40,
    'fifty': 50, 'sixty': 60, 'ninety': 90}


class InterpretationError(ValueError):
    """The interpreter's output is not usable."""


@dataclass
class Place:
    """A named search area, metres relative to the takeoff point."""

    key: str
    description: str
    north: Tuple[float, float]
    east: Tuple[float, float]
    aliases: Tuple[str, ...] = ()
    default_altitude_m: float = 5.0
    min_altitude_m: float = 4.0       # clears the 3 m walls and roof slabs
    max_altitude_m: float = 15.0
    default_time_budget_s: float = 240.0
    limit_range_m: float = 25.0
    limit_extent_m: float = 50.0
    limit_time_budget_s: float = 300.0


@dataclass
class Result:
    """Outcome of interpreting one instruction."""

    status: str
    instruction: str
    interpreter: str = ''
    fields: Optional[dict] = None
    spec: Optional[MissionSpec] = None
    reasons: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)   # shown in the read-back
    raw: str = ''
    latency_s: float = 0.0

    def mission_dict(self) -> dict:
        """The mission file content (ACCEPTED only)."""
        s = self.spec
        return {
            'name': s.name,
            'search_area': {'north': [s.north_min, s.north_max],
                            'east': [s.east_min, s.east_max]},
            'altitude_m': s.altitude_m,
            'time_budget_s': s.time_budget_s,
            'stop_policy': s.stop_policy,
            'require_vlm': s.require_vlm,
            'require_debris_protection': s.require_debris_protection,
            'battery_return_fraction': s.battery_return_fraction,
            'limit_range_m': s.limit_range_m,
            'limit_extent_m': s.limit_extent_m,
            'limit_time_budget_s': s.limit_time_budget_s,
            **({} if math.isnan(s.priority_north) else {
                'priority_north': s.priority_north,
                'priority_east': s.priority_east})}


def load_catalogue(path: str) -> Dict[str, Place]:
    """Named places from a JSON file."""
    with open(path) as handle:
        data = json.load(handle)
    out = {}
    for key, item in data['places'].items():
        out[key] = Place(
            key=key, description=item['description'],
            north=(float(item['north'][0]), float(item['north'][1])),
            east=(float(item['east'][0]), float(item['east'][1])),
            aliases=tuple(a.lower() for a in item.get('aliases', ())),
            default_altitude_m=float(item.get('default_altitude_m', 5.0)),
            min_altitude_m=float(item.get('min_altitude_m', 4.0)),
            max_altitude_m=float(item.get('max_altitude_m', 15.0)),
            default_time_budget_s=float(
                item.get('default_time_budget_s', 240.0)),
            limit_range_m=float(item.get('limit_range_m', 25.0)),
            limit_extent_m=float(item.get('limit_extent_m', 50.0)),
            limit_time_budget_s=float(
                item.get('limit_time_budget_s', 300.0)))
    return out


# -- text helpers -------------------------------------------------------------

def normalise(text: str) -> str:
    """Lower case, number words as digits, single spaces."""
    text = ' '.join(text.lower().replace('-', ' ').split())
    for word, value in NUMBER_WORDS.items():
        text = re.sub(rf'\b{word}\b', str(value), text)
    return re.sub(r'\bhalf an? (minute|min)\b', r'30 seconds', text)


def numbers_in(text: str) -> List[float]:
    """Every number written in the instruction (digits or words)."""
    return [float(m) for m in re.findall(r'\d+(?:\.\d+)?', normalise(text))]


SAFETY_WORD = (r'(safety|failsafe|fail safe|limit|check|geofence|avoidance|'
               r'rule|restriction|preflight|instruction)s?')
OFF_WORD = (r'(ignore|disable|disabled|override|bypass|skip|turn off|'
            r'turned off|switch off|switched off|without|off)')
# either order: "ignore the safety checks", "avoidance turned off"
OVERRIDE = re.compile(
    rf'\b{OFF_WORD}\b[^.;]*\b{SAFETY_WORD}\b'
    rf'|\b{SAFETY_WORD}\b[^.;,]*\b{OFF_WORD}\b')
FOREIGN_UNIT = re.compile(
    r'\d\s*(feet|foot|ft|yards?|miles?|km|kilomet\w*|hours?|hrs?)\b')


def text_screen(instruction: str) -> List[str]:
    """Deterministic refusals that do not depend on any interpreter."""
    text = normalise(instruction)
    reasons = []
    if not text.strip():
        reasons.append('the instruction is empty')
    if len(instruction) > 600:
        reasons.append('the instruction is longer than 600 characters')
    if OVERRIDE.search(text):
        reasons.append('the instruction asks to ignore or disable a safety '
                       'function; that is never accepted')
    if FOREIGN_UNIT.search(text):
        reasons.append('only metres and seconds/minutes are supported')
    return reasons


# -- keyword interpreter (baseline, and the path without a language model) ---

SEARCH_VERB = re.compile(
    r'\b(search|look for|find|locate|scan|sweep|check|survey)\b')
PERSON = re.compile(
    r'\b(survivors?|people|persons?|victims?|casualt\w+|anyone|somebody|'
    r'someone|trapped|humans?)\b')


def interpret_rules(instruction: str, catalogue: Dict[str, Place]) -> dict:
    """Keyword extraction. Deterministic; understands only plain phrasing."""
    text = normalise(instruction)
    out = {name: None for name in FIELDS}
    out['unclear'] = ''
    out['task'] = TASK_SEARCH if SEARCH_VERB.search(text) \
        and PERSON.search(text) else TASK_OTHER

    # longest alias first; a place whose alias is contained in a longer
    # matched alias of another place does not count twice
    # "prioritize the X" / "starting with the X": X is where to begin, not
    # a second area. It is taken out of the text before the area is read.
    aliases = sorted(((alias, place.key) for place in catalogue.values()
                      for alias in place.aliases), key=lambda a: -len(a[0]))
    for alias, key in aliases:
        m = re.search(
            r'\b(prioriti[sz]e|prioriti[sz]ing|priority to|start(?:ing)? '
            r'(?:with|at|in)|begin(?:ning)? (?:with|at|in)|first (?:search|'
            rf'check))\s+(?:the\s+)?{re.escape(alias)}\b', text)
        if m:
            out['priority_place'] = key
            text = text[:m.start()] + ' ' + text[m.end():]
            break
    hits = []
    for place in catalogue.values():
        for alias in place.aliases:
            for m in re.finditer(rf'\b{re.escape(alias)}\b', text):
                hits.append((m.start(), m.end(), place.key))
    hits.sort(key=lambda h: h[0] - h[1])
    taken, places = [], []
    for start, end, key in hits:
        if any(start < e and end > s for s, e in taken):
            continue
        taken.append((start, end))
        if key not in places:
            places.append(key)
    if len(places) == 1:
        out['place'] = places[0]
    elif len(places) > 1:
        out['unclear'] = ('more than one place is named: '
                          + ', '.join(sorted(places)))

    m = re.search(r'(?:\bat\b|altitude(?: of)?|height(?: of)?|fly(?:ing)? '
                  r'at|from)\s*(\d+(?:\.\d+)?)\s*(?:m\b|met(?:re|er)s?)',
                  text)
    if m:
        out['altitude_m'] = float(m.group(1))
    m = re.search(r'(\d+(?:\.\d+)?)\s*(seconds?|secs?|s\b|minutes?|mins?)',
                  text)
    if m:
        value = float(m.group(1))
        out['time_limit_s'] = value * 60.0 if m.group(2).startswith('min') \
            else value
    if re.search(r'\b(as soon as|once you|when you|the moment|first '
                 r'(survivor|person|one))\b', text) \
            and re.search(r'\b(come back|return|stop|land)\b', text):
        out['stop_when'] = STOP_FIRST_CONFIRMED
    if re.search(r'\b(debris|falling|rubble falling|collapse hazard)\b',
                 text):
        out['avoid_debris'] = True
    if re.search(r'\b(confirmed|verified|only report|be sure|certain)\b',
                 text):
        out['confirmed_only'] = True
    m = re.search(r'(\d+(?:\.\d+)?)\s*(?:%|percent)', text)
    if m and 'batter' in text:
        out['battery_return_percent'] = float(m.group(1))
    return out


# -- language-model interpreter ----------------------------------------------

def build_prompt(instruction: str, catalogue: Dict[str, Place]) -> str:
    """One prompt; the reply must be a single JSON object."""
    places = '\n'.join(
        f'- "{p.key}": {p.description} (also called: '
        + ', '.join(p.aliases) + ')' for p in catalogue.values())
    return (
        'You convert an instruction for a search drone into a JSON object. '
        'Copy only what the instruction says. Use null for anything it does '
        'not say. Never invent a value. Write numbers as JSON numbers and '
        'true/false without quotes.\n\n'
        'Known places:\n' + places + '\n\n'
        'Reply with one JSON object with exactly these keys:\n'
        '- "task": "search_survivors" if the instruction asks to search for '
        'or find people or survivors; otherwise "other"\n'
        '- "place": the key of the one known place the instruction means, or '
        'null if it names no place, an unknown place, or coordinates. If '
        'it names both the building and the yard, that is "whole_site". '
        'Do not guess a place that is not mentioned\n'
        '- "altitude_m": the flight height in metres if stated, else null\n'
        '- "time_limit_s": the time limit in seconds if stated (minutes '
        'times 60), else null\n'
        '- "stop_when": "first_confirmed" if it says to stop or return as '
        'soon as a survivor is found; "area_covered" if it says to cover '
        'the whole area; else null\n'
        '- "avoid_debris": true if it mentions falling debris or asks to '
        'avoid debris, else null\n'
        '- "confirmed_only": true if it asks to report only confirmed or '
        'verified survivors, else null\n'
        '- "battery_return_percent": the battery level in percent at which '
        'to return, if stated, else null\n'
        '- "unclear": an empty string, or a short note if the instruction '
        'is contradictory or cannot be understood\n'
        '- "priority_place": the key of a known place the instruction says '
        'to prioritize or start with, else null. That place is not the '
        '"place" unless it is the only one named\n\n'
        'Instruction:\n"""' + instruction.replace('"""', '"') + '"""')


def parse_fields(text: str, catalogue: Dict[str, Place]) -> dict:
    """Strict validation of an interpreter's JSON reply."""
    if not isinstance(text, str) or not text.strip():
        raise InterpretationError('empty reply')
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end <= start:
        raise InterpretationError('no JSON object in the reply')
    try:
        doc = json.loads(text[start:end + 1])
    except json.JSONDecodeError as exc:
        raise InterpretationError(f'invalid JSON: {exc}') from exc
    if not isinstance(doc, dict):
        raise InterpretationError('reply is not a JSON object')
    unknown = sorted(set(doc) - set(FIELDS))
    if unknown:
        raise InterpretationError(f'unknown keys: {unknown}')
    out = {name: doc.get(name) for name in FIELDS}
    if out['task'] not in (TASK_SEARCH, TASK_OTHER):
        raise InterpretationError(f'invalid task {out["task"]!r}')
    if out['place'] is not None and out['place'] not in catalogue:
        raise InterpretationError(f'unknown place {out["place"]!r}')
    if out['priority_place'] is not None \
            and out['priority_place'] not in catalogue:
        raise InterpretationError(
            f'unknown priority place {out["priority_place"]!r}')
    for name in ('altitude_m', 'time_limit_s', 'battery_return_percent'):
        value = out[name]
        if value is None:
            continue
        if isinstance(value, str) and re.fullmatch(
                r'\s*\d+(\.\d+)?\s*', value):
            value = float(value)          # "6" written as a string
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise InterpretationError(f'{name} is not a number')
        out[name] = float(value)
    if out['stop_when'] not in (None, STOP_AREA_COVERED,
                                STOP_FIRST_CONFIRMED):
        raise InterpretationError(f'invalid stop_when {out["stop_when"]!r}')
    for name in ('avoid_debris', 'confirmed_only'):
        if isinstance(out[name], str) and out[name].strip().lower() in (
                'true', 'false'):
            out[name] = out[name].strip().lower() == 'true'
        if out[name] not in (None, True, False):
            raise InterpretationError(f'{name} is not true/false/null')
    if out['unclear'] is None:
        out['unclear'] = ''
    if not isinstance(out['unclear'], str):
        raise InterpretationError('unclear is not a string')
    return out


def ask_model(prompt: str, endpoint: str, model: str,
              timeout_s: float = 180.0) -> str:
    """One blocking chat request to an OpenAI-compatible endpoint."""
    body = {'model': model, 'temperature': 0, 'max_tokens': 250,
            'response_format': {'type': 'json_object'},
            'messages': [{'role': 'user', 'content': prompt}]}
    request = urllib.request.Request(
        endpoint.rstrip('/') + '/chat/completions',
        data=json.dumps(body).encode('utf-8'),
        headers={'Content-Type': 'application/json'}, method='POST')
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        reply = json.loads(response.read().decode('utf-8'))
    try:
        return reply['choices'][0]['message']['content']
    except (KeyError, IndexError, TypeError) as exc:
        raise InterpretationError('unexpected response format') from exc


# -- deterministic checks and mission construction ---------------------------

def grounding_problems(fields: dict, instruction: str) -> List[str]:
    """Numbers proposed by the interpreter must be in the instruction."""
    present = numbers_in(instruction)
    reasons = []

    def near(value, candidates):
        return any(abs(value - c) < 1e-6 for c in candidates)

    if fields['altitude_m'] is not None and not near(
            fields['altitude_m'], present):
        reasons.append(f'altitude {fields["altitude_m"]:g} m does not '
                       'appear in the instruction')
    if fields['time_limit_s'] is not None and not near(
            fields['time_limit_s'], present + [60.0 * n for n in present]):
        reasons.append(f'time limit {fields["time_limit_s"]:g} s does not '
                       'appear in the instruction')
    if fields['battery_return_percent'] is not None and not near(
            fields['battery_return_percent'], present):
        reasons.append(f'battery level {fields["battery_return_percent"]:g} '
                       '% does not appear in the instruction')
    return reasons


def place_is_mentioned(place: Place, instruction: str) -> bool:
    """True if one of the place's names appears in the text (a spelling
    mistake of up to about one letter in five is tolerated)."""
    tokens = re.findall(r'[a-z]+', normalise(instruction))
    for alias in place.aliases:
        words = alias.split()
        for i in range(len(tokens) - len(words) + 1):
            window = ' '.join(tokens[i:i + len(words)])
            if difflib.SequenceMatcher(None, alias, window).ratio() > 0.8:
                return True
    return False


def cross_check(fields: dict, instruction: str,
                catalogue: Dict[str, Place]):
    """Check a language model's proposal against the text itself.

    Returns (fields to use, reasons to refuse, notes for the operator).
    The keyword rules are the reference for what the text literally says:

      place     must be mentioned in the text and must not contradict the
                keyword reading; several places named -> refuse
      safety-increasing flags (debris protection, confirmed-only): on if
                EITHER the model or the keywords say so
      safety-decreasing choice (stop at the first survivor instead of
                covering the area): only if BOTH say so
    """
    literal = interpret_rules(instruction, catalogue)
    out = dict(fields)
    reasons, notes = [], []
    if literal['unclear']:
        reasons.append(literal['unclear'] + '; name exactly one search area')
    elif fields['place'] is not None:
        place = catalogue[fields['place']]
        if not place_is_mentioned(place, instruction):
            reasons.append(f'the interpreter chose "{place.key}", which the '
                           'instruction does not mention')
        elif literal['place'] not in (None, fields['place']):
            reasons.append(
                f'the instruction reads as "{literal["place"]}" but the '
                f'interpreter chose "{fields["place"]}"; name the area '
                'more precisely')
    # where to begin comes from the wording only (a model may not add it)
    if fields.get('priority_place') != literal['priority_place']:
        if fields.get('priority_place'):
            notes.append('the priority place proposed by the interpreter is '
                         'not stated in the wording; ignored')
        out['priority_place'] = literal['priority_place']
    for name, label in (('avoid_debris', 'debris protection'),
                        ('confirmed_only', 'confirmed-only reporting')):
        if bool(fields[name]) != bool(literal[name]):
            notes.append(f'{label} switched on (stated by '
                         + ('the interpreter' if fields[name]
                            else 'the wording') + ' only)')
        out[name] = True if (fields[name] or literal[name]) else None
    if (fields['stop_when'] == STOP_FIRST_CONFIRMED) != (
            literal['stop_when'] == STOP_FIRST_CONFIRMED):
        notes.append('"stop at the first survivor" is not clearly stated; '
                     'the whole area will be covered instead')
        out['stop_when'] = None
    return out, reasons, notes


def build_mission(fields: dict, instruction: str,
                  catalogue: Dict[str, Place]):
    """Fields -> (MissionSpec or None, reasons for refusal)."""
    reasons = list(text_screen(instruction))
    if fields['task'] != TASK_SEARCH:
        reasons.append('only survivor-search missions can be flown')
    if fields['unclear']:
        reasons.append(f'the instruction is unclear: {fields["unclear"]}')
    place = catalogue.get(fields['place']) if fields['place'] else None
    if place is None:
        reasons.append('no known search area is named; known places: '
                       + ', '.join(sorted(catalogue)))
    reasons += grounding_problems(fields, instruction)
    if reasons:
        return None, reasons

    spec = MissionSpec(
        name=f'search_{place.key}',
        north_min=place.north[0], north_max=place.north[1],
        east_min=place.east[0], east_max=place.east[1],
        altitude_m=place.default_altitude_m,
        time_budget_s=place.default_time_budget_s,
        limit_range_m=place.limit_range_m,
        limit_extent_m=place.limit_extent_m,
        limit_time_budget_s=place.limit_time_budget_s)
    if fields['altitude_m'] is not None:
        spec.altitude_m = fields['altitude_m']
    if fields['time_limit_s'] is not None:
        spec.time_budget_s = fields['time_limit_s']
    if fields['stop_when'] is not None:
        spec.stop_policy = fields['stop_when']
    spec.require_debris_protection = bool(fields['avoid_debris'])
    # stopping at the first confirmed survivor can only happen with a VLM
    spec.require_vlm = bool(fields['confirmed_only']) \
        or spec.stop_policy == STOP_FIRST_CONFIRMED
    if fields['battery_return_percent'] is not None:
        spec.battery_return_fraction = \
            fields['battery_return_percent'] / 100.0

    priority = catalogue.get(fields.get('priority_place') or '')
    if priority is not None and priority.key != place.key:
        centre = ((priority.north[0] + priority.north[1]) / 2.0,
                  (priority.east[0] + priority.east[1]) / 2.0)
        if not (place.north[0] <= centre[0] <= place.north[1]
                and place.east[0] <= centre[1] <= place.east[1]):
            return None, [f'the priority place "{priority.key}" is not '
                          f'inside the search area "{place.key}"']
        spec.priority_north, spec.priority_east = centre
    reasons = list(spec.problems())            # nothing is clamped
    if spec.altitude_m < place.min_altitude_m:
        reasons.append(
            f'altitude {spec.altitude_m:g} m is below the minimum of '
            f'{place.min_altitude_m:g} m for "{place.key}" (structure)')
    if spec.altitude_m > place.max_altitude_m:
        reasons.append(
            f'altitude {spec.altitude_m:g} m is above the maximum of '
            f'{place.max_altitude_m:g} m for "{place.key}"')
    return (None, reasons) if reasons else (spec, [])


def interpret(instruction: str, catalogue: Dict[str, Place],
              interpreter: str = 'rules', endpoint: str = '',
              model: str = '', ask=ask_model) -> Result:
    """Instruction -> Result. `ask` is injectable for tests."""
    started = time.monotonic()
    result = Result(status=STATUS_ERROR, instruction=instruction,
                    interpreter=interpreter)
    screen = text_screen(instruction)
    if screen:                       # refused before any model sees it
        result.status, result.reasons = STATUS_REFUSED, screen
        return result
    try:
        if interpreter == 'rules':
            fields = interpret_rules(instruction, catalogue)
            result.raw = json.dumps(fields)
        elif interpreter == 'llm':
            if not endpoint or not model:
                raise InterpretationError('no language model configured')
            result.raw = ask(build_prompt(instruction, catalogue), endpoint,
                             model)
            fields = parse_fields(result.raw, catalogue)
            fields, check_reasons, result.notes = cross_check(
                fields, instruction, catalogue)
            if check_reasons:
                result.fields = fields
                result.status, result.reasons = STATUS_REFUSED, check_reasons
                result.latency_s = time.monotonic() - started
                return result
        else:
            raise InterpretationError(f'unknown interpreter {interpreter!r}')
    except InterpretationError as exc:
        result.reasons = [f'interpretation failed: {exc}']
        result.latency_s = time.monotonic() - started
        return result
    except (urllib.error.URLError, OSError, TimeoutError) as exc:
        result.reasons = [f'language model not reachable: {exc}']
        result.latency_s = time.monotonic() - started
        return result
    result.fields = fields
    spec, reasons = build_mission(fields, instruction, catalogue)
    result.latency_s = time.monotonic() - started
    if spec is None:
        result.status, result.reasons = STATUS_REFUSED, reasons
    else:
        result.status, result.spec = STATUS_ACCEPTED, spec
    return result


def read_back(result: Result, catalogue: Dict[str, Place]) -> str:
    """What the operator must confirm before the mission is flown."""
    if result.status != STATUS_ACCEPTED:
        return ('MISSION NOT ACCEPTED\n  instruction: '
                f'"{result.instruction}"\n' + '\n'.join(
                    f'  - {r}' for r in result.reasons))
    s = result.spec
    place = catalogue[result.fields['place']]
    lines = [
        'INTERPRETED MISSION (confirm before flight)',
        f'  instruction : "{result.instruction}"',
        f'  task        : search for survivors',
        f'  area        : {place.key} — {place.description}',
        f'                north {s.north_min:g} to {s.north_max:g} m, east '
        f'{s.east_min:g} to {s.east_max:g} m from the takeoff point',
        f'  altitude    : {s.altitude_m:g} m'
        + ('' if result.fields['altitude_m'] is not None else ' (default)'),
        f'  time budget : {s.time_budget_s:g} s'
        + ('' if result.fields['time_limit_s'] is not None
           else ' (default)'),
        f'  stop        : '
        + ('when a survivor is confirmed and located'
           if s.stop_policy == STOP_FIRST_CONFIRMED
           else 'when the area is covered'),
        f'  debris      : '
        + ('protection REQUIRED (will not start or continue without it)'
           if s.require_debris_protection
           else 'protection not required by the instruction'),
        f'  VLM         : '
        + ('REQUIRED (will not start without it)' if s.require_vlm
           else 'not required; candidates may be reported unverified'),
        f'  battery     : return at {100 * s.battery_return_fraction:g} %']
    if not math.isnan(s.priority_north):
        lines.append(f'  start near  : north {s.priority_north:g} m, east '
                     f'{s.priority_east:g} m ('
                     f'{result.fields["priority_place"]} first)')
    lines += [f'  note        : {note}' for note in result.notes]
    return '\n'.join(lines)
