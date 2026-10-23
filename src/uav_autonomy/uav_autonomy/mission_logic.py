"""Mission state machine (Phase 10).

Pure Python; no ROS, no simulator ground truth. The ROS wrapper is
mission_manager_node.py.

The mission manager ties the existing functions into one mission:

    preflight checks -> START -> takeoff -> search (coverage, inspection,
    debris avoidance, survivor perception and localisation all running)
    -> return -> land -> survivor report

It decides three things only: whether the flight may start, when the search
ends, and what goes into the report. It never commands PX4: its commands go
to flight_controller, which validates them (mission_interface.py). Flying,
avoiding, searching and locating stay with the nodes that already do them.
"""

from dataclasses import asdict, dataclass, field
import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

from uav_autonomy.mission_interface import (
    CMD_NONE, CMD_RETURN_HOME, CMD_START)

PHASE_INIT = 0
PHASE_PREFLIGHT = 1
PHASE_STARTING = 2
PHASE_TAKEOFF = 3
PHASE_SEARCH = 4
PHASE_RETURN = 5
PHASE_LANDING = 6
PHASE_COMPLETE = 7
PHASE_REFUSED = 8
PHASE_FAILSAFE = 9
PHASE_NAMES = {0: 'INIT', 1: 'PREFLIGHT', 2: 'STARTING', 3: 'TAKEOFF',
               4: 'SEARCH', 5: 'RETURN', 6: 'LANDING', 7: 'COMPLETE',
               8: 'REFUSED', 9: 'FAILSAFE'}
TERMINAL = (PHASE_COMPLETE, PHASE_REFUSED, PHASE_FAILSAFE)

VLM_AVAILABLE = 0          # uav_interfaces/SurvivorDetectionArray
STOP_AREA_COVERED = 'area_covered'
STOP_FIRST_CONFIRMED = 'first_confirmed'
LOC_LOCALIZED = 2          # uav_interfaces/SurvivorLocation

EARTH_RADIUS_M = 6371000.0
REASON_COVERED = 'search area covered'
REASON_CONFIRMED = 'a survivor is confirmed and located'
OBJECTIVE_REASONS = (REASON_COVERED, REASON_CONFIRMED)


@dataclass
class MissionSpec:
    """What the mission is. Phase 11 will produce this from language."""

    name: str = 'search'
    # search rectangle relative to the takeoff point [m]
    north_min: float = -4.0
    north_max: float = 4.0
    east_min: float = -4.0
    east_max: float = 4.0
    altitude_m: float = 5.0
    time_budget_s: float = 240.0          # for the search phase
    stop_policy: str = STOP_AREA_COVERED
    require_vlm: bool = False             # refuse to start without a VLM
    require_debris_protection: bool = False
    battery_return_fraction: float = 0.25
    perception_timeout_s: float = 3.0
    report_min_observations: int = 15
    # limits a mission must respect; the defaults are the validated small
    # world, a place catalogue may raise them for a larger one
    limit_range_m: float = 25.0           # area corners from the takeoff
    limit_extent_m: float = 50.0          # longest side of the area
    limit_time_budget_s: float = 300.0
    # where the search should begin (relative to takeoff); NaN = wherever
    # is nearest
    priority_north: float = float('nan')
    priority_east: float = float('nan')

    def problems(self) -> List[str]:
        """Reasons why this mission cannot be flown (empty = valid)."""
        out = []
        for lo, hi, axis in ((self.north_min, self.north_max, 'north'),
                             (self.east_min, self.east_max, 'east')):
            if not (math.isfinite(lo) and math.isfinite(hi)):
                out.append(f'{axis} bounds are not finite')
            elif hi - lo < 1.0:
                out.append(f'{axis} extent is below 1 m')
            elif hi - lo > self.limit_extent_m:
                out.append(f'{axis} extent is above '
                           f'{self.limit_extent_m:g} m')
        corners = [math.hypot(n, e) for n in (self.north_min, self.north_max)
                   for e in (self.east_min, self.east_max)]
        if all(math.isfinite(c) for c in corners) \
                and max(corners) > self.limit_range_m:
            out.append('part of the area is more than '
                       f'{self.limit_range_m:g} m from the takeoff '
                       'point (flight controller limit)')
        if not 2.0 <= self.altitude_m <= 15.0:
            out.append('altitude must be between 2 and 15 m')
        if not 10.0 <= self.time_budget_s <= self.limit_time_budget_s:
            out.append('time budget must be between 10 and '
                       f'{self.limit_time_budget_s:g} s')
        if self.stop_policy not in (STOP_AREA_COVERED, STOP_FIRST_CONFIRMED):
            out.append(f'unknown stop policy {self.stop_policy!r}')
        if not 0.0 <= self.battery_return_fraction < 1.0:
            out.append('battery return fraction must be in [0, 1)')
        return out

    @staticmethod
    def from_dict(data: dict) -> 'MissionSpec':
        """Build from a mission file's content; unknown keys are an error."""
        data = dict(data)
        area = data.pop('search_area', None)
        if area is not None:
            data['north_min'], data['north_max'] = (float(v) for v in
                                                    area['north'])
            data['east_min'], data['east_max'] = (float(v) for v in
                                                  area['east'])
        known = MissionSpec.__dataclass_fields__
        unknown = sorted(set(data) - set(known))
        if unknown:
            raise ValueError(f'unknown mission fields: {unknown}')
        spec = MissionSpec()
        for key, value in data.items():
            setattr(spec, key, type(getattr(spec, key))(value))
        return spec

    @staticmethod
    def load(path: str) -> 'MissionSpec':
        with open(path) as handle:
            return MissionSpec.from_dict(json.load(handle))


@dataclass
class Inputs:
    """Everything the mission manager looks at in one cycle."""

    now: float
    flight_state: Optional[str] = None
    flight_age: float = float('inf')
    detections_age: float = float('inf')   # survivor perception stream
    vlm_status: Optional[int] = None
    locations_age: float = float('inf')    # localisation stream
    locations: Sequence[dict] = ()
    planner_present: bool = False
    search_complete: bool = False          # planner reports the path flown
    coverage: float = 0.0
    inspections_done: int = 0
    avoidance_age: Optional[float] = None  # None: no debris chain running
    # time since the avoidance planner last reported debris perception that
    # was not STALE (None: never reported)
    debris_perception_age: Optional[float] = None
    avoidance_manoeuvres: int = 0
    battery_remaining: Optional[float] = None


@dataclass
class Decision:
    command: int = CMD_NONE
    sequence: int = 0
    reason: str = ''


FLIGHT_TO_PHASE = {
    'PRESTREAM': PHASE_TAKEOFF, 'REQUEST_OFFBOARD': PHASE_TAKEOFF,
    'WAIT_OFFBOARD': PHASE_TAKEOFF, 'WAIT_PREFLIGHT': PHASE_TAKEOFF,
    'REQUEST_ARM': PHASE_TAKEOFF, 'WAIT_ARMED': PHASE_TAKEOFF,
    'TAKEOFF': PHASE_TAKEOFF, 'SEARCH': PHASE_SEARCH,
    'RETURN_HOME': PHASE_RETURN, 'REQUEST_LAND': PHASE_LANDING,
    'LANDING': PHASE_LANDING, 'COMPLETE': PHASE_COMPLETE,
    'FAILSAFE': PHASE_FAILSAFE}


def classify(location: dict, min_observations: int) -> str:
    """Honest label of one located entry for the report."""
    if int(location.get('obs', 0)) < min_observations:
        return 'LOW_CONFIDENCE'
    if location.get('confirmed'):
        return 'SURVIVOR_CONFIRMED'
    if location.get('rejected'):
        return 'REJECTED_BY_VLM'
    return 'UNVERIFIED_CANDIDATE'


def local_to_geodetic(north: float, east: float, ref_lat: float,
                      ref_lon: float) -> Tuple[float, float]:
    """Latitude/longitude of a local NED point (small-area approximation)."""
    lat = ref_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(
        east / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat))))
    return lat, lon


class MissionManager:
    """Preflight gate, search supervision and report."""

    PREFLIGHT_TIMEOUT_S = 30.0
    PREFLIGHT_STABLE_S = 2.0       # every check must hold this long
    START_TIMEOUT_S = 10.0
    STREAM_TIMEOUT_S = 2.0         # preflight freshness of a required stream
    BATTERY_START_MARGIN = 0.10
    CONTROLLER_LOST_S = 10.0       # flight status silent this long in flight

    def __init__(self, spec: MissionSpec):
        self.spec = spec
        self.phase = PHASE_INIT
        self.timeline: List[dict] = []
        self.sequence = 0
        self.pending = Decision()
        self.end_reason = ''
        self.refusal: List[str] = []
        self.failed_checks: List[str] = []
        self.t_preflight: Optional[float] = None
        self.t_checks_ok: Optional[float] = None
        self.t_start: Optional[float] = None
        self.t_search: Optional[float] = None
        self.t_search_end: Optional[float] = None
        self.t_end: Optional[float] = None
        self.vlm_seen_available = False
        self.min_battery: Optional[float] = None
        self.last_inputs: Optional[Inputs] = None

    # -- helpers -----------------------------------------------------------
    def _enter(self, now, phase, note=''):
        if phase == self.phase:
            return
        self.phase = phase
        self.timeline.append({'t': round(now, 3), 'phase': PHASE_NAMES[phase],
                              'note': note})
        if phase in TERMINAL:
            self.t_end = now

    def _command(self, command, reason) -> Decision:
        self.sequence += 1
        self.pending = Decision(command, self.sequence, reason)
        return self.pending

    def checks(self, x: Inputs) -> List[Tuple[str, bool, str]]:
        """Preflight checks: (name, ok, detail)."""
        s = self.spec
        out = [('mission', not s.problems(), '; '.join(s.problems())),
               ('flight_controller',
                x.flight_age <= self.STREAM_TIMEOUT_S
                and x.flight_state == 'WAIT_FOR_DATA',
                f'state {x.flight_state}, age {x.flight_age:.1f} s'),
               ('survivor_perception',
                x.detections_age <= self.STREAM_TIMEOUT_S,
                f'age {x.detections_age:.1f} s'),
               ('survivor_localization',
                x.locations_age <= self.STREAM_TIMEOUT_S,
                f'age {x.locations_age:.1f} s'),
               ('search_planner', x.planner_present, 'no publisher on the '
                'goal topic' if not x.planner_present else '')]
        if s.require_vlm:
            out.append(('vlm', x.vlm_status == VLM_AVAILABLE,
                        f'status {x.vlm_status}'))
        if s.require_debris_protection:
            ok = x.avoidance_age is not None and \
                x.avoidance_age <= self.STREAM_TIMEOUT_S
            out.append(('debris_avoidance', ok,
                        'not running' if x.avoidance_age is None
                        else f'age {x.avoidance_age:.1f} s'))
        if x.battery_remaining is not None:
            need = s.battery_return_fraction + self.BATTERY_START_MARGIN
            out.append(('battery', x.battery_remaining >= need,
                        f'{x.battery_remaining:.2f} remaining, {need:.2f} '
                        'needed'))
        return out

    def _search_end_reason(self, x: Inputs) -> Optional[str]:
        s = self.spec
        if x.now - self.t_search > s.time_budget_s:
            return f'time budget of {s.time_budget_s:.0f} s used'
        if x.battery_remaining is not None \
                and x.battery_remaining <= s.battery_return_fraction:
            return (f'battery at {x.battery_remaining:.2f} (return level '
                    f'{s.battery_return_fraction:.2f})')
        if x.detections_age > s.perception_timeout_s:
            return (f'survivor perception lost for {x.detections_age:.1f} s')
        if x.locations_age > s.perception_timeout_s:
            return (f'survivor localisation lost for '
                    f'{x.locations_age:.1f} s')
        if s.require_debris_protection and (
                x.avoidance_age is None
                or x.avoidance_age > s.perception_timeout_s):
            return 'debris avoidance lost'
        if s.require_debris_protection \
                and x.debris_perception_age is not None \
                and x.debris_perception_age > s.perception_timeout_s:
            return (f'debris perception stale for '
                    f'{x.debris_perception_age:.1f} s')
        if s.stop_policy == STOP_FIRST_CONFIRMED and any(
                loc.get('confirmed')
                and int(loc.get('state', 0)) == LOC_LOCALIZED
                and int(loc.get('obs', 0)) >= s.report_min_observations
                for loc in x.locations):
            return REASON_CONFIRMED
        return None

    # -- main entry --------------------------------------------------------
    def update(self, x: Inputs) -> Decision:
        """One cycle. Returns the command to (re)send, CMD_NONE if none."""
        self.last_inputs = x
        if x.vlm_status == VLM_AVAILABLE:
            self.vlm_seen_available = True
        if x.battery_remaining is not None:
            self.min_battery = x.battery_remaining if self.min_battery is \
                None else min(self.min_battery, x.battery_remaining)
        if self.phase in TERMINAL:
            return Decision()

        if self.phase == PHASE_INIT:
            self.t_preflight = x.now
            self._enter(x.now, PHASE_PREFLIGHT)

        if self.phase == PHASE_PREFLIGHT:
            results = self.checks(x)
            self.failed_checks = [f'{name}: {detail}' if detail else name
                                  for name, ok, detail in results if not ok]
            if not self.failed_checks:
                if self.t_checks_ok is None:
                    self.t_checks_ok = x.now
                if x.now - self.t_checks_ok >= self.PREFLIGHT_STABLE_S:
                    self.t_start = x.now
                    self._enter(x.now, PHASE_STARTING, 'all checks passed')
                    return self._command(CMD_START, 'preflight checks passed')
            else:
                self.t_checks_ok = None
                if x.now - self.t_preflight > self.PREFLIGHT_TIMEOUT_S \
                        or self.spec.problems():
                    self.refusal = list(self.failed_checks)
                    self._enter(x.now, PHASE_REFUSED,
                                '; '.join(self.failed_checks))
            return Decision()

        # in flight: follow the flight controller's state
        if x.flight_age > self.CONTROLLER_LOST_S:
            # nothing can be commanded any more and nothing more will be
            # learnt: conclude, so that a report exists. PX4's own
            # offboard-loss failsafe handles the vehicle.
            if self.phase == PHASE_SEARCH:
                self.t_search_end = x.now
            self.end_reason = ('flight controller status lost for '
                               f'{x.flight_age:.0f} s; PX4 failsafe expected')
            self.pending = Decision()
            self._enter(x.now, PHASE_FAILSAFE, self.end_reason)
            return Decision()
        fresh = x.flight_age <= self.STREAM_TIMEOUT_S
        mapped = FLIGHT_TO_PHASE.get(x.flight_state) if fresh else None

        if self.phase == PHASE_STARTING:
            if mapped is not None:
                self.pending = Decision()
                self._enter(x.now, mapped)
            elif x.now - self.t_start > self.START_TIMEOUT_S:
                self.refusal = ['flight controller did not start']
                self._enter(x.now, PHASE_REFUSED,
                            'flight controller did not start')
                return Decision()
            else:
                return self.pending
        elif mapped is not None and mapped != self.phase:
            if self.phase == PHASE_SEARCH and mapped != PHASE_SEARCH:
                self.t_search_end = x.now
                if not self.end_reason:
                    if mapped == PHASE_FAILSAFE:
                        self.end_reason = 'flight controller failsafe'
                    elif x.search_complete:
                        self.end_reason = REASON_COVERED
                    else:
                        self.end_reason = ('flight controller ended the '
                                           'search (no valid goal or its own '
                                           'time limit)')
                self.pending = Decision()
            self._enter(x.now, mapped, self.end_reason
                        if mapped in (PHASE_RETURN, PHASE_FAILSAFE) else '')

        if self.phase == PHASE_SEARCH:
            if self.t_search is None:
                self.t_search = x.now
            if self.pending.command == CMD_RETURN_HOME:
                return self.pending          # repeat until the state changes
            reason = self._search_end_reason(x)
            if reason is not None:
                self.end_reason = reason
                return self._command(CMD_RETURN_HOME, reason)
        return Decision()

    # -- report ------------------------------------------------------------
    def report(self, home_ne: Optional[Sequence[float]] = None,
               origin: Optional[Sequence[float]] = None) -> dict:
        """The mission report. home_ne: takeoff point in the local frame;
        origin: (ref_lat, ref_lon) of the local frame, if known."""
        x = self.last_inputs or Inputs(now=0.0)
        s = self.spec
        entries = []
        for loc in x.locations:
            north, east, down = (float(v) for v in loc['ned'])
            item = {
                'id': int(loc['id']),
                'classification': classify(loc, s.report_min_observations),
                'local_ned_m': [round(north, 2), round(east, 2),
                                round(down, 2)],
                'horizontal_sigma_m': round(float(loc.get('sigma_h', 0.0)),
                                            2),
                'localized': int(loc.get('state', 0)) == LOC_LOCALIZED,
                'observations': int(loc.get('obs', 0))}
            if home_ne is not None:
                item['from_takeoff_m'] = {
                    'north': round(north - home_ne[0], 2),
                    'east': round(east - home_ne[1], 2)}
            if origin is not None:
                lat, lon = local_to_geodetic(north, east, origin[0],
                                             origin[1])
                item['latitude_deg'] = round(lat, 7)
                item['longitude_deg'] = round(lon, 7)
            entries.append(item)
        order = {'SURVIVOR_CONFIRMED': 0, 'UNVERIFIED_CANDIDATE': 1,
                 'REJECTED_BY_VLM': 2, 'LOW_CONFIDENCE': 3}
        entries.sort(key=lambda e: (order[e['classification']], e['id']))
        counts: Dict[str, int] = {}
        for e in entries:
            counts[e['classification']] = counts.get(
                e['classification'], 0) + 1
        notes = []
        if not self.vlm_seen_available:
            notes.append('No vision-language model was available: no entry '
                         'could be confirmed; candidates are unverified.')
        if counts.get('UNVERIFIED_CANDIDATE'):
            notes.append('Unverified candidates may be objects that only '
                         'look like a person to the colour/shape detector.')
        if self.phase == PHASE_REFUSED:
            notes.append('The flight was not started.')
        search_time = None
        if self.t_search is not None:
            search_time = round((self.t_search_end or x.now) - self.t_search,
                                1)
        return {
            'mission': asdict(s),
            'outcome': PHASE_NAMES[self.phase],
            # landed safely is not the same as having done the job
            'objective_met': self.phase == PHASE_COMPLETE
            and self.end_reason in OBJECTIVE_REASONS,
            'search_end_reason': self.end_reason,
            'refused_because': self.refusal,
            'timeline': self.timeline,
            'duration_s': None if self.t_start is None else round(
                (self.t_end or x.now) - self.t_start, 1),
            'search_time_s': search_time,
            'coverage_fraction': round(float(x.coverage), 3),
            'inspections': int(x.inspections_done),
            'avoidance_manoeuvres': int(x.avoidance_manoeuvres),
            'vlm_available': self.vlm_seen_available,
            'battery_min': self.min_battery,
            'summary': counts,
            'entries': entries,
            'notes': notes}
