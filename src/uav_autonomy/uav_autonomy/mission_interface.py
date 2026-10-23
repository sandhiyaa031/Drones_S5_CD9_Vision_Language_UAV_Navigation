"""Rules the flight controller applies to a mission command (no ROS).

The mission manager decides when the flight starts and when the mission
ends; flight_controller is the only PX4 command authority and acts on a
command only if every rule here holds.
"""

from typing import Optional, Tuple

CMD_NONE = 0
CMD_START = 1
CMD_RETURN_HOME = 2
CMD_LAND_NOW = 3
COMMAND_NAMES = {0: 'NONE', 1: 'START', 2: 'RETURN_HOME', 3: 'LAND_NOW'}

COMMAND_TIMEOUT_S = 2.0
START_STATES = ('WAIT_FOR_DATA',)
RETURN_STATES = ('SEARCH', 'HOLD', 'WAYPOINT_1', 'WAYPOINT_2')
LAND_STATES = RETURN_STATES + ('RETURN_HOME',)


def validate_mission_command(command: int, sequence: int,
                             last_sequence: int, age_s: float, state: str
                             ) -> Tuple[bool, Optional[str]]:
    """Return (act on it, reason-if-refused).

    A command already handled (same or older sequence number) returns
    (False, None): it is a repeat, not an error.
    """
    if command == CMD_NONE or sequence <= last_sequence:
        return False, None
    if age_s > COMMAND_TIMEOUT_S or age_s < 0:
        return False, f'mission command is stale ({age_s:.2f} s)'
    if command == CMD_START:
        allowed = START_STATES
    elif command == CMD_RETURN_HOME:
        allowed = RETURN_STATES
    elif command == CMD_LAND_NOW:
        allowed = LAND_STATES
    else:
        return False, f'unknown mission command {command}'
    if state not in allowed:
        return False, (f'{COMMAND_NAMES[command]} is not allowed in state '
                       f'{state}')
    return True, None
