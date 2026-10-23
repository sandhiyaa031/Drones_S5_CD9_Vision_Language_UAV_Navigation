# State of the mission manager (Phase 10), 2 Hz.

uint8 PHASE_INIT = 0
uint8 PHASE_PREFLIGHT = 1          # checking that every required function is alive
uint8 PHASE_STARTING = 2           # START sent, waiting for the controller
uint8 PHASE_TAKEOFF = 3
uint8 PHASE_SEARCH = 4
uint8 PHASE_RETURN = 5
uint8 PHASE_LANDING = 6
uint8 PHASE_COMPLETE = 7           # landed; report written
uint8 PHASE_REFUSED = 8            # not started: a preflight check failed
uint8 PHASE_FAILSAFE = 9           # the flight controller entered FAILSAFE

builtin_interfaces/Time stamp
string mission_name
uint8 phase
string phase_name
string flight_state                # flight_controller state
float32 elapsed                    # [s] since START
float32 search_elapsed             # [s] in the SEARCH phase
float32 time_budget                # [s] allowed for the search
float32 coverage_fraction          # from the search planner
uint32 candidates                  # located entries that qualify for the report
uint32 confirmed                   # of those, confirmed by the VLM
bool vlm_available
float32 battery_remaining          # 0..1, -1 = unknown
uint32 avoidance_manoeuvres
string end_reason                  # why the search ended (empty while searching)
string[] failed_checks             # preflight checks that are not satisfied
