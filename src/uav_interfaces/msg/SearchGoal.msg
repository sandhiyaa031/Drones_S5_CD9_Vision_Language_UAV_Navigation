# High-level search intent from the search planner to flight_controller,
# the only node that commands PX4 (Phase 9). The controller validates every
# goal and may refuse it; a refused or missing goal leaves the previous
# mission setpoint in force.

uint8 STATE_IDLE = 0               # not searching (vehicle not in the SEARCH state)
uint8 STATE_SEARCHING = 1          # following the coverage path
uint8 STATE_INSPECTING = 2         # detour to view a located candidate from the side
uint8 STATE_COMPLETE = 3           # coverage path finished: return and land

std_msgs/Header header             # stamp = PX4 time of the pose used; frame = px4_local_ned
uint8 state
geometry_msgs/Point target         # [m] horizontal goal, PX4 local NED (z is not used: the controller keeps its altitude)
float32 speed_limit                # [m/s] requested horizontal speed cap (> 0)
bool complete                      # true: the search is finished
uint32 waypoint_index              # next coverage waypoint
uint32 waypoint_count
float32 coverage_fraction          # 0..1 of the search area seen so far (planner's own estimate)
uint32 inspections_done
uint32 inspecting_id               # survivor_id being inspected (0 = none)
string reason
