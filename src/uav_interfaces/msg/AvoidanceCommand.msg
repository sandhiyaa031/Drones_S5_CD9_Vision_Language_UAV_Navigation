# High-level avoidance intent from the debris avoidance planner to
# flight_controller, the only node that commands PX4. The controller
# validates it and may refuse it.

uint8 STATE_NORMAL_FLIGHT = 0
uint8 STATE_PREPARE_AVOIDANCE = 1
uint8 STATE_AVOIDING = 2
uint8 STATE_CLEARING = 3
uint8 STATE_RESUME_MISSION = 4
uint8 STATE_AVOIDANCE_FAILSAFE = 5
uint8 STATE_DEFENSIVE = 6          # perception stale beyond the hold time

uint8 HEALTH_FRESH = 0
uint8 HEALTH_DEGRADED = 1
uint8 HEALTH_STALE = 2

std_msgs/Header header           # stamp = sensor time of the risk frame used; frame = px4_local_ned
uint8 state                      # planner state
bool active                      # true: fly to `target` instead of the mission setpoint
geometry_msgs/Point target       # [m] position setpoint, PX4 local NED
bool precautionary               # small manoeuvre taken on a POTENTIAL_THREAT
bool failsafe                    # no fully safe candidate existed, or inputs were stale
uint32[] threat_ids              # tracks that caused the manoeuvre
float32 predicted_min_separation # [m] smallest predicted clearance of the selected candidate
float32 cost                     # cost of the selected candidate
string candidate                 # name of the selected candidate
string reason
uint8 perception_health          # HEALTH_* from the depth-stream timing
float32 speed_limit              # [m/s] requested cap on mission motion, 0 = none
