# State of flight_controller, for nodes that propose high-level intent.
builtin_interfaces/Time stamp      # PX4 time of the position sample used
string state                       # controller state name
bool airborne_mission              # true in states where an avoidance command may be accepted
geometry_msgs/Point mission_target # [m] the mission's own position setpoint, PX4 local NED
geometry_msgs/Point commanded      # [m] the setpoint currently sent to PX4
bool avoidance_active              # an avoidance command is being executed
bool avoidance_recovering          # returning from an avoidance target to the mission setpoint
uint32 avoidance_accepted          # commands accepted so far (count of manoeuvres started)
uint32 avoidance_rejected          # commands refused by validation
string last_rejection              # why the last command was refused
float64 ground_z                 # local NED down coordinate of the takeoff point
bool ground_valid
float32 speed_limit              # [m/s] horizontal cap in force on the mission setpoint, 0 = none
