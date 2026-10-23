# Mission-level command from the mission manager to flight_controller, the
# only node that commands PX4 (Phase 10). The controller validates it; a
# command is acted on once per sequence number.

uint8 CMD_NONE = 0
uint8 CMD_START = 1                # begin the flight (controller waiting on the ground)
uint8 CMD_RETURN_HOME = 2          # end the mission: fly to the takeoff point and land
uint8 CMD_LAND_NOW = 3             # land at the present position

builtin_interfaces/Time stamp
uint8 command
uint32 sequence                    # increases with every new command
string reason
