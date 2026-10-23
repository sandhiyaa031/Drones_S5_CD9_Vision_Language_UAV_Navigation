# Collision-risk estimates for all debris tracks at one sensor timestamp.
std_msgs/Header header                 # stamp = track state time; frame = px4_local_ned
bool uav_state_valid
geometry_msgs/Point uav_position       # interpolated to header.stamp
geometry_msgs/Vector3 uav_velocity
uint8 highest_state                    # most severe DebrisRisk state present (confirmed tracks)
bool potential_threat_present
DebrisRisk[] risks
