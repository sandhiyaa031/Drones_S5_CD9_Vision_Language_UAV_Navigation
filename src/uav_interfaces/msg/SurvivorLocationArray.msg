# All located survivor candidates (Phase 8), published once per camera frame.

std_msgs/Header header               # stamp = camera frame time; frame = px4_local_ned
float32 ground_down                  # local z of the ground used for the height prior [m]
bool pose_available                  # the vehicle pose at the frame time was available
SurvivorLocation[] locations
