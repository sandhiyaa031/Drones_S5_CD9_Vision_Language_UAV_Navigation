# One survivor candidate located in 3D (Phase 8).
#
# Position of the detected point in the PX4 local NED frame, estimated from
# the downward camera's pixel detections, the camera geometry and the
# vehicle's own pose estimate. No simulator ground truth is used.

# localization_state
uint8 LOC_TENTATIVE = 0              # (not published)
uint8 LOC_COARSE = 1                 # on one viewing ray; height is the assumed value
uint8 LOC_LOCALIZED = 2              # horizontal 1-sigma below the threshold

std_msgs/Header header               # stamp = camera time of the last update; frame = px4_local_ned
uint32 survivor_id                   # stable for the whole flight
uint32 track_id                      # Phase 7 image track of the last update

geometry_msgs/Point position         # x north, y east, z down [m]
float32[9] covariance                # row-major 3x3 [m^2], same axes
float32 horizontal_sigma             # 1-sigma of the worst horizontal direction [m]
float32 height_above_ground          # of the detected point [m]; assumed until the vehicle has moved

uint8 localization_state
uint32 observations                  # camera frames used
float32 baseline                     # largest camera displacement between used frames [m]
float32 min_range                    # closest camera distance [m]
builtin_interfaces/Time first_seen
bool updated                         # used a detection of this frame

# Meaning, copied from Phase 7 (uav_interfaces/SurvivorDetection)
uint8 semantic_state                 # latest state of the image track
bool vlm_confirmed                   # the VLM confirmed this candidate at least once
bool vlm_rejected                    # the VLM rejected it at least once
