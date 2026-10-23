# Collision-risk estimate for one debris track. Advisory only.
#
# Separations are distances from the UAV centre to the tracked debris point
# (centroid of the object's visible surface), in metres, PX4 local NED.

uint8 STATE_SAFE = 0              # nominal path stays outside the safety volume, even with uncertainty
uint8 STATE_WATCH = 1             # only the uncertainty region reaches the safety volume
uint8 STATE_WARNING = 2           # predicted path enters the safety volume
uint8 STATE_CRITICAL = 3          # predicted contact, or entry sooner than the reaction time
uint8 STATE_POTENTIAL_THREAT = 4  # unconfirmed (tentative) fast object whose path enters the safety volume
uint8 STATE_UNKNOWN = 5           # UAV state unavailable at this timestamp

uint32 track_id
builtin_interfaces/Time stamp          # time of the track state used
uint8 track_status                     # DebrisTrack status
uint8 risk_state
float32 risk_score                     # 0..1, see docs/debris_risk.md

float32 time_to_closest_approach       # [s] from stamp, within the horizon
float32 minimum_predicted_separation   # [m] sampled along the predicted path
float32 relative_speed                 # [m/s] at closest approach
geometry_msgs/Point closest_point      # debris position at closest approach
geometry_msgs/Point uav_closest_point  # UAV position at closest approach
float32 current_separation             # [m]
float32[] horizons                     # [s]
float32[] separation_at_horizons       # [m]

float32 contact_radius                 # [m] UAV radius + debris radius
float32 safety_radius                  # [m] contact radius + margin
float32 uncertainty_margin             # [m] sigma_scale * 1-sigma of the prediction along the separation
float32 conservative_separation        # [m] minimum_predicted_separation - uncertainty_margin
bool safety_volume_intersected         # minimum_predicted_separation < safety_radius
bool predicted_collision               # minimum_predicted_separation < contact_radius
float32 prediction_horizon             # [s] furthest time evaluated

# constant-relative-velocity closed form, for comparison only
float32 linear_time_to_closest_approach
float32 linear_minimum_separation
