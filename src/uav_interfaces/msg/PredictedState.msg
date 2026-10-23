# Predicted state of one debris track at one horizon.
float32 horizon                    # [s] ahead of the track state time
builtin_interfaces/Time stamp      # time this prediction refers to
geometry_msgs/Point position       # [m] frame of the enclosing array header
geometry_msgs/Vector3 velocity     # [m/s]
float64[36] covariance             # 6x6 row-major, order px py pz vx vy vz
bool below_takeoff_plane           # predicted point is lower than the takeoff point
