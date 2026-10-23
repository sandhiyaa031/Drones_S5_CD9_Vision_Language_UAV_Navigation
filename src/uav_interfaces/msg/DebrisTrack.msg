# One tracked debris object, estimated from upward depth detections.
#
# The state is expressed in the frame of the enclosing DebrisTrackArray
# header (PX4 local NED). The tracked point is the centroid of the object's
# VISIBLE SURFACE as reported by the detector, not its geometric centre.

uint8 STATUS_TENTATIVE = 0   # too few measurements to trust
uint8 STATUS_CONFIRMED = 1   # updated by a measurement in this frame
uint8 STATUS_COASTING = 2    # confirmed, predicted without a measurement

uint32 id                          # unique for the life of the tracker
builtin_interfaces/Time stamp      # time the state refers to (sensor time)
geometry_msgs/Point position       # [m]
geometry_msgs/Vector3 velocity     # [m/s]
float64[36] covariance             # 6x6 row-major, order px py pz vx vy vz
float32 age                        # [s] since the first measurement
float32 time_since_update          # [s] since the last associated measurement
uint32 hits                        # number of associated measurements
uint32 consecutive_misses          # frames in a row without a measurement
float32 size                       # [m] approximate size from the detector
float32 confidence                 # heuristic, 0..1
uint8 status
