# Predicted future trajectory of one debris track.
#
# Positions are of the same point the tracker follows: the centroid of the
# object's visible surface, not its geometric centre.

uint8 MODEL_CONSTANT_VELOCITY = 0
uint8 MODEL_CONSTANT_ACCELERATION = 1   # acceleration estimated from the track
uint8 MODEL_BALLISTIC = 2               # known gravity along local down

uint32 track_id                    # same ID as in DebrisTrackArray
builtin_interfaces/Time stamp      # time of the track state the prediction starts from
uint8 track_status                 # DebrisTrack status at that time
float32 time_since_update          # [s] age of the last measurement behind the state
uint8 model
geometry_msgs/Vector3 acceleration # [m/s^2] acceleration used by the model
PredictedState[] states            # one entry per horizon, increasing
