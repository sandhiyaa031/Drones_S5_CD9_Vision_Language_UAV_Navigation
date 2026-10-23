# Predictions for all currently valid debris tracks at one sensor timestamp.
# A track that is absent has no valid prediction (tentative, stale or dead).
std_msgs/Header header             # stamp = track state time; frame = px4_local_ned
DebrisPrediction[] predictions
