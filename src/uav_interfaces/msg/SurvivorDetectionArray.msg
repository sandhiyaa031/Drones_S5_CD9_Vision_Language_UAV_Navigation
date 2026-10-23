# All survivor candidates of one camera frame.

uint8 VLM_AVAILABLE = 0
uint8 VLM_UNAVAILABLE = 1            # no endpoint configured or reachable
uint8 VLM_ERROR = 2                  # endpoint reachable, last request failed

std_msgs/Header header               # stamp = camera frame time; frame = camera frame
uint8 state                          # most significant SurvivorDetection state present (STATE_NO_TARGET if none)
uint8 vlm_status
string vlm_model
float32 processing_ms                # time spent in the camera callback for this frame
SurvivorDetection[] detections
