# One survivor candidate seen by the downward camera (image coordinates only).
#
# Two independent sources are kept apart:
#   candidate_*  from the deterministic image detector (every frame)
#   vlm_*        from the vision-language model, asynchronous, on a crop
# No 3D position is given here; localisation is a later phase.

# semantic_state
uint8 STATE_NO_TARGET = 0            # (array level only)
uint8 STATE_CANDIDATE = 1            # stable, person-like cue set; not verified
uint8 STATE_SURVIVOR_CONFIRMED = 2   # stable and confirmed by the VLM
uint8 STATE_UNCERTAIN = 3            # weak or partial cue set, or the VLM was unsure
uint8 STATE_REJECTED = 4             # the VLM judged it not a survivor

# verification_status
uint8 VERIFY_NOT_REQUESTED = 0
uint8 VERIFY_PENDING = 1             # a crop is with the VLM
uint8 VERIFY_VLM_CONFIRMED = 2
uint8 VERIFY_VLM_REJECTED = 3
uint8 VERIFY_VLM_UNCERTAIN = 4
uint8 VERIFY_VLM_UNAVAILABLE = 5     # no endpoint reachable: never "confirmed"
uint8 VERIFY_VLM_ERROR = 6           # timeout, HTTP error or malformed answer

# vlm_visibility
uint8 VISIBILITY_UNKNOWN = 0
uint8 VISIBILITY_CLEAR = 1
uint8 VISIBILITY_PARTIAL = 2
uint8 VISIBILITY_POOR = 3

std_msgs/Header header               # stamp = camera frame time; frame = camera frame
uint32 track_id                      # stable while the candidate is followed

# Bounding box and centre in pixels of the full camera image
int32 bbox_x                         # left
int32 bbox_y                         # top
int32 bbox_width
int32 bbox_height
float32 center_u
float32 center_v
uint32 image_width
uint32 image_height

# Deterministic candidate detector
float32 candidate_confidence         # 0..1, cue score of this frame (docs/survivor_perception.md)
bool head_cue                        # skin-coloured head region found with the clothing region
bool legs_cue
bool touches_border                  # box cut by the image edge
uint32 hits                          # frames in which this candidate was detected
float32 age                          # [s] since first detection (camera time)
bool measured                        # detected in this frame (false: held from an earlier one)

# Meaning
uint8 semantic_state
uint8 verification_status

# VLM verification (valid when verification_status is CONFIRMED, REJECTED or UNCERTAIN)
float32 vlm_confidence               # 0..1, the model's confidence in its decision; -1 when there is no VLM answer
uint8 vlm_visibility
string vlm_label                     # the model's label for the crop
builtin_interfaces/Time vlm_stamp    # camera time of the frame the verified crop came from
float32 vlm_latency                  # [s] wall time of the request
uint32 vlm_requests                  # crops of this candidate sent so far

string source                        # detector, and model if a VLM answer is included
