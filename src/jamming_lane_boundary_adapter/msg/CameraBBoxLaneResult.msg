# Observation only: NO_DETECTION does not imply a lane is safe to enter.
uint8 UNKNOWN=0
uint8 OWN=1
uint8 ADJACENT=2
uint8 BOTH=3
uint8 OUTSIDE=4
# Beyond both observed lane ends in the image; lane remains UNOBSERVED, not clear.
uint8 DEFERRED_FAR=5
uint8 UNOBSERVED=0
uint8 NO_DETECTION=1
uint8 DETECTED=2
std_msgs/Header header
time lane_stamp
bool valid
string reason
int8 own_lane
uint8 own_state
uint8 adjacent_state
CameraBBox[] boxes
uint8[] assignments
string[] box_reasons
# True only for a short, uniquely matched continuation of an already classified box.
# The current bottom contact is clipped; consumers must limit motion accordingly.
bool[] clipped_association

# Experimental local clearance evidence. These are image-ground CONTACT bands,
# not measured 3-D obstacle footprints. At the same image stamp and selected IMU
# plane; no GPS, latest-plane substitution, or obstacle height reconstruction.
# Parallel to boxes. Invalid/clipped contacts must never imply free space.
bool contact_projection_valid
float64 contact_support_x_max_m
geometry_msgs/Polygon[] ground_contacts
bool[] ground_contact_valid

# Opt-in local geometry support. No map pose/absolute speed is required.
# A LiDAR-supported contact has observed scan geometry (<=50ms image skew),
# not an image-plane depth claim. Original scan identity stays distinct.
string[] ground_contact_sources
bool local_obstacles_valid
time local_obstacles_stamp
string local_obstacles_session
uint64 local_obstacles_generation
uint64 local_obstacles_sequence
geometry_msgs/Polygon[] local_obstacle_footprints
