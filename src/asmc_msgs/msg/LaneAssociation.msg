# Stateful association of the vehicle reference point with one traversable HD-map lane.
# The reference point is the base_link origin (rear-axle center in this stack),
# expressed in the map frame.  A nearby result is intentionally distinct from
# an inside result so planning never mistakes a small lane departure for valid
# in-lane occupancy.

uint8 STATE_UNKNOWN=0
uint8 STATE_INSIDE=1
uint8 STATE_BOUNDARY=2
uint8 STATE_NEARBY=3
uint8 STATE_STALE=4

uint8 REFERENCE_UNKNOWN=0
uint8 REFERENCE_BASE_LINK_REAR_AXLE=1

std_msgs/Header header
string build_id
string map_id
string geometry_sha256

bool valid
uint8 state
uint8 reference_type
geometry_msgs/Point reference_point

string lane_id
string link_id
uint16 lane_number
string lane_number_scope_id
float32 centerline_distance_m
float32 signed_lateral_offset_m
float32 polygon_clearance_m
float32 heading_error_rad
float32 confidence

string previous_lane_id
bool transition_pending
string transition_candidate_lane_id
uint8 transition_candidate_samples
