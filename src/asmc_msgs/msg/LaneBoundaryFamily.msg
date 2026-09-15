std_msgs/Header header

# valid means the shared fit was accepted for this synchronized camera cycle.
# A fresh valid=false message is different from a missing or stale message.
bool valid

# This contract is frozen for the shadow-road planning handoff, but authority
# is not: planning owns ODD activation, route/lane semantics, fallback, and the
# closed-loop promotion gate. Perception never connects this message to control.
bool deployment_allowed
uint8 expected_boundary_count

string operating_domain
string algorithm_id
string representation
string config_sha256
string status_reason

float32 valid_x_min_m
float32 valid_x_max_m
float32 left_width_m
float32 right_width_m

float32 data_residual_p50_m
float32 data_residual_p95_m
float32 inlier_residual_p50_m
float32 inlier_residual_p95_m
float32 outlier_rate
float32 weighted_rms_m

asmc_msgs/LaneBoundaryFamilyMember[] boundaries
