uint8 OBSERVED=1
uint8 PREDICTED=2
int32 track_id
uint32 detection_index
uint8 observation_state
uint32 age
uint32 miss_count
string classification
string camera_class
# OBB center, not the Kalman cluster centroid; coordinates follow the array header.
geometry_msgs/Point center
# All eight transformed corners are authoritative under roll/pitch.
geometry_msgs/Point[8] obb_corners
# XY projection of the long axis, not a full 3-D orientation.
float64 yaw
bool yaw_rate_valid
float64 yaw_rate_rad_s
float64 length
float64 width
float64 height
# Invalid velocity/dynamic flags must not be interpreted as measured zero/static.
bool relative_velocity_valid
geometry_msgs/Vector3 relative_velocity_mps
bool absolute_velocity_valid
geometry_msgs/Vector3 absolute_velocity_mps
bool dynamic_valid
bool is_dynamic
# Support count after voxel/filter/cluster stages, not the raw scan point count.
uint32 source_point_count
