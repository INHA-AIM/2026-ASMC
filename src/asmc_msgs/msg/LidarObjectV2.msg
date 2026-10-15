# LidarObject contract v2: lossless temporal motion state (additive type).
# asmc_msgs/LidarObject stays frozen. Every LidarObject constant and field is
# repeated below with the same type, name and order; v2 appends the MOTION_*
# constants and the motion_valid/motion_state fields.
uint8 OBSERVED=1
uint8 PREDICTED=2
# Temporal motion state of the track, independent of classification. Values
# equal Main MotionState and planning_inputs/DetectedObject MOTION_*.
uint8 MOTION_UNKNOWN=0
uint8 MOTION_LOW_THREAT=1
uint8 MOTION_DYNAMIC_PENDING=2
uint8 MOTION_DYNAMIC=3
int32 track_id
uint32 detection_index
uint8 observation_state
uint32 age
uint32 miss_count
string classification
string camera_class
geometry_msgs/Point center
geometry_msgs/Point[8] obb_corners
float64 yaw
bool yaw_rate_valid
float64 yaw_rate_rad_s
float64 length
float64 width
float64 height
bool relative_velocity_valid
geometry_msgs/Vector3 relative_velocity_mps
bool absolute_velocity_valid
geometry_msgs/Vector3 absolute_velocity_mps
# Derived v1 compatibility copies; consumers branch on motion_state instead.
#   dynamic_valid == motion_valid
#   is_dynamic    == (motion_state == MOTION_DYNAMIC)
# is_dynamic=false does not mean static: UNKNOWN and DYNAMIC_PENDING are false
# too. Legacy Msg/TrackedObject.is_dynamic folds DYNAMIC_PENDING in; v2 does not.
bool dynamic_valid
bool is_dynamic
uint32 source_point_count
# motion_valid == (motion_state != MOTION_UNKNOWN). Warm-up, map-history
# invalidation (GPS denial/recovery) and any unrepresentable producer state
# are motion_valid=false with MOTION_UNKNOWN, never a valid LOW_THREAT.
bool motion_valid
uint8 motion_state
