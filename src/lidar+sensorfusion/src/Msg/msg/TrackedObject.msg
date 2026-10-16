int32 id
int16 type
# classification: current mission16 group (vehicle / dynamic_obstacle / static_obstacle / unknown).
# yolo_class: the raw camera model name (16 mission classes or 7 pretrained classes),
#             or "unknown" when no camera detection currently backs this track.
string classification
string yolo_class
# Filtered motion decision (including pending safety state), independent of
# classification mobility group. Do not infer velocity validity from this flag.
bool is_dynamic
int32 age
int32 miss_count

# Both centers and all OBB corners are expressed in TrackedObjectArray.header.frame_id.
geometry_msgs/Point cluster_center
geometry_msgs/Point obb_center
geometry_msgs/Point[8] obb_corners
float64 obb_yaw

# Absolute velocity in TrackedObjectArray.header.frame_id. SI is authoritative;
# speed_kph is provided only for display and validation convenience.
geometry_msgs/Vector3 velocity_mps
float64 speed_kph

float64 length
float64 width
float64 height
