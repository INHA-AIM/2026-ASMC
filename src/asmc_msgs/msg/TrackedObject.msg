int32 id
int16 type

# classification is the fused planning class.  yolo_class preserves the raw
# camera class that confirmed the track, or "unknown" when none did.
string classification
string yolo_class
bool is_dynamic
int32 age
int32 miss_count

# Centers and OBB corners use TrackedObjectArray.header.frame_id.
geometry_msgs/Point cluster_center
geometry_msgs/Point obb_center
geometry_msgs/Point[8] obb_corners
float64 obb_yaw

# Absolute velocity in the array header frame.  SI is authoritative;
# speed_kph is display/validation convenience only.
geometry_msgs/Vector3 velocity_mps
float64 speed_kph

float64 length
float64 width
float64 height
