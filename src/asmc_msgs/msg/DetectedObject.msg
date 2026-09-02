int32 id
uint8 type

geometry_msgs/Point center
float64 yaw
float64 vx
float64 vy
float64 yaw_rate

float64 width
float64 length
float64 height

# Perception-provided authoritative OBB geometry in the array header frame.
geometry_msgs/Point[8] obb_corners
