int32 id
uint8 type

# Perception의 시간적 운동 판정. type(semantic classification)과 별개다.
# LOW_THREAT만 planner에서 정적에 준해 취급하며, UNKNOWN/PENDING/DYNAMIC의
# vx/vy는 손실 없이 보존한다.
uint8 MOTION_UNKNOWN=0
uint8 MOTION_LOW_THREAT=1
uint8 MOTION_DYNAMIC_PENDING=2
uint8 MOTION_DYNAMIC=3
uint8 motion_state

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
