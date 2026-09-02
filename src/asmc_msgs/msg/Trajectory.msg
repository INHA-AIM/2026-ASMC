std_msgs/Header header

uint8 FAILURE_NONE=0
uint8 FAILURE_COLLISION_BLOCKED=1
uint8 FAILURE_GENERATION=2
uint8 FAILURE_STATE_STALE=3
uint8 FAILURE_BEHAVIOR_STALE=4
uint8 FAILURE_OBJECTS_STALE=5
uint8 FAILURE_INVALID_INPUT=6
uint8 FAILURE_UNSUPPORTED_BEHAVIOR=7
uint8 FAILURE_NOT_INITIALIZED=8

bool valid
uint8 failure_reason
float64 dt

asmc_msgs/VehicleState ego_at_plan
asmc_msgs/TrajectoryPoint[] points
