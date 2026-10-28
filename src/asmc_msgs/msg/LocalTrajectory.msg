# Experimental LOCAL-1; does not relax the map-only Trajectory contract.
# Frame is a producer-session/epoch-specific local odometry frame, never map.
# Point zero is at header.stamp; points use uniform dt and the same frame.
std_msgs/Header header
bool valid
string reason
string run_id
uint64 plan_sequence
# Original camera bundle time: freshness remains relative to this stamp,
# even after geometry is aligned to the later header (planning) time.
time scene_stamp
nav_msgs/Odometry ego_at_plan
float64 dt
asmc_msgs/TrajectoryPoint[] points
