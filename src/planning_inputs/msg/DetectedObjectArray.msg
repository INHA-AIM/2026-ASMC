std_msgs/Header header
# Preserve the producer epoch and scan identity across the private boundary.
# Header remains the source scan in map; receipt time is consumer-local steady
# time. Rebuild all planning consumers after this private message change.
string producer_session_id
uint64 generation
uint64 scan_sequence
string config_id
string calibration_id
planning_inputs/DetectedObject[] objects
