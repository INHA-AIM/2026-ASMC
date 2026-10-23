# Atomic result of one LiDAR scan. objects.header is the sole source stamp/frame.
# A generation changes only on explicit processing-state reset; sequence gaps
# are allowed when pending scans are replaced. Never count local/map twice.
asmc_msgs/LidarObjectArray objects
string producer_session_id
uint64 generation
uint64 scan_sequence
string config_id
string calibration_id
