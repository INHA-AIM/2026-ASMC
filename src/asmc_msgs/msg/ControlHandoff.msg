# Opt-in localization handoff. Existing control/message contracts are unchanged.
# request: valid + epoch + target_mode. candidate: same epoch plus fresh evidence.
std_msgs/Header header
uint64 epoch
uint8 target_mode
bool valid
string reason
time state_stamp
time trajectory_stamp
uint64 trajectory_sequence
string state_frame
uint8 longl_cmd_type
float64 steering
float64 accel
float64 brake
