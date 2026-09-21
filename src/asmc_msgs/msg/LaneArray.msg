std_msgs/Header header
string build_id
string map_id
string geometry_sha256

# false for local crops and other subsets. Lane IDs always refer to the global
# map namespace even when only nearby lanes are included.
bool complete
asmc_msgs/Lane[] lanes
