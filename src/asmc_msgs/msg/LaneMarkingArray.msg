std_msgs/Header header
string build_id
string map_id
string geometry_sha256

# true means this is the complete global set. Local crops must publish false.
bool complete
asmc_msgs/LaneMarking[] markings
