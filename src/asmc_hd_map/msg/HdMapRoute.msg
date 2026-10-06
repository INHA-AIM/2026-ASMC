# Immutable official global path. Stations are XY arc length from points[0].
# Do not equate these stations with a smoothed/modified planner reference.
std_msgs/Header header
string build_id
string map_id
string route_id
string route_sha256
string geometry_sha256
bool closed_loop
float64 length_m
float64[] local_origin_in_global
geometry_msgs/Point[] points
float64[] stations_m
