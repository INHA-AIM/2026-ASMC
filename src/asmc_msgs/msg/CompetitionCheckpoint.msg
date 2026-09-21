# Organizer checkpoint pose plus its projection onto the team route.

string id
uint16 sequence
bool is_start
bool is_finish

# Position is the organizer coordinate. Orientation is generated from
# official_yaw_deg around +Z using the map-frame convention.
geometry_msgs/Pose pose
float64 official_yaw_deg
float64 pass_radius_m

float64 route_station_m
string link_id
float64 link_station_m
string source_reference
