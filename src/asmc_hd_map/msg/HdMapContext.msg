uint8 POSE_WAITING=0
uint8 POSE_VALID=1
uint8 POSE_GPS_UNAVAILABLE=2
uint8 POSE_STALE=3
uint8 POSE_INVALID=4
uint8 REFERENCE_BASE_LINK_REAR_AXLE=1
# Publication/health timestamp; pose_stamp remains the source observation time.
std_msgs/Header header
string build_id
string map_id
string route_id
string route_sha256
time pose_stamp
uint8 pose_status
uint8 reference_type
bool pose_valid
geometry_msgs/Pose ego_pose
bool lane_valid
string lane_id
string link_id
uint16 lane_number
bool route_valid
float64 route_station_m
float64 route_distance_m
# Effective limit of the projected official route link, in SI units.
bool has_route_speed_limit
float64 route_speed_limit_mps
string[] active_mission_ids
asmc_hd_map/MissionBoundary[] boundaries
# Historical only: never an extrapolated current position or route station.
bool last_valid_context_available
time last_valid_pose_stamp
float64 last_valid_route_station_m
string[] last_valid_mission_ids
# Expected/confirmed shadow zones at last valid pose, retained during outage.
# GPS loss alone does not prove that ego is inside a configured shadow zone.
string[] gps_shadow_zone_ids
bool gps_shadow_context_from_last_valid_pose
