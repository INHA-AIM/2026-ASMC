# HD Map artifact identity. header.frame_id is the geometry frame (normally map)
# and header.stamp is the publication time. generated_at is artifact creation time.

uint8 SCOPE_UNKNOWN=0
uint8 SCOPE_GLOBAL=1
uint8 SCOPE_LOCAL_CROP=2

std_msgs/Header header
uint16 schema_major
uint16 schema_minor
string contract_id
string build_id
string map_id
string geometry_sha256
string manifest_sha256
time generated_at
uint8 scope
bool operational
bool complete
string coordinate_system

# These arrays are parallel and must have equal length. Each source ID names
# the dataset whose byte-level SHA-256 appears at the same index.
string[] source_ids
string[] source_sha256

uint32 lane_marking_count
uint32 lane_count
uint32 drivable_area_count
uint32 checkpoint_count
uint32 mission_zone_count
