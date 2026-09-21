# Competition mission metadata. STATUS_UNSPECIFIED is invalid for publication;
# consumers must never promote STATUS_PROVISIONAL into a mandatory rule.

uint8 STATUS_UNSPECIFIED=0
uint8 STATUS_OFFICIAL=1
uint8 STATUS_DERIVED_OFFICIAL=2
uint8 STATUS_PROVISIONAL=3

uint8 TYPE_UNKNOWN=0
uint8 TYPE_START_SUCCESS=1
uint8 TYPE_SPEED_LIMIT=2
uint8 TYPE_HIGH_SPEED_EXCEPTION=3
uint8 TYPE_GPS_SHADOW=4
uint8 TYPE_STATIC_OBSTACLE=5
uint8 TYPE_DYNAMIC_OBSTACLE=6
uint8 TYPE_ROUNDABOUT=7
uint8 TYPE_HIGHWAY_MERGE=8
uint8 TYPE_SURROUNDING_VEHICLE=9
uint8 TYPE_TRAFFIC_LIGHT=10

string id
uint8 mission_type
uint8 status
string source_reference
string description

float64 route_station_start_m
float64 route_station_end_m
string start_link_id
string end_link_id
string[] link_ids
string[] checkpoint_ids

# Optional map-frame footprint. An empty polygon means station/link bounds are
# authoritative. As elsewhere, a non-empty ring is open and has >=3 points.
geometry_msgs/Point[] polygon

bool has_speed_limit
float32 speed_limit_kph

# Canonical UTF-8 JSON object for mission-specific fields. Empty means none.
string rule_payload_json
