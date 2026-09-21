# A traversable lane and its topology. Lane marking paint is kept separately
# in LaneMarking so a marking can border more than one lane.

uint8 TYPE_UNKNOWN=0
uint8 TYPE_DRIVING=1
uint8 TYPE_MERGE=2
uint8 TYPE_DIVERGE=3
uint8 TYPE_INTERSECTION=4
uint8 TYPE_SHOULDER=5
uint8 TYPE_TOLL=6

uint8 DIRECTION_UNKNOWN=0
uint8 DIRECTION_FORWARD=1
uint8 DIRECTION_REVERSE=2
uint8 DIRECTION_BIDIRECTIONAL=3

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_SURVEY=1
uint8 SOURCE_LEGACY_MAP=2
uint8 SOURCE_PREPLAN=3
uint8 SOURCE_CURATED=4
uint8 SOURCE_DERIVED=5

string id
string link_id
uint8 lane_type
uint8 direction

# Lane numbers are 1-based, left-to-right within lane_number_scope_id for the
# route direction. Zero means unknown. The number is valid only on the stated
# route-station interval; -1.0 means the lane is not assigned to the route.
uint16 lane_number
string lane_number_scope_id
float64 route_station_start_m
float64 route_station_end_m

string left_boundary_id
string right_boundary_id
string left_adjacent_lane_id
string right_adjacent_lane_id
string[] predecessor_lane_ids
string[] successor_lane_ids

geometry_msgs/Point[] centerline

# Open polygon ring: at least three ordered points; do not repeat the first
# point at the end. The array message header supplies the coordinate frame.
geometry_msgs/Point[] polygon

uint8 source_type
string source_dataset
string source_feature_id
