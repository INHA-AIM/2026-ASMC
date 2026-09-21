# One physical or logical road marking in map coordinates.

uint8 COLOR_UNKNOWN=0
uint8 COLOR_WHITE=1
uint8 COLOR_YELLOW=2

uint8 PATTERN_UNKNOWN=0
uint8 PATTERN_SOLID=1
uint8 PATTERN_BROKEN=2

uint8 PHYSICAL_UNKNOWN=0
uint8 PHYSICAL_PAINTED=1
uint8 PHYSICAL_RAISED=2
uint8 PHYSICAL_VIRTUAL=3

uint8 ROLE_UNKNOWN=0
uint8 ROLE_ROAD_CENTER=1
uint8 ROLE_LANE_DIVIDER=2
uint8 ROLE_ROAD_EDGE=3
uint8 ROLE_STOP_LINE=4
uint8 ROLE_KEEP_OUT=5
uint8 ROLE_ISLAND_EDGE=6

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_SURVEY=1
uint8 SOURCE_LEGACY_MAP=2
uint8 SOURCE_PREPLAN=3
uint8 SOURCE_CURATED=4
uint8 SOURCE_DERIVED=5

string id
uint8 color
uint8 pattern
uint8 physical_type
uint8 role
float32 width_m

# Ordered points. A stop line is represented by the same field with
# role=ROLE_STOP_LINE; consumers must not infer solid/broken from point gaps.
geometry_msgs/Point[] polyline

# IDs of Lane objects geometrically adjacent to each side. Either array may be
# empty at an outer road edge or before lane topology is available.
string[] left_lane_ids
string[] right_lane_ids

uint8 source_type
string source_dataset
string source_feature_id
