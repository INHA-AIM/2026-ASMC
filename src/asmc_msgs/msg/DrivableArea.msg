# Vector drivable-area polygon. The raster representation is published
# separately as nav_msgs/OccupancyGrid.

uint8 TYPE_UNKNOWN=0
uint8 TYPE_LANE=1
uint8 TYPE_JUNCTION=2
uint8 TYPE_MERGE=3
uint8 TYPE_DIVERGE=4
uint8 TYPE_UNMARKED_ROAD=5

uint8 VALIDITY_UNSPECIFIED=0
uint8 VALIDITY_VALIDATED=1
uint8 VALIDITY_PROVISIONAL=2

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_ROAD_MESH=1
uint8 SOURCE_LANE_CLIPPED=2
uint8 SOURCE_HYBRID=3

string id
uint8 area_type
uint8 validity
uint8 source_type

# Open polygon ring: at least three ordered points; first point is not repeated.
geometry_msgs/Point[] polygon
string[] lane_ids
string[] link_ids

# True when clipping was deliberately suppressed to preserve an intersection,
# merge/diverge transition, or an unmarked road section.
bool intersection_guarded
string source_dataset
string source_feature_id
