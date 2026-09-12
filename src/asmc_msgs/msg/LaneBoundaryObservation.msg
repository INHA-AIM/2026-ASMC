# One observed longitudinal road boundary in the array header frame.
# This message deliberately carries no home/opposing or numbered-lane meaning.

uint8 COLOR_UNKNOWN=0
uint8 COLOR_WHITE=1
uint8 COLOR_YELLOW=2

uint8 SOURCE_FRONT=1
uint8 SOURCE_LEFT=2
uint8 SOURCE_RIGHT=4

uint32 tracking_id

# Zero-based physical order from +y (left) to -y (right) at reference_x_m.
# Consumers must ignore lateral_order/reference_y_m when order_valid is false.
uint16 lateral_order
bool order_valid

uint8 color
uint8 source_mask

# semantic_score is aggregated model evidence. geometric_quality is a
# deterministic geometry/continuity heuristic. Neither is a calibrated
# probability.
float32 semantic_score
float32 geometric_quality

float32 reference_x_m
float32 reference_y_m
float32 valid_x_min_m
float32 valid_x_max_m
float32 observed_length_m
float32 bridged_length_ratio

# IDs are local to the publisher process and may reset after a source-time
# epoch change, a long gap, or node restart.
uint32 age_frames
uint32 reacquired_after_missed_frames

# Points are ordered by increasing x and are not extrapolated beyond the
# observed/limited-bridge valid range.
geometry_msgs/Point32[] polyline
