uint8 STATE_UNKNOWN=0
uint8 STATE_NOT_DETECTED=1
uint8 STATE_RED=2
uint8 STATE_YELLOW=3
uint8 STATE_GREEN=4
uint8 STATE_RED_YELLOW=5
uint8 STATE_RED_LEFT=6
uint8 STATE_GREEN_LEFT=7
uint8 STATE_LEFT=8

# The source camera header.  stamp is the image acquisition/bridge receive
# time propagated by the producer, not the inference completion time.
std_msgs/Header header

uint8 state
string label
float32 confidence

# valid=false means the result must not activate a planning rule.  A stale
# result is always invalid, but is carried explicitly for diagnostics.
bool valid
bool stale

float32 inference_latency_ms
string model_id

# Parallel arrays ordered from highest to lowest confidence.
string[] candidate_labels
float32[] candidate_confidences
