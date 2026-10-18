# Small lineage sidecar, published after the two compact images, before native debug outputs.
# header is the original image stamp/frame; its own sequence is NOT an image sequence.
std_msgs/Header header
string producer_session_id
uint64 generation
string view
std_msgs/Header image_header
std_msgs/Header class_header
std_msgs/Header confidence_header
bool grid_wire_sequences_valid
bool source_timing_valid
string reason
SensorFrameTiming sensor_timing
