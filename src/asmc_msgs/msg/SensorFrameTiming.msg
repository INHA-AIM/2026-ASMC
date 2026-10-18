# Additive timing sidecar. Existing sensor Header/MD5 and clock policy are unchanged.
# Join to the sensor using topic + frame_id + stamp + sensor_wire_sequence, never stamp alone.
# bridge_session_id + stream_id + frame_sequence identifies this record across restarts.
string bridge_session_id
string stream_id
uint64 frame_sequence
string sensor_topic
string sensor_frame_id
time published_stamp
uint32 sensor_wire_sequence
bool sensor_wire_sequence_valid

# MORAI camera/IMU: raw sec/nsec, clock domain morai_sensor (exposure/clock offset unverified).
# Velodyne: raw first/last microseconds since device hour; NOT Unix or point deskew time.
string source_clock_domain
string source_reference
uint64 source_sec
uint32 source_nsec
uint64 source_device_first_us
uint64 source_device_last_us
bool source_valid
bool source_duplicate
bool source_conflict
bool source_regression
bool device_hour_rollover
# Repeated packet ticks within a LiDAR sweep need not mean duplicate clouds.
uint32 device_tick_duplicates

# Unix kernel software receive time, or explicitly counted user-space fallback.
# Only monotonic fields may be subtracted for in-process durations.
uint64 rx_first_unix_ns
uint64 rx_last_unix_ns
uint64 rx_first_monotonic_ns
uint64 rx_last_monotonic_ns
uint32 rx_userspace_fallback_packets
uint32 ancillary_truncated_packets
uint32 packet_count
bool assembly_timeout
time publish_ros_time
uint64 publish_monotonic_ns
bool use_sim_time
string header_timestamp_policy
string header_fallback_reason
