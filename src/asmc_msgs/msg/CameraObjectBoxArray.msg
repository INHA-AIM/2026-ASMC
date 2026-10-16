# Original camera header: frame_id identifies the view; stamp follows the bridge
# receive/source policy and is not guaranteed to be the sensor exposure time.
std_msgs/Header header
uint32 image_width
uint32 image_height
# true with no boxes is an observed empty result; false is invalidation/error.
# Missing/stale input must not be converted to a newly stamped empty result.
bool valid
CameraObjectBox[] boxes
