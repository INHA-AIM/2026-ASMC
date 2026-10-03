# Exact source image and detections. valid=false is not an empty detection.
std_msgs/Header header
bool valid
string reason
sensor_msgs/CompressedImage image
CameraBBox[] boxes
