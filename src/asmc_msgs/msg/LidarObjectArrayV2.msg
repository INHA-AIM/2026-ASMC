# LidarObjectArray contract v2. Same constants and header/status/reason/
# stamp_provenance semantics as asmc_msgs/LidarObjectArray (frozen); only the
# element type changes to LidarObjectV2.
uint8 OBSERVED=1
uint8 OBSERVED_EMPTY=2
uint8 INPUT_INVALID=3
uint8 POSE_UNAVAILABLE=4
std_msgs/Header header
uint8 status
string reason
string stamp_provenance
asmc_msgs/LidarObjectV2[] objects
