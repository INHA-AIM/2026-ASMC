# Per-object tracking information, paired by track_id with the observation.
# Former Main motion_debug states; a semantic camera group is not motion.
uint8 UNKNOWN=0
uint8 LOW_THREAT=1
uint8 DYNAMIC_PENDING=2
uint8 DYNAMIC=3
int32 track_id
uint8 motion_state
# Former Main cluster_center (Core after fusion), not the OBB center or a
# guaranteed Kalman centroid. Uses observation.objects.header.frame_id.
geometry_msgs/Point cluster_center
