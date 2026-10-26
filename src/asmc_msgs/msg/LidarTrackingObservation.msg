# Atomic geometry, estimated velocity, motion and producer identity for one scan.
# Existing LidarObservation stays byte-compatible for geometry consumers/bags.
# observation.objects is authoritative, including status/header/velocity flags.
# tracking has exactly one entry per object, in the same order and with the
# same track_id. Invalid/observed-empty arrays have no tracking entries.
asmc_msgs/LidarObservation observation
asmc_msgs/LidarTrackingState[] tracking
