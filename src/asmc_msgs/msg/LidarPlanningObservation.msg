# Additive planning boundary; existing LidarObservation/LidarObject MD5 unchanged.
# Geometry is the delivered map array, with explicitly qualified velocities.
uint8 UNKNOWN=0
uint8 LOW_THREAT=1
uint8 DYNAMIC_PENDING=2
uint8 DYNAMIC=3
asmc_msgs/LidarObservation observation
# Same order/length as observation.objects.objects. Never joined by arrival time.
uint8[] motion_states
