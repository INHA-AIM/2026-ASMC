std_msgs/Header header

uint8 COMPLETENESS_EMPTY=0
uint8 COMPLETENESS_PARTIAL=1
uint8 COMPLETENESS_COMPLETE=2

# valid means the synchronized perception cycle completed. A fresh valid empty
# array is different from a missing or stale message.
bool valid
uint8 completeness
uint8 expected_boundary_count

# The first implementation publishes operating_domain=shadow_two_way_v1.
# Consumers must not use that output outside the matching route/ODD state.
string operating_domain
string algorithm_id
string config_sha256

asmc_msgs/LaneBoundaryObservation[] boundaries
