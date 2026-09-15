# One role in a jointly fitted metric lane-boundary family.
# The role names describe geometric order in the shadow-road ODD only. They do
# not assign home/opposing semantics or an absolute lane number.
# Consumers should combine adjacent roles into candidate strips, then use
# route/MGeo context to assign driving meaning. Yellow/white alone is not a
# lane-change or avoidance command.

uint8 ROLE_UNKNOWN=0
uint8 ROLE_LEFT_OUTER_BOUNDARY=1
uint8 ROLE_ROAD_CENTRE_BOUNDARY=2
uint8 ROLE_RIGHT_OUTER_BOUNDARY=3

uint8 COLOR_UNKNOWN=0
uint8 COLOR_WHITE=1
uint8 COLOR_YELLOW=2

uint8 SOURCE_FRONT=1
uint8 SOURCE_LEFT=2
uint8 SOURCE_RIGHT=4

# OBSERVED and STRUCTURE_INFERRED are usable candidate geometry. A default
# consumer must split the polyline at INVALID samples and must never plan
# through an INVALID segment. INVALID coordinates remain in the message for
# audit and an explicitly opted-in low-authority preview/fallback experiment;
# they are not perception evidence and must never be relabelled as observed.
# STRUCTURE_INFERRED is deliberately lower-trust geometry inferred from the
# accepted same-frame parallel family, not temporal prediction. The current
# shadow-road producer may retain this state across a configured bounded
# completion window even where local multi-role support is momentarily absent;
# it never changes such a sample to OBSERVED.
uint8 SUPPORT_INVALID=0
uint8 SUPPORT_OBSERVED=1
uint8 SUPPORT_STRUCTURE_INFERRED=2

uint8 role
uint8 color
uint8 source_mask
uint8 overall_state

# confidence is aggregate semantic evidence, not a calibrated probability.
# An entirely inferred role is already down-weighted by the producer.
float32 confidence

bool residual_valid
float32 residual_p50_m
float32 residual_p95_m

float32 observed_length_m
float32 inferred_length_m
float32 invalid_length_m
float32 max_inferred_gap_m
float32 bridged_length_ratio

# Samples follow the shared family arc-length order. support_state has exactly
# the same length as polyline and determines which adjacent intervals are safe
# to expose to a downstream candidate adapter.
geometry_msgs/Point32[] polyline
uint8[] support_state
