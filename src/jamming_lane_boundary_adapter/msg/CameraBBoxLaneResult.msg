# Observation only: NO_DETECTION does not imply a lane is safe to enter.
uint8 UNKNOWN=0
uint8 OWN=1
uint8 ADJACENT=2
uint8 BOTH=3
uint8 OUTSIDE=4
# Beyond both observed lane ends in the image; lane remains UNOBSERVED, not clear.
uint8 DEFERRED_FAR=5
uint8 UNOBSERVED=0
uint8 NO_DETECTION=1
uint8 DETECTED=2
std_msgs/Header header
time lane_stamp
bool valid
string reason
int8 own_lane
uint8 own_state
uint8 adjacent_state
CameraBBox[] boxes
uint8[] assignments
string[] box_reasons
# True only for a short, uniquely matched continuation of an already classified box.
# The current bottom contact is clipped; consumers must limit motion accordingly.
bool[] clipped_association
# Ground projection of the unclipped bottom edge in base_link [m]. NaN when not measured
# (UNKNOWN/OUTSIDE/DEFERRED_FAR/clipped). Flat-ground estimate of the visible contact only:
# the far side of an oblique object projects closer to the ego centreline (conservative).
float32[] contact_x
float32[] contact_y_min
float32[] contact_y_max
