# Committed merge response checks

These execution policies apply only after a highway/roundabout merge is committed.
New-entry approval, physical current-contact checks, ego road/paint/kinematic
limits, speed caps, sensor messages and jamming are unchanged.

## Highway: one response policy through execution and maintenance

The supplied `planner.highway_merge.max_rear_decel_mps2` remains 5.0. It is a
conditional NPC response assumption, not a measured braking capability or an
ego acceleration limit. The one-second allowance is attached to each prepared
traffic lane's observed physical ego entry. A new observation or track ID does
not restart it. Before actual overlap, the full allowance remains.

A wholly separated, same-direction rear actor can retain its role through the
prepared predecessor/common-successor seam. A new UNKNOWN fragment may support
a candidate-local response when its measured body lies uniquely in a prepared
lane and its velocity agrees with that lane's heading. This does not certify
its normal motion fit or alter any measured corners. Its short provisional
motion interval remains strict; crossing motion and current contact are not
exempt. Both speed bounds and original observation age remain in the response.
The candidate's actual schedule is checked instead of first requiring that a
constant-current-ego-speed scalar braking estimate already pass.

A curve's speed-interval hull is a conservative collision occupancy, not one
vehicle occupying the curve's empty chord. Its whole hull need not fit inside
a lane. At an unresolved successor, prediction retains only its incoming
tangent. A full rear-response certificate requires support in the prepared
traffic corridor. Missing future coverage is explicitly UNVERIFIED: it cannot
turn an observed rear follower into new observed crossing traffic, nor certify
rear safety. An already-entered forward schedule can still be considered for
mitigation after all other checks. No NPC successor route is invented. Ego's full swept road certificate
and authoritative object collision geometry are still checked.

If bounded rear response cannot certify any moving schedule, an already-entered
execution may retain an existing forward/FOLLOW schedule as **mitigation** only.
It must finish the complete front/side collision scan, road/paint and kinematic
checks. Current contact, the initial reaction interval, lateral cut-ins and
unsupported traffic corridors never qualify. Fully certified forward schedules
remain preferred; among mitigation alternatives prefer actual forward progress,
then the existing cost. No extra candidate catalogue or larger deadline is used.
A stationary candidate is not forward mitigation.

`highway_forward_mitigation; rear_response_unverified=<id>` identifies this
conditional-risk output. It publishes a moving locked trajectory with feedback
`plan_valid=false`; it cannot issue READY or complete reference handover. This
is not a claim of collision-free rear motion. The same execution owner replans
from every new observation. The controller receives a finite valid trajectory,
not the one-point generation-failure STOP. Front blockage can still require
braking, and no feasible/usable output can still reach the existing emergency
branch; that branch is not labelled a certified collision-free stop.

The braking generator accepts a caller-scoped safety certificate. Highway
maintenance reconsiders a blocking rear actor **before** rejecting the generated
path, using the same response policy. Other actors retain the ordinary
maintenance certificate; ordinary STOP/AVOID callers retain their original
checker. A mitigation outcome cannot certify a braking path. The former
post-`maintenance_ok` diagnostic-only check is removed.

Segment response sweeps are cached. A clear conditional response can accept
separation before constructing a second ordinary speed sweep; an overlapping
broad phase still receives the exact dense check. All successful spans retain
the original relative-motion sampling guard. Planning budget remains 30 ms.

## Roundabout: evaluate the response and resulting relationship together

Execution-only lane support can use partial observations when fresh positions
and measured direction support a supplied circulating lane. Recent successful
execution evidence survives a transient fragment-fit failure only within that
same bounded continuity track, while current direction, recent lane positions
and supplied coverage still agree. Reusing it never renews its original age.
Missing geometry/velocity, ID discontinuity, stale/expired history or crossing
direction abstains. Current corners and both old/current speed bounds remain.
PREPARE's whole-body/confidence requirements do not inherit this evidence.

For a failed committed path, determine the resulting following relationship
under the **same bounded response** checked for full path clearance. Requiring
the unreacted actor to be a follower before considering yielding was circular.
Current physical separation and the leading-actor guard remain strict. Forward,
progress-to-stop and service-braking schedules share this execution check.

`roundabout_merge.execution_reaction_time_s` remains 0.3 and
`execution_max_rear_decel_mps2` remains 3.0. Reaction is anchored to observed
physical entry; pre-entry schedules use their actual entry plus reaction.
No circular continuation beyond the supplied map is invented.

## Validation and deployment

Fixed recorded inputs exercise the C++ core; ideal ego-prefix rollouts against
recorded world objects are additional diagnostics, not MORAI closed-loop tests
or NPC ground truth. Saved trajectories and output failure flags are checked;
ROS transport, actual publication freshness, NPC response and controller
tracking require the simulator environment.

Internal planner layout changed. Rebuild the planner and its consumers together:

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4
source /root/ws/devel/setup.bash
```

Optional package regression run after updating code:

```bash
catkin_make run_tests_asmc_planner -j4 -l4
catkin_test_results /root/ws/build/test_results
```

Stop/restart the planner launch manually after the build:

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch highway_merge:=true
```

`highway_merge` still defaults to false. No ROS node is restarted automatically.
