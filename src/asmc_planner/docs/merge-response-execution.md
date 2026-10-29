# Committed merge response checks

This change affects committed highway/roundabout execution. Entry approval,
road checks, authoritative object corners, STOP/AVOID policies and sensor topics
retain their existing contracts. Speed caps are unchanged.

## Highway

`planner.highway_merge.max_rear_decel_mps2` is 5.0 in the supplied configuration.
This is an NPC braking-response assumption, not an ego acceleration limit or a
LiDAR speed-error allowance. It is not an observed NPC braking capability.

A mapped rear vehicle on a prepared predecessor that converges into the same
actual target retains its following identity during execution. The candidate
must enter that lane, the actor must be behind and direction-consistent, and
the complete forward swept body must retain the existing road certificate.
The one-second response allowance starts from the current committed execution
check. Current contact and that entire allowance remain strict. PREPARE does
not inherit this continuation policy. Parallel routes without a common target,
pedestrians, cut-ins, reverse motion and leaving the certified road remain strict.

Rear prediction reuses the existing segment cache and bounded-braking sweep;
no new speed/lateral candidate catalogue is generated and no search deadline
is increased. Maintenance additionally checks the original interval of prepared
rear traffic. If only that conditional response fails, retain the existing
nominal road/front/physical-collision-checked controlled maintenance path, report
`rear_response_unverified; nominal_maintenance_only`, and replan next cycle.
This is risk mitigation, not a certificate of rear safety under every speed bound;
it does not issue new merge approval. It avoids replacing a forward maintenance
path with an even slower unconditional emergency command solely because of the
rear uncertainty envelope. If the original maintenance certificate also fails,
`emergency_command_not_certified_stop` identifies the last-resort emergency
command; it is not labelled a collision-free stopping plan.

## Roundabout

Execution-only lane support can use partial vehicle observations when fresh
position history and measured direction agree on the supplied circulating lane.
The original PREPARE whole-body/confidence requirements remain in effect.
Both supported speed estimates remain in the occupancy interval; corners are
not replaced. No circular successor is invented beyond supplied map coverage.

All existing passing certificates are retained first. A failed committed path
may then be checked under a mapped following vehicle's bounded response.
A leading/crossing actor cannot be reassigned as yielding rear traffic. Forward,
progress-to-stop and service-braking candidates use this same execution check.

`roundabout_merge.execution_reaction_time_s` is 0.3 in the supplied configuration;
`execution_max_rear_decel_mps2` is 3.0. Reaction time is anchored to the observed
physical entry, not restarted at every plan. Before physical entry the candidate's
entry time plus reaction is used. The full requested path and original observation
age are checked. Missing YAML keys retain conservative defaults of 1.0 s/3.0 m/s².

## Validation and deployment

Fixed recorded inputs are replayed through the C++ planner. This checks planning
regressions, not a closed-loop response of NPCs to the new control commands.
There is no independent NPC ground-truth speed in these recordings. Simulator
verification of response assumptions and control tracking is still required.

After updating the branch, rebuild the complete planner and its consumers:

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4
source /root/ws/devel/setup.bash
catkin_make run_tests_asmc_planner -j4 -l4
catkin_test_results /root/ws/build/test_results
```

Restart the planner launch manually after the build. Its private class layout
changed; retaining old adapters/shared libraries can cause an ABI mismatch.
Runtime launch options are unchanged. No process is restarted by these changes.
