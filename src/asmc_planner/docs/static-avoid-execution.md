# Static avoidance: continuous execution and recovery

This change is based on `c3c3579b` on
`fix/merge-response-consistency-20261029`. It does not alter the highway or
roundabout response model introduced by that commit.

## Acquisition and geometry

Behavior retains perception's authoritative eight corners. Acquisition and
static corridor generation project these corners into the reference frame;
fitted length/width/yaw do not replace valid corners. Legacy synthetic Behavior
inputs without corners retain their scalar projection.

The production body width is 1.9 m and the object clearance is 0.45 m in both
Behavior and Frenet configuration. The 2.161 m road exterior remains a separate
containment shape. Overlap ratios affect immediate confirmation; a smaller
intrusion into the collision corridor cannot permanently exclude a target.
The rear axle is the vehicle reference, so an alongside footprint can remain
actionable even when its center is behind the axle.
Frenet acquisition and unique spatial re-identification use that same footprint
gate. Open reference routes use signed station differences; only closed loops
wrap their longitudinal distance.

## Incumbent path and actuator limits

Static map avoidance keeps an accepted spatial path. Each cycle connects the
measured pose, heading and curvature to that path with a C2 spatial connector,
then updates time/speed from the current state. It rechecks the complete path
against current objects, road exterior, curvature, lateral acceleration and the
Controller's speed-dependent steering slew. An accepted path wins over a new
opportunistic corridor. Observation refresh does not widen its target first.

Retiming bounds forward acceleration by `planner/avoid/bypass_accel_cap` (2 m/s²,
matching Controller), while preserving independent braking/jerk limits. The
spatial profile includes a settled tail with a complete stop. A successful
handoff normally occurs before that tail, so the tail is a stopping witness,
not a requirement to stop after each obstacle.

A new hazard can invalidate an incumbent; retaining a path never bypasses a
fresh safety check. Failure leaves the accepted spatial geometry available for
checked reconnection when conditions recover. It does not authorize movement.

## Multiple obstacles and gentle S corridors

The primary ID remains the Behavior command's identity. All observed objects
continue to constrain collision safety. The static generator connects multiple
entry/pass gates, allowing different lateral offsets between blockers without
forcing a return to the reference after each object. Overlapping same-side
blockers reserve their joint width before entry. Direction/target-pose ownership
on the primary remains authoritative.

Groups are finite: up to eight actionable static footprints in an 80 m forward
window. The return-and-next-shift distance plus a response reserve determines
whether successive obstacles belong together. A later group is handled by a new
Behavior command after a safe handoff. Hard physical, road and actuator checks
remain the final authority; finite search failure is not proof that every
possible physical route is impossible.

Group passage, the accepted return schedule and Execution's PASS/RETURN facts
must agree. Passing the primary alone cannot finish a group. Completion checks
a finite response-and-stop corridor and KEEP preview, instead of requiring the
whole 120 m acquisition horizon to be obstacle-free.

## Initial margins and standstill

Several confirmed static objects can independently qualify for an initial
margin escape. Each must be physically separated; every escape must make
progress, cannot enter a margin from outside, and cannot re-enter after exit.
The maneuver retains measured progress anchors across planning cycles. Unique
nearby re-identification can retain an anchor; ambiguous matches do not.
UNKNOWN/PENDING objects do not gain this additional static recovery privilege.
Their existing moving/stationary hypotheses remain checked.

Standstill uses the same distance geometry and measured initial speed and
acceleration. Reconnection no longer invents a nonzero initial curvature at
rest. VehicleState has no measured wheel angle: MORAI validation must still
check restart with steering history/delay, not infer wheel position from zero
yaw rate. No timeout alone grants throttle.

## Failure and brake release

Planning preserves COLLISION_BLOCKED when publishing a one-point failure from
AVOID preparation or ordinary planning. Existing bound request/plan ownership
remains required for release. On legacy transport, an AVOID heartbeat alone
cannot clear a collision stop: release requires a newer, fresh, failure-free
moving trajectory. A normal intentional STOP and a planner failure remain
different outcomes.

A failed multi-point braking trajectory also caps Controller's positive
acceleration at zero. A positive initial acceleration retained for physical
continuity cannot reapply throttle on every failed planning cycle.

Feedback detail adds `retained spatial corridor` for successful retention,
`static corridor search pending` when applicable, and the last corridor
rejection stage (`road`, `collision`, or a specific retiming/actuation stage).
Candidate counts and one blocker ID do not describe every rejected candidate.

## Offline validation and MORAI acceptance

Verified on 2026-10-29 in the isolated build: the six selected packages and
their test targets build successfully; all 124 CTest registrations pass,
including six ROS integration registrations. `catkin_test_results` reports
873 tests with zero errors, failures or skips. These two counts are different
views of the same run, not additive coverage. MORAI closed-loop physics has
not been run as part of this verification.

`static_avoid_corridor_test` covers:

- start at 0, 0.49, 0.51 and 4 m/s, with a moving candidate and stop-capable tail;
- unchanged authoritative corners despite scalar-fit noise and small position
  jitter while progressing;
- an added physical wall, followed by checked recovery when it disappears;
- three alternating blockers and ideal tracking through a gentle S, including
  an injected standstill between blockers;
- an initially close non-primary object and multiple independent margin escapes;
- uncertain occupancy and physical overlap staying blocked;
- production acquisition/body/margin and positive-acceleration agreement.

Before commit, a paint-free corridor is preferred when found in the search
slice. Accepted geometry remains stable afterwards. Reconnection regressions
also advance an independent braking plant when a newly introduced obstacle
cannot be bypassed from the current speed/curvature; they require a checked
route and completion after braking, rather than assuming an instantaneous turn.

The updated completion regression requires safe handoff before a distant next
obstacle and rejects handoff when the next object is inside the response-and-stop
corridor. Existing FOLLOW, dynamic, merge, STOP, binding and watchdog tests are
also regression gates.

These tests do not reproduce perception or MORAI tire/actuator physics. The
25.84 s diagnostic recording (SHA256
`439e7550f653ab637524ee84d39428b7e80725f70bf9338fc9f3c2569e8b19cf`)
observed a successful commit followed by 12 valid-to-invalid transitions and 11
recoveries. It is evidence for the failure sequence, not a closed-loop oracle
for a changed planner. Its executable/configuration commit was not recorded.

After a full rebuild, use these MORAI gates:

1. Single obstacle: stationary start and approach at the original entry speed.
2. Two close same-side obstacles, then two sufficiently separated obstacles.
3. Three alternating obstacles forming a gentle S, mirrored left/right.
4. Stops during SHIFT, between blockers and during RETURN, followed by restart.
5. A newly blocking object and its removal; no release before a new valid path.
6. Original diagnostic placement, normal signals/traffic, then the full course.

Record the entire episode, including successful runs: perceived corners and
motion state, selected command, trajectory, Frenet feedback, vehicle state,
normal/selected controls and binding request/plan/status where enabled. Record
the actual binaries, parameters, map and seed. A stationary replay does not
restore the original moving approach or Controller history.

Pass requires physical clearance/road containment, feasible tracking, finite
handoff, no unexplained stop/go loop in a known feasible static scene, and
restart when the certified route and all other release conditions are present.
Actual blockage, uncertain input, signals and pedestrians remain valid stop
reasons.

## Build and restart

Frenet and Behavior internal layouts changed. Rebuild all dependent libraries
and nodes together, then restart the planning/controller stack. Do not hot-swap
only one shared library into already running nodes.

For a normal ROS workspace:

```bash
catkin_make -j4 -l4
catkin_make tests -j4 -l4
catkin_make run_tests_asmc_planner run_tests_asmc_controller -j4 -l4
catkin_test_results build/test_results
```

The implementation's isolated verification build uses `/tmp/asmc-static-build`
and `/tmp/asmc-static-devel`; it does not replace the workspace's running build.
