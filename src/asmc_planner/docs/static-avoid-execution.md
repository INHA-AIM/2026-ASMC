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
Both production launch defaults also use 0.45 m; the launch-margin regression
checks their overrides and forwarding as well as the YAML defaults.

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

## MORAI progress recovery

The MORAI `autonomy_no_bridges_v2.launch` now enables static recovery and
explicit contact progress. Existing launch commands work unchanged. Set
`static_avoid_contact_progress:=false` to keep physical contact blocked, or
`static_avoid_recovery:=false` for the original strict policy. The standalone
planner/controller launches default to contact progress disabled. Core YAML
callers retain strict defaults unless they explicitly enable recovery.

After nominal planning fails, confirmed fixed objects can use clearance levels
0.30, 0.20, 0.10 and 0 m; the vehicle body is never shrunk. Other objects keep
the nominal 0.45 m margin and their timed/stationary hypotheses. The selected
recovery speed cap is 1 m/s. Search cursors survive each bounded search slice;
finding no candidate is not a proof of global physical infeasibility.

When body-clear alternatives are not found, the explicitly enabled contact
policy follows a previously nominally validated spatial reference at 0.5 m/s.
It executes a rolling prefix with a full stopping tail, retimed from actual
pose, speed and acceleration. Road containment and actuator checks remain.
Only current LOW_THREAT fixed occupancy with valid corners can be a contact
cost. Vehicles, pedestrians, mobile semantics, UNKNOWN/PENDING/DYNAMIC motion,
unavailable velocity and invalid geometry retain strict collision checks.
No time spent waiting, by itself, authorizes throttle or relabels an actor.

Contact cost integrates overlap length and penetration severity; a searched
replacement with less contact can replace the retained prefix. A newly found
body-clear path takes precedence over contact progress. Reduced margins latch
through the maneuver; a normal retained corridor must pass three consecutive
checks before ordinary execution resumes. Contact execution never overwrites
the last nominally checked reference.

`Trajectory.execution_policy` distinguishes normal, reduced-margin and
contact-progress execution. Failure codes still mean failure. Controller
checks require a fresh AVOID owner, a stop-capable bounded speed profile, and
an independent opt-in for contact; braking release still requires a new valid
plan. Feedback detail records policy, chosen margin, minimum perceived fixed
object clearance and contact cost. These diagnostics do not certify absence
of simulator ground-truth contact.

The frozen fixture `static_obb_growth_stall.json` records moving-entry and
stopped observations from bag SHA256
`8089232c8ec9a7ceeed98b568fd7fb5e59ec875438319d99ab258523d6fd9003`.
The new regression retains the original COMMIT, recovers after OBB growth and
checks ideal-tracking completion. A separate synthetic test requires progress
through fixed occupancy while preserving actuator and mobile-actor guards.
Both regressions also feed every recovery speed profile through Controller's
independent policy gate. Margin-only execution stays blocked by the synthetic
physical wall; contact-enabled execution eventually returns to normal policy.
These are frozen-observation tests, not a replay of MORAI actuator physics.

For MORAI, compare strict, margin-only and contact-enabled runs on the same
placement and approach speed. Record actual contacts and forward progress
separately from planner validity. The scenario runner's default collision-end
policy must not be confused with the planner's contact-progress result. Add
`--continue-on-static-contact` to record registered fixed static-mission
contacts while continuing the run; other actors retain collision termination.

Verification includes a full 126-registration CTest run and successful reruns
of the five affected registrations after the final acceleration/profile fixes.
The cross-component profile regression also passes. Catkin XML results report
881 tests, zero errors/failures/skips. Runner mock integration (46), headless UI
(35), and old/current trajectory decoding (2) tests pass separately. These
checks do not start MORAI or the workspace's driving stack.

## Baseline offline validation and MORAI acceptance

Verified on 2026-10-29 in the isolated build: the six selected packages and
their test targets build successfully; all 124 CTest registrations pass,
including six ROS integration registrations. `catkin_test_results` reports
873 tests with zero errors, failures or skips. These two counts are different
views of the same run, not additive coverage. MORAI closed-loop physics has
not been run as part of this verification.
The subsequent launch-default correction passes the separately registered
`static_avoid_launch_margin_test` as well (125 CTest registrations total).

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

For strict and margin-only runs, pass requires physical clearance/road
containment, feasible tracking, finite handoff, no unexplained stop/go loop in
a known feasible static scene, and restart when the certified route and all
other release conditions are present. Contact-enabled runs report actual
contacts separately and require continued progress through opted-in fixed
occupancy. Uncertain input, signals, pedestrians, road and actuator limits
remain valid stop reasons under every policy.

## Build and restart

Trajectory's ROS message layout and control-binding wire version (now 3)
changed. Rebuild messages, planning, control and all dependent libraries
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
