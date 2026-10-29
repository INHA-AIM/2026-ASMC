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
policy executes at 0.5 m/s. A previously nominally validated spatial reference
is preferred when it reconnects and passes the current checks. Without one,
the planner generates a new C2 road-constrained rolling prefix from the actual
pose, speed, heading, curvature and acceleration, with a complete stopping tail.
This is available before the first successful nominal path and during PREPARE;
only atomic COMMIT releases an execution trajectory. Road containment and
actuator checks remain. The prefix is regenerated as the vehicle advances;
its temporary endpoint cannot become the end of the entire maneuver.
Only current LOW_THREAT fixed occupancy with valid corners can be a contact
cost. Vehicles, pedestrians, mobile semantics, UNKNOWN/PENDING/DYNAMIC motion,
unavailable velocity and invalid geometry retain strict collision checks.
No time spent waiting, by itself, authorizes throttle or relabels an actor.
Eligibility is checked per current object, independently of the commanded or
previously observed primary ID. If that ID disappeared before acquisition,
recovery can acquire the current fixed group intersecting the route; the
original command remains its execution owner. Known mobile primary requests
and in-lane-only dynamic maneuvers do not gain this acquisition exception.

Fresh-prefix recovery owns a finite group of nearby occupied footprints and
hands off using group passage, a checked response/stop corridor and KEEP
preview. A separated remote primary cannot keep the near group locked forever.
UNKNOWN transport type with LOW_THREAT motion uses the same confirmed-fixed
predicate for recovery and the dropout cache. A changed uncertain/mobile
observation invalidates that cache, and missing cached footprints expire after
the configured static dropout grace even during SHIFT/PASS.

Contact cost integrates overlap length and penetration severity; a searched
replacement with less contact can replace the retained prefix. A newly found
body-clear path takes precedence over contact progress. Reduced margins latch
through the maneuver; a normal retained corridor must pass three consecutive
checks before nominal clearance resumes. Speed follows the current checked
geometry: contact prefixes use 0.5 m/s and complete body-clear reduced-margin
routes use 1 m/s. A prior contact no longer forces later body-clear routes to
remain at 0.5 m/s. A complete nominal-clearance route can return to normal
collision policy and gradually regain speed during the same maneuver.

Nominal speed recovery requires at least five distinct perception timestamps,
one second of successful nominal checks and 0.5 m of measured forward progress
per step. Replanning one observation, waiting at rest, backward time/progress,
or a sensor gap over 0.25 s cannot accumulate this permission. A failed plan or
input/conversion failure resets the proof; a reduced-margin/contact selection
also resets it and chooses its own cap. Successful confirmation reopens only the
NEXT candidate's ordinary bypass target, up to 4.2 m/s and within the command and
bypass limits. This is permission to accelerate, never an instantaneous change
to the measured initial speed. The retimer still bounds acceleration and jerk.
That higher-speed candidate must pass current collision, road, lateral
acceleration, steering slew, longitudinal acceleration/jerk and stop-tail checks
before execution. Accepted profile history records its actual requested cap;
the zero-speed stopping endpoint cannot be used to infer cap continuity. If an
upgrade of the retained nominal corridor fails, the same connected geometry is
retimed at its previous accepted cap and fully rechecked against current objects
and road/actuation limits. A successful retry resets confirmation and reports
`speed_retry=1`; it does not reuse an unchecked old trajectory. If both speeds
fail, the ordinary recovery/failure handling remains active.
The current full road exterior must be inside the drivable area and the body
must already have nominal clearance from every perceived object. Initial-margin
escape and ongoing road-boundary recovery do not accumulate nominal speed proof.

The optional `planner/avoid/static_normal_resume_speed_cap`,
`static_speed_confirm_time`, `static_speed_confirm_distance` and
`static_speed_step` keys configure 4.2 m/s, 1.0 s, 0.5 m and 4.2 m/s defaults.
The step can still be reduced to request several separate confirmation windows.
These govern recovery inside the active AVOID maneuver. Ordinary AVOID without
a preceding recovery retains its existing bypass cap; completion hands back
to normal KEEP speed planning. Complete observation dropout and finite ghost
expiry can still regain a full nominal path when Behavior proposes KEEP; an
exception-bearing prefix still requires its fresh AVOID owner.
Recovery cannot retime SHIFT/PASS back into pre-entry TRACKING. Contact
execution never overwrites the last nominally checked reference unless a
complete nominal-clearance corridor is independently established.

`Trajectory.execution_policy` distinguishes normal, reduced-margin and
contact-progress execution. Failure codes still mean failure. Controller
checks require a fresh AVOID owner, a stop-capable bounded speed profile, and
an independent opt-in for contact; braking release still requires a new valid
plan. Feedback detail records policy, chosen margin, minimum perceived fixed
object clearance, contact cost and `fresh_prefix`. Speed recovery also reports
`static speed_cap`, `next_speed_cap` and `proof_frames`, including on normal
paths. They are appended even when
feedback also reports rejection of an attempted command replacement. Failure
detail separately reports `static recovery rejection`, including road,
curvature/retiming and strict-object collision rejection. These diagnostics
do not certify absence of simulator ground-truth contact.

The frozen fixture `static_obb_growth_stall.json` records moving-entry and
stopped observations from bag SHA256
`8089232c8ec9a7ceeed98b568fd7fb5e59ec875438319d99ab258523d6fd9003`.
The new regression retains the original COMMIT, recovers after OBB growth and
checks ideal-tracking completion. A separate synthetic test requires progress
through fixed occupancy while preserving actuator and mobile-actor guards.
Both regressions also feed every recovery speed profile through Controller's
independent policy gate. Margin-only execution stays blocked by the synthetic
physical wall; contact-enabled execution eventually hands off while every
published candidate obeys the cap for its current policy.
These are frozen-observation tests, not a replay of MORAI actuator physics.

`static_id_loss_stall.json` freezes entry, box collapse, ID disappearance and
settled observations from bag SHA256
`f9dd5d815a0a90675c7d0aacac3ba47bff9cc881356e4e2915523918cf1f33c7`.
Regression holds the stopped observation beyond the dropout grace, then
requires ideal-tracking completion. Synthetic cases start with no nominal
history, with a stale request ID, or a separated remote primary; repeated
tracker churn must not prevent completion or bypass the current policy cap. Actual
generator tests retain opt-out, uncertain motion, mobile-actor blocking and
explicit target-pose/direction constraints.
Complete observation dropout must regain a fully checked nominal route after
finite cache expiry without restoring cruise speed prematurely.

`static_speed_recovery_bag.json` freezes twelve paths and perceived-object
snapshots from bag SHA256
`8464fd158059d156bf6fb93d4a8a07faaf3f5878d73162450bbc6e898127c518`.
All ten nominal snapshots pass strict 0.45 m collision, road and retiming
checks for 1.0, 1.5 and 2.0 m/s targets from the recorded entry state. Two
reduced-margin snapshots pass the per-object reduced policy at 1.0 m/s.
Retargeting a low-speed path to 4.2 m/s lets the retimer reduce the actual peak
to approximately 2.72-3.90 m/s. Hypothetically entering those same geometries
already at 4.2 m/s rejects seven of ten for steering slew. Geometry clearance
therefore cannot, by itself, restore the old cruising speed.

The speed regression checks repeated-scan, stationary, sensor-gap and backward
time/progress guards. A synthetic ideal-tracking episode uses 456 cycles with
a fixed 0.5 m/s nominal recovery ceiling and 375 with certified recovery to
2 m/s; both complete. A new fixed blocker after acceleration must produce a
newly checked 0.5 m/s contact candidate and still complete. These comparisons
use frozen observations/ideal tracking and do not establish faster MORAI
closed-loop completion or ground-truth clearance.

For MORAI, compare strict, margin-only and contact-enabled runs on the same
placement and approach speed. Record actual contacts and forward progress
separately from planner validity. The scenario runner's default collision-end
policy must not be confused with the planner's contact-progress result. Add
`--continue-on-static-contact` to record registered fixed static-mission
contacts while continuing the run; other actors retain collision termination.

The speed-recovery build passes all 127 CTest registrations. Catkin XML results
report 881 tests, zero errors/failures/skips; the standalone progress and speed
regressions are separate CTest registrations. The final cap-continuity and
current-pose guards are rechecked by those two regressions. The old/current trajectory decoder's
two tests and the two launch/configuration tests pass. The prior progress-policy
change also passed runner mock integration (46) and headless UI (35) tests;
this group-recovery change does not modify runner code. None of these checks
starts MORAI or the workspace's driving stack.

An additional sequential-input audit retains the recorded nominal COMMIT and
feeds all 436 later feedback-time observations/poses from the ID-loss recording.
After 20.3 s, the old implementation rejects all 258 sampled cycles; the
updated implementation produces checked recovery trajectories in all 258.
Recorded poses stay fixed in this audit, so this is a planner re-entry check,
not a closed-loop completion result. The frozen-snapshot ideal-tracking test
separately requires recovery and completion.

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
7. Start at rest with a fixed blockage before any normal bypass is generated.
8. Let the primary OBB collapse/disappear while other fixed barriers remain;
   continue recording for more than the three-second dropout grace.
9. Check that repeated clear/expanded OBB observations never restore cruise
   speed from a transient clear frame. Sustained full nominal paths and actual
   progress should raise the cap gradually, up to 2 m/s; compare selected and
   normal control outputs and the speed-proof diagnostics.
10. Repeat cold start with a newly moving actor or pedestrian on the prefix;
    contact progress must remain blocked until strict collision checks pass.
11. After nominal acceleration, add a fixed blocker again. Require a checked
    reduced/contact speed profile rather than a stale high-speed continuation,
    followed by a new confirmation interval before nominal acceleration.

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
