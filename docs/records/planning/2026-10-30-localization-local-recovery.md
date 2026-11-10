# Localization → LOCAL recovery comparison (2026-10-30)

현재 정책·팀 통합·06:20 결과는 [후속 체크포인트](2026-10-30-team-shadow-0620.md)를 따른다. 아래는 당시 실행 이력이다.

Status: opt-in implementation restored after baseline rollback; recovery driving remains unaccepted. Start: `b001c2c1`,
backup ref `archive/pre-localization-local-recovery-20261030`.
Remote compare branch was fetched; six existing local commits are preserved.
No main merge/rebase, other worktree or perception parameter changes.

## Execution and ownership

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch \
  jamming_planner:=localization localization_local_recovery:=true
```

Keep the compare container's existing master/port profile. When its persistent
8089 dashboard is running, use `dashboard:=false`. Disable the experiment with
`localization_local_recovery:=false`; camera remains the repository launch default.
Only localization's MPC publishes the jamming command. The additional LOCAL node
and semantic input produce candidate paths, not control. NORMAL behavior is unchanged.

## Recovery policy and boundaries

- A valid native path with usable speed is preferred. Static geometry/path failures
  can request recovery after 0.5s below 0.5m/s. Stopping is not itself permission.
- Candidate source <=350ms, plan/receipt <=250ms, same run, finite geometry,
  matching ego/header identity and an available same-time native motion sample.
- LOCAL uses its existing full-body road/OBB checks and a 5km/h cap. Its speed and
  braking profile are not overwritten. No object is discarded by the handoff.
- Transform the LOCAL odometry-frame trajectory to the localization odometry frame
  using both ego poses at the candidate header time. Original header/scene stamps
  remain unchanged; this does not claim the two odometry estimators are identical.
- Native motion/IMU/object/anchor validity remains required. Moving or unresolved
  motion objects and dynamic-yield reasons veto recovery. This intentionally does
  not solve false dynamic classification or map-anchor failure.
- Return after 1s continuously usable native paths. Current selected path is exposed
  in localization/path and debug selected geometry; trace fields `selected_source`,
  `selected_valid`, `selected_reason`, `selected_scene_stamp`, `recovery_policy`
  identify the MPC input; `native_reason`/`native_valid` retain the native decision. Public debug `reason`/`valid` describe the selected path.
- No new controller, watchdog extension, minimum-speed override, timeout-based
  obstacle permission, or promise of always moving through blocked space.

## Verification before physical comparison

23 policy/conversion assertions, localization input regression, dashboard payload,
three baseline launch graphs and the optional recovery graph pass in compare container.
The host lacks ROS Python; launch checks run inside the ROS container, not skipped.
`local_recovery_test`, `asmc_shadow_localization_node` rebuilt in /build and /devel.

Runner flag `--local-recovery` records the option. `--recording-profile decision`
retains decision inputs/paths/commands but omits raw point clouds/JPEG; such a bag
cannot establish full raw perception replay. Default full recording is unchanged.

## Cutoff result (04:50 KST)

Implementation checkpoint: `75160004`. Final follow-up keeps dashboard selected
validity/reason consistent with the actual MPC input and retains native fields.
No automatic promotion or main/remote changes.

- Prepared handoff ROS test passes: valid prepared entry/return produce no
  transition-only stop; old epoch/stale candidate and watchdog checks still hold.
  This is synthetic handoff evidence, **not** proof that A448 physical stopping is fixed.
- Actual simulator run `recovery-entry-20261030-r03` ended with
  `startup_readiness_timeout`, elapsed driving0s, no departure event. It cannot
  compare avoidance or establish the user's A448 stop cause. GPS-derived state
  stayed invalid; localization motion/trajectory and raw sensor topics were absent.
- Read-only GetVehicleNetworkSetting reported ROS/9090 while the trial used UDP.
  That API result plus missing data requires an effective sensor/network preset
  check; API settings alone are not treated as proof of the previously observed
  moving vehicle's stop cause. No sensor/network preset was modified.
- r01 was blocked by the existing dashboard's old revision; r02 found missing
  localization_evaluate. Dashboard was relaunched for the same master/revision,
  and the missing evaluator plus LOCAL/MPC binaries were built. Both failed
  preparations remain preserved and do not count as driving tests.
- Physical trial restored the original ego/scene, Foggy conditions and control
  mode; cleanup_errors=[], no recording process or trial controller remains.
  Existing master11373 and read-only planning8089/LiDAR8097 observers remain.
  The original compare UDP bridge was stopped for the exclusively owned trial;
  the trial bridge/perception/autonomy were cleaned up, not left driving.

## Evidence and rollback

Decision bag/logs (active parent campaign; unique append-only trial):
`D:/morai dataset/shahn03/artifacts/jamming-three-way-20261029-r02/trials/recovery-entry-20261030-r03/`

New closeout manifest and test logs:
`D:/morai dataset/shahn03/artifacts/localization-local-recovery-20261030-r01/`
The container has only the parent campaign writable, so its trial writer used
that active registered parent and the closeout run references those files without
copying the bag or changing previous provenance.

Use `localization_local_recovery:=false` for immediate behavior rollback after
relaunch; no revert is needed. For code rollback, revert the recovery commits on a
clean checkout, preserving pre-existing local commits through `b001c2c1`; never
reset to the older origin branch. No guarantee of continuous movement is made.

Next: establish live GPS/IMU/camera/LiDAR reception; repeat identical 1830→1890
entry with recovery OFF/ON and inspect native reason, candidate age and handoff
trace at station1859.6416. Then one static blockage and one moving actor test.
Until this passes, camera/local/localization final selection remains unresolved.


## Re-enabled checkpoint after baseline investigation

The user requested the pre-rollback implementation again. Original `75160004` and
`08a43ca7` were reapplied as `ecb99af5` and `fc95209d`, preserving the subsequent
runner heading guard (`7a922a7a`). Rollback reference:
`archive/before-reenable-local-recovery-20261030` (7a922a7a).
The baseline short NORMAL drive passed after resetting the simulator pose;
see [the separate investigation](2026-10-30-localization-start-rollback.md).
That result is not a physical acceptance of LOCAL recovery or A448 handoff.

Inside `asmc-jamming-compare`, with the existing bridge/camera/LiDAR and dashboard:

```bash
source /devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11373
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=localization localization_local_recovery:=true dashboard:=false
```

The existing observer is http://localhost:8089/web/index_v2.html; LiDAR view is
http://localhost:8097. Only launch one autonomy instance. To return to baseline,
stop that launch and repeat with `localization_local_recovery:=false`.
The option remains false by default; camera remains the default jamming planner.
LOCAL recovery starts a planner and semantic input, not a second jamming MPC.
No new autonomous drive was started for this rebuild request.

Rebuild verification: explicit CMake reconfigure restored the newly re-added test
target; localization node, LOCAL planner/controller and recovery test built.
Recovery policy/conversion 23 assertions, localization input regression, dashboard
9 tests and all baseline/optional recovery launch graphs passed. Runtime source
under src/ matches checkpoint 08a43ca7; subsequent changes are the runner guard
and investigation/re-enable documentation only.

## 05:40–05:50 anchor-loss recovery correction

Branch/container: `integrate/jamming-three-way-20261029` /
`asmc-jamming-compare`; master11373, planning web8089, LiDAR web8097.
Start `f7d58921` is preserved by
`archive/pre-transition-recovery-20261030`. Implementation `446fa1b5`,
conservative handoff correction `7b1086b1`; no push or merge.

Observed at investigation start: localization rejected `motion_epoch_changed`
with an unavailable map anchor; LOCAL reported a valid 4m
`verified_stall_observed_recovery` path. The recovery gate nevertheless required
`transform.valid`. Thus independent local geometry could not recover from the
lost map anchor. The inspected localization object count was zero: this sample
is not evidence that localization deliberately crossed an object rejected by
LOCAL. The cause of the original motion-epoch reset itself remains unconfirmed.
The runtime subsequently moved before the saved `before.json` snapshot; that
snapshot must not be described as the original stationary failure frame.

Changes retained:
- A specific anchor-loss whitelist permits fresh, source-aligned LOCAL recovery
  without a valid map transform. IMU, object validity, source age, matching local
  state, and dynamic-risk gates remain enforced. Unavailable inputs are not
  converted to free space.
- A missing LOCAL candidate withholds output but retains recovery ownership;
  a fresh replacement need not wait to become stationary again. Frame/epoch,
  input invalidity and dynamic hazard still revoke ownership.
- Separate `control_ready` allows a validated LOCAL path to establish command
  readiness without claiming `anchor_ready`. One jamming MPC remains responsible.
- After GPS returns, LOCAL-owned recovery may discard a lost anchor and warm up
  a new anchor while moving; original GPS consistency checks remain in force.
- NORMAL exit readiness uses the validated returned GPS anchor. NORMAL still
  independently requires two fresh valid trajectories and a fresh handoff
  command; it no longer waits for the retiring localization maneuver to finish.
- Localization target/cap is 15km/h; existing approach25 and recovery5 remain.
  Camera remains the default and optional recovery remains OFF by default.

Rejected/deferred candidate: retaining NORMAL from A448 until GPS becomes invalid.
A short physical trial after teleport reported invalid native map candidates and
NORMAL low-speed fallback. The first attempt also retained the pre-teleport route
projection (s≈41 rather than≈1859); restarting the planner corrected the projection
but did not establish a valid pass. The user reported simulator reset/body-state
inconsistency. The final native body-heading guard was true, so this trial does
NOT independently prove a chassis/IMU mismatch. We retain the original zone
handoff and exclude these runs from physical acceptance. Do not tune vehicle yaw
or weaken collision checks based on these contaminated trials.

Validation: recovery policy/conversion 28 checks and default/optional launch
graphs passed; isolated ROS handoff test passed. Planner and mode-source test
binaries were rebuilt after restoring zone handoff. No accepted continuous
entry/exit or obstacle recovery pass was obtained in this timebox. The fresh
LOCAL candidate must actually exist; this change is not an unconditional-motion
policy and cannot guarantee that every stopped scene will clear.

Evidence (unique append-only trial in the active registered comparison campaign):
`D:/morai dataset/shahn03/artifacts/jamming-three-way-20261029-r02/transition-recovery-20261030-0540/`.
Includes native actor snapshot, decision JSONL, reset state, original failure
rosout excerpts, two autonomy logs and test results. No test2 actor was deleted
or moved; only ego was moved to the runner's tunnel entry. No scenario file was
saved over test2. Trial autonomy was stopped and ego held in P; bridge, cameras,
LiDAR and the existing dashboard were retained. No raw sensor bag was collected.

Next acceptance: clean simulator reset and fresh planner initialization, compare
reported chassis yaw/IMU/actual forward displacement before driving, then rerun
same test2 entry→recovery→exit. Verify both loss-of-anchor LOCAL selection and
GPS-return handoff in live traces. To roll back only recovery behavior relaunch
with `localization_local_recovery:=false`; full prior source reference is
`f7d58921` (use a separate checkout or revert, do not discard other work).

## 05:49 test2 retest — no reset or teleport

User requested a five-minute verification of the retained checkpoint `08502fc1`.
The existing test2 scene was preserved, ego held at its current A448 position,
then a fresh autonomy process was started on master11373. No simulator reset,
teleport, object change or scenario save was used. Body-heading precheck passed.
Only this comparison container owned control; existing bridge/perception/web
were retained. Source behavior was not changed during the trial.

Results (one pass, native state sampled approximately 1Hz):
- 54.25s / approximately254.45m including NORMAL departure after tunnel exit.
- Initial LOCAL recovery supplied a fresh path at roughly5km/h. At elapsed3s
  native localization became usable and reclaimed the same jamming MPC.
- GPS-zero began around8s; normal tunnel travel was roughly14–15km/h.
- At11s `shadow_dynamic_yield` requested brake1. Native speed was below0.5km/h
  during samples at12–13s and moving again at14s (sampled stopped interval≈2s).
  This was a dynamic-risk stop, not the former anchor-loss deadlock. Whether
  the object's motion estimate was correct has NOT been verified; preceding
  dynamic stopping-prefix trace identified track1741. The stop must not be
  presented as proof of a real crossing actor or a fully smooth ride.
- GPS was valid again at41s; at42s JAMMING recovery waited for NORMAL readiness;
  at43s NORMAL was active while speed remained14.14km/h. No sampled stop at
  exit. Detailed handoff trace: command_age0.000822s, transport_wait0.029209ms.
- No stop lasting10s or more occurred. No non-Ego collision ID appeared in the
  native samples; this is sampled evidence, not a continuous collision audit.
- The original motion-epoch loss was not injected in this trial. Its recovery
  policy is regression-tested; this drive establishes LOCAL→localization and
  GPS-zero→NORMAL operation, not exhaustive acceptance of all failure states.

Evidence: active comparison campaign's unique child
`D:/morai dataset/shahn03/artifacts/jamming-three-way-20261029-r02/test2-retest-20261030-0549/`:
`before.json` (source/actors), `live.jsonl`, `events.jsonl`, `autonomy.log`,
`after.json`, `summary.json`. Raw sensor replay bag was not collected.
The initial LOCAL recovery was the existing near-field-assisted path, so this
is not a new normal-speed obstacle avoidance acceptance.

End state: trial autonomy and additional recorder stopped, ego held in P;
bridge, camera, LiDAR and dashboard8089 remain. No push/merge or operating
default promotion. Immediate follow-up is to verify track1741's dynamic-risk
stop with matched perception evidence, rather than bypassing dynamic yield.
