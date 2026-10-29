# Localization → LOCAL recovery comparison (2026-10-30)

Status: implemented opt-in; physical trial blocked before departure, not accepted. Start: `b001c2c1`,
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
