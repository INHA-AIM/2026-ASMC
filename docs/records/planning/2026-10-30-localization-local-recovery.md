# Localization → LOCAL recovery comparison (2026-10-30)

Status: implemented opt-in; physical acceptance pending. Start: `b001c2c1`,
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
  separate the MPC input from native `reason`/`valid`.
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

Physical findings and exact final commit will be appended after comparison.
