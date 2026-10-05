# Planning input adapters

> **Document role:** Active Package Guide — private Behavior/Frenet/MPC inputs.

This package preserves the tested planning state/motion policies while leaving
main's `asmc_bridge`, `asmc_perception`, `asmc_hd_map`, nested LiDAR workspace,
shared `asmc_msgs`, and jamming controller unchanged.

## Runtime

Start main's bridge and LiDAR workspace as usual, then run:

```bash
roslaunch behavior_planner autonomy_no_bridges_v2.launch objects_source:=lidar
```

The input is main's actual `Msg/TrackedObjectArray` on `/last_lidar`.
The currently reserved `/perception/lidar/*` typed producer is not assumed to
exist. A wire bridge accepts only the frozen legacy/canonical MD5
`e127b2aabd7494ad01a5a56b005bf07e`; development messages with an additional
`motion_state` field are rejected. Rebuild the sensor workspace from this
main-based branch before connecting it. No nested `Msg` build dependency is
introduced into the root planning workspace.

The bridge publishes an internal `/planning/input/tracked_objects` copy,
keeping source headers, SI velocities, corners and classifications unchanged.
The planning motion adapter derives UNKNOWN/LOW_THREAT/PENDING/DYNAMIC from
track history, source-aligned ego heading and already transformed world OBB
bounds. Duplicate frames cannot confirm a state; missing/stale heading or a
source gap over 0.5 seconds resets confirmation. Only confirmed LOW_THREAT
receives zero velocity in the private output, matching the planning policy;
other states retain source velocity. The public main output is never modified.
Motion parameters were carried from the validated branch's launch into
`config/motion_params.yaml`.

GPS/IMU source-time propagation and the 0.35 m antenna lever arm correction
are retained in a separate state producer. Its default output is
`/planning/vehicle/state`; main's `/vehicle/state` producer is untouched.
All three planning consumers and the dashboard use the same private state.
The `state_source:=ego_dev` option retains main's dev adapter with a scoped
topic remap. Do not run two control stacks publishing `/ctrl_cmd` concurrently.
This launch does not start the jamming controller.

## Messages and recording

`planning_inputs/DetectedObject*` carries the extra motion field. Frozen
`asmc_msgs/DetectedObject*` and `Msg/TrackedObject*` are not changed.
Planning object output is `/planning/objects`, distinct from perception's
public topics. Existing speed/merge tuning remains in the planner YAML files.
The fixed Frenet map snapshot remains under `frenet_planner/config/hd_map`;
merging main does not switch this planner to a different runtime map.

Record `/planning/vehicle/state`, `/planning/objects`, `/last_lidar`, `/gps`,
`/imu`, trajectory, command, feedback and `/ctrl_cmd` for post-integration
validation. Old bags recorded only on `/vehicle/state` and
`/perception/objects` do not directly exercise this new ROS boundary.

## Validation boundary

The full transport-independent Frenet core builds and its selected avoidance,
roundabout, KEEP/braking, FOLLOW and collision regressions pass. Portable
motion tests cover static confirmation, rigid translation, one-sided OBB
growth, track misses and source-time heading interpolation. Main's legacy and
canonical LiDAR message MD5s agree; the wire gate rejects incompatible types.
ROS node compilation/transport and MORAI closed-loop driving must additionally
be verified in the ROS Noetic workspace. The old successful drives include
LiDAR internal changes intentionally excluded from this main-based branch;
identical closed-loop behavior is not claimed.

The pre-rebase reference is branch
`backup/avoid-merge-validated-20261005`, commit `0781402`.
The roundabout implementation notes are in
[roundabout-merge-implementation.md](../../docs/packages/planning-control/roundabout-merge-implementation.md).
