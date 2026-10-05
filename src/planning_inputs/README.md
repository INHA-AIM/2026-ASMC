# Planning input adapters

> **Document role:** Active Package Guide — Behavior/Frenet/MPC inputs.

## Validated LiDAR policy (default)

The pre-rebase reference is 0781402 on
backup/avoid-merge-validated-20261005. Its complete nested Main LiDAR processing
and final-publication policy are restored: constrained near/far ground fitting,
robust ground-clearance evidence, adaptive clustering, tracking association,
frame filtering, four-state motion history, synchronized source-time pose and
GPS sanity checks, road overlap/prediction and static boundary clipping,
known-static catalog and zones, and planning height acquisition/release grace.

The Main cpp/hpp processing and original tests are carried from that reference.
Current asmc_bridge topic remaps and relocated asset paths are retained.
Frozen Msg/TrackedObject and shared asmc_msgs schemas remain unchanged:
Main sends source-aligned motion state through /lidar/motion_debug JSON with
exact integer stamp_secs/stamp_nsecs.

Build both workspaces after pulling. Do not keep running an old Main executable:

```bash
# Main ROS workspace
./scripts/build_ws.sh
# Nested LiDAR workspace
cd src/lidar+sensorfusion
catkin_make -j2 -l2
source devel/setup.bash
```

Start Main/main.launch and the simulator bridge, then start the planning stack
from the root workspace environment:

```bash
roslaunch behavior_planner autonomy_no_bridges_v2.launch objects_source:=lidar
```

Default lidar_source_policy:=validated_lidar consumes Main's fully filtered
/last_lidar. The wire bridge requires matching exact source timestamps and
delivered IDs in /lidar/motion_debug, then carries the authoritative state into
the private planning schema. No second classifier, geometry clipping, or
height gate is applied. Missing/mismatched diagnostics produce no fabricated
planning frames. Empty aligned frames still clear prior objects.

The optional lidar_source_policy:=main_raw with
lidar_input_topic:=/lidar/tracked_objects_map keeps the earlier private
motion and three-filter adapter for integrations that explicitly use main's
unfiltered schema. It does not reproduce the complete validated sensor policy.

## Other integration contracts

Private planning state keeps source-time GPS/IMU propagation and the 0.35 m
antenna lever arm; all three planners share /planning/vehicle/state.
Latest driving/merge/avoidance parameter files and Frenet map snapshot are kept.
asmc_bridge, traffic-light inference, asmc_hd_map runtime, and jamming code are
unchanged by the LiDAR restoration. Standalone v2 starts traffic-light inference;
pass traffic_light:=false if an external producer already runs it.

The dashboard consumes /planning/objects and now receives restored Main
/lidar/motion_debug filter and motion diagnostics. Record both topics,
/last_lidar, /lidar/tracked_objects_map, /planning/vehicle/state, raw sensors,
trajectories, commands, feedback and /ctrl_cmd.

## Verification

CI builds the root workspace and the nested Main workspace with the frozen
message schema, runs original sensor/filter regressions, and checks actual
ROS transport for exact-time motion/geometry preservation and empty frames.
Old successful drives establish the reference policy; new MORAI driving must
confirm closed-loop behavior after integration. Main has not been merged.
