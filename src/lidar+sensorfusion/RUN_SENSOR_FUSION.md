# LiDAR + camera sensor fusion

**2026-10-14 current camera contract:** [E8 deployment, model choice and three groups](../../docs/operations/camera-deployment.md). Main consumes external camera results with `camera_class_profile:=mission16_v1`; camera E8 runs in the outer workspace. `start_yolo` is removed.

> **역할:** Active Standalone Guide — 독립 nested catkin workspace의 build·runtime·검증
>
> Phase 4 통합 경계와 중복 package 제거 gate는
> [Perception 개발·통합 가이드](../asmc_camera/docs/integration.md)를 따른다. 이
> workspace는 outer catkin package처럼 취급하거나 parity 확인 전에 이동하지 않는다.

This directory is a self-contained ROS1 Noetic catkin workspace nested under
the `2026-ASMC` repository. `CATKIN_IGNORE` keeps the outer workspace from
recursively building the copied packages; build this directory separately.

The authoritative ASMC v1 poses are:

- front camera: `xyz=(1.90, 0.00, 1.20)m`, `rpy=(0, 2, 0)deg`, 1280x720, HFOV 90 deg
- 3D LiDAR: `xyz=(0.80, 0.00, 1.45)m`, `rpy=(0, 0, 0)deg`
- GPS: `xyz=(0.35, 0.00, 1.25)m`

The runtime copy is in `src/Main/setting.yaml`; the simulator source is the
outer repository's `config/morai/sensors/ASMC_sensor_set_v1.json`.

## Current camera connection — 2026-10-16

Main consumes camera-owned YOLO through its front-only CPU adapter. The independent
Main YOLO node, weights, `start_yolo` and GPU options have been removed. Main does
not start a camera producer. Existing LiDAR processing and planner outputs remain.

The complete current build/source, PREPARE/ACTIVE, model recovery and stale-build
cleanup instructions are in [Camera → Main → LiDAR handoff](../../docs/handoffs/camera-to-lidar.md).
Build/source the outer workspace first, then the nested workspace:

```bash
cd /path/to/2026-ASMC
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2
source devel/setup.bash
cd src/lidar+sensorfusion
catkin_make -j2 -l2 -DCATKIN_WHITELIST_PACKAGES='Main;Msg;morai_msgs'
```

Start `asmc_bridge` once using the [current bridge guide](../asmc_bridge/docs/integration.md).
Camera terminal (outer setup):

```bash
source /opt/ros/noetic/setup.bash
source /path/to/2026-ASMC/devel/setup.bash
roslaunch asmc_camera camera.launch mode:=active models:=objects
```

Main terminal (outer setup before nested setup):

```bash
source /opt/ros/noetic/setup.bash
source /path/to/2026-ASMC/devel/setup.bash
source /path/to/2026-ASMC/src/lidar+sensorfusion/devel/setup.bash
roslaunch Main main.launch rviz:=false
```

Camera starts ACTIVE and publishes continuously by default. Only when explicitly
starting with `mode:=standby`, use PREPARE → prepared=true → ACTIVE to resume.
Mission bounds guide planning consumption, not publication gates; Frenet requires
fresh object input even on map-only roads. See the [consumption guide](../../docs/handoffs/perception-to-planning.md).
Camera defaults: E8 TensorRT, front544×960/sides480×640,
15Hz target, confidence0.40/NMS0.50. Recovery is `detector:=pretrained` after restart.
The supported packaged engine environment is RTX4060Ti/TensorRT10.13.0.35; Main itself
no longer performs a GPU preflight. Do not reuse old Main GPU instructions.

Only valid front results go to `/YOLO/BBox`; valid-empty is forwarded, invalid/side/zero
stamp is withheld. The wire has no confidence/group fields: filtering occurs before
conversion, original names are retained and Main reconstructs the three groups.
The extra adapter cutoff defaults to0.0. Fusion remains unique, ±50ms and200ms wait.
Classification confirmation/change/miss grace remains5/3/3; motion is independent.
`/last_lidar` and tracked/motion planner contracts remain unchanged. LiDAR-only uses
`enable_sensor_fusion:=false`. A missing camera is not evidence to delete LiDAR objects.

The old broad bottom-image exclusion remains disabled. Current YOLO uses3-view Ego
masks rather than the old standalone YOLO hood-filter runtime.
`asmc_lidar` provides coordinate/time foundations and a reusable camera receiver, bounded buffer and projection API.
Main reads `asmc_msgs/InferenceStatus` on `/perception/camera/objects/status`; rebuild/restart camera and Main together.
The old camera-owned status/service types remain on legacy node control paths.
Jaeho owns the remaining migration and3-view fusion implementation.
Historical standalone YOLO and comparison runners can be inspected in commit
`ae8dcbc0`; use a matching isolated checkout rather than reinstalling them here.

## LiDAR output profiles

`main.launch` is the production profile and its defaults are unchanged:
`enable_common_geometry:=false`, `lidar_level_roll:=false` and `planner_udp_enabled:=true`
(the planner UDP sender follows `setting.yaml` `PlannerInterface`, 127.0.0.1:7510). Legacy
`/last_lidar`, `/lidar/tracked_objects_map`, `/lidar/motion_debug` and UDP are published for every
processed scan; neither `/lidar/local_object` nor `/lidar/map_object` is advertised (nor are the v1 candidates
`/perception/lidar/objects_local` and `/perception/lidar/objects_map`). Known gap of this profile:
warm-up tracks still reach `/last_lidar` and UDP as speed-0 objects.

`main_integration.launch` is the opt-in LiDAR v2 integration profile. It includes `main.launch` and
pins `enable_common_geometry=true` and `lidar_level_roll=true`, so IMU roll and pitch are each removed
exactly once. Each scan publishes at most one `asmc_msgs/LidarObjectArrayV2`:
`/lidar/map_object` (`map`) when the map pose is valid,
`/lidar/local_object` (`base_link`) while GPS is missing, invalid, stale or recovering,
and nothing when the LiDAR, time, IMU, CAN or transform input is invalid (the reason is reported on
`/perception/lidar/diagnostics`). The legacy outputs and UDP follow only `/lidar/map_object` scans whose
objects all have a valid motion state. Offline replays, bags and tests pass
`planner_udp_enabled:=false`: the planner UDP sender socket is not created and nothing is sent (roscpp's
own UDPROS server socket may still exist; it sends nothing):

```bash
roslaunch Main main_integration.launch rviz:=false planner_udp_enabled:=false
```

Contract and truth table: [LiDAR object v2](../../docs/contracts/lidar-object-v2.md).
Rollout, legacy status and open blockers:
[LiDAR v2 rollout](../../docs/handoffs/lidar-v2-output-rollout.md). The planner still
consumes `/last_lidar`; moving it to the v2 topics is a separate later step.

## LiDAR-only motion decision

`MotionClassifier` keeps its decision history separately from the tracker's
current-frame velocity. Thus `velocity_mps` and `speed_kph` always remain the
latest KF result, while only `is_dynamic`, the final marker text, and the
dynamic log use the filtered decision.

- Ignore the first 3 valid observations.
- Keep the latest 5 accepted signed `(vx, vy)` samples and take each component's
  median before applying `abs`.
- Treat `|vy| >= 3 km/h` as the higher-priority lateral threat. A raw
  `|vy| >= 5 km/h` sample becomes `PENDING` immediately and two same-sign
  samples confirm `DYNAMIC`.
- Treat `|vx| >= 10 km/h` as longitudinal threat only after 5 accepted samples
  and an ENU displacement speed of at least 5 km/h.
- A velocity jump over 10 km/h does not replace the stable history until 3
  candidates have a direction cosine of at least 0.7 and adjacent candidate
  velocities are within 10 km/h.
- Preserve state/history across at most 3 missed observations; a longer miss
  resets only the classifier history. Only `miss_count == 0` objects are
  published to the typed map/final topics.

`PENDING` is deliberately serialized as `is_dynamic=true` for the safety path.
The exact state, raw velocity, decision velocity, ENU displacement check, jump,
sample counts, and age are included in the final-object log/marker diagnostics.
All thresholds are private ROS parameters prefixed with `motion_` in
`src/Main/main.launch`.

`start_main.sh` consumes `/image_jpeg/compressed` and `/velodyne_points` from
the outer bridge. It does not bind UDP 9299. For legacy direct-UDP operation,
stop the outer bridge and run `START_VELODYNE_DRIVER=true ./start_main.sh`.

## RViz

Run `rviz`, set **Fixed Frame** to `map`, and add:

| Display | Topic | Contents |
|---|---|---|
| MarkerArray | `/lidar_tracking_markers` | fitted 3D OBB, track ID, fused class, speed, motion flag, velocity arrow |
| PointCloud2 | `/lidar/debug_clustered` | one centroid per current cluster in the preprocessed LiDAR frame |
| PointCloud2 | `/lidar/debug_ransac` | non-ground points |
| PointCloud2 | `/lidar/debug_roi` | range/ROI-filtered points |
| PointCloud2 | `/velodyne_points` | raw LiDAR cloud |
| PointCloud2 | `/lidar/debug_clustered_map` | the same current-cluster centroids transformed to map ENU |
| Msg/TrackedObjectArray | `/lidar/tracked_objects_map` | numeric map ENU OBB centers/corners and absolute velocity vectors |
| MarkerArray | `/lidar/tracking_markers_map` | map ENU OBB, center, km/h text and one-second velocity vector |
| Msg/TrackedObjectArray | `/last_lidar` | drivable-area-filtered final objects in map ENU, including the fused `classification` and the raw YOLO label `yolo_class` |
| MarkerArray | `/last_lidar_markers` | final OBB, unit velocity arrow, class/YOLO label, speed/age/ENU/road ratio in `velodyne` |
| MarkerArray | `/lidar/object_boxes` | optional fused AABB and class label |
| PointCloud2 | `/lidar/object` | optional fused object-only cloud |
| Image | `/fusion/debug_image` | camera image with projected LiDAR/fusion debug overlay |

The one topic required for OBB + ID + speed visualization is
`/lidar_tracking_markers`.

Its OBB markers are rebuilt from only the objects observed in the current
LiDAR frame (`miss_count == 0`). Every MarkerArray starts with `DELETEALL`,
so an OBB from a disappeared or predict-only track cannot remain in RViz.
Predict-only tracks may still keep their ID/velocity annotations, but they do
not publish either an OBB or the old fallback AABB.

Useful checks:

```bash
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /Ego_topic
rostopic hz /YOLO/BBox
rostopic hz /lidar_tracking_markers
rostopic hz /fusion/debug_image
rostopic echo -n1 /lidar_tracking_markers
rosrun rqt_image_view rqt_image_view /camera/image/compressed
```

`/lidar_tracking_markers` is a visualization message, not a stable downstream
tracked-object API. Consumers can inspect it with `rostopic echo`.

`/lidar/tracked_objects_map` is a legacy map-frame topic kept for recording and
validation tools (`analyze_velocity_bag.py`, `src/Main/scripts/build_known_static_object_catalog.py`,
`tools/perception/*`). No runtime planner subscribes to it by default: `planning_inputs`
consumes `/last_lidar` and `/lidar/motion_debug`, and only its opt-in
`lidar_source_policy:=main_raw` path reads this topic. It is a removal candidate once these
consumers migrate; new consumers use the v2 topics. In `main_integration.launch` it
follows the same gate as `/last_lidar`. Its velocity components use m/s; `speed_kph` is
display-only.

Main is the only `/last_lidar` publisher.
`src/asmc_hd_map/development/won_tae_link_node/drivable_filter_node.py` republishes
`/lidar/tracked_objects_map` as `last_lidar` for manual cross-checks of the C++ drivable
filter. No launch file starts it; running it next to Main would create a second
`/last_lidar` publisher. `tools/audit/check_legacy_lidar_publishers.py` (CI) fails if a
launch file or shell launcher starts it.

To record one validation scenario after rebuilding:

```bash
rosbag record --lz4 -O lidar_velocity_test.bag \
  /velodyne_points /gps /imu /Ego_topic /Object_topic \
  /lidar/tracked_objects_map /lidar/tracking_markers_map \
  /lidar/debug_clustered_map
```

After recording, generate the synchronized samples, summary and plots:

```bash
./analyze_velocity_bag.py lidar_velocity_test.bag
```

To lock the comparison to known IDs, add `--gt-id 101 --track-id 7`.

Outputs are written below `lidar_velocity_test_velocity_analysis/`.

### MORAI 지도 방향 기준

`main.launch`의 `imu_yaw_is_true_enu` 기본값은 `true`입니다. MORAI의 IMU yaw는 실제 동·북 방향 기준이고, GPS 위치는 UTM52N 지도 눈금 기준이므로 지도 위치를 만들 때 위도·경도로 방향 차이를 계산합니다. `planning_inputs/config/vehicle_state.yaml`과 같은 설정을 사용해야 차량과 장애물의 방향 기준이 맞습니다. 이미 UTM yaw를 제공하는 다른 입력이면 두 곳을 함께 `false`로 설정하십시오.

`SensorPoseHistory`는 원본 IMU 이력을 보존하고 지도 출력용 복사본만 보정합니다. 기존 모드에서는 위치 외삽과 장애물 지도 변환에 보정된 yaw를 사용합니다. 공통 geometry 모드에서는 GPS 시점 자세와 LiDAR 시점 지도 자세에만 보정을 적용하므로 GPS 없이 동작하는 로컬 인지 경로는 바뀌지 않습니다. 재밍·GPS 유효성·최대 나이 제한은 유지됩니다.
