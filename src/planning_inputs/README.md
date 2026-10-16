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
roslaunch asmc_planner autonomy_no_bridges_v2.launch objects_source:=lidar
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

The MORAI profile publishes position and yaw in the same UTM zone 52N map
basis. Its raw IMU yaw is true ENU, so
`orientation/imu_yaw_is_true_enu: true` applies the local projection rotation
before lever-arm removal and pose propagation. At K-City this is approximately
-1.35 degrees; the rotation is calculated from each valid GPS fix, not stored
as a track-specific constant. An IMU producer that already supplies grid yaw
must set this option to `false` to avoid rotating twice. Message schemas and
the raw `/imu` topic are unchanged.

`position/gps_correction_gain: 0.35` softens GPS position corrections at their
source timestamps. This setting depends on consistent position/heading frames;
reducing the gain alone can increase drift. A GPS `(latitude, longitude)=(0,0)`
jamming sentinel is rejected even if its status is positive. It invalidates
the GPS-based planning state without adding the sentinel to its histories;
the existing jamming-mode selector still owns camera-control handover.

Latest driving/merge/avoidance parameter files and Frenet map snapshot are kept.
asmc_bridge, traffic-light inference, asmc_hd_map runtime, and jamming code are
unchanged by the LiDAR restoration. 현재 신호등 생산자는 카메라 담당의 asmc_camera로 단일화했습니다.
판제 launch는 추론을 시작하지 않으며 폐기된 traffic_light:=false 인자를 지정하지 않습니다.
중단·재개·주기는 카메라 set_mode에서 제어하고 판제는 신호 규칙과 freshness 처리를 유지합니다.

The dashboard consumes /planning/objects and now receives restored Main
/lidar/motion_debug filter and motion diagnostics. Record both topics,
/last_lidar, /lidar/tracked_objects_map, /planning/vehicle/state, raw sensors,
trajectories, commands, feedback and /ctrl_cmd.

## Verification

CI builds the root workspace and the nested Main workspace with the frozen
message schema, runs original sensor/filter regressions, and checks actual
ROS transport for exact-time motion/geometry preservation and empty frames.
Old successful drives establish the reference policy; new MORAI driving must
confirm closed-loop behavior after integration. Main 연결은 main에 반영됐으며, asmc_lidar로의 기능 이전은 아직 완료되지 않았습니다.

고속의 GPS 차분 속도 잡음은 `speed/gps_gain_reference_speed_mps: 5.0`과
`speed/gps_gain_minimum: 0.08`로 완화합니다. GPS 속도와 예측 속도 중 작은 값이
5m/s를 넘으면 기존 속도 보정 이득 0.35에 `(5 / speed)^2`를 곱하고 하한 0.08을
적용합니다. 감속·저속에서는 기존 강한 보정을 사용하므로, 전체 속도 영역의 이득을
0.08로 낮췄을 때 생기는 정지 후 속도 오차 지속을 피합니다. IMU 가속도 예측·입력
유효성·정지 판정 기준은 유지됩니다. 기준 속도 0으로 기존 고정 이득과 비교할 수 있습니다.
