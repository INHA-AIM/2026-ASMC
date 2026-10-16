# P4-A perception runtime baseline

> **식별자:** `p4a_perception_runtime_v1`
> **입력 commit:** `1bd8ec9`
> **범위:** 패키지 이동·삭제·동작 변경 전 현재 인지 runtime 계약 동결

## 결론

현재 인지 runtime은 아직 하나의 `asmc_perception` 패키지가 아니다. camera semantic, metric BEV/lane,
traffic light, LiDAR/fusion, recorder가 분산돼 있고 LiDAR는 outer catkin에서 제외된 nested workspace다.
P4-A는 이 상태를 정상 목표로 승인한 것이 아니라, 이후 이동에서 frame·timestamp·topic·model 의미를
잃지 않기 위한 회귀 기준으로 동결했다.

- 감사 scope: 17개, tracked file 567개
- launch: 29개, node 선언 23개, declared entrypoint 34개
- 계약 입력: 8개, 출력: 50개
- 모델 artifact: 4개(선택 3, 미선택 중복 1)
- bag profile: 11개(deployment 허용 5, privileged GT 3)
- 중복 ROS message: 4개

기계 판독 정본은 `config/baselines/perception_runtime_baseline_v1.yaml`, 재생성·검증기는
`tools/audit/build_perception_runtime_baseline.py`다.

## 현재 package 경계

| ID | 현재 경로 | 분류 | 이관 목표 | tracked file |
|---|---|---|---|---:|
| `camera_semantic` | `src/perception/camera_semantic_perception` | `catkin_team_runtime` | asmc_perception/camera | 25 |
| `drivable_bev` | `src/perception/drivable_bev` | `catkin_team_mixed` | asmc_perception/bev_lane + tools/perception | 94 |
| `traffic_light` | `src/traffic_light` | `catkin_team_runtime` | asmc_perception/traffic_light | 10 |
| `data_collection` | `src/data_collection` | `catkin_team_mixed` | asmc_perception/recorder + tools/perception | 49 |
| `lidar_main` | `src/lidar+sensorfusion/src/Main` | `nested_catkin_team_runtime` | asmc_perception/lidar_fusion | 62 |
| `lidar_msg` | `src/lidar+sensorfusion/src/Msg` | `nested_catkin_team_duplicate` | asmc_msgs | 6 |
| `lidar_morai_msgs` | `src/lidar+sensorfusion/src/morai_msgs` | `nested_catkin_external_duplicate` | pinned MORAI-ROS_morai_msgs | 123 |
| `velodyne_vendor` | `src/lidar+sensorfusion/src/velodyne/velodyne` | `nested_catkin_upstream_vendor` | system dependency or vendor | 3 |
| `velodyne_driver_vendor` | `src/lidar+sensorfusion/src/velodyne/velodyne_driver` | `nested_catkin_upstream_vendor` | system dependency or vendor | 26 |
| `velodyne_laserscan_vendor` | `src/lidar+sensorfusion/src/velodyne/velodyne_laserscan` | `nested_catkin_upstream_vendor` | system dependency or vendor | 16 |
| `velodyne_msgs_vendor` | `src/lidar+sensorfusion/src/velodyne/velodyne_msgs` | `nested_catkin_upstream_vendor` | system dependency or vendor | 6 |
| `velodyne_pcl_vendor` | `src/lidar+sensorfusion/src/velodyne/velodyne_pcl` | `nested_catkin_upstream_vendor` | system dependency or vendor | 5 |
| `velodyne_pointcloud_vendor` | `src/lidar+sensorfusion/src/velodyne/velodyne_pointcloud` | `nested_catkin_upstream_vendor` | system dependency or vendor | 58 |
| `twinlite_morai` | `src/perception/twinlite_morai` | `non_catkin_backend_offline` | asmc_perception backend + tools/perception | 9 |
| `morai_3d_detection` | `src/perception/morai_3d_detection` | `non_catkin_research` | tools/perception candidate | 36 |
| `learning` | `src/learning` | `legacy_placeholder` | delete after reference gate | 1 |
| `learning_by_cheating` | `src/learning_by_cheating` | `legacy_runtime_research` | replace then delete; preserve in git history | 38 |

## 좌표·시간 계약

- camera metric BEV와 lane observation은 `base_link`이며 x-forward, y-left, z-up이다.
- LiDAR tracked object 위치·속도는 array header의 `map` frame 절대좌표다.
- HD Map context도 `map`이다. consumer는 `map↔base_link` 변환을 정확히 한 번만 적용한다.
- camera semantic은 입력 image header를 보존하고, BEV/lane은 동기화된 source stamp를 이어받는다.
- traffic-light의 현행 String 두 토픽에는 header/source stamp가 없다. stale timeout은 node 내부
  monotonic time뿐이므로 typed additive 계약이 필요하다.
- 설정·nominal 기준은 RGB `20/20/20/10 Hz`, VLP16 설정·소비 loop `15 Hz`, GPS `5 Hz`,
  IMU `50 Hz`다. local 실측은 VLP16 완성 scan 약 `12 Hz`, IMU 약 `42~44 Hz`였으며
  camera fused BEV 상한은 `10 Hz`다.

## 선택 모델

| ID | 선택 | 크기 | SHA-256 | 경로 |
|---|:---:|---:|---|---|
| `camera_twinlite_v6` | yes | 8141406 | `a26311b3af4e92880f632509eff3c81defc1b95135f5b463c7a08e5b6e23d24b` | `artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt` |
| `traffic_light_resnet18` | yes | 44795403 | `70ddaf222e7bc6ff2b4748c4b07c1b24dec72a2671f39bc18e0b243ddd93c0d5` | `src/traffic_light/best.pt` |
| `lidar_camera_yolo11m` | yes | 40684120 | `d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95` | `src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m.pt` |
| `lidar_camera_yolo11m_duplicate` | no | 19155482 | `6cf89c17d9a1122b9b6f3f6abe7574bf3f80cc4653e549d1bfd102ec8e421bf9` | `src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m(1).pt` |

현재 모델 파일은 모두 tracked돼 있지만 목표 정책은 실행 image 밖 immutable artifact + SHA manifest다.
경로만 바꾸거나 같은 이름의 weight를 덮어쓰지 않는다.

## 반드시 해결할 통합 위험

- **high · `camera_calibration_v2_provenance`:** operational three-view lane launch and cameras_v2.yaml name v2 while bridge deployment baseline is sensor set v3  조치: prove geometry parity or create a v3-named calibration before changing runtime default
- **critical · `lidar_stale_drivable_mask`:** Main loads ../../../link_node/out/road_mask and map id waize_katri_2025-10-23 instead of the final asmc_hd_map contract  조치: replace with /hd_map vector/grid consumer and run object-filter parity
- **high · `nested_message_and_morai_duplicates`:** Main depends on nested Msg and a second morai_msgs package  조치: migrate TrackedObject/Yolo messages additively to asmc_msgs and pin the outer morai_msgs dependency
- **high · `traffic_light_unheadered_output`:** state/detail are std_msgs/String without source timestamp, validity, or typed confidence  조치: define typed additive output and retain String compatibility until planning cutover
- **medium · `lidar_legacy_side_effects`:** Main relays /camera/image/compressed and publishes /ctrl_cmd_0 in addition to perception results  조치: separate perception ownership; preserve relay compatibility only while YOLO consumer requires it
- **medium · `lidar_legacy_sensor_rate_provenance`:** Main/setting.yaml names sensor set v1 and camera/GPS 30 Hz although the bridge baseline is v3 at 20/5 Hz; LiDAR loop is correctly 15 Hz  조치: make bridge topics authoritative and validate queues/timing with measured v3 rates before moving Main
- **medium · `bag_profiles_v2_provenance`:** deployment-allowed camera_raw, lidar_raw, and full_raw profiles still declare sensor set v2 while the bridge deployment default is v3  조치: add explicit v3 recording profiles or a reviewed sensor-set override before using these profiles as deployment evidence
- **medium · `mixed_runtime_and_offline_tools`:** drivable_bev and data_collection contain runtime nodes and offline dataset/audit tools together  조치: move only offline entrypoints to tools/perception after import and bag parity
- **low · `duplicate_unselected_yolo_weight`:** yolo11m(1).pt is tracked but no audited launch selects it  조치: remove only after artifact provenance review

특히 LiDAR `Main`의 구 `link_node` road mask는 완성된 `/hd_map/*`와 다른 지도 계약이다.
P4-A에서는 사실을 기록만 했고, 필터 결과 parity 없이 경로를 즉시 바꾸지 않았다.

## Oracle 금지선

- MORAI Capture Mode or gRPC truth
- Semantic/Instance GT camera or LiDAR
- /Object_topic or simulator object truth
- simulator TrafficLight status
- disallowed global pose during GPS-shadow operation
- GT-derived masks, pseudo-GT, or map overlays used as live perception input

이 입력들은 `privileged_gt=true`, `deployment_allowed=false`인 offline profile에서만 사용할 수 있다.
기록 profile이 있다는 이유로 본선 runtime 입력으로 승격되지 않는다.

## P4 완료 gate

- canonical asmc_perception launch reproduces audited input/output topics or supplies an explicit compatibility remap
- all geometric outputs keep the audited frame and source-stamp policy
- selected model paths and SHA-256 values are recorded in the deployment artifact manifest
- raw/result bag replay demonstrates topic, type, geometry, rate, staleness, and latency parity
- traffic-light typed replacement is additive until every consumer leaves String topics
- lidar tracked-object messages move to asmc_msgs without silent field/unit/frame changes
- nested morai_msgs and Msg are removed only after clean build and zero consumer references
- learning and learning_by_cheating are removed only after blocking external reference count reaches zero and BEV/map replacement parity passes
- vendor Velodyne packages remain external/vendor dependencies rather than being absorbed into asmc_perception
- oracle profiles remain deployment-blocked

## 재생성·검증

```bash
python3 tools/audit/build_perception_runtime_baseline.py --check
python3 -m unittest tools/audit/test_perception_runtime_baseline.py
```

동일 source에서 두 번 생성한 결과는 byte-identical해야 한다. package/topic/model/bag/message 기준이
누락되거나 선택 모델 SHA가 달라지면 검증은 실패한다.

## 다음 작업

P4-B에서 먼저 canonical `asmc_perception` façade와 compatibility launch를 만들고, traffic-light typed
출력 및 LiDAR tracked-object의 `asmc_msgs` 이관 계약을 additive하게 결정한다. 기존 source의 물리 이동과
삭제는 그 뒤 bag replay parity를 통과한 단위별 PR에서 수행한다.
