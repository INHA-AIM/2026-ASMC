# LiDAR 파이프라인 — Main 이식본과 이관 검증

현재 구조·이름·식별 계약은 [첫 단위 정리](structure-contract-refactor.md)와 [이전표](name-mapping.md)가 정본이다. 아래 PR #103 수치·실패와 비교 기준선은 당시 근거를 보존한다.

**2026-10-25 현행:** PR #108은 main `ab8db094`에 병합됐다. 새 경로의 발행 배열에는 HD맵·상부
전달 필터가 기본 적용된다. 판제는 아직 기존 Main을 쓰며 기존 기능 보존 이식·공용 속도/상태·판제 직접 소비는
[현재 실행 기준선](planning-runtime.md)에서 구현했다. 다음은 실제 소비 인수와 구 패키지 폐기다. 광범위한
리팩토링·3뷰 확대보다 이 기반을 먼저 진행한다.

nested `Main`의 common-geometry LiDAR 처리를 `asmc_lidar` 안에 동작 보존 방식으로 옮긴
**검증용 이관 검증 노드**다. 기준은 Tracker 수정이 반영된 Main `8f93c22e`
(`enable_common_geometry=true`)이다. 판제는 계속 기존 Main의 `/last_lidar`를 소비한다.
이 노드는 어떤 기존 launch에도 포함되지 않으며 `/migration/lidar/*`에만 발행한다.
운영 전환·기본값 승격·실시간 성능 수락은 이 문서의 범위가 아니다.

## 2026-10-23 입력 확인과 L1 연결

현재 `/Ego_topic`은 사용자 UI의 허용 CompetitionInfoPublisher(9088→9099)를 받는 호환 이름이다.
[실측 기록](../../../docs/records/bridge/2026-10-23-competition-channel-audit.md)에서181B packet과
현재 bridge의 m/s 변환을 확인했다. 별도9798은 Detail 형식으로 거부되므로 이름만 보고
`/Competition_topic`으로 바꾸지 않는다. 주행·후진 부호는 이번 정차 시험으로 검증하지 않았다.

이 문서의 구현/기존 검증은 유지한다. [L1 목표](migration-design.md)와 차이가 남은 사항은
속도 없이 local 형상 유지, XYZ norm의 signed x 전환, 공개 속도/운동 상태,
실시간·소비 전환이다. atomic 관측 ID는 PR #104, HD맵·상부 전달 필터는 PR #108에서 구현했다.
단위별 상태는 [작업표](migration-tasks.md)를 따른다.

## 현재 상태와 제한

- 구현 완료: 독립 LiDAR pipeline(아래 처리 흐름)과 전방(`camera_front`) class fusion.
- 검증 완료: roll·pitch 수평화와 local/map 출력. 일반·터널 bag replay가 Main `8f93c22e` 기준선과 bit 단위로
  같았고, GPS 무효·중단·복구와 IMU stale·무효를 실제 노드 rostest로 시험했다.
- 기존 Main과 판제는 계속 기존 실행 경로(`/last_lidar` 등)를 쓴다. 이 패키지는 그 경로를 바꾸지 않는다.
- 새 노드는 명시적으로 실행해야 하며 `/migration/lidar/*`만 발행한다.
- PR #105의 Main MotionClassifier는 진단 전용 복사본으로 연결했다. 기존 판제용 최종 속도·4단계 상태를 공용 추적 관측에 연결했다. 유효 근거와 map 가용성을 검사하며 새 정확도 개선을 뜻하지 않는다.
- PR #108은 v7 HD맵·높이 ROI·상부 전달 필터를 추가했다. 구 known-static·structure catalog·relevance와 OBB clipping을 그대로 이식한 것은 아니다([편입 경계](filter-migration.md)).
- 웹 화면은 별도 `view.launch`, 단계 샘플은 `lidar.launch debug:=true`로 요청한다. [웹 모니터](web-monitor.md)를 따른다.
- 상대속도 validity는 false이며 절대 적용 추정값·운동 validity는 유효 map/계산 근거에 따른다. UNKNOWN의 수치0은 정지 확인이 아니다.
- 실제 카메라 bag에서의 fusion 성능과 실시간 처리 성능은 수락하지 않았다.
- TunnelFilter는 기본 OFF이며 ON 성능은 수락하지 않았다.

## 기준선과 저장소 Main과의 의도적 차이

`8f93c22e`는 이 브랜치 기준(`d7aae120`)에 Main Tracker yaw 수정 1개(`fix/lidar-tracking-frame-yaw`)를 더한
커밋이다. main에는 병합되지 않았으며 **수정된 Main의 비교 기준선**으로만 쓴다. shadow에는 이 수정이 이미
들어 있으므로 이 이식은 Main Tracker 수정 PR의 선행 병합을 요구하지 않는다. 기존 Main은 동결하며 코드를 바꾸지 않는다.

그래서 저장소 안의 Main(`d7aae120`)과 shadow는 다음이 의도적으로 다르다.

- matched track: shadow는 현재 scan에서 잰 OBB를 유지하고 KF 상태만 새 tracking frame으로 재표현한다.
  저장소 Main은 그 OBB의 yaw를 한 번 더 회전한다(yaw 이중 회전).
- predicted track: shadow는 center·8코너·yaw·Z를 한 번 재표현한다. 저장소 Main은 yaw만 회전하고
  center·8코너·Z는 이전 scan frame에 남긴다(yaw와 코너 불일치).
- 그 결과 OBB center·yaw·코너·Z와 이를 쓰는 연관(ShapeAgreement)·ID가 저장소 Main과 다를 수 있다.

Main 공통의 의도적 차이는 따로 있다. 운영 출력은 제외했고([이식 경계](#이식-경계)), 카메라 경로는 Receiver 기반으로
판정한다([카메라 fusion](#카메라-fusion)). 범위 밖 위경도 GPS는 무효로 처리한다(Main은 GeographicLib 예외로 종료).
아래 줄 번호는 `8f93c22e` 기준이다.

## 처리 흐름

```text
/velodyne_points ─┐                     /imu, /Ego_topic(CAN) → scan 시각 local 입력(≤120ms, 최대 50ms 대기)
                  ▼                     /gps → sanity·HD맵 원점 검사 → bounded map pose(300ms, 3회 복구)
 ROI·자차 제거 → Voxel 0.1m → roll·pitch 수평화(행렬을 scan Context에 저장)
 → IMU 수평 지면 기준 높이 ROI(2.5m) → RANSAC 지면 제거 → [실험 TunnelFilter, 기본 OFF] → 적응 군집 → centroid·L-shape OBB
 → CA Kalman/Hungarian 추적(local) → 전방 카메라 class fusion → typed local/map 구성
 → HD맵·상부 전달 필터(원본 track 보존) → 배열·관측 발행
```

PR #103 이식의 지면·군집·추적은 Main 기준선과 대조했다. 이후 PR #108의 높이 ROI·전달 필터는
별도 변경이며 당시의 bit 동등성 근거를 현재 발행 배열 전체의 동등성으로 확대하지 않는다.
local 처리는 GPS와 독립이다. GPS 정상/음영/복구는 map 가용성에만 영향을 주며 tracker를 reset하지 않는다.

## 이식 경계

아래 표는 **PR #103 당시 이식 범위**다. 현재 추가된 전달 필터·속도/운동 연결은
[현행 기능 대조](cutover-foundation.md#구-main과-새-패키지의-기능-대조)를 따른다.

| 포함 (Main 동작 보존) | 제외 (이번 경계 밖, 사유) |
|---|---|
| ROI·Voxel·수평화·RANSAC·군집·OBB·consolidation·Hungarian·CA KF·ID 생애주기·OBB 안정화 | 판제 후처리: drivable road mask, known static catalog, static structure exclusion, planning relevance/confirmation (`PlannerInterface/*`) |
| Tracker 수정 8f93c22e: matched는 현재 OBB 유지·KF 상태만 재표현, predicted는 center·8코너·yaw·Z를 한 번 재표현 | MotionClassifier(map 기반 동정적 판정): typed 출력의 `dynamic_valid`가 false라 소비되지 않음 |
| SensorPoseHistory(common ns 경로)·GPS sanity·BoundedPoseHistory map pose | legacy(pitch-only float, map 중심 연관) 경로, `/last_lidar`·UDP·marker·debug cloud, `/ctrl_cmd_0` |
| FrameBoundaryPublisher의 typed 배열 의미(status·reason·frame·validity) | 3뷰 fusion(좌·우 카메라), 카메라 debug image |
| 전방 fusion 정책(±50ms, 200ms 대기, unique, IoU 0.25, 5/3/3 class 유지) | 속도 정확도 개선, 새 ego 보상 |

bag 비교의 동등성 경계는 **추적 결과와 typed local/map 배열**이다.
`MotionClassifier`와 판제 필터 이후의 Main 출력(`/last_lidar`)은 비교하지 않는다.

## 원본 → 이식 위치

| Main (8f93c22e, `src/lidar+sensorfusion/src/Main/src/`) | asmc_lidar |
|---|---|
| `Global/Global.hpp` 필요한 구조체만 (`LiDARObjectInfo` 등) | `include/asmc_lidar/core/types.hpp` (`TrackedObject`, 필드명 유지) |
| `Lidar/RoiVoxel/RoiVoxel.cpp` | `src/preprocessing/roi_voxel.cpp` |
| `Lidar/ImuRansac/ImuFusion.cpp` (common) | `src/preprocessing/leveling.cpp` |
| `Lidar/ImuRansac/Ransac.cpp`, `GroundEvidence.hpp` | `src/preprocessing/ground_filter.cpp`, `pipeline/ground.hpp` |
| `Lidar/Clustering/Clustering.cpp` | `src/detection/clustering.cpp` |
| `Lidar/Tracking/Tracking.cpp` `obb_util` | `src/detection/obb.cpp` |
| `Lidar/Tracking/Tracking.cpp` 검출·연관, `Lidar/FrameFilter/FrameFilter.cpp` | `src/detection/detections.cpp`, `src/tracking/tracker.cpp` |
| `Lidar/Kalman/Kalman.cpp`, `Lidar/HungarianAlg/HungarianAlg.cpp` | `src/tracking/kalman.cpp`, `src/tracking/hungarian.cpp` |
| `Geometry/FrameBoundary.cpp` (scan Context·tracking frame) | `src/geometry/scan_frame.cpp` (`ScanFrame`) |
| `Lidar/LidarProcess/LidarProcess.cpp`, `main.cpp` TemporalThread | `src/pipeline/pipeline.cpp`, `src/ros/runtime.cpp` |
| `Parser/SensorPoseHistory.cpp`, `Parser/GpsSanityFilter.cpp`, `Parser/Parser.cpp` 콜백 계산 | `src/geometry/pose_history.cpp`, `src/geometry/gps_ingest.cpp` |
| `Geometry/FrameBoundaryPublisher.cpp` | `src/ros/typed_output.cpp` |
| `SensorFusion/FusionCamera.cpp`, `FusionClassPolicy.hpp`, `FusionTimeMatcher.hpp`, `FusionClassState.hpp`, `FusionLiDAR.cpp` `UpdateAABB` | `src/fusion/front_camera_fusion.cpp`, `pipeline/fusion_policy.hpp`, `src/ros/runtime.cpp` |
| `Lidar/TunnelFilter/TunnelFilterCore.{hpp,cpp}`, `../test/test_tunnel_filter_core.cpp` (PR #102) | `src/experimental/tunnel_filter/Lidar/TunnelFilter/` 와 `test/experimental/` 에 byte 동일 복사 (blob `b1016c35`, `3aec8db8`, `c54fec48`) |

각 이식 함수 앞에 `Ported from Main <file>:<lines> (8f93c22e)` 주석이 있다. 전역 상태는
`Pipeline`·`Tracker`·`ScanFrame`·`PoseHistory` 멤버가 됐다.

## 출력 토픽·frame·상태

| 토픽 (기본값, `~*_topic` 또는 remap으로 변경) | 타입 | 내용 |
|---|---|---|
| `/migration/lidar/objects_local` | `asmc_msgs/LidarObjectArray` | frame `base_link`(후륜축), 관측 scan의 source stamp |
| `/migration/lidar/objects_map` | `asmc_msgs/LidarObjectArray` | frame `map`, 같은 객체·ID를 관측 시각 pose로 변환 |
| `/migration/lidar/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1Hz. 처리·무효·덮어쓴 scan 수, 처리시간 p50/p95, map·카메라 사유 |
| `/migration/lidar/trace` | `std_msgs/String` | 구독자가 있을 때만 scan별 JSON ([형식](scan-trace-schema.md), debug 필드 제외) |

- `OBSERVED`/`OBSERVED_EMPTY`: 정상 처리. 두 배열 모두 `INPUT_INVALID`(객체 0)인 경우는
  (a) scan 시각의 IMU/CAN이 없거나 stale·무효(reason=pose 사유), (b) 점 0개 scan
  (reason=`empty_or_invalid_lidar_input`) — (a)(b)는 tracker를 진행하지 않는다 — 그리고
  (c) 처리 후 scan 시각·수평화가 무효이거나 발행할 객체 하나라도 치수·기하가 무효인 경우다
  (`leveling_transform_invalid`, `invalid_object_dimensions` 등; tracker는 이미 갱신됨). Main과 같다.
  map은 pose가 무효면 `POSE_UNAVAILABLE`과 사유만 보내고 객체는 0개다. 센서가 멈추면 새 stamp의
  빈 관측을 만들지 않는다.
- 발행 객체는 관측된 track(`miss_count=0`)뿐이다. 중심·8코너·축은 저장된 수평화의 역변환 →
  장착 변환 → (map) 유효 pose를 각각 한 번 적용한 결과다(`asmc::lidar::transform`).
- `relative_velocity_valid`는 false다. 구 MotionClassifier 적용 속도를 공개 절대속도 추정값으로
  연결하고4단계 운동 상태와 함께 제공한다. GPS 무효에서는 둘 다 invalid다([현재 의미](planning-runtime.md#공용-메시지와-속도의-의미)).
- TF는 기본 OFF다(`~publish_tf`). 켜면 `base_link → lidar_pipeline/lidar_leveled`로 보내며
  Main의 `lidar_leveled`와 child를 공유하지 않는다.
- 출력 이름(param·remap 적용 후)이 `/last_lidar`, `/lidar_nogps`, `/YOLO/BBox`, `/tf`, `/tf_static`,
  `/perception/lidar/`·`/planning/` 아래 이름, 또는 노드의 입력 토픽이면, TF child가 `lidar_leveled`·
  `base_link`·`map`·`velodyne`이면 `~allow_operational_output:=true` 없이는 노드가 시작을 거부한다.

## 카메라 fusion

- 입력: `/perception/camera/objects/observations` (`asmc_msgs/CameraObjectObservation`),
  `/perception/camera/objects/status` (`asmc_msgs/InferenceStatus`). 기존 `camera::Receiver`가
  수신·검증하고, fusion은 그 버퍼의 **`camera_front` view만** 읽는다. 좌·우 view는 후보가 아니며
  기다리지도 않는다. 모드 서비스는 호출하지 않는다. `/YOLO/BBox`와 `Msg`는 사용하지 않는다.
- 시간 정책(Main): scan과 영상 stamp 차이 ±50ms 이내만 매칭, 다음 영상이 오지 않으면
  주입된 decision clock으로 200ms 후 drop, 같은 관측은 한 번만 사용(unique), 대기 scan 최대 32개
  (첫 관측 전 2개). LiDAR 처리는 카메라를 기다리지 않는다.
- Receiver의 보관(뷰당 8개, 350ms)과 상태 신선도(350ms)는 매칭 허용값과 별개 설정이다.
- 영상-scan 시간차의 pose 보정은 하지 않는다(`FusionReport.match_offsets_ns`로 기록).
- 카메라가 없거나 STANDBY·stale·invalid·세션/세대 변경이면 class 근거를 폐기(unknown)하고
  LiDAR 객체와 기하는 그대로 유지한다. 카메라 미검출로 객체를 지우지 않는다.

Main 대비 의도적 차이: WallTime gate 대신 Receiver 상태·view 사유와 주입 clock(노드는 fusion 직전의
현재 시각, Main도 fusion 호출 시각 사용), decode한 JPEG 대신 보정 파일의 영상 크기(1280×720),
상태 수신 전 fail-closed(Main은 fail-open), Receiver가 frame 전체를 거부한 경우(`invalid`, `bad_box`,
`calibration_mismatch`, `identity_conflict`) 즉시 비활성·class 초기화(Main은 어댑터가 문제 box만 버리거나
frame을 보내지 않아 0.35초 후 비활성), `successful_batches`/`source_image_stale` 미검출,
보관량 8개·350ms(Main 128개). ACTIVE 상태에서 전방 관측이 아직 없으면(`missing`) Main처럼 활성·관측 없음으로
scan 2개까지 대기시키고, 관측이 있던 뒤의 `missing`(세션/세대 전환으로 Receiver가 view를 비움)은 즉시 epoch를 바꾼다.

## 실험 TunnelFilterCore

`experimental.tunnel_filter.enabled` 기본 false다(노드 `~experimental_tunnel_filter`, launch 인자,
replay `--tunnel-filter on|off`를 주면 그 값이 설정 파일보다 우선한다). OFF이면
stage를 만들지 않고 호출하지 않는다. ON이면 GPS와 무관하게 모든 scan에서 RANSAC 뒤·군집 앞에
core를 호출하고, core가 천장/벽으로 표시한 voxel 점만 지면 제거 결과에서 뺀다.
`IsGpsDenied`/`ShouldRunExperiment`는 사용하지 않는다. ON의 필터 성능은 수락하지 않았다.

## 실행

```bash
# 빌드 (outer workspace만; nested Main/Msg 불필요)
source /opt/ros/noetic/setup.bash
cd <repo> && catkin_make -DCATKIN_WHITELIST_PACKAGES=""
source devel/setup.bash

# 이관 검증 노드 (기존 bridge/카메라가 발행 중인 ROS 입력을 구독)
roslaunch asmc_lidar lidar.launch namespace:=/migration/lidar
# LiDAR 단독 (카메라 구독 없음)
roslaunch asmc_lidar lidar.launch fusion:=false
# pitch-only 수평화 / 실험 TunnelFilter ON (인자를 주지 않으면 config/lidar_pipeline.yaml 값)
roslaunch asmc_lidar lidar.launch lidar_level_roll:=false
roslaunch asmc_lidar lidar.launch experimental_tunnel_filter:=true

# local/map 상태 확인
rostopic echo -n1 /migration/lidar/objects_local/status
rostopic echo -n1 /migration/lidar/objects_map/reason
rostopic echo /migration/lidar/diagnostics

# bag 재생 (ROS master 불필요, 지정한 토픽만 읽음)
rosrun asmc_lidar lidar_replay --bag in.bag --output out.jsonl \
  --config $(rospack find asmc_lidar)/config/lidar_pipeline.yaml --profile roll_pitch
```

기존 실행으로 돌아가려면 이관 검증 노드만 종료한다. 이 노드는 운영 토픽·TF·차량 제어 서비스를 소유하지
않으므로 기존 Main 실행 구성은 변하지 않는다.

## 시험

```bash
catkin_make && catkin_make camera_receiver_ros_test   # 기존 rostest gtest는 EXCLUDE_FROM_ALL
(cd build/asmc_lidar && ctest --output-on-failure)
```

이식 모듈 gtest 18개(TunnelFilterCore 27개 사례 포함), 실제 노드 rostest(`test/ros/lidar_pipeline_node.test`:
GPS 무효·중단·복구, IMU stale·무효, 카메라 정상·empty·invalid·STANDBY·세션 변경·중복·중단, 격리·guard;
`../asmc_camera/config/cameras_v3.yaml`이 있을 때만 등록), 격리 계약 시험(`test/pipeline_contract_test.py`)이다.
합성 카메라 관측은 인터페이스·동기화 시험이며 YOLO 정확도 시험이 아니다.
PCL·pcl_conversions·GeographicLib·morai_msgs가 없는 환경에서는 pipeline target을 경고와 함께 건너뛰고
기존 target만 빌드한다(기본값). CI는 이 생략을 허용하지 않는다.

- `ci.yml`은 의존성을 설치하고 `-DASMC_LIDAR_PIPELINE_REQUIRED=ON`으로 빌드한다. 의존성이나 rostest 보정 파일이 없으면
  configure가 실패한다. `ctest -T Test` 뒤 `test/pipeline_ci_check.py`가 두 실행파일, 필수 시험 31개의
  등록·통과·결과 갱신·최소 사례 수, skip 0, catkin export 불변을 확인하며 하나라도 어긋나면 실패한다.
- `planning-integration.yml`의 `validated-lidar-build`는 shadow가 활성인 `asmc_lidar` 위에서 nested Main을 빌드·시험하고,
  Main 링크에 shadow 라이브러리가 없는지 같은 스크립트(`--dependent-build`)로 검사한다.

CI와 같은 확인을 로컬에서 하려면 다음을 실행한다.

```bash
catkin_make -DASMC_LIDAR_PIPELINE_REQUIRED=ON && catkin_make camera_receiver_ros_test
(cd build/asmc_lidar && ctest --output-on-failure -T Test --no-compress-output)
python3 src/asmc_lidar/test/pipeline_ci_check.py --devel devel --build build/asmc_lidar --results build/test_results/asmc_lidar
```
