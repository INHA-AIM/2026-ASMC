# asmc_lidar — 좌표·시간 기반과 카메라 입력 인계

**2026-10-23 이관:** 안승현이 [L0–L8 작업표](docs/migration-tasks.md)를 직접 진행하며 손재호 개발은 병행한다.
카메라 인계는 [현재 연결·실행·구현 경계](../../docs/handoffs/camera-to-lidar.md).
`config/camera_objects.yaml`은 실제 카메라 관측 계약/보정/라벨 설정을 참조하고 교차 시험으로 검사한다.
관측 수신·뷰별 버퍼·보정/투영 인터페이스를 제공한다. 전방(`camera_front`) fusion은 shadow 노드에 이식했으며 3뷰 객체 매칭·fusion은 손재호의 후속 구현이다. Main의 독립 YOLO는 제거하고 외부 카메라 전방 어댑터만 사용한다.


현재는 **좌표·시간 라이브러리, 공용 카메라 수신 컴포넌트와 계약 시험**, 그리고 Main LiDAR
처리(ROI→Voxel→roll·pitch→RANSAC→군집→OBB→추적→전방 fusion→typed local/map)를 옮긴
**검증용 shadow 노드와 bag replay**다. [Shadow 파이프라인](docs/shadow-pipeline.md)에 이식 경계·원본 대응·
토픽·실행 방법을 정리했다. 실제 주행에는 기존 `lidar+sensorfusion` Main을 계속 사용한다.
shadow 노드는 `/asmc_lidar/shadow/*`에만 발행하며 어떤 기존 launch에도 포함되지 않는다.
구현·검증 상태, 미수락 항목과 저장소 Main과의 의도적 차이는 [현재 상태와 제한](docs/shadow-pipeline.md#현재-상태와-제한)에 있다.

[카메라 입력 API·공용 상태 전환·직접 시험](docs/camera-input.md)을 카메라 연결 시작점으로 사용한다.

2026-10-08 B1a의 GPS·map·수평화·카메라 투영·추적 경계와 담당별 후속 작업은
[B1a 개선 과제](docs/migration-tasks.md)에 정리했다.
공통 기반은 `Main`의 opt-in `Geometry/` 어댑터에 연결했다. 검증 상태와 직접 launch는
[좌표·GPS 경계 인계](../../docs/records/lidar/lidar-camera-frame-boundaries.md)에 있다.
전체 이전·터널 필터 운영 수락과 기본값 승격은 별도다.

담당 작업과 완료 조건은 [LiDAR 인계 문서](../../docs/handoffs/lidar-migration.md),
카메라와의 역할 분담은 [통합 로드맵](../../docs/architecture/consolidation-roadmap.md)에 있다.

## 목표 구조와 현재 경계

[L1 설계](docs/migration-design.md)에 입력·pose·속도·출력과 PR #103 이후 L2 인수 경계를 정리했고
[L1 manifest](../../config/baselines/lidar_migration_l1_v1.yaml)에 source/설정/입력 SHA와 검증 범위를 기록했다.
설계 결정이며 새 LiDAR 실행이 구현된 상태는 아니다.

목표 책임은 `geometry / preprocessing / detection / tracking / fusion / filtering / ros`다.
아래 기존 코어를 재사용하며 실제 폴더는 구현할 때 만든다. 빈 골격·새 LiDAR launch는 이번에 추가하지 않는다.
새 실행의 출력은 `/migration/lidar/...`로 격리하고 기존 운영 입력은 유지한다.
정식 이름·전환·복구는 [통합 인계](../../docs/handoffs/lidar-migration.md#시험-namespace와-전환)를 따른다.

## 남긴 구성

| 경로 | 역할 |
|---|---|
| `include/asmc_lidar/frame_time.hpp`, `src/frame_time_core.cpp` | 실제 수평화 역변환·장착 변환·관측시점 pose 보간·속도 의미 변환 |
| `include/asmc_lidar/projection.hpp`, `src/projection.cpp` | 장착 회전을 고려한 수평화와 관측별 합성 카메라 투영 |
| `include/asmc_lidar/bounded_pose.hpp`, `src/bounded_pose.cpp` | 별도 300ms 제한 GPS 위치 보정·무효 barrier·3회 복구 API |
| `include/asmc_lidar/imu_selection.hpp` | 이미 도착한 IMU 중 scan 시각 이전의 최신 유효 표본 선택, 누락/stale 진단 |
| `config/camera_objects.yaml`, `test/camera_objects_contract_test.py` | 카메라 관측·보정·라벨 참조와 교차 검사; 실제 API/시험은 camera-input 문서 |
| `config/autonav_v3.yaml` | 장착 변환의 출처/SHA와 값. 후속 ROS 어댑터에서 검증·로딩할 참고 설정 |
| `camera_observations.hpp`, `camera_receiver.hpp`, `camera_calibration.hpp` | ROS 독립 버퍼·ROS 수신·보정 로더·투영 경계 |
| `examples/camera_observation_probe.cpp` | 모드/제어/객체 출력 없이 diagnostics만 제공하는 연결 예제 |
| `test/` | 중심/8코너·시간·pose·속도·IMU 선택의 회귀시험 |
| `include/asmc_lidar/pipeline/`, `src/{preprocessing,ground,clustering,obb,tracking,pose,pipeline,fusion,experimental}/` | Main common-geometry 경로의 동작 보존 이식(정적 라이브러리, 외부 export 없음) |
| `include/asmc_lidar/shadow/`, `src/ros/`, `tools/shadow_replay.cpp`, `launch/`, `config/shadow_pipeline.yaml` | shadow 노드·replay·설정. 노드와 replay는 같은 runtime을 호출 |
| `src/experimental/tunnel_filter/`, `test/experimental/` | PR #102 TunnelFilterCore byte 동일 복사본과 27개 시험. 연결 stage는 기본 OFF |

`asmc::lidar::transform()`은 local 결과와 map 가용성을 분리한다. 호출자는 실제 적용한
수평화 행렬과 장착 변환을 넣어야 한다. `PoseHistory`는 동일 clock의 도착 완료 pose만
최대 100ms 간격에서 보간하고 외삽하지 않는다. map 기준 높이와 pose 출처 검증은
호출자의 책임이며, 이 라이브러리가 GPS/차량 상태를 구독하거나 추정하지는 않는다.
`SelectImu()`의 `newer_invalid_seen`도 진단에 전달해야 한다.

점 변환 함수는 XYZ/intensity를 보존한다. 후속 point-cloud 어댑터는 ring/time 등 추가
필드와 원본 index 계보를 별도로 보존해야 한다. 점별 시간이나 deskew 기능은 없다.

## 구조물 처리 확장 지점 — 아직 미구현

L8 후속 후보는 지면 뒤·군집 앞에 구조물 근거 추출을 추가하고, 군집 뒤에 벽/객체 혼합 여부를
검사한다. 동일 경로를 모든 구간에 사용하며 GPS나 미션으로 지면/군집을 전환하지 않는다.
구체적인 단계·내부 결과 요구·실패 사례·검증 순서는
[모드 없는 구조물 분리 계약](../../docs/handoffs/lidar-migration.md#모드-없는-구조물-분리--손재호의-후속-구현-계약)을 따른다.
같은 문서의 [참고 구현 검수](../../docs/handoffs/lidar-migration.md#참고-구현-검수와-현실적인-첫-제안--2026-10-05)에
PCL 기반 첫 실험, depth_clustering의 조건부 비교와 소스 주의점, 후순위 논문을 정리했다.
읽기 검수만 완료했으며 새 backend가 빌드·실행됐다는 뜻은 아니다.

- 객체 후보는 OBB/추적, 구조물은 관측된 충돌 경계, 미확정은 보존 대상으로 나눈다.
- 평평함·긴 OBB·정지 상태·카메라 미검출 하나만으로 억제하지 않는다. 벽과 차량이 섞인
  큰 OBB를 통째로 삭제하는 대신 원본점 대응과 돌출부/연결 근거를 검사한다.
- 근거 추출과 실제 억제 마스크를 분리한다. 새 억제는 OFF로 시작하고 객체 보존 회귀와
  구조물 경계 소비 계약을 확인한 뒤 별도 승격한다. 빈 객체 배열은 자유공간을 뜻하지 않는다.
- `src/structure` 등은 구현할 때 만든다. 현재 구조물 분류기·설정 스위치·ROS 경계 토픽은
  없으며, 기존 `LidarObject*`를 구조물 경계 메시지로 임의 재사용하지 않는다.

## 빌드·시험

Noetic 컨테이너의 저장소 루트에서 실행한다. 기존 build 산출물을 재사용하지 않고
별도 build/devel 경로로 현재 소스만 검사한다.

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws
catkin_make --build /tmp/asmc_lidar_handoff_build \
  -DCATKIN_DEVEL_PREFIX=/tmp/asmc_lidar_handoff_devel \
  --only-pkg-with-deps asmc_lidar -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make --build /tmp/asmc_lidar_handoff_build \
  -DCATKIN_DEVEL_PREFIX=/tmp/asmc_lidar_handoff_devel camera_receiver_ros_test -j2 -l2
(cd /tmp/asmc_lidar_handoff_build/asmc_lidar && ctest --output-on-failure)
source /tmp/asmc_lidar_handoff_devel/setup.bash
python3 src/asmc_msgs/test/test_message_serialization.py
```

기존 라이브러리(`asmc_lidar_frame_time`, `asmc_lidar_camera_core`, `asmc_lidar_camera_ros`)와
`catkin_package` export는 바뀌지 않았다. shadow target은 PCL·pcl_conversions·GeographicLib·
`morai_msgs`(submodule)를 쓰는 패키지 전용 정적 라이브러리이며 의존 패키지(Main 등)에 링크되지 않는다.
이 의존성이 없으면 shadow target만 경고와 함께 건너뛴다. CI는 `-DASMC_LIDAR_SHADOW_REQUIRED=ON`으로 이 생략을
configure 실패로 바꾸고, `test/shadow_ci_check.py`로 shadow 실행파일과 필수 시험의 실제 생성·통과를 확인한다
([시험](docs/shadow-pipeline.md#시험)). `Main`·`Msg` nested workspace에는 의존하지 않는다.
수신 확인용 probe는 diagnostics만 발행하며 LiDAR 객체를 발행하지 않는다.

이전 연구 소스·보고서는 정리 전 커밋 `020f6c2`에 남아 있다. 선택적 복구는 별도
checkout에서 검토한다. 예전 `unified.launch`와 `ground_compare_live.launch`는 삭제됐다.
기존 devel에 같은 이름의 실행파일이 남아 있어도 현재 패키지의 지원 기능으로 사용하지 않는다.

## 시간 API 선택

`PoseHistory`는 같은 clock에서 최대 100ms bracketing 보간만 허용한다.
`BoundedPoseHistory`는 Main의 별도 300ms 제한 보정·GPS 정상 3회 복구 정책이다.
공통 Main은 `Observation.clock=ros`, `receipt_clock=steady`를 명시하고 source age와
수신/처리 소요시간을 서로 다른 축에서 계산한다. strict API는 mixed clock을 거부한다.
GPS stamp가 0인 무효 입력은 bounded API의 수신 시각 barrier로만 처리한다.
local 준비 조건은 유효 IMU·차량 속도이며 GPS와 독립적이다.

shadow 경로의 bag 재현기는 `asmc_lidar_shadow_replay`다(노드와 같은 runtime 호출, 지정 토픽만 읽음).
nested Main의 연속 재현기와 그 기록은
[연속 회귀와 재호 과제](../../docs/records/lidar/lidar-continuous-regression.md)를 참고한다.
