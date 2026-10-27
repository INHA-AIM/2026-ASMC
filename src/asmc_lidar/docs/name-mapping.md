# LiDAR 경로·이름 이전표

2026-10-23 구조 정리. 옛 경로는 이전 안내이며 호환 wrapper는 제공하지 않는다.

| 이전 | 현재 |
|---|---|
| `src/asmc_lidar/config/shadow_pipeline.yaml` | `src/asmc_lidar/config/lidar_pipeline.yaml` |
| `src/asmc_lidar/docs/shadow-pipeline.md` | `src/asmc_lidar/docs/pipeline.md` |
| `src/asmc_lidar/include/asmc_lidar/bounded_pose.hpp` | `src/asmc_lidar/include/asmc_lidar/geometry/bounded_pose.hpp` |
| `src/asmc_lidar/include/asmc_lidar/camera_calibration.hpp` | `src/asmc_lidar/include/asmc_lidar/geometry/camera_calibration.hpp` |
| `src/asmc_lidar/include/asmc_lidar/camera_observations.hpp` | `src/asmc_lidar/include/asmc_lidar/fusion/camera_observations.hpp` |
| `src/asmc_lidar/include/asmc_lidar/camera_receiver.hpp` | `src/asmc_lidar/include/asmc_lidar/ros/camera_receiver.hpp` |
| `src/asmc_lidar/include/asmc_lidar/frame_time.hpp` | `src/asmc_lidar/include/asmc_lidar/core/frame_time.hpp` |
| `src/asmc_lidar/include/asmc_lidar/imu_selection.hpp` | `src/asmc_lidar/include/asmc_lidar/core/imu_selection.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/clustering.hpp` | `src/asmc_lidar/include/asmc_lidar/detection/clustering.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/front_camera_fusion.hpp` | `src/asmc_lidar/include/asmc_lidar/fusion/front_camera_fusion.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/fusion_policy.hpp` | `src/asmc_lidar/include/asmc_lidar/fusion/fusion_policy.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/ground.hpp` | `src/asmc_lidar/include/asmc_lidar/preprocessing/ground.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/hungarian.hpp` | `src/asmc_lidar/include/asmc_lidar/tracking/hungarian.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/kalman.hpp` | `src/asmc_lidar/include/asmc_lidar/tracking/kalman.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/obb.hpp` | `src/asmc_lidar/include/asmc_lidar/detection/obb.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/pose_history.hpp` | `src/asmc_lidar/include/asmc_lidar/geometry/pose_history.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/preprocessing.hpp` | `src/asmc_lidar/include/asmc_lidar/preprocessing/preprocessing.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/scan_frame.hpp` | `src/asmc_lidar/include/asmc_lidar/geometry/scan_frame.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/scan_trace.hpp` | `src/asmc_lidar/include/asmc_lidar/ros/scan_trace.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/tracker.hpp` | `src/asmc_lidar/include/asmc_lidar/tracking/tracker.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/tunnel_filter_stage.hpp` | `src/asmc_lidar/include/asmc_lidar/experimental/tunnel_filter_stage.hpp` |
| `src/asmc_lidar/include/asmc_lidar/pipeline/types.hpp` | `src/asmc_lidar/include/asmc_lidar/core/types.hpp` |
| `src/asmc_lidar/include/asmc_lidar/projection.hpp` | `src/asmc_lidar/include/asmc_lidar/geometry/projection.hpp` |
| `src/asmc_lidar/include/asmc_lidar/shadow/ros_inputs.hpp` | `src/asmc_lidar/include/asmc_lidar/ros/ros_inputs.hpp` |
| `src/asmc_lidar/include/asmc_lidar/shadow/runtime.hpp` | `src/asmc_lidar/include/asmc_lidar/ros/runtime.hpp` |
| `src/asmc_lidar/launch/shadow_pipeline.launch` | `src/asmc_lidar/launch/lidar.launch` (기본 fusion) |
| `src/asmc_lidar/launch/shadow_pipeline_lidar_only.launch` | `src/asmc_lidar/launch/lidar.launch` + `fusion:=false` |
| `src/asmc_lidar/src/bounded_pose.cpp` | `src/asmc_lidar/src/geometry/bounded_pose.cpp` |
| `src/asmc_lidar/src/camera_calibration.cpp` | `src/asmc_lidar/src/geometry/camera_calibration.cpp` |
| `src/asmc_lidar/src/camera_observations.cpp` | `src/asmc_lidar/src/fusion/camera_observations.cpp` |
| `src/asmc_lidar/src/camera_receiver.cpp` | `src/asmc_lidar/src/ros/camera_receiver.cpp` |
| `src/asmc_lidar/src/clustering/clustering.cpp` | `src/asmc_lidar/src/detection/clustering.cpp` |
| `src/asmc_lidar/src/frame_time_core.cpp` | `src/asmc_lidar/src/core/frame_time_core.cpp` |
| `src/asmc_lidar/src/ground/ground_filter.cpp` | `src/asmc_lidar/src/preprocessing/ground_filter.cpp` |
| `src/asmc_lidar/src/obb/obb.cpp` | `src/asmc_lidar/src/detection/obb.cpp` |
| `src/asmc_lidar/src/pose/gps_ingest.cpp` | `src/asmc_lidar/src/geometry/gps_ingest.cpp` |
| `src/asmc_lidar/src/pose/pose_history.cpp` | `src/asmc_lidar/src/geometry/pose_history.cpp` |
| `src/asmc_lidar/src/projection.cpp` | `src/asmc_lidar/src/geometry/projection.cpp` |
| `src/asmc_lidar/src/ros/shadow_node.cpp` | `src/asmc_lidar/src/ros/lidar_pipeline_node.cpp` |
| `src/asmc_lidar/src/ros/shadow_runtime.cpp` | `src/asmc_lidar/src/ros/runtime.cpp` |
| `src/asmc_lidar/src/tracking/detections.cpp` | `src/asmc_lidar/src/detection/detections.cpp` |
| `src/asmc_lidar/src/tracking/scan_frame.cpp` | `src/asmc_lidar/src/geometry/scan_frame.cpp` |
| `src/asmc_lidar/test/ros/shadow_node.test` | `src/asmc_lidar/test/ros/lidar_pipeline_node.test` |
| `src/asmc_lidar/test/ros/shadow_node_test.py` | `src/asmc_lidar/test/ros/lidar_pipeline_node_test.py` |
| `src/asmc_lidar/test/shadow/test_runtime.cpp` | `src/asmc_lidar/test/runtime/test_runtime.cpp` |
| `src/asmc_lidar/test/shadow/test_typed_output.cpp` | `src/asmc_lidar/test/runtime/test_typed_output.cpp` |
| `src/asmc_lidar/test/shadow_ci_check.py` | `src/asmc_lidar/test/pipeline_ci_check.py` |
| `src/asmc_lidar/test/shadow_contract_test.py` | `src/asmc_lidar/test/pipeline_contract_test.py` |
| `src/asmc_lidar/tools/shadow_replay.cpp` | `src/asmc_lidar/tools/lidar_replay.cpp` |

`asmc_lidar_shadow_node` → `lidar_pipeline_node`, `asmc_lidar_shadow_replay` → `lidar_replay`.
출력 namespace는 `/asmc_lidar/shadow`에서 `/migration/lidar`로 변경했다.
이는 이관 당시 대응표다. 이후 정본화로 운영은 `/perception/lidar`가 됐고,
현재 비교 예시는 `/test/lidar`다. 구 namespace 별칭·중계는 제공하지 않는다.

이 PR의 초기 이름 `lidar_pipeline.launch`와 `lidar_only.launch`도 최종적으로
`lidar.launch` 하나로 합쳤다. 일반 launch 인자는 `fusion`이며 내부 파라미터
`enable_camera_fusion`은 그대로다. 옛 launch와 인자 alias는 제공하지 않는다.

## 내부 필드와 공용 타입

| 이전 표현 | 현재 표현 | 의미 |
|---|---|---|
| 한 `TrackedObject`의 혼합 필드 | `detection`, `tracking`, `camera` | 점군·측정 기하 / 추적 상태 / 카메라 근거 |
| `f32_OBB_Width` | `detection.obb_length_m` | yaw 방향 수평 변 |
| `f32_OBB_Height` | `detection.obb_width_m` | yaw 수직 방향 수평 변. 수직 높이가 아님 |
| `OBB_Corners_Body` | `detection.obb_corners_level` | 수평화 센서 좌표의 8코너 |
| `ScanInput.generation`, `Context.input_generation` | `scan_sequence` | scan 순번. 새 `generation`은 처리 초기화 세대 |
| `Msg/TrackedObject*` 운영 wire | 현재 유지 | 새 `LidarObject*`와 필드·의미가 달라 이름만 치환하지 않음 |
| `asmc_msgs/LidarObject*` | 이름 유지 | 기존 local/map typed 객체·상태, MD5 보존 |
| 신규 `asmc_msgs/LidarObservation` | 원자적 관측 계약 | 배열 + 생산자 UUID·세대·scan 순번·설정/보정 ID |

`detection/detections.hpp`가 검출 생성 선언을 소유한다(이전 `pipeline/tracker.hpp`).
기존 Main의 공유 헤더 include와 Context 필드 참조도 새 경로로 바꿨으며 계산·운영 출력은 바꾸지 않았다.

## PR #105 웹 모니터 통합

| 이전 | 현재 |
|---|---|
| `launch/monitor.launch` (처리+화면) | `launch/lidar.launch debug:=true`와 별도 `launch/view.launch` |
| `pipeline/motion_display.hpp`, `motion_evidence.hpp` | `tracking/`의 같은 파일명 |
| `shadow/debug_publisher.hpp` | `ros/debug_publisher.hpp` |
| `TrackedObject.display_motion` | `TrackedObject.tracking.display_motion` (진단 전용) |

기존 HTTP·binary/JSON v1 형식은 보존하며, JSON의 역사적 `shadow_nodes` 키는 실행 이름 alias가 아니다.

## B4 정본화 대응

| 이전 이름·경로 | 현행 |
|---|---|
| `tracking/motion_display.hpp/.cpp` | `tracking/motion_classifier.hpp/.cpp` |
| `TrackState.display_motion`, `Pipeline.display_motion_` | `TrackState.motion`, `Pipeline.motion_classifier_` |
| `/last_lidar` + `/lidar/motion_debug` 결합 | `/perception/lidar/tracks_map`, 공용 원자적 관측 |
| `lidar_source_policy` / `source_policy` | 선택 인자 제거, 공용 map 입력이 기본 |
| nested Main/Msg source·빌드 | outer workspace만 빌드·source |

속도 계산과4단계 상태·진단 필드 의미는 유지했다. 옛 이름은 이전 설명이며 alias/wrapper를 제공하지 않는다.
손재호의 진행 브랜치는 정본화 revision 위에 반영하고 새 헤더·필드를 사용한다. 필요 원본·시험 근거는
[보존 manifest](../../../config/baselines/lidar_canonicalization_v1.json)에서 찾는다.
