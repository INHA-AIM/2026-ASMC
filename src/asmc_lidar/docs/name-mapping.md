# LiDAR 경로·이름 이전표

2026-10-23 구조 정리. 옛 경로는 이전 안내이며 호환 wrapper는 제공하지 않는다.

| 이전 | 현재 |
|---|---|
| `src/asmc_lidar/config/shadow_pipeline.yaml` | `src/asmc_lidar/config/lidar_pipeline.yaml` |
| `src/asmc_lidar/docs/pipeline.md` | `src/asmc_lidar/docs/pipeline.md` |
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
