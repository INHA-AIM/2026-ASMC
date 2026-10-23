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
| `src/asmc_lidar/launch/shadow_pipeline.launch` | `src/asmc_lidar/launch/lidar_pipeline.launch` |
| `src/asmc_lidar/launch/shadow_pipeline_lidar_only.launch` | `src/asmc_lidar/launch/lidar_only.launch` |
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
