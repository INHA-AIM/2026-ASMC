# Scan trace 형식 (`asmc_lidar_scan_trace_v1`)

입력 PointCloud2 한 개당 JSON 객체 한 개다. `asmc_lidar_shadow_replay`는 JSONL 파일로 쓰고
`asmc_lidar_shadow_node`는 구독자가 있을 때만 `/asmc_lidar/shadow/trace`(`std_msgs/String`)로 보낸다.
직렬화 코드는 `include/asmc_lidar/pipeline/scan_trace.hpp`이며 [shadow 파이프라인](shadow-pipeline.md)의
bag 비교에 사용한다.

- 숫자는 유효숫자 17자리다(float을 double로 넓혀도 정확히 복원). 행렬은 row-major다.
- 값이 없으면 필드를 생략한다(가짜 0을 쓰지 않음). JSON에는 NaN/Inf가 없으므로 비유한 값은 `null`이다.
- `tracks` 순서는 Main `st_LiDAR.vec_Objects`와 같다(`f32_Dist` 정렬).
- `*` 필드는 `--capture-debug`(replay)일 때만 있다. 노드 trace에는 없다.

```text
{
 "schema": "asmc_lidar_scan_trace_v1", "producer": "<writer>",
 "seq": 1-based points-topic index, "stamp_ns": header stamp, "arrival_ns": record/receipt time,
 "accepted": LiDAR callback gate (zero stamp / >62500 points / non-increasing stamp),
 "accept_reason": "", "pose_ready": local IMU/CAN lookup, "reason": "", "imu_ns", "can_ns",
 "local": {"roll_rad", "pitch_rad", "yaw_rad", "yaw_deg", "speed_mps"},
 "map": {"valid", "reason", "map_from_base": [16], "gps_stamps": [...]},
 "processed", "process_reason", "dt_s",
 "counts": {"raw", "roi", "voxel", "leveled", "ground_kept", "tunnel_removed", "cluster_input",
            "clusters", "centroids", "detections", "tracks", "observed", "predicted"},
 "digests"*: {"roi", "voxel", "leveled", "ground_kept"},
 "plane": {"valid", "coefficients": [4]}, "leveling": [9], "base_from_input": [16],
 "clusters"*: [{"size", "centroid": [3]}], "detections"*: [<object>],
 "tracks": [<object> + track fields], "typed": {"local": <array>, "map": <array>}
}
<object> = {"idx", "core": [3], "min": [3], "max": [3], "dist", "points",
            "obb": {"center": [3], "width", "height", "zmin", "zmax", "yaw"}, "corners": [[3] x 8]}
track fields = {"id", "age", "miss", "kf_initialized", "kf_init_count", "kf_ca": [6], "kf_p": [36],
            "prev_yaw", "pred_core": [2], "abs_v": [2], "velocity", "tracking_frame_valid",
            "tracking_height_input", "class", "raw_class", "class_misses", "continuity",
            "ground_evidence": {"valid", "samples", "median_clearance", "band"}}
<array> = {"accepted", "status", "reason", "frame", "stamp_ns", "provenance",
           "objects": [{"track_id", "detection_index", "center": [3], "corners": [[3] x 8], "yaw",
                        "length", "width", "height", "points", "classification", "camera_class"}]}
```

`digests.*`는 점마다 `float x, y, z, intensity` 16바이트(little endian)를 cloud 순서로 이은 값의
SHA-256(소문자 hex)이다. 빈 cloud는 0바이트의 해시다. `accepted=false`이거나 `pose_ready=false`인
scan에는 그 시점까지 알 수 있는 필드와 `typed`(INPUT_INVALID 배열 또는 `accepted:false`)만 있다.
`abs_v`는 Main 내부의 "KF 상대속도 + CAN 속도" 값이며 좌표계가 검증되지 않아 typed 출력에서는
속도 validity가 항상 false다.
