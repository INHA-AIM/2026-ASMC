# Scan trace 형식 (`asmc_lidar_scan_trace_v1`)

입력 PointCloud2 한 개당 JSON 객체 한 개다. `lidar_replay`는 JSONL 파일로 쓰고
`lidar_pipeline_node`는 구독자가 있을 때만 `/migration/lidar/trace`(`std_msgs/String`)로 보낸다.
직렬화 코드는 `include/asmc_lidar/ros/scan_trace.hpp`이며 [shadow 파이프라인](pipeline.md)의
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

## 선택적인 HD맵 audit와 발행 지연

A1.1의 내부 설정이 `audit`일 때만 `hd_map_audit` 블록을 추가한다. 기존 `typed`·`tracks`는
수정하지 않는다. 구형 trace는 이 필드가 없어도 유효하고 공개 ROS 메시지·MD5는 그대로다.
상위 `producer_session_id`·`generation`·`seq`·`stamp_ns`·`config_id`와 각 `track_id`로 연결한다.

- `enabled`, `available`, `reason`: OFF면 블록 없음. 지도 오류는 `map_audit_unavailable`이며 객체 발행은 유지.
- `map_id`, `map_sha256`, `bundle_sha256`, `policy_sha256`: 로딩 시 검증한 제품·파생 입력·정책 식별.
- `map_identity_provenance`: pinned bundle 및 실제 runtime 원점 설정에서 가져온 ID임을 표시.
- `position_error_verified`, `mission_scope_verified`: 현재 항상 false. GPS age 통과를 정확도 검증으로 바꾸지 않음.
- `calculation_ms`, `objects[]`: scan 계산 비용과 객체별 판정. `decision`은 keep/exclude_candidate/undetermined,
  `reason`은 사유, `spatial`은 파생 grid의 겹침·보호 범위·unknown·예외 공간이다.
- 객체별 `input_state`는 observed/predicted를 구분한다. 이 값과 분류 그룹은 실제 운동 판정이 아니다.

점군/높이 미리보기를 켜면 `preview_config_id`에 실제 runtime 설정 SHA를 더한다.
기존 bundle의 `policy_sha256`를 새 정책 SHA로 가장하지 않는다. 객체별 선택 블록은 다음과 같다.

- `obb_reference_decision/reason`: 기존 A1 OBB 판정. Python/C++ 기하 대조는 이 값을 비교한다.
- `preview.decision/reason`: 검수용 keep/exclude_candidate/undetermined. 실제 전달 제외 허가와 다르다.
- `preview.support_points/road_points/guard_points/unknown_points/road_point_ratio`:
  화면 샘플이 아닌 객체 전체 cluster support의 지도 관계. voxel 이전 원본 점 비율은 아니다.
- `preview.min_support_base_z/obb_bottom_base_z/roof_limit_base_z`: `base_link` 높이. 근거가 없으면 null.
- `preview.camera_dynamic_protected/camera_known`: 같은 fusion consumer epoch의 최신 양성 연결 근거.
  실제 이동 여부나 카메라만으로 생성한3D 객체가 아니다.

`decision/reason`은 미검증 하부면·점군 정책·위치 오차를 계속 보류할 수 있다. 따라서 preview의
제외 후보 수와 최종 제외 후보 수를 혼합하지 않는다. 기존 trace에 이 선택 블록이 없어도 유효하다.
웹용 `asmc_lidar_debug_scan_v1`도 같은 audit 결과와 producer session/generation/config ID를
구독 수요가 있을 때 제공한다. 웹을 위해 큰 전체 trace를 추가 구독할 필요는 없다.

실제 노드의 `input_to_publish_ms`는 PointCloud callback 시작부터 객체 배열/관측의 ROS publish
호출 완료까지 **동일 monotonic clock**으로 측정한다. 네트워크 소비자 수신·노출/laser 발사 시각을
포함하는 end-to-end 지연은 아니다. replay에는 실제 발행이 없어 이 필드를 만들지 않는다.
기존 `timings_ms.total`은 runtime 내부 대기·계산이며 이 지연과 구분한다.
1Hz diagnostics는 최근100scan의 판정/사유 수·계산 p95·설정 오류와 callback→publish p95를
제공한다. 상세 trace는 구독할 때만 직렬화하며 판제 정본 입력으로 사용하지 않는다.
