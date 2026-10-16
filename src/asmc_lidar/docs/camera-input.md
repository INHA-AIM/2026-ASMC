# 카메라 관측 연결

이 컴포넌트는 카메라 입력과 투영 근거를 제공한다. 실행 중인 YOLO는 `asmc_camera` 한 곳에
두고, 손재호의 LiDAR 노드 안에서 `camera::Receiver`를 생성한다. 현재 운영 Main의 전방
fusion과 이 수신 라이브러리는 별도다. 전체 LiDAR 이전·3뷰 객체 매칭은 아직 완료되지 않았다.

## 공용 상태와 구형 제어 전환

정본 상태는 `asmc_msgs/InferenceStatus`, 서비스는 `asmc_msgs/SetInferenceMode`다.
정본 경로는 `/perception/camera/{objects,lane,traffic_light}/status`, `/set_mode`다.
새 상태의 `producer_session_id`는 해당 모델 제어부의 UUID이며, 객체 관측의 UUID와 같다.
재시작 때 generation이 같은 숫자가 되더라도 이전 실행과 구분한다.

구형 `asmc_camera` 타입은 각 기존 노드 루트의 `/inference_status`, `/set_inference_mode`에
남는다. 같은 제어 상태를 변환해 제공하며 추론 생산자나 중계 노드를 추가하지 않는다.
정본 경로를 사용하던 구형 타입 소비자는 새 타입으로 재빌드해야 한다. 과거 bag의 구형
상태를 새 타입 토픽으로 이름만 바꿔 재생하지 않는다. 구형 경로/고정 checkout으로 재현하거나
원본을 보존한 변환 fixture를 사용한다. 새 UUID를 알 수 없는 기록은 새 세션 정답으로 간주하지 않는다.

카메라·Main·새 소비자를 같은 변경으로 빌드하고 재시작한다. 바깥 workspace를 먼저 source한
뒤 nested Main을 빌드/source한다. LiDAR 수신부는 모드 서비스를 호출하지 않는다.
모드 결정은 판제가 한다. 기존 메시지와 서비스의 MD5는 보존하며 새 상태 MD5는
`8b2caf96bbf9a9eaea848fcb9551b79f`다. 공용 서비스 MD5는
`94a789aab65be7cc8d09e8f777708430`이지만 타입명도 바뀌므로 서비스 호환을 MD5만으로 판정하지 않는다.

## LiDAR 노드에 연결

```cpp
#include <asmc_lidar/camera_receiver.hpp>
#include <asmc_lidar/camera_calibration.hpp>

// NodeHandle와 Receiver를 LiDAR 노드의 수명 동안 유지한다. 스피닝은 호출자가 담당한다.
const auto calibrations = asmc::lidar::camera::load_calibrations(camera_yaml_path);
asmc::lidar::camera::Receiver receiver(node_handle, calibrations);
auto buffer = receiver.buffer();

// scan_stamp_ns는 scan의 ROS source stamp, max_difference_ns는 호출자의 허용 시간차다.
auto observations = buffer->query(scan_stamp_ns, max_difference_ns,
    ros::Time::now().toNSec(), asmc::lidar::camera::steady_now_ns());
for (const auto& view : observations.views) {
  for (const auto& candidate : view.second.candidates) {
    const auto& frame = candidate.record->observation;
    // frame.boxes + calibrations.at(frame.header.frame_id)를 객체 매칭에 사용한다.
    // 무거운 계산은 버퍼 잠금 밖에서 수행하고 상태 반영 전 아래 검사를 다시 한다.
    if (!buffer->current(candidate, ros::Time::now().toNSec(),
                         asmc::lidar::camera::steady_now_ns())) continue;
    // 실제 LiDAR 객체 연결·분류 반영과 관측 소비 정책은 호출자가 구현한다.
  }
}
```

`asmc_lidar_camera_core`는 ROS에 독립적인 버퍼·보정 로더·투영 경계이고,
`asmc_lidar_camera_ros`는 공용 메시지를 core로 변환하는 구독부다. Main/Msg, 카메라 Python,
모델·TensorRT를 import/link하지 않는다. 통합 실행에서 카메라 정본 YAML 경로를 제공한다.

조회는 비차단이며 뷰별로 허용 시간차 이내의 후보를 가까운 순으로 반환한다. 같은 거리면
최신 관측이 먼저다. 조회가 관측을 소비하지 않는다. 호출자는
`session/generation/frame_id/stamp/calibration_id`로 이미 사용한 근거를 식별한다.
여러 카메라 또는 반복 조회를 서로 독립적인 시간 확인으로 세지 않는다.
현재 전방 Main의 ±50ms·200ms 대기는 이 라이브러리의 3뷰 매칭 정책이 아니다.

## 상태·시간·메모리

기본은 뷰별 8관측, 수신 후/원본 source age 각각350ms, 상태 freshness350ms,
source 미래 허용50ms다. `camera::Limits`로 명시적으로 전달할 수 있고 설정 목록에는 기본값을 기록한다.
보관 기간과 query의 매칭 허용 시간차를 구분한다. source는 ROS clock, 경과 시간은
`steady_clock`이며 `/clock`이 정지해도 수신 후 만료가 작동한다.

상태 heartbeat는 모델 전체의 ACTIVE 여부와 실행 식별자를 검증한다. `prepared`는 세 뷰
전체 준비 상태이므로, 한 뷰 중단 때 다른 유효 뷰까지 차단하는 조건으로 쓰지 않는다.
각 관측의 valid·시간·보정을 검사한다. STANDBY/PREPARE는 운영 후보를 반환하지 않는다.

- `ok`: 검증된 양성 박스가 있는 관측. `empty`: 검증된 정상 빈 관측.
- `invalid`: 해당 뷰의 무효 결과. `missing`: 아직 사용 가능한 입력이 없음.
- `stale`/`status_stale`: 영상/상태 시간 만료. `no_time_match`: 입력은 있지만 요청 scan과 시간차 초과.
- 세션·세대 변경은 전체 버퍼를 비운다. 단일 뷰 무효화/보정 오류는 해당 뷰만 비운다.
- 같은 실행·세대에서 모델/설정 ID가 달라지면 `identity_conflict`로 차단하고 새 세대/세션을 기다린다.
- 중복·역행 관측은 거부한다. 무효화는 마지막 정상 관측과 같은 stamp를 사용할 수 있다.
- counters는 고정 사유별 누적 수치다. 전체 프레임/이미지 journal을 메모리에 보관하지 않는다.
- 후보는 immutable snapshot이다. 모드 전환 후에도 메모리를 읽을 수 있지만 `current()`는 false다.
  호출자의 최종 fusion 상태 반영은 자체 실행/출력 게이트와 함께 처리해야 한다.

카메라 미검출은 LiDAR 객체 삭제나 자유 공간의 근거가 아니다. 원본 이름과 세 소비 그룹을
보존하고 실제 움직임은 추적에서 판단한다. 장면의 노출 시각은 header만으로 보장되지 않으며
기존 bridge source/fallback 정책과 scan 시각 차이를 함께 기록한다.

## 투영과 보정

카메라 정본 `cameras_v3.yaml`을 읽고, 카메라별 전체 mapping과 source SHA의 정렬 JSON을
해시한다. Python 생산자와 C++ 로더의 해시는 교차 시험한다. 현재 정본의 유한한 숫자와
JSON 호환 YAML을 사용하며 모호한 숫자 표현은 로더가 거부한다.

전방1280×720·좌우640×480의 bbox와 intrinsic을 사용한다. 전방544×960 tensor의 padding을
다시 적용하지 않는다. `project()`에는 scan에 실제 적용한 Context와 그 좌표계의 점을 넣는다.
뒤쪽·화면 밖·잘못된 변환은 valid=false와 사유로 반환한다.

영상 시점 base ← scan 시점 base 변환을 호출자가 제공하면 `time_compensated=true`로 반환한다.
변환을 주지 않고 시각이 다르면 `ok_uncompensated_time_offset`과 시간차를 명시한다.
이 결과를 시간 정렬이 검증된 투영으로 표시하지 않는다. 자동 pose 조회·외삽·map 변환은 없다.

```cpp
// scan_context와 point_in_scan_frame은 실제 전처리 결과의 좌표/시간을 사용한다.
const auto& image = candidate.record->observation;
auto pixel = asmc::lidar::camera::project(
    calibrations.at(image.header.frame_id), scan_context,
    image.header.stamp_ns, point_in_scan_frame, image_base_from_scan_base);
if (pixel.valid) {
  // pixel.u/v는 원본 영상 좌표. image_minus_scan_ns와 time_compensated를 함께 검사한다.
  // 실제 bbox 연결과 시간 허용 여부는 호출자의 fusion 정책으로 결정한다.
}
```

`image_base_from_scan_base`는 `std::optional<asmc::lidar::Pose>`다. 시간 보정 근거가 없으면
`std::nullopt`를 전달하고 미보정 시간차를 명시적으로 처리한다.

## 직접 빌드·시험·관측 확인

기존 실행과 분리된 build/devel을 사용한다. `--only-pkg-with-deps asmc_lidar`에는
카메라 추론 패키지가 포함되지 않는다.

```bash
source /opt/ros/noetic/setup.bash
cd /path/to/2026-ASMC
catkin_make --build /tmp/lidar-camera-build \
  -DCATKIN_DEVEL_PREFIX=/tmp/lidar-camera-devel \
  --only-pkg-with-deps asmc_lidar -DCATKIN_ENABLE_TESTING=ON -j2 -l2
catkin_make --build /tmp/lidar-camera-build \
  -DCATKIN_DEVEL_PREFIX=/tmp/lidar-camera-devel camera_receiver_ros_test -j2 -l2
(cd /tmp/lidar-camera-build/asmc_lidar && ctest --output-on-failure)
source /tmp/lidar-camera-devel/setup.bash
python3 src/asmc_msgs/test/test_message_serialization.py
```

카메라가 이미 실행 중인 동일 master에서 관측만 확인한다. 모드를 바꾸거나 추론/rqt를
새로 시작하지 않는다. 예제의 조회 허용 시간차50ms는 확인용 값이다.

```bash
rosrun asmc_lidar camera_observation_probe \
  _calibration_file:=/path/to/2026-ASMC/src/asmc_camera/config/cameras_v3.yaml
rostopic echo /lidar_camera_probe/diagnostics
```

Probe는 자신의 diagnostics만 발행한다. `/YOLO/BBox`, `/last_lidar`, 차량 제어를 발행하지 않는다.
실제 이전 순서는 [카메라→LiDAR 인계](../../../docs/handoffs/camera-to-lidar.md)를 따른다.
