# 센서 원본 시각·수신 시각 보존

## 상태와 목표

**Camera·IMU·LiDAR timing 기록 경로를 구현했다.** 현재 bridge의 `timestamp_mode=receive`, 기존 센서 메시지와
header·MD5·포트·LiDAR 프레임 구성·GPS/IMU state·Main의 ±50ms/200ms 정책은 바꾸지 않는다.
카메라/IMU의 원본 시각과 실제 bridge 수신/처리 시각을 따로 보존한 뒤 운동 보정·지연 분석에 사용한다.
첫 범위는 `morai_sensor_bridge`의 camera·IMU·LiDAR다. 네트워크 Ego/Object/Collision의 전역
source 전환까지 한 번에 확대하지 않는다.

## 변경 전 경로와 보존 이유

- `_camera_loop`는 모든 JPEG 조각의 원본 `(sec,nsec)`와 index 연속성을 확인한다.
  완성 후 `_stamp_from_sec_nsec`가 receive 또는 source 중 하나를 header에 넣는다.
  원본 시각은 기존 receive 정책의 결과 메시지 하나만으로 복구할 수 없었다.
- `_imu`도 원본 sec/nsec와 quaternion을 읽지만 결과에는 선택한 header stamp 하나만 남는다.
- `BridgeTelemetry.receive`는 `recvmsg()`로 `SO_RXQ_OVFL`을 읽고 `(bytes,address)`만 반환한다.
  기존에는 커널 packet timestamp를 활성화하지 않았고, 처리 시작/조립 소요시간은 네트워크 수신 시각과 달랐다. 현재는 `receive_timed`가 별도 receipt를 반환한다.
- `_lidar_publish`는 회전 안의 device time 중 마지막 유효 값을 source stamp 후보로 사용한다.
  Velodyne의 시간은 hour 내 microseconds다. Unix sec/nsec로 직접 해석하지 않는다.
- receive는 완성 message를 만들 때의 ROS 시간이다. camera 노출 시각·첫 UDP 도착 시각이 아니다.
  `timestamp_mode=source`에서 source 값이 무효/역행하면 receive fallback을 적용한다. 그 결과 stamp를 원본으로 간주하면 안 된다.
- GPS NMEA에는 camera/IMU와 동일한 원본 sec/nsec 필드가 없다. 없는 값을 source라고 만들지 않는다.

MORAI는 camera와 IMU 프로토콜에 timestamp를 제공한다. 프로토콜의 timestamp가 실제 노출의
어느 단계에 찍히는지는 별도 검증 대상으로 남긴다.
[공식 센서 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/-35)

## 구현 계약: 센서 메시지와 timing 기록을 함께 발행

공용 **`asmc_msgs/SensorFrameTiming`**을 `/morai_sensor_bridge/frame_timing`에 기본 발행한다.
[기계 계약](../../config/contracts/sensor_frame_timing_v1.yaml)과
[메시지 정의](../../src/asmc_msgs/msg/SensorFrameTiming.msg)가 필드의 정본이다.
기존 이미지/IMU/cloud를 감싸서 다시 발행하거나 JPEG를 복제하지 않는다.

| 정보 | 보존 목적 |
|---|---|
| `bridge_session_id`, `stream_id`, `frame_sequence` | 재시작·중복·누락을 식별하는 독립 유일 키 |
| `sensor_topic`, `sensor_frame_id`, `published_stamp`, wire sequence | 기존 센서 결과와 연결; 선택한 header를 정확히 기록 |
| 원본 raw sec/nsec 또는 device tick, `source_valid`, `source_clock_domain` | 원본을 변형 없이 보존; Unix/device-hour/없음 구분 |
| `source_reference` 및 필요시 source start/end | camera frame·IMU sample·LiDAR sweep의 시간 의미 구분 |
| `rx_first_unix_ns`, `rx_last_unix_ns`, 수신 방식·유효 여부 | kernel 수신과 user-space fallback 구분 |
| first/last receive·publish의 monotonic ns | 같은 bridge 세션 안의 조립·처리 소요시간 계산 |
| ROS publish time, `/use_sim_time` 상태, header 정책·fallback 사유 | ROS/시뮬/Unix clock 혼합 방지 |
| packet 수·조립 상태·중복/역행 표시 | 불완전 frame과 정상 빈 관측을 구분 |

현재 Noetic의 `rospy.msg.serialize_message`가 최상위 `header.seq`를 publisher sequence로
덮어쓰는 것을 코드로 확인했다. 원 센서 발행의 실제 wire sequence를 metadata의 별도 필드에
복사한다. 같은 publisher lock에서 serialize 전후 sequence를 확인하며, 무구독으로
직렬화를 생략한 경우 `sensor_wire_sequence_valid=false`다. 독립 ROS master의 실제 UDP/serialization
시험에서 무구독·연결·raw 중복/충돌/역행을 검증했다.
메타데이터 자체의 header sequence와 센서의 wire sequence를 같은 것으로 가정하지 않는다.
같은 header stamp가 반복될 수 있으므로 **stamp 하나만으로 1:1 조인하지 않는다.** Session·stream·
sequence를 보조 키로 쓰고, 필요시 원본 payload의 짧은 digest를 추가한다. 해소되지 않는 조인은
ambiguous로 배제하며 다른 IMU나 이미지에 임의로 붙이지 않는다.

원본 IMU 시각이 같고 quaternion/gyro도 같으면 동일 측정의 재전송으로 취급한다. 수신 기록은 각각
보존하되 보간의 독립 자세 수를 늘리지 않는다. 같은 원본 시각에 값이 충돌하면 별도 오류다.
원본에 임의1ns를 더하거나 receive fallback을 원본 필드에 덮어쓰지 않는다.

## 수신부 구현

1. `BridgeTelemetry`에 별도 timed receive 경로를 두고 기존2값 반환 인터페이스를 보존한다.
   실제 수신은 현재 소켓 하나에서 수행한다. 두 번째 소켓으로 같은 UDP 포트를 경쟁 수신하지 않는다.
2. Linux `SO_TIMESTAMPNS_NEW` 계열 지원을 확인하고 ancillary buffer에 timestamp와
   `SO_RXQ_OVFL` 공간을 모두 확보한다. `MSG_CTRUNC`·timestamp 미지원도 진단한다.
   Python3.8·64bit WSL의 실제 상수/ABI를 시험하고, 지원하지 않으면 **user-space receive**로 명시한다.
3. JPEG index0에서 first receive, final 조각에서 last receive를 보존한다. 모든 조각의 source key가
   같은지 계속 검사한다. 완성 전에 실패한 frame은 진단으로 남기며 정상 영상 metadata로 쓰지 않는다.
4. IMU는 packet 하나의 source/receive 쌍을 보존한다. LiDAR는 기존 회전 assembler의 첫/마지막
   device time·수신 범위를 함께 보존한다. PointCloud를 단일 순간 측정으로 설명하지 않는다.
5. frame당 작은 metadata 하나를 만든다. 고주파 LiDAR packet마다 ROS metadata를 발행하지 않는다.
   Bridge 수신 경로에는 파일 쓰기를 추가하지 않는다. 별도 확인기가 전체 metadata/header를
   파일로 기록하고 대응표는 bounded 상태로 관리한다. queue 누락·sequence gap은 별도 집계한다.
   receive callback에서 JSON 파일 쓰기·이미지 decode·전체 payload 복사는 추가하지 않았다.

커널 timestamp도 시스템 시간이며 단조 증가가 보장되는 시간이 아니다. 지속시간에는 같은 프로세스의
monotonic clock을 쓴다. ns 단위 필드는 ns 정확도 보장이 아니며, 현재 원본 timestamp의 약1ms
양자화도 함께 기록한다. 소프트웨어 수신 timestamp를 하드웨어/노출 timestamp라고 부르지 않는다.
[Linux kernel timestamping 문서](https://docs.kernel.org/networking/timestamping.html)

## 보존한 시각의 활용

### 1단계: 기록과 관측

센서 토픽·timing 토픽·run의 clock/header 정책을 함께 bag에 기록한다. Camera frame과 metadata,
IMU sample과 metadata가 정확히 연결되는지 먼저 확인한다. 기존 receive-only bag에서 버려진
원본 시각을 나중에 복구할 수는 없다. 기록 손실이나 metadata 미수신은 missing으로 남긴다.

차량이 정차해 있으면 source↔receive의 지연·조립·GIL 대기·중복은 볼 수 있지만 동적인 센서 시간
offset을 정확히 식별할 수 없다. Windows 송신 clock과 WSL 커널 clock의 차이가 포함되므로
`rx-source`를 순수 네트워크 지연으로 표시하지 않는다. 앞선20초 passive 표본에서도 음수 차이가
있었으며 자동 clock 보정의 근거로 사용하지 않는다.

### 2단계: 카메라 시각에 맞춘 IMU 자세

원본 clock domain이 같고 source가 검증된 camera/IMU만 약1초의 bounded 시간 버퍼에서 조회한다.
각 영상의 원본 시각을 둘러싼 두 quaternion으로 SLERP한다. 앞선 실측 bracket p50약42ms,
p95약46.2ms를 참고하되 최대 허용 bracket100ms·외삽 금지 조건을 시험한다.
이미 추론이 진행되는 동안 확보된 IMU를 사용해 결과 발행에 별도 긴 대기를 넣지 않는다.

추론 완료 시점의 최신 IMU나 단순 nearest pose를 영상 시각의 자세로 취급하지 않는다.
Metadata 누락·역행·재시작·clock 불일치·bracket 부족이면 운동 보정 사용을 거부하고 사유를 표시한다.
현재 정적 지면 경로를 유지하는 경우도 `motion_compensation_used=false`로 드러낸다.
기존 freshness350ms를 늘리거나 과거 결과를 새 stamp로 재발행하지 않는다.

### 3단계: 운동/퓨전 검증 후 전환

카메라와 IMU의 장착 변환·정차 도로 법선과 함께 동적 원본 RGB 재투영을 대조한다.
원본 시각을 사용한다고 자동으로 M1 정확도가 좋아졌다고 결론 내리지 않는다.
카메라 뒤쪽·지원 범위·새 family 무효·지연·세대 전환을 확인한다.

LiDAR sweep은 시간 구간이므로 마지막 device time만 맞춘 것을 point별 motion deskew로 설명하지 않는다.
실제 scan 시각 기준과 per-point 시간·운동 입력은 LiDAR 팀과 별도 결정한다. 기존 Main의 ±50ms/200ms
매칭이나 판제 timeout은 이 metadata 추가만으로 변경하지 않는다.
Source header가 필요한 소비자는 충분한 대조 후 별도 명시적 프로필/계약으로 전환한다.
전역 `timestamp_mode=source` 하나로 Ego·LiDAR·camera·IMU를 동시에 바꾸는 방식은 이번 권고가 아니다.

## 검증 범위와 남은 작업

- 공용 timing 계약과 실제 wire key, 정상 frame당 metadata 수, ROS bag 재생 시 조인.
- 커널/사용자 수신 timestamp·ancillary truncation·overflow·미지원 fallback과 기존 socket drop 계측 공존.
- UDP 조각 누락/재순서·source 무효/역행/동일 시각·device-hour rollover·세션 재시작·ROS clock 점프.
- 메모리 상한·저장 종료·metadata queue 누락 진단, 기존 센서 header/MD5·Hz·지연의 회귀 없음.
- 정확한 source-time SLERP와 보간 불가 사유, 현재 정적 경로와의 품질·지원 범위 비교.

합성 영상도 단일 header가 세 뷰의 모든 입력 시각을 나타내지는 않는다. 뷰별 원본 header/관측 ID를
별도로 유지해야 하며 합성 header의 반복을 곧 동일 영상 재표시로 단정하지 않는다.

구현과 실제 MORAI 측정은 [작업 기록](../records/bridge/2026-10-18-sensor-frame-timing.md)을 따른다.
Camera/IMU 원본 시각과 kernel/user receive 보존·wire 연결·중복/충돌/역행 진단은 구현했다.
운영 camera/IMU SLERP·동적 운동 보정·LiDAR point deskew는 **아직 적용하지 않았다**.
기존 bag에서 버린 source 시각을 복원하거나 receive를 source로 바꾸어 표시하지 않는다.

## 구체적인 구현 제한

- `SO_TIMESTAMPNS_NEW=64`를 먼저 사용하고, 미지원 시 native OLD timespec 또는 user-space로
  fallback한다. `rx_userspace_fallback_packets`와 `ancillary_truncated_packets`로 구분한다.
  monotonic receipt는 syscall 반환 직후여서 kernel 수신 이후 사용자 스케줄링을 포함한다.
- `publish_ros_time`·`publish_monotonic_ns`는 센서 publish 직후 timing을 조립하는 시점이다.
  커널 송신 완료·센서 노출·소비자 수신 시각으로 해석하지 않는다.
- LiDAR의 `source_duplicate`는 sweep 마지막 tick이 이전 sweep 끝과 같은 경우다.
  `device_tick_duplicates`는 sweep 내부에서 반복된 packet tick 수다. 이를 cloud 중복으로 세지 않는다.
- JPEG 조각 최대4096개/16MiB, LiDAR 조립 최대 `max(4096, packets_per_frame*4)`개,
  raw source history는 stream당128키, metadata 출력 큐는128개로 제한한다.
- IMU 발행 큐는 기존1개에서16개로 늘렸다. 몰려온 서로 다른 source pose가 ROS 비동기 큐에서
  사라지는 것을 줄이기 위한 버퍼이며 영상의 queue1 최신 우선·센서 stamp 정책은 유지한다.
- Diagnostics에 source 무효·중복·충돌·역행, timing 실패·발행 수, ancillary truncation과
  kernel timestamp option을 추가했다. 콘솔5초·진단1초 주기는 유지한다.
- Metadata는 센서와 별도 ROS 전송이므로 함께 도착한다는 보장은 없다. sequence gap·missing·ambiguous를
  기록하고 보간에서 배제한다. 무구독 센서에는 wire sequence가 없으며 stamp만으로 대신 조인하지 않는다.

## 직접 기록·확인

기존 raw bag 프로필(camera/lidar/full/development GT)은 timing 토픽을 선택 항목으로 포함한다.
기록 시작 때 발행 중이면 함께 저장한다. 구형 bridge에서 토픽이 없더라도 raw 수집을 막지 않는다.
원본 시각 분석을 할 run은 실제 metadata 포함 여부를 확인한다. Capture GT 파일 수집 경로는 변경하지 않는다.

기존 source 순서로 workspace를 source한 뒤 별도 터미널에서 실행한다.

```bash
# 영상 payload를 복제하지 않는 짧은 측정. 출력 경로는 새 run마다 다르게 지정한다.
python3 tools/bridge/verify_frame_timing.py --duration 30 --output /tmp/sensor-timing-review
rostopic echo -n 1 /morai_sensor_bridge/frame_timing
# 후속 운동 분석용 bag에는 원 센서와 metadata를 함께 기록한다.
rosbag record -O /tmp/camera-imu-timing.bag /image_jpeg/compressed /cam_left/image_jpeg/compressed /cam_right/image_jpeg/compressed /imu /morai_sensor_bridge/frame_timing
```

확인기는 `timing.jsonl`·`sensor-headers.jsonl` 전체 timing/header와 정확히 대응한 `matched.jsonl`,
누락·보류·중복·원본 역행·kernel fallback·진단을 담은 `report.json`을 생성한다.
대형 payload는 보관하지 않고 대응표는 stream당256개·2초로 제한한다.

## TwinLite 자세 보정 연결

이 timing을 소비하는 [IMU M1 시험 경로](camera-imu-projection.md)를 추가했다.
카메라 생산자는 `CameraModelGridTiming`으로 원 JPEG와 class/confidence의 실제 wire header를 연결한다.
Family는 같은 원본 clock의 IMU quaternion을 보간해 roll/pitch를 지면 투영에 반영한다.
센서 header를 source로 바꾸거나 수신 시각을 원본으로 대체하지 않는다.
정본 기본값은 고정 평면이며, 자세 보정 후보의 시각 대응과 품질·공급 수락은 별도로 판단한다.
