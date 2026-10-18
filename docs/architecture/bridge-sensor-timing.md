# 센서 원본 시각·수신 시각 보존 설계

## 상태와 목표

**이 문서는 구현 전 분석이다.** 현재 bridge의 `timestamp_mode=receive`, 기존 센서 메시지와
header·MD5·포트·LiDAR 프레임 구성·GPS/IMU state·Main의 ±50ms/200ms 정책은 바꾸지 않는다.
카메라/IMU의 원본 시각과 실제 bridge 수신/처리 시각을 따로 보존한 뒤 운동 보정·지연 분석에 사용한다.
첫 범위는 `morai_sensor_bridge`의 camera·IMU·LiDAR다. 네트워크 Ego/Object/Collision의 전역
source 전환까지 한 번에 확대하지 않는다.

## 현재 코드에서 알 수 있는 것

- `_camera_loop`는 모든 JPEG 조각의 원본 `(sec,nsec)`와 index 연속성을 확인한다.
  완성 후 `_stamp_from_sec_nsec`가 receive 또는 source 중 하나를 header에 넣는다.
  원본 시각은 receive 정책의 결과 메시지에서 복구할 수 없다.
- `_imu`도 원본 sec/nsec와 quaternion을 읽지만 결과에는 선택한 header stamp 하나만 남는다.
- `BridgeTelemetry.receive`는 `recvmsg()`로 `SO_RXQ_OVFL`을 읽고 `(bytes,address)`만 반환한다.
  커널 packet timestamp는 활성화하지 않으며, 현재의 처리 시작/조립 소요시간은 네트워크 수신 시각과 다르다.
- `_lidar_publish`는 회전 안의 device time 중 마지막 유효 값을 source stamp 후보로 사용한다.
  Velodyne의 시간은 hour 내 microseconds다. Unix sec/nsec로 직접 해석하지 않는다.
- receive는 완성 message를 만들 때의 ROS 시간이다. camera 노출 시각·첫 UDP 도착 시각이 아니다.
  source 값이 무효/역행하면 receive fallback을 적용한다. 그 결과 stamp를 원본으로 간주하면 안 된다.
- GPS NMEA에는 camera/IMU와 동일한 원본 sec/nsec 필드가 없다. 없는 값을 source라고 만들지 않는다.

MORAI는 camera와 IMU 프로토콜에 timestamp를 제공한다. 프로토콜의 timestamp가 실제 노출의
어느 단계에 찍히는지는 별도 검증 대상으로 남긴다.
[공식 센서 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/-35)

## 제안 계약: 센서 메시지와 작은 timing 기록을 함께 발행

공용 **`asmc_msgs/SensorFrameTiming`**을 제안한다. 아래 필드는 설계안이며 아직 생성 메시지나
운영 토픽을 추가하지 않았다. 제안 토픽은 `/morai_sensor_bridge/frame_timing`이다.
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
복사하는 시점과 무구독/직렬화 실패를 실제 연결 시험으로 확정해야 한다.
메타데이터 자체의 header sequence와 센서의 wire sequence를 같은 것으로 가정하지 않는다.
같은 header stamp가 반복될 수 있으므로 **stamp 하나만으로 1:1 조인하지 않는다.** Session·stream·
sequence를 보조 키로 쓰고, 필요시 원본 payload의 짧은 digest를 추가한다. 해소되지 않는 조인은
ambiguous로 배제하며 다른 IMU나 이미지에 임의로 붙이지 않는다.

원본 IMU 시각이 같고 quaternion/gyro도 같으면 동일 측정의 재전송으로 취급한다. 수신 기록은 각각
보존하되 보간의 독립 자세 수를 늘리지 않는다. 같은 원본 시각에 값이 충돌하면 별도 오류다.
원본에 임의1ns를 더하거나 receive fallback을 원본 필드에 덮어쓰지 않는다.

## 수신부 개선 지점

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
   로그 쓰기는 bounded journal에서 처리하고 queue overflow·sequence gap을 별도로 기록한다.
   receive callback에서 JSON 파일 쓰기·이미지 decode·전체 payload 복사는 추가하지 않는다.

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

## 구현 전 확정/시험 목록

- 공용 timing 계약과 실제 wire key, 정상 frame당 metadata 수, ROS bag 재생 시 조인.
- 커널/사용자 수신 timestamp·ancillary truncation·overflow·미지원 fallback과 기존 socket drop 계측 공존.
- UDP 조각 누락/재순서·source 무효/역행/동일 시각·device-hour rollover·세션 재시작·ROS clock 점프.
- 메모리 상한·저장 종료·metadata queue 누락 진단, 기존 센서 header/MD5·Hz·지연의 회귀 없음.
- 정확한 source-time SLERP와 보간 불가 사유, 현재 정적 경로와의 품질·지원 범위 비교.

합성 영상도 단일 header가 세 뷰의 모든 입력 시각을 나타내지는 않는다. 뷰별 원본 header/관측 ID를
별도로 유지해야 하며 합성 header의 반복을 곧 동일 영상 재표시로 단정하지 않는다.

공용 메시지 추가와 bridge 구현은 다음 작업이다. 이번에는 분석과 문서만 반영했다.
