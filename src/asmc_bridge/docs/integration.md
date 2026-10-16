# MORAI bridge 개발·통합 가이드

> **문서 역할:** Active Package-Group Guide — 현재 bridge/state 구현과 Phase 3 통합 gate
>
> **현재 상태:** P3-G local closeout 완료. Network/Sensor/VehicleState/launch는 `asmc_bridge` 하나

Docker·MORAI UI·현재 포트로 실행하는 절차는 [운영 가이드](../../../docs/operations/README.md),
대회 허용 센서와 현장 조건은 [대회 환경](../../../docs/competition/README.md), 외부 message 호환은
[MORAI message 정책](../../../docs/contracts/simulator-morai-msgs-beta-policy.md)이 우선한다. 이 문서는
bridge 구현을 개발·검증하고 Phase 3에서 안전하게 통합하는 방법을 설명한다.

## 1. 현재 구현

| 역할 | 현재 경로 | 상태 |
|---|---|---|
| Network UDP와 Ctrl | `src/asmc_bridge/scripts/morai_udp_bridge.py` | 통합 완료 |
| Camera/LiDAR/GPS/IMU decode | `src/asmc_bridge/scripts/morai_sensor_bridge.py` | 통합 완료 |
| GPS/IMU→팀 내부 차량 상태 | `src/asmc_bridge/src/vehicle_state/` | 통합 완료 |
| component/canonical launch | `src/asmc_bridge/launch/` | 통합 완료, 기본 `/vehicle/state` 포함 |
| `local`/`lan2lan` 계약·MORAI JSON | P3-A 완료 | live LAN gate 대기 |
| Competition Vehicle Status decoder | P3-B 정적 계약·fixture 완료 | live capture gate 대기 |
| stream allowlist·sensor IP 파생본 | P3-C 구현 | live LAN gate 대기 |
| 단일 launch façade·vehicle state composition | P3-D 구현 | P3-F 물리 이전 완료 |
| local MolitComp03 수신·LiDAR frame gate | P3-E 실측 | 설정·소비 15 Hz 승인, Competition status 현장 대기 |
| 네 package→`asmc_bridge` | P3-F 완료 | full build·post-move live/정지 Ctrl smoke 통과 |
| local/field Competition gate | P3-G 완료 | local 선택, 현장 `--require-competition-status` 명시 |

P3-F는 node 이름, ROS topic/type, diagnostics 이름과 binary packet 계약을 유지하면서 구현을
한 패키지로 옮겼다. 공개 전체 실행 진입점은 canonical `bridge.launch` 하나이며, 이 파일이
Network/Sensor component와 기본 VehicleState를 직접 include한다. 단위 시험용 component launch는
보존하되 이전 `integration_launch`의 중간 조합 파일 `morai_udp_bridges.launch`는 제거했다.
이전 package 경로는 더 이상 runtime 계약이 아니다.

## 2. 현재 데이터 흐름

```text
MORAI Network UDP ──> morai_udp_bridge ──> /Ego_topic, /Competition_topic,
       │                                      /Object_topic, /CollisionData
       └─< /ctrl_cmd ───────────────────── Ctrl UDP

MORAI Sensor UDP ──> morai_sensor_bridge ──> camera, PointCloud2, /gps, /imu
                                               │
                                               └─> vehicle_state ─> /vehicle/state
```

`morai_msgs`는 현재 외부 packet adapter와 개발 호환 경계에 남아 있다. 판제·인지의 새 내부
interface는 가능한 한 `asmc_msgs`를 사용하되, Phase 3 전환 PR에서 producer와 consumer를 같이
검증한다.

### Network bridge

| 방향 | 현재 UDP | ROS topic·type | 비고 |
|---|---:|---|---|
| MORAI→팀 | Ego Host/Dest `9088/9099` | `/Ego_topic`, `morai_msgs/EgoVehicleStatus` | 현재 공통 상태 입력 |
| MORAI→팀 | MolitComp Competition `9797/9798`, type 787 | `/Competition_topic`, `morai_msgs/EgoVehicleStatus` | local 선택; 주최 현장 explicit gate·live emission 대기 |
| MORAI→팀 | TrafficLight `7603/7502`, type 258 | 발행하지 않음 | simulator oracle, production bridge 비소비 |
| MORAI→팀 | Object Dest `7505` | `/Object_topic`, `morai_msgs/ObjectStatusList` | privileged 개발 oracle |
| MORAI→팀 | Collision Dest `9092` | `/CollisionData`, `morai_msgs/CollisionData` | 충돌 때만 publish |
| 팀→MORAI | Ctrl Host `9093` | subscribe `/ctrl_cmd`, `morai_msgs/CtrlCmd` | Dest metadata 9094, mode 2, D gear packet |

Ego UDP velocity는 ROS에서 m/s로 변환한다. Object velocity는 현재 `morai_msgs` 관례에 따라
km/h다. Ctrl steering 입력은 radian이며 ±40°를 ±1로 정규화한다. 이 단위를 topic 이름만 보고
추정하지 않는다. 실측에서 `/Ego_topic.position`이 `(0,0)`인 대회 packet도 있었으므로 이를
authoritative map pose나 GPS/IMU fallback으로 사용하지 않는다.

TrafficLight type 258은 연속 status publisher이고 `GetTrafficLightStatus` 조회 서비스가 아니다.
검사한 MolitComp03/26.R1 export에는 MoraiTLServiceProvider type 778이 없으므로, bridge와
production planner는 GetTrafficLight를 필수 입력으로 요구하지 않는다. 26.R1의 gRPC proto가
존재해도 실행판의 API 제공 여부는 별도 live gate다.

### Sensor bridge

| 입력 | Dest UDP | 현재 ROS 출력 | frame |
|---|---:|---|---|
| RGB front/left/right/extra | `9291/9293/9295/9297` | 네 `CompressedImage` topic | `camera_*` |
| VLP16 intensity | `9299` | `/velodyne_points` | `velodyne` |
| GPS NMEA | `9301` | `/gps`, `/gps/fix` | `gps` |
| IMU | `9303` | `/imu` | `imu` |
| Semantic camera 3대 | `9305/9307/9309` | `/sem_*/image_jpeg/compressed` | 개발 GT 전용 |
| Instance LiDAR | `9311` | `/velodyne_points_instance` | 개발 GT 전용 |

기본 `competition` stream profile은 Object/Semantic/Instance socket과 publisher를 생성하지 않는다.
`development`는 이 입력을 추가하지만 local transport에서만 허용된다. CLI를 우회해
launch를 직접 실행해도 bridge node가 `lan2lan + development`를 거부한다.

VLP16은 기본 `lidar_frame_mode=azimuth`로 packet의 물리 회전 경계를 검출한다. 고정 75 packet
조립은 v3에서 약 8 Hz로 잘못 보였고, azimuth 조립 뒤 약 12 Hz로 측정됐다. 이는 프레이밍은
바로잡은 결과다. v3 설정과 손재호 소비 loop는 15 Hz이고 실제 완성 scan 약 12 Hz는 허용한다.
없는 scan을 복제하지 않으며 diagnostics의 `raw_packet_hz`, `packets_per_frame_median`,
`interval_hz`를 함께 확인한다.

GPS/IMU 원본 datagram은 기본값으로 `127.0.0.1:9401/9403`에도 복제된다. 이는 단일 MORAI UDP를
동시에 소비하던 standalone planner 호환 경로다. 현재 consumer가 이를 요구하는지 Phase 3
인벤토리에서 다시 확인하고, 사용 근거 없이 새 패키지의 필수 interface로 승격하지 않는다.

## 3. 시간·좌표계·TF 규칙

### Timestamp

- 기본 `timestamp_mode=receive`는 bridge가 완성된 message를 publish할 때 ROS 시간을 붙인다.
- `source`는 Ego/Object/Collision, camera, IMU와 Velodyne device time에만 적용한다.
- GPS NMEA에는 동일한 source timestamp가 없어 현재 receive time을 사용한다.
- source 값이 invalid/regression이면 단조 증가하는 receive time으로 fallback하고 diagnostics에
  `timestamp_fallbacks`, `timestamp_regressions`를 기록한다.
- camera/LiDAR는 datagram 한 개가 아니라 frame 조립 완료 뒤 한 message로 publish한다.

센서 융합 run은 모든 입력의 timestamp mode를 manifest에 기록한다. receive/source bag을 설명
없이 섞지 않는다.

### Coordinate frame

- GPS offset은 UTM52N에서 `(302595, 4124145) m`를 뺀 K-City `map` 좌표다.
- `GPSMessage.eastOffset/northOffset`에 같은 값을 넣어 consumer 변환과 일치시킨다.
- `base_link→velodyne` static TF는 기본 발행하며 v2/v3 공통 pose `(0.80, 0, 1.45) m`를 쓴다.
- raw GPS+latest IMU의 `map→base_link`는 `publish_map_tf=false`가 기본이다.
- localization이 authoritative `map→base_link`를 발행할 때 sensor bridge TF를 동시에 켜지 않는다.

## 4. 팀 내부 `/vehicle/state`

본선 후보 source는 GPS+IMU adapter다.

```bash
roslaunch asmc_bridge vehicle_state.launch
```

| 항목 | 현재 계약 |
|---|---|
| 입력 | `/gps`, `/imu` |
| 출력 | `/vehicle/state`, `asmc_msgs/VehicleState`, 50 Hz |
| frame | `map` |
| pose | UTM52N에서 GPS offset을 뺀 x/y, IMU quaternion yaw |
| speed | GPS position window + longitudinal IMU prediction/correction |
| freshness | GPS 0.50 s, IMU 0.30 s |
| invalid | 입력 누락·오류·stale이면 `valid=false` |

`vehicle_state_ego_dev.launch`는 `/Ego_topic`을 변환하는 개발 전용 source다. 두 launch를 동시에
켜면 같은 `/vehicle/state`에 publisher가 둘 생기므로 금지한다. 본선에서 Ego status를 GPS/IMU
fallback으로 자동 선택하지 않는다.

## 5. 실행과 관측

컨테이너에서 현재 개발 bridge를 실행한다.

```bash
cd /root/ws
source devel/setup.bash
./scripts/udp_bridge.sh
```

26r1과 대회판 모두 `morai_udp` runtime을 쓰므로 bridge 명령에서 simulator를 선택하지 않는다.
release별 MORAI import는 UI에서 선택하고, bridge에서는 transport만 선택한다.

현장 후보에서는 `--transport lan2lan`을 지정한다. 스크립트는 `asmc_bridge/bridge.launch`를
기동해 Network/Sensor bridge와 GPS/IMU `/vehicle/state`를 함께 연다. 두 release에 대응하는 generated
MORAI JSON 경로와 Ctrl 대상 IP를 출력하며 diagnostics에도 transport 이름을 남긴다.

수신 socket은 어느 profile에서도 같은 port의 `0.0.0.0`에 bind한다. 명시적 transport는 주로
Ctrl 송신 목적지를 `127.0.0.1` 또는 `192.168.0.1`로 결정하고 잘못된 현장 송신을 fail-closed로
막기 위한 것이다. 수신 sender를 보고 자동 선택하면 첫 packet 전 제어, mirrored/NAT와 다중
NIC에서 모호하므로 사용하지 않는다.

기본 local에서는 Competition 입력을 열어 두되 packet이 없으면 diagnostics를
`OK: optional; no data`로 낸다. 주최 현장판을 검사할 때만 다음처럼 필수화한다.

```bash
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
./scripts/docker_ros_up.sh verify-field 30 molitcomp03
```

`/vehicle/state`는 GPS+IMU에서 만든 독립 팀 메시지이므로 `/Competition_topic` 부재와 무관하다.

다른 터미널에서:

```bash
rostopic hz /Ego_topic /image_jpeg/compressed /velodyne_points /gps /imu
rostopic echo /morai_udp_bridge/diagnostics -n1
rostopic echo /morai_sensor_bridge/diagnostics -n1
rostopic echo /vehicle/state -n1
```

콘솔은 Network/Sensor 각각5초마다 줄바꿈한 요약 블록, diagnostics는1초 주기다. monotonic clock으로
주기를 관리하며 [`diagnostics.yaml`](../config/diagnostics.yaml)에서 조절한다.
diagnostics의 `interval_hz`, `last_message_age_sec`, `health`, parse/header error,
processing/assembly/output-gap p95와 timestamp fallback을 함께 본다. 수신 중단은 누적 수가
남아 있어도 `STALE`이며 socket/thread 실패는 `ERROR`다. 선택 입력의 최초 미수신은 `IDLE`이지만
알 수 없거나 손상된 헤더를 보내면 `INVALID`로 구분한다. 관측한 local 선택 입력
`#DetailEgoInfo$`(105B body·138B datagram·정상CRLF)는 `UNSUPPORTED`로 별도 집계하고
1회 INFO만 남긴다. `data_available=false`이며 decoder로 전달하지 않는다.
필수 gate 또는 LAN2LAN에는 이 예외를 적용하지 않는다.
콘솔 제목은 `[NETWORK]`, `[SENSORS]`이며 주기를 표시하지 않는다.
`RX`는 MORAI에서 수신, `TX`는 MORAI로 송신이다. 센서는 `cam1=19.4Hz`처럼
각 이름과 Hz를 표시하고, RGB / GPS·IMU·LiDAR / GT semantic을 줄별로 나눈다.
`err +N`, `drop +N`은 최근5초 증가량이다. 1초 diagnostics 발행은 매초 오류 로그를
출력한다는 뜻이 아니다. 상태 전환 경고와 반복 파싱·송신 오류의5초 throttle을 유지한다.
K-City 링크의 전체 ID는 Network diagnostics의 `last_link_id`에 보존한다.
LiDAR는 `raw_packet_hz`, `rotation_hz`(최근 구간의 방위각 경계),
`wall_rotation_hz`/`device_rotation_hz`(최근 최대4096회전 평균),
`packets_per_frame_median`, `socket_drops`, `discarded_revolutions`도 본다.
표본은 잠금 안에서 복사하고 집계·ROS 발행은 잠금 밖에서 한다. 통계 예외는 다음 주기에
재시도하며 `reporter_failures`로 남는다. Topic rate 하나만으로 Windows ingress, UDP loss,
decoder와 ROS scheduling 중 어디서 손실됐는지 단정하지 않는다.

## 6. build와 회귀 검사

ROS Noetic container에서:

```bash
./scripts/build_ws.sh
catkin_make run_tests_asmc_bridge
./devel/lib/asmc_bridge/vehicle_state_adapter_test
```

이전에 `CATKIN_WHITELIST_PACKAGES`로 일부 package만 configure한 workspace라면 먼저 whitelist를
비우고 full build한다. 그렇지 않으면 `catkin_make --pkg ...`가 source가 정상이어도 “No rule to
make target”으로 실패할 수 있다.

```bash
./scripts/build_ws.sh -DCATKIN_WHITELIST_PACKAGES=""
```

변경 범위별 최소 검사는 다음과 같다.

| 변경 | 최소 gate |
|---|---|
| UDP parser/packet | 정상·잘린·다른 버전 fixture, parse error 0, 기존 bag replay |
| sensor assembly | camera fragment 순서, LiDAR field/type/frame, timeout partial frame |
| timestamp | receive/source/fallback/regression과 fusion skew |
| vehicle state | 정속·가감속·저속·정지·outlier·stale |
| port/profile | bind 충돌, packet 방향, topic rate, Ctrl 실제 응답 |
| competition launch | Object/Semantic/Instance/gRPC/rosbridge 미기동 검사 |

실제 rate·latency 값은 [transport 기록](../../../docs/records/bridge/runtime-transport-2026-09.md)에
날짜와 환경을 붙여 남긴다. 특정 측정값을 package guide의 영구 성능으로 복사하지 않는다.

## 7. Phase 3 계약 경계

계약 조합은 `runtime_protocol × simulator_release × transport_profile × sensor_set`이다.

| 축 | 예 | 책임 |
|---|---|---|
| runtime | `morai_udp` | 공통 packet adapter와 ROS topic |
| simulator release | `26r1`, `molitcomp03` | MORAI import schema와 provenance |
| transport | `local`, `lan2lan` | IP, port, direction, allowlist |
| sensor | `ASMC_sensor_set_v1/v2/v3/GT` | pose, rate와 센서 종류 |

확정된 목표:

- 개발·현장의 채널과 포트는 같고 IP만 profile로 분리한다.
- Ctrl `9093/9094`, mode 2
- MolitComp Competition Vehicle Status `9797/9798`, type 787
- TrafficLight status `7603/7502`, type 258은 simulator oracle이며 bridge 입력이 아님
- GetTrafficLight/MoraiTLServiceProvider type 778은 조사한 export에 없고 필수 계약이 아님
- CollisionData `9091/9092`
- Autonav v2 sensor `9290~9303`
- 개발 IP 양쪽 `127.0.0.1`
- 현장 MORAI `192.168.0.1`, Autonav `192.168.0.10`
- VLP16은 MORAI UI 저장본과 손재호 소비 loop를 15 Hz로 유지한다. 실제 완성 scan 약 12 Hz는
  허용하며 중복 재발행하지 않는다.

P3-A/P3-B 산출물은 [`config/bridge`](../../../config/bridge)와
[`config/morai`](../../../config/morai)에 있다. 26r1/대회판 raw export는 entry 수와
`PUBSUB_TYPE`이 달라 template을 분리하고, transport별 IP는 generator가 일괄 적용한다.

JSON의 `sensorPeriod`만 추측해 고치지 않는다. 10 Hz baseline과 손재호 tracking 결과, packet
loss, processing latency와 CPU/GPU를 비교한다. 현장 실제 두 PC, firewall·route·restart·offline
복구는 Phase 6 gate이며 개발 PC loopback 성공으로 대신하지 않는다.

## 8. P3-G local closeout 결과와 남은 gate

1. package/topic/port/timestamp baseline과 local live 결과를 P3-E에서 캡처했다.
2. parser, sensor decoder, state core와 fixture/test를 동작 변경 없이 `asmc_bridge`로 옮겼다.
3. planning launch dependency도 `asmc_bridge`로 전환했다.
4. full build, local live topic과 정지 Ctrl post-move smoke를 통과해 P3-F를 닫았다.
5. 참가팀 local은 `/Competition_topic`을 선택 사항으로 닫고, 주최판에서만 명시적으로
   필수화하는 P3-G 계약과 회귀 검사를 추가했다.
6. Competition Vehicle Status 실제 packet과 두 PC LAN2LAN은 별도 현장 gate로 남긴다.

P3-G HEAD의 현재 graph는 `674 tests, 0 errors, 0 failures`이며 `asmc_bridge` 자체는 14개다.

단일 패키지 안에서도 Network/Sensor/VehicleState는 별도 node·소스 디렉터리·테스트로 실패
경계를 유지한다. 배포 단위만 하나이며 diagnostics topic 이름도 기존 소비자 호환을 보존한다.
물리 통합 증거는 [P3-F baseline](../../../docs/records/bridge/bridge-p3f-consolidated-v1.md), local
closeout 판단은 [P3-G baseline](../../../docs/records/bridge/bridge-p3g-local-closeout-v1.md)에 보존한다.
