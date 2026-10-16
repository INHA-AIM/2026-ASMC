# asmc_bridge

MORAI Network UDP, Sensor UDP와 GPS/IMU 기반 `/vehicle/state`를 소유하는 단일 ROS1
패키지다. P3-F에서 `morai_udp_bridge`, `morai_sensor_bridge`, `vehicle_state`,
`integration_launch`의 구현을 이 패키지로 물리 통합했다. node와 diagnostics 이름은 소비자
호환을 위해 그대로 유지한다.

```bash
./scripts/udp_bridge.sh
# 같은 canonical launch를 직접 실행할 수도 있다.
roslaunch asmc_bridge bridge.launch
```

기본값은 `morai_udp + local + competition`이며 `/vehicle/state`까지 발행한다. simulator
release는 runtime 인자가 아니라 MORAI UI에 import한 network JSON schema로만 구분한다.
`local`과 `lan2lan`은 수신 포트가 같고 Ctrl 목적지 IP만 다르므로 현장에서만
`--transport lan2lan`을 명시한다.

일반 실행용 공개 진입점은 `bridge.launch` 하나다. 구성요소 단독 시험이나 다른 launch에서의
선택적 include를 위해 다음 component launch를 같은 패키지 안에 둔다.

- `morai_udp_bridge.launch`: Ego/Competition/Collision, `/ctrl_cmd`
- `morai_sensor_bridge.launch`: RGB/LiDAR/GPS/IMU
- `vehicle_state.launch`: GPS/IMU→`asmc_msgs/VehicleState`
- `vehicle_state_ego_dev.launch`: 개발 전용 EgoStatus state source

이전 `integration_launch`에서 옮겨 온 중간 조합 파일 `morai_udp_bridges.launch`는 제거했다.
`bridge.launch`가 Network/Sensor component와 기본 VehicleState를 직접 include하므로 전체 실행에
추가 조합 계층이 필요하지 않다.

Object/Semantic/Instance oracle은 `./scripts/udp_bridge.sh --gt`로만 열 수 있고
LAN2LAN과 함께 사용할 수 없다. `/vehicle/state` authoritative publisher는 하나만 실행한다.
`--streams`와 `ASMC_STREAMS`는 제거했다. 인자 없는 실행은 일반 입력만 받고, `--gt`는
내부 `development` 프로필을 선택한다. 내부 `competition/development` 이름은 MORAI 버전 구분이 아니다.
GT 센서셋의 Intensity·Instance LiDAR 모두15Hz이며, 변경된 JSON을 MORAI UI에서 다시 로드해야 한다.
Bridge의 기본 `lidar_rotation_rate=15`는 조립 참고값이고 실제 센서 송신 주기를 바꾸지 않는다.

v3는 MORAI 저장 설정과 손재호 소비 loop가 15 Hz이며 완성 scan 약 12 Hz 입력은 허용한다.
bridge는 azimuth wrap으로 실제 회전을 조립하고 중복 cloud를 만들지 않는다. MolitComp
Competition Vehicle Status는 type 787 `9797→9798`이고, type 258 `7603→7502`는
TrafficLight oracle이라 production bridge에서 소비하지 않는다.

참가팀 local에서는 type 787 packet이 나오지 않아도 정상이다. decoder와
`/Competition_topic` publisher는 유지하며, 주최 현장판을 검사할 때만 다음처럼 필수화한다.

```bash
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
./scripts/docker_ros_up.sh verify-field 30 molitcomp03
```

기본 diagnostics는 미수신 Competition 입력을 `optional; no data`로 표시한다.

콘솔은 Network와 Sensor가 각각 **5초마다 줄바꿈한 요약 블록**으로 출력한다.
`RX`는 MORAI에서 수신, `TX`는 MORAI로 송신이며 현재 링크도 함께 표시한다.
Sensor는 `cam1=19.4Hz`처럼 센서마다 이름·단위를 표시하고, RGB와 GPS/IMU/LiDAR,
GT semantic을 줄별로 나눈다. 제목에는 보고 주기를 표시하지 않는다. `state`는 현재 문제,
`err +N`과 `drop +N`은 이번5초의 오류·socket drop 증가량이다. 누적 수와 상세 통계는
diagnostics에 보존한다. K-City의 `A2256W000126`은 콘솔에서 `A126`으로 줄이고 원본은
Network diagnostics의 `last_link_id`에 보존한다.
`/morai_udp_bridge/diagnostics`, `/morai_sensor_bridge/diagnostics`는 **1초마다** 발행하며,
누적 메시지 수·마지막 수신 age·처리/조립/출력 공백·LiDAR 회전율을 보존한다.
주기는 ROS clock 정지와 무관한 monotonic clock을 사용한다.
1초는 상태 토픽의 발행 주기이며 오류 메시지를 매초 반복하는 설정이 아니다.
상태 알림은 장애·복구 전환에 출력하고, 반복 파싱·송신 오류 로그는5초 throttle을 유지한다.

정본 설정은 [`config/diagnostics.yaml`](config/diagnostics.yaml)이다. 수신 age가 기본0.5초,
GPS1.0초, IMU0.3초를 넘으면 `STALE`; 최초 미수신은 `WAITING`, 선택 입력의 최초 미수신은
`IDLE`, 잘못된 입력은 `INVALID`, 수신 스레드 실패는 `ERROR`다. 최근 파싱/송신 오류 또는
socket drop은 `DEGRADED`로 표시한다. 이 진단 기준은 차량 상태·fusion의 유효성 기준을 바꾸지 않는다.
통계는 잠금 안에서 제한된 표본을 복사한 뒤 잠금 밖에서 집계한다. 진단 callback 실패는
오류 로그와 `reporter_failures`에 남기고 다음 주기에 재시도한다.

26.R1 export에는 Competition(type787)이 없다. 참가팀 MolitComp local에서 관측한
`#DetailEgoInfo$`(본문105B·전체138B)는 기존 `#MoraiInfo$`(본문152B)와 다른 형식이다.
Local의 선택 입력일 때만 이 정상 길이·CRLF envelope를1회 INFO로 안내하고,
`UNSUPPORTED`·`unsupported_packets`·`data_available=false`로 diagnostics에 남긴다.
이를 수신 성공으로 세거나 `/Competition_topic`으로 변환하지 않는다. 반복 콘솔 오류 합계에서는
제외한다. 알 수 없는 형식·손상된 envelope·필수 입력 설정·LAN2LAN에서는 기존 경고를 유지한다.
자세한 형식 구분과 새 출력 예시는 [로그 정리 기록](../../docs/records/bridge/2026-10-17-bridge-log-readability.md)을 따른다.
최종 줄바꿈·센서별 출력은 [콘솔 배치 기록](../../docs/records/bridge/2026-10-17-bridge-console-layout.md)을 따른다.

LiDAR 저하를 구분할 때는 `rotation_hz`, `wall_rotation_hz`, `device_rotation_hz`와
실제 PointCloud `interval_hz`, `socket_drops`, `discarded_revolutions`를 함께 확인한다.
Hz는 각각 최근 보고 구간, 지연·패킷 중앙값·wall/device 회전 평균은 최근 최대4096표본이며
집계 범위가 다르다. `socket_drops=0`도 Windows 송신과 Docker Desktop 이전 구간의 무손실을
보장하지 않는다. [2026-10-17 확인 결과](../../docs/records/bridge/2026-10-17-lidar-cadence-diagnostics.md)를 참고한다.
