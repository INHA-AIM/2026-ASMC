# P3-E MORAI competition local live baseline

> 기준일: 2026-09-23
>
> 기계 판독 정본: [`config/bridge/bridge_profiles_v1.json`](../../../config/bridge/bridge_profiles_v1.json)
>
> 상태: local 실측 완료, LiDAR 설정·소비 주기와 일반 runtime은 **승인**, Competition Vehicle Status만 현장 확인 대기

## 시험 조건

- simulator: 25.S4.MolitComp03 대회판
- network import: `ASMC_network_set_molitcomp03_local.json`
- sensor import: `ASMC_sensor_set_v3_local.json`
- runtime: Docker Desktop host network, `morai_udp + local + competition`
- canonical entrypoint: `./scripts/udp_bridge.sh`

`9111/9112`는 사용하지 않았다. Ctrl 계약과 양쪽 MORAI import의 현재값은
Host/Destination `9093/9094`, mode 2다.

## 관측 결과

| 항목 | 관측 | 판정 |
|---|---:|---|
| front/left/right RGB | 약 20 Hz | 수신 |
| extra RGB | 약 10 Hz | 수신 |
| GPS | 약 5 Hz | 수신 |
| IMU | 약 42~44 Hz | 수신, nominal 50 Hz 미만 |
| `/vehicle/state` | 약 50 Hz, 표본 모두 `valid=true` | 수신 |
| VLP16 raw UDP | 약 600 packet/s | 수신 |
| VLP16 packets/revolution | 50 | 물리 회전 경계 확인 |
| 기존 75 packet 조립 출력 | 약 7.86 Hz | 잘못된 v3 프레이밍 |
| azimuth 조립 출력 | 약 12.01 Hz | 프레이밍 수정, 허용된 실제 입력률 |
| 손재호 LiDAR 소비 loop | `ros::Rate(15)`, `Vehicle.LiDAR.Hz=15` | 요구 충족 |
| Ctrl closed loop | 정지 brake 1.0 송신 후 Ego brake 1.0 | `9093` 왕복 승인 |
| runtime bag replay | 6.5초, 1,700 message, 필수 9 topic | 별도 ROS master 검증 통과 |
| UDP destination 7502, type 258 | `#TrafficLight$` | TrafficLight oracle, bridge 비소비 |
| UDP destination 9798, type 787 | 8초간 0 packet | Competition mode live emission 대기 |
| UDP destination 911, type 770 | `#MoraiInfo$`, 229 B, 약 45 Hz | legacy MoraiInfo stream 확인 |

Network를 bridge 기동 뒤 다시 Connect한 표본에서는 Ego/Collision이 약 45.8 Hz로 들어왔다.
별도 정지 표본에서는 IMU quaternion이 `(0,0,0,0)`으로 송신돼 `/vehicle/state`가
`valid=false`였는데, adapter가 invalid quaternion을 거부한 정상 동작이다. 시뮬 Play 상태의
유효 quaternion 표본과 혼동하지 않는다.

LiDAR device timestamp와 wall time으로 계산한 회전율은 모두 약 11.85~12.1 Hz다. v3 JSON에는
`sensorPeriod≈0.06666667`, `frequencyIndex=2`가 저장돼 있고, 손재호 코드는
`Vehicle.LiDAR.Hz=15`와 `ros::Rate(15)`로 callback을 처리한다. 팀 요구는 이 설정·소비 주기가
15 Hz인 것이며 완성 scan이 약 12 Hz로 들어오는 것은 허용된다. 따라서 없는 scan을 복제해
15 Hz로 재발행하지 않는다. 같은 timestamp를 반복 처리하지 않는 기존 보호도 유지한다.

`/vehicle/state`의 기동 직후 `valid=false`는 5 Hz GPS 두 표본으로 속도 baseline을 만들기 전의
정상 warm-up이었다. 유효 GPS/IMU 뒤에는 약 50 Hz, `valid=true`로 안정됐다. 이 정상 상태를
고장처럼 경고하지 않도록 1초 startup grace를 두되, 이후 invalid/stale은 계속 경고한다.

Ctrl은 차량 정지 상태에서 `/ctrl_cmd`의 brake 1.0을 약 2초 보냈고, bridge diagnostics의
송신 count와 `/Ego_topic.brake=1.0`을 모두 확인했다. 이는 ROS→UDP 9093→MORAI→Ego UDP→ROS
왕복을 검증한 것이다. 이어서 참가팀 runtime 토픽을 6.5초 bag(1,700 message)으로 임시
기록해 별도 ROS master에서 재생했고, 카메라 4개·LiDAR·GPS·IMU·Ego·`/vehicle/state` 필수
9개 토픽과 유효 VehicleState 327개를 소비자 검증기가 모두 통과했다. 검증 bag은 기준 데이터로
복사하지 않고 재현 명령과 결과만 남긴다.

## 적용한 런타임 보정

`morai_sensor_bridge`는 VLP16을 고정 packet 수가 아니라 azimuth wrap으로 한 물리 회전씩
조립한다. 시작 시 첫 partial revolution은 버리고 다음 wrap부터 동기화한다. diagnostics에는
LiDAR `raw_packet_hz`와 `packets_per_frame_median`을 추가했다. 회귀가 필요할 때만
`lidar_frame_mode:=packet_count`를 명시한다.

transport 선택도 보정했다. Compose의 전역 `MORAI_CMD_IP=127.0.0.1`을 제거했고,
`udp_bridge.sh`가 `local→127.0.0.1`, `lan2lan→192.168.0.1`로 Ctrl 목적지를 결정한다.
수신 socket은 두 transport 모두 동일 port의 `0.0.0.0`에 bind한다.

## 남은 gate

1. MolitComp03 Competition Vehicle Status는 현장·팀 export의 type 787,
   `9797→9798`이다. Competition mode에서 packet을 실제 발생시켜 181 B 계약과 field를
   교차 검증한다. type 258, `7603→7502`는 TrafficLight status라 이 gate에 사용할 수 없다.
2. Network publisher는 bridge 재기동 뒤 MORAI UI에서 Connect/reload가 필요할 수 있다.
   센서 수신만으로 Ego/Collision network channel까지 정상이라고 판정하지 않는다.
3. 실제 두 PC LAN2LAN·방화벽·재시작 복구는 Phase 6에서 확인한다.

## 재검증 명령

```bash
./scripts/udp_bridge.sh

python3 scripts/verify_morai_topics.py \
  --duration 30 \
  --required-topic /velodyne_points

rostopic echo /morai_sensor_bridge/diagnostics -n1
rostopic echo /vehicle/state -n1
```

`--lidar-target-hz`는 정확한 도착률을 별도로 시험할 때만 쓰며, v3의 설정·소비 15 Hz 요구를
검증하는 옵션이 아니다. Competition status가 해결되기 전에는 MolitComp03 전체 verifier의
해당 실패를 낮춰서 통과시키지 않는다. 개발 중에는 participant runtime 9개 토픽을 명시해
나머지 경로를 독립 검증한다.

## Network export 비교

IP를 제외한 `PUBSUB_TYPE / Division_TYPE`를 비교했다.

| export | entry | runtime에 중요한 차이 |
|---|---:|---|
| 주최 PC `2026_molit_comp_network.json` | 10 | Ctrl mode 2, type 787 Competition, type 258 TrafficLight 포함 |
| 팀 `ASMC_network_set_molitcomp03_local_dev.json` | 12 | 위 10종 + type 266 SensorSync, type 783 MoraiSL subscriber; 원본 Ctrl mode 1 |
| 팀 `ASMC_26r1_network_set_local_v2.json` | 23 | Ctrl type 786, Ego type 770; type 787 없음; sync/service 12종 + 별도 gRPC type 4865 |

MolitComp 두 파일의 공통 type은 `771, 261, 267, 295, 770, 772, 258, 787, 259, 780`이다.
팀 local dev의 추가 `266/783`은 개발 제어용이며 LAN allowlist에 넣지 않는다. 주최 파일의
TrafficLight publisher/control과 event service가 존재한다는 사실은 참가팀 production 입력
허용을 뜻하지 않는다. type 258 TrafficLight Status publisher는
`GetTrafficLightStatus`/type 778 MoraiTLServiceProvider 조회 서비스가 아니다. 세 export 모두
type 778이 없고 참가팀 MolitComp UI에도 GetTrafficLight 항목이 없었다. 26.R1의 별도 gRPC
type 4865와 proto 존재 역시 해당 release가 GetTrafficLight query를 실제 제공한다는 증거가
아니다. bridge는 두 기능을 모두 필수 입력에서 제외하며 type 258 비소비 정책을 강제한다.

IP를 제외한 세부 종류는 아래와 같다.

| export | publisher (Division 1) | subscriber (2) | controller (3) | service (4) | gRPC ego |
|---|---|---|---|---|---|
| 주최 MolitComp03 | 258, 261, 267, 295, 787, 770, 772 | 259 | 771 | 780 | 없음 |
| 팀 MolitComp03 | 동일 | 259, 266, 783 | 771 | 780 | 없음 |
| 팀 26.R1 | 258, 261, 265, 263, 267, 271, 770, 772, 779 | 259, 262, 264, 266, 776, 783 | 786 | 269, 270, 274, 275, 780, 782 | 4865 |

MolitComp 두 원본의 공통 종류는 같지만 TF2(type 267)가 주최측에서 disabled, 팀 local에서
UDP enabled이고 Ctrl mode가 각각 2/1이다. 생성본은 release별 구조를 보존하면서 Ctrl만 mode
2로 정규화한다.

26r1과 MolitComp는 Network export schema가 달라 JSON template을 하나로 합치지 않는다.
대신 ROS bridge는 공통 binary UDP 구현 하나를 사용한다. renderer는 `Division_TYPE=3` Ctrl을
release별 type 786/771과 무관하게 `9093/9094 mode 2`로 만들고, Ego는 release별 type
770/295를 공통 `9088/9099`로 정규화한다. type 787 Competition은 MolitComp에서만 생성한다.

검증기는 release별 필수 토픽을 구분한다. MolitComp03은 `/Competition_topic`을 요구하고,
type 787이 없는 26.R1은 요구하지 않는다.

```bash
./scripts/docker_ros_up.sh verify 8 molitcomp03
./scripts/docker_ros_up.sh verify 8 26r1
```

이 인자는 bridge runtime을 분기하지 않고 verifier inventory만 선택한다.
