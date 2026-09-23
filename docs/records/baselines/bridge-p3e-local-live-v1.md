# P3-E MORAI competition local live baseline

> 기준일: 2026-09-23
>
> 기계 판독 정본: [`config/bridge/bridge_profiles_v1.json`](../../../config/bridge/bridge_profiles_v1.json)
>
> 상태: local 실측 완료, LiDAR 15 Hz와 Competition Vehicle Status는 **미승인**

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
| azimuth 조립 출력 | 약 12.01 Hz | 프레이밍 수정, 15 Hz gate 실패 |
| UDP destination 7502, type 258 | `#TrafficLight$` | TrafficLight oracle, bridge 비소비 |
| UDP destination 9798, type 787 | 8초간 0 packet | Competition mode live emission 대기 |
| UDP destination 911, type 770 | `#MoraiInfo$`, 229 B, 약 45 Hz | legacy MoraiInfo stream 확인 |

Network를 bridge 기동 뒤 다시 Connect한 표본에서는 Ego/Collision이 약 45.8 Hz로 들어왔다.
별도 정지 표본에서는 IMU quaternion이 `(0,0,0,0)`으로 송신돼 `/vehicle/state`가
`valid=false`였는데, adapter가 invalid quaternion을 거부한 정상 동작이다. 시뮬 Play 상태의
유효 quaternion 표본과 혼동하지 않는다.

LiDAR device timestamp와 wall time으로 계산한 회전율이 모두 약 11.85~12.1 Hz여서 단순한
ROS publish 지연이나 wall-clock slowdown으로 설명되지 않는다. v3 JSON에는
`sensorPeriod≈0.06666667`, `frequencyIndex=2`가 저장돼 있지만, 이 실행의 MORAI 원 UDP는
14~16 Hz를 만들지 않았다. 따라서 v3를 15 Hz 승인으로 표시하지 않는다.

## 적용한 런타임 보정

`morai_sensor_bridge`는 VLP16을 고정 packet 수가 아니라 azimuth wrap으로 한 물리 회전씩
조립한다. 시작 시 첫 partial revolution은 버리고 다음 wrap부터 동기화한다. diagnostics에는
LiDAR `raw_packet_hz`와 `packets_per_frame_median`을 추가했다. 회귀가 필요할 때만
`lidar_frame_mode:=packet_count`를 명시한다.

transport 선택도 보정했다. Compose의 전역 `MORAI_CMD_IP=127.0.0.1`을 제거했고,
`udp_bridge.sh`가 `local→127.0.0.1`, `lan2lan→192.168.0.1`로 Ctrl 목적지를 결정한다.
수신 socket은 두 transport 모두 동일 port의 `0.0.0.0`에 bind한다.

## 미해결 gate

1. MORAI UI에서 v3를 다시 15 Hz로 선택·저장한 export와 실제 raw revolution rate를 비교한다.
2. `/velodyne_points`를 30초 이상 측정해 평균 14~16 Hz를 통과시킨다.
3. 10 Hz 대비 손재호 tracking, latency, CPU와 packet 오류를 비교한다.
4. MolitComp03 Competition Vehicle Status는 현장·팀 export의 type 787,
   `9797→9798`이다. Competition mode에서 packet을 실제 발생시켜 181 B 계약과 field를
   교차 검증한다. type 258, `7603→7502`는 TrafficLight status라 이 gate에 사용할 수 없다.
5. Network publisher는 bridge 재기동 뒤 MORAI UI에서 Connect/reload가 필요할 수 있다.
   센서 수신만으로 Ego/Collision network channel까지 정상이라고 판정하지 않는다.
6. `/ctrl_cmd`를 안전한 정지 상태에서 publish해 `9093` 송신과 실제 차량 응답을 확인한다.

## 재검증 명령

```bash
./scripts/udp_bridge.sh

python3 scripts/verify_morai_topics.py \
  --duration 30 \
  --required-topic /velodyne_points \
  --lidar-target-hz 15 \
  --strict-thresholds

rostopic echo /morai_sensor_bridge/diagnostics -n1
rostopic echo /vehicle/state -n1
```

Competition status가 해결되기 전에는 전체 `competition` verifier 실패를 낮춰서 통과시키지
않는다. 미확인 채널을 선택 항목으로 바꾸는 대신 배포 차단 상태를 유지한다.

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
