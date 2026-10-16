# P3-F `asmc_bridge` consolidated baseline

> 기준일: 2026-09-23
>
> 기계 판독 정본: [`config/bridge/bridge_profiles_v1.json`](../../../config/bridge/bridge_profiles_v1.json)
>
> 상태: **local 통합·회귀 승인**, Competition Vehicle Status live packet과 실제 두 PC LAN2LAN은 후속 gate

## 목적과 범위

P3-E에서 동결한 packet/topic 동작을 바꾸지 않고 다음 네 ROS package의 배포 경계를
`asmc_bridge` 하나로 합쳤다.

- `morai_udp_bridge`: Network UDP와 Ctrl
- `morai_sensor_bridge`: Camera/LiDAR/GPS/IMU UDP
- `vehicle_state`: GPS/IMU 기반 `/vehicle/state`
- `integration_launch`: 공동 component launch

runtime node 이름(`/morai_udp_bridge`, `/morai_sensor_bridge`,
`/gps_imu_vehicle_state_adapter`), ROS topic/type와 diagnostics topic 이름은 유지했다. 따라서
consumer는 package 내부 파일 경로를 참조하지 않는 한 변경할 필요가 없다. 이전 package는
source와 catkin package graph에서 제거했고 rollback은 중복 package를 상시 유지하지 않고 Git
이력으로 수행한다.

## 통합 결과

| 항목 | 이전 | P3-F |
|---|---:|---:|
| catkin package 수 | 16 | 12 |
| bridge 관련 package 수 | 4 | 1 |
| canonical launch | `asmc_bridge/bridge.launch` façade + 외부 component packages | 모든 구현·launch가 `asmc_bridge` 내부 |
| runtime node/topic 이름 | 기존 이름 | 변경 없음 |
| 기본 profile | `morai_udp + local + competition` | 변경 없음 |

소스는 역할별 디렉터리와 별도 node를 유지한다. 통합은 배포·dependency 단위의 통합이며,
Network/Sensor/VehicleState의 장애 경계나 diagnostics를 하나의 process로 합친 것이 아니다.

## 검증 결과

ROS Noetic container `asmc-ros-noetic`, MolitComp03 local v3 조건에서 확인했다.

| gate | 결과 |
|---|---|
| full `catkin_make -DCATKIN_ENABLE_TESTING=ON` | PASS, 12 packages |
| `catkin_make run_tests_asmc_bridge` | PASS, UDP 5 + sensor 4 tests |
| `vehicle_state_adapter_test` | PASS |
| 현재 12 package `catkin_test_results` | **559 tests, 0 errors, 0 failures** |
| `rospack find asmc_bridge` | `/root/ws/src/asmc_bridge` |
| 제거 package 4종 `rospack find` | 모두 absent |
| 실행 binary 경로 | 모두 `/root/ws/devel/lib/asmc_bridge/` |
| `/vehicle/state` | `valid=true`, `frame_id=map` |
| Ctrl closed loop | 정지 brake 1.0 → `/Ego_topic.brake=1.0` |

live diagnostics에서 front/left/right camera 약 20 Hz, extra camera 약 10 Hz, GPS 약 5 Hz,
IMU 약 42 Hz, LiDAR 완성 scan 약 12.2 Hz를 확인했다. v3 MORAI 설정과 손재호 소비 loop는
15 Hz이고, 실제 완성 scan 약 12 Hz는 승인된 동작이다. bridge는 없는 cloud를 복제하거나
재표본화하지 않는다.

컨테이너의 전체 `build/test_results`에는 삭제된 package와 과거 `hd_map_builder` 결과가 남아
있어 `--all`만 쓰면 663개로 과대 집계된다. 위 559개는 현재 `catkin_topological_order`의
12 package 결과만 다시 실행해 합산한 값이다. CTest XML 밖의
`asmc_bridge/vehicle_state_adapter_test`도 별도로 실행해 통과했다.

P3-E에서 만든 6.5초 임시 bag은 통합 전에 필수 runtime 9 topic과 유효 VehicleState 327개로
replay 검증했다. 통합은 동일 topic/type를 보존하고 parser/state test 및 post-move live smoke를
통과했다. 임시 bag 자체는 저장소 artifact로 복사하지 않는다.

## 남은 gate

1. MolitComp Competition Vehicle Status type 787, `9797→9798`은 참가팀 simulator에서 live
   packet이 나오지 않았다. 대회장 competition mode에서만 field와 181 B decoder를 교차 검증한다.
2. `local`은 승인됐다. `lan2lan`의 실제 두 PC, Windows/WSL mirrored networking, 방화벽,
   packet loss와 재시작 복구는 Phase 6에서 검증한다.
3. type 258 `7603→7502` TrafficLight status는 simulator oracle이며 production bridge 입력으로
   승격하지 않는다.

현재 실행·개발 계약은 [bridge package guide](../../../src/asmc_bridge/docs/integration.md)를 따른다.
P3-E의 측정 근거는 [local live baseline](./bridge-p3e-local-live-v1.md)에 보존한다.
