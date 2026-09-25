# Planning/Control 개발 가이드

> **문서 역할:** Active Package-Group Guide — 현재 Behavior·Frenet·MPC 경계와 변경 gate
>
> **현재 상태:** MORAI 외부 경계는 UDP, 팀 PC 내부 Behavior→Frenet→MPC는 ROS1/catkin

실행 순서와 관측 명령은 [Planning/Control 운영 규약](../../operations/planning-control.md),
공용 message와 topic은 [ROS interface](../../../src/interfaces/README.md), FSM↔Frenet v2 세부
계약은 [v2 frozen contract](../../contracts/planning/fsm-frenet-v2.md)를 우선한다. 이 문서는
코드를 수정할 때 package 책임과 아직 닫히지 않은 통합 경계를 설명한다.

## 1. 현재 구현 지도

| 역할 | 현재 경로 | runtime |
|---|---|---|
| Behavior/FSM | `src/behavior_planner/` | ROS node, v2 command가 현재 경로; v1 호환 가능 |
| Frenet trajectory | `src/frenet_planner/` | ROS node + transport-independent C++ core |
| MPC control | `src/mpc_controller/` | ROS node + transport-independent C++ core |
| state source | `src/asmc_bridge/src/vehicle_state/` | GPS/IMU→`asmc_msgs/VehicleState` |
| MORAI bridge | `src/asmc_bridge/` | 외부 UDP↔내부 ROS |
| camera lane pilot | `src/lane_boundary_adapter/` | read-only ROS debug, 제어 미연결 |
| bridge-only launch | `src/asmc_bridge/launch/` | planning launch가 아니라 bridge 공동 기동 |

`lane_boundary_adapter`는 이름 그대로 perception→planning read-only adapter이며
Behavior/Frenet/MPC를 감싸는 `planning_control` package가 아니다. 세 planning package를
임의로 이동하거나 이름을 바꾸지 않는다.

## 2. 현재 데이터 흐름

```text
MORAI sensors/status --UDP--> bridges --> /vehicle/state
                                      └-> /Object_topic (development oracle)

/vehicle/state + /Object_topic + /traffic_light/state
  --> Behavior(v2) --> /behavior/command_v2
                         │
/perception/objects ------┴--> Frenet --> /planning/trajectory
                                      └-> /planning/frenet_feedback_v2
                                               │
/vehicle/state + trajectory + command ----------┴--> MPC --> /ctrl_cmd
                                                               │
                                                               └-> UDP bridge --> MORAI
```

현재 `autonomy_no_bridges_v2.launch`의 object adapter는 개발용 `/Object_topic`을
`/perception/objects`로 변환한다. Behavior도 아직 `morai_msgs/ObjectStatusList`를 직접
구독한다. 이는 sensor-only 본선 경계가 아니며 LiDAR perception producer로 교체하기 전까지
competition-ready라고 부르지 않는다.

## 3. 역할과 권한

### Behavior/FSM

- route·rule·signal·object context에서 maneuver와 target을 결정한다.
- v2 `command_id`, PREPARE/EXECUTE/CANCEL transaction을 소유한다.
- trajectory 생성, collision candidate 평가와 actuator 명령을 소유하지 않는다.
- 현재 local `link_set.json`, `node_set.json`, `route_links_1.yaml`을 읽는다.

완성된 `/hd_map/*`를 직접 소비하는 전환은 아직 구현되지 않았다. 로컬 map copy와
`asmc_hd_map` 정본을 자동으로 같은 것으로 가정하지 않는다.

### Frenet

- Behavior가 정한 maneuver·target을 검증하고 trajectory 후보를 생성·평가한다.
- `/vehicle/state`, `/perception/objects`와 v2 command를 source time/freshness와 함께 검사한다.
- KEEP, STOP, EMERGENCY_STOP, FOLLOW, AVOID, LANE_CHANGE gate가 열려 있다.
- MERGE는 아직 production gate가 닫혀 있다.
- v1은 v2 command가 도착하지 않은 cycle의 호환 경로다.

### MPC

- 최신 `/vehicle/state`를 유일한 현재 x0로 사용한다.
- `Trajectory.ego_at_plan`은 planning provenance이며 현재 state로 재사용하지 않는다.
- trajectory·vehicle state stale, invalid reference와 solver failure는 STOP으로 수렴한다.
- EMERGENCY_STOP command는 controller emergency brake override를 활성화한다.

## 4. 현재 contract와 legacy를 구분한다

| 경계 | 현재 v2 | 호환·기록 |
|---|---|---|
| Behavior→Frenet | `/behavior/command_v2`, `BehaviorCommand` | `/behavior/context`, `BehaviorContext` v1 |
| Frenet→Behavior | `/planning/frenet_feedback_v2`, `FrenetFeedback` | `/planning/feedback`, `PlanFeedback` v1 |
| Frenet→MPC | `/planning/trajectory`, `Trajectory` | standalone UDP `:9300`은 종료된 baseline |
| MPC→Bridge | `/ctrl_cmd`, `morai_msgs/CtrlCmd` | 직접 Ctrl UDP는 종료된 baseline |

v1 handoff의 상세 설계는 [legacy behavior handoff](../../handoffs/planning/behavior-legacy.md),
standalone UDP 시절 내용은 [archived baseline](../../archive/planning/standalone-udp-baseline.md)에
보존한다. 현재 코드에 두 기록의 경로나 executable 이름을 적용하지 않는다.

## 5. 좌표·시간·상태 소유권

- `VehicleState`, object와 trajectory는 `map` frame, SI 단위를 사용한다.
- `header.stamp`는 callback time이 아니라 source/reference time이다.
- freshness watchdog은 `ros::SteadyTime` receive age로 계산하며 source-time 보정과 섞지 않는다.
- Behavior가 WHAT/WHERE, Frenet이 HOW, MPC가 actuator 추종을 소유한다.
- 같은 `/vehicle/state`나 `map→base_link`의 authoritative producer를 둘 이상 켜지 않는다.
- core는 ROS message·callback·UDP를 알지 않으며 node boundary converter에서 변환한다.

상세 구조·dependency·리팩토링 원칙은 [refactoring guide](./refactoring.md)를 따른다.

## 6. 개발 launch의 의미

| launch | 용도 | 제한 |
|---|---|---|
| `behavior_planner/autonomy_no_bridges_v2.launch` | 현재 v2 full planning chain 개발 | bridge 별도, `/Object_topic` oracle adapter 사용 |
| `behavior_planner/autonomy_no_bridges.launch` | v1·empty-object 호환 smoke | production v2 아님 |
| `frenet_planner/keep_morai_no_obstacles.launch` | obstacle-free KEEP E2E | perception 미사용 |
| package 단독 launch | component build/topic/fail-safe 확인 | full-stack 검증 아님 |

어느 launch도 현재 본선 sensor-only 배포 완성을 의미하지 않는다. bridge profile, 실제
LiDAR object producer, HD Map 정본, mission orchestration과 closed-loop full course를 함께
검증해야 한다.

## 7. Camera lane handoff

`lane_boundary_adapter`는 `/perception/lane/*`를 Marker와 diagnostics로만 변환한다.
`BehaviorCommand`, `Trajectory`, `/ctrl_cmd`를 발행하지 않으며 camera lane에 제어 권한을
부여하지 않는다. 음영구간 소비 조건은
[shadow camera handoff](../../handoffs/perception/shadow-camera.md), adapter 구현은
[package README](../../../src/lane_boundary_adapter/README.md)를 따른다.

이 package는 음영구간 판제 개발을 위한 임시 read-only reference 경계다. 판제가
canonical `LaneBoundaryFamily`·camera drivable BEV를 직접 소비하게 되면
message→strip core, freshness/invalid 규칙, 설정과 테스트를 판제 소유 모듈에
흡수한다. 동일 bag parity, feature-off/on closed-loop, fallback·rollback과 외부 참조 0을
확인한 뒤에만 standalone `lane_boundary_adapter` package를 별도 PR에서 삭제한다.
흡수 전까지는 현 package를 production control source로 승격하지 않는다.

## 8. 변경 gate

| 변경 | 최소 확인 |
|---|---|
| message/topic | `asmc_msgs` MD5 영향, producer+consumer build, v1/v2 선택권 |
| Behavior policy | command lifecycle, stale feedback, route/signal/object scenario |
| Frenet core | 해당 maneuver unit test, collision/kinematic/fail-safe regression |
| MPC core | converter, timing, reference validation, controller regression |
| map 연동 | local map copy와 `/hd_map/*` link/route parity, station·frame 검증 |
| object source | ID·class·OBB·velocity·timestamp parity와 dropout behavior |
| launch | privileged input allowlist, 단일 state/TF/control publisher |

리팩토링, threshold tuning, 새 maneuver, sensor 계약과 배포 profile 변경을 한 커밋에 섞지
않는다. 현재 단위 테스트가 통과해도 MORAI full-course 검증을 실행하지 않았다면 그대로
명시한다.

## 9. 알려진 정리 대상

- 이전 standalone planning/control build/run script는 현재 catkin target과 맞지 않아
  P4-C6에서 제거했다. 빌드는 `scripts/build_ws.sh`, 실행은 package launch를 사용한다.
- Behavior의 raw `/Object_topic`과 local map copy를 공용 perception/HD Map contract로 옮겨야 한다.
- v1 제거는 v2 full-stack·rollback gate와 consumer 참조 0을 확인한 뒤 수행한다.
- LANE_CHANGE의 runtime reference handoff와 MERGE production gate는 별도 검증 대상이다.

## 10. 관련 문서

- [운영 규약](../../operations/planning-control.md)
- [v2 frozen contract](../../contracts/planning/fsm-frenet-v2.md)
- [공용 ROS interface](../../../src/interfaces/README.md)
- [handoff index](../../handoffs/README.md)
- [문서 통합 기록](../../structure/d2-planning-handoffs-report.md)
