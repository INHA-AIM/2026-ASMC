# Frenet Planner

## Responsibility

Behavior/FSM이 **어떤 maneuver를 할지** 결정한다. Frenet Planner는 그 maneuver를
**실행**한다 - candidate trajectory를 생성하고, feasibility/collision/cost로
평가하고, 선택된 trajectory와 feedback을 publish한다.

Frenet은 상위 behavior 선택 로직을 다시 들여오지 않는다. 모드(KEEP/STOP/
EMERGENCY_STOP/FOLLOW/AVOID/...)는 항상 `/behavior/command_v2`로 들어온
명령을 따른다.

## Runtime I/O

Canonical topic(현재 production 경로, v2):

| 방향 | Topic | Message |
|---|---|---|
| subscribe | `/vehicle/state` | `asmc_msgs/VehicleState` |
| subscribe | `/perception/objects` | `asmc_msgs/DetectedObjectArray` |
| subscribe | `/behavior/command_v2` | `asmc_msgs/BehaviorCommand` |
| publish | `/planning/trajectory` | `asmc_msgs/Trajectory` |
| publish | `/planning/frenet_feedback_v2` | `asmc_msgs/FrenetFeedback` |

레거시 v1 경로(`/behavior/context` → `/planning/feedback`, `BehaviorContext`/
`PlanFeedback`)가 같은 노드 안에 병행 운영 중이다 - v2 명령이 하나라도 도착하면
그 사이클은 v2가 소유하고, v2 명령이 없을 때만 v1 경로가 동작한다. v1은 D10
cutover 전까지의 과도기 경로다(`../../docs/contracts/planning/fsm-frenet-v2.md` 10절).

현재 v2에서 production gate가 열린 maneuver: KEEP, STOP, EMERGENCY_STOP,
FOLLOW, AVOID, LANE_CHANGE. MERGE는 미구현이며, gate가 닫힌 maneuver 명령은
accept는 하되 planning하지 않고 안전정지로 응답한다.

### LANE_CHANGE validation scope

`frenet_planner_node`의 gate는 열려 있지만, Behavior Planner의 scorer/policy는
LANE_CHANGE를 자동으로 고르지 않는다 - 이 단계의 수동 검증은
`tools/lane_change_test_publisher.cpp`로 직접 command를 발행해 수행한다.

현재 구현의 범위는 다음으로 고정되어 있다:

- **기존 RefLine을 그대로 유지한다.** Route/FSM이 새 전역 경로를 보내 참조선
  자체를 교체하는 기능(runtime global-path/reference handoff)은 이번 범위에
  없다 - `docs/packages/planning-control/refactoring.md`의 "one policy, one
  authoritative owner" 원칙에 따라, 아직 검증되지 않은 좌표계 전환 로직을
  넣기보다 이번 단계는 고정 참조선 위의 순수 lateral shift로 좁혔다.
- **실행 목표는 Behavior/FSM이 준 `target_pose_map`(MGeo 목표 Link
  centerline에서 뽑은 실제 목표점)을 현재 RefLine에 투영한 값이다.**
  Frenet은 MGeo/Link/Route를 직접 알지 못하며, `lane_width` 기반 근사값으로
  대체하지 않는다 - node가 투영/heading/방향 sanity 검증을 통과시킨 값만
  core의 `target_d`로 잠근다. Behavior가 WHERE(목표 차로 중심)를 정하고
  Frenet은 HOW(duration/충돌회피/kinematic feasibility)만 최적화한다 -
  PREPARE 단계부터 lateral 후보 tolerance를 0으로 두어, 종단 target d가
  투영값과 정확히 같은 candidate만 유효로 인정한다(전역 `lateral_target_tolerance`
  범위 안에서 값을 고르지 않는다).
- **목표 차선 도달 후 `COMPLETED`로 latch하지 않는다.** 아직 새 참조선으로
  전환할 방법이 없으므로, `HOLD_TARGET_LANE`으로 전환해 잠긴 target_d를
  유지한 채 계속 전진한다 - `CommandExecutionManager`의 execution_state는
  `EXECUTING`으로 남는다.
- Route/GlobalPath 연동, 그리고 그에 따른 새 차선 참조선으로의 전환 및
  `COMPLETED` latch는 이후 FSM/Route 통합 작업으로 미룬다.

## Runtime Flow

```text
input snapshot(vehicle/objects/behavior) 획득
  → freshness/유효성 검사
  → v2 command validation + transaction 처리(CommandExecutionManager)
  → ROS message → Core datatype 변환(message_converter)
  → FrenetPlanner::Plan(maneuver pipeline)
      → candidate 생성(Generator)
      → feasibility/collision/cost 평가(Evaluation)
  → CartesianPath → asmc_msgs::Trajectory 변환
  → /planning/trajectory, /planning/frenet_feedback_v2 publish
```

입력 누락/무효/stale, collision blocked, 명령 검증 실패 등 실패 경로는 모두
안전정지(STOP trajectory, `valid=true` + `failure_reason`)로 수렴한다. 전이/
fail-safe 표는 `../interfaces/README.md` 9절, v2 세부 계약은
`../../docs/contracts/planning/fsm-frenet-v2.md`를 따른다.

## Core Folder Map

`src/frenet_planner/`(도메인 core, ROS 비의존):

| 폴더 | 역할 |
|---|---|
| `Common/` | Frenet 상태/경로/명령 등 공유 데이터 타입 |
| `Generator/` | lateral/longitudinal candidate 샘플링 |
| `Evaluation/` | 비용, 운동학, 충돌 필터링 |
| `Maneuver/` | 모드별(KEEP/STOP/AVOID/FOLLOW/EMERGENCY) 파이프라인 조립 |
| `Speed/` | 커브 안전속도, 목표속도 정책 |
| `Reference/` | RefLine 로드/투영/보간 |
| `Geometry/` | OBB corner 검증, polygon 충돌 유틸 |
| `Math/` | Frenet↔Cartesian 변환 |

`frenet_planner.hpp/.cpp`가 이 폴더들을 오케스트레이션하는 `FrenetPlanner`
클래스다.

### Reference topology

`RefLine`은 자신이 열린 경로인지 닫힌 트랙(폐루프)인지를 `closed_loop`/
`length` 필드로 명시한다 - 대회 global path는 첫점/끝점이 물리적으로 같은
폐루프다. `BuildRefLine`이 이 topology를 원본 waypoint의 첫점/끝점 거리로
판정하고, smoothing/균일 재샘플링/theta·kappa·d_kappa 미분을 모두 그
topology에 맞게(폐루프면 순환 이웃, 열린 경로면 기존 편측 처리) 수행한다.
`Interpolate`/`FindClosestS`(둘 다 `Reference.cpp`)와 `LookaheadTargetSpeed`
(`Speed/CurveSpeed.cpp`)는 이 `ref.closed_loop`/`ref.length`만 보고 wrap
여부를 정한다 - topology 판정을 여러 곳에서 각자 다시 하지 않는다("one
policy, one authoritative owner").

## Runtime Boundary Files

`src/frenet_planner/src/`(ROS 경계, Core를 호출):

| 파일 | 역할 |
|---|---|
| `frenet_planner_node.cpp/.hpp` | subscribe, freshness, 20Hz 타이머, publish, fail-safe 오케스트레이션(v1+v2) |
| `behavior_command_adapter.cpp/.hpp` | `BehaviorCommand` 형식 검증 |
| `command_execution_manager.cpp/.hpp` | v2 command transaction lifecycle 소유 |
| `feedback_builder.cpp` | `FrenetFeedback` 메시지 조립(`FrenetPlannerNode::publishFeedbackV2` 정의) |
| `message_converter.cpp/.hpp` | ROS message ↔ Core datatype 변환, `Trajectory.header.stamp` provenance 규칙 적용 |
| `main.cpp` | 노드 엔트리 포인트 |

## Safety / Fail-safe

Vehicle/objects/behavior 입력 누락·무효·stale, `force_stop`, collision blocked는
모두 STOP trajectory로 응답한다. `Trajectory.header.stamp`는 항상 그 trajectory를
만든 `ego_at_plan` 스냅샷의 source time과 같다 - callback 수신 시각이나 재발행
시각으로 바뀌지 않는다. 세부 계약은 링크된 canonical 문서를 참고한다 - 여기서
중복 설명하지 않는다.

## Build/Test

이 저장소는 ROS1/catkin 기반이며, 이 패키지는 UDP 없이 ROS topic으로만
통신한다(패키지 자체 launch는 `launch/frenet_planner.launch`).

```bash
# 저장소 표준 빌드 스크립트(frenet_planner + mpc_controller)
bash scripts/build_planning_control.sh

# 또는 catkin 워크스페이스에서 이 패키지만
catkin_make --pkg frenet_planner

# 등록된 회귀 테스트 전체 실행(현재 23개)
cd <catkin build dir>/frenet_planner   # 또는 워크스페이스 build/frenet_planner
ctest --output-on-failure
```

`tests/v2_ros_regression_test.cpp`는 살아있는 `roscore`와 `frenet_planner_node`가
필요해 `add_test`에 등록되어 있지 않다 - 파일 상단 주석의 3터미널 실행법을
따른다.

## Related Documentation

- [../../docs/packages/planning-control/refactoring.md](../../docs/packages/planning-control/refactoring.md) — 아키텍처 경계, ownership, refactoring 규칙
- [../../docs/contracts/planning/fsm-frenet-v2.md](../../docs/contracts/planning/fsm-frenet-v2.md) — FSM↔Frenet v2 계약 고정 문서
- [../interfaces/README.md](../interfaces/README.md) — 공용 ROS 계약(canonical topic, timestamp, fail-safe)
- [../../docs/README.md](../../docs/README.md) — 문서 허브
