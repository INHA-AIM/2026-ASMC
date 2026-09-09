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
cutover 전까지의 과도기 경로다(`../../docs/fsm-frenet-interface-v2.md` 10절).

현재 v2에서 구현된 maneuver: KEEP, STOP, EMERGENCY_STOP, FOLLOW, AVOID.
MERGE/LANE_CHANGE 등 나머지는 미구현이며, 미구현 maneuver 명령은 accept는 하되
planning하지 않고 안전정지로 응답한다.

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
`../../docs/fsm-frenet-interface-v2.md`를 따른다.

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

# 등록된 회귀 테스트 전체 실행(현재 21개)
cd <catkin build dir>/frenet_planner   # 또는 워크스페이스 build/frenet_planner
ctest --output-on-failure
```

`tests/v2_ros_regression_test.cpp`는 살아있는 `roscore`와 `frenet_planner_node`가
필요해 `add_test`에 등록되어 있지 않다 - 파일 상단 주석의 3터미널 실행법을
따른다.

## Related Documentation

- [../../docs/planning-control-refactoring-guidelines.md](../../docs/planning-control-refactoring-guidelines.md) — 아키텍처 경계, ownership, refactoring 규칙
- [../../docs/fsm-frenet-interface-v2.md](../../docs/fsm-frenet-interface-v2.md) — FSM↔Frenet v2 계약 고정 문서
- [../interfaces/README.md](../interfaces/README.md) — 공용 ROS 계약(canonical topic, timestamp, fail-safe)
- [../../docs/README.md](../../docs/README.md) — 문서 허브
