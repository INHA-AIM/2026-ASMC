# Planning/Control 리팩토링 가이드라인

> **문서 상태:** Active Reference — 팀 리뷰 후 Canonical 승격 여부를 결정한다.
> **현재 package 경계:** [Planning/Control 개발 가이드](./README.md)
> **범위:** `src/asmc_planner/`, `src/asmc_controller/` 코드를 수정할 때 지켜야 하는
> 아키텍처 규칙. 알고리즘 파라미터 튜닝 가이드가 아니다.
> **기준 시점:** `Fix/yuntae-planner` 브랜치, Commit E(트래잭토리 provenance 수정 +
> 이 문서 추가) 시점의 실제 코드 구조.
> **문서 허브:** [README.md](../../README.md)

이 문서는 "Frenet Planner나 MPC Controller를 고칠 때 어떤 아키텍처 규칙을 지켜야
하는가?"에 답한다. 코드가 바뀌면 이 문서도 같은 PR에서 갱신한다 - 여기 적힌 구조와
실제 코드가 어긋나면 이 문서가 아니라 코드가 맞다고 가정하지 말고 둘 다 고친다.

## 1. North Star

목표는 파일을 최대한 잘게 쪼개는 것이 아니다. 새 팀원이 위에서 아래로 읽으면서
점점 구체적인 구현으로 내려갈 수 있는 계층을 유지하는 것이다.

```text
폴더(패키지)
  ↓
런타임/진입점(main, node)
  ↓
최상위 process/orchestration 함수
  ↓
domain helper
```

동시에 다음을 지킨다.

- **Core/ROS 분리**: 알고리즘 코드는 `ros::`/`asmc_msgs::`/`morai_msgs::`를 모른다.
- **state ownership**: 상태는 그 상태를 실제로 쓰는 클래스가 소유한다. 여러
  장기 구조체에 같은 authoritative 값을 중복 보관하지 않는다.
- **testability**: ROS 경계를 넘기 전에 순수 함수로 뽑을 수 있으면 뽑는다.
- **명확한 authoritative source**: "지금 상태를 누가 대표하는가"에 대해 항상
  하나의 정답만 있어야 한다.

## 2. 아키텍처 경계 (현재 구조)

```text
Behavior/FSM (BehaviorCommand, /behavior/command_v2)
        ↓
Frenet ROS Runtime (frenet_planner_node, behavior_command_adapter,
                     command_execution_manager, message_converter,
                     feedback_builder)
        ↓
FrenetPlanner Core (frenet_planner.hpp/cpp + Common/Generator/Evaluation/
                     Maneuver/Speed/Reference/Geometry/Math)
        ↓
/planning/trajectory (asmc_msgs/Trajectory)

/planning/trajectory + /vehicle/state
        ↓
MPC ROS Runtime (main.cpp / Control.cpp, message_converter)
        ↓
Reference/Timing (path_resampler, trajectory_timing)
        ↓
MPC (warm-start / last control / stuck-recovery 상태 소유)
        ↓
Model / Cost / Constraints / Solver (mpc_core, ROS 비의존)
        ↓
/ctrl_cmd (morai_msgs/CtrlCmd)
```

Frenet과 MPC는 `/planning/trajectory` 하나로만 연결된다. Behavior/FSM은
`/behavior/command_v2`로만 Frenet에 연결된다. 이 경계를 우회하는 직접 참조나
패키지 간 UDP 재도입은 하지 않는다.

## 3. 책임 매트릭스

| 구성요소 | 소유 | 소유하지 않음 |
|---|---|---|
| Behavior/FSM | maneuver 선택, command_id/transaction lifecycle | trajectory 생성, collision 검사 |
| Frenet ROS Node (`frenet_planner_node`) | subscribe, freshness, ROS↔Core 변환 호출, 20Hz 타이머, publish, fail-safe 분기 | candidate 생성/평가 수식 |
| Behavior Command Adapter (`behavior_command_adapter`) | `BehaviorCommand` 유효성 검증 | transaction 상태 |
| Command Execution Manager (`command_execution_manager`) | v2 명령 transaction 상태(IDLE/READY/COMMITTED/EXECUTING/...) | planning 결과 자체 |
| Feedback Builder (`feedback_builder.cpp`) | `FrenetFeedback` 메시지 조립 | planning 로직 |
| Message Converter (frenet, `MessageConverter.cpp`) | ROS message ↔ Core datatype 변환, 필드 유효성 검증 | candidate 선택/비용 정책 |
| FrenetPlanner Core (`frenet_planner.hpp/cpp`) | maneuver pipeline 오케스트레이션, candidate 상태 캐시 | ROS 타입, 타이머 |
| Generator | lateral/longitudinal candidate 샘플링 | 비용 평가, 충돌 검사 |
| Evaluation | 비용/운동학/충돌 필터링 | candidate 생성 |
| Maneuver | 모드별(KEEP/STOP/AVOID/...) 파이프라인 조립 | 저수준 수식 |
| Reference/Speed/Geometry/Math | RefLine, 곡률, 다항식, 좌표 변환 등 순수 계산 | 모드 정책 |
| MPC ROS Node (`main.cpp / Control.cpp`) | subscribe(trajectory, vehicle_state, behavior_command), freshness, ROS↔Core 변환 호출, 20Hz 타이머, publish, fail-safe | 최적화 수식, warm-start 정책 |
| MPC Message Converter (`MessageConverter.cpp`) | Trajectory→PlannedPath, VehicleState→MPCState, 필드 유효성 검증, CtrlCmd 조립 | 시간 정렬 계산, 최적화 정책 |
| MPC Reference (`path_resampler`, `trajectory_timing`) | trajectory 리샘플링, source-time offset 계산 | ROS 타입, freshness watchdog |
| MPC | warm-start, last_control, stuck/recovery 상태, solveMPC 호출 순서 | 최적화 수식 자체 |
| Model/Cost/Constraints/Solver (`mpc_core`) | bicycle model 선형화, 비용 함수, box/rate 제약, gradient descent | ROS, provenance 검증 |

## 4. State Ownership 규칙

- `FrenetPlanner`는 참조선, 마지막 target speed, AVOID lock 등 **planner 사이클
  상태**를 소유한다.
- `CommandExecutionManager`는 v2 command transaction lifecycle을 소유한다.
- `MPC`는 `warm_start_`, `last_control_`, stuck/recovery cycle 카운터를
  소유한다. `main.cpp / Control.cpp`는 이 상태를 직접 만지지 않고 `MPCProcess`/
  `ResetOnFailSafe`를 통해서만 접근한다.
- **Ego(현재 차량 상태)의 authoritative source는 하나뿐이다**: `/vehicle/state`.
  - Frenet은 `/vehicle/state`를 직접 구독해 planning의 현재 상태로 쓴다.
  - MPC도 `/vehicle/state`를 직접 구독해 `ControlProcess`의 x0로 쓴다.
  - `Trajectory.ego_at_plan`은 **provenance 전용 스냅샷**이다 - "이 trajectory가
    어떤 Ego 상태에서 계획됐는가"를 기록할 뿐, 어느 쪽 control 입력으로도 다시
    쓰이지 않는다. `PlannedPath`에 relay용 `ego_*` 필드를 두지 않는다(Commit E에서
    제거) - 필요하면 `ConvertVehicleState`로 최신 상태를 새로 만든다.
- file-scope mutable 전역 상태를 두지 않는다. 모든 사이클 상태는 클래스 멤버로
  소유한다.

## 5. Data Provenance와 Timestamp 규칙

이 절이 Commit E의 핵심이다.

- `header.stamp`는 **callback 수신 시각이 아니라 그 메시지가 표현하는
  source/reference time**이다(`src/interfaces/README.md` 4절).
- `ros::SteadyTime`은 **receive-time freshness watchdog 전용**이다
  (`IsMessageFresh`). stale/missing 판정에만 쓴다.
- 이 둘을 한 계산식에서 섞지 않는다. Source time으로 latency를 보정하는 동안
  receive time으로 freshness를 판정하는 코드 경로는 항상 분리되어 있어야 한다.
- `Trajectory.header.stamp`는 그 trajectory를 만든 `ego_at_plan`(VehicleState)
  스냅샷의 source time과 항상 같다. `ConvertCartesianPath`/`MakeStopTrajectory`는
  별도 timestamp 인자를 받지 않고 `ego_at_plan.header.stamp`를 내부에서 그대로
  쓴다 - 호출자가 실수로 다른 시각(`ros::Time::now()` 등)을 넣을 수 있는 API를
  만들지 않는다.
- **재발행(republish)은 새 source time을 만들지 않는다.** point content가 바뀌지
  않은 trajectory(latch, 캐시 재사용)를 다시 publish할 때 `header.stamp`를
  현재 시각으로 덮어쓰지 않는다 - 재발행 수신 시각은 provenance가 아니다.
- `ego_at_plan`은 provenance 검증 계약이지 control 입력이 아니다. MPC의 x0는
  항상 최신 `/vehicle/state`에서 만든다(`ConvertVehicleState`).
- MPC의 reference 시작 offset은 `최신 /vehicle_state.header.stamp -
  trajectory.header.stamp`(둘 다 source time, `ComputeReferenceStartOffset`)로
  계산한다. 이 계산은 `reference/trajectory_timing`에 있고 `ros::Time`에
  의존하지 않는다 - `ros::Time → double` 변환은 ROS 경계(`main.cpp / Control.cpp`)
  에서 한다.
- Feedback류 메시지(`FrenetFeedback.header.stamp`, `PlanFeedback.header.stamp`)는
  다른 계약이다 - "이 피드백이 생성된 시각"이며 `ros::Time::now()`가 맞다. 모든
  `header.stamp`를 일괄적으로 source-time 규칙으로 바꾸지 않는다 - 메시지 종류별
  계약을 먼저 분류한 뒤 바꾼다.

## 6. Dependency Direction

**Frenet:**

```text
ROS Runtime (frenet_planner_node)
    ↓
Adapter/Manager (behavior_command_adapter, command_execution_manager)
    ↓
Planner Core (frenet_planner.hpp/cpp)
    ↓
Generator / Evaluation / Maneuver / Geometry / Reference / Speed / Math
```

**MPC:**

```text
ROS init (main.cpp) → ControlProcess (Control.cpp)
    ↓
Reference / Control (path_resampler, trajectory_timing, MPC)
    ↓
Model / Cost / Constraints / Solver / Common (mpc_core)
```

두 스택 모두 Core는 위쪽(ROS Runtime)에 의존하지 않는다. `mpc_core`/
`frenet_planner_core`가 `ros::`, `asmc_msgs::`, `morai_msgs::`를 include하면
이 규칙 위반이다.

## 7. Process 지향 가독성

파일은 다음 질문에 답할 수 있어야 한다.

- 무엇을 소유하는가?
- 입력은 무엇인가?
- 무엇을 만들어내는가?
- 명시적으로 하지 않는 것은 무엇인가?

`Control.cpp::ControlProcess`이나
`frenet_planner_node.cpp::planningTimerCallback`처럼, 대표 orchestration 함수를
먼저 두고 그 아래에 단계별 helper를 배치하는 구조를 유지한다. 세부 수식은 더
깊은 계층(Core)으로 내려보낸다.

## 8. 파일/주석 규칙

- 핵심 파일(node, core class, message converter)은 파일 상단이나 함수 주석에
  역할/입력/출력/비책임을 명시한다.
- 사소한 getter나 자명한 대입에는 주석을 달지 않는다.
- 주석은 WHY(숨은 제약, 과거 사고의 교훈, 비직관적 이유)를 설명한다. WHAT은
  코드 자체가 이미 말한다.
- 오래된 주석이 현재 아키텍처와 모순되면(예: "MPC는 Planner가 중계한 Ego를
  쓴다" 같은 문장이 `/vehicle/state` 직접 구독 이후에도 남아있는 경우) 그
  코드를 건드리는 PR에서 함께 고친다.

## 9. Refactoring vs 기능 변경

다음을 같은 커밋에 섞지 않는다.

- 아키텍처 정리
- 동작 threshold 튜닝
- 새 maneuver 기능
- solver 튜닝
- 센서 계약 변경

리팩토링 커밋 안에 작은 동작 수정이 들어갈 수는 있다 - 단, 그 수정이 무엇인지
커밋 메시지에 명시하고 회귀 테스트를 남긴 경우에만 그렇다(예: Commit E의
trajectory timestamp provenance 수정은 "리팩토링"이 아니라 명시적으로
"fix"로 분류됐다).

## 10. Anti-pattern

다음을 발견하면 그 자리에서 고치거나 최소한 이슈로 남긴다.

- Core가 ROS에 의존한다.
- 패키지 내부에 UDP를 다시 들여온다(Planner/MPC 경계 안에서).
- 파일 스코프 mutable 전역 상태.
- 같은 authoritative Ego 상태를 여러 장기 구조체가 각자 들고 있다.
- message converter가 알고리즘 정책(어떤 candidate를 고를지, 어떤 steering을
  쓸지)을 갖고 있다.
- trajectory 시간 정렬 계산이 message conversion 코드 안에 섞여 있다.
- 바뀌지 않은 캐시/latch trajectory의 `header.stamp`를 재발행 시각으로 바꾼다.
- 현재 아키텍처와 모순되는 legacy 주석이 그대로 남아 있다.
- 하나의 거대한 node 파일 안에 수치 최적화/planner 알고리즘이 통째로 들어있다.
- 같은 공식(예: steering feed-forward)을 여러 곳에 복붙한다.
- constraints/solver 바깥에서 사후적으로 actuator 값을 조작한다(post-hoc
  feed-forward 등).
- regression test를 지워서 리팩토링을 "통과"시킨다.

## 11. Testing Gate

커밋 전에 확인한다.

- [ ] 옮긴 순수 로직에 대한 unit test가 있는가
- [ ] 적절한 경우 negative/regression test가 있는가
- [ ] 기존 패키지 테스트가 모두 남아있고 통과하는가
  - Frenet: catkin build의 `build/frenet_planner`에서 `ctest` (현재 25개 등록 테스트)
  - MPC: catkin build의 `build/asmc_controller`에서 `ctest` (현재 11개 gtest 바이너리)
  - `v2_ros_regression_test`는 `add_test`에 등록되어 있지 않다 - roscore와
    `frenet_planner_node`를 수동으로 띄운 뒤 `rosrun`으로 실행한다(파일 상단
    주석의 실행법 참고).
- [ ] 통합 회귀(가능하면 `v2_ros_regression_test`)를 실행했는가
- [ ] `git diff --check`가 깨끗한가
- [ ] ROS/catkin 빌드가 깨끗한가(`scripts/build_ws.sh` 또는 관련 package `catkin_make`)

MORAI 실주행 결과는 실제로 실행한 경우에만 보고한다. "Static/unit/regression
verification complete, MORAI full-course validation pending" 같은 정확한
문구로 미실행 상태를 분명히 한다.

## 12. Refactoring 체크리스트

- [ ] 이 상태의 authoritative 소유자가 단 하나인가?
- [ ] source time과 receive time이 필요한 곳에서 명확히 분리되어 있는가?
- [ ] Core가 이번 변경으로 ROS 의존성을 새로 얻지 않았는가?
- [ ] converter가 정책을 갖고 있지 않은가?
- [ ] 상태를 갖는 필드가 그것을 실제로 쓰는 클래스에 있는가?
- [ ] 예전 주석이 여전히 사실인가?
- [ ] "리팩토링만 했다"고 하면서 실제로는 동작을 바꾸지 않았는가?
- [ ] 기존 테스트가 모두 남아있는가?
- [ ] 문서가 실제 구현과 일치하도록 갱신됐는가?

## Future Candidate

다음은 현재 구조가 아니라 이후 검토 후보다. 이 문서의 다른 절과 혼동하지 않는다.

- v1(`BehaviorContext`/`PlanFeedback`) 경로는 D10 cutover 이후 제거 후보다
  (현재는 v2와 병행 운영 중, `fsm-frenet-interface-v2.md` 10절).
- `main.cpp / Control.cpp`가 여러 개별 timeout(`trajectory`, `vehicle_state`)을
  갖는 대신 하나의 input-freshness aggregator로 통합하는 안은 아직 결정되지
  않았다.

## 관련 문서

- [src/asmc_planner/README.md](../../../src/asmc_planner/README.md) — Frenet 모듈 진입점
- [src/interfaces/README.md](../../../src/interfaces/README.md) — 공용 ROS 계약(canonical topic, timestamp, fail-safe)
- [fsm-frenet-interface-v2.md](../../contracts/planning/fsm-frenet-v2.md) — FSM↔Frenet v2 계약 고정 문서
- [architecture-overview.md](../../architecture/system-overview.md) — 팀 전체 아키텍처 요약(일부 항목은 이 문서 기준 시점보다 오래됐을 수 있다 - 충돌 시 실제 코드를 우선한다)
