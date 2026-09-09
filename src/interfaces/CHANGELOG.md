# Interface Changelog

공용 ROS 계약의 breaking change와 합의 사항을 기록한다.

## Unreleased - feature/yuntae-planner-lane-change (D7 follow-up)

이 항목은 바로 아래 "D7" 항목이 추가한 것 중 일부를 이번 후속 작업에서 되돌린
기록이다. 아래 D7 항목 자체는 지우지 않는다 - 실제로 그 시점에 추가되었다가
이번에 제거되었다는 사실을 남긴다.

### Removed

- `asmc_msgs/GlobalPath.msg`와 그에 딸린 `frenet_planner_node`의 `/planning/global_path` 구독·`FrenetPlanner::ReplaceReferenceLine`을 제거했다 - 이번 LANE_CHANGE 검증 단계는 runtime GlobalPath/reference handoff를 쓰지 않기로 범위를 좁혔다(premature interface). Route/GlobalPath 연동은 이후 FSM/Route 통합 작업으로 미룬다.

### Changed

- LANE_CHANGE의 실행 목표(target_d) 결정 정책을 바꿨다. 기존에는 FSM `target_pose_map`을 Frenet d로 투영한 값을 그대로 실행 목표로 썼다. 이제는 **차선 변경을 시작한 시점의 실제 ego d**를 기준으로 `LEFT: start_d + lane_width`, `RIGHT: start_d - lane_width`를 실행 목표로 쓴다 - 아직 참조선을 새 차선으로 바꿔줄 GlobalPath 연동이 없는 상태에서, FSM이 가정한 좌표계/차로 폭이 실제 RefLine과 정확히 맞는다는 보장이 없기 때문이다. `target_pose_map` 투영은 방향/헤딩이 명령과 모순되지 않는지 확인하는 sanity 검증에만 남는다(`PlannerCommand.target_pose_projection_*`, 기존 `has_lane_change_target_d`/`lane_change_target_d`/`lane_change_target_s`에서 이름 변경).
- LANE_CHANGE가 목표 차선에 도달해도 더 이상 `COMPLETED`로 latch하지 않는다. 대신 `HOLD_TARGET_LANE` phase로 전환해 잠긴 target_d를 유지한 채 계속 전진한다 - `CommandExecutionManager`의 execution_state는 `EXECUTING`으로 남는다. `lane_change_commit_coordinator.hpp`의 `CompleteLaneChangeAtomically`(미사용 dead helper였다)를 제거했다.
- Frenet `frenet_planner_node`의 LANE_CHANGE production gate(`SetImplementedManeuver(LANE_CHANGE, true)`)를 열었다 - fixed-reference 검증을 위해 ROS/MORAI 수동 테스트가 필요하다. Behavior Planner의 scorer/policy는 그대로 두었으므로 실제 FSM이 LANE_CHANGE를 자동으로 고르지는 않는다 - 수동 검증은 `tools/lane_change_test_publisher.cpp`로 수행한다(이 tool에서 GlobalPath 발행 기능은 제거했다).

## Unreleased - feature/yuntae-planner-lane-change (D7)

### Added

- `asmc_msgs/GlobalPath.msg`(신규) - Route/FSM이 Frenet에 새 전역 경로를 전달하는 계약(`header`, `revision`, `active_link_id`, `link_ids`, `waypoints`). Frenet은 `/planning/global_path`로 수신해 검증 후 안전한 시점(비-preemptible transaction이 없는 cycle)에만 `RefLine`을 교체한다 - production Route Manager가 이 topic을 실제로 발행하도록 연결하는 작업은 이번 범위가 아니다.
- Frenet `frenet_planner_node`가 `LANE_CHANGE_LEFT`/`LANE_CHANGE_RIGHT`의 PREPARE → READY → EXECUTE → COMMITTED → EXECUTING → COMPLETED 전체 lifecycle을 구현했다 - AVOID(D6)의 transaction/commit-atomicity 패턴을 그대로 재사용한다. `FrenetFeedback`의 `target_d`/`maneuver_progress`/후보 진단 필드도 AVOID와 동일하게 채운다.
- **production gate는 아직 닫혀 있다** - `frenet_planner_node`의 `SetImplementedManeuver(LANE_CHANGE, true)`를 호출하지 않았으므로, 실제 FSM이 LANE_CHANGE를 보내면 여전히 `MANEUVER_NOT_IMPLEMENTED`로 안전정지 응답한다. MORAI 실차 검증 후 별도로 연다.

### Changed

- Frenet `LateralGenerator.cpp`의 `ResolveLateralOffset`이 `LANE_CHANGE_LEFT`/`RIGHT`에서 더 이상 고정 `±lane_width`를 쓰지 않는다 - FSM이 `target_pose_map`으로 지정한 목표를 Frenet d로 투영한 값(`PlannerCommand.lane_change_d_offset`)을 그대로 겨냥한다.

## Unreleased - fix/yuntae-planner Commit G

### Added

- MPC가 `ToReferencePath`로 만든 `ReferencePath`를 `ControlProcess` 호출 직전에 `ValidateReference`로 검증한다(defense-in-depth) - Commit F가 Frenet 쪽 폐루프 seam 불연속을 근본 수정했지만, 향후 Planner/interface 버그가 다시 물리적으로 불가능한 참조(배열 크기 불일치, NaN/Inf, 속도/곡률 초과, 위치 텔레포트)를 보내는 경우에도 MPC가 그것을 그대로 따라가지 않는다.
- 검증 실패는 기존 fail-safe STOP 경로(`publishStop` -> `ResetOnFailSafe` + `MakeRosStopCommand`)로 수렴한다 - 새 fail-safe 구현을 만들지 않았고, 외부에 노출되는 `/ctrl_cmd` 계약도 바뀌지 않는다.
- 이 검증은 순수(ROS 비의존) `src/mpc_controller/src/reference/reference_validator.{hpp,cpp}`에 있으며 `MPCParams`(steering_max/wheelbase/vel_min/vel_max/dt)가 이미 소유한 물리 한계만 사용한다 - Frenet의 `max_curvature`(0.1704) 같은 planner 쪽 값을 복사하지 않았다.

## Unreleased - fix/yuntae-planner Commit E

### Fixed

- `Trajectory.header.stamp` provenance 버그를 고쳤다. Frenet의 `ConvertCartesianPath`/`MakeStopTrajectory`가 `ego_at_plan`과 별개로 `plan_stamp`(항상 `ros::Time::now()`)를 받아, trajectory의 source time이 실제로 그 trajectory를 만든 Ego snapshot의 시각과 다를 수 있었다. 두 함수 모두 `plan_stamp` 인자를 제거하고 `header.stamp = ego_at_plan.header.stamp`를 내부에서 강제한다 - 호출자가 실수로 다른 시각을 넣을 수 없다.
- 완료 latch가 캐시된 trajectory를 재발행할 때 `header.stamp`를 `now`로 덮어쓰던 지점을 제거했다. Point content가 바뀌지 않은 재발행은 원래 source timestamp를 유지한다.
- 위 수정으로 Commit B의 `최신 vehicle_state.header.stamp - trajectory.header.stamp` latency 보정이 실제로 의미 있는 age를 계산하게 됐다(수정 전에는 두 값이 항상 비슷한 "지금"이라 보정이 사실상 무력화될 수 있었다).

### Changed

- MPC의 `ConvertTrajectory`는 이제 `Trajectory -> PlannedPath`만 변환한다. `ego_at_plan`의 provenance 검증(frame_id/valid/finite)은 그대로 하지만, 더 이상 `MPCState`로 변환해 반환하지 않는다 - x0는 `ConvertVehicleState`(최신 `/vehicle/state`) 전용 경로로만 만든다.
- `PlannedPath`에서 쓰이지 않던 relay-era 필드(`ego_x/y/yaw/v/steer/accel`)를 제거했다.
- Trajectory 시간 정렬(`ComputeTrajectoryAge` -> `ComputeReferenceStartOffset`)을 `message_converter`(ROS message <-> core datatype 변환)에서 `reference/trajectory_timing`(순수, `ros::Time` 비의존)으로 옮겼다.
- `IsTrajectoryFresh`를 `IsMessageFresh`로 rename했다 - trajectory와 vehicle_state 양쪽에 재사용되는 일반 receive-time freshness 헬퍼임을 이름에 반영했다.

## Unreleased - fix/yuntae-planner Commit B

### Changed

- MPC Controller가 `/vehicle/state`(`asmc_msgs/VehicleState`)를 canonical subscriber로 추가했다. Control 초기상태(x0)는 더 이상 `Trajectory.ego_at_plan`이 아니라 이 최신 vehicle state에서 만든다.
- `Trajectory.ego_at_plan`은 provenance(Planner가 trajectory 생성에 사용한 Ego snapshot) 검증 용도로만 유지된다 - 계약 자체는 바뀌지 않았다.
- `/vehicle/state` 누락/무효/stale 시 MPC도 Frenet과 동일하게 STOP한다(fail-safe 표에 행 추가).

## V1 - 2026-09-02

### Added

- Canonical internal topics: `/vehicle/state`, `/perception/objects`, `/behavior/context`, `/planning/trajectory`, `/planning/feedback`, `/ctrl_cmd`
- `asmc_msgs` 공용 message package
- MAP frame 및 SI unit 규칙
- Source/reference timestamp 및 consumer freshness 정책
- Public Behavior enum과 lane/Frenet-d 부호
- `command_id`와 optional target 규칙
- Perception OBB corner 8개를 authoritative geometry로 사용하는 계약
- Trajectory `dt`와 `ego_at_plan` 계약
- Planner/MPC fail-safe 및 STOP trajectory 의미
- Competition/development runtime source gating

### Compatibility

- 기존 `behavior_planner` 자체 message와 Frenet 내부 `BehaviorState`는 V1 public enum과 숫자가 완전히 같지 않다.
- Converter에서 명시적으로 매핑하며 `static_cast`를 금지한다.
- 기존 standalone UDP Planner/MPC는 ROS 전환 검증이 끝날 때까지 legacy baseline으로 보존한다.

### Open before LiDAR integration

- 실제 LiDAR/Perception 출력과 대조해 `obb_corners[0..7]` 순서를 확정해야 한다.
