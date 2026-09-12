# Interface Changelog

공용 ROS 계약의 breaking change와 합의 사항을 기록한다.

## Unreleased - feature/ahn-camera-bev-pilot (typed lane-boundary observation)

### Added

- `asmc_msgs/LaneBoundaryObservation`과 `LaneBoundaryObservationArray`를 추가했다. 카메라가
  관측한 N개 경계를 `base_link` metric polyline, 색상, source bitmask, 유효 범위,
  short-term tracking ID와 두 종류의 품질 score로 전달한다.
- provisional canonical topic `/perception/lane/boundary_observations`를 추가했다. 첫 producer는
  `shadow_two_way_v1` ODD에 제한되며 fresh empty/partial/complete/invalid를 구분한다.
- 메시지에는 home/opposing 의미나 절대 차로 번호를 넣지 않는다. 그 의미는 route,
  MGeo/Frenet과 경계 순서를 함께 아는 판제가 결정한다.

### Compatibility

- 기존 `/perception/lane/boundaries` MarkerArray는 debug와 과거 result bag 호환을 위해
  유지한다. 기존 canonical topic과 메시지는 변경하지 않은 non-breaking addition이다.

## Unreleased - feature/yuntae-planner-lane-change (exact target preservation fix)

바로 아래 "target geometry source change" 항목이 실행 목표의 **출처**를
`target_pose_projection_d`로 바꿨지만, PREPARE 단계의 후보 생성이 여전히
전역 `lateral_target_tolerance`(기본 0.5m)를 쓰고 있어 실제로 선택되는
`selected_target_d`가 그 투영값 ± tolerance 범위 안에서 흔들릴 수 있었다.
COMMIT은 그 값을 그대로 잠그므로, 잠긴 목표가 FSM이 지정한 값과 최대
tolerance만큼 어긋날 수 있는 잠재적 계약 위반이었다(ROS 계약 자체를
바꾸지 않으므로 breaking change는 아니다).

### Fixed

- `Maneuver/LaneChange.cpp`의 `LaneChangeProcess`가 PREPARE 단계부터
  `generation_cfg.lateral_target_tolerance = 0.0`을 무조건 적용한다 - 이전에는
  `lane_change_committed || holding`일 때만 0으로 좁혔다. Behavior가 WHERE를
  정하고 Frenet은 HOW만 최적화한다는 계약(§3)에 따라, PREPARE도 COMMIT과
  같은 정확도로 목표를 겨냥해야 두 단계 사이에 값이 갈리지 않는다.
- 저속(ego.v < 0.5 m/s) PREPARE에서 `GenerateLateralCandidates`가
  `lateral_d1` 격자 자체를 무시하고 현재 d/차선 중앙 후보만 만드는 기존 동작
  (수학적 특이점 회피, 변경하지 않음)과 결합하면, 요청한 target_d에 실제로
  도달하지 않는 후보가 `candidate_valid=true`로 잘못 보고될 수 있었다.
  `LaneChangeProcess`에 종단 d가 target_d와 정확히 일치하는 후보만 유효로
  인정하는 가드를 추가했다 - 불일치하면 WAITING으로 내려가고 현재 차선 유지
  궤적은 그대로 안전하게 publish된다.
- `command_lane_change_plan_test.cpp`의 target_d 일치 검증을 느슨한
  `±0.5m`/`±0.6m`에서 `1e-6` tight tolerance로 강화하고, COMMIT locked
  target_d가 PREPARE가 고른 candidate뿐 아니라 원본
  `command.target_pose_projection_d`와도 직접 일치하는지 확인하는 항목,
  그리고 저속 PREPARE가 거짓 READY를 보고하지 않는지 확인하는 항목을
  추가했다.

## Unreleased - feature/yuntae-planner-lane-change (target geometry source change)

이 항목은 바로 아래 "D7 follow-up" 항목이 도입한 `start_d ± lane_width`
실행 목표 정책을 되돌린 기록이다. 그 항목 자체는 지우지 않는다 - 실제로 그
시점에 그 정책이 쓰였다가 이번에 다시 바뀌었다는 사실을 남긴다.

### Changed

- LANE_CHANGE의 실행 목표(target_d) 결정 정책을 다시 바꿨다. 직전에는
  차선 변경을 시작한 시점의 실제 ego d 기준 `start_d ± lane_width_`를 썼다.
  이제는 **Behavior/FSM이 MGeo 목표 Link centerline에서 뽑아 넣은
  `target_pose_map`을 node가 현재 RefLine에 투영한 값**(`command.
  target_pose_projection_d`)을 그대로 authoritative 실행 목표로 쓴다 -
  Behavior는 이미 adjacent-lane 허용 여부와 실제 목표 차선 형상을 알고
  있으므로, core가 nominal lane width로 그 정보를 다시 근사할 이유가 없다.
  node는 여전히 이 투영값을 RefLine에 넣기 전에 방향(delta_d)/heading이
  명령과 모순되지 않는지 검증한다 - 그 검증을 통과한 값만 core가 신뢰한다.
  `PlannerCommand.lane_change_d_offset`은 이 잠긴 target_d를 그대로
  옮겨싣는다(여전히 Generator가 아니라 core가 채운다).
- target 투영이 없거나(has_target_pose_projection=false) 값이 유한하지
  않으면 lane_width 기반 값으로 대체하지 않고 명령 실패로 수렴한다
  (`Maneuver/LaneChange.cpp`의 `PrepareExternalLaneChangeTarget`) - "새
  차선 중심을 확실히 모르면 차선을 바꾸지 않는다"는 안전 원칙을 유지한다.
- COMMIT 이후 `locked_target_d`는 여전히 다시 계산되지 않는다 - 새로
  들어오는 target pose 갱신이나 ego d 변화가 진행 중인 transaction의 잠긴
  목표를 바꾸지 않는다. HOLD_TARGET_LANE 상태 기계, PREPARE/COMMIT
  handshake, `CommandExecutionManager`의 EXECUTING 유지 정책은 이번
  변경으로 바뀌지 않았다.
- Runtime GlobalPath/reference handoff는 이번에도 범위 밖이다 - 기존
  RefLine을 그대로 유지한 채, 그 위에서의 lateral target 산출 방식만
  바뀌었다.

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
