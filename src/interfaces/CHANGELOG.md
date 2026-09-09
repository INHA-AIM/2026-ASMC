# Interface Changelog

공용 ROS 계약의 breaking change와 합의 사항을 기록한다.

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
