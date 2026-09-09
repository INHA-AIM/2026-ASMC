# Interface Changelog

공용 ROS 계약의 breaking change와 합의 사항을 기록한다.

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
