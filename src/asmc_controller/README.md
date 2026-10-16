# MPC Controller

> **역할:** ROS1 trajectory tracking runtime + transport-independent MPC core

Planning 전체 경계는 [Planning/Control 개발 가이드](../../docs/architecture/planning-control.md),
시간·frame·fail-safe 계약은 [ROS interface](../interfaces/README.md)를 따른다.

## 현재 I/O

| 방향 | Topic | Type | 의미 |
|---|---|---|---|
| subscribe | `/planning/trajectory` | `asmc_msgs/Trajectory` | Frenet reference |
| subscribe | `/vehicle/state` | `asmc_msgs/VehicleState` | 유일한 현재 x0 |
| subscribe | `/behavior/command_v2` | `asmc_msgs/BehaviorCommand` | emergency override |
| publish | `/ctrl_cmd/normal` | `morai_msgs/CtrlCmd` | Mode selector가 NORMAL에서 `/ctrl_cmd`로 전달 |

`Trajectory.ego_at_plan`은 trajectory provenance 검증용이며 현재 vehicle state로 재사용하지
않는다. reference start offset은 trajectory와 최신 vehicle source time의 차이로 계산한다.

## Fail-safe

다음 경우 해당 control cycle은 STOP command로 수렴한다.

- trajectory 또는 vehicle state missing/stale
- invalid·STOP trajectory
- reference interpolation 또는 연속성 검증 실패
- vehicle state 변환 실패
- MPC solver 실패

`BehaviorCommand::EMERGENCY_STOP`은 일반 solve보다 우선하는 emergency brake override다.
wheelbase와 steering limit는 Frenet, MPC와 MORAI bridge 설정에서 일치해야 한다.

## 빌드·실행

```bash
cd /root/ws
./scripts/build_ws.sh
source devel/setup.bash
roslaunch asmc_controller controller.launch
```

MPC 단독 실행은 입력 누락으로 STOP을 내는 fail-safe 확인에는 유효하지만 차량 추종 검증은
아니다. full chain은 `behavior_planner/autonomy_no_bridges_v2.launch`를 사용한다.

## 검증

```bash
catkin_make run_tests_asmc_controller
catkin_test_results build/test_results/asmc_controller
```

최소 회귀 범위는 message conversion, trajectory timing, reference validation, steering
feed-forward와 controller state다. unit test 통과와 MORAI closed-loop 주행은 별도 결과로
보고한다.
