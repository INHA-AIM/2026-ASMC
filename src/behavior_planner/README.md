# Behavior Planner

> **역할:** ROS1 Behavior/FSM runtime — maneuver와 실행 target 결정
>
> **현재 권장 interface:** v2 `BehaviorCommand` ↔ `FrenetFeedback`

Planning 전체 경계는 [Planning/Control 개발 가이드](../../docs/packages/planning-control/README.md),
v2 transaction의 정확한 규칙은 [frozen contract](../../docs/contracts/planning/fsm-frenet-v2.md)를
따른다.

## 현재 I/O

| 방향 | Topic | Type | 비고 |
|---|---|---|---|
| subscribe | `/vehicle/state` | `asmc_msgs/VehicleState` | authoritative ego |
| subscribe | `/Object_topic` | `morai_msgs/ObjectStatusList` | 현재 개발 경계, 본선 교체 필요 |
| subscribe | `/traffic_light/state` | `std_msgs/String` | stale 검사 |
| subscribe | `/planning/frenet_feedback_v2` | `asmc_msgs/FrenetFeedback` | v2 선택 시 |
| publish | `/behavior/command_v2` | `asmc_msgs/BehaviorCommand` | v2 선택 시 |
| publish | `/behavior/feature_debug` | `behavior_planner/UrbanFeatureDebug` | debug |
| publish | `/behavior/markers` | `visualization_msgs/MarkerArray` | debug |

v1에서는 `/planning/feedback`을 구독하고 `/behavior/context`를 발행한다. 하나의 node가
`interface_version`에 따라 한 경로만 선택한다.

## Map과 route

현재 runtime은 package 내부 `link_set.json`, `node_set.json`, `route_links_1.yaml`을 읽는다.
완성된 `asmc_hd_map`의 `/hd_map/*`를 직접 소비하는 구현은 아직 아니다. 두 데이터의 link,
route station과 mission 의미를 대조하지 않고 package-local map을 삭제하거나 바꾸지 않는다.

## 실행

bridge는 별도 터미널에서 먼저 실행한다. 현재 v2 개발 chain:

```bash
cd /root/ws
source devel/setup.bash
roslaunch behavior_planner autonomy_no_bridges_v2.launch
```

이 launch는 `/Object_topic` 개발 oracle을 `/perception/objects`로 변환한다. 본선 sensor-only
launch가 아니며, traffic-light weight가 준비되지 않은 배관 시험은
`traffic_light:=false`로 명시할 수 있다.

단독 node:

```bash
roslaunch behavior_planner behavior_node_v2.launch dashboard:=false
```

## 검증

```bash
catkin_make run_tests_behavior_planner
catkin_test_results build/test_results/behavior_planner
```

Behavior 변경은 route/signal/object scenario, command ID·transaction, stale feedback과
Frenet consumer를 함께 확인한다. dashboard·Marker가 정상이어도 trajectory와 feedback
contract가 통과했다는 뜻은 아니다.
