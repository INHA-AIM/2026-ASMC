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
| subscribe | `/perception/traffic_light/detection` | `asmc_msgs/TrafficLightDetection` | P4-C3 권위 입력, `valid/stale` fail-safe |
| subscribe | `/traffic_light/state` | `std_msgs/String` | typed producer가 한 번도 나타나지 않은 구 배포용 fallback |
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
launch가 아니다. 신호등 producer는 `asmc_perception` runtime이 소유하므로 이 launch의
`traffic_light` 기본값은 `false`다. 독립 호환 시험에서만 `traffic_light:=true`로 같은
canonical executable을 포함하며, perception launch와 동시에 켜 중복 publisher를 만들지 않는다.

P4-C3부터 호환 옵션으로 이 launch가 시작할 수 있는 신호등 executable은 기존
`src/traffic_light/live_infer.py`가 아니라
`src/asmc_perception/scripts/traffic_light_node.py`다. 기존
`/traffic_light/state`, `/traffic_light/detail`은 바뀌지 않았으며 typed
`/perception/traffic_light/detection`이 추가로 발행된다. 판제 adapter는 typed 결과를 우선하고,
typed publisher를 한 번도 받지 못한 구 실행에서만 String을 사용한다. typed 결과가 invalid 또는
stale이면 String으로 우회하지 않고 UNKNOWN으로 처리한다.

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
