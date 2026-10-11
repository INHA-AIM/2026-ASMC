# Planning/Control ROS 운영 규약

> **문서 역할:** Canonical Operations — 현재 Behavior·Frenet·MPC 실행과 관측
>
> **상태:** Active — MORAI 외부는 UDP, 팀 PC 내부 Planning/Control은 ROS1/catkin
>
> **개발 경계:** [Planning/Control package-group guide](../packages/planning-control/README.md)

## 1. 범위

현재 Planning/Control은 `behavior_planner`, `frenet_planner`, `mpc_controller` ROS package로
운영한다. MORAI와 직접 통신하는 것은 bridge뿐이며 planner/controller가 UDP socket을 직접
열지 않는다.

```text
MORAI --UDP--> bridge/state/perception --ROS--> Behavior/FSM
                                                 │ BehaviorCommand v2
                                                 ▼
                                              Frenet
                                                 │ Trajectory
                                                 ▼
                                                MPC --/ctrl_cmd--> UDP bridge --> MORAI
```

이전에 사용한 standalone UDP `Planner :9300 → MPC`와 직접 Ctrl UDP 경로는
[역사 기준선](../archive/planning/standalone-udp-baseline.md)에 보존한다. 현재 소스에는
적용하지 않는다.

## 2. 빌드

ROS Noetic container에서 전체 workspace를 빌드한다.

```bash
cd /root/ws
./scripts/build_ws.sh
source devel/setup.bash
```

필요하면 관련 package만 빌드한다.

```bash
catkin_make --only-pkg-with-deps \
  behavior_planner frenet_planner asmc_controller asmc_bridge
source devel/setup.bash
```

P4-C6에서 현재 catkin target과 맞지 않던 이전 standalone build/run script를
제거했다. 전체 workspace는 `scripts/build_ws.sh`, 실행은 아래의 canonical launch를
사용한다.

## 3. 개발 실행

> **판제 오너십 주의:** P5-D에서 perception 통합 측이 변경한 항목은 planning launch의 신호등
> producer 기본값뿐이다. 목적은 중복 publisher 방지이며 판제 정책 변경이 아니다. 호환 include
> 제거와 consumer 정책 변경은 판제 오너가 리뷰·승인한다.

터미널 1에서 현재 개발 bridge를 실행한다.

```bash
cd /root/ws
source devel/setup.bash
./scripts/udp_bridge.sh
```

터미널 2에서 통합 planner와 controller를 실행한다. LiDAR와 카메라 중계기는 별도로 실행한다.

```bash
cd /root/ws
source devel/setup.bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch objects_source:=lidar traffic_light:=false
```

- `asmc_planner` 내부에서 Behavior → Execution → Frenet을 직접 호출한다.
- `/planning/vehicle/state`, `/planning/objects`는 planner와 MPC의 입력이다.
- Behavior 10 Hz, Planning 20 Hz 및 controller/jamming selector 계약은 유지한다.
- 신호 인식은 `traffic_light` 기본값 `true`이며, 외부 producer가 실행 중이면 `false`로 둔다.
- 공개 debug 메시지는 `asmc_msgs/UrbanFeatureDebug`다.

## 4. Component·회귀 검증

| 명령 | 용도 | 주의 |
|---|---|---|
| `roslaunch asmc_planner planner_v2.launch` | 통합 planner 단독 | state/object 입력 별도 필요 |
| `roslaunch asmc_controller controller.launch` | MPC 단독 | 입력 없으면 STOP이 정상 |
| `rosrun asmc_planner v2_ros_regression_test` | 명시적 회귀시험 | 별도의 ROS master와 시험 입력 필요 |

같은 production topic에는 authoritative publisher 하나만 실행한다. `behavior_planner`와
`frenet_planner` 패키지는 제거되었다. 기존 ROS 파라미터 namespace는 유지한다.

## 5. 현재 topic 계약

| Topic | Type | Producer→Consumer | 상태 |
|---|---|---|---|
| `/vehicle/state` | `asmc_msgs/VehicleState` | state→Behavior/Frenet/MPC | current |
| `/perception/objects` | `asmc_msgs/DetectedObjectArray` | perception/adapter→Frenet | current boundary, 개발 adapter 사용 중 |
| `/behavior/command_v2` | `asmc_msgs/BehaviorCommand` | Behavior→Frenet/MPC | current v2 |
| `/planning/frenet_feedback_v2` | `asmc_msgs/FrenetFeedback` | Frenet→Behavior | current v2 |
| `/planning/trajectory` | `asmc_msgs/Trajectory` | Frenet→MPC | current |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | MPC→bridge | current |
| `/behavior/context` | `asmc_msgs/BehaviorContext` | Behavior→Frenet | v1 compatibility |
| `/planning/feedback` | `asmc_msgs/PlanFeedback` | Frenet→Behavior | v1 compatibility |

필드·enum·transaction은 [FSM↔Frenet v2 contract](../contracts/planning/fsm-frenet-v2.md), frame과
timestamp는 [ROS interface](../../src/interfaces/README.md)가 정본이다.

## 6. 관측

```bash
rostopic info /vehicle/state
rostopic info /perception/objects
rostopic info /behavior/command_v2
rostopic info /planning/frenet_feedback_v2
rostopic info /planning/trajectory
rostopic info /ctrl_cmd

rostopic hz /vehicle/state /behavior/command_v2 /planning/trajectory /ctrl_cmd
rostopic echo /behavior/command_v2 -n1
rostopic echo /planning/frenet_feedback_v2 -n1
rostopic echo /planning/trajectory -n1
```

`rostopic hz`만으로 유효성을 판단하지 않는다. 다음도 함께 본다.

- `header.stamp`, `frame_id`, `valid`와 failure/result code
- v2 `command_id`, execution request/state와 terminal latch
- trajectory source/provenance와 ego snapshot
- Frenet/MPC stale·invalid·fail-safe 로그
- `/ctrl_cmd` publisher가 하나인지 여부

RViz debug는 `/behavior/markers`와 `/planning/markers`를 함께 표시한다. camera lane adapter
Marker는 read-only이며 현재 control 입력이 아니다.

## 7. Fail-safe 기준

- Vehicle/object/behavior 입력 누락·invalid·stale은 Frenet의 안전정지 경로로 수렴한다.
- MPC는 trajectory나 vehicle state가 없거나 stale이면 STOP을 발행한다.
- invalid·불연속 reference, interpolation/solver 실패도 STOP이다.
- EMERGENCY_STOP command는 MPC emergency override가 우선한다.
- v2에서 gate가 닫힌 maneuver는 다른 행동으로 조용히 대체하지 않는다.

STOP이 발행된다는 사실만으로 안전 검증이 끝나지 않는다. 제동거리, packet loss, bridge mode와
실제 차량 응답은 MORAI closed-loop에서 확인한다.

## 8. 테스트

```bash
catkin_make run_tests_behavior_planner run_tests_asmc_controller

cd build/frenet_planner
ctest --output-on-failure
cd ../..

catkin_test_results build/test_results/behavior_planner
catkin_test_results build/test_results/asmc_controller
```

Frenet은 catkin package 단위 `run_tests_frenet_planner` target을 등록하지 않고 package build
directory에 25개 CTest를 등록한다. `catkin_test_results`는 결과 경로 하나씩 집계한다.

`v2_ros_regression_test`는 살아 있는 `roscore`와 Frenet node가 필요해 자동 CTest에 포함되지
않는다. 실행법은 `src/asmc_planner/tests/frenet/v2_ros_regression_test.cpp`의 상단 주석을 따른다.

MORAI를 실제로 돌리지 않았다면 unit/static 결과와 full-course 미검증 상태를 분리해 보고한다.

## 9. 현재 본선 준비 gap

- Behavior는 package-local MGeo/route JSON을 읽고 `/hd_map/*`를 아직 직접 소비하지 않는다.
- Behavior는 raw `/Object_topic`, Frenet은 개발 adapter가 만든 `/perception/objects`를 사용한다.
- `autonomy_no_bridges_v2.launch`는 privileged object input 때문에 field-safe가 아니다.
- LANE_CHANGE reference handoff, MERGE gate와 full-course 반복주행은 별도 검증이 필요하다.
- Phase 3 network profile과 15 Hz LiDAR 전환은 아직 이 문서의 현재 실행값이 아니다.

이 gap이 닫히기 전에는 단위 테스트나 개발 주행 성공을 본선 배포 완료로 표현하지 않는다.
