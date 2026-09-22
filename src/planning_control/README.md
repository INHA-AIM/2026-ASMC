# Planning/Control

현재 Behavior Planner, Frenet Planner와 MPC Controller는 ROS1/catkin package이며 내부
`asmc_msgs` topic으로 연결된다. MORAI UDP는 bridge package만 소유한다. 자세한 현재 구조는
[Planning/Control 개발 가이드](../../docs/packages/planning-control/README.md), 실행은
[운영 규약](../../docs/operations/planning-control.md)을 따른다.

아래 `lane_boundary_adapter`는 camera 관측 계약을 판제와 검토하기 위한 별도 read-only ROS
pilot이다. 기존 planner/controller 실행 경로나 차량 제어 명령에 연결되지 않는다.

| 프로젝트 | 담당 | 빌드 |
|---|---|---|
| `src/frenet_planner/` | 이강태 | Catkin, C++17 ROS runtime + core |
| `src/mpc_controller/` | 정윤태·양서준 | Catkin, C++17 ROS runtime + core |
| `src/behavior_planner/` | 정윤태·양서준 | Catkin, v2 Behavior/FSM runtime |
| `src/planning_control/lane_boundary_adapter/` | 안승현·판제 공동 검토 | Catkin, C++17, read-only ROS pilot |

```bash
./scripts/build_ws.sh
source devel/setup.bash
roslaunch behavior_planner autonomy_no_bridges_v2.launch
```

위 launch는 개발 `/Object_topic` adapter를 사용하므로 본선 sensor-only launch가 아니다.
이전 standalone helper는 현재 executable과 맞지 않으므로 사용하지 않는다.

카메라 경계 관측을 제어와 분리해 확인하려면
[lane boundary adapter README](./lane_boundary_adapter/README.md)를 따른다. 해당 출력은
Marker/diagnostics 전용이며 canonical 판제 입력 토픽은 공동 검증 후 별도 합의한다. 음영
왕복 2차로에서 관측 중앙선이 잠깐 누락될 때의 inferred-center 출력도 명시적 degraded WARN
상태일 뿐 현재 planner/controller 입력이 아니다.
