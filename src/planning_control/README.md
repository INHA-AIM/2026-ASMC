# Planning/Control

현재 대회 운용의 Frenet Planner와 MPC Controller는 정본 문서대로 standalone UDP 경로를
사용한다. 카메라 인지는 MORAI UDP bridge 뒤의 내부 ROS 토픽을 사용하며, 아래
`lane_boundary_adapter`는 그 관측 계약을 판제와 검토하기 위한 별도 ROS pilot이다. 이
pilot은 기존 planner/controller 실행 경로나 차량 제어 명령에 연결되지 않는다.

| 프로젝트 | 담당 | 빌드 |
|---|---|---|
| `src/frenet_planner/` | 이강태 | CMake, C++17 |
| `src/mpc_controller/` | 정윤태·양서준 | CMake, C++17 |
| `src/behavior_planner/` | 정윤태·양서준 | 현재 UDP 파이프라인 미연동 |
| `src/planning_control/lane_boundary_adapter/` | 안승현·판제 공동 검토 | Catkin, C++17, read-only ROS pilot |

```bash
./scripts/build_planning_control.sh
./scripts/run_planner_controller.sh
```

정본 규약과 포트표는 [docs/planning-control-udp.md](../../docs/planning-control-udp.md)를 따른다. ROS 모듈은 별도로 `./scripts/build_ws.sh`로 빌드한다.

카메라 경계 관측을 제어와 분리해 확인하려면
[lane boundary adapter README](./lane_boundary_adapter/README.md)를 따른다. 해당 출력은
Marker/diagnostics 전용이며 canonical 판제 입력 토픽은 공동 검증 후 별도 합의한다. 음영
왕복 2차로에서 관측 중앙선이 잠깐 누락될 때의 inferred-center 출력도 명시적 degraded WARN
상태일 뿐 현재 planner/controller 입력이 아니다.
