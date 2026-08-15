# Planning/Control

Planner와 Controller는 ROS를 사용하지 않는 standalone UDP 프로그램이다.

| 프로젝트 | 담당 | 빌드 |
|---|---|---|
| `src/frenet_planner/` | 이강태 | CMake, C++17 |
| `src/mpc_controller/` | 정윤태·양서준 | CMake, C++17 |
| `src/behavior_planner/` | 정윤태·양서준 | 현재 UDP 파이프라인 미연동 |

```bash
./scripts/build_planning_control.sh
./scripts/run_planner_controller.sh
```

정본 규약과 포트표는 [docs/planning-control-udp.md](../../docs/planning-control-udp.md)를 따른다. ROS 모듈은 별도로 `./scripts/build_ws.sh`로 빌드한다.
