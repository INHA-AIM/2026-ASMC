# Planning/Control UDP-only 규약

> **문서 역할:** Canonical — 현재 standalone Frenet/MPC 실행 규약
> **문서 허브:** [README.md](./README.md)
> **상태:** Active — Planning/Control의 정본
> **기준 upstream:** `yuntae12-sudo/mpc_ws` `465833e`

## 범위

Frenet Planner와 MPC Controller는 ROS, catkin, ROS topic, `morai_msgs`에 의존하지 않는 standalone C++17 프로그램으로 운용한다. Behavior Planner는 현재 파이프라인에 연결하지 않는다. 다른 모듈의 ROS 사용 여부는 각 파트에서 별도로 판단한다.

```text
MORAI GPS/IMU ─UDP─> frenet_planner_node
                         │ PlannedPath UDP :9300
                         v
                     mpc_node ─UDP CtrlCmd─> MORAI
```

## 빌드·실행

```bash
./scripts/build_planning_control.sh
./scripts/run_planner_controller.sh
```

`./scripts/build_ws.sh`는 ROS 모듈만 빌드한다. Planning/Control은 `src/frenet_planner/build`, `src/mpc_controller/build`에서 각각 plain CMake로 빌드한다.

본선 global path는 Git에 넣지 않고 `$ASMC_DATA`에서 컨테이너 `/data`로 읽기 전용 마운트한다. 기본 설정은 `/data/R_KR_PG_K-city_2025/2026_molit_comp_global_path.txt`를 사용한다.

## ASMC 개발 포트

| 데이터 | 주소/포트 |
|---|---|
| GPS 수신 | `0.0.0.0:9301` |
| IMU 수신 | `0.0.0.0:9303` |
| Planner → MPC | `127.0.0.1:9300` |
| MPC → MORAI CtrlCmd | `host.docker.internal:9111` |

대회 SIM에서는 현장 제공 포트를 다시 확인한다. 포트·IP는 알고리즘 소스가 아니라 각 모듈의 `src/udp_network/network.yaml`에서 관리한다.

## 대회 입력 규약

- 기본 localization은 GPS/IMU다.
- `ObjectInfo:7505`는 센서 기반 객체 입력이 들어오기 전까지 현재 대회 경로의 Planner 검증에 임시 사용한다.
- 현재 `object_info.enabled` 기본값은 `true`다.
- perception 객체 인터페이스가 연결되면 동일 역할을 교체하고 ObjectInfo receiver와 설정을 제거한다.
- MPC 차량 파라미터는 wheelbase `3.0 m`, steering limit `40°`를 유지한다.

## 제거된 ROS 경로

- `src/planner/` ROS/catkin 패키지
- `src/mpc_controller/launch/` 및 ROS node/topic/parameter-server 경로
- `integration_launch/launch/integration.launch`
- `/behavior/context`, `/planner/plan_feedback`, `/frenet_planner/trajectory`, `/ctrl_cmd` 기반 Planning/Control 연결

이전 구현은 Git 이력에서 확인한다. `docs/planning-behavior-handoff.md`는 레거시 ROS 설계 참고용이며 현재 실행 규약이 아니다.
