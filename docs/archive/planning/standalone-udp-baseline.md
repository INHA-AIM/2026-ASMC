# Planning/Control standalone UDP 기준선

> **문서 역할:** Historical Runtime Baseline — ROS runtime 전환 전 standalone C++17+UDP 경로
>
> **Archive 허브:** [README.md](../README.md)
>
> **원문 위치:** `docs/operations/planning-control.md`
>
> **보존 시점:** 2026-09-22 D2 planning-control 통합 직전
>
> **현재 실행 금지:** 현재 소스는 ROS1/catkin runtime으로 전환됐다. 실행은
> [Planning/Control 운영 규약](../../operations/planning-control.md)을 따른다.

아래 내용은 이전 정본의 정보를 잃지 않기 위해 그대로 보존한 snapshot이다. 현재
`CMakeLists.txt`, executable, topic과 launch를 설명하지 않는다.

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
| MPC → MORAI CtrlCmd | `127.0.0.1:9111` (Docker Desktop host mode) |

위 값은 아직 적용 중인 개발 기준이다. Phase 3에서는 개발·현장 포트를 `9093/9094`로
통일하고 IP만 profile로 분리한다. 전환 전에는 임의로 바꾸지 않는다. 현재값과 목표값은
[운영 가이드 §1](../../operations/README.md#1-현재값과-승인된-후속-목표)을 따른다. 포트·IP는 알고리즘
소스가 아니라 각 모듈의 `src/udp_network/network.yaml`에서 관리한다.

## 대회 입력 규약

- 기본 localization은 GPS/IMU다.
- `ObjectInfo:7505`는 센서 기반 객체 입력이 들어오기 전까지 현재 대회 경로의 Planner 검증에 임시 사용한다.
- 현재 `object_info.enabled` 기본값은 `true`다.
- perception 객체 인터페이스가 연결되면 동일 역할을 교체하고 ObjectInfo receiver와 설정을 제거한다.
- MPC 차량 파라미터는 wheelbase `3.0 m`, steering limit `40°`를 유지한다.

## 최소 시뮬 gate

- global path load와 GPS/IMU localization 유효
- Planner `:9300` 송신과 MPC 수신
- 임시 ObjectInfo를 켠 경우 `:7505` 수신과 장애물 반응
- Ctrl 목적지·mode 확인 후 실제 차량 이동
- 급커브·저속 구간에서 경로 이탈과 조향 진동 없음
- Ctrl-C 뒤 두 process와 UDP socket 종료

## 당시 제거된 ROS 경로

- `src/planner/` ROS/catkin 패키지
- `src/mpc_controller/launch/` 및 ROS node/topic/parameter-server 경로
- `integration_launch/launch/integration.launch`
- `/behavior/context`, `/planner/plan_feedback`, `/frenet_planner/trajectory`, `/ctrl_cmd` 기반 Planning/Control 연결

이 목록은 당시 standalone 전환의 기록이다. 이후 현재 ROS runtime이 새 package 구조와
`asmc_msgs` 계약으로 다시 구현됐으므로 현재 파일 존재 여부를 판단하는 삭제 목록으로 쓰지 않는다.
