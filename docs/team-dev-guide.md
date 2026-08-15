# 팀원별 개발·검증 가이드

> **Planning/Control 변경:** §1–3의 ROS topic·launch 기반 설명은 이전 구현 참고용이다.
> 현재 작업 경로는 `src/frenet_planner/`, `src/mpc_controller/`이며 빌드·실행·포트는
> [planning-control-udp.md](./planning-control-udp.md)를 우선 적용한다. Behavior Planner는 미연동이다.

> **최종 수정:** 2026-07-08  
> **대상:** 팀원 전원 (통합 코드 이식 완료 후)  
> **원칙:** 새 작업은 **항상 `2026-ASMC` `main`에서 `feature/<이름>-…` 브랜치**로 한다. 개인 GitHub(`aim_ws`, `mpc_ws` 등)에는 **새 기능을 올리지 않는다.**

관련: [collaboration.md](./collaboration.md) · [sim-verification-checklist.md](./sim-verification-checklist.md) · [repository-layout.md](./repository-layout.md) · [git-workflow.md](./git-workflow.md) · [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md)

### GitHub 계정 (PR·CODEOWNERS)

| GitHub | 이름 | § |
|--------|------|---|
| @ahnsh03 | 안승현 | §4 |
| @yuntae12-sudo | 정윤태 | §1 |
| @yangseojun | 양서준 | §2 |
| @kante2 | 이강태 | §3 |
| @wkddnjsxo | 장원태 | §5 |
| @sonshiny | 손재호 | §6 |

---

## 0. 공통 (전원)

### 어디에 개발하나

| 항목 | 값 |
|------|-----|
| 레포 | https://github.com/INHAautonav/2026-ASMC |
| 베이스 | `main` (현재 planning-v2까지 merge 완료) |
| 브랜치 | `feature/<이름>-<기능>` — 예: `feature/yuntae-mpc-udp` |
| 금지 | `main` 직접 push, 개인 upstream에만 새 코드 누적 |

### 공통으로 손대면 안 되는 / 합의 필요한 것

| 경로 | 규칙 |
|------|------|
| `src/MORAI-ROS_morai_msgs/` | submodule — **직접 커밋 금지**. 브랜치·핀 변경은 통합 리드 |
| `docker/` | Dockerfile 변경 시 안승현 리뷰 |
| `src/interfaces/README.md` | τ·BEV 스펙 — 3파트 합의 후 |
| `src/integration_launch/` | ROS bridge launch 전용. Planning/Control 실행 추가 금지 |
| `docs/` (공용 규약) | 내용 변경은 통합 리드 또는 PR 리뷰 |

### 공통 시작 명령

```bash
cd "$ASMC"   # 또는 clone 경로
git checkout main && git pull
git submodule update --init --recursive
git checkout -b feature/<이름>-<기능>

./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash
# 컨테이너 안
./scripts/build_ws.sh
source devel/setup.bash
```

### 공통 검증 (시뮬 전)

| # | 확인 | 파일/명령 |
|---|------|-----------|
| C1 | submodule | `src/MORAI-ROS_morai_msgs` @ `beta_drive` |
| C2 | build | `./scripts/build_ws.sh` |
| C3 | bridge | `./scripts/bridge.sh` + `./scripts/diagnose_morai_bridge.sh` |
| C4 | ego | `rostopic echo /Ego_topic -n1` → `wheel_angle` 필드 존재 |

시뮬 항목 상세: [sim-verification-checklist.md](./sim-verification-checklist.md)

---

## 1. 정윤태 (planning lead · behavior · MPC · launch)

### 개발 위치 (주 소유)

| 경로 | 역할 |
|------|------|
| `src/behavior_planner/` | MGeo 기반 상위 의사결정 |
| `src/mpc_controller/` | MPC 추종 (`mpc_node` 패키지명) |
| `src/integration_launch/` | 3노드 통합 launch |

### 자주 수정하는 파일

| 파일 | 용도 |
|------|------|
| `src/behavior_planner/src/node/behavior_node.cpp` | ego/object 구독, `/behavior/context` publish |
| `src/behavior_planner/src/decision/behavior_decision.cpp` | 행동 선택 |
| `src/behavior_planner/src/rule/hard_rule_filter.cpp` | 정지선·신호·TTC 하드룰 |
| `src/behavior_planner/src/config/behavior_params.yaml` | 경로·토픽·안전 파라미터 (`$(find behavior_planner)` 유지) |
| `src/behavior_planner/src/config/route_links_1.yaml` | 주행 루트 링크 |
| `src/behavior_planner/launch/behavior_node.launch` | 노드·대시보드 |
| `src/mpc_controller/src/node/mpc_node.cpp` | 제어 루프, trajectory 구독 |
| `src/mpc_controller/src/planner/path_planner.cpp` | ref path / external trajectory |
| `src/mpc_controller/src/config/mpc_params.yaml` | horizon, cost, curve_* |
| `src/mpc_controller/src/config/path_route1.txt` | CSV fallback waypoint |
| `src/mpc_controller/launch/mpc_node.launch` | `waypoint_file`/`ref_file` 주입 |
| `src/integration_launch/launch/integration.launch` | behavior → planner → mpc |

### 검증 (본인 담당)

[sim-verification-checklist.md](./sim-verification-checklist.md) **§1.1–1.4, 1.7–1.9**

```bash
roslaunch integration_launch integration.launch
rostopic echo /behavior/context -n1
# planner 켠 상태 vs 끈 상태: MPC trajectory vs CSV fallback
```

핸드오프 계약: [planning-behavior-handoff.md](./planning-behavior-handoff.md)

### 추가 개발 시 주의

| 주의 | 이유 |
|------|------|
| `behavior_params.yaml`에 `/home/...` 절대경로 금지 | clone/컨테이너 깨짐 → `$(find behavior_planner)/…` |
| `EgoVehicleStatus`는 **`wheel_angle`** (beta) | `front_steer` / `angular_velocity` 금지 |
| planner 메시지 스키마 바꾸면 `PlanFeedback.msg`·bridge와 **동시** 수정 | 계약 문서 §1–2 |
| UDP `Ctrl_cmd` 전환은 `feature/yuntae-*` | ROS 토픽 경로와 분리 검증 |
| `link_set.json`(대용량)은 함부로 포맷 변경하지 말 것 | behavior·맵 로더 의존 |

### 브랜치 예

`feature/yuntae-mpc-udp`, `feature/yuntae-behavior-tl`

---

## 2. 양서준 (MPC · behavior 공조)

### 개발 위치 (공유 소유)

| 경로 | 역할 |
|------|------|
| `src/mpc_controller/` | 추종·제약·곡률 속도 |
| `src/behavior_planner/` | 스코어·룰 (정윤태와 공조) |

### 자주 수정하는 파일

| 파일 | 용도 |
|------|------|
| `src/mpc_controller/src/solver/mpc_solver.cpp` | 최적화 |
| `src/mpc_controller/src/cost/cost_function.cpp` | cost |
| `src/mpc_controller/src/constraints/constraints.cpp` | steer/accel 제약 |
| `src/mpc_controller/src/model/vehicle_model.cpp` | bicycle model |
| `src/mpc_controller/src/config/mpc_params.yaml` | `curve_lookahead_m` 등 |
| `src/behavior_planner/src/decision/behavior_scorer.cpp` | 점수 |
| `src/behavior_planner/src/feature/feature_builder.cpp` | urban feature |

### 검증

checklist **§1.7, 1.9** (MPC tracking, 저속·곡률)

### 추가 개발 시 주의

| 주의 | 이유 |
|------|------|
| wheelbase **3.0 m**, steer max **~40°** (`0.6981 rad`) 유지 | architecture·대회 스펙 |
| trajectory 레이아웃 `[n, x[], y[], yaw[], kappa[], v[], a[]]` 깨지 말 것 | planner↔mpc 계약 |
| planner/`behavior_bridge` 단독 수정 금지 | 이강태·정윤태 합의 |

### 브랜치 예

`feature/yang-mpc-cost`, `feature/yang-curve-vel`

---

## 3. 이강태 (Frenet planner)

### 개발 위치 (주 소유)

| 경로 | 역할 |
|------|------|
| `src/planner/` | Frenet 후보 생성·선택·피드백 |

### 자주 수정하는 파일

| 파일 | 용도 |
|------|------|
| `src/planner/src/main.cpp` | 루프, `/Ego_topic`, behavior 구독, trajectory publish |
| `src/planner/src/global/behavior_bridge.cpp` | `BehaviorContext` → `PlannerCommand` |
| `src/planner/src/global/behavior_bridge.hpp` | 매핑 API |
| `src/planner/src/frenet/path_generator.cpp` | 후보 샘플링 |
| `src/planner/src/frenet/ref_line.cpp` | 참조선 |
| `src/planner/src/frenet/collision_checker.cpp` | 충돌 필터 |
| `src/planner/src/frenet/cost.hpp` | 비용 |
| `src/planner/src/math/frenet_converter.cpp` | 좌표 변환 |
| `src/planner/src/config/params.yaml` | `lane_width`, timeout 등 |
| `src/planner/launch/planner_node.launch` | `waypoint_file` → `$(find mpc_node)/…/path_route1.txt` |

### 검증

checklist **§1.5, 1.6** (`/planner/plan_feedback`, `/frenet_planner/trajectory`)

```bash
rostopic echo /planner/plan_feedback -n1
rostopic hz /frenet_planner/trajectory
```

### 추가 개발 시 주의

| 주의 | 이유 |
|------|------|
| **ochang / `gps_logger` 이식 금지** | 팀 결정 — skip |
| FSM 디렉터리 복구하지 말 것 | planning-v2에서 제거, behavior이 상위 의사결정 |
| `wheel_angle` 단위(deg→rad) TODO 검증 후 수정 | `main.cpp` 주석 |
| `behavior_planner` 메시지 필드 변경 시 bridge·msg **동시** | handoff 문서 |
| 절대경로 waypoint 금지 — launch `$(find …)`만 |

### 브랜치 예

`feature/kangtae-frenet-cost`, `feature/kangtae-lane-change`

---

## 4. 안승현 (learning · bridge · 통합 리드)

### 개발 위치 (주 소유)

| 경로 | 역할 |
|------|------|
| `src/learning_by_cheating/` | LBC BEV·학습·자율주행 |
| `scripts/bridge.sh` | rosbridge |
| `scripts/diagnose_morai_bridge.sh`, `wait_morai_topic.sh` | 진단 |
| `config/morai_bridge.env` | bridge 설정 |
| `mgeo_toolkit/` | HD map bake |
| `R_KR_PG_KATRI/` | 맵 JSON 자산 |
| `docker/`, `docs/` | 환경·문서 |
| `src/interfaces/README.md` | τ·BEV 합의 초안 (주관) |

### 자주 수정하는 파일

| 파일 | 용도 |
|------|------|
| `src/learning_by_cheating/lbc_bev/morai_adapters.py` | ROS msg → BEV |
| `src/learning_by_cheating/lbc_bev/renderer.py` | BEV 렌더 |
| `src/learning_by_cheating/lbc_bev/spec.py` | 채널·해상도 |
| `src/learning_by_cheating/scripts/lbc_bev_collector.py` | NPZ 수집 |
| `src/learning_by_cheating/scripts/lbc_bev_visualizer.py` | ROS visualizer (va 기준) |
| `src/learning_by_cheating/scripts/start_lbc_morai.sh` | 수집 런처 |
| `src/learning_by_cheating/scripts/start_lbc_imshow.sh` | 시각 확인 |
| `src/learning_by_cheating/scripts/verify_lbc_spec.py` | 스펙 단위 테스트 |
| `src/learning_by_cheating/autonomous_driving.py` | SA 추론 → `/ctrl_cmd` |
| `src/learning_by_cheating/train.py`, `train_v2.py` | 학습 |
| `scripts/bridge.sh` | 포트 9090 |

### 검증

checklist **§2, §3**

```bash
./scripts/bridge.sh
python3 src/learning_by_cheating/scripts/verify_lbc_spec.py
./src/learning_by_cheating/scripts/start_lbc_imshow.sh
```

### 추가 개발 시 주의

| 주의 | 이유 |
|------|------|
| LBC GT는 **va ROS visualizer** 기준 유지 | jang gRPC multi-TL은 D-09 전까지 미통합 |
| `CtrlCmd.steering` / ego `wheel_angle` | beta 정책 |
| privileged(`Object_topic`)는 **학습·로깅만**, 배포 추론 경로 금지 | 대회 규정 |
| NPZ·체크포인트 git 커밋 금지 | `$ASMC_DATA` / NAS |
| τ·BEV 확정 전 planner 토픽과 이름 맞출 것 | `integrate/interfaces` |

### 브랜치 예

`feature/seunghyun-lbc-train`, `feature/seunghyun-sync-logger`

---

## 5. 장원태 (scenario · gRPC)

### 개발 위치 (주 소유)

| 경로 | 역할 |
|------|------|
| `tools/aim_scenario_runner/` | 시나리오 오케스트레이션 |
| `tools/grpc_inha_univ/` | MORAI gRPC / ROS↔gRPC |

### 자주 수정하는 파일

| 파일 | 용도 |
|------|------|
| `tools/aim_scenario_runner/scenario_launcher.py` | 엔트리, workspace root 탐지 |
| `tools/aim_scenario_runner/scenario_base.py` | 시나리오 베이스 |
| `tools/aim_scenario_runner/run.sh` | 실행 래퍼 |
| `tools/aim_scenario_runner/config/runtime.yaml` | host, 모드 |
| `tools/aim_scenario_runner/config/urban_scenarios.yaml` | 시나리오 정의 |
| `tools/aim_scenario_runner/config/urban_route_links.yaml` | 루트 |
| `tools/aim_scenario_runner/utils/morai_sim_bridge.py` | 시뮬 연동 |
| `tools/aim_scenario_runner/tools/save_ego_pose.py` | ego 저장 |
| `tools/grpc_inha_univ/src/api/morai_sim_client.py` | gRPC 클라이언트 |
| `tools/grpc_inha_univ/src/ros_ctrl_to_grpc_bridge.py` | 제어 브릿지 |
| `tools/grpc_inha_univ/run_mode.sh` | 모드 실행 |

### 검증

checklist **§4**

```bash
# runtime.yaml 의 gRPC host = WSL→Windows IP
./tools/aim_scenario_runner/run.sh
```

### 추가 개발 시 주의

| 주의 | 이유 |
|------|------|
| **jang LBC gRPC visualizer를 learning에 바로 merge하지 말 것** | D-09 검증 후 |
| `route_bev_visualizer_enabled` 기본 **false** 유지 | LBC viz 충돌 방지 |
| `/home/jang/...` 하드코딩 금지 | ASMC 경로·`_detect_workspace_root()` |
| D-10: `/dataset_control` publisher는 손재호와 **공동 PR** | perception live 연동 |
| Camera/LiDAR 레거시 `aim_ws/src/main` 통째 복사 금지 | 필요 시 파일 단위·합의 |

### 브랜치 예

`feature/jang-scenario-dataset-ctrl`, `feature/jang-grpc-host`

---

## 6. 손재호 (3D detection · perception train)

### 개발 위치 (주 소유)

| 경로 | 역할 |
|------|------|
| `src/perception/morai_3d_detection/` | 학습·추론·라이브 수집 |
| `docker/perception-train/` | CUDA 학습 이미지 |
| `scripts/docker_perception_up.sh` | 컨테이너 헬퍼 |

### 자주 수정하는 파일

| 파일 | 용도 |
|------|------|
| `src/perception/morai_3d_detection/train.py` | 학습 엔트리 |
| `src/perception/morai_3d_detection/inference.py` | 추론 |
| `src/perception/morai_3d_detection/morai_3d_live.py` | ROS 라이브 라벨 |
| `src/perception/morai_3d_detection/morai_dataset.py` | Dataset |
| `src/perception/morai_3d_detection/decoder.py` | 디코더 |
| `src/perception/morai_3d_detection/camera_configs.py` | 카메라 |
| `src/perception/morai_3d_detection/SETUP_NOTES.md` | 환경 노트 |
| `docker/perception-train/Dockerfile` | torch/CUDA |
| `docker/perception-train/requirements-train.txt` | pip |

### 검증

checklist **§5**

```bash
./scripts/docker_perception_up.sh up
# 컨테이너에서 train.py 1 epoch smoke
# ros-noetic에서 morai_3d_live.py
```

### 추가 개발 시 주의

| 주의 | 이유 |
|------|------|
| `dataset/`, `*.pth` **git 금지** | `$ASMC_DATA` / NAS |
| 개인 경로(`/home/jang/dataset`) 하드코딩 금지 | `dataset_root` 인자·환경변수 |
| D-10: upstream `adb1056`의 `/dataset_control`는 **scenario publisher와 세트**로만 이식 | 반쪽 연동 방지 |
| ROS live는 `docker/ros-noetic`, 학습은 `perception-train` 분리 | 이미지 목적 다름 |
| beta msg 쓰는 노드면 [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md) | |

### 브랜치 예

`feature/jaeho-inference-node`, `feature/jaeho-dataset-ctrl`

---

## 7. 파트 간 인터페이스 (공동)

확정 전 초안: `src/interfaces/README.md`, [architecture-overview.md](./architecture-overview.md) §5

| 토픽/데이터 | 발행 | 구독 | 비고 |
|-------------|------|------|------|
| `/Ego_topic` | MORAI bridge | 전원 | `wheel_angle` |
| `/Object_topic` | MORAI bridge | behavior, LBC(학습) | 배포 추론 금지 |
| `/behavior/context` | `behavior_planner` | `planner` | BehaviorContext.msg |
| `/planner/plan_feedback` | `planner` | `behavior_planner` | PlanFeedback.msg |
| `/frenet_planner/trajectory` | `planner` | `mpc_node` | Float32MultiArray |
| `/ctrl_cmd` | mpc / LBC AD | MORAI | `steering` |
| τ (학습↔MPC) | **미확정** | | `integrate/interfaces` |
| `/dataset_control` | scenario (예정) | `morai_3d_live` (예정) | D-10 |

---

## 8. 논리 그룹 README vs 실제 코드

| 논리 README (문서만) | 실제 코드 |
|----------------------|-----------|
| `src/planning_control/README.md` | `behavior_planner/`, `planner/`, `mpc_controller/`, `integration_launch/` |
| `src/learning/README.md` | `learning_by_cheating/` (+ `mgeo_toolkit/`, `scripts/bridge.sh`) |
| `src/perception/README.md` | `perception/morai_3d_detection/` (+ `tools/` 시나리오) |
| `src/integration/README.md` | `scripts/bridge.sh`, `tools/grpc_inha_univ/` (확장 예정) |
| `src/interfaces/README.md` | 스펙 초안만 (코드 패키지 아님) |

**코드는 README 폴더가 아니라 위 실제 경로에서 수정한다.**

---

## 9. 다음 우선순위 (통합 이후)

| 순위 | 작업 | 담당 |
|------|------|------|
| 1 | 시뮬 검증 §1–5 수행·기록 | 각자 |
| 2 | `integrate/interfaces` τ·BEV 확정 | 안승현 주관 + 전 파트 |
| 3 | MPC UDP / perception 추론 노드 | 정윤태, 손재호 |
| 4 | D-10 scenario↔collector | 장원태 + 손재호 |
| 5 | D-09 TL gRPC (선택) | 장원태 + 안승현 |
