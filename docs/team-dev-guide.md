# 팀원별 개발·검증 가이드

> **문서 역할:** Active Guide — 팀원별 담당 경로와 검증 책임
> **정본 연결:** ownership 정책은 [collaboration.md](./collaboration.md), 전체 문서 지도는 [README.md](./README.md)
> **Planning/Control 변경:** §1–3의 ROS topic·launch 기반 설명은 이전 구현 참고용이다.
> 현재 작업 경로는 `src/frenet_planner/`, `src/mpc_controller/`이며 빌드·실행·포트는
> [planning-control-udp.md](./planning-control-udp.md)를 우선 적용한다. Behavior Planner는 미연동이다.

> **최종 수정:** 2026-09-05
> **Git:** [collaboration.md](./collaboration.md) — 현재는 Phase 1이며 최신 `main` 기반의 짧은 feature 브랜치와 PR을 사용한다.
> **네트워크:** [network-udp-ros.md](./network-udp-ros.md) · **인지 eval:** [perception-agentic-loop.md](./perception-agentic-loop.md)
> **인지 아키텍처:** camera/MGeo/LiDAR Working Proposal은 [perception-map-fusion-roadmap.md](./perception-map-fusion-roadmap.md)를 우선한다.
> 로컬 현황: [`docs/competition/team-status.md`](../../docs/competition/team-status.md)

관련: [sim-verification-checklist.md](./sim-verification-checklist.md) · [repository-layout.md](./repository-layout.md) · [git-workflow.md](./git-workflow.md) · [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md)

### Phase 0 개발 위치 기록 (참고)

| 팀원 | Phase 0 작업 위치 | ASMC 이식 시기 |
|------|-------------------|----------------|
| 안승현 | **ASMC** `drivable_bev/`, `tools/perception_eval/`, VIP2 차선 이식 | 지금~ (후보 실험) |
| 장원태 | `external/team/traffic_runner/` | Wave 2 |
| 이강태 | `external/team/aim_ws-va/` (bridge·판단) | Wave 2 |
| 손재호 | `external/team/morai-3d-detection/` (+ ASMC 기존 패키지) | 지속·Wave 2 |
| 정윤태·양서준 | `external/team/mpc_ws/` | Wave 2 |

### GitHub 계정 (CODEOWNERS · Path B 리뷰)

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
| 베이스 | **`main` (trunk)** — 일상 합류점 |
| Path A (기본) | 소유 모듈만 수정 → `main`에 커밋·푸시 |
| Path B | `feature/<이름>-<기능>` + PR — interfaces/UDP/docker/교차/대형 |
| 금지 | 장기 개인 브랜치 누적, 개인 upstream에만 새 코드 |

결정 트리·ownership: [collaboration.md §2~§4](./collaboration.md#2-한-장-요약-외울-것)

### 공통으로 손대면 안 되는 / Path B 필수

| 경로 | 규칙 |
|------|------|
| `src/MORAI-ROS_morai_msgs/` | submodule — **Path B + 통합 리드만** 핀 변경 |
| `docker/`, `.github/` | **Path B** · 안승현 리뷰 |
| `src/interfaces/` | **항상 Path B** · 3파트 합의 (BEV는 Phase C) |
| `src/integration_launch/` | 타 파트 연결·토픽 → **Path B** |
| UDP / `scripts/udp_bridge.sh` · rosbridge `scripts/rosbridge.sh` | 시뮬 경계 → **Path B** |
| `docs/` 규약 | 통합 리드 또는 Path B |

### 공통 시작 명령

```bash
cd "$ASMC"   # 또는 clone 경로
git checkout main && git pull --rebase origin main
git submodule update --init --recursive

# Path A: 이대로 소유 경로만 수정 후 commit → push origin main
# Path B: git checkout -b feature/<이름>-<기능>

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
| C3 | SIM/bridge profile | `./scripts/udp_bridge.sh --simulator 26r1` 또는 `competition` + 진단 |
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
| `src/mpc_controller/config/mpc_params.yaml` | horizon, cost, curve_* |
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
| UDP `Ctrl_cmd` 전환은 **Path B** (`feature/yuntae-udp-…`) | 시뮬 경계 · ROS 토픽 경로와 분리 검증 |
| `link_set.json`(대용량)은 함부로 포맷 변경하지 말 것 | behavior·맵 로더 의존 |

### Git Path 안내

| Path | 예 |
|------|-----|
| **A** (`main`) | MPC cost·behavior 룰 튜닝 (소유 경로만) |
| **B** (PR) | `feature/yuntae-udp-ctrl-cmd` (UDP), `feature/yuntae-behavior-tl` (TL↔FSM 최초 연결) |

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
| `src/mpc_controller/config/mpc_params.yaml` | `curve_lookahead_m` 등 |
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

### Git Path 안내

| Path | 예 |
|------|-----|
| **A** (`main`) | cost·curve velocity 튜닝 |
| **B** (PR) | msg/토픽 계약이 바뀌는 변경, launch 연동 |

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

### Git Path 안내

| Path | 예 |
|------|-----|
| **A** (`main`) | Frenet cost·lane change 로직 (planner만) |
| **B** (PR) | `behavior_planner` msg와 동시 변경, `feature/kangtae-…` |

---

## 4. 안승현 (카메라 인지 · learning · bridge · 통합 리드)

### 개발 위치 (주 소유)

| 경로 | 역할 |
|------|------|
| `src/perception/twinlite_morai/` | TwinLiteNet+ MORAI 데이터·학습·평가 adapter |
| `src/perception/camera_semantic_perception/` | 모델 독립 live 추론·시각화·성능계측 |
| `src/perception/drivable_bev/` | 3-camera BEV와 음영 shared lane-family v3; 전역 map 결합은 공동 작업 |
| `src/data_collection/` | Capture GT·mask·dataset, Real Time UDP raw/result bag |
| `src/learning_by_cheating/` | LBC BEV·학습·자율주행 |
| `scripts/rosbridge.sh` | 종료된 direct ROS 실험 재현용 WebSocket |
| `scripts/udp_bridge.sh` | 26.R1.H3/대회판 공용 UDP bridge와 실행 profile |
| `scripts/diagnose_morai_rosbridge.sh`, `wait_morai_topic.sh` | 진단 |
| `config/morai_rosbridge.env` | 종료된 direct ROS 실험 설정 |
| `mgeo_toolkit/` | HD map bake |
| `R_KR_PG_KATRI/` | 맵 JSON 자산 |
| `docker/`, `docs/` | 환경·문서 |
| `src/interfaces/README.md` | τ·BEV 합의 초안 (주관) |

### 자주 수정하는 파일

| 파일 | 용도 |
|------|------|
| `scripts/train_twinlite.py`, `evaluate_twinlite.py` | 2-head 학습·고정 평가 |
| `scripts/render_twinlite_predictions.py` | 동일 frame offline overlay |
| `scripts/bake_perception_masks.py`, `build_perception_dataset.py` | GT mask·versioned dataset 생성 |
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
| `scripts/rosbridge.sh` | 종료된 direct ROS 실험 재현용 포트 9090 (WebSocket) |

### 검증

checklist **§2, §3**

```bash
./scripts/udp_bridge.sh --simulator 26r1
# 대회판: ./scripts/udp_bridge.sh --simulator competition
# 과거 direct ROS 실패 재현만: ./scripts/rosbridge.sh
python3 src/learning_by_cheating/scripts/verify_lbc_spec.py
./src/learning_by_cheating/scripts/start_lbc_imshow.sh
```

### 추가 개발 시 주의

| 주의 | 이유 |
|------|------|
| camera live 전에는 공용 BEV 메시지를 확정하지 않음 | 실제 head 범위·신뢰도·처리율 확인 후 합의 |
| 추론과 시각화/기록을 별도 프로세스로 유지 | 모델 교체와 성능계측 오염 방지 |
| 26.R1.h3 model/backend와 MORAI adapter 분리 | 10월 대회판 호환 이식 범위 축소 |
| LBC GT는 **va ROS visualizer** 기준 유지 | jang gRPC multi-TL은 D-09 전까지 미통합 |
| `CtrlCmd.steering` / ego `wheel_angle` | beta 정책 |
| privileged(`Object_topic`)는 **학습·로깅만**, 배포 추론 경로 금지 | 대회 규정 |
| NPZ·체크포인트 git 커밋 금지 | `$ASMC_DATA` / NAS |
| τ·BEV 확정 전 planner 토픽과 이름 맞출 것 | `integrate/interfaces` |

### Git Path 안내

| Path | 예 |
|------|-----|
| **A** (`main`) | LBC/logger 내부, docs 소수정정 |
| **B** (PR) | `feature/seunghyun-udp-bridge`, interfaces, docker, submodule |

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

### Git Path 안내

| Path | 예 |
|------|-----|
| **A** (`main`) | scenario runner·gRPC 도구 내부 |
| **B** (PR) | perception live와 토픽 계약 (`feature/jang-scenario-dataset-ctrl`) |

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

### Git Path 안내

| Path | 예 |
|------|-----|
| **A** (`main`) | Det3D 모델·NMS·추론 노드 내부 |
| **B** (PR) | planner가 구독하는 출력 토픽 변경, `feature/jaeho-…` |

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
| `src/learning/README.md` | `learning_by_cheating/` (+ `mgeo_toolkit/`, `scripts/rosbridge.sh`) |
| `src/perception/README.md` | `perception/morai_3d_detection/` (+ `tools/` 시나리오) |
| `src/integration/README.md` | `scripts/rosbridge.sh`, `scripts/udp_bridge.sh`, `tools/grpc_inha_univ/` |
| `src/interfaces/README.md` | 스펙 초안만 (코드 패키지 아님) |

**코드는 README 폴더가 아니라 위 실제 경로에서 수정한다.**

---

## 9. 다음 우선순위 (통합 이후)

| 순위 | 작업 | 담당 |
|------|------|------|
| 1 | 센서셋·규정·수집 모드·차량 좌표계 정본화 | 완료, 팀 검토 |
| 2 | Real Time UDP raw/result bag | 구현 완료, 현재 재현·성능 기준 |
| 3 | Streaming/ROS GT transport 조사 | 종료, `origin/backup/ros-gt-experimental-20260910` 보존 |
| 4 | GPS lever arm 검증과 camera/map 공통 BEV 기준 | 안승현·장원태 |
| 5 | 음영 3-camera BEV/lane-family v3 | 구현·판제 인수 완료 |
| 6 | MGeo crop과 정상 GPS/Blackout source policy 평가 | 안승현·장원태 |
| 7 | LiDAR association, safe corridor ownership과 판제 interface | 인지·planning/control |
| 8 | 10월 대회판 데이터 보강·장기 재학습·안정화 이식 | 전원 |
