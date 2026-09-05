# 저장소 레이아웃·파트 격리

> **문서 역할:** Active Reference — 실제 코드 위치와 ownership 요약
> **문서 허브:** [README.md](./README.md)
> **최종 수정:** 2026-09-05
> 팀원별 상세 경로·검증: [team-dev-guide.md](./team-dev-guide.md)  
> Git 규약 (trunk · ownership · Path A/B): [collaboration.md](./collaboration.md)

## 1. 최상위 구조

```
2026-ASMC/
├── docs/                 # 팀 문서
├── docker/
│   ├── ros-noetic/       # ROS catkin 개발 (+ libyaml-cpp)
│   └── perception-train/ # PyTorch 학습
├── src/                  # catkin 패키지 + 논리 README
├── tools/                # scenario runner, grpc (non-catkin)
├── scripts/              # bridge, build, docker 헬퍼
├── config/               # morai_rosbridge.env (WebSocket; UDP는 launch)
├── mgeo_toolkit/         # 기존 HD map bake 도구
├── R_KR_PG_KATRI/        # 기존 KATRI 맵 자산; K-City 대회 데이터는 $ASMC_DATA
├── CONTRIBUTING.md
└── README.md
```

**Git에 넣지 않음:** `build/`, `devel/`, `data/`, `checkpoints/`, `.bag`, `*.pth`

## 2. catkin·코드 패키지

ROS 패키지와 standalone 프로젝트는 **`src/<모듈명>/`** 바로 아래 둔다. 논리 그룹은 `src/<그룹>/README.md`만 둔다.

### planning_control (✅ PR #1·#4)

| 패키지 | 경로 | 담당 | GitHub |
|--------|------|------|--------|
| Behavior planner | `src/behavior_planner/` | 정윤태·양서준 | @yuntae12-sudo · @yangseojun |
| MPC (`mpc_node`, standalone UDP) | `src/mpc_controller/` | 정윤태·양서준 | @yuntae12-sudo · @yangseojun |
| Frenet planner (standalone UDP) | `src/frenet_planner/` | 이강태 | @kante2 |
| ROS bridge launch | `src/integration_launch/` | 정윤태 | @yuntae12-sudo |

현재 실행 흐름: [planning-control-udp.md](./planning-control-udp.md). 이전 ROS 통합 기록은
[planning-behavior-handoff.md](./planning-behavior-handoff.md)에 보존한다.

### learning (✅ PR #2)

| 경로 | 담당 | GitHub |
|------|------|--------|
| `src/learning_by_cheating/` | 안승현 | @ahnsh03 |
| `scripts/rosbridge.sh`, `config/morai_rosbridge.env` | 안승현 | @ahnsh03 |
| `scripts/udp_bridge.sh`, `src/morai_*_bridge/` | 이강태·안승현 | @kante2 · @ahnsh03 |
| `mgeo_toolkit/`, `R_KR_PG_KATRI/` | 안승현 | @ahnsh03 |

### perception · scenario (✅ PR #3)

| 경로 | 담당 | GitHub |
|------|------|--------|
| `src/perception/morai_3d_detection/` | 손재호 | @sonshiny |
| `src/perception/twinlite_morai/` | 안승현 | @ahnsh03 |
| `src/perception/drivable_bev/` | 인지·맵·LiDAR 공동 후보 | @ahnsh03 주관 협의 |
| `src/data_collection/` | Capture GT·mask·dataset | @ahnsh03 |
| `tools/aim_scenario_runner/` | 장원태 | @wkddnjsxo |
| `tools/grpc_inha_univ/` | 장원태 | @wkddnjsxo |

### interfaces (⬜ 합의 중)

| 경로 | 내용 | 주관 |
|------|------|------|
| `src/interfaces/README.md` | τ·BEV 초안 — [architecture-overview.md](./architecture-overview.md) §5 | @ahnsh03 (3파트 합의) |

### 공통

| 패키지 | 경로 | GitHub |
|--------|------|--------|
| morai_msgs | `src/MORAI-ROS_morai_msgs` (submodule, `beta_drive`) | @ahnsh03 (핀·브랜치 변경) |

## 3. 수정 권한 매트릭스

| 경로 | @yuntae12-sudo<br>정윤태 | @yangseojun<br>양서준 | @kante2<br>이강태 | @ahnsh03<br>안승현 | @wkddnjsxo<br>장원태 | @sonshiny<br>손재호 |
|------|--------|--------|--------|--------|--------|--------|
| `src/behavior_planner/` | ✅ | ✅ | 👀 | 👀 | — | — |
| `src/mpc_controller/` | ✅ | ✅ | 👀 | 👀 | — | — |
| `src/frenet_planner/` | 👀 | 👀 | ✅ | 👀 | — | — |
| `src/integration_launch/` | ✅ | 👀 | 👀 | 👀 | — | — |
| `src/learning_by_cheating/` | — | — | — | ✅ | — | — |
| `scripts/rosbridge.sh` | 👀 | 👀 | 👀 | ✅ | — | — |
| `scripts/udp_bridge.sh` · `src/morai_*_bridge/` | 👀 | 👀 | ✅ | ✅ | — | — |
| `tools/aim_scenario_runner/` | — | — | — | 👀 | ✅ | 👀 |
| `tools/grpc_inha_univ/` | — | — | — | 👀 | ✅ | — |
| `src/perception/morai_3d_detection/` | — | — | — | 👀 | 👀 | ✅ |
| `src/perception/twinlite_morai/` · `src/data_collection/` | — | — | — | ✅ | 👀 | 👀 |
| `src/perception/drivable_bev/` | 👀 | 👀 | 👀 | ✅ 주관 | ✅ 합의 | ✅ 합의 |
| `docker/` | 👀 | 👀 | 👀 | ✅ | 👀 | ✅ train |
| `docs/` | 👀 | 👀 | 👀 | ✅ | 👀 | 👀 |
| `src/interfaces/` | ✅ 합의 | ✅ 합의 | ✅ 합의 | ✅ 주관 | ✅ 합의 | ✅ 합의 |

✅ 주 수정 · 👀 리뷰·합의 · — 원칙적 미수정

CODEOWNERS 자동 리뷰: [.github/CODEOWNERS](../.github/CODEOWNERS)

## 4. 레거시 upstream (본 repo 밖)

본 repo에 포함되지 않습니다. diff·이력 확인이 필요할 때 **각자 원하는 경로에 clone**합니다.

| 종류 | GitHub | 용도 |
|------|--------|------|
| planning (구 `mpc_ws`) | [yuntae12-sudo/mpc_ws](https://github.com/yuntae12-sudo/mpc_ws) | 이식 전 참조 |
| learning·scenario (구 `aim_ws`) | [kante2/aim_ws](https://github.com/kante2/aim_ws) | 브랜치별 레거시 |
| perception | [sonshiny/morai-3d-detection](https://github.com/sonshiny/morai-3d-detection) | 이식 전 참조 |
| morai_msgs (26.R1) | [MORAI-ROS_morai_msgs](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs) `26.R1` | beta diff 참고 |

**금지:** 외부 베이스라인·레거시 코드를 **본 repo에 통째 복사** (이식 PR로만 반영).  
매핑 상세: [integration-plan.md](./integration-plan.md)

## 5. 아키텍처 → 경로

| 모듈 | ASMC 경로 |
|------|-----------|
| Behavior / Frenet / MPC | `src/behavior_planner/`(미연동), `frenet_planner/`, `mpc_controller/` |
| Camera lane/drivable | `src/perception/twinlite_morai/`, `camera_semantic_perception/`(제안) |
| LiDAR/3D perception | `src/perception/morai_3d_detection/`, `src/lidar+sensorfusion/` |
| 공통 BEV | `src/perception/drivable_bev/`(후보) |
| Capture·dataset | `src/data_collection/`, `scripts/*perception*`, `scripts/*twinlite*` |
| SA_plan / LBC | `src/learning_by_cheating/` |
| Scenario / gRPC | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` |
| Bridge | `scripts/udp_bridge.sh` (UDP) · `scripts/rosbridge.sh` (WebSocket 개발용) |

상세 파일명: [team-dev-guide.md](./team-dev-guide.md)
