# 2026-ASMC

**2026 대학생 AI·SW 모빌리티 경진대회** (AI 융합 자율주행) 팀 통합 개발 저장소.

| 항목 | 값 |
|------|-----|
| 조직 | [INHAautonav](https://github.com/INHAautonav) |
| 팀원 | 정윤태, 양서준, 이강태, 안승현, 장원태, 손재호 |
| 통합 리드 | 안승현 |
| 상태 | 기능별 통합 개발 중 — 현재 정본과 작업 순서는 [문서 허브](docs/README.md) 참고 |

## 문서 (팀원 필독)

| 문서 | 내용 |
|------|------|
| [docs/README.md](docs/README.md) | **★ 정본·가이드·설계안·기록을 구분한 전체 문서 허브** |
| [docs/collaboration.md](docs/collaboration.md) | **★ Phase0/1 · WIP · Path A/B (필독)** |
| [docs/network-udp-ros.md](docs/network-udp-ros.md) | **UDP↔ROS (26.R1.h3 · Docker `127.0.0.1`)** |
| [docs/planning-control-udp.md](docs/planning-control-udp.md) | **Planning/Control UDP-only 정본** |
| [docs/perception-agentic-loop.md](docs/perception-agentic-loop.md) | Loc-Aid/bake 에이전틱 eval |
| [docs/perception-map-fusion-roadmap.md](docs/perception-map-fusion-roadmap.md) | **카메라·MGeo·LiDAR 인지 융합 및 대회판 이식 제안** |
| [docs/getting-started.md](docs/getting-started.md) | 클론·첫 빌드 |
| [docs/team-dev-guide.md](docs/team-dev-guide.md) | 팀원별 경로 |
| [docs/integration-plan.md](docs/integration-plan.md) | Wave 1/2 |
| [docs/sim-verification-checklist.md](docs/sim-verification-checklist.md) | 시뮬 검증 |
| [docs/architecture-overview.md](docs/architecture-overview.md) | 아키텍처 |
| [docs/git-workflow.md](docs/git-workflow.md) | Path A/B 명령 |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 기여 요약 |

## 빠른 시작

```bash
git clone --recurse-submodules https://github.com/INHAautonav/2026-ASMC.git
cd 2026-ASMC
git checkout main && git pull
git submodule update --init --recursive

xhost +local:docker
./scripts/docker_ros_up.sh build
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh check
docker exec -it asmc-ros-noetic bash -c './scripts/build_ws.sh'

# ROS 없는 Planning/Control
docker exec -it asmc-ros-noetic bash -c './scripts/build_planning_control.sh'
```

상세: [docs/getting-started.md](docs/getting-started.md)

## 저장소 구조 (요약)

```
2026-ASMC/
├── docs/                      # 팀 문서
├── docker/                    # ros-noetic · perception-train
├── src/
│   ├── behavior_planner/      # planning ✅
│   ├── frenet_planner/        # standalone UDP planning ✅
│   ├── mpc_controller/        # standalone UDP control ✅
│   ├── integration_launch/    # ROS bridge launch only
│   ├── learning_by_cheating/  # learning ✅
│   ├── data_collection/        # Capture GT · mask · dataset ✅
│   ├── perception/twinlite_morai/        # camera 2-head 학습·평가 ✅
│   ├── perception/morai_3d_detection/  # perception ✅
│   ├── morai_udp_bridge/      # UDP↔ROS (Ego/Ctrl) ✅
│   ├── morai_sensor_bridge/   # UDP↔ROS (sensors) ✅
│   └── MORAI-ROS_morai_msgs/  # submodule beta_drive
├── tools/
│   ├── aim_scenario_runner/   # scenario ✅
│   └── grpc_inha_univ/        # gRPC ✅
├── scripts/
│   ├── rosbridge.sh           # WebSocket rosbridge (개발)
│   ├── udp_bridge.sh          # UDP bridges (26 개발 / 대회 경로)
│   └── …
├── mgeo_toolkit/ · R_KR_PG_KATRI/
└── config/morai_rosbridge.env
```

팀원별 **어느 폴더를 고칠지:** [docs/team-dev-guide.md](docs/team-dev-guide.md)

## Git (Phase 0/1 · Trunk)

- **Phase 0:** 기능은 개인 레포 OK. ASMC `main` = 빌드되는 문서/인프라/안승현 인지만. **WIP는 Draft PR** ([collaboration §2.5](docs/collaboration.md))
- **Phase 1:** 소유 모듈 → `main` 자주 푸시; interfaces/UDP/docker/교차 → Path B
- 상세: [collaboration.md](docs/collaboration.md) · [network-udp-ros.md](docs/network-udp-ros.md)

## 레거시 · Phase 0 분산 코드

Phase 0에는 **개인/분산 레포**에서 기능을 완성한다 (`external/team/`).  
Wave 2(8/7~)에 ASMC로 이식. 매핑: [docs/integration-plan.md](docs/integration-plan.md) · [team-status](../docs/competition/team-status.md)
