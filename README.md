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
| [docs/governance/collaboration.md](docs/governance/collaboration.md) | **★ Phase0/1 · WIP · Path A/B (필독)** |
| [docs/operations/network-udp-ros.md](docs/operations/network-udp-ros.md) | **UDP↔ROS (26.R1.h3 · Docker `127.0.0.1`)** |
| [docs/operations/planning-control.md](docs/operations/planning-control.md) | **Planning/Control UDP-only 정본** |
| [docs/archive/perception/agentic-loop.md](docs/archive/perception/agentic-loop.md) | Loc-Aid/bake 에이전틱 eval |
| [docs/packages/asmc_perception/roadmap.md](docs/packages/asmc_perception/roadmap.md) | **카메라·MGeo·LiDAR 인지 융합 및 대회판 이식 제안** |
| [docs/onboarding/getting-started.md](docs/onboarding/getting-started.md) | 클론·첫 빌드 |
| [docs/onboarding/team-dev-guide.md](docs/onboarding/team-dev-guide.md) | 팀원별 경로 |
| [docs/archive/integration/wave1-wave2-plan.md](docs/archive/integration/wave1-wave2-plan.md) | Wave 1/2 |
| [docs/operations/sim-verification-checklist.md](docs/operations/sim-verification-checklist.md) | 시뮬 검증 |
| [docs/architecture/system-overview.md](docs/architecture/system-overview.md) | 아키텍처 |
| [docs/governance/git-workflow.md](docs/governance/git-workflow.md) | Path A/B 명령 |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 기여 요약 |

## 빠른 시작

Windows 팀원은 최초 한 번 Docker Desktop **4.34 이상**에서 다음 설정을 적용해야 합니다.

1. Docker Desktop을 Linux containers/WSL 2 모드로 실행합니다.
2. **Settings → Resources → Network → Enable host networking**을 켭니다.
3. **Apply and restart**를 눌러 Docker Desktop 재시작까지 완료합니다.
4. **Settings → Resources → WSL Integration**에서 사용하는 WSL 배포판을 활성화합니다.

`Network` 또는 `Enable host networking`이 보이지 않으면 Docker Desktop을 업데이트하고
Windows containers 모드가 아닌지 확인합니다. 상세 확인법은
[UDP↔ROS 네트워크 가이드](docs/operations/network-udp-ros.md#31-docker-desktop-최초-설정팀원별-1회)를
따릅니다. MORAI의 기존 `127.0.0.1`과 센서 포트는 변경하지 않습니다.

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

> **GPU 추론 이미지 선택**: RTX 40 팀원은 위의 기본 `build`/`up`을 그대로
> 사용합니다(Python 3.8 + PyTorch 2.1.2/cu118). 이번 통합에서 기본 GPU stack은
> 변경하지 않았고 LiDAR 빌드 의존성과 기존 Ultralytics 8.4.150 고정만 반영했습니다.
> RTX 50 팀원은 최초 1회
> `./scripts/docker_ros_up.sh build-rtx50`, 실행할 때 `up-rtx50`을 사용합니다.
> RTX 50 이미지에서는 센서퓨전 YOLO와 신호등 ResNet18이 모두 격리된
> `/opt/yolo-venv`(Python 3.9 + PyTorch cu128)를 사용합니다. 두 이미지 모두
> NVIDIA 드라이버와 Container Toolkit은 **host**에 설치돼 있어야 하며,
> 컨테이너 안에 드라이버를 설치하면 안 됩니다. 확인·문제해결 절차는
> [src/lidar+sensorfusion/RUN_SENSOR_FUSION.md](src/lidar+sensorfusion/RUN_SENSOR_FUSION.md#camera-inference-gpu-cuda)를 보십시오.

상세: [docs/onboarding/getting-started.md](docs/onboarding/getting-started.md)

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
│   ├── perception/twinlite_morai/        # camera 2-head binary/4-class 학습·평가 ✅
│   ├── perception/morai_3d_detection/  # perception ✅
│   ├── morai_udp_bridge/      # UDP↔ROS (Ego/Ctrl) ✅
│   ├── morai_sensor_bridge/   # UDP↔ROS (sensors) ✅
│   └── MORAI-ROS_morai_msgs/  # submodule beta_drive
├── tools/
│   ├── aim_scenario_runner/   # scenario ✅
│   └── grpc_inha_univ/        # gRPC ✅
├── scripts/
│   ├── rosbridge.sh           # 종료된 direct ROS 실험 재현용(본선/현재 개발 금지)
│   ├── udp_bridge.sh          # UDP bridges (--simulator 26r1|competition)
│   └── …
├── mgeo_toolkit/ · R_KR_PG_KATRI/
└── config/morai_rosbridge.env # 위 실험의 레거시 설정
```

팀원별 **어느 폴더를 고칠지:** [docs/onboarding/team-dev-guide.md](docs/onboarding/team-dev-guide.md)

## Git (Phase 0/1 · Trunk)

- **Phase 0:** 기능은 개인 레포 OK. ASMC `main` = 빌드되는 문서/인프라/안승현 인지만. **WIP는 Draft PR** ([collaboration §2.5](docs/governance/collaboration.md))
- **Phase 1:** 소유 모듈 → `main` 자주 푸시; interfaces/UDP/docker/교차 → Path B
- 상세: [collaboration.md](docs/governance/collaboration.md) · [network-udp-ros.md](docs/operations/network-udp-ros.md)

## 레거시 · Phase 0 분산 코드

Phase 0에는 **개인/분산 레포**에서 기능을 완성한다 (`external/team/`).  
Wave 2(8/7~)에 ASMC로 이식. 매핑: [docs/archive/integration/wave1-wave2-plan.md](docs/archive/integration/wave1-wave2-plan.md) · [team-status](../docs/competition/team-status.md)
