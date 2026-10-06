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
| [문서 허브](docs/README.md) | 정본·가이드·설계안·기록의 전체 색인 |
| [개발 시작](docs/onboarding/README.md) | clone, Docker, 빌드와 환경 기준 |
| [실행 명령어 모음](docs/runtime-commands.md) | bridge·HD Map·perception·planning·bag 명령, 모든 공개 옵션과 LAN2LAN 준비 |
| [팀 ownership](docs/onboarding/ownership.md) | 담당 경로와 검증 책임 |
| [협업 규칙](docs/governance/README.md) | 브랜치, PR, 리뷰와 main 안정성 |
| [시스템 개요](docs/architecture/system-overview.md) | 현재 아키텍처와 모듈 경계 |
| [통합 로드맵](docs/architecture/consolidation-roadmap.md) | 패키지 통합부터 현장 배포까지의 순서 |
| [운영 가이드](docs/operations/README.md) | Docker, MORAI UDP, 시뮬 검증과 현장 전환 기준 |
| [검증·실험 기록](docs/records/README.md) | 날짜·commit·artifact별 baseline과 실험 결과 |
| [역사·레거시 기록](docs/archive/README.md) | 현재 실행 금지인 과거 설계와 대체 문서 |
| [기여 요약](CONTRIBUTING.md) | PR 전 최소 확인 사항 |

## 빠른 시작

Windows 팀원은 최초 한 번 Docker Desktop **4.34 이상**에서 다음 설정을 적용해야 합니다.

1. Docker Desktop을 Linux containers/WSL 2 모드로 실행합니다.
2. **Settings → Resources → Network → Enable host networking**을 켭니다.
3. **Apply and restart**를 눌러 Docker Desktop 재시작까지 완료합니다.
4. **Settings → Resources → WSL Integration**에서 사용하는 WSL 배포판을 활성화합니다.

`Network` 또는 `Enable host networking`이 보이지 않으면 Docker Desktop을 업데이트하고
Windows containers 모드가 아닌지 확인합니다. 상세 확인법은
[개발·시뮬레이터 운영 가이드](docs/operations/README.md)를
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

상세: [docs/onboarding/README.md](docs/onboarding/README.md)

## 저장소 구조 (요약)

```
2026-ASMC/
├── docs/                      # 팀 문서
├── docker/                    # ros-noetic · perception-train
├── src/
│   ├── behavior_planner/      # planning ✅
│   ├── frenet_planner/        # ROS trajectory planning + C++ core ✅
│   ├── asmc_controller/        # ROS trajectory control + C++ core ✅
│   ├── asmc_msgs/             # 팀 내부 ROS1 공용 message 계약
│   ├── asmc_bridge/           # MORAI Network/Sensor UDP + VehicleState
│   ├── asmc_hd_map/           # HD Map builder/runtime/검수
│   ├── asmc_camera/       # camera·BEV·lane·traffic runtime + offline/training
│   ├── lane_boundary_adapter/ # read-only perception→planning adapter
│   ├── data_collection/        # Capture GT · mask · dataset ✅
│   ├── lidar+sensorfusion/     # 독립 nested LiDAR/fusion workspace
│   └── MORAI-ROS_morai_msgs/  # submodule beta_drive
├── tools/
│   ├── aim_scenario_runner/   # scenario ✅
│   ├── research/lbc_teacher_bev/ # privileged GT/teacher BEV 연구 도구
│   └── grpc_inha_univ/        # gRPC ✅
├── scripts/
│   ├── rosbridge.sh           # 종료된 direct ROS 실험 재현용(본선/현재 개발 금지)
│   ├── udp_bridge.sh          # 통합 asmc_bridge 실행(local/lan2lan 선택)
│   └── …
├── mgeo_toolkit/ · R_KR_PG_KATRI/
└── config/morai_rosbridge.env # 위 실험의 레거시 설정
```

팀원별 **어느 폴더를 고칠지:** [docs/onboarding/ownership.md](docs/onboarding/ownership.md)

### Perception 경로 안내

P4-C6부터 camera semantic, camera BEV/lane, traffic-light의 실행·offline·학습
정본은 모두 `src/asmc_camera/`이다. 기존
`camera_semantic_perception`, `drivable_bev`, `traffic_light`, `src/perception/` 경로는
제거했으며 topic과 node 이름은 호환성을 위해 유지했다. 실시간 추론은
`asmc-ros-noetic`에서 `roslaunch asmc_camera perception.launch`로 실행하며, 세부
경로와 인수 경계는
[Perception package README](src/asmc_camera/README.md)를 따른다. `src/data_collection/`과
`src/lidar+sensorfusion/`은 독립 경계로 남아 있다.

## Git과 PR

- 최신 `main`에서 짧은 브랜치를 만든다.
- 모든 변경은 PR과 CODEOWNERS 리뷰를 거쳐 merge한다.
- WIP 공유는 Draft PR을 사용한다.
- 과거 Phase 0 개인 레포와 `main` 직푸시 정책은 현재 규칙이 아니다.

상세 명령은 [협업 규칙](docs/governance/README.md), 과거 이식 기록은
[Wave 1/2 archive](docs/archive/integration/wave1-wave2-plan.md)를 참고한다.
