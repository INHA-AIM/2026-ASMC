# 개발 시작과 환경 기준

> **문서 역할:** Active Guide — clone부터 첫 빌드까지 필요한 최소 절차와 개발환경 정본
>
> **기준:** Ubuntu 20.04 · ROS1 Noetic · Python 3.8 · 2026-09-22

세부 Docker·MORAI IP·포트·시뮬 검증은 [운영 가이드](../operations/README.md), Git 작업은
[협업 규칙](collaboration.md)을 따른다.

## 1. 준비

| 항목 | 개발 환경 | 현장 검증 |
|---|---|---|
| OS | WSL2 또는 native Ubuntu 20.04 | native Linux Docker 권장 |
| ROS | `asmc/ros-noetic:dev` 컨테이너 | 동일 image/profile |
| 네트워크 | Docker host networking, MORAI와 loopback | 실제 두 PC LAN2LAN은 6단계에서 검증 |
| 데이터 | 저장소 밖 `$ASMC_DATA` | 동일 artifact hash 사용 |

Windows 개발 PC는 Docker Desktop 4.34 이상에서 Linux containers, WSL Integration과
`Settings → Resources → Network → Enable host networking`을 켠다. 저장소는 WSL의
Linux 파일시스템(`~/...`)에 clone하며 `/mnt/c/...`에는 두지 않는다.

필수 도구는 Git, Docker Compose와 GitHub CLI `gh`다. GUI/RViz를 쓸 때만 호스트에서
`xhost +local:docker`를 실행한다.

## 2. Clone과 데이터 경로

```bash
git clone --recurse-submodules https://github.com/INHAautonav/2026-ASMC.git
cd 2026-ASMC
git submodule update --init --recursive
```

이미 등록된 `src/MORAI-ROS_morai_msgs`를 다시 `git submodule add`하지 않는다. 현재 빌드
기준은 `beta_drive`의 pinned commit `45c6baf`이며, 핀 변경은 별도 PR과 호환 검증이 필요하다.

환경변수는 선택 사항이다. 설정하지 않으면 스크립트가 저장소와 나란한 `data/`를 사용한다.

```bash
export ASMC="<clone한-2026-ASMC-경로>"
export ASMC_DATA="<맵·bag·dataset을 둘-저장소-밖-경로>"
```

- Git에 넣지 않음: `data/`, `checkpoints/`, bag, dataset, model checkpoint
- ROS 컨테이너: `$ASMC_DATA`를 `/data`에 읽기/쓰기로 마운트
- 학습 컨테이너: 원본 보호를 위해 `/data`를 읽기 전용으로 마운트
- 코드와 설정에 `/home/<사용자>/...` 같은 호스트 절대경로를 넣지 않음

## 3. ROS 개발 컨테이너와 빌드

```bash
cd "${ASMC:-<clone한-2026-ASMC-경로>}"
./scripts/docker_ros_up.sh build
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh status
./scripts/docker_ros_up.sh check
docker exec -it asmc-ros-noetic bash
```

컨테이너 안에서:

```bash
cd /root/ws
./scripts/build_ws.sh
source devel/setup.bash
```

Behavior/Frenet/MPC도 현재 catkin package이므로 `build_ws.sh`에서 함께 빌드한다.
이전 standalone planning/control build script는 P4-C6에서 제거했다. 정상적인 Docker
상태는 network mode가 `host`이고 published port가 없는 상태다.

개발 MORAI 26.R1 프로필은 다음처럼 실행한다.

```bash
./scripts/udp_bridge.sh
```

현재 개발 설정의 Host/Destination IP는 `127.0.0.1`이다. 현장 IP·9093/9094 등의 승인된
포트와 LiDAR 15 Hz 적용은 `asmc_bridge` 통합 단계에서 profile로 구현하며, 그 전에는
개별 설정 파일을 임의로 바꾸지 않는다.

## 4. 인지 학습 컨테이너

실시간 인지 추론은 앞의 `asmc-ros-noetic`에서 실행한다. 아래
컨테이너는 모델 학습과 학습 중 validation이 필요할 때만 기동한다. bag replay를 포함한
ROS 추론·시각화는 여기서 실행하지 않는다.

```bash
./scripts/docker_perception_up.sh build
./scripts/docker_perception_up.sh up
```

| 환경 | 기준 |
|---|---|
| ROS/runtime 기본 image | Python 3.8, PyTorch 2.1.2, CUDA 11.8 |
| perception training image | Ubuntu 20.04, PyTorch 2.1.2+cu118 |
| RTX 50 runtime overlay | PyTorch 2.7.1 / torchvision 0.22.1 |

CUDA 드라이버는 호스트에만 설치한다. 컨테이너 안에서 NVIDIA driver를 설치하지 않는다.

## 5. 언어와 의존성 기준

| 영역 | 기준 |
|---|---|
| ROS Python | Python 3.8 호환 문법과 package |
| Planning/Control과 LiDAR Main | C++17 |
| Catkin/upstream package | 각 패키지 CMake 기준 유지; Velodyne vendor는 C++11 포함 |
| `morai_msgs` | pinned `src/MORAI-ROS_morai_msgs`, 복사본 생성 금지 |

ROS와 CUDA 의존성을 한 image에 무리하게 합치지 않는다. 학습·dataset 분석은 학습 image,
ROS topic·bridge·runtime 검증은 ROS image를 사용한다.

## 6. 첫 검증과 다음 문서

```bash
git status --short
./scripts/docker_ros_up.sh verify 8 molitcomp03  # 대회판
# ./scripts/docker_ros_up.sh verify 8 26r1      # 26.R1 개발판
```

| 작업 | 다음 문서 |
|---|---|
| 담당 경로 확인 | [팀 ownership](./ownership.md) |
| 브랜치·PR | [협업 규칙](collaboration.md) |
| Docker·UDP·GPU·시뮬 검증 | [운영 가이드](../operations/README.md) |
| 저장소 통합 단계 | [통합 로드맵](../architecture/consolidation-roadmap.md) |
