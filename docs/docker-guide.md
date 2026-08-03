# Docker 가이드

## 1. 이미지 요약

| 컨테이너 | 이미지 | 베이스 | 마운트 |
|----------|--------|--------|--------|
| `asmc-ros-noetic` | `asmc/ros-noetic:dev` | `ros:noetic` (20.04) | `$ASMC` → `/root/ws` |
| `asmc-perception-train` | `asmc/perception-train:dev` | CUDA 11.8 + 20.04 | `$ASMC` → `/root/ws` |

맵·대용량 데이터 (읽기 전용): `$ASMC_DATA` → `/data`  
미설정 시 `../data` (repo 형제 폴더)를 사용합니다.

```bash
export ASMC_DATA="$HOME/asmc-data"
```

## 2. ROS 개발 컨테이너

### 최초 빌드

```bash
cd "$ASMC"
xhost +local:docker
./scripts/docker_ros_up.sh build
```

### 시작·접속

```bash
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash
./scripts/build_ws.sh
```

### 종료

```bash
./scripts/docker_ros_up.sh down
xhost -local:docker
```

### 재빌드가 필요한 경우

| 상황 | 명령 |
|------|------|
| Dockerfile / requirements 변경 | `./scripts/docker_ros_up.sh rebuild` |
| pip 패키지만 추가 | requirements 수정 후 `rebuild` |
| 코드만 변경 | **재빌드 불필요** — `build_ws.sh`만 |

## 3. Perception 학습 컨테이너

```bash
cd "$ASMC"
./scripts/docker_perception_up.sh build
./scripts/docker_perception_up.sh up
docker exec -it asmc-perception-train bash
cd /root/ws/src/perception   # 통합 후 경로
python3 train.py             # 예시
```

GPU 확인:

```bash
python3 -c "import torch; print(torch.cuda.is_available(), torch.__version__)"
```

## 4. 레거시 aim_ws Docker와의 차이

| 항목 | 레거시 `aim-noetic-pc` | `asmc-ros-noetic` |
|------|------------------------|-------------------|
| 마운트 | `aim_ws` 루트 | **`2026-ASMC` 루트** |
| WORKDIR | `/root/aim_ws` | `/root/ws` |
| 이미지명 | `aim-noetic:pc` | `asmc/ros-noetic:dev` |

레거시 컨테이너와 **동시 실행 금지** (ROS master 포트 11311 충돌).

## 5. 트러블슈팅

### ROS master 포트 충돌

```bash
docker exec -it asmc-ros-noetic bash
pkill -9 -f rosmaster; pkill -9 -f roscore
rm -rf /tmp/rosmaster* /tmp/ros_*
```

### X11 / RViz

```bash
xhost +local:docker
echo $DISPLAY   # :0 또는 :1 이어야 함
```

### WSL에서 GPU

Docker Desktop → WSL Integration 활성화, NVIDIA Container Toolkit 확인.

### MORAI SIM + UDP 브릿지 (26.R1.h3 개발)

> **개발 SIM = 26.R1.h3** (대회 SIM 아님).  
> 치트시트: [`config/morai_udp_sim_cheat_sheet.md`](../config/morai_udp_sim_cheat_sheet.md) · 상세 [`network-udp-ros.md`](./network-udp-ros.md)

| 항목 | 내용 |
|------|------|
| 실행 | `./scripts/udp_bridge.sh` (rosbridge 아님) |
| MORAI IP | Host/Dest 모두 **`127.0.0.1`** (compose UDP publish) |
| Ctrl | `cmd_ip=host.docker.internal` · `cmd_port` **9111** (팀 표준 Host) |
| rosbridge | `./scripts/rosbridge.sh` — 개발 편의 · **당일 금지** |

```bash
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash
source devel/setup.bash
./scripts/udp_bridge.sh
```

**주의:** `aim-noetic-pc`와 `asmc-ros-noetic` **동시 실행 금지** (11311 충돌).
