# Docker 가이드

> **문서 역할:** Active Guide — 컨테이너 build/up/down과 장애 대응
> **정본 연결:** UDP·포트·Docker 네트워크의 상세 기준은 [network-udp-ros.md](./network-udp-ros.md)
> **문서 허브:** [README.md](./README.md)

## 1. 이미지 요약

| 컨테이너 | 이미지 | 베이스 | 마운트 |
|----------|--------|--------|--------|
| `asmc-ros-noetic` | `asmc/ros-noetic:dev` | `ros:noetic` (20.04) | `$ASMC` → `/root/ws`, `$ASMC_DATA` → `/data` (`rw`) |
| `asmc-perception-train` | `asmc/perception-train:dev` | CUDA 11.8 + 20.04 | `$ASMC` → `/root/ws`, `$ASMC_DATA` → `/data` (`ro`), TwinLite external → `/opt/baselines/TwinLiteNetPlus` (`ro`) |

맵·대용량 데이터: `$ASMC_DATA` → `/data`. ROS 수집기는 쓰기 가능하고 학습 컨테이너는 읽기 전용입니다.
TwinLiteNet+ 공식 소스는 팀 레포로 복사하지 않고
`$AIM_PROJECT/external/baselines/TwinLiteNetPlus`를 학습 컨테이너에 읽기 전용으로
마운트합니다. 실제 loader가 고정 upstream commit을 검사합니다.
미설정 시 실행 스크립트가 `2026-ASMC`와 나란한 워크스페이스 `data/`를 사용합니다.

```bash
export ASMC_DATA="$(dirname "$ASMC")/data"
```

## 2. ROS 개발 컨테이너

### Docker Desktop 선행 설정 — 팀원별 최초 1회

1. Docker Desktop **4.34 이상**, Linux containers/WSL 2 모드를 사용합니다.
2. **Settings → Resources → Network → Enable host networking**을 켭니다.
3. **Apply and restart**로 Docker Desktop 재시작을 완료합니다.
4. **Settings → Resources → WSL Integration**에서 사용하는 배포판을 켭니다.

`Network` 메뉴가 없으면 Windows containers 모드인지 확인합니다. 설정 근거와 상세
트러블슈팅은 [network-udp-ros §3.1](./network-udp-ros.md#31-docker-desktop-최초-설정팀원별-1회)을
따릅니다.

### 최초 빌드

```bash
cd "$ASMC"
xhost +local:docker
./scripts/docker_ros_up.sh build
```

### 시작·접속

```bash
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh status  # mode=host, published={}
./scripts/docker_ros_up.sh check   # Windows↔컨테이너 host UDP 검사
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

이미지는 `XDG_RUNTIME_DIR=/tmp/asmc-xdg-runtime`, `QT_X11_NO_MITSHM=1`과
2GB shared memory를 기본 제공한다. 과거 듀얼모니터 창 좌표나 transport-specific topic이
저장돼 rqt/RViz가 화면 밖에서 열리는 문제를 피하려면 컨테이너 안에서 공통 helper를 쓴다.

```bash
asmc-ros-gui rqt-image
asmc-ros-gui rqt-image /perception/bev/debug/shadow_map_rgb
asmc-ros-gui rviz -d /root/ws/path/to/config.rviz
```

`rqt-image`에 `/compressed`를 붙여도 helper가 base topic으로 정규화한 뒤 compressed
transport를 선택한다. 실행마다 Qt 창 배치만 임시 디렉터리에서 시작하며 사용자가 지정한
`.rviz` 파일은 삭제하거나 수정하지 않는다. 이 변경은 이미지 재빌드 후 적용된다.

```bash
./scripts/docker_ros_up.sh build
# 실행 중인 브릿지·roslaunch를 종료한 뒤 컨테이너를 새 이미지로 재생성
./scripts/docker_ros_up.sh up
```

`up`은 현재 컨테이너를 재생성하므로 MORAI 주행·녹화 중에는 실행하지 않는다.

### WSL에서 GPU

Docker Desktop → WSL Integration 활성화, NVIDIA Container Toolkit 확인.

### MORAI SIM + UDP 브릿지 (26.R1.h3 개발)

> **개발 SIM = 26.R1.h3** (대회 SIM 아님).  
> 치트시트: [`config/morai_udp_sim_cheat_sheet.md`](../config/morai_udp_sim_cheat_sheet.md) · 상세 [`network-udp-ros.md`](./network-udp-ros.md)

| 항목 | 내용 |
|------|------|
| 실행 | `./scripts/udp_bridge.sh --simulator 26r1` (rosbridge 아님) |
| Docker 네트워크 | Docker Desktop **host networking** (`ports:` 포워딩 없음) |
| MORAI IP | Host/Dest 모두 **`127.0.0.1`** |
| Ctrl | `cmd_ip=127.0.0.1` · `cmd_port` **9111** (팀 표준 Host) |
| rosbridge | `./scripts/rosbridge.sh` — 개발 편의 · **당일 금지** |

```bash
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash
source devel/setup.bash
./scripts/udp_bridge.sh --simulator 26r1
```

다른 WSL 터미널에서 전체 센서 수신을 한 번에 검사할 수 있습니다.

```bash
./scripts/docker_ros_up.sh verify 8
```

**주의:** `aim-noetic-pc`와 `asmc-ros-noetic` **동시 실행 금지** (11311 충돌).
