# 시작 가이드

> **문서 역할:** Active Guide — 처음 clone하고 실행하는 최소 절차
> **정본 연결:** 환경 기준은 [development-environment.md](./development-environment.md), UDP 값은 [network-udp-ros.md](./network-udp-ros.md)
> **문서 허브:** [README.md](./README.md)

## 1. 사전 준비

| 항목 | 요구 |
|------|------|
| Git | 2.30+ |
| GitHub CLI | `gh` (PR 생성) |
| Docker Desktop | WSL 연동 활성화 (개발 PC) |
| 대회 리허설 PC | **Ubuntu 20.04 네이티브** 1대 권장 |

### WSL 개발 PC (Windows, 최초 1회)

대회 당일 PC는 **네이티브 Ubuntu 20.04**입니다. Windows PC에서는 **WSL2 + Docker**로 일상 개발합니다.

| 항목 | 내용 |
|------|------|
| WSL | Ubuntu 20.04 권장 (`wsl --install -d Ubuntu`) |
| Docker | Docker Desktop → Settings → **WSL Integration** → Ubuntu **ON** |
| clone 위치 | **WSL 홈** (`~/...`) 어디든 가능. `/mnt/c/...` **금지** — [getting-started.md §2](./getting-started.md) |
| 맵 데이터 | repo 밖 `$ASMC_DATA`. 탐색기 `\\wsl.localhost\<배포판>\home\<user>\asmc-data` 로 복사 가능 |
| MORAI SIM | **Windows에서 실행**. 최초 실행 시 방화벽 **개인·공용 네트워크 허용** |
| IDE | WSL Remote 지원 에디터 또는 `docker exec -it asmc-ros-noetic bash` |

```bash
# Git 인증 (Windows + WSL, 선택) — 본인 환경의 credential helper 사용
git clone --recurse-submodules https://github.com/INHAautonav/2026-ASMC.git
```

셸 변수 (`~/.bashrc`, **선택** — 편하면 설정):

```bash
export ASMC="<clone한-2026-ASMC-경로>"
export ASMC_DATA="<맵·대용량-데이터-경로>"
```

`ASMC`를 설정하지 않아도, **repo 루트에서** `./scripts/...` 를 실행하면 스크립트가 경로를 자동으로 찾습니다.  
팀원마다 clone 위치가 달라도 됩니다. 통일할 필요는 없습니다.

## 2. 클론

### 주의사항 (필수)

| 규칙 | 이유 |
|------|------|
| **WSL 리눅스 파일시스템**에 clone (`~/...`) | `/mnt/c/Users/...` 는 **금지** — Git·맵·NPZ 파일명 충돌, 느린 I/O, 권한 오류 |
| **호스트 절대 경로를 코드에 넣지 않음** | 컨테이너 안은 항상 `/root/ws`. 맵은 `$ASMC_DATA` → `/data` |
| **submodule 포함 clone** | `morai_msgs` 누락 방지 |

### 명령

`ASMC` 변수를 쓰는 경우:

```bash
mkdir -p "$(dirname "$ASMC")"
git clone --recurse-submodules https://github.com/INHAautonav/2026-ASMC.git "$ASMC"
cd "$ASMC"
git checkout main && git pull
```

경로를 정하지 않고 clone하는 경우:

```bash
git clone --recurse-submodules https://github.com/INHAautonav/2026-ASMC.git <원하는-경로>
cd <원하는-경로>
git checkout main && git pull
```

이미 clone한 뒤 submodule만 받을 때:

```bash
cd <clone한-2026-ASMC-경로>
git submodule update --init --recursive
```

**하지 말 것:** 윈도우 탐색기로 repo 안의 `build/`, `devel/` 을 수정·삭제 (WSL 권한 꼬임). 데이터 파일은 repo 밖 `$ASMC_DATA`에 둡니다.

## 3. morai_msgs 추가

```bash
cd "$ASMC"
git submodule add -b beta_drive \
  https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git \
  src/MORAI-ROS_morai_msgs
git submodule update --init --recursive
```

이미 submodule이 등록된 경우:

```bash
git submodule update --init --recursive
```

## 4. Docker로 ROS 환경 실행

### 4.1 Windows Docker Desktop 최초 설정

이 저장소의 ROS 컨테이너는 Docker Desktop host networking을 기본으로 사용합니다.
팀원마다 최초 한 번 다음을 적용합니다.

1. Docker Desktop **4.34 이상**을 설치하고 Linux containers/WSL 2 모드로 실행합니다.
2. **Settings → Resources → Network**로 이동합니다.
3. **Enable host networking**을 체크합니다.
4. **Apply and restart**를 눌러 재시작이 끝날 때까지 기다립니다.
5. **Settings → Resources → WSL Integration**에서 레포를 실행할 WSL 배포판을 켭니다.

`Network` 메뉴가 없으면 Windows containers 모드인지 확인하고 Linux containers로
전환합니다. `Enable host networking` 항목이 없으면 Docker Desktop 버전을 업데이트합니다.
공식 기준과 제약은 [Docker host network driver 문서](https://docs.docker.com/engine/network/drivers/host/)를
참고합니다.

### 4.2 빌드·실행·검사

```bash
cd "$ASMC"
xhost +local:docker
./scripts/docker_ros_up.sh build
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh status
./scripts/docker_ros_up.sh check
docker exec -it asmc-ros-noetic bash
# 컨테이너 안
./scripts/build_ws.sh
```

정상 상태에서는 `status`에 `mode=host`, `published={}`가 나오고 `check`의 두 방향 UDP가
모두 `[OK]`여야 합니다. MORAI의 Host/Destination IP `127.0.0.1`과 센서 포트는 바꾸지
않습니다.

### MORAI UDP (26.R1.h3 개발)

시뮬 UI IP는 **Host/Dest 모두 `127.0.0.1`**. 포트 치트시트: [`config/morai_udp_sim_cheat_sheet.md`](../config/morai_udp_sim_cheat_sheet.md).  
상세: [network-udp-ros.md](./network-udp-ros.md).

```bash
source devel/setup.bash
./scripts/udp_bridge.sh --simulator 26r1
```

> 대회 배포 SIM이 아니라 **개발용 26.R1.h3** 기준이다.
> 대회판에서는 `./scripts/udp_bridge.sh --simulator competition`을 사용한다.
## 5. 일상 작업 (Git)

```bash
git checkout main && git pull --rebase origin main
# Path A: 소유 모듈 + 빌드 통과 → push origin main
# WIP 공유: feature/… + Draft PR (collaboration §2.5)
# Phase 0 기능 개발: 개인 레포도 OK — team-status.md
```

- **필독:** [collaboration.md](./collaboration.md) · [network-udp-ros.md](./network-udp-ros.md)
- [team-dev-guide.md](./team-dev-guide.md) · [sim-verification-checklist.md](./sim-verification-checklist.md)

`behavior_planner` 빌드에는 Docker 이미지의 `libyaml-cpp-dev`가 필요합니다 (`./scripts/docker_ros_up.sh build`로 이미지 재생성).
