# 개발 환경 규약

> **문서 역할:** Active Reference — 개발 OS·언어·데이터 경로 기준
> **정본 연결:** 대회 제약은 [competition-environment.md](../competition/environment.md), Docker 실행은 [docker-guide.md](../operations/docker.md)
> **문서 허브:** [README.md](../README.md)
> **기준:** 대회 본선 v1.1 (참가 PC **OS 자유**, 통신 **UDP**) · 팀 개발 **Ubuntu 20.04 + ROS1 Noetic**
> **최종 수정:** 2026-08-30

## 1. 대회 규정 vs 팀 개발 환경

| 구분 | 대회 당일 (v1.1) | 팀 일상 개발 |
|------|------------------|--------------|
| OS | **자유** (데스크톱·노트북) | WSL2 + Docker **또는** 네이티브 20.04 |
| ROS | 팀 선택 (UDP 출력 필수) | ROS1 **Noetic** (컨테이너) |
| 네트워크 | **UDP only** | UDP + ROS (`network_mode: host`) |
| Python | 팀 PC 기준 | **3.8 고정** (ML·ROS 공통) |

### WSL vs 네이티브

| 리스크 | WSL만 사용 시 | 네이티브 20.04 |
|--------|---------------|----------------|
| LAN/UDP 재현 | NAT·브리지 편차 | 규정과 동일 |
| 지연 측정 | 호스트-게스트 영향 | 신뢰 가능 |
| 현장 장비 | 휴대용 모니터·LAN선 제공, 키보드·마우스 지참 | 동일 조건 리허설 |
| 당일 트러블슈팅 | WSL+Windows 이중 이슈 | 단일 환경 |

**팀 정책**
- **WSL + Docker:** 문서·코드·catkin build
- **네이티브 20.04:** 8월 이전부터 주 1회 이상 통합 리허설 (판정 프로그램·UDP)

대회 PC 요건·당일 절차·센서 스펙: [competition-environment.md](../competition/environment.md)

## 2. 언어·버전 표준

### C++

| 항목 | 값 |
|------|-----|
| 표준 | ROS 모듈 **C++14**, Planning/Control **C++17** |
| 빌드 | ROS: catkin / Planning·Control: standalone CMake |
| 최적화 | `-O2` |

### Python

| 용도 | 버전 |
|------|------|
| ROS Noetic 시스템 | **3.8** |
| 컨테이너 pip | 3.8 |
| **팀 고정** | **3.8.x** — 3.9+ 문법·패키지 사용 금지 |

### PyTorch (인지 학습)

| 항목 | 값 |
|------|-----|
| torch | **2.1.2+cu118** |
| CUDA (컨테이너) | 11.8 |
| 호스트 드라이버 | CUDA 12.x 지원 드라이버 (하위 호환) |

### 주요 apt 패키지 (ROS 컨테이너)

- `libyaml-cpp-dev` — `behavior_planner` 빌드 (route YAML 파싱)

### 주요 pip 패키지 (ROS 컨테이너)

- LBC: `numpy`, `opencv-python-headless`, `scipy` — `docker/ros-noetic/requirements-lbc.txt`
- MGeo: `shapely`, `matplotlib` — `requirements-mgeo.txt`
- gRPC: `grpcio==1.44.0`, `protobuf==3.20.3`

## 3. Docker 이미지 2종

| 이미지 | 경로 | 용도 | 담당 파트 |
|--------|------|------|-----------|
| `asmc/ros-noetic:dev` | `docker/ros-noetic/` | catkin 모듈 + standalone UDP Planning/Control 빌드, 로깅 | planning, integration, learning |
| `asmc/perception-train:dev` | `docker/perception-train/` | PyTorch 학습·추론 | perception |

**왜 2개인가:** ROS desktop-full과 CUDA PyTorch를 한 이미지에 넣으면 용량·충돌·재빌드 비용이 큼.  
파트별로 분리하고 **코드는 동일 repo (`/root/ws`) 마운트**로 공유.

## 4. morai_msgs

- **브랜치:** `beta_drive` (대회 규정) — **유일한 빌드 기준**
- **위치:** `src/MORAI-ROS_morai_msgs` (submodule, 통합 시 등록)
- **금지:** 패키지마다 `morai_msgs` 복사본·영구 symlink
- **버전 매핑:** [morai-msgs-versions.md](../records/simulator/morai-msgs-versions.md)
- **통합·일상 작업 규약:** [morai-msgs-beta-policy.md](../contracts/simulator/morai-msgs-beta-policy.md) ← **필독**

현재 ASMC: `src/MORAI-ROS_morai_msgs` submodule @ **`beta_drive`** (`45c6baf`)

## 5. 데이터·맵

- Git **미포함:** `data/`, `checkpoints/`, `.bag`
- 맵 데이터는 로컬에 별도 보관 후 Docker에 마운트 (아래 참고)
- K-city 2025 vs KATRI 확장 레이어: MORAI 문의 **답변 대기** — [integration-plan.md](../archive/integration/wave1-wave2-plan.md) D-08

### 데이터 디렉터리

맵·대용량 파일은 **repo 밖**에 둡니다. 경로는 팀원마다 달라도 됩니다.

```bash
# ~/.bashrc 예시 (선택)
export ASMC_DATA="$(dirname "$ASMC")/data"   # R_KR_PG_K-city_2025, datasets 등
```

미설정 시 실행 스크립트는 `2026-ASMC` 레포와 나란한 워크스페이스 `data/`를 사용합니다.
다른 위치를 쓰면 반드시 `ASMC_DATA`를 export한 뒤 `./scripts/docker_ros_up.sh up` 하세요.

Docker 컨테이너 안에서는 **`/data`**로 보입니다. ROS 수집 컨테이너는 쓰기 가능(`rw`),
학습 컨테이너는 원본 보호를 위해 읽기 전용(`ro`)으로 마운트됩니다.

## 6. IDE·도구

| 도구 | 용도 |
|------|------|
| Cursor (WSL Remote) | 메인 편집 |
| `gh` | PR·이슈 |
| Sublime Merge | diff·머지 충돌 (선택) |
