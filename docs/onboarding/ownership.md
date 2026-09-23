# 팀 ownership과 검증 책임

> **문서 역할:** Active Reference — 현재 경로의 주 담당자와 리뷰 경계
>
> 실제 자동 리뷰 규칙은 [`.github/CODEOWNERS`](../../.github/CODEOWNERS)가 정본이다.

## 팀과 담당 영역

| GitHub | 이름 | 현재 주 담당 |
|---|---|---|
| `@ahnsh03` | 안승현 | 통합, HD Map, camera/BEV, bridge 공동, Docker·문서 |
| `@yuntae12-sudo` | 정윤태 | behavior, MPC, planning integration |
| `@yangseojun` | 양서준 | MPC·behavior 공동 |
| `@kante2` | 이강태 | Frenet planner, UDP bridge 공동 |
| `@wkddnjsxo` | 장원태 | scenario, gRPC, traffic-light 연계 |
| `@sonshiny` | 손재호 | LiDAR/3D perception, sensor fusion |

## 현재 코드 경계

| 영역 | 현재 경로 | 주 담당·필수 리뷰 |
|---|---|---|
| 공용 message | `src/asmc_msgs/` | 통합 주관, 소비 파트 공동 리뷰 |
| HD Map | `src/asmc_hd_map/` | 안승현; planning consumer 변경 시 판제 리뷰 |
| Behavior | `src/behavior_planner/` | 정윤태·양서준 |
| Frenet | `src/frenet_planner/` | 이강태·정윤태 |
| MPC | `src/mpc_controller/` | 정윤태·양서준 |
| Planning adapter | `src/planning_control/` | 통합·판제 공동 |
| UDP/sensor/state bridge와 통합 launch | `src/asmc_bridge/`; [개발 가이드](../packages/asmc_bridge/README.md) | 이강태·안승현, planning consumer 공동 리뷰 |
| Perception façade와 typed 계약 | `src/asmc_perception/`; [runtime 계약](../contracts/perception/runtime-v1.md) | 안승현 주관, 손재호·장원태·planning consumer 공동 리뷰 |
| Camera/BEV | `src/perception/camera_semantic_perception/`, `drivable_bev/`, `twinlite_morai/` | 안승현, perception 공동 리뷰 |
| LiDAR/3D | `src/perception/morai_3d_detection/`, `src/lidar+sensorfusion/` | 손재호, 통합 공동 리뷰 |
| 데이터 수집 | `src/data_collection/` | 안승현, perception 공동 리뷰 |
| Scenario/gRPC | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` | 장원태 |
| 공용 infra | `docker/`, `.github/`, `docs/`, `scripts/` 공용 진입점 | 통합 리뷰 필수 |

`src/planner/`는 현재 경로가 아니다. 판제 Frenet 정본은 `src/frenet_planner/`다.

## 통합 중인 경계

| 목표 | 현재 상태 |
|---|---|
| `asmc_hd_map` | 2단계 완료. 운영 map과 global/local topic 사용 |
| `asmc_bridge` | 3단계 예정. [현재 세 bridge/state 패키지와 이관 gate](../packages/asmc_bridge/README.md)를 검증 후 통합 |
| `asmc_perception` | P4-C1 GPU·시각화 진입점과 typed 신호등 병행 발행 완료. [현재 package-group 경계와 이관 gate](../packages/asmc_perception/README.md)를 검증하며 parity 전에는 perception/LiDAR/data 패키지를 삭제하지 않음 |
| `learning*` 정리 | 4단계 gate 전 삭제 금지. `src/learning`은 링크 이관 후 제거 후보 |
| 단일 실행·시각화 | 5단계 예정 |
| LAN2LAN 현장 리허설 | 6단계 예정 |

현재 경로와 목표 경로를 혼용하지 않는다. 이식 전까지 실행 명령은 각 package README와
[저장소 레이아웃](../architecture/repository-layout.md)을 따른다.

## 변경과 검증 책임

| 변경 종류 | 최소 검증 | 추가 리뷰 |
|---|---|---|
| 한 패키지 내부 알고리즘 | 해당 unit/build test | package owner |
| message/topic/좌표계 | producer·consumer build와 replay | 영향을 받는 전 파트 |
| HD Map geometry/metadata | map deterministic test, RViz/topic 검증 | planning consumer |
| UDP/센서 profile | packet/topic test, 기존 profile 회귀 | bridge·인지·판제 |
| perception 출력 | bag replay, rate/latency, planner adapter | perception·planning |
| Docker/CI/submodule | clean build | 통합 lead |

세부 파일 목록은 빠르게 낡으므로 이 문서에 복제하지 않는다. 실행법은 각 package README,
계약은 `docs/contracts/`, 진행 순서는 [통합 로드맵](../architecture/consolidation-roadmap.md)을
정본으로 사용한다.

## 공용 경계를 변경할 때

1. 관련 owner에게 변경 목적과 소비자를 먼저 공유한다.
2. 짧은 브랜치와 PR을 사용한다.
3. producer와 consumer를 같은 PR 또는 명시된 순서의 연속 PR로 검증한다.
4. 기존 실행 경로는 parity와 rollback이 확인될 때까지 제거하지 않는다.

브랜치·PR 명령은 [협업 규칙](../governance/README.md)을 따른다.
