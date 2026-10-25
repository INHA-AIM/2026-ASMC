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
| Behavior | `src/asmc_planner/` | 정윤태·양서준 |
| Frenet | `src/asmc_planner/` | 이강태·정윤태 |
| MPC | `src/asmc_controller/` | 정윤태·양서준 |
| 판제 입력 | `src/planning_inputs/` | 정윤태·판제 공동, 상태/좌표 변경 시 LiDAR·통합 리뷰 |
| Planning adapter | `src/jamming_lane_boundary_adapter/` | 통합·판제 공동; 판제 consumer 흡수 후 삭제 gate |
| UDP/sensor/state bridge와 통합 launch | `src/asmc_bridge/`; [개발 가이드](../../src/asmc_bridge/docs/integration.md) | 이강태·안승현, planning consumer 공동 리뷰 |
| Perception façade와 typed 계약 | `src/asmc_camera/`; [runtime 계약](../contracts/perception-runtime-v1.md) | 안승현 주관, 손재호·장원태·planning consumer 공동 리뷰 |
| Camera/BEV/traffic | `src/asmc_camera/` | 안승현, perception·planning consumer 공동 리뷰 |
| LiDAR/3D·camera fusion | 운영 원본 `src/lidar+sensorfusion/`, 공동 개발 `src/asmc_lidar/` | 안승현: HD맵 필터·카메라 연결 지원. 손재호: 추적·속도·운동 상태·matching/fusion, GPS 없는 터널 local 인식·벽/천장 보완. 공용 계약·판제 전환은 공동 리뷰 |
| 데이터 수집 | `src/data_collection/` | 안승현, perception 공동 리뷰 |
| Scenario/gRPC | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` | 장원태 |
| 공용 infra | `docker/`, `.github/`, `docs/`, `scripts/` 공용 진입점 | 통합 리뷰 필수 |

`src/planner/`는 현재 경로가 아니다. 판제 Frenet 정본은 `src/asmc_planner/`다.

## 통합 중인 경계

| 목표 | 현재 상태 |
|---|---|
| `asmc_hd_map` | 2단계 완료. 운영 map과 global/local topic 사용 |
| `asmc_bridge` | P3-G local closeout 완료. 실제 두 PC LAN2LAN은 6단계 현장 리허설에서 검증 |
| `asmc_camera`(구 `asmc_perception`) | P4-C6 camera/BEV/lane/traffic runtime·offline·training 통합과 기존 package closeout 완료. `data_collection`은 독립 package이며 Main은 카메라 외부 결과 소비로 연결됨. LiDAR 이관은 아래 L0–L8 담당을 따름 |
| `asmc_camera` 현재 | rename·3뷰 YOLO 단일화·TensorRT·family 최적화 병합. 원본16종/소비3그룹 계약 제공. 추가학습은 판제 요청 시 |
| 목표 `asmc_lidar` | PR #108까지 전달 필터 적용. 다음은 [전환 기반·구 패키지 폐기 B0–B4](../../src/asmc_lidar/docs/cutover-foundation.md); 리팩토링·3뷰 확대는 이후. 담당별 작업은 [공동 인수표](../../src/asmc_lidar/docs/migration-tasks.md#공동-개발-인수와-우선-작업), 운영은 기존 Main 유지 |
| `learning*` 정리 | 완료. LBC teacher/debug 필요 기능만 `tools/research/lbc_teacher_bev` 이관 |
| 실행·시각화 | P5-A~E 완료 기록 유지. bridge 별도 실행. LiDAR는 lidar.launch·RViz 선택 옵션, 웹은 구독 전용 view.launch |
| LAN2LAN 현장 리허설 | 6단계 예정 |

현재 경로와 목표 경로를 혼용하지 않는다. 이식 전까지 실행 명령은 각 package README와
[저장소 레이아웃](../architecture/repository-layout.md)을 따른다.
전체 개발 방향은 [통합 로드맵](../architecture/consolidation-roadmap.md)을 따르며, LiDAR의 상세 순서·완료 상태는 [통합 인계](../handoffs/lidar-migration.md)와 [작업표](../../src/asmc_lidar/docs/migration-tasks.md)가 정본이다. 이 문서의 역할 계획만으로 CODEOWNERS를 변경하거나
판제 소유 코드·손재호 브랜치를 대신 수정하지 않는다. 이전 PR에서 실제 담당자 리뷰 규칙을 확인한다.

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

브랜치·PR 명령은 [협업 규칙](collaboration.md)을 따른다.

## 다음 작업의 정렬 기준

담당별 다음 결과와 대회 완료 증거는 [통합 로드맵](../architecture/consolidation-roadmap.md)을 따릅니다.
카메라 모델 추가학습·3뷰 fusion·패키지 이전을 모두 끝내야만 현재 주행을 시험할 수 있는 것은 아닙니다.
먼저 현재 전체 미션의 실패를 기록·재현하고 해당 owner가 작은 변경으로 개선합니다.
PR #89의 좌표·고속 추종 변경은 현재 코드에 반영됐으며 남은 회귀 실패와 현장 수락은 별도입니다.
