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
| Planning adapter | `src/jamming_lane_boundary_adapter/` | 통합·판제 공동; 판제 consumer 흡수 후 삭제 gate |
| UDP/sensor/state bridge와 통합 launch | `src/asmc_bridge/`; [개발 가이드](../packages/asmc_bridge/README.md) | 이강태·안승현, planning consumer 공동 리뷰 |
| Perception façade와 typed 계약 | `src/asmc_perception/`; [runtime 계약](../contracts/perception/runtime-v1.md) | 안승현 주관, 손재호·장원태·planning consumer 공동 리뷰 |
| Camera/BEV/traffic | `src/asmc_perception/` | 안승현, perception·planning consumer 공동 리뷰 |
| LiDAR/3D·camera fusion | 원본 `src/lidar+sensorfusion/`, 이전용 골격 `src/asmc_lidar/` | 손재호가 기능 이전·수락 후 asmc_lidar 정본화, 안승현 계약·개선사항 인계 |
| 데이터 수집 | `src/data_collection/` | 안승현, perception 공동 리뷰 |
| Scenario/gRPC | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` | 장원태 |
| 공용 infra | `docker/`, `.github/`, `docs/`, `scripts/` 공용 진입점 | 통합 리뷰 필수 |

`src/planner/`는 현재 경로가 아니다. 판제 Frenet 정본은 `src/frenet_planner/`다.

## 통합 중인 경계

| 목표 | 현재 상태 |
|---|---|
| `asmc_hd_map` | 2단계 완료. 운영 map과 global/local topic 사용 |
| `asmc_bridge` | P3-G local closeout 완료. 실제 두 PC LAN2LAN은 6단계 현장 리허설에서 검증 |
| `asmc_perception` | P4-C6 camera/BEV/lane/traffic runtime·offline·training 통합과 기존 package closeout 완료. `data_collection`은 독립 package이며 팀원 개발 중인 `src/lidar+sensorfusion`은 handoff 전 변경 금지 |
| 목표 `asmc_camera` | 안승현: 현재 asmc_perception의 이름 변경부터 별도 검증. YOLO 생산자 이전·공용 검출 계약은 다음 변경. 기존 /perception 토픽 유지 |
| 목표 `asmc_lidar` | 손재호: 좌표·시간 골격 위에 원본 기능 보존·개선 적용·메시지/좌표 어댑터 이전 마무리. camera fusion 내부 유지, 개인 연구 runtime은 Git 이력에서만 조회 |
| `learning*` 정리 | 완료. LBC teacher/debug 필요 기능만 `tools/research/lbc_teacher_bev` 이관 |
| 실행·시각화 | P5-A~E 완료 기록 유지. bridge는 별도 실행, 웹 콘솔은 수요 발생 후 검토 |
| LAN2LAN 현장 리허설 | 6단계 예정 |

현재 경로와 목표 경로를 혼용하지 않는다. 이식 전까지 실행 명령은 각 package README와
[저장소 레이아웃](../architecture/repository-layout.md)을 따른다.
상세 순서·책임·삭제 조건은 [통합 로드맵](../architecture/consolidation-roadmap.md)의
카메라/LiDAR 이전 계획이 정본이다. 이 문서의 역할 계획만으로 CODEOWNERS를 변경하거나
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

브랜치·PR 명령은 [협업 규칙](../governance/README.md)을 따른다.
