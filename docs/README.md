# 2026-ASMC 문서 허브

> **문서 상태:** Active Index
> **최종 정리:** 2026-09-22
> **대상:** 팀원 전원 · 관리: 통합 리드와 각 문서 담당자

문서는 독자와 역할에 따라 `onboarding/`, `contracts/`, `packages/`, `operations/`,
`records/`, `archive/` 등으로 나눈다. 주요 이전 경로에는 한 릴리스 동안 새 정본으로 연결하는
호환 안내만 남긴다. 이동 결과는 [D1 경로 이전 보고서](./structure/d1-path-migration-report.md),
본문 통합은 [D2 온보딩·협업 보고서](./structure/d2-onboarding-governance-report.md)와
[D2 대회·운영 보고서](./structure/d2-competition-operations-report.md),
[본문 축약 보존 감사](./structure/d2-content-parity-audit.md),
[D2 계약 보고서](./structure/d2-contracts-report.md),
[D2 HD Map·message 보고서](./structure/d2-hd-map-msgs-report.md),
[D2 bridge 보고서](./structure/d2-bridge-report.md),
[D2 perception 보고서](./structure/d2-perception-report.md),
[D2 Planning/Control·handoff 보고서](./structure/d2-planning-handoffs-report.md)를 참고한다.
이 페이지에서 정본, 작업 중 설계안, 사용 가이드와 과거 기록을 구분한다. 내용이
충돌하면 아래의 `정본 우선순위`를 적용하고, 기록 문서를 현재 설정으로 해석하지 않는다.

## 1. 처음 보는 팀원이 읽을 문서

아래 문서를 순서대로 읽고 필요한 파트의 상세 문서로 이동한다.

1. [개발 시작](./onboarding/README.md) — clone, Docker, 환경과 첫 빌드
2. [팀 ownership](./onboarding/ownership.md) — 담당 경로와 검증 책임
3. [협업 규칙](./governance/README.md) — 브랜치, PR과 리뷰 기준
4. [대회 환경·규정](./competition/README.md) — 허용 입력과 당일 제약
5. [팀 아키텍처](./architecture/system-overview.md) — 현재 시스템 경계

현재 인지 개발자는 [Perception 개발·통합 가이드](./packages/asmc_perception/README.md),
planning/control 개발자는 [Planning/Control 개발 가이드](./packages/planning-control/README.md)를 이어서
읽는다.

## 2. 정본 우선순위

| 주제 | 현재 정본 | 충돌 시 처리 |
|---|---|---|
| 대회 규정·공개 자산 | 주최 측 최신 원문 → [대회 요약](./competition/README.md) | 공식 원문이 항상 우선. 요약 문서를 함께 갱신 |
| Docker·시뮬 통신·포트 | [운영 가이드](./operations/README.md) | 현재 적용값과 3단계 목표를 혼용하지 않음 |
| bridge 구현·통합 gate | [MORAI bridge guide](./packages/asmc_bridge/README.md) | 현재 세 패키지와 아직 없는 `asmc_bridge`를 구분 |
| perception 구현·통합 gate | [Perception package-group guide](./packages/asmc_perception/README.md) | 현재 분산 패키지와 아직 없는 `asmc_perception`을 구분 |
| Git·ownership | [협업 규칙](./governance/README.md)과 [ownership](./onboarding/ownership.md) | 과거 Phase 0/Path A 문서보다 우선 |
| 공용 계약 탐색·변경 gate | [계약 문서 허브](./contracts/README.md) | 실제 구현은 링크된 개별 계약과 기계 정본을 함께 확인 |
| Planning/Control 실행 | [Planning/Control 운영 규약](./operations/planning-control.md) | standalone UDP와 v1 handoff를 현재 경로로 사용하지 않음 |
| 카메라 데이터·라벨·split | [perception-dataset-labeling-split.md](./contracts/perception/dataset-labeling-split.md) | 개별 학습 기록보다 우선 |
| 인지 융합 방향·우선순위 | [perception-map-fusion-roadmap.md](./packages/asmc_perception/roadmap.md) | BEV 세부안·초기 agentic 문서보다 우선 |
| 공용 HD Map·대회 metadata 계약 | [HD Map topic contract v1](./contracts/hd-map/topic-v1.md) | P1-B에서 동결한 `asmc_msgs` 1.0.0 및 `/hd_map/*` 정본 |
| 인지·판제 실험 interface | [src/interfaces/README.md](../src/interfaces/README.md) | 현재는 초안. pilot 뒤 합의된 항목만 정본으로 승격 |

`Working Proposal`은 구현을 시작하기 위한 후보안이지 다른 파트가 반드시 따라야 하는
확정 계약이 아니다.

## 3. 주제별 문서 지도

### 3.1 온보딩·개발 환경

| 문서 | 역할 | 상태 |
|---|---|---|
| [개발 시작과 환경](./onboarding/README.md) | clone, build, OS·언어·데이터 경로 기준 | Active Guide |
| [repository-layout.md](./architecture/repository-layout.md) | 코드 위치와 경로별 ownership 요약 | Active Reference |
| [팀 ownership](./onboarding/ownership.md) | 담당 경로와 검증 책임 | Active Reference |

### 3.2 협업·통합

| 문서 | 역할 | 상태 |
|---|---|---|
| [협업과 Git 규칙](./governance/README.md) | 브랜치, PR, review와 main 안정성 | Canonical |
| [integration-plan.md](./archive/integration/wave1-wave2-plan.md) | Wave 1/2 이식 과정과 당시 판단 | Historical Record |
| [architecture-overview.md](./architecture/system-overview.md) | 전체 모듈과 현재 목표 구조 요약 | Working Overview |
| [repository-consolidation-roadmap.md](./architecture/consolidation-roadmap.md) | msg·bridge·HD Map·perception 4-패키지 통합, 체크포인트/미션, 현장 배포 단계 | Working Proposal |
| [baselines/runtime-baseline-v1.md](./records/baselines/runtime-v1.md) | P1-A package/topic/message/sensor/bag/HD Map 회귀 기준과 후속 요구 | Frozen Baseline |
| [contracts/hd-map-topic-contract-v1.md](./contracts/hd-map/topic-v1.md) | P1-B `asmc_msgs` HD Map·체크포인트·미션 message/topic 계약 | Frozen Interface |
| [contracts/kcity2025-competition-metadata-v1.md](./contracts/competition/kcity2025-metadata-v1.md) | P1-C 공식 체크포인트 projection과 권위별 mission-zone 정본 | Frozen Metadata |
| [contracts/hd-map-runtime-visualization-v1.md](./contracts/hd-map/runtime-v1.md) | 2단계 global/local ROS publisher, RViz와 paint-gap 소비 규칙 | Active Runtime |
| [baselines/kcity2025-drivable-area-input-audit-v1.md](./records/baselines/kcity2025-drivable-input-v1.md) | P2-A Node/Link road prior, lane-clip 후보와 intersection guard 정본 | Frozen P2 Input |
| [baselines/kcity2025-compact-drivable-map-v1.md](./records/baselines/kcity2025-compact-drivable-v1.md) | P2-B route-compact Lane/vector/grid 결과와 판제 소비 규칙 | Active HD Map Product |

공용 계약을 구현하거나 바꾸려면 먼저 [계약 문서 허브](./contracts/README.md)에서 consumer,
기계 정본과 검증 gate를 확인한다.

### 3.3 대회·시뮬·네트워크

| 문서 | 역할 | 상태 |
|---|---|---|
| [대회 환경·규정·공개 자산](./competition/README.md) | 규정 v1.1, 센서, 미션과 배포 자산 | Canonical Summary |
| [개발·시뮬레이터 운영](./operations/README.md) | Docker, 현재 UDP profile, 진단과 시뮬 gate | Canonical |
| [rosbag 기록·평가](./operations/rosbag-evaluation.md) | raw/result 기록, replay와 lineage | Active Guide |
| [morai-msgs-beta-policy.md](./contracts/simulator/morai-msgs-beta-policy.md) | `beta_drive` submodule과 필드 정렬 규칙 | Canonical until port |
| [morai-msgs-versions.md](./records/simulator/morai-msgs-versions.md) | 26.R1과 대회 msg 차이 조사 기록 | Version Reference |
| [2026-09 transport 검증](./records/simulator/runtime-transport-2026-09.md) | host network, rate/latency와 종료된 GT 실험 | Experiment Record |
| [MORAI bridge 개발·통합](./packages/asmc_bridge/README.md) | 현재 parser/state 경계와 Phase 3 이관 gate | Active Package-Group Guide |

MORAI 화면에 입력할 값만 필요하면
[UDP 치트시트](../config/morai_udp_sim_cheat_sheet.md)를 사용한다. 상세 설명과 장애 대응은
치트시트가 아니라 운영 가이드에서 관리한다.

### 3.4 Planning/Control

| 문서 | 역할 | 상태 |
|---|---|---|
| [Planning/Control 개발 가이드](./packages/planning-control/README.md) | 현재 Behavior/Frenet/MPC 경계와 변경 gate | Active Package-Group Guide |
| [Planning/Control 운영 규약](./operations/planning-control.md) | 현재 ROS build·launch·topic·fail-safe 확인 | Canonical Operations |
| [planning-control-refactoring-guidelines.md](./packages/planning-control/refactoring.md) | Frenet/MPC 아키텍처 경계, ownership, dependency 방향, refactoring 체크리스트 | Active Reference |
| [planning-behavior-handoff.md](./handoffs/planning/behavior-legacy.md) | 이전 ROS 3-package 통합 설계와 검증 기록 | Legacy Reference |
| [Handoff index](./handoffs/README.md) | 현재 인지→판제·인지→LiDAR 인수 조건과 legacy 구분 | Active Index |

현재 Behavior/Frenet v2는 연결돼 있다. legacy handoff 본문의 옛 경로·message·launch는
현재 실행에 적용하지 않는다.

### 3.5 Perception·데이터·BEV

| 문서 | 역할 | 상태 |
|---|---|---|
| [Perception 개발·통합](./packages/asmc_perception/README.md) | 현재 코드·문서 경계, runtime interface와 Phase 4 gate | Active Package-Group Guide |
| [perception-map-fusion-roadmap.md](./packages/asmc_perception/roadmap.md) | camera/MGeo/LiDAR 전체 전략과 시간순 개발 근거 | Working Proposal + Chronological Record |
| [shadow-camera-perception-handoff.md](./handoffs/perception/shadow-camera.md) | 음영구간 v3 lane-family·drivable 판제 소비 계약과 성능 동결 | Active Handoff Guide |
| [camera-object-detection-handoff.md](./handoffs/perception/camera-object-detection.md) | 손재호용 YOLO 즉시 사용·경량 후보·NAS fine-tune 인수 가이드 | Active Guide |
| [rosbag 기록·재생·평가](./operations/rosbag-evaluation.md) | Real Time UDP raw/result bag과 lineage | Active Guide |
| [perception-dataset-labeling-split.md](./contracts/perception/dataset-labeling-split.md) | Capture 원본, mask, ignore, manifest와 split 계약 | Canonical Data Contract |
| [perception-bev-lane-interface-plan.md](./packages/asmc_perception/bev-lane-interface-plan.md) | GT 정책, metric lane 후보, BEV/lane topology 상세 | Detailed Design Note |
| [perception-v4-training-evaluation.md](./records/perception/twinlitenet-v4-evaluation.md) | TwinLiteNet+ v4 10/18 epoch 결과 | Experiment Record |
| [perception-agentic-loop.md](./archive/perception/agentic-loop.md) | 초기 Loc-Aid/bake/BEV 평가 루프 아이디어 | Research Reference |

인지 문서의 역할은 다음처럼 분리한다.

```text
roadmap       무엇을 어떤 순서로 개발할지
package guide 현재 무엇이 구현됐고 어느 문서를 우선할지
dataset       데이터와 정답을 어떻게 재현할지
BEV note      정책 선택의 근거와 후보 인터페이스
evaluation    특정 데이터·체크포인트의 실험 사실
agentic loop  반복 평가 도구에 대한 초기 연구 아이디어
```

학습 결과가 바뀌면 모든 문서에 수치를 복사하지 않는다. 실험 수치는 evaluation 문서에
추가하고 roadmap에는 현재 의사결정에 필요한 요약만 반영한다.

## 4. 작업별 바로가기

| 하려는 작업 | 먼저 볼 문서 | 다음 문서 |
|---|---|---|
| 처음 실행 | [개발 시작](./onboarding/README.md) | [운영 가이드](./operations/README.md) |
| UDP 센서가 안 들어옴 | [운영 가이드](./operations/README.md) | [UDP 치트시트](../config/morai_udp_sim_cheat_sheet.md) |
| bridge parser·timestamp·vehicle state 수정 | [MORAI bridge guide](./packages/asmc_bridge/README.md) | [운영 gate](./operations/README.md#7-변경-범위별-시뮬-검증) |
| Git 작업·PR | [협업 규칙](./governance/README.md) | [ownership](./onboarding/ownership.md), [CONTRIBUTING](../CONTRIBUTING.md) |
| 본선 허용 여부 판단 | [대회 환경](./competition/README.md) | [운영 가이드](./operations/README.md) |
| Behavior/Frenet/MPC 실행 | [Planning/Control 운영 규약](./operations/planning-control.md) | [package-group guide](./packages/planning-control/README.md) |
| Capture/마스크/학습 데이터 | [dataset contract](./contracts/perception/dataset-labeling-split.md) | [data_collection README](../src/data_collection/README.md) |
| raw bag 녹화·재생·모델 A/B | [rosbag 재현 평가 guide](./operations/rosbag-evaluation.md) | [data_collection README](../src/data_collection/README.md) |
| 카메라 live 추론 | [Perception package-group guide](./packages/asmc_perception/README.md#8-실행검증-진입점) | [camera README](../src/perception/camera_semantic_perception/README.md) |
| 카메라–LiDAR 객체 분류 | [object detector handoff](./handoffs/perception/camera-object-detection.md) | [sensor-fusion 실행](../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md), [rosbag 평가](./operations/rosbag-evaluation.md) |
| HD Map·카메라·LiDAR 문맥 | [Perception package-group guide](./packages/asmc_perception/README.md) | [fusion roadmap](./packages/asmc_perception/roadmap.md), [BEV design note](./packages/asmc_perception/bev-lane-interface-plan.md) |
| 음영구간 카메라 출력을 판제에 연결 | [shadow camera handoff](./handoffs/perception/shadow-camera.md) | [lane boundary adapter](../src/planning_control/lane_boundary_adapter/README.md) |
| 통합 전 package/topic/HD Map 회귀 확인 | [P1-A runtime baseline](./records/baselines/runtime-v1.md) | [통합 로드맵](./architecture/consolidation-roadmap.md) |
| HD Map·체크포인트 토픽을 구현/소비 | [P1-B topic contract](./contracts/hd-map/topic-v1.md) | [asmc_msgs README](../src/asmc_msgs/README.md), [통합 로드맵](./architecture/consolidation-roadmap.md) |
| 주행가능영역 clip·보호구간을 확인 | [P2-A input audit](./records/baselines/kcity2025-drivable-input-v1.md) | [runtime/visualization contract](./contracts/hd-map/runtime-v1.md), [통합 로드맵](./architecture/consolidation-roadmap.md) |
| compact Lane·주행가능영역을 실행/소비 | [P2-B compact map](./records/baselines/kcity2025-compact-drivable-v1.md) | [runtime/visualization contract](./contracts/hd-map/runtime-v1.md), [HD Map README](../src/asmc_hd_map/README.md) |
| 체크포인트·미션 경계를 확인 | [P1-C competition metadata](./contracts/competition/kcity2025-metadata-v1.md) | [생성 bundle](../src/asmc_hd_map/config/kcity2025_competition_metadata.yaml) |
| HD Map 토픽·RViz를 실행 | [2단계 runtime/visualization](./contracts/hd-map/runtime-v1.md) | [P1-B topic contract](./contracts/hd-map/topic-v1.md) |
| HD Map builder·산출물을 변경·검증 | [asmc_hd_map package guide](./packages/asmc_hd_map/README.md) | [P2-B compact map](./records/baselines/kcity2025-compact-drivable-v1.md) |
| 공용 ROS message를 추가·검증 | [asmc_msgs package guide](./packages/asmc_msgs/README.md) | [계약 허브](./contracts/README.md) |
| 대회판 이식 | [fusion roadmap §9](./packages/asmc_perception/roadmap.md#9-26r1h3-개발판과-대회판-이식-전략) | [msg versions](./records/simulator/morai-msgs-versions.md), [운영 가이드](./operations/README.md) |

## 5. 모듈 README 진입점

| README | 역할 |
|---|---|
| [asmc_hd_map guide](./packages/asmc_hd_map/README.md) | HD Map builder/runtime/source comparison 개발·검증 |
| [asmc_msgs guide](./packages/asmc_msgs/README.md) | 공용 ROS1 message 소유·호환·추가 절차 |
| [MORAI bridge guide](./packages/asmc_bridge/README.md) | 현재 bridge/state 구현과 Phase 3 통합 gate |
| [Perception package-group guide](./packages/asmc_perception/README.md) | 현재 camera/BEV/LiDAR/traffic/data 경계와 Phase 4 gate |
| [Planning/Control package-group guide](./packages/planning-control/README.md) | 현재 Behavior/Frenet/MPC 경계, legacy와 변경 gate |
| [Handoff index](./handoffs/README.md) | 파트 간 인수 조건과 deployment/legacy 상태 |
| [morai_udp_bridge](../src/morai_udp_bridge/README.md) | Network UDP·Ctrl 빠른 시작 |
| [morai_sensor_bridge](../src/morai_sensor_bridge/README.md) | Sensor UDP decode 빠른 시작 |
| [vehicle_state](../src/vehicle_state/README.md) | GPS/IMU 상태 정규화와 배타적 source |
| [src/perception](../src/perception/README.md) | 인지 파트 상태·코드·문서 지도 |
| [twinlite_morai](../src/perception/twinlite_morai/README.md) | binary/4-class 2-head 학습 adapter와 현재 후보 |
| [drivable_bev](../src/perception/drivable_bev/README.md) | 3-camera fused metric BEV, 범위 audit와 calibration/live gate |
| [data_collection](../src/data_collection/README.md) | Capture, sync, mask/cache와 dataset 실행법 |
| [frenet_planner](../src/frenet_planner/README.md) | Frenet Planner 책임, runtime I/O, core 폴더 지도, 빌드/테스트 |
| [interfaces](../src/interfaces/README.md) | provisional topic과 합의된 계약의 승격 위치 |
| [lane_boundary_adapter](../src/planning_control/lane_boundary_adapter/README.md) | 카메라 metric 경계를 판제와 검토하는 read-only C++ consumer pilot |
| [integration](../src/integration/README.md) | UDP bridge와 내부 ROS 진입점 |
| [learning](../src/learning/README.md) | 기존 LBC 연구 경로와 현재 인지 경로 구분 |

모듈 README에는 실행법과 현재 상태를 두고, 배경 설명·팀 규약·실험 이력은 `docs/`의 해당
문서로 연결한다.

## 6. 문서 상태 정의

| 상태 | 의미 | 변경 방법 |
|---|---|---|
| `Canonical` | 팀이 현재 따라야 하는 규약 | 관련 파트 리뷰 후 변경 |
| `Active Guide/Reference` | 현재 설정을 설명하는 사용 문서 | 구현 변경과 같은 PR에서 갱신 |
| `Working Proposal` | 실증 전 후보 설계 | 열린 질문과 근거를 남기며 수정 |
| `Experiment Record` | 특정 시점의 데이터·설정·결과 | 기존 사실을 덮어쓰지 않고 결과 추가 |
| `Historical/Legacy` | 과거 이식·설계 기록 | 삭제하지 않되 상단에 현재 정본 링크 유지 |

## 7. 유지관리 규칙

1. 포트, 허용 통신, Git 정책과 데이터 계약은 각각의 정본 한 곳에서만 정의한다.
2. 다른 문서에서는 긴 내용을 복사하지 않고 정본의 절을 링크한다.
3. 구현·설정 변경 PR에는 같은 PR의 관련 Active/Canonical 문서 수정이 포함되어야 한다.
4. 제안 문서는 `확정`, `후보`, `보류`를 표나 문장으로 구분하고 열린 질문을 유지한다.
5. 실험 문서는 dataset version, split, checkpoint hash/경로, 날짜와 한계를 기록한다.
6. 오래된 문서는 삭제하기 전에 Legacy banner와 대체 정본 링크를 먼저 추가한다.
7. 파일 이동·이름 변경은 전체 상대 링크와 외부 북마크 영향을 확인한 별도 PR에서만 한다.
8. 문서의 날짜는 실제로 내용을 재검토했을 때만 변경한다.

### 새 내용을 어디에 적을지

```text
팀이 따라야 할 규칙인가? ─ Yes → 해당 Canonical 문서
          │ No
실행 방법·장애 대응인가? ─ Yes → Active Guide
          │ No
아직 검증 전 설계인가?   ─ Yes → Working Proposal
          │ No
특정 run의 결과인가?     ─ Yes → Experiment Record
          │ No
과거 의사결정 근거인가?  ─ Yes → Historical/Legacy
```

## 8. 저장소 포함 범위

| Git에 포함 | Git 밖에서 관리 |
|---|---|
| catkin/standalone 코드와 설정 | 원본·파생 데이터 (`$ASMC_DATA`) |
| 재현 가능한 스크립트·manifest schema | 학습 checkpoint와 대용량 artifact |
| 규약·가이드·설계·실험 요약 | 주최 배포 원본, 논문 PDF, baseline clone |

외부 baseline과 MORAI 예제는 필요할 때 별도로 clone하고, 팀 레포 문서에는 source URL,
commit과 적용한 adapter만 기록한다.
