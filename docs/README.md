# 2026-ASMC 문서 허브

> **문서 상태:** Active Index
> **최종 정리:** 2026-09-13
> **대상:** 팀원 전원 · 관리: 통합 리드와 각 문서 담당자

문서 파일은 기존 링크와 팀원의 북마크가 깨지지 않도록 현재 경로를 유지한다. 이 페이지에서
정본, 작업 중 설계안, 사용 가이드와 과거 기록을 구분한다. 내용이 충돌하면 아래의
`정본 우선순위`를 적용하고, 기록 문서를 현재 설정으로 해석하지 않는다.

## 1. 처음 보는 팀원이 읽을 문서

아래 여섯 개를 먼저 읽고, 필요한 파트의 상세 문서로 이동한다.

1. [시작 가이드](./getting-started.md) — clone, Docker와 첫 실행
2. [협업 가이드](./collaboration.md) — Git, ownership와 PR 기준
3. [대회 환경·규정](./competition-environment.md) — v1.1 허용 입력과 당일 제약
4. [UDP↔ROS 네트워크](./network-udp-ros.md) — 26.R1.h3 개발 및 대회 통신 경계
5. [팀 아키텍처 개요](./architecture-overview.md) — 현재 시스템 한 장 요약
6. [팀원별 개발 가이드](./team-dev-guide.md) — 담당 경로와 검증 책임

현재 인지 개발자는 [카메라·MGeo·LiDAR 융합 로드맵](./perception-map-fusion-roadmap.md),
planning/control 개발자는 [Planning/Control UDP 규약](./planning-control-udp.md)을 이어서
읽는다.

## 2. 정본 우선순위

| 주제 | 현재 정본 | 충돌 시 처리 |
|---|---|---|
| 대회 규정 | 주최 측 최신 규정집·공식 공지 → [competition-environment.md](./competition-environment.md) | 공식 원문이 항상 우선. 요약 문서를 함께 갱신 |
| 시뮬 통신·포트 | [network-udp-ros.md](./network-udp-ros.md) | Docker/getting-started의 중복 설명보다 우선 |
| Git·ownership | [collaboration.md](./collaboration.md) | git-workflow/team-dev-guide의 예시보다 우선 |
| Planning/Control 실행 | [planning-control-udp.md](./planning-control-udp.md) | planning-behavior-handoff의 ROS 경로는 레거시 |
| 카메라 데이터·라벨·split | [perception-dataset-labeling-split.md](./perception-dataset-labeling-split.md) | 개별 학습 기록보다 우선 |
| 인지 융합 방향·우선순위 | [perception-map-fusion-roadmap.md](./perception-map-fusion-roadmap.md) | BEV 세부안·초기 agentic 문서보다 우선 |
| 공용 메시지 계약 | [src/interfaces/README.md](../src/interfaces/README.md) | 현재는 초안. pilot 뒤 합의된 항목만 정본으로 승격 |

`Working Proposal`은 구현을 시작하기 위한 후보안이지 다른 파트가 반드시 따라야 하는
확정 계약이 아니다.

## 3. 주제별 문서 지도

### 3.1 온보딩·개발 환경

| 문서 | 역할 | 상태 |
|---|---|---|
| [getting-started.md](./getting-started.md) | clone부터 첫 build/run까지 최소 절차 | Active Guide |
| [development-environment.md](./development-environment.md) | OS, Python, ROS, 데이터 경로 표준 | Active Reference |
| [docker-guide.md](./docker-guide.md) | ROS/학습 컨테이너 운영과 트러블슈팅 | Active Guide |
| [repository-layout.md](./repository-layout.md) | 코드 위치와 경로별 ownership 요약 | Active Reference |
| [team-dev-guide.md](./team-dev-guide.md) | 팀원별 작업 경로·주의사항·검증 | Active Guide |

### 3.2 협업·통합

| 문서 | 역할 | 상태 |
|---|---|---|
| [collaboration.md](./collaboration.md) | trunk, Path A/B, ownership의 정책 정본 | Canonical |
| [git-workflow.md](./git-workflow.md) | collaboration 정책을 실행하는 명령 예시 | Command Reference |
| [integration-plan.md](./integration-plan.md) | Wave 1/2 이식 과정과 당시 판단 | Historical Record |
| [architecture-overview.md](./architecture-overview.md) | 전체 모듈과 현재 목표 구조 요약 | Working Overview |

### 3.3 대회·시뮬·네트워크

| 문서 | 역할 | 상태 |
|---|---|---|
| [competition-environment.md](./competition-environment.md) | 규정 v1.1, 센서와 미션 요약 | Canonical Summary |
| [competition-data.md](./competition-data.md) | 배포 map/path/scenario/checkpoint 자산 | Active Reference |
| [network-udp-ros.md](./network-udp-ros.md) | UDP bridge, 포트, Docker 네트워크와 진단 | Canonical |
| [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md) | `beta_drive` submodule과 필드 정렬 규칙 | Canonical until port |
| [morai-msgs-versions.md](./morai-msgs-versions.md) | 26.R1과 대회 msg 차이 조사 기록 | Version Reference |
| [sim-verification-checklist.md](./sim-verification-checklist.md) | 파트별 시뮬 통합 검증 | Active Checklist, 일부 레거시 항목 표기 |

MORAI 화면에 입력할 값만 필요하면
[UDP 치트시트](../config/morai_udp_sim_cheat_sheet.md)를 사용한다. 상세 설명과 장애 대응은
치트시트가 아니라 `network-udp-ros.md`에서 관리한다.

### 3.4 Planning/Control

| 문서 | 역할 | 상태 |
|---|---|---|
| [planning-control-udp.md](./planning-control-udp.md) | 현재 standalone Frenet/MPC 실행 규약 | Canonical |
| [planning-control-refactoring-guidelines.md](./planning-control-refactoring-guidelines.md) | Frenet/MPC 아키텍처 경계, ownership, dependency 방향, refactoring 체크리스트 | Active Reference |
| [planning-behavior-handoff.md](./planning-behavior-handoff.md) | 이전 ROS 3-package 통합 설계와 검증 기록 | Legacy Reference |

Behavior Planner가 다시 연결되기 전에는 handoff 문서의 토픽과 launch를 현재 실행 경로에
적용하지 않는다.

### 3.5 Perception·데이터·BEV

| 문서 | 역할 | 상태 |
|---|---|---|
| [perception-map-fusion-roadmap.md](./perception-map-fusion-roadmap.md) | camera/MGeo/LiDAR 전체 전략, live와 대회판 이식 | Current Working Proposal |
| [camera-object-detection-handoff.md](./camera-object-detection-handoff.md) | 손재호용 YOLO 즉시 사용·경량 후보·NAS fine-tune 인수 가이드 | Active Guide |
| [rosbag-evaluation-streaming-gt.md](./rosbag-evaluation-streaming-gt.md) | raw/result bag, source timestamp, Streaming GT와 ROS/UDP gate | Active Guide |
| [perception-dataset-labeling-split.md](./perception-dataset-labeling-split.md) | Capture 원본, mask, ignore, manifest와 split 계약 | Canonical Data Contract |
| [perception-bev-lane-interface-plan.md](./perception-bev-lane-interface-plan.md) | GT 정책, metric lane 후보, BEV/lane topology 상세 | Detailed Design Note |
| [perception-v4-training-evaluation.md](./perception-v4-training-evaluation.md) | TwinLiteNet+ v4 10/18 epoch 결과 | Experiment Record |
| [perception-agentic-loop.md](./perception-agentic-loop.md) | 초기 Loc-Aid/bake/BEV 평가 루프 아이디어 | Research Reference |

인지 문서의 역할은 다음처럼 분리한다.

```text
roadmap       무엇을 어떤 순서로 개발할지
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
| 처음 실행 | [getting-started](./getting-started.md) | [docker-guide](./docker-guide.md), [network-udp-ros](./network-udp-ros.md) |
| UDP 센서가 안 들어옴 | [network-udp-ros](./network-udp-ros.md) | [UDP 치트시트](../config/morai_udp_sim_cheat_sheet.md), [sim checklist](./sim-verification-checklist.md) |
| Git 작업·PR | [collaboration](./collaboration.md) | [git-workflow](./git-workflow.md), [team-dev-guide](./team-dev-guide.md) |
| 본선 허용 여부 판단 | [competition-environment](./competition-environment.md) | [network-udp-ros](./network-udp-ros.md), [competition-data](./competition-data.md) |
| MPC/Frenet 실행 | [planning-control-udp](./planning-control-udp.md) | [sim checklist](./sim-verification-checklist.md) |
| Capture/마스크/학습 데이터 | [dataset contract](./perception-dataset-labeling-split.md) | [data_collection README](../src/data_collection/README.md) |
| raw bag 녹화·재생·모델 A/B | [rosbag/Streaming GT guide](./rosbag-evaluation-streaming-gt.md) | [data_collection README](../src/data_collection/README.md) |
| 카메라 live 추론 | [fusion roadmap §5~7](./perception-map-fusion-roadmap.md#5-코드-경계-제안) | [v4 evaluation](./perception-v4-training-evaluation.md) |
| 카메라–LiDAR 객체 분류 | [object detector handoff](./camera-object-detection-handoff.md) | [sensor-fusion 실행](../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md), [rosbag 평가](./rosbag-evaluation-streaming-gt.md) |
| MGeo·카메라·LiDAR BEV | [fusion roadmap](./perception-map-fusion-roadmap.md) | [BEV design note](./perception-bev-lane-interface-plan.md) |
| 대회판 이식 | [fusion roadmap §9](./perception-map-fusion-roadmap.md#9-26r1h3-개발판과-대회판-이식-전략) | [msg versions](./morai-msgs-versions.md), [network](./network-udp-ros.md) |

## 5. 모듈 README 진입점

| README | 역할 |
|---|---|
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
