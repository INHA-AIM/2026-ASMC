# D2 perception 문서 통합 기록

> **상태:** 2026-09-22 package-group 진입점 통합 완료
>
> **범위:** `packages/asmc_perception/`, 관련 package README·handoff·contract·record의 역할 정리
>
> **시점 주의:** 이 문서는 D2 당시 구조화 판단을 보존한 기록이다. 이후 P4-C6에서
> `asmc_perception` 통합과 구 package 제거가 완료됐으므로 현재 경로·실행법·topic 정본은
> [Perception 개발·통합 가이드](../../../src/asmc_camera/docs/integration.md)와
> [runtime 명령](../../operations/commands.md)을 우선한다. 아래 당시 문장은 소급 수정하지 않는다.

## 결과

인지 문서 약 6,100행에는 현재 실행법뿐 아니라 날짜별 성능, 실패 원인, GT 정책 비교,
후보 알고리즘의 변화와 장기 계획이 함께 있다. 이를 짧은 문서 하나로 합치면 판단 근거가
사라지므로 이번 묶음에서는 원문 본문을 삭제·축약하지 않았다.

대신 [Perception package-group guide](../../../src/asmc_camera/docs/integration.md)를 추가해 현재 코드
경계, 문서 권위, runtime interface와 Phase 4 gate를 한곳에서 찾게 했다. 긴 문서에는 역할과
시점 주의만 보강했다.

## 원문 절과 현재 행선지

| 원문 | 고유 내용 | 현재 역할·행선지 | 처리 |
|---|---|---|---|
| `packages/asmc_perception/roadmap.md` §1~2 | 전체 우선순위와 본선 통신 경계 | Working Proposal + Chronological Record | 본문 보존 |
| roadmap §3.1 | HD Map 후보의 시간순 구축·검수 | 날짜별 개발 기록; 현재 HD Map guide가 runtime 우선 | 본문 보존, 상단 경고 보강 |
| roadmap §3.2~5 | camera/LiDAR/좌표/코드 경계 제안 | 설계 근거 | 본문 보존 |
| roadmap §6~7 | provisional topic과 camera C1~C4 과정 | 당시 후보와 구현 기록 | 본문 보존 |
| roadmap §8~9 | bag·GT 조사와 개발판/대회판 이식 | 운영·역사적 근거 | 본문 보존 |
| roadmap §10~11 | 단계 gate와 열린 질문 | Working Proposal | 본문 보존 |
| `bev-lane-interface-plan.md` §1~3 | GT 정책 비교와 P0~P4 결정 | Dated Detailed Design Note | 본문 보존 |
| BEV note §4~7 | MGeo·BEV/vector 초안·보류·규정 해석 | 상세 설계 근거 | 본문 보존 |
| `drivable_bev/README.md` §좌표~검증 | 현재 실행·토픽·calibration | Active Package Guide | 본문 보존 |
| drivable README의 날짜별 절 | range/loss/map/shared-family audit 전체 | Package-local Experiment Log | 본문 보존 |
| `camera_semantic_perception/README.md` | live 실행·토픽·계측과 1차 실측 | Active Package Guide + dated measurement | 본문 보존 |
| `twinlite_morai/README.md` | adapter 사용법과 학습 변화 | Active Module Guide + training log | 본문 보존 |
| `data_collection/README.md` | Capture·bag·mask·dataset 상세 절차 | Active Package Guide | 본문 보존 |
| `contracts/perception/dataset-labeling-split.md` | label·split·manifest 정본과 채택 근거 | Canonical Data Contract | 본문 보존 |
| `handoffs/perception/*` | 판제/LiDAR consumer 인수 조건 | Active Handoff Guides | 본문 보존 |
| `records/perception/*` | 특정 checkpoint·날짜의 성능 | Experiment Records | 본문 보존 |
| `traffic_light/README.md` | 독립 inference 실행·출력 | Active Package Guide | 본문 보존 |
| `lidar+sensorfusion/RUN_SENSOR_FUSION.md` | nested workspace build/run/fusion | Active Standalone Guide | 본문 보존 |

## 명확히 한 현재 경계

- `asmc_perception` package는 아직 없고 Phase 4 목표다.
- 완성된 HD Map은 별도 `asmc_hd_map`과 `/hd_map/*`가 소유한다. 과거 MGeo candidate 기록은
  현재 map 상태를 뜻하지 않는다.
- camera BEV/lane은 `base_link`의 관측이고 HD Map은 `map`의 정적 구조다.
- `/perception/lane/boundary_observations`는 typed provisional observation이며 절대 차로 의미나
  closed-loop 안전 승인을 자동 제공하지 않는다.
- traffic-light mission metadata가 있어도 inference process의 구간별 orchestration은 아직 별도
  구현 대상이다.
- nested LiDAR workspace, 중복 `Msg`·`morai_msgs`, vendor Velodyne는 parity gate 없이 합치거나
  삭제하지 않는다.
- 현재 LiDAR 10 Hz baseline과 Phase 3의 승인 목표 15 Hz를 분리한다.

## 변경하지 않은 것

- Python/C++ source, launch, config, model과 checkpoint
- topic 이름, message type, frame, timestamp와 threshold
- package 경로와 catkin workspace 구성
- 날짜별 성능 수치, 실패 분석, 계획과 열린 질문
- 본선 허용 입력 여부와 기존 handoff 상태

## 후속 정리 원칙

날짜별 audit를 package README에서 `records/perception/`으로 옮기면 탐색성이 좋아질 수 있다.
다만 다음 조건을 만족하는 별도 변경에서만 수행한다.

1. 제목 단위 원문→새 경로 대응표를 만든다.
2. 수치, 명령, artifact 경로와 실패 원인을 한 문장도 누락하지 않는다.
3. 기존 anchor를 사용하는 링크를 전수 교정하거나 compatibility anchor를 둔다.
4. package README에는 현재 실행·I/O·검증 gate만 남긴다.
5. `git diff --word-diff`와 link checker로 의미·링크 parity를 확인한다.

## 검증

- package.xml, launch, README와 contract/handoff/record의 역할을 대조
- 핵심 장문 원문 6개는 삭제 없이 기존 4,346행을 유지하고 안내 25행만 추가
- 변경 문서의 로컬 Markdown target·anchor 277개 검사 통과
- ROS Noetic container에서 camera 19, drivable BEV 169, data collection 31 unit tests 통과
- `git diff --check`
- 빈 `docs/` 디렉터리 없음

문서만 변경했으므로 runtime source와 model 회귀 수치는 변경하지 않는다. 다음 D2 묶음은
`packages/planning-control/`와 `handoffs/`다.
