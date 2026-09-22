# D2 공용 계약 문서 정리 기록

> **후속 상태:** D4에서 실제 `asmc_hd_map`·`asmc_msgs` package guide를 각 source README에
> 합쳤다. 아래 마지막 문장의 중앙 경로는 D2 당시 작업 순서를 기록한다.

> **상태:** 2026-09-22 정리 완료
>
> **범위:** `docs/contracts/`의 탐색 구조, 현재 상태 표기와 구현 경로 정합성

## 결과

`contracts/README.md`를 공용 계약의 단일 진입점으로 추가하고 여섯 계약의 소비자, 상태,
기계·구현 정본과 변경 gate를 한 표에 모았다. 세 개의 D1 이전 경로는 외부 링크 유예를 위해
호환 안내로 유지한다.

| 계약 | 처리 |
|---|---|
| `hd-map/topic-v1.md` | P1-B 당시 범위와 현재 Phase 2 구현 완료 상태를 분리 |
| `hd-map/runtime-v1.md` | 현재 topic·planner 소비·RViz 실행 정본으로 유지 |
| `competition/kcity2025-metadata-v1.md` | 공식/파생/잠정 권위와 checkpoint·mission 정본 유지 |
| `planning/fsm-frenet-v2.md` | frozen 의미는 유지하고 production 경로·rollout 기록 상태를 교정 |
| `perception/dataset-labeling-split.md` | active data contract와 날짜별 rollout 결과의 읽기 경계를 명시 |
| `simulator/morai-msgs-beta-policy.md` | 이미 등록된 submodule의 현재 사용법·pin·ownership으로 교정 |

## 왜 계약을 한 파일로 합치지 않았는가

이 묶음은 문서 수보다 의미 경계가 중요하다. 다음 항목은 owner, 호환성 실패 형태와 검증기가
서로 달라 한 파일로 합치면 변경 리뷰 범위가 불명확해진다.

- HD Map wire schema와 runtime 제품
- 공식 체크포인트/미션 metadata
- Behavior/Frenet transaction state machine
- 인지 원본·라벨·split과 artifact provenance
- MORAI 외부 message schema

따라서 반복된 “어디서 무엇을 읽을지”는 index로 통합하되 enum, MD5, topic, 상태 전이,
dataset version과 SHA 의미는 축약하지 않았다. 특히 2,295줄의 FSM/Frenet 문서는 계약과
rollout 기록이 섞여 있지만 이번 묶음에서 대량 재작성하지 않았다. 1~11절은 wire/runtime
계약, 12절은 역사적 rollout이라는 경계를 먼저 명시했다. 구현 상세 이동은 후속
`packages/planning-control/`·`handoffs/` 묶음에서 절 단위 대응표를 만든 뒤 진행한다.

## 현재화한 드리프트

- HD Map topic 문서의 “Lane/DrivableArea는 이후 구현” 표현을 현재 완료 상태로 바꿨다.
- FSM/Frenet의 production topic과 message 경로를 현재형으로 바꾸고 존재하지 않는
  `global/global.hpp`, `src/ros/`를 실제 `Common/BehaviorTypes.hpp`, `src/Runtime/`으로 교정했다.
- EMERGENCY_STOP ownership은 현재 열린 production gate와 맞췄고 MERGE만 열린 질문으로 남겼다.
- `morai_msgs`를 새로 `submodule add`하라는 과거 절차를 제거하고 현재 pinned submodule의
  clone/update 절차로 바꿨다.
- 팀 저장소 밖 로컬 “대회 ROS 실측” 링크에 의존하지 않고 저장소의 운영 정본을 연결했다.

계약의 필드, 상수, MD5, topic 이름, 좌표계, mission/checkpoint 값, dataset split 값은 바꾸지
않았다. source code, message와 config도 수정하지 않았다.

## 호환 파일

| 호환 안내 | 정본 |
|---|---|
| `contracts/hd-map-topic-contract-v1.md` | `contracts/hd-map/topic-v1.md` |
| `contracts/hd-map-runtime-visualization-v1.md` | `contracts/hd-map/runtime-v1.md` |
| `contracts/kcity2025-competition-metadata-v1.md` | `contracts/competition/kcity2025-metadata-v1.md` |

이 파일은 D3에서 외부 참조와 한 릴리스 유예를 확인한 뒤 제거 여부를 결정한다.

## 검증

- `config/contracts/hd_map_topic_contract_v1.yaml`의 25개 message MD5와 문서의 기존 13개
  ROS1 MD5를 validator로 대조한다.
- `BehaviorCommand.msg`, `FrenetFeedback.msg`, production topic과 현재 Frenet source/test
  경로가 실제 tree에 존재하는지 확인한다.
- pinned `morai_msgs` commit이 `45c6baf148f2327f4c9fefd48262f25bdfe4b567`인지 확인한다.
- 저장소 내부 Markdown target/anchor와 `git diff --check`를 통과해야 한다.

다음 D2 묶음은 `packages/asmc_hd_map/`과 `packages/asmc_msgs/`다. 계약을 다시 설명하지 않고
builder/runtime 개발법, 생성물, 검증 책임을 package 관점에서 정리한다.
