# 공용 계약 문서

> **문서 역할:** Active Index — 파트 간 구현이 반드시 맞아야 하는 계약의 단일 진입점
>
> 계약 본문의 enum, MD5, topic, 좌표계와 상태 전이는 요약만 보고 구현하지 않는다.

## 무엇을 먼저 읽을지

| 소비자·변경 범위 | 정본 | 상태 | 기계·구현 정본 |
|---|---|---|---|
| ROS1 공용 message, HD Map topic | [HD Map topic v1](./hd-map/topic-v1.md) | Frozen Interface | `config/contracts/hd_map_topic_contract_v1.yaml`, `src/asmc_msgs/msg/` |
| HD Map 실행·판제 소비·RViz | [HD Map runtime v1](./hd-map/runtime-v1.md) | Active Runtime | `src/asmc_hd_map/` |
| 체크포인트·미션·속도 구간 | [K-City metadata v1](./competition/kcity2025-metadata-v1.md) | Frozen Metadata | `src/asmc_hd_map/config/kcity2025_competition_metadata*.yaml` |
| Behavior/FSM ↔ Frenet | [FSM↔Frenet v2](./planning/fsm-frenet-v2.md) | Frozen Interface | `BehaviorCommand.msg`, `FrenetFeedback.msg`, `src/asmc_planner/` |
| 인지 원본·라벨·split | [dataset 계약](./perception/dataset-labeling-split.md) | Canonical Data Contract | versioned manifest와 `$ASMC_DATA` artifact |
| 인지 runtime typed topic | [Perception runtime v1](./perception/runtime-v1.md) | Frozen Additive Interface | `config/contracts/perception_runtime_topic_contract_v1.yaml`, `src/asmc_msgs/msg/` |
| 인지 runtime/debug 발행 정책 | [P5-A topic 감사](../records/baselines/perception-p5a-topic-audit-v1.md) | Frozen Input Audit | `config/contracts/perception_topic_policy_v1.yaml` |
| 인지 live producer·consumer·rate·frame | [P5-C runtime acceptance](../records/baselines/perception-p5c-runtime-acceptance-v1.md)·[P5-D/E closeout](../records/baselines/perception-p5de-closeout-v1.md) | Active Acceptance Gate | `config/contracts/perception_runtime_acceptance_v1.yaml` |
| MORAI message 호환 | [`beta_drive` 정책](./simulator/morai-msgs-beta-policy.md) | Canonical until competition port | pinned submodule `45c6baf` |

문서가 길다는 이유로 서로 다른 계약을 한 파일에 합치지 않는다. HD Map, planning transaction,
인지 데이터와 simulator wire schema는 변경 주체와 검증 방법이 달라 독립 정본이어야 한다.
이 페이지가 탐색용 요약이고 링크된 파일이 세부 규칙의 정본이다.

## 공통 불변식

- 좌표 geometry는 별도 명시가 없으면 `map` frame이다. local HD Map도 `base_link`로 다시
  변환한 geometry가 아니라 `map` frame subset이다.
- 단위는 SI를 기본으로 하며 속도 제한처럼 계약이 `kph`를 명시한 필드만 예외다.
- ROS1 message는 필드 타입·이름·순서·상수·중첩 타입이 MD5를 결정한다. 동결된 타입을
  제자리 수정하지 않고 새 version/type으로 확장한다.
- `official`, `derived_official`, `provisional`을 섞지 않는다. provisional metadata는 판제의
  강제 규칙이 될 수 없다.
- global HD Map 배열은 같은 `build_id`와 `geometry_sha256`일 때만 함께 사용한다.
- GT·oracle 입력과 개발 전용 gRPC는 competition runtime 입력으로 승격하지 않는다.
- dataset, checkpoint, bag과 생성 map은 이름만 같다고 같은 artifact로 보지 않고 manifest와
  SHA를 함께 확인한다.

## 변경할 때

| 변경 | 필요한 검증 |
|---|---|
| 기존 `asmc_msgs` | MD5 validator, serialization test, 모든 producer/consumer build |
| HD Map geometry/metadata | deterministic build, source SHA, count/hash, topic·RViz와 planner 소비 |
| Behavior/Frenet 상태 전이 | adapter/manager/core tests, ROS E2E, v1 병행 경로 회귀 |
| dataset label/split | manifest·run leakage·palette·shape/range·artifact SHA audit |
| 인지 launch/topic/consumer 경로 | P5-C 정적 validator와 live runtime acceptance |
| `morai_msgs` pin/field | submodule diff, 전체 catkin build, bridge packet/topic 회귀 |

계약 변경 PR에는 영향을 받는 producer와 consumer, 호환 방식, 실제 검증과 rollback을 적는다.
상태가 Frozen인 문서는 오탈자 외 변경도 관련 파트 합의가 필요하다.

## 문서 역할 경계

- 실행 명령과 장애 대응은 [운영 가이드](../operations/README.md)에 둔다.
- package 내부 설계와 파일 배치는 `docs/packages/`와 해당 package README에 둔다.
- 날짜가 있는 성능·학습 결과는 `docs/records/`에 둔다.
- 이식 과정과 더 이상 실행하지 않는 설계는 `docs/archive/`에 둔다.

일부 계약에는 아직 초기 rollout 기록이 같이 남아 있다. 그 절은 배경과 결정 근거이지 현재
작업 순서가 아니다. 별도 이동은 절 단위 의미 보존표를 만든 뒤 해당 package/records 정리
묶음에서 수행한다.

## 호환 경로

아래 파일은 D1 이전 외부 링크를 위한 한 릴리스 호환 안내이며 정본이 아니다.

- `hd-map-topic-contract-v1.md` → `hd-map/topic-v1.md`
- `hd-map-runtime-visualization-v1.md` → `hd-map/runtime-v1.md`
- `kcity2025-competition-metadata-v1.md` → `competition/kcity2025-metadata-v1.md`

D3에서 외부 참조와 내용 parity를 다시 확인하기 전에는 삭제하지 않는다.
