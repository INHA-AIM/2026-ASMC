# D2 `asmc_hd_map`·`asmc_msgs` 문서 통합 기록

> **상태:** 2026-09-22 통합 완료
>
> **범위:** 두 공용 package의 개발·검증 문서와 직접 연결된 index·현재 상태

## 결과

| 이전 위치 | 새 역할 | 처리 |
|---|---|---|
| `src/asmc_hd_map/README.md` 703줄 | package 빠른 시작 85줄 | 운영 실행과 정본 링크만 유지 |
| 같은 문서의 현재 builder/runtime 절 | `docs/packages/asmc_hd_map/README.md` | 생성물·pose·비교·변경 gate 통합 |
| 같은 문서의 날짜별 후보 절 | 기존 `reports/`와 `docs/records/hd-map/` | 정량·선택 근거의 기존 정본으로 연결 |
| `src/asmc_msgs/README.md` | package 빠른 설명 | 상세 변경 절차를 package guide로 분리 |
| 분산된 message 유지 규칙 | `docs/packages/asmc_msgs/README.md` | 25개 타입, MD5와 추가 절차 통합 |

Active package 문서에서는 “어떻게 실행·검증·변경하는가”만 찾을 수 있게 하고, 날짜가 있는
후보 생성 과정은 records/reports로 남겼다. Frozen topic/message 의미는 contracts에서
중복 작성하지 않았다.

## HD Map 원문 절 대응

| 기존 `src/asmc_hd_map/README.md` 절 | 현재 행선지 |
|---|---|
| 운영과 개발 진입점, 구조, 최종 runtime | [HD Map package guide](../packages/asmc_hd_map/README.md) §1~7 |
| 승격 원칙 | package guide §2, §8 |
| 첫 legacy audit | `reports/kcity2025_legacy_alignment_20260916/` |
| current-only corridor | `reports/kcity2025_current_corridor_20260916/` |
| 원태 관측 보정 | `reports/kcity2025_observed_calibration_20260916/` |
| 3-source candidate review | `reports/kcity2025_candidate_review_20260916/` |
| 구맵 채택·hybrid completion | 해당 `legacy_adoption`·`hybrid_completion` report |
| operational candidate | `reports/kcity2025_operational_map_20260916/`, 고정 config와 source SHA |
| route curation 1~14 | [route curation 기록](../records/hd-map/kcity2025-route-curation-20260917.md)과 package report |
| origin-only/H5 개발 비교 | package guide §7와 [runtime contract](../contracts/hd-map/runtime-v1.md) |
| Competition metadata | [metadata contract](../contracts/competition/kcity2025-metadata-v1.md) |
| P2-A/P2-B | [input audit](../records/baselines/kcity2025-drivable-input-v1.md), [compact product](../records/baselines/kcity2025-compact-drivable-v1.md) |

정량 후보 이력을 package guide에 다시 복사하지 않았다. 각 report의 CSV/JSON/PNG와 source
hash가 더 완전한 기록이며, 운영 제품과 섞이지 않는 것이 중요하다. 원문은 이 브랜치의 직전
commit `e6d732f`에도 보존돼 있다.

## 현재화한 내용

- 최종 build `kcity2025_roundabout_extended_drivable_v5`와 128 markings, 70 lanes,
  102 areas, 16 checkpoints, 15 zones를 명시했다.
- global vector가 판제 정본이고 local 120 m crop은 map-frame cache라는 경계를 유지했다.
- `/Ego_topic.position=(0,0)`을 map XY로 쓰지 않는 pose 규칙과 현재 차로 hysteresis를 보존했다.
- source comparison 다섯 레이어는 review-only이고 운영 product와 자동 혼합되지 않음을 명시했다.
- P1-A snapshot에는 없던 `LaneAssociation`을 포함해 현재 `asmc_msgs`가 25종임을 명시하되,
  frozen baseline 자체는 덮어쓰지 않았다.
- perception roadmap의 HD Map 절은 2026-09 후보 개발 이력임을 표시해 현재 운영 상태와의
  충돌을 막았다.

## 변경하지 않은 것

- `.msg`, contract YAML, builder source와 HD Map YAML/JSON/NPZ
- lane/stopline/drivable geometry와 feature count
- ROS1 MD5, topic 이름, frame·단위와 latch/scope
- 원태 H5, legacy map, report artifact

## 검증

- P1-B validator: 현재 25 messages, 11 HD Map topics
- metadata/input-audit/compact-map builder의 read-only `--check`
- `asmc_hd_map` ROS 비의존 unit tests 118개
- 저장소 Markdown target/anchor와 `git diff --check`

다음 D2 묶음은 `packages/asmc_bridge/`다. 현재 세 bridge/state package와 UDP profile 문서를
먼저 인벤토리화하고, 실제 통합 구현 전까지 존재하지 않는 `asmc_bridge` package를 완료된
것처럼 문서화하지 않는다.
