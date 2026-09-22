# D2 bridge 문서 통합 기록

> **상태:** 2026-09-22 통합 완료
>
> **범위:** 현재 MORAI bridge/state 구현, 실행 문서와 Phase 3 통합 계획

## 결과

| 기존 정보 위치 | 새 역할 | 처리 |
|---|---|---|
| `operations/README.md` | Docker·MORAI UI·다중 package 운영 | Canonical 운영 절차 유지 |
| `architecture/consolidation-roadmap.md` §6.2·§7 | Phase 3/6 목표 | 순서·확정 요구 유지 |
| bridge launch·source 주석 | 실제 I/O·단위·timestamp·TF | package-group guide로 설명 |
| `vehicle_state` source/config | 상태 추정과 source 선택 | package-group guide로 설명 |
| package 로컬 문서 없음 | 각 package 빠른 시작 | 네 README 추가 |
| `integration/README.md` | 논리 그룹 index | 상세 bridge guide로 연결 |

운영 가이드의 현재/목표 포트표를 다시 다른 정본으로 만들지 않았다. 새
[bridge guide](../packages/asmc_bridge/README.md)는 코드의 책임·단위·failure boundary와 통합
gate를 설명하고 포트 운영 정본으로 연결한다.

## 발견하고 명확히 한 경계

- `asmc_bridge` package와 `dev_local`/`competition_lan` profile은 아직 존재하지 않는다.
- `udp_bridge.sh --simulator competition`은 `simulator_profile` 라벨만 바꾸며 IP·port·allowlist를
  전환하지 않는다.
- 통합 launch는 현재 v2+GT 포트 합집합과 Object oracle을 열기 때문에 field-safe competition
  launch로 간주할 수 없다.
- sensor bridge의 GPS/IMU `9401/9403` 복제는 과거 standalone consumer 호환 경로이며 Phase 3
  필수 interface로 자동 승격하지 않는다.
- `vehicle_state.launch`의 GPS/IMU와 `vehicle_state_ego_dev.launch`는 같은 topic의 배타적 source다.
- `publish_map_tf=false`가 기본이고 localization과 sensor bridge가 동시에 map TF를 소유하면 안 된다.

이는 새 요구를 만든 것이 아니라 현재 launch/source와 기존 “아직 미적용” 문장을 대조해
오해 가능성을 제거한 것이다. 코드·launch·sensor JSON과 포트는 변경하지 않았다.

## 정보 보존 대응

| 내용 | 현재 행선지 |
|---|---|
| 현재/Phase 3 IP·port·Hz | [운영 가이드 §1·§4·§8](../operations/README.md) |
| 센서 pose·규정·본선 금지 입력 | [대회 환경](../competition/README.md) |
| package I/O·단위·timestamp·TF | [bridge guide §2~4](../packages/asmc_bridge/README.md) |
| 실행·diagnostics·test | [bridge guide §5~6](../packages/asmc_bridge/README.md) |
| Phase 3 profile와 이관 gate | [bridge guide §7~8](../packages/asmc_bridge/README.md) |
| 날짜별 transport 실측 | [2026-09 transport 기록](../records/simulator/runtime-transport-2026-09.md) |
| P1-A 당시 package/topic/sensor snapshot | [runtime baseline](../records/baselines/runtime-v1.md) |

## 변경하지 않은 것

- Python/C++ parser, estimator, test와 launch parameter
- 현재 `127.0.0.1`, Ctrl `9111`, Ego `909`, Collision `9092`
- sensor JSON과 VLP16 10 Hz
- topic 이름·message type·frame·timestamp 기본값
- planning 공동 소유 `integration_launch` 동작

## 검증

- 세 package와 integration launch의 parameter, publisher/subscriber를 source와 대조
- ROS Noetic container에서 관련 5개 package build
- bridge Python unit test 4개와 `vehicle_state_adapter_test` 통과
- container 누적 catkin 결과 485 tests, 0 errors/failures/skips
- `git diff --check`
- 저장소 Markdown target/anchor 검사와 빈 `docs/` 디렉터리 검사

다음 D2 묶음은 `packages/asmc_perception/`이다. 긴 roadmap, 세부 BEV 설계, package README와
날짜별 evaluation의 역할을 먼저 대조하며 모델 성능 기록을 현재 계약처럼 합치지 않는다.
