# `asmc_msgs` 개발·호환 가이드

> **문서 역할:** Active Package Guide — 팀 내부 ROS1 typed message의 소유·변경·검증 규칙
>
> **현재 패키지:** `asmc_msgs` 1.0.0, message 25개

필드와 topic 의미는 [공용 계약 허브](../../contracts/README.md)의 개별 Frozen contract가
정본이다. 이 문서는 message package를 안전하게 유지하는 절차를 설명한다.

## 1. 책임과 경계

`asmc_msgs`는 팀이 소유하는 planning, perception, HD Map·대회 metadata message만 둔다.
publisher나 변환 로직은 포함하지 않는다.

- 기존 planning/perception 타입: 13개, ROS1 MD5 동결
- HD Map/competition 타입: 12개
- 합계: 25개
- 외부 MORAI wire type: pinned `src/MORAI-ROS_morai_msgs`, 이 패키지로 복사하지 않음
- 중복 `Msg` package: consumer 이관과 clean build 전까지 삭제하지 않음

| 계열 | 타입 |
|---|---|
| 차량·객체 | `VehicleState`, `DetectedObject*` |
| perception lane | `LaneBoundaryObservation*`, `LaneBoundaryFamily*` |
| planning v1/v2 | `BehaviorContext`, `PlanFeedback`, `BehaviorCommand`, `FrenetFeedback`, `Trajectory*` |
| HD Map | `HdMapMeta`, `LaneMarking*`, `Lane*`, `LaneAssociation`, `DrivableArea*` |
| competition | `CompetitionCheckpoint*`, `MissionZone*` |

현재 전체 목록은 `src/asmc_msgs/msg/`, 등록 순서는 `src/asmc_msgs/CMakeLists.txt`가 구현
정본이다.

## 2. 호환성 규칙

ROS1 MD5는 `.msg` 파일 SHA가 아니다. 주석과 공백을 제외한 상수, 필드 타입·이름·순서와
중첩 message MD5로 계산된다. publisher와 subscriber의 type 이름이 같아도 MD5가 다르면 연결이
거부된다.

따라서 동결 타입에는 다음 변경을 하지 않는다.

- 필드 추가·삭제·순서 변경
- 필드 타입이나 이름 변경
- 상수 값·이름 변경
- 중첩 message를 비호환하게 변경

새 정보가 필요하면 기존 타입을 유지하고 새 contract version과 새 message type을 추가한다.
주석 변경도 wire MD5는 유지하지만 source SHA가 달라지므로 감사 결과에는 나타난다.

## 3. 기계 판독 정본

| 정본 | 역할 |
|---|---|
| `config/contracts/hd_map_topic_contract_v1.yaml` | 25개 message MD5, 필수 필드와 11개 HD Map topic |
| `config/baselines/runtime_baseline_v1.yaml` | P1-A 당시 package/topic/message 회귀 snapshot |
| `src/asmc_msgs/test/test_message_serialization.py` | generated ROS type MD5와 serialization 왕복 |
| `tools/audit/validate_p1b_contract.py` | ROS 설치 없이 source contract 검증 |

P1-A baseline은 당시 시점의 frozen snapshot이고 이후 추가된 `LaneAssociation`까지 포함한 현재
계약은 topic contract v1을 따른다. 두 파일의 개수가 다르다는 이유로 baseline을 새 현재값으로
덮어쓰지 않는다.

## 4. build와 검증

ROS 없이 source contract를 검사한다.

```bash
python3 tools/audit/validate_p1b_contract.py
python3 -m unittest tools/audit/test_p1b_contract.py
```

ROS Noetic workspace에서는 generated type을 포함해 검사한다.

```bash
catkin_make --pkg asmc_msgs
catkin_make run_tests_asmc_msgs
```

message 변경 PR은 최소한 다음을 확인한다.

1. `msg/`, `CMakeLists.txt`, contract YAML이 같은 타입 목록을 갖는다.
2. 기존 13개 MD5가 P1-A 값과 같다.
3. 신규/후속 타입의 계산 MD5와 required field가 contract와 같다.
4. serialization test와 모든 영향 producer/consumer build가 통과한다.
5. topic 이름, frame, 단위와 latch/scope 변경은 별도 계약 version으로 리뷰한다.

## 5. 새 message를 추가할 때

1. producer와 consumer가 먼저 의미, frame, 단위, sentinel과 freshness를 합의한다.
2. 기존 타입 확장으로 해결하지 말고 새 타입/version 필요성을 판단한다.
3. `.msg`를 추가하고 `CMakeLists.txt`에 등록한다.
4. 기계 판독 계약에 MD5, required field와 topic 규칙을 추가한다.
5. source validator, generated serialization과 실제 producer/consumer를 함께 검증한다.
6. package README보다 [계약 문서](../../contracts/README.md)를 먼저 갱신한다.

`asmc_msgs` 타입이라는 사실만으로 데이터가 authoritative해지는 것은 아니다. 예를 들어
`MissionZone.status=PROVISIONAL`은 typed message여도 판제 mandatory rule로 사용할 수 없다.
