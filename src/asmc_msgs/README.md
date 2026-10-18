# `asmc_msgs` 개발·호환 가이드

ASMC 내부 ROS1 메시지의 정본 패키지다. 패키지 버전 `1.0.0`부터 기존 planning/perception
메시지 13개와 HD Map·competition metadata 메시지 12개의 MD5를 동결하며, P4-B에서
perception runtime 메시지 3개를 additive하게 추가했다.

> **문서 역할:** Active Package Guide — 팀 내부 ROS1 typed message의 소유·변경·검증 규칙

- 공용 계약 허브: [contracts/README.md](../../docs/contracts/README.md)
- 사람이 읽는 규약: [HD Map topic contract v1](../../docs/contracts/hd-map-topic-v1.md)
- perception 규약: [Perception runtime topic contract v1](../../docs/contracts/perception-runtime-v1.md)
- 기계 판독 규약: [hd_map_topic_contract_v1.yaml](../../config/contracts/hd_map_topic_contract_v1.yaml)
- message registry: [asmc_msgs_contract_registry_v1.yaml](../../config/contracts/asmc_msgs_contract_registry_v1.yaml)
- 검증: `python3 tools/audit/validate_p1b_contract.py`

필드와 topic 의미는 공용 계약 허브의 개별 Frozen contract가 정본이다. 이 README는 message
package를 안전하게 유지하는 절차를 설명한다.

## 책임과 경계

`asmc_msgs`는 팀이 소유하는 planning, perception, HD Map·대회 metadata message만 둔다.
publisher나 변환 로직은 포함하지 않는다.

- 기존 planning/perception 타입: 13개, ROS1 MD5 동결
- HD Map/competition 타입: 12개
- perception runtime additive 타입: 3개
- 위 P1/P4 등록은 당시28개 subset. 현재 CMake 등록은38개 메시지·1개 서비스이며 UrbanFeatureDebug, LiDAR/camera handoff·공용 추론 제어·센서/grid timing·시각화 지면 metadata를 포함
- 외부 MORAI wire type: pinned `src/MORAI-ROS_morai_msgs`, 이 패키지로 복사하지 않음
- 중복 `Msg` package: consumer 이관과 clean build 전까지 삭제하지 않음

| 계열 | 타입 |
|---|---|
| 차량·객체 | `VehicleState`, `DetectedObject*` |
| perception lane | `LaneBoundaryObservation*`, `LaneBoundaryFamily*` |
| planning v1/v2 | `BehaviorContext`, `PlanFeedback`, `BehaviorCommand`, `FrenetFeedback`, `Trajectory*` |
| HD Map | `HdMapMeta`, `LaneMarking*`, `Lane*`, `LaneAssociation`, `DrivableArea*` |
| competition | `CompetitionCheckpoint*`, `MissionZone*` |
| perception runtime v1 | `TrafficLightDetection`, `TrackedObject*` |
| 카메라/이전용 LiDAR | `CameraObjectBox*`, `CameraObjectObservation`, `LidarObject*` |
| 추론 제어 v2 | `InferenceStatus`, `SetInferenceMode.srv` — 생산자 UUID·모드·준비 상태 |
| 판제 debug | `UrbanFeatureDebug` |
| 센서 timing v1 | `SensorFrameTiming` — 원본/수신/발행 clock과 기존 sensor wire의 연결 키 |
| Model-grid timing v1 | `CameraModelGridTiming` — 원본 JPEG와 class/confidence의 실제 wire header·추론 세대 연결 |
| 내부 BEV 시각화 | `CameraGroundProjection` — family가 선택한 프레임별 지면을 RGB·semantic 비교 화면에 재사용 |

공용 추론 제어는 [v2 계약](../../config/contracts/camera_inference_control_v2.yaml)을 따른다.
생산자는 asmc_camera이며 구형 카메라 타입은 기존 제어 경로에서 호환용으로 유지한다.
정본 상태 경로 소비자는 공용 타입으로 재빌드해야 한다.

센서 timing은 [v1 계약](../../config/contracts/sensor_frame_timing_v1.yaml)과
[기록·활용 규칙](../../docs/architecture/bridge-sensor-timing.md)을 따른다.
기존 `sensor_msgs`의 header/MD5를 바꾸거나 LiDAR matching·차선 운동 보정을 자동 활성화하지 않는다.

현재 전체 목록은 `msg/`·`srv/`, 등록 순서는 `CMakeLists.txt`가 구현 정본이다.

이 패키지는 interface만 제공한다. HD맵 publisher/metadata/geometry는 asmc_hd_map,
카메라 관측은 asmc_camera, 현재 LiDAR 실행은 nested Main, 판제는 asmc_planner가 소유한다. 판제는 `/hd_map/global/*` latched 토픽을 한 번 받아 위치 기반으로
조회하는 방식을 기본으로 하고, `/hd_map/local/lanes`와 `/hd_map/local/drivable_grid`는
선택적 파생 캐시로만 사용한다. `/hd_map/local/current_lane`은 후륜축 중심 `base_link`를
기준으로 한 상태 포함(`INSIDE/BOUNDARY/NEARBY/STALE`) 차로 association이다. local crop에는
mission/checkpoint/marking metadata가 중복되지 않는다.

## ROS1 호환성 규칙

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

## 기계 판독 정본

| 정본 | 역할 |
|---|---|
| `config/contracts/asmc_msgs_contract_registry_v1.yaml` | 현재 모든 공용 message와 소유 계약의 대응 |
| `config/contracts/hd_map_topic_contract_v1.yaml` | 기존·HD Map 25개 message MD5, 필수 필드와 11개 HD Map topic |
| `config/contracts/perception_runtime_topic_contract_v1.yaml` | perception additive 3개 message와 canonical/compatibility topic |
| `config/baselines/runtime_baseline_v1.yaml` | P1-A 당시 package/topic/message 회귀 snapshot |
| `config/baselines/hd_map_runtime_pins_v1.yaml` | HD Map publisher가 읽는 현재 map·metadata SHA와 개수 pin |
| `src/asmc_msgs/test/test_message_serialization.py` | generated ROS type MD5와 serialization 왕복 |
| `tools/audit/validate_p1b_contract.py` | ROS 설치 없이 source contract 검증 |

P1-A baseline은 당시 시점의 frozen snapshot이고 이후 추가된 `LaneAssociation`까지 포함한 현재
계약은 topic contract v1을 따른다. 두 파일의 개수가 다르다는 이유로 baseline을 새 현재값으로
덮어쓰지 않는다.

## Build와 검증

ROS 없이 source contract를 검사한다.

```bash
python3 tools/audit/validate_p1b_contract.py
python3 tools/audit/validate_perception_contract.py
python3 -m unittest tools/audit/test_p1b_contract.py
python3 -m unittest tools/audit/test_perception_contract.py
```

ROS Noetic 환경의 단독 build 명령은 다음과 같다.

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

## 새 message를 추가할 때

1. producer와 consumer가 먼저 의미, frame, 단위, sentinel과 freshness를 합의한다.
2. 기존 타입 확장으로 해결하지 말고 새 타입/version 필요성을 판단한다.
3. `.msg`를 추가하고 `CMakeLists.txt`에 등록한다.
4. 기계 판독 계약에 MD5, required field와 topic 규칙을 추가한다.
5. source validator, generated serialization과 실제 producer/consumer를 함께 검증한다.
6. package README보다 [계약 문서](../../docs/contracts/README.md)를 먼저 갱신한다.

`asmc_msgs` 타입이라는 사실만으로 데이터가 authoritative해지는 것은 아니다. 예를 들어
`MissionZone.status=PROVISIONAL`은 typed message여도 판제 mandatory rule로 사용할 수 없다.
