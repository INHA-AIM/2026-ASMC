# HD Map·competition metadata topic contract v1

> **상태:** Frozen Interface — P1-B
> **기계 판독 정본:**
> [`config/contracts/hd_map_topic_contract_v1.yaml`](../../../config/contracts/hd_map_topic_contract_v1.yaml)
> **적용 범위:** 메시지와 토픽 의미 동결. P1-C metadata와 P1-D read-only publisher 구현 완료,
> Lane/DrivableArea 제품은 2단계에서 구현

## 1. 호환성 원칙

`asmc_msgs` 1.0.0은 기존 13개 타입을 수정하지 않고 HD Map/대회 metadata 타입 12개를
추가한다. 기존 타입의 ROS1 MD5는 P1-A 값과 같아야 한다. 신규 타입도 contract YAML에 MD5를
고정하며, 필드 수정은 같은 이름으로 하지 않고 새 계약 버전과 새 message type으로 진행한다.

ROS1 MD5는 `.msg` 파일 자체의 SHA가 아니다. 주석과 공백을 제외한 상수, 필드 타입·이름·순서와
중첩 message의 MD5로 계산되는 wire schema 식별자다. 동일한 topic/type 이름이라도 publisher와
subscriber의 MD5가 다르면 ROS master 협상 뒤 TCPROS/UDPROS 연결이 거부된다. 기존 13개 값은
다음과 같이 고정했다.

| 기존 타입 | ROS1 MD5 |
|---|---|
| `BehaviorCommand` | `fad196232c8f54cbbe2a1bced9d3972a` |
| `BehaviorContext` | `450f8f3ab65064fbb3622550828cadc5` |
| `DetectedObject` | `a965285ae904ae3ade687b9c54c2c515` |
| `DetectedObjectArray` | `14fbd9679c2d87ae7498e4269d9678da` |
| `FrenetFeedback` | `072b15bb1dfe3f5b99d17c1487408114` |
| `LaneBoundaryFamily` | `70ac79f98ca1244c782ea4067becd6af` |
| `LaneBoundaryFamilyMember` | `516612ff4e656e2e61852056fda78969` |
| `LaneBoundaryObservation` | `6ae63de01bc8e43ba30dbb4add2129c3` |
| `LaneBoundaryObservationArray` | `f4c14183f7aa8631b2893327df0651f1` |
| `PlanFeedback` | `b5132603c037f3cf42de5a795df54d52` |
| `Trajectory` | `38657ba74f761a20afb310b0265cb085` |
| `TrajectoryPoint` | `f3382a560d156ee4a98b9951e48b3f3e` |
| `VehicleState` | `7b5b81807cda33ae90638895dcf749f1` |

따라서 필드 추가·삭제, 순서나 타입 변경, 상수 변경, 중첩 message 변경은 MD5를 바꾼다. 주석만
바꾸는 것은 MD5를 바꾸지 않지만 source SHA는 달라지므로 P1-A 감사 기록에는 별도로 나타난다.

```bash
python3 tools/audit/validate_p1b_contract.py
python3 -m unittest tools/audit/test_p1b_contract.py
# ROS Noetic workspace build/source 후 실제 generated type serialization 검사
python3 -m unittest src/asmc_msgs/test/test_message_serialization.py
```

검증기는 ROS 설치 없이 ROS1 MD5 text 규칙을 적용한다. 실제 배포 전에는 ROS Noetic
환경에서 `catkin_make --pkg asmc_msgs`와 `catkin_make run_tests_asmc_msgs`도 통과해야 한다.

## 2. 메시지 경계

| 계열 | 역할 |
|---|---|
| `HdMapMeta` | schema/build/map ID, artifact/source SHA, 생성 시각, global/local scope와 feature count |
| `LaneMarking*` | 흰색/노란색, 실선/점선, 폭, 물리 형태, 논리 역할, 원본과 polyline |
| `Lane*` | 차로 번호의 유효 범위, centerline/area, 경계·인접·선후행 topology |
| `LaneAssociation` | 후륜축 중심 현재 차로, 횡오차·경계 여유·전환 상태와 stale 여부 |
| `DrivableArea*` | lane/junction/merge 등의 vector polygon과 intersection guard 여부 |
| `CompetitionCheckpoint*` | 공식 pose/yaw, 3 m 통과 반경, route/link station |
| `MissionZone*` | mission 종류·구간·규칙과 `official/derived_official/provisional` 권위 상태 |

차선 도색과 통행 차로를 분리했으므로 하나의 `LaneMarking`을 양쪽 `Lane`이 공유할 수 있다.
`lane_number`는 전역 고정 번호가 아니라 같은 진행 방향 단면의 왼쪽→오른쪽 1-based 번호다.
`lane_number_scope_id`와 route station 구간 밖에서는 그 번호를 사용하면 안 된다.

polygon은 첫 점을 끝에 반복하지 않는 open ring이고 `map` 좌표를 사용한다. local 토픽도
차량 좌표로 변환한 geometry가 아니라 global ID를 유지하는 `map` frame crop이다.

## 3. 토픽

| 토픽 | 타입 | 전달 규칙 |
|---|---|---|
| `/hd_map/meta` | `asmc_msgs/HdMapMeta` | global, latched |
| `/hd_map/global/lane_markings` | `asmc_msgs/LaneMarkingArray` | global, latched |
| `/hd_map/global/lanes` | `asmc_msgs/LaneArray` | global, latched |
| `/hd_map/global/drivable_areas` | `asmc_msgs/DrivableAreaArray` | global, latched |
| `/hd_map/global/drivable_grid` | `nav_msgs/OccupancyGrid` | global, latched |
| `/hd_map/global/checkpoints` | `asmc_msgs/CompetitionCheckpointArray` | global, latched |
| `/hd_map/global/mission_zones` | `asmc_msgs/MissionZoneArray` | global, latched |
| `/hd_map/local/lanes` | `asmc_msgs/LaneArray` | ego update, non-latched |
| `/hd_map/local/drivable_grid` | `nav_msgs/OccupancyGrid` | ego update, non-latched |
| `/hd_map/local/current_lane` | `asmc_msgs/LaneAssociation` | ego update, non-latched |
| `/hd_map/debug/markers` | `visualization_msgs/MarkerArray` | debug only, non-latched |

모든 publisher queue size는 1이다. 전역 토픽은 build가 바뀔 때 다시 발행하고, 늦게 접속한
consumer가 전체 정적 상태를 즉시 받을 수 있게 latch한다. `OccupancyGrid` 값은 drivable `0`,
non-drivable `100`, unknown `-1`로 고정한다.

## 4. 안전 규칙

- `LaneMarking.pattern`만 실선/점선의 정본이다. polyline 점 간격으로 추정하지 않는다.
- `DrivableArea.intersection_guarded=true`면 교차로·합류·분기·무차선 구간의 road mesh를
  보존한 것이며 차선 안쪽으로 다시 clip하지 않는다.
- `MissionZone.STATUS_UNSPECIFIED`는 publish 불가다.
- `STATUS_PROVISIONAL`은 RViz/debug에는 사용할 수 있지만 mandatory planning/control 규칙을
  활성화할 수 없다.
- checkpoint의 `official_yaw_deg`는 공식 원문을 보존하고 `pose.orientation`은 publish 시
  map-frame quaternion으로 생성한다.
- global/local 배열의 ID는 동일 namespace를 사용한다. crop 때문에 ID를 다시 매기지 않는다.
- 현재 차로 기준점은 GPS 안테나나 전방 범퍼가 아닌 map-frame `base_link` 원점(후륜축 중심)이다.
  공유 경계 위에서는 기존 차로를 유지하고, 인접 차로 안쪽 0.30 m pose가 2회 연속 관측될 때
  횡방향 전환한다. 링크 선후행 전환은 이전 polygon을 벗어나 successor에 들어오면 즉시 수행한다.
- 노드 전환에서는 명시된 predecessor/successor를 우선한다. 확정돼 있던 차로가 검색 범위를
  완전히 벗어난 뒤 보호 교차로 support polygon과 전역경로 polygon이 겹치는 경우에만 유효한
  route station 차로를 재획득 기준으로 우선한다. 최초 획득과 인접차로 이동은 거리 및 위
  hysteresis를 그대로 적용하므로 장애물 회피용 support link도 선택할 수 있다.
- 교차로 내부에서 topology로 직접 연결되지 않은 경로로 이동할 때는 이전 polygon을 벗어나고
  새 교차로 centerline이 0.75 m 이상 더 가까울 때만 즉시 전환한다. 큰 교차로 polygon의
  중첩 때문에 이미 지나온 branch를 계속 현재 차로로 유지하는 현상을 방지한다.
- `NEARBY`는 최대 2 m의 작은 이탈 중 연속성을 위한 상태일 뿐 `INSIDE`가 아니다. `STALE`은
  planning에 유효하지 않다.
- 고주로의 A433 연결 1차로는 버스전용이며 global Lane 제품에서 제외한다. 대회 주행 후보는
  차로번호 2·3·4와 진입 합류차로 5뿐이다.

## 5. 이 단계에서 하지 않는 일

P1-B에는 기존 HD Map geometry 변환, 체크포인트 값 채우기, 미션 경계 확정, drivable-area
clipping, publisher/RViz 구현이 포함되지 않는다. 각각 P1-C, P1-D와 2단계의 검증 gate를 거친다.
