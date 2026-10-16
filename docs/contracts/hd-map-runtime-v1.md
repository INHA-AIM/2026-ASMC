# HD Map runtime·visualization v1

> **상태:** Active Runtime — P2-B compact lane/drivable product
> **선행 계약:** [HD Map topic contract v1](./hd-map-topic-v1.md)
> **metadata 정본:** [K-City 2025 competition metadata v1](competition-kcity2025-metadata-v1.md)

## 1. 실행

ROS Noetic 컨테이너에서 다음처럼 실행한다.

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_hd_map competition_map_debug.launch
```

판제용 정식 진입점은 RViz를 기본으로 띄우지 않는다.

```bash
roslaunch asmc_hd_map competition_map.launch
rosrun asmc_hd_map verify_competition_map_topics.py
```

publisher는 시작 전에 stage14 map, P1-C metadata, 공식 global path와 P2-B product의 SHA를
검사한다. 하나라도 다르면 토픽을 일부 발행하지 않고 노드 시작 자체를 거부한다.

## 2. 현재 발행 토픽

| 토픽 | 타입 | 전달 | 내용 |
|---|---|---|---|
| `/hd_map/meta` | `asmc_msgs/HdMapMeta` | latched | build/map/geometry/manifest SHA와 준비 상태 |
| `/hd_map/global/lane_markings` | `asmc_msgs/LaneMarkingArray` | latched | 차선 도색 121개와 정지선 7개 |
| `/hd_map/global/lanes` | `asmc_msgs/LaneArray` | latched | route/same-direction/mission lane, A134 로터리와 음영 반대차로 총 70개 |
| `/hd_map/global/drivable_areas` | `asmc_msgs/DrivableAreaArray` | latched | Lane 70 + corridor fill 18 + surface patch 14 = vector area 102개 |
| `/hd_map/global/drivable_grid` | `nav_msgs/OccupancyGrid` | latched | vector area에서 만든 0.2 m lookup raster |
| `/hd_map/global/checkpoints` | `asmc_msgs/CompetitionCheckpointArray` | latched | 공식 체크포인트 15개(규정집 v1.2), 3 m, yaw, route/link station |
| `/hd_map/global/mission_zones` | `asmc_msgs/MissionZoneArray` | latched | 미션·속도 profile·상시 인지 policy zone 15개 |
| `/hd_map/local/lanes` | `asmc_msgs/LaneArray` | pose update, non-latched | ego 전방 90 m·후방 30 m·좌우 각 30 m 창과 교차하는 Lane subset |
| `/hd_map/local/drivable_grid` | `nav_msgs/OccupancyGrid` | pose update, non-latched | ego heading으로 회전한 `600×300`, 0.2 m local crop |
| `/hd_map/local/current_lane` | `asmc_msgs/LaneAssociation` | pose update, non-latched | 후륜축 중심 현재 허용 차로와 경계/이탈/전환 상태 |
| `/hd_map/debug/markers` | `visualization_msgs/MarkerArray` | 1 Hz, non-latched | 차선·경로·link 방향·station·체크포인트·미션·build 정보 |

P2-B v6의 `HdMapMeta.operational=true`, `complete=true`, `lane_count=70`,
`drivable_area_count=102`가 현재 기준이다. `lane_marking_count=128`은 물리 차선 도색 121개와
정지선 7개를 같은 `LaneMarking` 계약으로 전달하기 때문이다.

### 2.1 판제의 권장 소비 방식

판제의 정본은 위치에 맞춰 잘라서 보내는 local 토픽이 아니라, 시작 시 한 번 수신하는
latched global 토픽이다. 현재 vector 규모는 Lane 70개, DrivableArea 102개, Marking 128개로
작기 때문에 consumer가 `map` 좌표 polygon/centerline의 공간 인덱스를 한 번 만들고 Ego의
현재 map XY로 조회하는 방식이 가장 단순하고 결정적이다.

| 판제 목적 | 정본 입력 | 사용법 |
|---|---|---|
| 차로·차로 번호·연결 관계 | `global/lanes` | polygon/centerline 공간 조회 후 predecessor/successor 및 adjacency 사용 |
| 연속 주행가능영역 | `global/drivable_areas` | 모든 vector polygon의 union을 collision/corridor 정본으로 사용 |
| 빠른 주행가능 여부 조회 | `global/drivable_grid` | map XY를 0.2 m cell로 변환; `0=drivable`, `100=non-drivable` |
| 도색 색·실선/점선·폭 | `global/lane_markings` | `color`, `pattern`, `width_m`, `role`을 직접 사용 |
| 미션·속도·인지 활성 정책 | `global/mission_zones` | 현재 link ID 또는 route station이 zone 범위에 드는지 consumer가 판정 |
| 체크포인트 | `global/checkpoints` | pose, yaw, 3 m 반경, route/link station 사용 |

`local/lanes`와 `local/drivable_grid`는 bounded lookup 또는 디버그를 위한 선택적 파생
캐시다. planner horizon을 뜻하지 않으며 local 메시지만으로 미션·체크포인트·도색을
판단하면 안 된다. 기본 local 창은 ego heading 기준 전방 90 m·후방 30 m·좌우 각 30 m다.
전방을 위로 표시한 BEV에서는 후륜축 중심이 화면 폭의 50%, 아래에서 25% 위치에 온다.
RViz의 카메라 중심 설정과 grid 창은 별개이며, `follow_ego` 뷰는 계속 base_link를 따라간다.
`local_crop_mode:=map_square local_extent_m:=120`이면 기존 map 축 정사각 크롭을 사용한다.
운영 `competition_map.launch`의 크롭 기본값은 꺼짐이다. `publish_local:=true`로 켠
로컬 크롭 주기는 최대 20 Hz이고 해당 토픽 구독자가 있을 때만 계산한다.
현재 차로는 grid 계산을 기다리지 않고 각 유효 pose callback에서 즉시 발행한다.
pose가 0.5 s 이상 만료되면 현재 차로는 `valid=false/STATE_STALE`로 발행하고 크롭은 중단한다.
소비자는 크롭의 원래 pose stamp를 확인해야 하며 마지막 수신 데이터를 계속 새 데이터로 취급하면 안 된다.

`rostopic list`에 보일 수 있는 `/hd_map/global/drivable_grid_updates`는 RViz Map display가
증분 갱신을 위해 구독하는 표준 보조 이름이며 이 패키지의 발행 계약이 아니다. publisher가
없고 판제는 구독할 필요가 없다.

### 2.2 선택 구독과 규칙 조회

`hd_map_consumer_example.py`의 `~layers`로 `lanes`, `areas`, `markings`, `checkpoints`, `missions`
중 필요한 레이어만 구독한다. 기본은 `[lanes, missions, markings]`이고 22.3 MB 전역 grid를
받을 필요가 없다. `SemanticMapIndex(meta, layers)`는 일치하는 build/frame/geometry만 받아
배열과 bounds, link lookup, 파싱한 미션 payload를 한 번 캐시한다. ROS에 의존하지 않는
조회 API이므로 판제 프로세스 안에서 사용하면 매 제어 주기의 ROS 왕복이 없다.

```python
from asmc_hd_map.semantic_query import SemanticMapIndex
from asmc_hd_map.local_crop import EgoWindow

index = SemanticMapIndex(meta, {"lanes": lanes, "missions": missions, "markings": markings})
nearby = index.spatial((ego_x, ego_y), ego_yaw, EgoWindow(), layers=["lanes", "markings"])
speed = index.speed_limit(current_link_id)  # None이면 경로 밖/정보 미수신: 무제한이 아님
missions_now = index.active_missions(current_link_id, route_station_m)
signals = index.signals_ahead(route_station_m, distance_m=120.)
```

- 제한속도: 속도 zone의 `rule_payload_json.route_speed_limits`에 공식 경로 링크 39개의
  `mgeo_max_speed_kph`, `competition_cap_kph`(없으면 null), `effective_limit_kph`를 전달한다.
  유효 상한은 둘의 최솟값이다. A202/A315는 70→60 km/h이며 고속 예외는 대회 상한만 없애므로
  MGeo 상한은 유지된다. 판제 SI 입력에는 `effective_limit_kph / 3.6`을 쓴다.
- 주행구간: `active_missions(link_id, route_station_m)`는 부분 링크 구간을 station으로 좁히고
  provisional을 제외한다. station을 생략하면 링크에서 가능한 구간 후보이므로 실제 활성화에는
  station을 함께 사용한다. HD 공식 station은 `/hd_map/context`로 제공한다. 판제 자체 경로의 station은 경계 좌표를 재투영한다.
- 신호등: 인지 활성 구간과 실제 정지선 경계 5개를 구분한다. `signals_ahead`는 길이가 0인
  경계도 route distance로 조회하고 `traffic_light_id`, 정지선 ID 및 선택한 도색 레이어의
  정지선 geometry를 반환한다. 총 7개 검수 정지선은 `LaneMarking.ROLE_STOP_LINE`이다.
  신호등 색·현재 현시는 인지 또는 simulator state와 ID로 결합한다. 정적 지도에는 현시가 없다.
- spatial 조회는 bounds의 보수적 교차 검사다. 잘린 polygon이 아닌 원본 객체를 돌려주므로
  collision/정확한 ROI 교차는 소비자가 polygon으로 판단한다. 곡선 앞쪽의 규칙은 공간 크롭에
  가두지 말고 route station과 topology로 조회한다.

### 2.3 크롭 거리 선정

90/30/30/30 m는 이 맵의 로컬 노면 조회·시각화 기본값이다. 전방은 기존 방향 무관 60 m보다
늘리고 후방은 차로 전환·이탈·교차로 연속성 확인을 위해 남긴다. 좌우 30 m는 주변 차로와
교차로를 포함하는 cache 폭이며 계산/전송 셀 수는 360,000→180,000으로 줄어든다.
`local_front_m`, `local_rear_m`, `local_left_m`, `local_right_m`를 독립적으로 조절할 수 있다.

판제 감속 horizon은 `v * response_time + v² / (2 * deceleration) + margin`과 경로 곡률·상한을
고려해 별도로 잡는다. 예를 들어 설계 가정 response 0.5 s, deceleration 3 m/s², margin 10 m이면
60 km/h에서 약 65 m, 80 km/h에서 약 103 m, 90 km/h에서 약 127 m다. 이 수치는 차량에서
측정한 제동 성능이 아니라 거리 검토용 가정이다. 고속 운용 시 local 전방을 120~150 m로
늘릴 수 있지만, 원거리 신호등/속도 전환은 항상 전역 경로에서 필요한 horizon까지 조회해야 한다.
후진 시에는 앞뒤 창을 바꾸거나 `map_square`를 선택한다. BEV 카메라 화면의 crop을 그대로
판제 planning horizon으로 채택하지 않는다.

전체 지도와 규칙 연결을 캐시하고 조회하는 구조는 [Autoware map loader](https://autowarefoundation.github.io/autoware_core/latest/map/autoware_map_loader/)와
[Lanelet2 regulatory elements](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/RegulatoryElementTagging.md)의
역할 분리를 참고했다. 위 거리 기본값은 해당 프로젝트의 권장 상수가 아니라 이 패키지의 설계 선택이다.

### 2.4 발행 지연 검증 (2026-10-06)

ROS Noetic 컨테이너, 별도 test master, `ego_status` 입력 20 Hz × 100 pose에서 측정했다.
각 pose stamp의 최초 도착만 집계하므로 stale 감시 timer의 재발행을 지연 표본으로 세지 않는다.

| 항목 | 표본 | p50 | p95 | 최대 |
|---|---|---|---|---|
| 현재 차로 pose→수신 | 100 | 1.49 ms | 2.33 ms | 3.63 ms |
| local grid pose→수신 | 100 | 27.41 ms | 53.64 ms | 72.01 ms |
| 선택 vector/규칙 내부 조회 | 500 | 0.086 ms | 0.137 ms | 0.465 ms |

이 값은 해당 호스트 시험 결과이며 실차의 무지연/실시간 보장은 아니다. grid에는 최대 한 주기의
timer 대기와 계산·전송 시간이 있다. 중요 규칙은 전역 cache의 내부 조회와 pose callback 차로를
사용한다. ROS master 연결·파일 SHA 검사·전역 배열의 최초 로딩 시간은 steady-state 지연에
포함하지 않았다. GPS/IMU 동기화, 실제 센서·판제 부하와 폐루프 주행은 별도 검증이 필요하다.

재현 절차는 **실차/주행 master와 분리된** 개발 shell에서만 사용한다. 검증기가 합성 pose를 보낸다.

```bash
export ROS_MASTER_URI=http://127.0.0.1:11378
export ROS_IP=127.0.0.1
roscore -p 11378
# 같은 환경변수를 둔 다른 shell
roslaunch asmc_hd_map competition_map.launch rviz:=false local_pose_source:=ego_status
# 같은 환경변수를 둔 다른 shell
rosrun asmc_hd_map verify_competition_map_topics.py
rosrun asmc_hd_map verify_local_map_topics.py
rosrun asmc_hd_map verify_hd_map_latency.py
```

전체 HD맵 unit test 150개와 추가 ROS pose/직렬화 회귀 5개, 전역·로컬 토픽 smoke,
기존 message serialization, P1-B MD5 계약 검사와 catkin workspace build를 통과했다.
geometry/metadata artifact와 pin은 그대로다. 기존 정사각 모드는
`local_crop_mode:=map_square`와 verifier `_crop_mode:=map_square`로 확인한다.

### 2.5 필드 해석과 알려진 의도적 공백

- 모든 global 배열은 같은 `build_id`/`geometry_sha256`를 사용한다. 하나라도 다르면 함께
  사용하지 않는다.
- `Lane.route_station_*=-1`은 전역경로 자체가 아닌 동일방향 인접차로·미션 회피차로라는
  뜻이다. 이 차로는 `link_id`, polygon과 mission `link_ids`로 조회한다.
- `MissionZone.polygon`이 비어 있으면 오류가 아니라 `route_station_*`과 link 범위가
  authoritative하다는 뜻이다.
- `MissionZone.status=PROVISIONAL`은 시각화·후보 policy에는 쓸 수 있지만 강제 판제 규칙으로
  승격하면 안 된다. 현재 15개 zone은 모두 `DERIVED_OFFICIAL`로 발행된다. 공식 규칙에서 계산된
  4개와, 2026-10-05 팀 검수 기준값 11개(`rule_payload_json.authority=team_final`, 수정 가능)다. 메시지가
  동결돼 team-final enum은 없으므로 근거 수준은 payload로 구분한다.
- 혼합 source의 polyline 방향을 억지로 정규화하지 않으므로 `LaneMarking.left_lane_ids`와
  `right_lane_ids`는 현재 비워 둔다. 판제의 정확한 경계 관계는 `Lane.left_boundary_id`와
  `Lane.right_boundary_id`이고, 필요하면 이 필드로 marking→lane reverse index를 만든다.
- 물리 도색의 짧은 gap은 `LaneMarking`에 그대로 남지만, 주행 연속성은 `Lane.polygon`,
  topology와 `DrivableArea` union으로 전달된다.
- `DrivableArea.validity=VALIDATED`인 polygon은 `source_type`이 `LANE_CLIPPED`든 `HYBRID`든
  동일한 통행 가능 면이다. `source_type`, `source_dataset`, `source_feature_id`는 생성 근거를
  추적하는 provenance이지 planner cost나 우선순위가 아니다. 판제는 validated polygon 전체를
  union하거나 동일한 값 `0`으로 합쳐진 `OccupancyGrid`를 사용한다.
- surface patch/corridor fill은 빈 노면을 보완하지만 새 차로를 만들지는 않는다.
  `/hd_map/local/current_lane`은 계속 `Lane.polygon`과 topology로 차로 ID를 판정하고,
  `DrivableArea`/grid는 collision·주행면 판정에 사용한다. 즉 같은 통행 가능 면이면서도 차로
  번호를 임의로 부여하지 않는 의도적인 역할 분리다.

local 토픽은 2단계 consumer handoff에서 활성화했다. 기본 입력은 `/gps`와 `/imu`이며,
WGS84→UTM52N, 맵 원점 `(302595, 4124145)m`, GPS 전방 lever arm 0.35 m를 적용해 map XY를
계산한다. 대회판에서 관측한 `/Ego_topic.position=(0,0)`은 위치원으로 사용하지 않는다.
센서 offset과 맵 origin 차이가 1 m를 넘으면 발행을 거부한다.
`local_pose_source:=pose_stamped local_pose_topic:=/localization/pose`로 정규화된 localization
pose를 사용할 수도 있다. 두 출력의 geometry는 `base_link`로 변환하지 않고 계속 `map`
frame이며 Lane ID도 global namespace를 유지한다. Lane geometry 자체는 자르지 않고 polygon
또는 centerline bounds가 crop과 교차하는 Lane 전체를 선택한다. `LaneArray.complete=false`가
subset임을 나타낸다. 기본 grid는 output cell 중심을 global grid에 최근접 표본화하며 원본
`0/100/-1`을 보간하지 않는다. global 범위 밖은 `-1`이다. `header.frame_id=map`이며 grid origin의
quaternion이 ego yaw를 담는다. map XY에서 grid cell을 구하려면 origin translation을 빼고
origin rotation의 역회전을 적용한 뒤 resolution으로 나눈다. vector Lane은 이미 map 좌표이므로
ego transform을 다시 적용하지 않는다. `map_square` 모드만 원본 global cell 경계에 정렬된다. 판제가 global latched 데이터를 한 번 받아 자체 ROI index를 만드는 방식도
계속 허용하는 정도가 아니라 권장 기본 방식이다. P2-B 정책과 결과는
[`kcity2025-compact-drivable-map-v1.md`](../records/hd-map/kcity2025-compact-drivable-v1.md)에서 관리한다.

개발 환경에서 local/global parity는 테스트용 `ego_status` 입력을 명시한 별도 실행에서
확인한다. 검증기는 START 좌표의 map XY를 한 번 발행하므로 실제 주행 중에는 실행하지 않는다.

```bash
roslaunch asmc_hd_map competition_map.launch local_pose_source:=ego_status
# 다른 shell
rosrun asmc_hd_map verify_local_map_topics.py
```

## 3. 노드 부근의 짧은 차선 단절을 판제가 받는 방식

`LaneMarking`은 도로의 통행 가능성을 직접 표현하는 선이 아니라 **실제 도색 하나의 관측된
polyline**이다. 따라서 현재 stage14 geometry에서 노드 주변 선이 잠깐 끊겼다면 publisher는
그 끝점을 자동으로 붙이지 않고 두 개의 독립 polyline 또는 빈 구간으로 그대로 전달한다.

이 정책이 필요한 이유는 다음과 같다.

- 실제로 도색이 없는 노드·교차로 구간을 임의의 실선으로 만들지 않는다.
- 점선의 gap을 실선으로 메우지 않는다.
- 가까운 두 끝점을 자동 연결하면서 다른 차로 또는 분기선을 잘못 잇지 않는다.
- geometry 오류와 물리적으로 도색이 없는 구간을 검수 단계에서 구분할 수 있다.

판제는 `LaneMarkingArray`만으로 주행 corridor를 만들거나 endpoint를 자동 연결하면 안 된다.

P2-B는 물리 도색과 별개로 다음 데이터를 발행한다.

```text
Lane.centerline + Lane.polygon
Lane.predecessor_lane_ids / successor_lane_ids
Lane.left_boundary_id / right_boundary_id
DrivableArea vector polygon + OccupancyGrid
```

따라서 노드에서 도색이 잠시 없더라도 predecessor/successor와 lane polygon으로
차로 연속성을 전달할 수 있다. 반대로 검수 결과 “끊기면 안 되는 실제 실선의 geometry 오류”로
확인된 곳은 publisher에서 임시 보간하지 않고 HD Map 원본을 수정한 뒤 geometry hash를 새로
승인해야 한다.

교차로도 반경이나 전체 road mesh로 확장하지 않는다. 일반 교차로는 route link만, 장애물
교차로는 config의 reviewed link만 폭 polygon으로 만든다. 동적 장애물 교차로는 anchor 4개의
시작 노드 집합에서 끝 노드 집합으로 향하는 link를 닫아 총 8개 내부 link를 쓰며, 이 union은
단일 연결 주행면이다. 정적·동적 장애물 교차로는 이 선택 link envelope 내부의 H5 road만
gap patch로 더하며, 둘 다 내부 hole 0개의 단일 연결면으로 검증한다. 정적 장애물 교차로의
우회전 `A226`·`A111`은 모서리 안쪽 포장면이 3.5 m 폭 밖에 있어, config의
`intersection_link_widening`에 따라 오른쪽을 각각 최대 1.0 m·1.5 m 넓힌다(양 끝 노드 0인
half-sine). 근거는 공식 경로 그림을 전역경로에 정합한 측정값이며, H5와 관측 차선은 이
모서리를 덮지 않는다. 일반도로는 reviewed
marking의 유효한 국소 횡거리로 Lane 외곽을 보정하고 MGeo 인접 차로 사이를 corridor fill로
연결한다. 각 Lane polygon은 round cap으로 노드를 넘어가지 않고 노드 법선의 flat cap으로
끝난다. 이 link들은
`DrivableArea.intersection_guarded=true`로 전달되지만, 이는 공간 coverage이며 policy 밖 경로
이탈 허가는 아니다.

## 4. RViz 표시

`competition_map_debug.rviz`는 `map` frame에서 다음 namespace를 제공한다.

- 흰색/노란색 실선·점선과 청록색 정지선
- 공식 전역경로, 100 m station, link ID와 진행 방향
- 체크포인트 3 m 원, 공식 heading, route station
- 미션 종류별 namespace. 각 종류를 RViz `MarkerArray`의 namespace 체크박스로 따로 켜고 끈다.

  | 종류 (색) | 전역경로 span | 구간 면적 외곽선 | actor 후보 링크·방향 화살표 |
  |---|---|---|---|
  | 출발 5% (연두) | `mission_start_success` | — | — |
  | 정적 장애물 (주황) | `mission_static_obstacle` | `mission_surface_static_obstacle` | `mission_actors_static_obstacle` |
  | 동적 장애물 (자홍) | `mission_dynamic_obstacle` | `mission_surface_dynamic_obstacle` | `mission_actors_dynamic_obstacle` |
  | GPS 음영 (보라) | `mission_gps_shadow` | `mission_surface_gps_shadow` | `mission_actors_gps_shadow` |
  | 회전교차로 (파랑) | `mission_roundabout` | `mission_surface_roundabout` | `mission_actors_roundabout` |
  | 주변 차량 (연두) | `mission_surrounding_vehicle` | `mission_surface_surrounding_vehicle` | `mission_actors_surrounding_vehicle` |
  | 신호 정지선 | `mission_boundaries` (station 점) | — | — |

  - 구간 면적 외곽선은 점선이라 실선 차선과 구분된다. 미션 색에는 차선 색(흰색·노랑)을 쓰지 않는다.
  - actor 후보 링크는 `full`에서만 보인다. 선 색은 정적 장애물 빨강, 동적 장애물 분홍, NPC 차량 흰색이고 화살표는 진행 방향이다.
  - 미션 라벨은 `종류 [DER|TEAM]`, station 범위, actor 링크 요약이다(예: `ROUNDABOUT [TEAM] 859–915 m · NPC A147/A133/A134`).
  - 면적과 actor는 [미션 시나리오 spec](../handoffs/hd-map-mission-scenario.md)에서 publisher 시작 시
    매번 만든다. spec을 만들 수 없으면 이 두 레이어만 빠지고 경고가 남는다.
- 청록색 60 km/h 구간과 자홍색 `COMP LIMIT NONE` 고주로 속도 예외
- 연두색 `A236→A308` 신호등 인지 활성화 policy
- build ID, geometry hash 앞 16자리와 P2-B product count
- 0.2 m OccupancyGrid와 cyan Lane polygon/centerline
- route-only connector(파랑), 정적 장애물(주황), 동적 장애물(자홍), 일반 우회전(연두), 음영 반대차로(보라)
- A134 로터리·A151 접속 surface(밝은 파랑), 인접차로 corridor fill(노랑), 그 밖의 H5 patch(주황) outline

### 4.0 Ego 차량 박스와 base_link 추적

`local_map_crop_node`는 현재 차로 판정과 같은 pose(`/gps`+`/imu`, GPS 안테나에서 0.35 m 뒤의 후륜축
`base_link`)로 `/hd_map/debug/ego_markers`를 발행한다. RViz의 `Ego footprint (base_link, GPS+IMU)`가 이를 표시한다.

- 박스: Ioniq 5 규정 치수로, `base_link` 뒤 0.79 m부터 앞 3.845 m까지, 폭 1.892 m다.
- 박스 색: 차체가 compact 주행가능 grid 밖으로 나간 비율에 따라 초록(0%), 노랑(10% 미만), 빨강(그 이상)이다. pose가
  0.5 s 넘게 갱신되지 않으면 회색 `GPS STALE`이다. 대회 음영 구간처럼 GPS가 (0, 0)으로 오면 마지막 pose에서
  멈추고 회색이 된다.
- 모서리 점은 각 모서리가 주행가능 칸 위인지(초록/빨강), 파란 점은 `base_link`다.
- 텍스트: map 좌표, yaw, 현재 차로 link·상태·횡오프셋, 주행가능영역 밖 비율.
- `publish_ego_tf`(debug launch에서 `rviz`와 같은 기본값)가 켜지면 같은 pose로 `map→base_link` TF를 낸다.
  이때 브리지의 `publish_map_tf`는 끈 상태로 둔다. 브리지 TF는 GPS 안테나를 `base_link`로 써서 0.35 m 어긋난다.
- RViz는 기본으로 `base_link`를 따라간다(`follow_ego` 기본값 `true`). 전체 경로 보기로 띄우려면
  `follow_ego:=false`를 준다. 실행 중에는 오른쪽 **Views** 패널에서 바꾼다.
  - **Saved** 목록의 `Follow ego (base_link)` 또는 `Competition route overview (upright text)`를 더블클릭하면
    그 시점이 현재 시점이 된다.
  - 또는 **Current View**의 `Target Frame` 값을 `base_link`(따라가기)나 `map`(고정)으로 바꾼다.

### 4.0.1 표시 수준과 부하

`debug_marker_detail` 기본값 `lite`는 차선 도색, 병합한 주행가능영역 외곽선(`drivable_area_outline`),
체크포인트, 속도·미션 레이어만 보낸다. `full`은 차로별 polygon·centerline, 링크 화살표·라벨, station
눈금, corridor fill·H5 patch·교차로 외곽 같은 HD맵 제작 검수용 레이어를 더한다.

| 항목 | full | lite (기본) |
|---|---|---|
| `/hd_map/debug/markers` | 478 marker, 선 점 17.7만 개, 4.3 MB | 86 marker, 3.6만 개, 0.9 MB |
| actor 후보 링크(`mission_actors_*`)와 라벨의 링크 목록 | 표시 | 숨김 |
| 주행가능영역 표시 (RViz 기본) | full 상세 레이어 | 병합한 전역 외곽선, raster 면 비활성 |

marker는 시작할 때 한 번 만들어 두고 새 구독자가 붙을 때만 보낸다. 고정된 내용을 다시 보내면 RViz가 매번
다시 만들 뿐이라 주기 전송은 기본으로 끈다(`debug_republish_hz` 0). RViz에서 Reset을 누르면 marker가
지워지므로, 해당 display 체크를 껐다 켜면 다시 받는다. RViz가 없으면 아무것도 보내지 않는다.

운영과 debug launch 모두 크롭을 기본으로 끈다. 두 RViz 프리셋에서
`Drivable area near ego` display를 제거했고, 기본 주행가능영역은 전역 외곽선 marker로 확인한다.
전체 raster 면이 필요한 검수에는 비활성 display인
`Drivable area whole map (0.2 m planner grid, heavy)`(22.3M칸)를 수동으로 켠다.
차선 도색과 외곽선은 고정 marker라 RViz가 한 번 올린 뒤에는 거의 부하가 없어 따로 크롭하지 않는다.
선택 크롭 기능은 `publish_local:=true`로 켤 수 있다. 별도 검수에서 필요한 경우에만
RViz Map display를 추가해 `/hd_map/local/drivable_grid`를 지정한다.

미션 면적과 전체 주행가능영역 외곽선은 차로 polygon의 합집합이다. 인접 차로 사이 미세한 틈이 만드는
조각 고리를 없애기 위해 0.2 m closing을 하고, 5 m² 미만 구멍은 버린 뒤 0.1 m로 단순화한다. 벡터 합집합
면적과의 차이는 0.1% 수준이다.

점선은 message의 `pattern`과 dash/gap 속성으로 시각화한다. polyline point 간격을 보고
실선/점선을 추정하지 않는다.

### 4.1 개발용 HD map source 비교

최종 검토 차선, 구 맵 origin 정합 차선, 원태 H5 vector, 최신 compact 주행가능영역과
자르기 전 구 H5 road bake는 운영 publisher와 분리된 다음 개발 launch로 확인한다.

```bash
roslaunch asmc_hd_map hd_map_source_comparison.launch repo_root:=/root/ws
```

| 레이어 | 개발 전용 토픽 |
|---|---|
| 최종 검토 차선 | `/asmc_hd_map/development/hd_map_sources/lanes/final_curated` |
| 구 맵 origin 정합 차선 | `/asmc_hd_map/development/hd_map_sources/lanes/legacy_origin_aligned` |
| 원태 H5 원본 vector | `/asmc_hd_map/development/hd_map_sources/lanes/won_tae_h5` |
| 최신 compact 주행가능영역 | `/asmc_hd_map/development/hd_map_sources/drivable/latest` |
| 구 H5 road bake | `/asmc_hd_map/development/hd_map_sources/drivable/legacy_h5_bake` |

차선 세 source는 별도 `MarkerArray`, 주행가능영역 두 source는 별도 `OccupancyGrid`로
발행되며 자동 병합되지 않는다. RViz preset은 최종 차선과 최신 영역만 기본 활성화하고
나머지는 수동으로 켜도록 구성했다. 이 개발 launch와
`development/won_tae_link_node/`의 보존 자료는 판제 runtime launch에 포함되지 않는다.

고정 TopDownOrtho 프리셋은 화면 기준으로 글자가 바로 보이도록 180도 회전을 명시한다.
텍스트는 원본 메타데이터를 바꾸지 않는 debug 전용 축약 표기다. 예를 들어
`A2256W000846`은 `A846`, 상태는 `OFF/DER/PROV`로 표시된다. 링크, checkpoint, mission label은
공식 route의 국소 법선을 기준으로 서로 다른 측면에 배치하고, 이미 놓인 글자와 가까우면
다음 offset 후보로 옮긴다. station tick은 100 m마다 유지하되 숫자 label은 200 m마다 표시한다.
따라서 정확한 full ID, station 소수값과 설명은 해당 latched metadata topic을 정본으로 사용한다.

미션 색은 상태가 아니라 종류를 나타낸다. 상태(`DER`/`TEAM`/`PROV`)는 라벨로 읽는다. 신호등 인지 활성화 policy는
미션 corridor와 분리한 `perception_activation_corridors`/`policy_text`로 표시한다. 범위는 A236
시작부터 A308 종료까지이며 밖에서는 신호등 노드를 중지할 수 있다는 hint를 제공한다.
A866·A054·A846은 일반 전역경로 추종 구간이며, 이 구간에 보이는 청록색은 미션이 아니라
60 km/h 속도 profile이다.

## 5. 검증 기준

컨테이너 smoke validator는 늦게 접속해 latched 전역 토픽을 실제 구독하고 다음을 검사한다.

- 정확한 topic type 11개(global/meta/debug 8개 + local 3개)
- local topic 3개의 등록과 별도 smoke 검사에서 global/local lane·grid parity 및 현재 차로 상태
- geometry SHA `01ec671f…4bcfbf`
- marking 128개 = lane marking 121개 + stopline 7개
- Lane 70개/vector area 102개, A432·A131·A134 포함/A427·A179·A1212 제외
- non-route 교차로가 reviewed set 15개와 정확히 일치
- 동적 장애물 node-pair closure 8개와 파생 link `A066/A087/A211/A1125`
- 인접차로 fill 18개, H5-검증 patch 14개, 정적·동적 면의 단일 연결성과 내부 hole 0개
- A222 교차로 모서리 probe `(-54.38,-131.38)`·`(-55.07,-149.22)`=0, 그 바깥 probe=100
- A134 로터리·A151 접속부의 단일 연결성, H5 기반 2 m envelope 확장과 중앙 섬 비주행 보존
- 음영구간 `A126/A131` bridge와 일반도로 reviewed-boundary refinement
- grid 0.2 m, `3085×7232`, 값 `0/100`, published boundary ID의 marking 참조 무결성
- checkpoint 15개, 반경 3 m, route 2184.611723336 m
- mission/policy/speed zone 15개와 발행 상태 `OFFICIAL 0 / DERIVED_OFFICIAL 15 / PROVISIONAL 0`, 그중 `authority=team_final` 11개
- 정적 장애물 구간 `A213` 중간→`A220` 끝(장애물 `A222`), 동적 `A202` 중간→`A315` 30%(장애물 `A728→A599→A207→A205`)
- `60→상한 없음→60` 세 속도 구간의 전역경로 완전·비중첩 coverage
- A054·A846 미션 제외, 신호등 활성화 policy `A236→A308`, 정지 경계 5개의 policy 내부 포함
- RViz marker namespace와 최소 marker 수
- link label 39개, checkpoint label 15개, 축약 ID 사용과 text 중심 최소 간격 9 m

## 5. 판제 선택 소비와 GPS 음영 처리

운영 기본 실행은 전역 레이어와 `/hd_map/global/route`, `/hd_map/context`를 제공한다.
크롭 노드는 유효 상태·현재 차로 조회를 계속 수행하지만 기본으로 local lane/grid publisher와
전역 grid 구독을 만들지 않는다. RViz·ego marker·local crop을 켜면 필요한 grid를 받아 사용한다.

`HdMapRoute`의 points와 stations는 공식 path의 map 좌표와 XY 누적거리다.
`HdMapContext`는 후륜축 ego pose, 별도의 lane/route valid, route 상한(m/s), 활성 mission ID,
기본 후방 30 m·전방 200 m의 미션 경계와 좌표를 제공한다. `~boundary_ahead_m`과
`~boundary_behind_m`은 context 조회 창이며 전역 정보를 제한하지 않는다.
`SemanticMapIndex.mission_boundaries(s, ahead_m, behind_m)`로 더 먼 구간도 조회한다.

명시적 GPS 무효/status≤0 또는 좌표 sentinel은 즉시 현재 정보를 무효화한다. GPS 미수신은
source stamp와 receipt 기준 0.5초 후 무효다. 유효 위치 추정·누적거리 외삽은 하지 않는다.
마지막 유효 관측의 구간·음영구간은 `last_valid_*`와
`gps_shadow_context_from_last_valid_pose=true`로 구분하며 현재 위치라고 사용하지 않는다.
GPS 복귀 시 새 관측으로 차로와 route를 재획득한다. 음영에서 camera lane/drivable 소비,
신호 색상, 속도 정책과 판제 자체 경로의 station 재투영은 판제가 반영한다.

실행·파일 export·현재 main의 차이·판제 인수 검증은
[판제 인수 문서](../handoffs/hd-map-to-planning.md)를 따른다.
