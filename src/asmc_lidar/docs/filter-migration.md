# 판제 인지 필터 편입 목록

PR #108 이후 리팩토링의 입력 목록이다. **이번에는 소유권·위험·완료 조건만 정리하며 기존 Main과
planning_inputs의 실행·기본값·판제 입력을 변경하지 않는다.** 전체 전환은
[LiDAR 인계](../../../docs/handoffs/lidar-migration.md#판제-소비-전환-시-정리할-중복-처리)를 따른다.
안승현은 지도·카메라 연결을, 손재호는 추적·속도·운동 상태·matching을 담당한다.

## 현재 호출 경계

| 경로 | 실제 처리 |
|---|---|
| 기존 Main `PlannerObjectRosPublisher.cpp` | road → known-static → configured structure → PlanningRelevance → OBB clipping → `/last_lidar` |
| `planning_inputs`의 `objects_node.cpp`·`PlanningObjectFilter.cpp` | 선택 입력 변환·private MotionClassifier, 설정된 경로에서 road/known-static/structure → clipping → 판제 배열 |
| 새 `asmc_lidar` | 군집·OBB·추적·카메라 fusion → 원본 배열/판정 → HD맵·상부 전달 필터. 발행 배열만 제외하며 tracker/원본 측정은 유지. Main·판제 입력은 기존 경로 |

동명 필터가 Main과 planning_inputs 양쪽에 있다. 두 경로가 한 실행에서 모두 적용된다고 가정하지
않고 입력 선택·launch·활성 파라미터를 먼저 기록한다. 코드 중복과 실행 중 이중 적용은 구분한다.
정본 원본 이름·그룹·속도 validity가 구형 enum/wire 변환에서 소실되는지도 함께 확인한다.

## 편입·분리 결정

| 기능·현재 위치 | 목표 소유권·구현 위치 | 그대로 복사하면 안 되는 부분 | 다음 PR 완료 기준 |
|---|---|---|---|
| `DrivableBoundaryFilter`·`DrivableFootprint`(Main 및 planning_inputs) | 지도와 관측 객체의 물리적 관련성 계산은 `asmc_lidar/filtering` | 구 도로 mask·기본0.15·보행자 예외를 현재 v7/10cm/점군3점·10% AND OBB10%로 무조건 섞지 않음 | 같은 좌표·stamp의 원본 점군/footprint·원래 지도/현행 지도·임계값을 분리 대조하고 지도/pose 무효 시 local 보존 |
| `KnownStaticObjectFilter`(양쪽) | 검증된 고정 시설물과의 일치 근거는 `filtering` | 현재 기본 중심2.5m 이내·UNKNOWN-only·구간 bounds·확정 클래스 보호. 중심 거리만으로 실제 주차 차량도 지울 수 있어 catalog를 자동 이식하지 않음 | 원본 catalog SHA·생성 조건·현재 맵/보정 일치·실제 장애물 보존을 검수한 항목만 후보 근거로 제공 |
| `StaticStructureExclusionFilter`(양쪽) | 구조물의 위치·형상·충돌 높이 근거는 `filtering` | UNKNOWN-only·motion 보호와 고정 map bounds, 높이2.5m/하단2m 기본값. map Z와 base/지면 상대 높이를 혼동하지 않음 | 수평화/장착 변환을 한 번 적용하고 관측 하부·상부만 보이는 큰 차량·음영/GPS 무효를 대조. YOLO 그룹과 운동 상태 분리 |
| `PlanningRelevanceFilter`의 지면 파편/높이 근거 | 측정 품질 부분만 `detection/filtering` | OBB Z span이 작아도 물체가 지면 위에 떠 있는 관측일 수 있음. 현재 ground_evidence의 median clearance·rejection band·표본 수를 함께 봄 | 낮은 장애물·얇은 단일 ring·노면 파편·기울어짐·가림에서 원래 근거와 판단을 재현 |
| 같은 클래스의 관측 확인·소실 완충 | 물리적 track 생애주기/관측 확인은 `tracking`에서 재호가 검토 | 기존 relevance의 good/bad height3/5회·짧은 dropout·공간 일치와 tracker의 confirm/miss가 중복될 수 있음 | session/generation/ID별 bounded 상태로 합의하고 실제 관측 없는 재발행·무효를 정상0으로 바꾸지 않음 |
| `MotionClassifier`·속도 급변 gate | 객체 속도·실제 운동 추정은 `tracking`에서 재호가 소유 | `objects_node`의 private classifier가 known일 때 우선되고 unknown이면 upstream을 쓰는 현재 분기, visible face 변화·yaw/heading·warmup·10km/h jump 등을 전체 이식하지 않음 | 같은 GT/기준점·단위·시간·자차 보상에서 위치/속도/ID·반응 지연을 검증한 공용 validity로 제공 |
| 도로 밖 객체의 3초 진입 예측(`DrivableBoundaryFilter`) | 물리적 궤적 근거는 추적/인지 후보, 어떤 위험을 회피할지는 판제 | 현재0.5초×6회 map 속도 외삽이 검증되지 않은 속도에 의존할 수 있음. motion이 dynamic이라는 이유만으로 보호하지 않음 | 속도 validity·불확실성·예측 범위가 확인되기 전 보류. 현재는 유효 YOLO vehicle/dynamic 보호 유지 |
| OBB clipping·`BuildPlanningObject` | 측정 OBB를 자르는 구현은 새 인지 정본에 편입하지 않음 | 중심·8코너·yaw·크기가 원본 측정과 달라지며 속도/투영/융합에도 영향을 줄 수 있음 | 인지는 원본을 제공. 판제가 별도 충돌 영역을 필요로 하면 측정 형상과 구분해 소비 경계에서 생성 |
| `PlanningRelevanceFilter`의 근접 긴급 보존 | 판제에 유지 | ego 속도·차량 길이로 계산하는 emergency distance·확인 횟수 우회는 행동 위험 정책 | 인지 품질 근거를 받아 판제에서 보수적으로 소비. TTC·정지·추종·회피·재출발과 함께 검토 |
| `objects_node`·`converter`의 wire/유효성 검사 | 생산자 측 수치/좌표 정합성은 인지, 수신 어댑터·age/session 검사는 판제에도 유지 | 원본 enum·16종/3그룹·관측/예측·local/map 유효성·헤더를 잃으면 필터가 UNKNOWN을 오해할 수 있음 | 공용 `asmc_msgs` 계약으로 변환을 검증하고 센서 장애/정상 빈 결과·stale·재시작을 구분 |

`PlanningRelevanceFilter`는 지면 품질과 ego 위험 정책이 섞여 있어 파일 전체를 인지로 옮기지 않는다.
현재 pedestrian/dog·known vehicle의 즉시 보존도 각 소비 경로의 원본 분류와 확인 정책이 다르다.
최신 양성 매칭·확정 분류·추정 운동 상태를 서로 대체하지 않는다.

기존 road의 무조건 클래스 보호는 person/pedestrian이고, known-static의 raw label 보호는
car/vehicle/truck/bus/person/pedestrian 일부다. relevance는 motorcycle/bicycle도 known vehicle로
묶는다. 현재 정본의 vehicle3종·dynamic_obstacle5종과 같지 않다. 각 필터의 조건을 합칠 때 dog/deer/
bicycle/stroller의 보호 누락이나 motorcycle의 그룹 변경이 생기지 않도록16종 표로 대조한다.

## 원본 위치

- Main: [발행 조합](../../lidar+sensorfusion/src/Main/src/PlannerInterface/PlannerObjectRosPublisher.cpp),
  [road](../../lidar+sensorfusion/src/Main/src/PlannerInterface/DrivableBoundaryFilter.cpp),
  [known-static](../../lidar+sensorfusion/src/Main/src/PlannerInterface/KnownStaticObjectFilter.cpp),
  [구조물](../../lidar+sensorfusion/src/Main/src/PlannerInterface/StaticStructureExclusionFilter.cpp),
  [relevance](../../lidar+sensorfusion/src/Main/src/PlannerInterface/PlanningRelevanceFilter.cpp).
- 판제 입력: [필터 조합](../../planning_inputs/src/filters/PlanningObjectFilter.cpp),
  [road](../../planning_inputs/src/filters/DrivableBoundaryFilter.cpp),
  [footprint](../../planning_inputs/src/filters/DrivableFootprint.cpp),
  [known-static](../../planning_inputs/src/filters/KnownStaticObjectFilter.cpp),
  [구조물](../../planning_inputs/src/filters/StaticStructureExclusionFilter.cpp),
  [운동 추정](../../planning_inputs/src/MotionClassifier.cpp),
  [입력/운동 조합](../../planning_inputs/src/objects_node.cpp),
  [변환 검사](../../planning_inputs/src/converter.cpp),
  [현재 기본 설정](../../planning_inputs/config/object_filter_params.yaml).
- 현재 분석·비교 정본은 [HD맵 필터 분석](hd-map-filter-analysis.md), 좌표/공통 IMU 리팩토링은
  [후속 설계](../../../docs/architecture/perception-coordinate-refactoring.md)를 따른다.

## 다음 PR 작업 카드

상태는 **미착수**다. PR #108의 표시 검수와 원본 발행 유지 상태를 기준선으로 고정한다.

1. **호출·설정 고정:** 두 기존 경로의 실제 입력 선택·필터 순서·설정/지도 SHA·단위·frame을 기록한다.
2. **공통 계산 추출:** road/구조물/품질의 ROS 독립 계산을 기존 geometry/filtering 경계에 배치한다.
   새 단계별 노드나 Main/Msg 빌드 의존성을 추가하지 않는다. 이전/신규 호출은 같은 코어를 사용한다.
3. **소유권 분리:** relevance의 품질과 긴급 위험 정책, tracking의 확인과 판제의 행동 완충을 분리한다.
   상태 이관은 재호와 확인하고 단위 변경과 임계값 변경을 같은 비교에 섞지 않는다.
4. **동일 입력 대조:** 정상 장애물·가드레일·터널 벽/천장·partial box·GPS5Hz 사이·GPS0/중단·복귀·
   카메라 miss/세대 전환에서 원본 OBB/ID·후보 사유·계산 비용·메모리 상한을 비교한다.
5. **적용 경계 검수:** 시험 namespace의 필터 전후 후보와 판제의 실제 입력을 별도로 검수한다.
   원본 추적 객체를 보존한다. 새 패키지의 HD맵·상부 전달 제외는 A1.9에서 활성화했다.
   판제 중복 호출 제거와 후속 편입 필터의 활성화는 공동 확인 후 별도 소비 전환 변경으로 제출한다.
   기존 Main·복구 경로를 먼저 삭제하지 않는다.

완료 근거에는 원본/새 코드 대응표·판정 차이·필요 객체의 보존·시간/좌표 유효성·성능·복구를 둔다.
리팩토링 완료를 운영 필터 수락이나 판제 인수 완료로 기록하지 않는다.
