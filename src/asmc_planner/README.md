# asmc_planner

Single production package: Input/Parser → PlannerContext → BehaviorDecision →
ExecutionRequest → FrenetResult → Execution update → PlanningResult → ROS output.
Internal handoff uses C++ structures and direct calls. Behavior runs at 10 Hz;
Planning runs at 20 Hz. ROS topic and parameter contracts remain unchanged,
except `UrbanFeatureDebug` is now `asmc_msgs/UrbanFeatureDebug`.

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch
```

전체 컨테이너 단독 실행은 `/root/ws`의 `./scripts/drive.sh`를 쓴다.
LOCAL 선택은 `./scripts/drive.sh jamming_planner:=local`이다. master11311·bridge·인지·판제·
웹8088/8091을 포함한다. 위 `autonomy_no_bridges_v2.launch`는 기존 분리 실행용이며 중복 실행하지 않는다.
[실행/복귀 안내](../../docs/operations/commands.md#local-배포-선택과-기본-방식-복귀--2026-10-29)를 따른다.

신호등 추론·발행은 카메라 담당의 `asmc_camera/camera.launch`가 단독 소유합니다.
판제는 `/perception/traffic_light/detection`을 소비하며 별도 ResNet을 시작하지 않습니다.
기존 `traffic_light`·장치·checkpoint·camera_topic 인자는 제거했습니다. 실행 중 중단·재개·주기는
카메라의 `/perception/camera/traffic_light/set_mode`에서 제어하고 판제 신호 규칙은 유지합니다.
[인지 소비·제어 안내](../../docs/handoffs/perception-to-planning.md)를 따릅니다.

`behavior_planner` and `frenet_planner` packages have been removed. Their C++
namespaces and parameter namespaces remain for source/parameter compatibility.
Do not rename them as part of ownership cleanup.

## Regression assets

- `tests/behavior`: six registered Behavior tests and dashboard test.
- `tests/frenet`: 36 registered Frenet tests, fixtures and explicit v2 ROS test.
- `tests/equivalence`: frozen lifecycle/request/FSM comparisons and maneuver gate.
- `tests/acceptance`: isolated Behavior decision-loop helper (no control outputs).
- `tools`: test publishers and offline shadow/visualization tools.
- `scripts`, `web`: dashboard and calibration tools.

Known baseline: registered planner 39/41, AVOID 111 assertion failures;
`fsm_command_binding_test` retains its original missing-main link failure.
Explicit v2 ROS regression: 113/177. Assertions are unchanged by migration.

HD-map input paths now point to `config/behavior`. Historical hashes, map
provenance and frozen interface documents retain their recorded original paths.

Map source JSON values are unchanged. Their original CRLF bytes are retained so
frozen provenance SHA256 values remain valid. `asmc_hd_map.asset_paths` maps
historical recorded source identities to current assets without rewriting
immutable map products, manifests or reports.

Observer-only dashboard (does not start another planner):

```bash
roslaunch asmc_planner dashboard.launch state_topic:=/planning/vehicle/state
```

## 정적 미션의 객체 분류

`autonomy_no_bridges_v2.launch`는 `static_obstacle_unknown:=true`가 기본입니다.
`highway_merge:=true`와 함께 사용할 수 있으며, 일반 Behavior/Frenet에 동일한 정책을 적용합니다.
HD map의 `kcity2025_competition_metadata.yaml`에서 `static_obstacle_candidate`를 읽고,
메타데이터에 지정된 공식 경로에 자차와 객체를 투영합니다. 자차와 객체가 모두 해당
구간 안에 있을 때만 `person`·`vehicle`의 내부 `type`을 `UNKNOWN`으로 처리합니다.
자차는 경로 진행 방향도 확인하며, 횡 허용거리는 기본 10m입니다.

행동 판단이 사용한 변환 결과는 읽기 전용 `/planning/static_mission_policy`
(`std_msgs/String`, JSON) 진단으로 제공합니다. 진단은 제어 입력이 아니며,
입력 순서와 관측 identity를 보존합니다.

주행 대시보드(`dashboard.launch`, 기본 8088 포트)는 이 진단을 관측 identity와
내부 객체 ID로 연결합니다. 변환되면 지도에 `UNKNOWN · 인식 person` / `policy UNKNOWN`을
표시하며, 원본 `/planning/objects.type`을 실제 적용 정책으로 표시하지 않습니다.
관측이 일치하지 않거나 진단이 오래되면 `policy 미확인 (입력 PEDESTRIAN)`처럼 구분합니다.
새 관측의 진단이 늦으면 최대 300ms 이내의 직전 객체/진단 쌍을 함께 표시하고
`관측 동기화 대기`를 명시합니다. 이전 타입을 새 객체 좌표에 덮어쓰지 않습니다.
진단 도착 즉시 새 쌍으로 갱신하며, 실제 타입 변경은 그대로 표시합니다. 대기 기한 초과,
입력 무효 또는 생산자/설정 변경 시 이전 쌍을 재사용하지 않고 `판단 미확인`을 표시합니다.
화면 변경 반영에는 대시보드 서버 재시작과 브라우저 새로고침이 필요합니다.
LiDAR 뷰어(`roslaunch asmc_lidar view.launch`)는 원본 인식 분류를 표시합니다.

원본 `/planning/objects`, `classification`, `camera_class`, 운동 상태, 속도, ID,
OBB는 보존합니다. `DYNAMIC_PENDING`·`DYNAMIC`은 분류를 바꿔도 정적 회피 후보가
되지 않으며, 모든 객체는 기존 충돌 검사에 남습니다. 경로가 없으면 정지합니다.
자차 구간은 매 주기 확인하므로 새 객체 메시지가 없어도 구간 이탈 시 원래 분류가 복구됩니다.
공식 경로 전체의 선분에 공간 인덱스를 만들어 최근접 탐색 비용을 줄이며,
동일 거리에서는 기존처럼 경로 순서상 앞선 선분을 선택합니다. Behavior의 객체 복사와
자료형 변환은 새 관측 또는 자차 구간 상태 변경 시에만 수행합니다. 원본 관측이 같은지
확인할 때는 불변 메시지의 소유권을 유지하며, 지도 인덱스는 Behavior/Frenet에서 읽기만 합니다.

공식 경로는 메타데이터의 `$ASMC_DATA`를 환경변수 `ASMC_DATA`(기본 `/data`)로
치환하여 읽습니다. 경로 SHA256·점 개수·길이와 미션의 `planning_enforcement_allowed`를
검증하며 잘못된 자료로는 시작하지 않습니다. 시작 로그의 `Static mission UNKNOWN policy`와
적용 중 `Static mission: ... person/vehicle objects use UNKNOWN policy`로 확인할 수 있습니다.
기존 정책으로 실행하려면 `static_obstacle_unknown:=false`를 지정합니다.
단독 `planner_v2.launch`의 MORAI 객체 입력에는 이 변환을 적용하지 않습니다.

## 제어 명령 중단 시 정지

`asmc_control_selector`는 선택된 NORMAL/JAMMING 입력의 마지막 수신 시각을 steady clock으로 감시합니다. `~command_timeout_s`(기본 0.25초)가 지나거나 모드가 없거나 유효하지 않으면 20Hz로 가속 0·제동 1 명령을 보냅니다. 모드 전환 시에도 새로 선택된 입력이 도착할 때까지 정지합니다. 선택되지 않은 입력이 계속 들어와도 타이머를 갱신하지 않습니다. 따라서 MPC 프로세스 종료 뒤 시뮬레이터가 마지막 가속 명령을 계속 유지하는 일을 막습니다. `/ctrl_cmd`의 유일한 발행자는 계속 이 선택기입니다.

관련 검증: `catkin_make run_tests_asmc_planner_rostest_tests_control_selector.test`. 시뮬레이터 검증에서는 프로세스를 종료한 뒤 실제 속도도 확인해야 하며, `SetVelocity(0)` 응답만으로 정지를 확정하지 않습니다.

### 동적 장애물 구간 확장 판단

`dynamic_obstacle_autonomy.launch`의 새 Follow/Avoid/Stop/E-Stop 평가는
`dynamic_behavior.route_links`에 설정한 동적 장애물 구간에서만 작동한다.
구간은 자차의 원본 경로 투영으로 판단하며, 교차로 연결부의 별도 지도 ID 때문에
판단이 잠깐 꺼지지 않도록 `route_required_link_id`를 사용한다.
구간 밖에서는 기존 Behavior의 선행차·보행자 판단을 복원하고, Frenet에는
모든 최종 객체를 그대로 전달한다. 인지 입력이나 기존 충돌 검사를 끄는 설정이 아니다.

`dynamic_behavior.section_speed_limit_kph: 25.0`은 기존 교차로 접근 상한을
구간 끝까지 유지하는 임시 시뮬레이터 설정이다. 정지선 통과 직후 일반 순항으로
재가속하는 요청을 막으며, 더 낮은 신호·장애물 제한이 우선한다.
`/planning/dynamic_behavior`의 `in_scope`와 `scope_link_id`로 적용 범위를 확인한다.

기존 Frenet 충돌 검사도 자차와 객체의 관측 시각을 맞춘 뒤 예측한다.
원본 객체로 운동 이력을 갱신하고, 평가용 사본만 자차 관측 시각까지 이동한다.
공용 `AdvanceObjectObservation`이 기존 운동 예측과 8개 꼭짓점의 평행 이동을
재사용하며, 원본 메시지·관측 시각·형상을 덮어쓰지 않는다. 수신 시각의 신선도와
source time 차이는 별도로 검사하고, 설정된 입력 제한을 넘으면 기존 안전 정지로 간다.

### 회피 중 제동 해제의 연속성

지도 기반 회피는 이미 검증한 가속도 계획을 진행시켜, 20Hz 재계획마다 제동 해제
과정을 처음부터 되풀이하지 않는다. 실제 위치·속도는 계속 최신 측정값을 사용한다.
속도가 회피 상한 아래이고, 최근의 승인된 계획이 제동을 해제 중이며, 상한이 더 낮아지지
않았을 때만 그 계획을 초기 가속도에 반영한다. 후보는 모든 최신 객체·도로·운동학 검사를
다시 통과해야 한다. PREPARE와 실패 시 유지·정지 경로에는 이 제동 해제를 적용하지 않는다.
오래되거나 빠진 이력도 사용하지 않는다. 회피 속도 상한과 기존 긴급정지 설정은 유지한다.

KEEP의 저속·정지 후 출발 경로도 같은 공용 충돌 검사를 통과해야 한다. 곡률을 만족하는
기어가기 경로라도 최종 객체의 예측 점유와 겹치면 반환하지 않는다. 이때 실패 사유는
`COLLISION_BLOCKED`이며, 실행 계층이 과거 이동 경로를 재사용하지 않고 정지한다.

저속 KEEP 출발은 기존 HD Map의 도로 포함 검사도 사용한다. 기어가기 경로는
도로 안에 드는 횡복귀 거리를 찾고, 일반 후보도 기존 저속 경계(1.5m/s) 안에서는
도로 밖으로 나가는 후보를 제외한다. 남은 후보에는 기존 곡률·충돌 검사를 그대로
적용한다. 도로에 맞는 경로가 없으면 `ROAD_BOUNDARY_BLOCKED`로 과거 경로 재사용을
금지한다. 회전교차로의 생성·대기·충돌 정책은 이 변경 대상이 아니다.
필수 설정 `planner/vehicle_shape/road_outer_width`는 MORAI SDK에서 관측한 외곽 폭
2.161m이며, 기존 객체 충돌 모델의 폭 1.9m와 0.45m 여유를 바꾸지 않는다.
Config가 크기·유한성·누락을 검증하며, 원본 지도와 시나리오는 유지한다.
이 검사는 계획된 경로에 대한 조건이다. 실제 차체의 도로 포함은 MORAI 기록으로
별도 검증해야 하며, 이 조건만으로 주행 성공을 선언하지 않는다.

저속 KEEP는 조향각의 크기와 함께 시간별 변화율도 검사한다. 필수
`planner/steering_profile`의 `curvature_gain`, `max_rate`, `reference_speed`,
`min_rate`는 현재 Controller의 같은 물리 모델·제약을 반영하며, 시험에서 두 설정의
동등성을 확인한다. 기존 wheelbase와 최종 Cartesian 곡률로 필요한 조향각을 계산한다.
기어가기와 Controller의 속도별 slew 기준점(현재 5m/s) 아래의 일반 KEEP 후보만
대상이며, 미래 속도에 따라 더 낮아지는 한계도 검사한다. 만족하지 못하는 후보는
제외하고, 모든 대안이 불가능하면 `ACTUATION_BLOCKED`로 과거 경로 재사용을 금지한다.
Controller의 조향 한계를 높이거나 MPC 상태를 Planner에 복제하지 않는다.
정지·저속 회복 범위(기존 1.5m/s)에서는 차체 전체가 현재 차선 안에 있을 때
작은 횡위치를 유지하는 기존 연속성 후보를 우선한다. 출발하자마자 짧은 시간 안에
중앙으로 돌아오는 S자 경로를 강요하지 않는다. 중앙 복귀 후보도 계속 생성·검사하며,
속도가 회복되거나 차체가 차선 경계를 넘으면 원래 중앙 목표를 사용한다. 위치·속도·방향을
바꿔 가정하거나 별도 출발 타이머·기동 상태를 추가하지 않는다.

KEEP는 기존 경로가 유효하면 그대로 선택한다. 운동학 검사를 통과한 모든 후보가
충돌 검사에서 막혔을 때에만, 실제 가속 가능한 추가 후보를 같은 파이프라인으로
검사한다. 이 추가 후보는 0.05초 뒤 지령을 미래 전체의 상한으로 고정하지 않고,
지도·곡률·외부 명령 상한과 기존 가속도·jerk 한계로 미래 속도를 적분한다.
후보 곡률 때문에 상한이 바뀌면 즉시 지령이 같더라도 추가 후보를 다시 생성한다.
객체 제외·충돌 범위 축소 없이 검사하며, 추가 후보도 막히면 기존 보호 동작을 유지한다.
정지 상태의 기어가기 경로가 충돌로 거부된 경우에도, 같은 입력을 재사용해 이 공용
KEEP 후보 검사로 이어간다. 대안 생성까지 실패하면 최초 충돌 실패를 유지하고 출력
경로를 비운다. 추적 입력을 두 번 갱신하거나 거부한 경로를 재사용하지 않는다.
이때 출발 경로가 관측된 차체 방향을 보존하지 못하면 정지를 유지한다. 수치적 방향
동등성만 허용하며, 차체를 참조선 방향으로 바꿔 가정해 통과시키지 않는다.

동적 구간의 주행가능 영역에 있는 UNKNOWN 객체는 잠시 정지했다는 이유만으로
기존 정적 AVOID가 먼저 선점하지 않는다. 정적 회피의 겹침 이력을 만들기 전에
자격을 확인하므로, 미관측·ID 변경 때 그 이력으로 회피 준비를 반복하지 않는다.
구간 진입 전에도 해당 대상의 정적 회피
자격만 보류하며, 새 FOLLOW/STOP/E-Stop 작동 구간은 기존 자차 구간 제한을 유지한다.
확정 STATIC_OBSTACLE·구간 밖 객체와 이미 COMMITTED된 기동은 이 규칙으로 바꾸지
않는다. 인지 분류·좌표·속도·형상과 Frenet의 전체 최종 객체 입력도 그대로 유지한다.

### MORAI 일반 주행 상한 58 km/h 설정

`config/behavior/behavior_params.yaml`의 `normal_cruise`는 명시적으로 켜는
시뮬레이터 시험용 정책이다. 일반 KEEP, 유효한 차량/지도 상태, 지도 제한
40 km/h 이상에서 목표와 계획 상한 58 km/h를 사용한다. 해당 조건에서는
원본 지도를 수정하지 않고 지도 속도 상한(예: 40 km/h)을 대체한다.
`enabled: false`이면 기존 지도 기반 정책으로 복귀한다.
`link_speed_caps_kph`는 도로 ID별 상한을 명시하는 선택 설정이다. 현재는
회전교차로 합류를 마친 뒤 일반 도로 `A2256W000054`만 50 km/h로 설정한다.
이 항목은 같은 일반 주행 조건을 통과해야 적용되며, 전체 58 km/h 상한을
넘을 수 없다. 값이 유효하지 않으면 고속 정책 전체를 끄고 지도 제한으로 복귀한다.
ROS/YAML 입력과 km/h 변환은 Parser, 속도 정책과 보호 조건은 Behavior가 담당한다.

신호 접근·정지·정적 장애물·선행 차량/FOLLOW 유지·보행자/긴급 위험·회전교차로
합류/진입 링크에서는 이 정책을 적용하지 않는다. 감지되지 않은 장애물을 이
정책이 새로 검출하는 것은 아니다. 신호 접근은 25 km/h, 정지선 전 여유 25 m,
통과 후 유지 5 m 설정이며, 회피·합류 및 경로 충돌 검사는 계속 우선한다.
명시되지 않은 지도 제한 40 km/h 미만 도로는 기존 값을 유지한다.

2026-10-14 고속 안정성 수정에서는 `config/frenet/params.yaml`의 기존
횡가속도 한계 2.6 m/s², 곡률 속도 여유 계수 0.95, 감속 계수 0.5를 유지했다.
횡가속도 한계를 높인 시험 후보는 반복 추종 오차 때문에 채택하지 않았다.
반경 80 m의 일정한 곡선에서 계산 상한은 약 50.6 km/h다.
더 급한 곡선, 앞쪽 급커브, 지도 제한 및 특수 구간의
낮은 속도 요청은 계속 감속을 요구한다. 이 계수는 곡률 속도 계산 공통값이며
일반 KEEP 전용이 아니다. 장애물 회피/합류의 별도 속도 상한은 변경하지 않았다.

2026-10-15 평균속도 튜닝에서는 감속 계수를 0.65로 조정했다. 곡선을 앞두고
사용하는 감속 가정이 1.5에서 1.95 m/s²로 바뀌며, 횡가속도·jerk 한계는 유지한다.
이 공통 곡선 설정은 합류 경로 계산에도 쓰인다. 신호 접근/STOP의 별도 감속
설정 1.5 m/s²와 신호·장애물·합류 속도 상한은 유지한다.

58 km/h는 순항 요청값이며 구간 평균의 보장값이 아니다. 실제 차량의 순간 속도는
요청값을 조금 넘을 수 있으므로, 사용자 기준인 60 km/h 미만을 Bag으로 확인한다.
이번 시험의 실시간 중단 기준은 59 km/h로 추가 여유를 두었다. 기존 직진 안정성
보고서는 이전 설정의 결과이며 이 정책의 검증 결과로 재사용하지 않는다.
함께 변경된 Controller는 정상 STOP의 조향 연속성과 조향 지연 이력을 보존하고,
1.5초 KEEP 경로에 맞춰 기본 예측 길이를 29 × 0.05초로 사용한다.
코드 변경 뒤 관련 패키지를 재빌드하고 주행 launch를 다시 실행해야 적용된다.
10월 15일 튜닝만 되돌리려면 순항 목표 57.8, 지도 최소 속도 50,
`link_speed_caps_kph: {}`, 곡선 감속 계수 0.5로 복원한다. ROS 파라미터 서버에는
삭제한 하위 키가 남을 수 있으므로, 실행 중인 Planner를 종료한 뒤
`rosparam delete /behavior_planner/normal_cruise`하고 launch를 다시 실행한다.

### 일반 고속 KEEP 곡선 프로필

`planner/keep_cruise`는 지정된 `source_links` 도로에서 외부 명령이 KEEP이고
요청 속도가 40 km/h 이상인 경우에만 횡가속도 한계 3.8 m/s²와 속도 오차 비용 4.0을 선택한다.
외부 요청 상한도 40 km/h 이상이어야 하며, merge 태그가 있으면 제외한다.
STOP가 내부적으로 KEEP 생성기를 사용해 접근하는 경우도 제외한다.
기본 횡가속도 2.6, 기본 속도 오차 비용 1.0은 계속 다른 기동에 사용한다.
최대 목표 58 km/h, 신호·회피·합류 속도, 충돌 필터는 그대로 유지한다.

설정 읽기/검증은 Config, 적용 여부는 Maneuver/KeepCruise가 소유한다.
선택된 한계를 목표속도 계산, 초기 경계조건, 후보 생성/검사에 동일하게
전달하며, 공통 Speed/Evaluation 계층에 모드 판단을 넣지 않는다.
Controller는 기본 `cost.contour_error: 6`을 유지하고 참조 곡률
0.004~0.012 /m에서 `curve_contour_boost: 6`을 점진적으로 더한다.
직선 GPS 잡음 시험을 유지하면서 곡선 추종을 보강한다. 이 비용은
공통값이므로 전체 주행 검증이 필요하다.

회귀: `keep_cruise_scope_test`, `config_loader_validation_test`와 기존
KEEP/STOP/FOLLOW/곡선 속도 시험. 이전 사용자 정의 Frenet YAML에도
`keep_cruise` 다섯 필드가 필요하다. 키 누락·잘못된 값은 초기화를 거부한다.
롤백: `keep_cruise.enabled: false`, Controller `cost.curve_contour_boost: 0.0`.
실측 결과와 미달성 목표는 작업별 Bag/검증 보고서에서 확인한다.

고속도로 출구에 높은 횡가속도 한계를 적용한 후보는 전체 주행 검사에서
탈락했으므로 제외했다. 현재 프로필은 회전교차로 이후 2개와 재밍 이후 5개
도로만 지정하며, 나머지 도로는 기본 한계를 사용한다. 도로 ID는 기존
외부 명령의 source_link_id를 사용하고 지도 파일을 바꾸지 않는다.

합류 실행 중 후행 차량 처리와 조건부 NPC 반응 가정은 [merge-response-execution.md](docs/merge-response-execution.md)를 참고하세요. 완전한 후방 안전 인증과 기존 명목 유지 경로의 위험 완화를 진단에서 구분합니다.
