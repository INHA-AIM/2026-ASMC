# asmc_planner

Single production package: Input/Parser → PlannerContext → BehaviorDecision →
ExecutionRequest → FrenetResult → Execution update → PlanningResult → ROS output.
Internal handoff uses C++ structures and direct calls. Behavior runs at 10 Hz;
Planning runs at 20 Hz. ROS topic and parameter contracts remain unchanged,
except `UrbanFeatureDebug` is now `asmc_msgs/UrbanFeatureDebug`.

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch objects_source:=lidar traffic_light:=false
```

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

## 제어 명령 중단 시 정지

`asmc_control_selector`는 선택된 NORMAL/JAMMING 입력의 마지막 수신 시각을 steady clock으로 감시합니다. `~command_timeout_s`(기본 0.25초)가 지나거나 모드가 없거나 유효하지 않으면 20Hz로 가속 0·제동 1 명령을 보냅니다. 모드 전환 시에도 새로 선택된 입력이 도착할 때까지 정지합니다. 선택되지 않은 입력이 계속 들어와도 타이머를 갱신하지 않습니다. 따라서 MPC 프로세스 종료 뒤 시뮬레이터가 마지막 가속 명령을 계속 유지하는 일을 막습니다. `/ctrl_cmd`의 유일한 발행자는 계속 이 선택기입니다.

관련 검증: `catkin_make run_tests_asmc_planner_rostest_tests_control_selector.test`. 시뮬레이터 검증에서는 프로세스를 종료한 뒤 실제 속도도 확인해야 하며, `SetVelocity(0)` 응답만으로 정지를 확정하지 않습니다.

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
