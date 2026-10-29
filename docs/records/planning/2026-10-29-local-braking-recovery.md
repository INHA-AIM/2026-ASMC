# LOCAL-2F — 경로·제동·복구 연속성과 인지 입력 점검

기준은 LOCAL-2E `1aba8dbe`다. GPS 독립 LOCAL 경로를 개선하며 일반 판제, 기존 camera
음영 방식의 기본 선택, 공용 메시지·MD5, 카메라 모델과 LiDAR 검출·추적·fusion은
유지한다. 별도 localization worktree는 수정하지 않았다.
[이전 LOCAL-2E 기록](2026-10-29-local-avoidance-continuity.md)은 당시 근거로 보존한다.

등록한 모든 시험과 실패는 [ASMC 검수](http://localhost:8000/local2f-20261029-validation01/index.html)에서
본다. `review.json`은 수치, `native-review.html`은 실제 주행·복귀, 각 run의
`local-gallery`는 원본 RGB와 같은 시각의 노면·OBB·선택 경로다. 준비 실패와 실제
주행 실패를 구분하며, 거리 도달을 운영 수락으로 자동 처리하지 않는다.

## 구조와 변경 경계

| 위치 | 책임과 변경 |
|---|---|
| `asmc_planner/src/local/node.cpp` | ROS 입력·식별·시각 정렬·발행. 현재 속도/가속/yaw-rate와 복구 출처도 진단에 보존 |
| `local/planner.cpp` | 후보 생성·현재 차체 sweep·공통 비용·이전 후보 재검사 |
| `local/speed_profile.{hpp,cpp}` | jerk 제동거리·공간 속도 상한·물리적 속도 궤적. 일반/복구 후보의 동일 계약 |
| `local/recovery.{hpp,cpp}` | 명시적인 저속 복구 후보. 실제 OBB·현재 도로 외곽 검사를 공유 |
| `local/obstacle_memory.hpp` | 측정 asset의 짧은 소실 보조 후보. 결과가 충분하지 않아 기본 OFF |
| `asmc_controller/src/local` | 기존 MPC 연결을 재사용. 이번 단위에서 두 번째 제어기나 운동 분류기를 추가하지 않음 |
| `tools/scen_runner` | 실제 시나리오·native 비교·기록·gallery. 제어와 평가용 native pose를 분리 |

노드는 planner/controller 두 개를 유지한다. 일반 MPC의 bicycle 모델·페달 LUT·행동
정책을 변경하지 않았다. `node.cpp`의 장면 조립/JSON은 후속 분리 여지가 있지만 이번에는
속도·복구 계산부터 분리했다. 계산 라이브러리에 ROS 입출력을 새로 넣지 않았다.

## 확인한 결함과 수정

### 1. 고정 길이와 실제 제동 가능성의 불일치

기존 복구는 유효한 swept 경로도 6m 미만이면 버리고 4m arc로 대체했다. 복구의 실제
상태 검사에서는 실행 지연까지 포함한 제동거리를 확인하지 않았다. 합성 장면에서
필요 4.389m인데 4m arc를 유효하다고 표시했다. 수정 후 같은 장면의 5.2m swept 경로를
선택한다. 수정 전/후 로그와 native 회귀를 등록 run에 보존했다.

고정 6m 조건 대신 현재 속도·가속·jerk·실행 지연으로 후보를 검증한다. generic 복구는
필요 제동거리+25cm, 최대 8m까지 비교하되 기존 근거리 4m 밖은 엄격한 현재 sweep을
통과해야 한다. 거리나 관측 범위를 늘려 표시하지 않는다. 제동할 수 없는 경로는 무효다.

### 2. 쾌적 감속과 실제 제동 한계의 분리

`drive11`의 69/70계획에서는 실제 속도 2.87/2.82m/s로 이미 감속하고 있었다.
쾌적 제동 1.5m/s² 기준은 4.48/4.31m가 필요하지만 경로는 3.81/3.61m였다. 이를
항상 경로 무효→full-brake로 바꾸고 있었다. 이는 차량의 실제 제동 불가능과 다르다.

공용 MPC의 음가속 한계는 5m/s²다. 기존 pedal 보정표에는 0.20→2.71,
0.30→4.75m/s²가 있다. 기존/이번 native 기록의 안정된 0.12–0.20 pedal window 54개에서
감속 중앙값은 3.15m/s², 범위는 1.76–3.90이었다. 정식 actuator identification이나
모든 조건의 3m/s² 보장은 아니다. 일반 제어기 LUT를 이 수치로 덮지 않았다.

`controlled-brake-trial.yaml`만 `controlled_braking_limit_mps2: 3.0`을 사용한다.
패키지 기본은 1.5다. **평소 공간 속도 상한은 1.5와 기존 overshoot reserve를 유지**한다.
실제 현재 상태가 쾌적 범위에 안 들어갈 때만 별도 제동 후보를 확인한다. 동일 jerk와
실행 지연을 포함해 경로 안에 속도 궤적을 담고, 3.0으로도 부족하면 무효다.
선택 감속·쾌적 제동거리·필요/확보 거리를 진단과 웹에서 구분한다.

### 3. 정지 후 짧은 재배치와 도로 외곽 보호

`drive07`은 약 9.82m에서 정지해 12초 stationary 판정이 났다. 동일 자세로 시작한
`drive09/10`도 준비 실패였다. 실패는 삭제하지 않았다. 현재 입력이 끊긴 것은 아니며
가까운 barrier와 본네트 가림 때문에 짧은 후보가 남는 장면이다. 250ms의 오프라인
계산 예산만 늘려도 기존 경로는 생기지 않았다.

현재는 실제 충돌 pose 이전의 prefix도 제동 검사하고, **현재 관측한 opening 방향**의
큰 곡률 후보를 함께 비교한다. 원점 차체가 이미 OBB와 겹치면 통과시키지 않는다.
다음 pose만 보는 대신 기존 연속 sweep를 사용한다. 짧은 재배치는 정지·1km/h 이하의
명시적 복구이며 속도 상한은 3km/h다. 일반 free-space 인증으로 해석하지 않는다.

기존 generic 복구는 도로 외곽의 비노면까지 비용으로만 처리했다. 현재 raster 각 행의
노면 양끝 밖에 있는 **관측 비노면은 차체+기존 최소 여유에 대해 hard veto**다.
내부 가림 hole만 복구 비용으로 남긴다. 이는 새 벽 검출기나 독립 터널 벽 측정이 아니다.
원본 probability/coverage는 바꾸지 않으며 추가 색인은 40kB/scene이다.

Git fixture는 실제 정지 scene에서 파생했다. 원본 8bit raster는 D에 보존하고,
153/166 threshold에서 결정이 동등한 3레벨 RLE를 작은 회귀 fixture로 사용한다.
coverage와 geometry는 그대로다. 미관측 전체, 현재 도로 외곽, 실제 OBB, 연결되지 않은
partial-road 반례도 검사한다. native 경로 성공과 실제 폐루프 성공은 따로 기록한다.

### 4. 이전 경로를 같은 정책으로 현재 재검사

이전 복구 경로를 다음 프레임에는 일반 노면 규칙으로만 검사하던 불일치도 수정했다.
`previous_recovery`를 내부에 보존하고, 현재 opening·현재 OBB·현재 도로 외곽·운동학을
다시 통과할 때만 같은 bounded 정책으로 평가한다. 전체 미관측이나 새로운 OBB에서는
과거 경로가 통과 근거가 되지 않는다. strict 검사를 이미 통과한 후보를 다시 복구로
낮추지 않으며, 더 짧은 새 arc가 더 긴 현재 재검증 후보를 덮지 않게 한다.

10cm 경로의 첫 점을 `x<=0.1`로 버리는 문제도 있었다. 두 step의 조향 변화를 더 짧은
chord 길이와 비교해 같은 변화율로 생성한 경로를 거부했다. 원점 중복만 제외하고
10cm 점은 유지한다. 재검사의 수치 허용은 1e-8rad다. 오래된 명령을 재발행하거나
watchdog·source freshness를 갱신하는 방식은 사용하지 않는다.

## 인지 입력·클래스와 남은 제약

현재 E3 모델의 15종은 카메라/3뷰 fusion 계약과 일치한다.

- vehicle: car, bus, truck
- dynamic_obstacle: person, dog, deer, bicycle, stroller
- static_obstacle: motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier

traffic_cone은 이전 16종 모델의 클래스다. 현재 15종 모델에서 빠진 것을 fusion 누락으로
해석하지 않는다. 카메라 계약 5개와 native fusion 정책 32개 검사를 확인했다.
LOCAL은 실제 관측된 8코너, 클래스·그룹, 속도 validity를 받는다. 정적/동적 그룹을
물리적 운동 상태와 같게 취급하지 않는다. motorcycle도 움직일 수 있고 차량/사람도
정적으로 배치될 수 있다. YOLO 검출 여부만으로 5km/h를 강제하는 정책은 넣지 않았다.

미확정 속도에는 기존 5m/s 나이별 envelope를 유지한다. 100ms이면 50cm라 좁은 통로에서
보수적이다. 이를 semantic 이름만으로 0으로 만들지 않는다. asset 소실 보조는 같은
scan을 중복 확인하지 않고 원래 시각부터 최대 200ms만 측정 hull을 보존한다. 그러나
OFF/ON 반복에서 일관된 개선이 확인되지 않아 `retain_static_observations: false`를
유지한다. public 객체에 없는 최근 YOLO 양성 시각을 임의로 만들어 쓰지 않는다.

정지 실패 구간에는 전방/우측 검출이 비고 좌측에서만 plastic_barrier가 드문드문 잡혔다.
track ID와 분류의 소실/재확인을 원본 camera observation과 native fusion trace로 남겼다.
클래스 표 누락으로 단정하지 않는다. bbox가 잘리는 근거리·부분 군집의 관측 연속성,
실제 물체보다 얇은 OBB와 matching 근거는 인지 후속 과제다.

## MPC 점검

축거 3m, curvature gain 0.844, 조향 지연 0.15초, dt 50ms의 기존 kinematic bicycle을
유지한다. LOCAL horizon은 2.4초다. 실제 최종 wire 조향과 속도·IMU yaw-rate의 인과적
표본만 대조하고 전반부 선택/후반부 오차를 구분했다. LOCAL 구간만 남긴 세 기록에서
가장 잘 맞는 effective delay는 모두 0.15초였고 gain은 약 0.854–0.865였다.
현재 값에 가까워 새 동역학 모델이나 공유 제어 파라미터를 추가하지 않았다.

수신 시각 기반 effective delay는 순수 actuator delay가 아니다. 명령은 측정 wheel
angle도 아니다. 직진/저속 등 충분한 excitation이 없는 표본은 성공으로 계산하지 않는다.
[Autoware MPC](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/model_predictive_control_algorithm/)와
[Path Sampler](https://autowarefoundation.github.io/autoware_universe/main/planning/sampling_based_planner/autoware_path_sampler/)의
후보·검증·비용·실행 분리 구조를 참고한다. 외부 ROS2 실행 의존성을 추가하지 않는다.

## 검증·실행 경계

source freshness 350ms, motion forecast 100ms, selector watchdog 250ms를 유지한다.
ROS master 조회는 기존 별도 worker에 두고 제어 콜백으로 되돌리지 않았다. 평가용 native
pose와 고정 경로는 LOCAL 제어 입력으로 사용하지 않는다. 자동시험은 독립 master와
공유 physical lease를 사용한다. 다른 worktree의 노드/컨테이너는 종료하지 않았다.

ROS 정합 시험은 네 topic이 원자적으로 도착한다고 가정하지 않는다. 실제 선택 stamp
차이와 age 차이의 일치, 모든 표본의 기존 freshness, 의도한 150ms 이상 독립 시각 사례의
존재를 검사한다. 운영 시간 한계를 완화하지 않는다.

수동 확인은 `asmc-local2c-20261028`의 공유 11311 환경에서 camera/LiDAR/autonomy를
한 세트씩 실행한다. 재현하려면 autonomy에 다음 실험 profile을 명시한다.

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local local_planner_config:=/artifacts/local2f-20261029-validation01/controlled-brake-trial.yaml
```

기본 `camera` 선택과 package의 `semantic_reference_fallback: false`, 쾌적 감속 1.5는
운영 기본으로 유지한다. 위 profile은 semantic 방향 보조와 controlled brake 후보를
사용하며 asset 기억은 OFF다. 기본 LOCAL 명령만으로 같은 후보를 켰다고 표현하지 않는다.

### 실패 직후 후보 상태 보존

경로 계산 실패마다 마지막 기하 후보까지 지우던 동작도 분리했다. 실패 결과 자체는
controller에 그대로 전달하지만, 기하 seed는 다음 현재 관측의 재검사 대상으로 남긴다.
세대·설정·모드 teardown과 후진 미지원에서는 비운다. 이전 명령/유효 결과를 cache해서
발행하는 방식이 아니다. 실제 ROS 회귀는 현재 노면이 모두 막힌 동안 유효 경로가
나오지 않음, 기하 seed만 남음, 새 노면이 들어오면 재계산 재개를 함께 검사한다.

## 반복 주행 결과와 미수락 항목

총 34회, 누적 2,991.58m를 기록했다. 29회가 설정 거리에 도달했고, 준비 실패 3회,
12초 정체 종료 2회다. native 접촉과 수신 collision 이벤트는 없었다. 13시·기본 날씨의
통제시험이며 여러 날씨·센서 장애·실제 충돌 외곽까지 검증한 운영 수락은 아니다.

| 마지막 실행 코드 `dadf9037` | 거리 | native 주행 중 full-brake tick | 계획 critical p95 | ACTIVE 유효 경로 최대 공백 | 마지막 2초 기준 경로 최대 거리 |
|---|---:|---:|---:|---:|---:|
| drive31 · 우측 장애물 | 80.09m | 0 | 45.27ms | 183.82ms | 2.77cm |
| drive32 · 좌측 장애물 | 80.21m | 0 | 44.07ms | **262.44ms** | 2.56cm |
| drive33 · 연속 장애물 | 60.27m | 0 | 44.46ms | 165.66ms | 2.34cm |
| drive34 · 일반→터널→일반 | 235.69m | 0 | 43.37ms | 164.37ms | 10.29cm |

마지막 네 시험의 입력 잠금 유지 p95는 0.072–0.079ms다. 위치 거리는 고정 기준 경로와
평가용 native pose의 segment 투영값이며 독립 측량 GT가 아니다. 회피 중 의도적인
횡이동을 추종 오차로 평가하지 않는다. integrated 시험의 NORMAL 최고속도를 LOCAL
10km/h 달성치에 섞지 않았다. 소스 `aaf590f2`의 우/좌 각 3회도 정지·접촉 없이
완료했으나, 마지막 코드와 서로 다른 revision이라는 점을 `review.json`에 남긴다.

drive32는 250ms 공급 기준을 넘었으므로 전체 공급 수락을 선언하지 않는다. 해당 두
계획의 계산은 22.8/25.3ms였다. bag에는 중간 family가 도착했지만 같은 stamp의
노면 probability/coverage가 없고, 다음 완성 입력까지 source 간격이 236.9ms였다.
계획 계산 과부하만으로 설명되지 않는다. 이 사실은 upstream 누락의 근본 원인까지
확정한 것이 아니다. `supply-gap-drive32.json`과 원본 bag으로 후속 분석한다.

가까운 barrier 앞 정지 자세는 여전히 실패한다. drive07에서 9.82m 주행 후 정체했고,
같은 자세의 drive09/10/25는 준비에 실패했다. 최종 drive30은 AUTO 출발은 했지만
약 4cm 후 다시 12초 정체했다. 무조건 정지 명령을 지운 결과를 개선으로 세지 않았다.
저속 복구가 통과할 실제 차체 여유와 현재 관측 근거가 아직 부족하다.

측정 asset의 소실 보조는 일관된 개선이 없어 OFF다. 차체 폭은 SDK actor bbox
2.160835m를 유지한다. VehicleSpec 1.892m와 실제 접촉 외곽의 관계가 미확정인 상태에서
폭을 줄여 통과시킨 시험은 없다. 따라서 이번 결과는 **시험 후보 개선·빌드 준비 완료**,
전체 LOCAL-2D 수락·운영 기본 승격·PR 병합은 아니다.

planning CI 그룹의 NORMAL 회피·고주로·로터리·도로 제약·대시보드와 LOCAL 정합·소유권·
진입·현재 관측 재검사·실제 명령 이력 시험이 통과했다. 기존 145개 고유 CI 검사 보존도
확인했다. 초기 실패 로그를 성공 로그로 덮지 않았다.

## 다음 단위 제안 — LOCAL-2G

**차체 충돌 외곽과 근거리 정지 후 복구를 먼저 확정하고 공급 공백을 분리한다.**
새 모델·3/5개 차선 선택기·별도 운동 분류기를 추가하기보다 현재 실패를 재현한다.

1. SDK bbox·VehicleSpec·후륜축 위치·실제 접촉을 좁은 통로 저속 시험으로 대조한다.
   물리 외곽과 선호 여유를 분리하고, 증거 없이 2.1608m를 1.892m로 바꾸지 않는다.
2. drive07/30의 정지 자세와 1.5m 뒤의 pretrap 자세를 고정한다. 같은 fixture의
   native 계산과 MORAI 폐루프를 연결해 opening·본네트 가림·현재 OBB·조향 지연 prefix의
   첫 거부 원인을 나눈다. MPC 미래 예측과 입력 정렬에서 이동을 이중 보상하지 않는다.
3. 현재 관측에서 검증한 짧은 연결 공간만 3km/h 복구로 확장한다. 차체 회전 sweep,
   실제 OBB·외부 비노면·350ms freshness를 유지한다. 실패가 반복되면 원인을 기록하며
   이미 막힌 경로를 유효하다고 발행하지 않는다.
4. drive32의 중간 family와 노면 입력 소실을 같은 stamp별로 추적한다. source 공급,
   노면 adapter, callback 대기와 탐색 시간을 분리하고 watchdog 연장으로 숨기지 않는다.
5. 정상 접근 좌우 각 3회·연속 장애물·일반 진출입을 같은 최종 revision에서 반복한다.
   준비 실패·정체·제어 공백을 함께 보고한다. 인지 담당에게 근거리 bbox 잘림·군집/ID
   소실·얇은 barrier OBB 사례를 전달하고 본 단위의 플래너 변경과 분리한다.

실패 자료 정본은 D의 `local2f-20261029-validation01/handoff/README.md`이며 갤러리에서
직접 연결한다. 원본 bag과 provenance를 복제·삭제하지 않고 경로와 SHA로 인계한다.
