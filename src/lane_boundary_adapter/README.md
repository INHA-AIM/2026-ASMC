# Lane Boundary Adapter

> Planning 전체 경계와 제어 승격 gate는
> [Planning/Control 개발 가이드](../../docs/packages/planning-control/README.md)를 따른다.
>
> 음영구간 v3의 판제 소비 규칙, camera drivable 결합과 파트별 소유권은
> [음영구간 카메라 인지 v3 판제 인수 가이드](../../docs/handoffs/perception/shadow-camera.md)를
> 먼저 따른다.

카메라 인지가 발행하는 typed metric 경계 관측을 판제 개발자가 확인할 수 있는 차로 띠
후보로 바꾸는 **read-only C++ pilot**이다. MORAI UDP 패킷을 직접 다루지 않는다. 데이터
흐름은 다음과 같다.

```text
MORAI sensors --UDP--> asmc_bridge sensor node --ROS--> camera perception
  ├─/perception/lane/boundary_family───────> canonical shadow-lane handoff
  │    └─> shared-family read-only adapter --> MarkerArray + DiagnosticArray
  └─/perception/lane/boundary_observations─> compatibility/diagnostic pilot only
```

이 패키지는 `behavior_planner`, `frenet_planner`, `mpc_controller`, UDP 명령 송신 코드를
수정하거나 호출하지 않는다. `BehaviorCommand`, `Trajectory` 및 차량 제어 명령도 발행하지
않는다. 따라서 판제 파트가 현재 작업 중인 실행 경로와 독립적으로 빌드·검토할 수 있다.

`boundary_family`가 현재 음영구간 판제 인수 정본이다. shared-family adapter는
공동 피팅 결과의 sample별 support를 보존한 채 두 인접 경계 사이를
strip/centerline segment로 변환한다. independent pilot과
`boundary_observations`는 기존 호환·진단 재현용으로만 남겨 신규 판제 consumer가
사용하지 않는다. 이 package의 node 자체는 여전히 read-only monitor이며
제어 승격을 의미하지 않는다.

shared-family C++ core는 향후 판제가 재사용할 수 있지만 현재 node는 계속 read-only monitor다.
음영구간 on/off, route 의미, camera drivable threshold, LiDAR occupancy 차감과 fallback은 이
패키지에 추가하지 않는다. 그 정책과 연결·closed-loop 검증은 판제 파트가 소유한다.

## 수명주기와 판제 흡수 조건

이 package는 최종 배포 구조가 아니라 perception→planning 계약을 검증하는
임시 reference implementation이다. 판제가 음영구간 lane/corridor consumer를 구현할 때
다음 자산을 판제 소유 module로 흡수한다.

- `BuildBoundaryFamilyStripContext()`를 포함한 message→strip 변환 core
- frame·ODD·algorithm·representation·freshness 검사
- `OBSERVED`/`STRUCTURE_INFERRED`/`INVALID` 권한과 invalid hard-split
- consumer horizon, 차로폭·순서·교차 검사 설정값과 C++ 회귀 테스트
- Marker/diagnostics 중 판제 개발·운영에 필요한 read-only 관측 항목

최종 소유 package를 `behavior_planner` 또는 `frenet_planner`로 미리 고정하지 않는다.
route/mission 문맥·LiDAR occupancy·corridor 후보를 함께 소유할 판제 경계를
팀이 합의한 뒤 그 모듈에 흡수한다. 다음 gate를 모두 통과하기 전에는
`lane_boundary_adapter`를 삭제하거나 production planning source로 연결하지 않는다.

1. 판제 feature flag 기본값 `off`, 음영 ODD 진입·이탈과 stale/fallback 정책 합의
2. 현 C++ core test를 흡수 모듈에서 동일하게 통과
3. 동일 4개 maneuver bag의 strip/degraded/gap·diagnostics parity
4. read-only live shadow 주행에서 차로 이탈·반대 차로 회피·복귀 관찰
5. feature off/on closed-loop와 perception missing/stale/invalid fallback 통과
6. 대체 launch·문서·diagnostics 확정, 외부 참조 0과 rollback 경로 확인

삭제는 흡수 코드와 parity 기록을 먼저 merge한 뒤 별도 commit/PR로 수행한다.
이때도 canonical perception topic인 `/perception/lane/boundary_family`와
`/perception/bev/camera/*`를 삭제한다는 뜻은 아니다.

## Legacy independent 범위

- 입력: `/perception/lane/boundary_observations`
  (`asmc_msgs/LaneBoundaryObservationArray`)
- 출력:
  - `/planning/debug/camera_lane_strips` (`visualization_msgs/MarkerArray`)
  - `/planning/lane_observation/diagnostics`
    (`diagnostic_msgs/DiagnosticArray`)
- 좌표계: `base_link`, x 전방, y 좌측
- ODD: `shadow_two_way_v1`
- 경계 의미: 왼쪽부터 오른쪽 순서, white/yellow 색상, local tracking ID

`home_lane`, `opposing_lane`, 절대 차로 번호는 추정하지 않는다. 노란선의 좌우 위치와 현재
전역경로/주행 상태를 함께 해석해 의미를 부여하는 것은 판제 책임이다. adapter가 표시하는
`vehicle projection`은 consumer의 5m preview station에서 y=0이 어느 관측 띠 안에 있는지
나타내는 기하 정보일 뿐 home-lane 판정이 아니다. producer의 lateral-order
`reference_x_m`은 모든 경계가 공유하는 support에 맞춰 frame마다 달라질 수 있으며,
adapter는 한 frame의 경계들이 동일한 reference x를 공유하는지 검증한다.

첫 ODD에서는 인접한 white-yellow 또는 yellow-white 쌍만 차로 띠 후보로 허용한다. 폭,
공통 관측 길이, 좌우 순서와 source freshness가 맞지 않으면 해당 pair 또는 전체 cycle을
거부한다. partial/empty 관측은 장애가 아니라 명시적인 상태로 진단한다. 이 제한은 주차선과
안전지대 흰 표식을 곧바로 주행 차로 경계로 오인하는 것을 줄이기 위한 보수적 gate이며,
일반 다차로 계약은 아니다.

회피 중 노란 중앙선만 잠깐 가려지고 두 흰 outer boundary가 남은 경우에는 음영 왕복
2차로 ODD에서만 제한적인 fallback을 허용한다. 두 흰 선 사이 폭의 절반이 전 구간에서
`2.4~4.6m`, 폭 변화가 `0.8m` 이하이고 공통 support가 6m 이상일 때만 가운데를 임시
경계로 계산한다. 이 결과는 `degraded_inferred_center` WARN 상태이며 confidence/quality에
0.65를 곱한다. perception 원본 메시지에는 합성 경계를 넣지 않고, 한 경계 이하이거나
전체 가림이면 fallback하지 않는다.

### Canonical shared-family 소비 정책

- 입력: `/perception/lane/boundary_family` (`asmc_msgs/LaneBoundaryFamily`)
- 출력:
  - `/planning/debug/lane_boundary_family/strips`
    (`visualization_msgs/MarkerArray`)
  - `/planning/diagnostics/lane_boundary_family_adapter`
    (`diagnostic_msgs/DiagnosticArray`)
- 현재 read-only audit ROI: `x[0,15]m`; producer의 `valid_x`와 교집합만 사용
- `deployment_allowed=false`인 `shadow_two_way_v1` 후보만 수용

`/planning/debug/lane_boundary_family/strips`는 위 reference adapter가 수락한 구간을 보여주는
시각화 MarkerArray이지, 길이가 잘린 planning message가 아니다. 정본
`/perception/lane/boundary_family`에는 producer 전체 polyline과 sample별
`OBSERVED/STRUCTURE_INFERRED/INVALID`가 그대로 들어간다. 판제는 typed family에서 별도 horizon을
선택할 수 있지만 freshness·`INVALID` hard split·폭/support gate를 그대로 유지해야 한다.
15m 밖을 제어에 사용하려면 far-range 정확도와 연속 support를 별도 audit한 뒤 확장한다.

adapter는 shared-family의 공통 sample index를 직접 사용하며 raster로 되돌리거나 새 spline을
피팅하지 않는다. 인접한 두 sample 사이의 네 support 값(왼쪽 경계 두 점과 오른쪽 경계 두
점)을 함께 판정한다.

- 모두 `OBSERVED`: 정상 관측 segment
- 하나라도 `STRUCTURE_INFERRED`, `INVALID`는 없음: 주황색 degraded segment
- 하나라도 `INVALID`: 해당 interval을 출력하지 않고 앞뒤 segment를 즉시 분리

따라서 accepted same-frame family 구조로 복원한 주황 구간은 순간 단절을 줄이는 데 사용할 수
있지만, 직접 관측과 같은 등급은 아니다. candidate v2는 strict-accepted family의
`x[0,15]m` fitted sample을 주황 `STRUCTURE_INFERRED`로 유지했고, 현재 candidate v3는
`insufficient_local_multi_role_support` reject에만 bounded distributed-support gate를 더한다.
둘 이상의 same-frame 역할이 각 2m 이상이고 union 6m 이상이며 heading/residual/outlier/곡률
조건을 모두 통과해야 한다. 한 역할 복원과 temporal/map 보간은 여전히 없다. 국소 inferred
segment는 두 경계 confidence의 최솟값에
`0.60`을 곱한다. 한 역할 전체가 producer에서 이미 inferred로 강등됐다면 같은 계수를 두 번
곱하지 않는다. `INVALID`에는 centerline이나 strip 면을 만들지 않으므로 붉은 공백을 가로질러
계획 후보가 이어질 수 없다.

`observed/degraded`는 geometry의 출처·품질 상태이지 home/opposing 판정이 아니다. y=0이
어느 띠 안에 있는지도 진단용 기하 정보일 뿐, 차량이 중앙 경계 위에 있으면 두 띠가 모두
겹칠 수 있다. 실제 주행 차로 의미와 사용 우선순위는 route/Frenet/MGeo 문맥을 가진 판제가
결정해야 한다.

producer는 `INVALID` sample의 수학적 polyline 좌표도 버리지 않는다. 2026-09-15의 시간 제한
결정으로 accepted family의 `x[0,15]m` 안에서는 producer가 이를
`STRUCTURE_INFERRED`로 명시 승격한다. 이 정책은 동일 frame의 구조만 사용하고 map/이전
frame을 쓰지 않으며, `OBSERVED`로 위장하지 않는다. 15m 밖의 잔여 invalid와 family reject는
계속 분할·제외되고 adapter 자체에는 임의 승격 옵션이 없다. node는 여전히 read-only이고
기본 제어 권한도 없다.

`x[0,15]m`는 이 adapter의 bounded 검증 설정이며 perception 출력의 고정 유효거리나 최종
판제 lookahead 계약이 아니다. producer는 설정 범위 전체에서 sample 상태를 보존한다. 향후
판제 계약은 frame별 연속 trusted interval, 중간 gap, 최대 candidate horizon을 함께 해석해야
하며, 모델 개선으로 유효거리가 늘면 설정과 gate를 새 audit 결과로 갱신한다.

## 빌드와 실행

```bash
cd /root/ws
catkin_make --only-pkg-with-deps lane_boundary_adapter
source devel/setup.bash
roslaunch lane_boundary_adapter shadow_lane_consumer_pilot.launch
```

인지 pipeline은 별도 터미널에서 실행한다.

```bash
roslaunch asmc_perception perception.launch \
  camera_checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt
```

shared-family read-only adapter는 ROS Noetic 컨테이너의 별도 터미널에서 실행한다.
기본값으로 `base_link` 전용 RViz preset도 함께 열린다.

```bash
cd /root/ws
source devel/setup.bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch
```

GUI가 필요 없는 bag replay·성능 측정에서는 명시적으로 끈다.

```bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch rviz:=false
```

검사:

```bash
rostopic hz /perception/lane/boundary_family
rostopic echo /planning/diagnostics/lane_boundary_family_adapter -n1
```

RViz에서 `MarkerArray` display를 추가하고
`/planning/debug/camera_lane_strips`를 선택한다. 녹색은 y=0 vehicle projection을 포함한
관측 띠, 파란색은 나머지 관측 띠다. 추정 중앙선을 사용한 degraded 띠는 주황색과
`[DEGRADED inferred center]` label로 구별한다. 이것은 경로 선택이나 제어 권고가 아니다.

자동 RViz preset의 Fixed Frame은 `base_link`다. shared-family adapter의
`/planning/debug/lane_boundary_family/strips`와 producer의
`/perception/lane/debug/shared_family/markers`가 기본으로 겹쳐 표시된다. 청록 segment는
observed, 주황 segment는 structure-inferred다. invalid 구간은 빨간 선으로 연결하지 않고
실제 공백으로 표현하며 producer marker에서 그 근거를 비교할 수 있다. 기존 independent
경계 marker는 preset에 포함하되 기본 비활성화했다.

producer marker는 전체 fitting 범위의 세 경계와 support 상태를 보여주므로 adapter strip보다
앞뒤로 길 수 있다. strip은 `x[0,15]m` 안에서 인접 경계 두 개가 동시에 usable한 interval만
면으로 만든다. 두 표시의 길이 차이는 의도된 품질 gate이며 인지 메시지의 정보 손실이 아니다.

같은 MarkerArray에는 실제 차량 제원의 반투명 파란 footprint가 함께 표시된다. `base_link`는
후륜축 중앙의 빨간 점이고, 주황 가로선은 후륜축 `x=0`, 녹색 가로선은 전륜축 `x=3.0m`다.
차체 범위는 `x[-0.790,3.845]m`, `y[-0.946,0.946]m`이며, 인지 입력이 stale/invalid라 차선
segment가 지워져도 차량 좌표 기준은 남는다.

## 실패 안전 계약

다음 상태에서는 marker를 비우고 diagnostics를 WARN/ERROR로 낸다.

- 입력 수신이 0.30초 이상 끊김
- source timestamp가 0, 0.30초 이상 오래됨, 또는 허용치보다 미래임
- `frame_id`, ODD, algorithm ID, expected count 불일치
- producer가 `valid=false`로 표시
- completeness와 실제 배열 길이 불일치
- 중복 tracking ID, 비연속 lateral order, 역전·교차 polyline
- 공통 길이, 차로폭 또는 폭 변화 기준 미충족

typed 토픽 도입 직후 만든 과거 smoke bag은 spline 표시용 polyline 꼬리가 선언된
`valid_x` 밖까지 남아 있다. adapter는 호환을 위해 메시지를 받을 수 있지만 strip 계산에는
선언된 `valid_x` 안쪽만 사용한다. 현재 producer는 새 메시지의 polyline도 그 구간으로 잘라
내보내도록 회귀 테스트를 둔다.

bag replay에서는 `/use_sim_time=true`와 `rosbag play --clock`을 사용해야 source freshness가
올바르게 평가된다. **모든 노드를 시작하기 전에** `/use_sim_time=true`를 설정해야 한다.
노드 기동 뒤 바꾸면 기존 프로세스가 wall time을 계속 사용해 과거 bag source stamp를 stale로
거부할 수 있다. independent와 family 파라미터는 각각
`config/shadow_lane_consumer.yaml`, `config/shadow_boundary_family_consumer.yaml`에 있으며
변경 근거 없이 판제 코드에 숫자를 복제하지 않는다.

## 검증 기준선

### 2026-09-12 최초 typed smoke

- C++ core: 12개 geometry/contract 단위 테스트 통과
- 당시 구 `drivable_bev` typed producer: 80개 테스트 통과
- launch smoke: 3개 경계에서 2개 strip, y=0을 포함한 strip index 1 생성 확인
- 실제 Foggy 11시 `shadow_clear` typed smoke bag 1배속 replay:
  - bag 기록 240 frame, 마지막 측정에서 subscriber 239 frame 수신
  - valid strip 196 frame (`82.0%`)
  - `order_invalid` 5 frame
  - `no_usable_strip` 38 frame
  - frame/domain/algorithm/array/boundary 계약 오류 0 frame
  - replay 종료 0.30초 뒤 receive-stale 전환과 marker 삭제 확인

위 수치는 한 run의 consumer smoke 결과이지 카메라 정확도나 판제 사용 가능률의 최종
합격 기준이 아니다. result bag의 첫 publisher 연결 시점에 따라 첫 frame 한 개가 빠질 수
있어, 전체 12-run 수락률은 최신 producer로 raw bag을 다시 처리하면서 별도 audit한다.

### 2026-09-13 회피·차로 이탈 replay

중앙선 straddle, lateral sweep, 정적 회피·복귀 Sunny/Foggy 4개 bag으로 고정 5m 순서
계약, 공통 support 계약, tracking 보강을 순차 비교했다.

- 공통 support 기준점 적용 후 네 run 모두 `order_invalid=0`
- motion-aware tracker 적용 후 straddle/lateral/Sunny 회피·복귀의 stable ID 수가 각각
  `5→4`, `12→8`, `10→4`
- C++ core 16개 geometry/contract test 통과
- 최종 result replay에서 1,258 message 수신
  - 정상 관측 strip: 1,090 (`86.65%`)
  - ODD 제한 degraded inferred center: 131 (`10.41%`)
  - usable 합계: 1,221 (`97.06%`)
  - no usable strip: 37 (`2.94%`)
  - frame/domain/algorithm/array/boundary/order 오류: 0

degraded 수치는 실제 노란선을 인식한 것으로 계산하지 않는다. 판제는 WARN 상태와
`inferred_center_boundary`를 확인하고, MGeo/이전 상태/LiDAR와 교차검증한 뒤에만 사용할 수
있다. 완전 가림에서도 계속 차로가 나온다는 보장은 없으며 2.94% 무관측은 명시적으로
보존한다.

### 2026-09-15 shared-family read-only adapter smoke

신규 core는 기존 independent 16개와 별도로 11개 contract/geometry 단위 테스트를 통과했다.
테스트에는 local inferred confidence 강등, producer 측 전체 강등의 이중 스케일 방지,
`INVALID` 앞뒤 hard split, producer valid range clip과 ODD/representation/deployment 거부가
포함된다.

Sunny 13시 중앙선 straddle raw bag을 11312 격리 master에서 1배속 재생했다. 올바른
replay-clock 조건에서 candidate 398개를 모두 usable로 변환했고 contract/geometry 오류는
0개였다. 이 run은 모든 frame에 일부 inferred sample이 있어 cycle 상태는 모두
`degraded_usable`이었다. 마지막 frame에는 두 차로 띠가 총 9개 segment(관측 4, inferred 5)로
나뉘고 62개 invalid pair-interval이 출력에서 제외됐다. 10초간 MarkerArray 출력은 약
`10.1~10.6Hz`였으며 replay 종료 뒤 receive-timeout WARN과 명시적 marker clear를 확인했다.

이 수치는 shared family의 인지 정확도를 새로 평가한 것이 아니라 consumer가 support 상태를
손실 없이 보수적으로 해석하는지 확인한 통합 smoke다. `deployment_allowed=false`,
`read_only=true`, `publishes_control=false`는 diagnostics에 계속 기록된다.

## 다음 통합 단계

1. 같은 4개 maneuver bag에서 independent pilot과 shared-family adapter의 usable/degraded/gap,
   strip 연속성과 full-stack 지연을 나란히 기록한다.
2. live shadow ODD에서 제어 미연결 상태로 경계 이탈·반대 차로 진입·복귀를 관찰한다.
3. 판제 파트와 `home/opposing`이 아닌 N-boundary/N-segment 요구사항, inferred penalty와
   invalid 시 fallback을 합의한다.
4. 합의 후에만 feature flag가 있는 planner 입력 adapter 또는 canonical derived message를
   추가한다.
5. MGeo/전역경로와 stale policy를 포함한 closed-loop shadow test를 통과한 뒤 production
   승격 여부를 결정한다.

그 전까지 이 패키지의 출력은 시각화와 계약 검증용이며 차량 제어 입력으로 사용하면 안 된다.
