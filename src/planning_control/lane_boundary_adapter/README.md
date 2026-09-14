# Lane Boundary Adapter

카메라 인지가 발행하는 typed metric 경계 관측을 판제 개발자가 확인할 수 있는 차로 띠
후보로 바꾸는 **read-only C++ pilot**이다. MORAI UDP 패킷을 직접 다루지 않는다. 데이터
흐름은 다음과 같다.

```text
MORAI sensors --UDP--> morai_sensor_bridge --ROS--> camera perception
  --/perception/lane/boundary_observations-->
  lane_boundary_adapter --ROS debug--> MarkerArray + DiagnosticArray
```

이 패키지는 `behavior_planner`, `frenet_planner`, `mpc_controller`, UDP 명령 송신 코드를
수정하거나 호출하지 않는다. `BehaviorCommand`, `Trajectory` 및 차량 제어 명령도 발행하지
않는다. 따라서 판제 파트가 현재 작업 중인 실행 경로와 독립적으로 빌드·검토할 수 있다.

## 현재 범위

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

## 빌드와 실행

```bash
cd /root/ws
catkin_make --only-pkg-with-deps lane_boundary_adapter
source devel/setup.bash
roslaunch lane_boundary_adapter shadow_lane_consumer_pilot.launch
```

인지 pipeline은 별도 터미널에서 실행한다.

```bash
roslaunch drivable_bev three_view_lane_pipeline.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt
```

검사:

```bash
rostopic hz /perception/lane/boundary_observations
rostopic echo /planning/lane_observation/diagnostics -n1
rviz
```

RViz에서 `MarkerArray` display를 추가하고
`/planning/debug/camera_lane_strips`를 선택한다. 녹색은 y=0 vehicle projection을 포함한
관측 띠, 파란색은 나머지 관측 띠다. 추정 중앙선을 사용한 degraded 띠는 주황색과
`[DEGRADED inferred center]` label로 구별한다. 이것은 경로 선택이나 제어 권고가 아니다.

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
올바르게 평가된다. 파라미터는 `config/shadow_lane_consumer.yaml`에 있으며 변경 근거 없이
판제 코드에 숫자를 복제하지 않는다.

## 검증 기준선

### 2026-09-12 최초 typed smoke

- C++ core: 12개 geometry/contract 단위 테스트 통과
- typed producer를 포함한 `drivable_bev`: 80개 테스트 통과
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

## 다음 통합 단계

1. RViz에서 4개 maneuver bag의 정상/degraded/no-observation 전환과 false strip을 수동 검수한다.
2. 판제 파트와 `home/opposing`이 아닌 일반 N-boundary/N-strip 입력 요구사항을 합의한다.
3. 합의 후에만 별도 canonical derived message/topic을 `asmc_msgs`에 추가한다.
4. feature flag가 있는 shadow-ODD adapter로 behavior/planner에 연결한다.
5. MGeo/전역경로, 차량 상태와 stale policy를 포함한 closed-loop shadow test를 거친다.

그 전까지 이 패키지의 출력은 시각화와 계약 검증용이며 차량 제어 입력으로 사용하면 안 된다.
