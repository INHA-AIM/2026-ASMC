# MPC Controller

> **역할:** ROS1 trajectory tracking runtime + transport-independent MPC core

Planning 전체 경계는 [Planning/Control 개발 가이드](../../docs/packages/planning-control/README.md),
시간·frame·fail-safe 계약은 [ROS interface](../interfaces/README.md)를 따른다.

## 현재 I/O

| 방향 | Topic | Type | 의미 |
|---|---|---|---|
| subscribe | `/planning/trajectory` | `asmc_msgs/Trajectory` | Frenet reference |
| subscribe | `/vehicle/state` | `asmc_msgs/VehicleState` | 유일한 현재 x0 |
| subscribe | `/behavior/command_v2` | `asmc_msgs/BehaviorCommand` | emergency override |
| publish | `/ctrl_cmd/normal` | `morai_msgs/CtrlCmd` | Mode selector가 NORMAL에서 `/ctrl_cmd`로 전달 |

`Trajectory.ego_at_plan`은 trajectory provenance 검증용이며 현재 vehicle state로 재사용하지
않는다. reference start offset은 trajectory와 최신 vehicle source time의 차이로 계산한다.

## Fail-safe

다음 경우 해당 control cycle은 STOP command로 수렴한다.

- trajectory 또는 vehicle state missing/stale
- invalid·STOP trajectory
- reference interpolation 또는 연속성 검증 실패
- vehicle state 변환 실패
- MPC solver 실패

`BehaviorCommand::EMERGENCY_STOP`은 일반 solve보다 우선하는 emergency brake override다.
wheelbase와 steering limit는 Frenet, MPC와 MORAI bridge 설정에서 일치해야 한다.

## 속도에 따른 조향 변화 제한

`constraints/steering/rate_reference_speed_mps: 5.0`은 18 km/h 이하에서 기존
`max_rate: 1.0` rad/s를 사용하고, 그보다 빠르면 `max_rate * (5 / |speed|)^2`로
조향 변화 한계를 줄인다. 하한은 `min_rate: 0.1` rad/s다. 58 km/h에서는
한 제어 주기(0.05 s)에 최대 약 0.005 rad씩 변할 수 있다. 조향각 자체의
물리 한계는 그대로이며, 이 제한이 횡가속도나 충돌 회피를 보장하지는 않는다.

제한은 solver의 초기 해와 모든 최적화 단계에 동일하게 적용한다. 따라서
출력에만 필터를 추가해 예측 궤적과 실제 명령이 달라지는 문제를 피한다.
현재 속도를 기준으로 한 번의 solve 전체에 같은 한계를 사용한다. 유효한
Planner STOP은 마지막 정상 경로의 위치·방향·곡률을 저장해 두었다가, 현재 위치와
제동 속도에 맞게 다시 샘플링해 조향을 계산한다. 같은 조향 변화 한계를 적용하고,
제동 페달은 기존 0.3을 유지한다. 계산에는 YAML의 페달 응답표에 따른 감속을 고정하므로
정지 중 가속이나 stuck recovery를 요청하지 않는다. 경로 끝을 넘어서는 제동 예측은
마지막 곡률을 이어 쓰는 제한적인 대안이며, 아직 보지 못한 도로 모양을 보장하지 않는다.
차량이 정지하거나 센서/경로 fail-safe가 발생하면 저장 경로를 지운다. 저장 경로가 없거나
계산이 실패하면 기존 조향을 속도별 변화 한계 안에서 0 방향으로 줄인다.
매 STOP 주기마다 실제 발행한 조향과 시각을 갱신한다. 긴급 제동 경로는 유지한다.
비교/rollback 시 reference speed를
`0.0`으로 설정하면 기존 고정 변화 한계를 사용한다.

`mpc/steering_response_delay_s: 0.15`는 조향 명령부터 IMU 방향 변화 관측까지의
유효 지연을 모델링한다. 최근 발행한 조향 명령을 보관하고, 예측·선형화·gradient
평가가 같은 지연된 조향을 사용한다. 가속·제동 명령을 지연시키는 기능은 아니다.
센서 stale·invalid 및 긴급 제동에서는 이력을 지운다. 유효한 Planner STOP에서는
최적화 초기값과 가속 상태만 초기화하고, 이미 발행한 조향 이력에 실제 STOP 조향을
추가한다. 이력은 설정된 지연 길이만큼만 남기므로 오래된 명령은 순서대로 제거된다.
다시 시작할 때 마지막으로 발행한 조향과 남아 있는 지연 이력을 사용한다.
설정을 `0.0`으로 바꾸면 기존 즉시 반응 모델로 비교할 수 있다.

`mpc/horizon: 29`, `dt: 0.05`는 1.5초인 일반 KEEP 경로와 관측된 source-time
offset에 맞춘 설정이다. 마지막 참조 시각은 offset 전 1.40초다. 이전 36-step
설정은 경로 끝의 위치를 약 0.3초 반복하면서 목표 속도는 유지하는 입력을 만들었다.
더 긴 예측을 쓰려면 실제 입력 경로 길이와 offset부터 함께 검증해야 한다.
이 설정이 임의의 지연이나 모든 짧은 특수 경로를 보장한다는 뜻은 아니다.
조향 변화 비용 `cost/control_rate`는 100에서 110으로 조정했다. 예측 길이를
줄인 설정은 GPS 흔들림 모사 회귀 테스트와 시뮬레이터 주행을 함께 확인한다.

새 경로가 가속을 요구할 때도 이전의 약한 가속 해를 계속 재사용하지 않도록,
참조 속도의 시간 미분으로 만든 가속 초기값과 이전 해를 비교한다. 조향 이력과
직전 명령 기준은 유지하며, 전체 예측 비용이 1% 이상 낮아질 때만 새 초기값을
택한다. 기존 감속 초기값 비교도 같은 경로로 유지한다.

## 빌드·실행

```bash
cd /root/ws
./scripts/build_ws.sh
source devel/setup.bash
roslaunch asmc_controller controller.launch
```

MPC 단독 실행은 입력 누락으로 STOP을 내는 fail-safe 확인에는 유효하지만 차량 추종 검증은
아니다. full chain은 `asmc_planner/autonomy_no_bridges_v2.launch`를 사용한다.

## 검증

```bash
catkin_make run_tests_asmc_controller
catkin_test_results build/test_results/asmc_controller
```

최소 회귀 범위는 message conversion, trajectory timing, reference validation, steering
feed-forward와 controller state다. unit test 통과와 MORAI closed-loop 주행은 별도 결과로
보고한다.

## 곡선 중앙 추종 비용

`cost.contour_error`는 직선에서 6.0을 유지한다. `curve_contour_boost` 6.0은
참조 경로 각 표본의 절대 곡률이 0.004~0.012 /m 사이일 때 선형으로 더해져,
급한 쪽 곡선에서는 총 12.0이 된다. 좌우 곡선에 같은 규칙을 사용한다.
현재 측정 위치 오차를 이 가중치 선택에 쓰지 않으며, 조향 속도/각도 제한,
지연 모델, terminal 비용과 정지 명령 우선순위는 그대로다.

정적 비용을 전체 12.0으로 높인 후보는 고속 GPS 잡음 회귀에서 탈락했으므로
사용하지 않는다. 곡률별 비용은 `MPC/Cost`의 순수 함수가 소유하고 YAML 로드는
`Util/ParameterLoader`가 담당한다. 새 세 키가 누락되거나 유효하지 않으면
boost를 끈다. `curve_contour_boost: 0.0`으로 기존 비용에 복귀할 수 있다.

관련 시험: `tracking_error_test`, `parameter_loader_test`,
`straight_stability_test`(고속 GPS 잡음·좌우 곡선·지연·제동 추종).
