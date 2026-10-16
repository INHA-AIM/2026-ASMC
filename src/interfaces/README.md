# ASMC ROS Interface Specification

> **문서 역할:** Canonical — 현재 구현된 팀 내부 ROS 계약
> **인지 확장 작업안:** [인지 융합 로드맵 §6](../../docs/architecture/perception-roadmap.md#6-working-topicinterface-proposal)
> **문서 허브:** [docs/README.md](../../docs/README.md)

이 문서는 `2026-ASMC` 내부 ROS 모듈 사이의 사람이 읽는 인터페이스 정본이다.
실제 실행 계약은 `asmc_msgs`가 제공하며, 두 정의는 함께 변경한다.

## 1. Architecture boundary

```text
MORAI / competition environment
        │ UDP and allowed sensor I/O
        ▼
Bridge / State / Perception
        │ canonical ROS topics
        ▼
Behavior/FSM → Frenet Planner → MPC Controller
```

- MORAI와 팀 PC의 외부 경계는 UDP를 유지한다.
- 팀 PC 내부 모듈은 ROS topic으로 통신한다.
- Behavior/FSM은 행동을 결정한다.
- Frenet은 명령받은 행동에 맞는 안전한 trajectory를 생성한다.
- MPC는 trajectory를 추종하여 `/ctrl_cmd`를 발행한다.
- 알고리즘 내부 구조체와 ROS message는 node boundary의 converter로 분리한다.

## 2. Canonical topics

| Topic | Publisher | Subscriber | Message | Purpose |
|---|---|---|---|---|
| `/vehicle/state` | State Estimator | FSM, Frenet, MPC | `asmc_msgs/VehicleState` | 통합 Ego state |
| `/perception/objects` | Perception | FSM, Frenet | `asmc_msgs/DetectedObjectArray` | 센서 기반 객체 상태와 형상 |
| `/perception/lane/boundary_observations` | Camera Perception | FSM/Frenet adapter | `asmc_msgs/LaneBoundaryObservationArray` | metric 차선 경계 관측; 현재 음영구간 provisional |
| `/perception/lane/boundary_family` | Camera Perception | planning candidate adapter | `asmc_msgs/LaneBoundaryFamily` | 공동 피팅 3경계와 sample별 support 상태; non-deployment 후보 |
| `/perception/bev/camera/drivable_probability` | Camera Perception | planning candidate adapter | `sensor_msgs/Image` | `base_link` metric BEV의 mono8 drivable score; coverage와 함께 사용 |
| `/perception/bev/camera/coverage` | Camera Perception | planning candidate adapter | `sensor_msgs/Image` | drivable cell의 camera 관측 유효성; 0은 unknown |
| `/behavior/command_v2` | Behavior/FSM | Frenet, MPC | `asmc_msgs/BehaviorCommand` | 현재 maneuver transaction과 target |
| `/planning/frenet_feedback_v2` | Frenet | Behavior/FSM | `asmc_msgs/FrenetFeedback` | 현재 transaction 실행 결과 |
| `/planning/trajectory` | Frenet | MPC | `asmc_msgs/Trajectory` | 최종 추종 trajectory |
| `/ctrl_cmd` | MPC | MORAI UDP Bridge | `morai_msgs/CtrlCmd` | 차량 제어 명령 |
| `/behavior/context` | Behavior/FSM | Frenet | `asmc_msgs/BehaviorContext` | v1 compatibility command |
| `/planning/feedback` | Frenet | FSM | `asmc_msgs/PlanFeedback` | v1 compatibility feedback |

실시간 topic은 기본 `queue_size=1`로 사용하고 이름은 launch/remap으로 변경 가능하게 한다.

## 3. Frame and units

- `VehicleState.header.frame_id`: `map`
- `DetectedObjectArray.header.frame_id`: `map`
- `Trajectory.header.frame_id`: `map`
- `LaneBoundaryObservationArray.header.frame_id`: `base_link`
- `LaneBoundaryFamily.header.frame_id`: `base_link`
- 위치와 길이: `m`
- 각도: `rad`
- 곡률: `1/m`
- 속도: `m/s`
- 가속도: `m/s^2`
- yaw rate: `rad/s`

MORAI raw 단위 변환은 Bridge/State 계층에서 끝내고 algorithm message에는 SI 단위만 전달한다.

## 4. Timestamp

`header.stamp`는 callback 수신 시각이 아니라 메시지가 표현하는 source/reference time이다.

- Vehicle state: 상태 추정 기준 시각
- Objects: OBB와 motion state가 생성된 센서 기준 시각
- Behavior: command가 결정된 시각
- Trajectory: planning에 사용한 snapshot의 기준 시각
- Feedback: planning 결과가 생성된 시각
- Lane boundaries: 세 카메라 model-grid 입력 묶음의 최신 source 시각

Consumer는 message 존재 여부와 별도로 freshness를 검사한다.

초기 timeout 제안:

```yaml
timeouts:
  vehicle_state: 0.20
  objects: 0.30
  behavior: 0.50
  trajectory: 0.25
  lane_boundaries: 0.30
```

수치는 실측 후 조정할 수 있지만 freshness 검사 자체는 필수 계약이다.

## 5. Public behavior enum

```text
KEEP=0
FOLLOW=1
LEFT_CHANGE=2
RIGHT_CHANGE=3
STOP=4
YIELD=5
EMERGENCY_STOP=6
AVOID=7
MERGE=8
```

Public enum과 각 알고리즘의 내부 enum은 서로 다른 타입이다. Converter는 명시적인 `switch`로 매핑하며 숫자 `static_cast`를 사용하지 않는다.

횡방향 부호:

```text
target_lane_offset: +1 LEFT, 0 CURRENT, -1 RIGHT
Frenet d:           +d LEFT,             -d RIGHT
```

## 6. Behavior command rules

현재 v2 transaction, validation과 lifecycle의 정본은
[FSM↔Frenet v2 contract](../../docs/contracts/planning-fsm-frenet-v2.md)다. 아래
`BehaviorContext` 규칙은 v1 compatibility 경로에 적용한다.

`BehaviorContext`는 판단 과정이나 score가 아니라 선택된 행동과 실행 target만 전달한다.

- 같은 행동과 target을 반복 발행하는 동안 같은 `command_id`를 유지한다.
- 행동 또는 target이 의미 있게 바뀌면 `command_id`를 증가시킨다.
- 동일 ID에 서로 다른 내용이 들어오면 consumer는 invalid command로 취급한다.
- 늦게 도착한 과거 command는 timestamp/freshness와 ID를 함께 검사해 사용하지 않는다.
- `force_stop`은 정상 planning보다 우선한다.
- Optional target은 값 자체가 아니라 대응하는 `has_*`로 적용 여부를 판단한다.

KEEP의 최종 속도는 다음 정책을 따른다.

```text
curve_safe_speed = Frenet LookaheadTargetSpeed()
requested_speed  = desired_speed if present, otherwise planner default
target_speed     = min(curve_safe_speed, requested_speed)
if max_speed is present: target_speed = min(target_speed, max_speed)
```

## 7. Object geometry

`DetectedObject.obb_corners[8]`은 Perception이 계산한 authoritative collision geometry다.

```text
Geometry = obb_corners[8]
Motion   = vx / vy / optional yaw_rate
Metadata = center / yaw / size / ID / type
```

Planner는 corner를 `center + yaw + width + length`로 다시 생성하거나 OBB를 재-fitting하지 않는다. 2D collision은 전달된 corner를 XY로 투영하고 convex hull을 만든 뒤 SAT로 검사한다.

> **Interface freeze blocker:** `corner[0]`부터 `corner[7]`의 순서는 실제 LiDAR/Perception 출력과 대조한 뒤 확정한다. Planner가 순서를 임의로 정의하지 않는다. 순서가 확정되기 전까지 실제 LiDAR OBB 연결을 완료로 간주하지 않는다.

## 8. Trajectory semantics

Trajectory point는 `x`, `y`, `yaw`, `curvature`, `velocity`, `acceleration`을 포함한다.

- `dt`는 Planner source trajectory의 sample time이다.
- `header.stamp`는 **callback 수신 시각이나 publish 시각이 아니라 이 trajectory가
  대표하는 source/reference time**이다(4절 timestamp 규칙과 동일). 현재 Frenet
  구현에서는 이 trajectory를 만드는 데 사용한 `ego_at_plan`(VehicleState) 스냅샷의
  `header.stamp`와 항상 같다 - `ConvertCartesianPath`/`MakeStopTrajectory`는 별도
  timestamp 인자를 받지 않고 `ego_at_plan.header.stamp`를 그대로 쓴다. `ego_at_plan`이
  알 수 없는 상태(zero stamp, `valid=false`)라면 `header.stamp`도 그 상태를 그대로
  반영한다 - `ros::Time::now()`로 유효한 것처럼 위장하지 않는다.
- **동일 trajectory를 재발행(republish)할 때**(예: 완료 latch가 이전 궤적을 계속
  내보내는 경우) point content가 바뀌지 않았다면 `header.stamp`도 원래 값 그대로
  유지한다. 재발행 수신 시각은 새 source time이 아니다.
- `ego_at_plan`은 Planner가 trajectory 생성에 사용한 Ego snapshot이다 - MPC 관점에서는
  **provenance 검증 전용**이다. MPC의 실제 control 초기상태(x0)는 `ego_at_plan`이
  아니라 MPC가 독립적으로 구독하는 최신 `/vehicle/state`에서 만든다(코너/선회 구간에서
  planning과 control 사이 시간차를 줄이기 위함).
- MPC는 `dt`를 사용해 자신의 timestep으로 시간축 보간하고,
  `최신 /vehicle/state.header.stamp - trajectory.header.stamp`만큼 reference 시작점을
  앞으로 이동시켜 trajectory latency를 보정한다(둘 다 source time이므로 같은
  timebase). Receive freshness는 이 계산과 분리된 별개의 `SteadyTime` watchdog으로
  판정한다.
- 안전 STOP trajectory는 Controller가 실행할 수 있는 유효 명령이므로 `valid=true`이고, 원인은 `failure_reason`에 남긴다.
- `valid=false`는 points 자체를 추종하면 안 되는 경우에 사용한다.

## 9. Fail-safe

| Condition | Required behavior |
|---|---|
| Vehicle state missing/invalid/stale | Frenet STOP |
| Vehicle state missing/invalid/stale (MPC) | MPC STOP |
| Behavior missing/invalid/stale | Frenet STOP |
| `force_stop=true` | Frenet STOP |
| Objects missing/invalid/stale | 기본 STOP; 별도 합의 후에만 degraded mode |
| Collision blocked | 과거 path 재사용 금지, 즉시 STOP |
| Transient generation failure | 최대 5 planning cycles 이전 valid path 재사용 |
| Transient failure grace 초과 | STOP |
| Trajectory missing/invalid/stale | MPC STOP |
| Lane boundaries missing/invalid/stale | 카메라 경계를 사용한 차로 추론 금지; 유효 MGeo/route로 fallback하거나 보수 상태 유지 |

Fresh empty `DetectedObjectArray`는 정상적인 "객체 없음"이다. 미수신 또는 stale array를 빈 배열로 해석하지 않는다.

## 10. Lane-boundary observation contract

`LaneBoundaryObservationArray`는 카메라가 실제로 관측한 N개의 경계를 `base_link`
(x-forward, y-left) metric polyline으로 전달한다. 현재 producer의 적용 범위는
`operating_domain=shadow_two_way_v1`이며, 이 문자열과 route/ODD 상태가 맞지 않으면
consumer가 사용해서는 안 된다.

- 원소는 공통 `reference_x_m`을 모두 통과할 때만 `order_valid=true`이고, 그때
  `lateral_order=0`이 가장 왼쪽(+y)이다.
- `color`는 white/yellow 관측 속성이다. 중앙선, home/opposing, lane 1/2/3 같은 의미가 아니다.
- `tracking_id`는 publisher process 안의 short-term ID다. replay epoch 변화, 긴 누락,
  재시작 뒤에는 재사용될 수 있으므로 전역 ID로 저장하지 않는다.
- `semantic_score`와 `geometric_quality`는 서로 다른 heuristic evidence이며 calibrated
  probability가 아니다. consumer가 두 값을 임의로 곱해 안전 확률로 해석하지 않는다.
- `source_mask`는 front=1, left=2, right=4 bitmask다.
- `valid=true, completeness=EMPTY`는 동기화 cycle은 처리됐지만 유효 경계가 없다는 뜻이다.
  미수신/stale 또는 `valid=false`와 다르다.
- `PARTIAL`은 정상적인 관측 상태다. 현재 음영구간 producer의 `COMPLETE` 기준은 노란 중앙
  경계와 양쪽 흰 outer 후보 3개이며, 일반 도로의 고정 경계 수 계약이 아니다.

판제는 경계 색상과 좌우 순서, route/Frenet/MGeo 문맥을 함께 사용해 차로 의미를 정한다.
인지 메시지만으로 역주행, 원래 차로, 복귀 방향을 확정하지 않는다. 이 typed 계약은
provisional executable contract이고, 전 구간 topology와 lane-center 계약은 후속 합의다.

### 10.1 Shared boundary-family candidate

`/perception/lane/boundary_family`는 위 independent 관측 계약을 교체하지 않는 별도 후보
인터페이스다. 구현 기법인 `shared_family`를 토픽 이름에 넣지 않고 결과 의미인
`boundary_family`를 사용하므로, 향후 fitting 구현이 바뀌어도 동일 의미 계약을 유지할 수
있다. 현재 `operating_domain=shadow_two_way_v1`, `deployment_allowed=false`이며 판제
closed-loop gate 전에는 authoritative 입력으로 사용할 수 없다.

- 세 원소의 `role`은 왼쪽 흰 outer, 노란 road-centre, 오른쪽 흰 outer의 기하 순서다.
  home/opposing/절대 차로 번호는 아니다.
- `polyline`과 `support_state`는 길이가 같으며 arc-length 순서로 대응한다.
- `SUPPORT_OBSERVED`는 해당 역할의 직접 evidence가 지지한다.
- `SUPPORT_STRUCTURE_INFERRED`는 accepted same-frame family의 공동 normal-offset 구조로
  만든 강등된 **사용 가능 후보**다. 디버그 화면의 주황색 구간이며 temporal prediction이나
  map 보간이 아니다. 현재 시간 제한 shadow pilot에서는 family acceptance gate를 먼저
  통과한 frame에 한해 `x[0,15]m` 안의 local support 공백도 이 상태로 유지한다.
- `SUPPORT_INVALID`는 local support가 없는 붉은 구간이다. 좌표 자체는 원인 audit과 향후
  명시적 opt-in 저권한 preview/fallback 실험을 위해 보존하지만 **인지 관측을 뜻하지
  않는다**. 현재 bounded completion 밖(주로 15m 이후)에서는 기본 consumer가 polyline을
  분할하며, 별도 정책 합의 없이 이 구간을 차로 경계나 corridor로 사용하거나
  `OBSERVED`로 재표기해서는 안 된다.
- `valid=false`는 fresh reject/error 결과이며 missing/stale과 구분한다. 이때 boundaries는
  비어 있고 `status_reason`이 원인을 설명한다.
- aggregate confidence와 residual은 calibration된 안전 확률이 아니다. 판제 후보 adapter는
  inferred를 observed보다 낮은 신뢰도로 다루고 freshness·ODD·config hash를 함께 검사한다.

기존 `/perception/lane/boundary_observations`와 C++ adapter는 그대로 유지한다. 별도
`lane_boundary_adapter/boundary_family_monitor_node`가 새 후보를 read-only strip/centerline
segment로 변환한다. 이 consumer는 네 endpoint support 중 하나라도 `INVALID`이면 해당
interval을 생략하고, `STRUCTURE_INFERRED`가 하나라도 있으면 segment를 degraded로 분리해
confidence를 낮춘다. 출력은 debug MarkerArray와 DiagnosticArray뿐이며 Behavior/Frenet/MPC나
제어 명령에는 연결하지 않는다.

```text
/planning/debug/lane_boundary_family/strips
/planning/diagnostics/lane_boundary_family_adapter
```

현재 candidate algorithm ID는
`twinlite_v6_shadow_boundary_family_candidate_v3`다. v2가 strict-accepted family의
`x[0,15]m` 수학 곡선만 `STRUCTURE_INFERRED`로 유지했다면, v3는 strict gate가 오직
`insufficient_local_multi_role_support`로 거부한 경우에만 보수적인 distributed-support gate를
추가한다. 둘 이상의 same-frame 역할이 각 2m 이상, 합집합 6m 이상이며 heading, residual,
outlier, 곡률 조건을 모두 통과해야 한다. 역할별 직접 support 밖은 계속 inferred이고 one-role
복원, temporal prediction과 map 보간은 없다. strict-accepted frame은 기존 결과와 동일하다.
실제 high-heading dropout의 activation/false-family 비율과 read-only adapter를 확인한 뒤에만
planner feature flag나 canonical derived message를 논의한다.

2026-09-15 v3 handoff에서는 lane-family 메시지와 파라미터를 동결했지만 제어 권한은
승격하지 않았다. drivable BEV 최소 계약과 장애물 회피 결합 순서, on/off 및 파트별 소유권은
[음영구간 카메라 인지 v3 판제 인수 가이드](../../docs/handoffs/camera-to-planning.md)를
따른다. 판제는 `coverage=0`인 drivable cell을 non-drivable로 확정하지 않고 unknown으로
보존해야 하며, LiDAR occupancy/object를 충돌 veto로 우선 적용한다.

## 11. Runtime separation and compliance

```text
competition:
  allowed sensor sources only
  no GT/oracle fallback

development:
  explicit adapters and mocks allowed
  source provenance required
```

`/Ego_topic`, `/Object_topic`, `/CollisionData`는 development adapter/validation에만 사용할 수 있다. Competition algorithm path는 이 topic으로 자동 fallback하지 않는다.

Legacy UDP와 ROS runtime은 같은 core library를 공유할 수 있지만 동시에 실행하지 않는다.

```text
Legacy: UDP Frenet + UDP MPC
ROS:    ROS Frenet + ROS MPC + MORAI UDP Bridge
```

## 12. Change control

- `asmc_msgs`가 executable contract다.
- 이 문서가 사람이 읽는 specification이다.
- Breaking change는 Planning, Behavior, Perception/Integration 담당 합의 후 반영한다.
- 변경 시 `CHANGELOG.md`를 함께 갱신한다.

## Appendix A. Camera·BEV interface working proposal

아래 raster 항목은 camera live와 MGeo local-crop pilot 전 단계의 **비정본 후보**다.
차선 경계 vector는 위 10절의 typed provisional 계약으로 구현됐다. 음영구간 v3는
`drivable_probability + coverage`의 정확한 grid와 Image envelope를 별도 인수 가이드에서
동결했지만, 이는 공통 BEV custom message 승격을 뜻하지 않는다. BEV raster의 범용 metadata와
전 구간 topology/centerline 메시지는 아직 정본 계약으로 확정하지 않았다.

### A.1 Camera image-plane

```text
/perception/camera/{view}/drivable/probability
/perception/camera/{view}/lane/probability
/perception/camera/{view}/debug/panel/compressed
/perception/camera/debug/mosaic/compressed
/perception/diagnostics
```

- `{view}` 후보: `front`, `left`, `right`
- probability 초기 후보: `sensor_msgs/Image`, `mono8` 0~255
- hard mask와 overlay는 독립 visualizer가 threshold로 파생한다.
- bbox/object detection은 segmentation과 다른 계약으로 분리한다.

### A.2 Common ego-BEV

```text
/perception/bev/map/{drivable,lane,route_corridor,stopline}
/perception/bev/camera/{drivable_probability,lane_probability}
/perception/bev/lidar/occupancy
/perception/bev/fused/{safe_drivable,lanes}
```

좌표계는 REP-103 `base_link`의 x-forward, y-left, z-up을 우선 후보로 한다. BEV 범위,
meter/pixel, ego origin, probability bit depth와 custom lane message는 아직 확정하지 않는다.
모든 최종 메시지는 timestamp, frame ID, source, age/validity를 표현해야 한다.

## Appendix B. Existing research drafts

다음 값은 LBC 또는 후속 학습 연계를 위해 남긴 연구 초안이며 현재 Planning/MPC나 공통
BEV 계약이 아니다.

### B.1 Trajectory τ draft

| 필드 | 기존 제안 | 확정 |
|---|---|---|
| 좌표계 | ego-centric 또는 map frame 중 하나 | ☐ |
| 표현 | waypoint `(x, y)` × N | ☐ |
| N | 5~10 (LBC: N_STEP=5) | ☐ |
| 시간 간격 | 0.1 s (10 Hz) | ☐ |
| 속도 프로파일 | MPC에 별도 전달 여부 | ☐ |

현재 Planning/Control 계약은 위 초안이 아니라 이 문서의 `asmc_msgs/Trajectory` 정의와
[Planning/Control 운영 규약](../../docs/operations/planning-control.md)을 따른다.

### B.2 LBC BEV tensor draft

| 필드 | 기존 제안 | 확정 |
|---|---|---|
| 크기 | 192×192 (LBC 기본) 또는 팀 합의 | ☐ |
| pixels_per_meter | morai-roach 설정과 동일 | ☐ |
| 채널 | road, lane, vehicle, pedestrian 등 | ☐ |
| 원점 | ego vehicle center | ☐ |

`192×192`는 기존 LBC 실험값이며 새 camera/map/LiDAR 공통 BEV 크기로 확정하지 않는다.

### B.3 Common log record candidate

```yaml
timestamp:
scenario_id:
ego: {x, y, yaw, speed}
sensors: {cameras, lidar, gps, imu}
gt_bev: ...
tau_expert: ...
tau_pa: ...      # optional
command: ...
mpc_cmd: ...
weather: ...
```

첫 versioned perception message를 추가할 때 관련 파트 합의와 함께
`interfaces/CHANGELOG.md`를 만들고 이후 breaking change를 기록한다.
