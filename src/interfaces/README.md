# ASMC ROS Interface Specification

> **문서 역할:** Canonical — 현재 구현된 팀 내부 ROS 계약
> **인지 확장 작업안:** [인지 융합 로드맵 §6](../../docs/perception-map-fusion-roadmap.md#6-working-topicinterface-proposal)
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
| `/behavior/context` | Behavior/FSM | Frenet | `asmc_msgs/BehaviorContext` | 선택된 행동과 실행 target |
| `/planning/trajectory` | Frenet | MPC | `asmc_msgs/Trajectory` | 최종 추종 trajectory |
| `/planning/feedback` | Frenet | FSM | `asmc_msgs/PlanFeedback` | planning 및 행동 실행 결과 |
| `/ctrl_cmd` | MPC | MORAI UDP Bridge | `morai_msgs/CtrlCmd` | 차량 제어 명령 |

실시간 topic은 기본 `queue_size=1`로 사용하고 이름은 launch/remap으로 변경 가능하게 한다.

## 3. Frame and units

- `VehicleState.header.frame_id`: `map`
- `DetectedObjectArray.header.frame_id`: `map`
- `Trajectory.header.frame_id`: `map`
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

Consumer는 message 존재 여부와 별도로 freshness를 검사한다.

초기 timeout 제안:

```yaml
timeouts:
  vehicle_state: 0.20
  objects: 0.30
  behavior: 0.50
  trajectory: 0.25
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
- `ego_at_plan`은 Planner가 trajectory 생성에 사용한 Ego snapshot이다 - **provenance 전용**이다. MPC의 실제 control 초기상태(x0)는 `ego_at_plan`이 아니라 MPC가 독립적으로 구독하는 최신 `/vehicle/state`에서 만든다(코너/선회 구간에서 planning과 control 사이 시간차를 줄이기 위함).
- MPC는 `dt`를 사용해 자신의 timestep으로 시간축 보간하고, `header.stamp`(trajectory 생성 시각)와 최신 `/vehicle/state.header.stamp`(현재 시각) 사이의 age만큼 reference 시작점을 앞으로 이동시켜 trajectory latency를 보정한다.
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

Fresh empty `DetectedObjectArray`는 정상적인 "객체 없음"이다. 미수신 또는 stale array를 빈 배열로 해석하지 않는다.

## 10. Runtime separation and compliance

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

## 11. Change control

- `asmc_msgs`가 executable contract다.
- 이 문서가 사람이 읽는 specification이다.
- Breaking change는 Planning, Behavior, Perception/Integration 담당 합의 후 반영한다.
- 변경 시 `CHANGELOG.md`를 함께 갱신한다.

## Appendix A. Camera·BEV interface working proposal

아래 항목은 camera live와 MGeo local-crop pilot 전 단계의 **비정본 후보**다. 아직
`asmc_msgs`나 다른 custom message로 구현되지 않았으며, 카메라 출력 범위·신뢰도·처리율과
Planning/LiDAR 소비 요구를 측정한 뒤 별도 Path B PR에서 승격한다.

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
[planning-control-udp.md](../../docs/planning-control-udp.md)를 따른다.

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
