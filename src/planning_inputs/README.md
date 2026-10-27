# Planning input adapters

공용 LiDAR 관측과 차량 상태를 판제 내부 입력으로 연결한다. 객체 입력은
`/perception/lidar/planning_observations_map`의 `asmc_msgs/LidarPlanningObservation` 하나다.

```bash
roslaunch asmc_lidar lidar.launch
roslaunch asmc_planner autonomy_no_bridges_v2.launch
```

두 번째 명령은 차량 제어를 포함한다. 브릿지·카메라는 별도 실행한다. nested workspace를 source하지 않는다.
예전 `lidar_source_policy`·`objects_source` 선택은 없다. 일반 autonomy·시나리오·수집용 planner가
같은 `objects.launch` 기본 입력을 사용한다. `tracks_map`은 `LidarTrackingObservation`으로,
현재 어댑터가 받는 `LidarPlanningObservation`과 다르므로 토픽만 바꿔 연결하지 않는다.

## 객체 입력 계약

session/generation·source stamp·scan sequence·설정/보정 ID·원본 OBB·속도 validity·운동 상태를
검사해 `/planning/objects`로 변환한다. queue1·source age300ms·미래 허용50ms를 유지한다.
정상 빈 결과는 전달하고 무효/stale/pose 부족은 빈 객체로 만들지 않고 발행을 보류한다.
TTC·추종·정지·회피·센서 장애 정책은 판제에 있다. 인지 필터·속도 계산·JSON join은 수행하지 않는다.

원본 클래스/그룹은 공용 관측에 있고 private numeric enum의 unknown은 객체 부재를 뜻하지 않는다.
GPS 없는 local 객체를 global Frenet에 이름만 remap하지 않는다.
[속도 의미·현재 제한·검증](../asmc_lidar/docs/planning-runtime.md)을 따른다.

## 차량 상태 계약

Private planning state keeps source-time GPS/IMU propagation and the 0.35 m
antenna lever arm; all three planners share /planning/vehicle/state.

The MORAI profile publishes position and yaw in the same UTM zone 52N map
basis. Its raw IMU yaw is true ENU, so
`orientation/imu_yaw_is_true_enu: true` applies the local projection rotation
before lever-arm removal and pose propagation. At K-City this is approximately
-1.35 degrees; the rotation is calculated from each valid GPS fix, not stored
as a track-specific constant. An IMU producer that already supplies grid yaw
must set this option to `false` to avoid rotating twice. Message schemas and
the raw `/imu` topic are unchanged.

`position/gps_correction_gain: 0.35` softens GPS position corrections at their
source timestamps. This setting depends on consistent position/heading frames;
reducing the gain alone can increase drift. A GPS `(latitude, longitude)=(0,0)`
jamming sentinel is rejected even if its status is positive. It invalidates
the GPS-based planning state without adding the sentinel to its histories;
the existing jamming-mode selector still owns camera-control handover.


고속의 GPS 차분 속도 잡음은 `speed/gps_gain_reference_speed_mps: 5.0`과
`speed/gps_gain_minimum: 0.08`로 완화합니다. GPS 속도와 예측 속도 중 작은 값이
5m/s를 넘으면 기존 속도 보정 이득 0.35에 `(5 / speed)^2`를 곱하고 하한 0.08을
적용합니다. 감속·저속에서는 기존 강한 보정을 사용하므로, 전체 속도 영역의 이득을
0.08로 낮췄을 때 생기는 정지 후 속도 오차 지속을 피합니다. IMU 가속도 예측·입력
유효성·정지 판정 기준은 유지됩니다. 기준 속도 0으로 기존 고정 이득과 비교할 수 있습니다.

## 검증과 과거 실행

`test/common_lidar_ros_test.py`는 독립 master에서 실제 LiDAR·공용 입력·GPS/IMU/재시작 전환을 검사한다.
`test/ros_boundary_smoke_test.py`는 별도 master에서 통합 autonomy 구독·변환을 검사하며 차량 전송을 시작하지 않는다.
구 wire/필터 실행과 근거는 [PR #111까지의 기록](../../docs/records/lidar/planning-input-through-pr111.md)에 보존한다.

PR #114의 `LidarObservationGate`를 단일 `planning_objects_adapter`에서 재사용한다.
속도 미확정은 유효한 OBB를 `velocity_unavailable`·UNKNOWN으로 보존하고 session/generation별
private ID를 분리한다. 진단은1Hz `DiagnosticArray`이며 판제 행동 판단 입력이 아니다.
