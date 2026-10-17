# LiDAR 객체 출력 계약 v2 — scan당 local/map 배타 출력

> **문서 역할:** Additive Candidate Contract — `asmc_msgs/LidarObjectV2`·`LidarObjectArrayV2`,
> `/lidar/local_object`·`/lidar/map_object`, scan별 진리표, 입력 유효성과 수평화 계약
>
> **상태:** Main에 opt-in으로 구현했고 `roslaunch Main main_integration.launch`에서만 켜진다.
> 소비자 수락 전이다. production `main.launch` 기본값, legacy 출력 형식과 판제 입력은 바꾸지 않았다.
> 단계별 적용과 남은 차단 항목은 [LiDAR v2 출력 rollout](../handoffs/lidar-v2-output-rollout.md)에 있다.

[LiDAR→판제 공통 소비 계약 v1](./lidar-planning-consumer-v1.md)의 좌표·시간·속도·유효성 기준은 v2에도
그대로 적용된다. 이 문서는 v2가 새로 정하는 내용만 적는다. 사람이 읽는 요약과 기계 판독 정본이 다르면
기계 판독 정본이 우선한다.

| 정본 | 역할 |
|---|---|
| [`lidar_candidate_topic_contract_v2.yaml`](../../config/contracts/lidar_candidate_topic_contract_v2.yaml) | v2 message MD5, v2 topic, 규칙. `base_contract`는 v1 계약 파일이다 |
| [`LidarObjectV2.msg`](../../src/asmc_msgs/msg/LidarObjectV2.msg), [`LidarObjectArrayV2.msg`](../../src/asmc_msgs/msg/LidarObjectArrayV2.msg) | wire 정의 |
| [`OutputGate.hpp`](../../src/lidar+sensorfusion/src/Main/src/Geometry/OutputGate.hpp) | scan별 출력 선택. ROS 없는 순수 함수다 |
| [`FrameBoundaryPublisher.cpp`](../../src/lidar+sensorfusion/src/Main/src/Geometry/FrameBoundaryPublisher.cpp) | `BuildV2`, `FillMotionV2`, `PublishV2`, 진단 |
| [`main_integration.launch`](../../src/lidar+sensorfusion/src/Main/main_integration.launch) | v2를 켜는 통합 launch |

## 1. 메시지

| 타입 | ROS MD5 | 구성 |
|---|---|---|
| `asmc_msgs/LidarObjectV2` | `c22c9b52da848aeab0e2b4e73343d9f0` | `LidarObject`의 상수 2개와 필드 22개를 같은 타입·이름·순서로 반복한다. 그 뒤에 `MOTION_*` 상수 4개, `bool motion_valid`, `uint8 motion_state`만 덧붙인다 |
| `asmc_msgs/LidarObjectArrayV2` | `d728e7fd984c24641abe10dddfda6dd5` | `LidarObjectArray`의 상수 4개, `header`, `status`, `reason`, `stamp_provenance`를 유지하고 원소 타입만 `LidarObjectV2[]`로 바꾼다 |

v1 타입은 동결 상태이며 정의, MD5, 기존 bool의 의미가 모두 그대로다.

- `LidarObject` `b94cf82e0f325e9ce0364bb143df9f06`, `LidarObjectArray` `5f774d07dffd4c7dfa93a8d4b62c60b4`
- legacy `Msg/TrackedObject`와 `asmc_msgs/TrackedObject` `6c96135eb03a3ee93bba474fa835f57d`
- `TrackedObjectArray` `e127b2aabd7494ad01a5a56b005bf07e`

append-only 상위집합 규칙은 `tools/audit/validate_p1b_contract.py`의 `validate_lidar_object_v2`가 검사한다.

### motion 필드

| `motion_state` | 값 | `motion_valid` | `dynamic_valid` | `is_dynamic` | 뜻 |
|---|---|---|---|---|---|
| `MOTION_UNKNOWN` | 0 | false | false | false | warm-up, GPS 음영·복구로 지운 map 이력, 표현할 수 없는 생산자 상태. **정지라는 뜻이 아니다** |
| `MOTION_LOW_THREAT` | 1 | true | true | false | 확정 저위협 |
| `MOTION_DYNAMIC_PENDING` | 2 | true | true | false | 동적 확인 중 |
| `MOTION_DYNAMIC` | 3 | true | true | true | 확정 동적 |

- 소비자는 `motion_state`로 분기한다. `motion_valid == (motion_state != MOTION_UNKNOWN)`이다.
- `dynamic_valid`와 `is_dynamic`은 v1 필드를 채우기 위한 호환 파생값이다(compat-derived).
  `dynamic_valid = motion_valid`이고 `is_dynamic = (motion_state == MOTION_DYNAMIC)`이다.
  `is_dynamic=false`는 정지의 근거가 아니다. legacy `Msg/TrackedObject.is_dynamic`은 PENDING도 true로
  내보내지만 v2 `is_dynamic`은 그렇지 않다.
- 생산자(`FillMotionV2`)는 Main `MotionState`의 유효한 세 상태를 1:1로 옮긴다. `motion_valid=false`이거나
  값이 범위 밖이면 `(false, MOTION_UNKNOWN)`이다. `b_IsDynamic`이나 class에서 값을 만들지 않는다.
- 나머지 22개 필드는 v1 `LidarObject`를 만드는 같은 코드에서 복사한다. 현재
  `relative_velocity_valid`, `absolute_velocity_valid`, `yaw_rate_valid`는 항상 false다. 속도 값을 유효한
  0속도로 소비하지 않는다.

## 2. topic·frame·stamp

| Topic | Type | frame | 이 topic이 선택되는 scan |
|---|---|---|---|
| `/lidar/map_object` | `asmc_msgs/LidarObjectArrayV2` | `map` | 그 scan의 map pose가 유효하다(`gps_observed`, `gps_bounded_prediction`). query stamp와 clock이 scan과 같다 |
| `/lidar/local_object` | `asmc_msgs/LidarObjectArrayV2` | `base_link`(후륜축) | local 입력과 객체 기하가 유효한 모든 scan(GPS 상태와 무관) |
| `/perception/lidar/diagnostics` | `diagnostic_msgs/DiagnosticArray` | — | 1초마다 v2 출력·사유·카운터를 보고한다 |

- local 입력이 유효한 scan에는 `/lidar/local_object`가 항상 나온다. 같은 scan의 map pose도 유효하면
  같은 stamp로 `/lidar/map_object`가 추가로 나온다. 선택은 구독자 수와 무관하다.
- `header.stamp`는 scan의 bridge 수신 source stamp다(`stamp_provenance`). v2는 v1과 별도의 stamp guard를 둔다.
  0, 역행, 중복 stamp에는 아무것도 발행하지 않고 `v2_stamp_errors`를 1 올린다.
- `status`는 `OBSERVED`(1)와 `OBSERVED_EMPTY`(2)만 쓴다. `reason`은 local 객체가 있으면 `observed`,
  map 객체가 있으면 map pose 사유(`gps_observed`/`gps_bounded_prediction`), 빈 배열이면 `observed_empty`다.
  `INPUT_INVALID`(3)와 `POSE_UNAVAILABLE`(4) 상수는 v1 배열과 같게 남아 있지만 현재 생산자는 v2에 내지 않는다.
- `OBSERVED_EMPTY`는 입력이 모두 유효하고 활성 topic의 그 scan에 발행 대상 객체가 0개일 때만 나간다.
  빈 배열을 모드 신호로 반복하지 않고 좌표 0의 더미 객체를 만들지 않는다. 빈 배열은 자유공간 보장도 아니다.
- 발행 대상 객체 조건은 v1과 같다. `track_id >= 0`, `miss_count == 0`, 점 1개 이상이고 class가
  `noise`/`지면노이즈`가 아니어야 한다.
- 입력이 무효인 scan(3절의 N 행)은 두 v2 topic 모두에 **아무것도 발행하지 않는다**. 사유는
  `/perception/lidar/diagnostics`의 `v2_output=none`, `v2_reason`, `v2_none_frames`에 남는다.
  `fusion_trace_path`를 켰으면 trace `lidar_process_end`의 `v2_mode`/`v2_reason`에도 남는다.
  소비자는 이 침묵을 진단과 자신의 timeout으로 판단한다. 빈 배열로 해석하지 않는다.

진단의 v2 키는 `v2_output`(`none`/`local`/`local_and_map`), `v2_reason`, `gps_state`, `v2_local_frames`,
`v2_map_frames`, `v2_none_frames`, `v2_stamp_errors`, `v2_local_subscribers`, `v2_map_subscribers`,
`leveling_store_count`다. 기존 키는 그대로이며 `legacy_output`에는 새 값 `frame_invalid`가 생겼다.
종료할 때 stdout에 `[LIDAR_V2_TOTAL] local=… map=… none=… stamp_errors=…`를 출력한다.
기존 `[LIDAR_FRAME_TOTAL]` 줄은 바꾸지 않았다.

## 3. scan별 진리표

`enable_common_geometry=true`(`main_integration.launch`)일 때 위에서부터 처음 맞는 행 하나가 적용된다.
N1–N7은 `output_gate::Decide`, N8–N9는 `PublishV2`와 `BuildV2`, M1·M2의 legacy 구분은
`output_gate::PlanLegacy`가 정한다.

"legacy"는 `/last_lidar`, `/lidar/tracked_objects_map`, `/lidar/motion_debug`와 planner UDP를 함께 뜻한다.
네 출력은 `main.cpp`의 불리언 하나(`PlanLegacy(...).publish`)로 묶여 있다. legacy를 보류한 scan에는
`/lidar/tracking_markers_map`과 `/last_lidar_markers`에 DELETEALL만 나간다(stamp가 0이면 생략).
`legacy_output`과 `gps_state`는 진단 값이다. "평가값"은 그 scan의 map pose를 분류한 결과다.

| 행 | 조건 | v2 | `v2_reason` | legacy | `legacy_output` | `gps_state` |
|---|---|---|---|---|---|---|
| N0 | Parser가 LiDAR scan을 거부했다(0·중복·역행 source stamp, buffer 초과) | 없음 | 처리하지 않음 | 없음, DELETEALL도 없음 | 변화 없음 | 변화 없음 |
| N1 | local 입력이 무효다. IMU·CAN이 없거나 120ms보다 오래됐거나 무효 표본 barrier에 걸렸다. scan 시각이 무효인 경우도 같다 | 없음 | `local_input_invalid:<사유>` | 보류 | `input_invalid` | `not_evaluated` |
| N2 | LiDAR 처리 실패(빈 cloud 등) | 없음 | `lidar_input_invalid` | 보류 | map pose가 유효하면 `frame_invalid`, 아니면 `pose_unavailable` | 평가값 |
| N3 | scan context 세대 불일치 | 없음 | `context_stale` | 보류 | N2와 같음 | 평가값 |
| N4 | scan 시각 검사 실패 | 없음 | `time_invalid:<오류>` | 보류 | `pose_unavailable` | 평가값 |
| N5 | 수평화 무효(`StoreLeveling` 0회·2회 이상 또는 무효 회전) | 없음 | `leveling_transform_invalid` | 보류 | N2와 같음 | 평가값 |
| N6 | GPS가 아닌 이유로 map pose가 무효다. `map_speed_invalid`(CAN 속도가 비유한이거나 35m/s 초과), `local_motion_invalid`, 시각 오류, 알 수 없는 사유가 여기에 속한다 | 없음 | `map_pose_invalid:<사유>` | 보류 | `pose_unavailable` | `non_gps_invalid` |
| N7 | map pose는 유효하지만 query stamp·clock이 scan과 다르다 | 없음 | `map_pose_time_mismatch` | 보류 | `frame_invalid` | `healthy` |
| N8 | v2 stamp guard 거부 또는 context stamp 불일치 | 없음 | `v2_stamp_rejected`, `context_stamp_mismatch` | 보류 | N2와 같음 | 평가값 |
| N9 | 객체 하나라도 기하가 무효다(치수 NaN·음수, 비유한 corner, map 변환 실패). 프레임 전체를 거부한다 | 없음 | `invalid_object_dimensions`, 변환 사유, `map_transform_invalid:<사유>` | 보류 | N2와 같음 | 평가값 |
| L1 | GPS 음영: `gps_missing`, `gps_invalid`, `gps_stale` | `/lidar/local_object` | `local_only:<GPS 사유>` | 보류 | `pose_unavailable` | `denied` |
| L2 | GPS 복구 중: `gps_recovering` | `/lidar/local_object` | `local_only:gps_recovering` | 보류 | `pose_unavailable` | `recovering` |
| M1 | map pose가 정상이고 발행 대상 객체 가운데 `motion_valid=false`(warm-up)가 하나라도 있다 | `/lidar/local_object` + `/lidar/map_object`. 해당 객체는 `MOTION_UNKNOWN`이다 | `local_and_map:<map 사유>` | 프레임 전체 보류 | `motion_state_unrepresentable` | `healthy` |
| M2 | map pose가 정상이고 발행 대상 객체가 모두 `motion_valid`다(빈 장면 포함) | `/lidar/local_object` + `/lidar/map_object` | `local_and_map:gps_observed` 또는 `local_and_map:gps_bounded_prediction` | 발행 | `available` | `healthy` |

모든 행에서 v1 `/perception/lidar/objects_local`·`objects_map`은 변경 전 동작을 그대로 따른다.
구독자가 있으면 같은 scan에 local과 map이 함께 나갈 수 있으며 `INPUT_INVALID`·`POSE_UNAVAILABLE` 배열도 그대로다.
v1은 호환 전용이고 최종 출력이 아니다. `/lidar/object`, `/lidar/object_boxes`, `/lidar_tracking_markers`,
`/lidar/debug_*`는 시각화 topic이다.

`enable_common_geometry=false`(production `main.launch` 기본)에서는 v2를 advertise하지 않는다. legacy는
LiDAR 처리에 성공한 scan마다 발행한다. DELETEALL, `legacy_output` 보고, `LegacyMotionReady` 적용은 없으며
변경 전과 같다. 이 모드에서 warm-up 객체가 legacy로 나가는 문제는 알려진 공백이다(rollout 문서 KNOWN_GAP).

### 불변식

| 불변식 | 구조적 근거 |
|---|---|
| 유효한 local scan마다 `/lidar/local_object`가 1개 나오고, `/lidar/map_object`는 유효한 같은-stamp local 출력과 함께만 나온다 | `PublishV2`의 `kLocal`/`kBoth` 분기 |
| `/lidar/map_object`는 M1·M2에서만, `/lidar/local_object`는 L1·L2와 M1·M2에서 나온다 | `Decide`와 `PublishV2` |
| legacy 발행은 같은 scan의 `/lidar/map_object` 발행과 발행 대상 전체의 `motion_valid`를 요구한다. UDP도 같은 불리언을 쓴다 | `PlanLegacy`, `main.cpp`의 `legacy.publish` |
| local/map 선택은 구독자 수와 무관하다 | `BuildV2`는 구독 수를 보지 않는다 |
| GPS 3표본 복구 상태는 `BoundedPoseHistory`에만 있고 gate는 상태가 없다. 복구된 첫 scan은 `/lidar/map_object`다 | `OutputGate.cpp` |

## 4. GPS·IMU·CAN 유효성과 음영·복구

local 입력(IMU와 CAN)은 GPS와 무관하다.

- scan 이전에 도착한 IMU와 차량 속도(CAN) 표본만 쓴다. 최대 age는 각각 120ms다
  (`lidar_pose_imu_max_age_s`, `lidar_pose_can_max_age_s`). 최대 50ms까지 기다리며(`lidar_pose_wait_timeout_s`)
  GPS는 기다리지 않는다.
- 무효 IMU·CAN 표본은 barrier가 된다. stamp 0, 준비되지 않았거나 비유한 IMU(잘못된 quaternion 포함),
  비유한·음수 속도, 비현실적 yaw·속도 jump가 여기에 속한다. barrier 뒤에 정상 표본이 오기 전까지는 이전 정상
  표본으로 대신하지 않는다(`Parser/SensorPoseHistory.cpp` `LookupLocalLocked`). 이 검사에 걸리면 N1이다.
  stamp가 역행한 표본은 barrier 없이 버린다.
- barrier 판정은 source stamp 기준이다. 공통 경로는 모든 무효 표본의 source stamp를 기억하고(sample 이력과
  같은 5초 범위, 최대 1024개), 선택된 정상 표본부터 scan stamp까지 구간 안에 무효 stamp가 하나라도 있으면
  N1이다. scan보다 늦은 stamp의 무효 표본은 언제 도착하든(50ms 대기 중 포함) 그 scan의 판정을 바꾸지 않는다.
  GPS 표본의 IMU 자세 gate도 같은 구간 규칙을 쓴다. 예전 결함 F1(최대 stamp 하나만 기억해서 다음 무효
  표본이 barrier를 scan 뒤로 밀고 무효 시작 직후 첫 scan이 이전 정상 IMU로 local에 나가던 문제)은 이것으로
  고쳤다(`InvalidBarrierOrderB0.*`). legacy 경로(`enable_common_geometry=false`)는 barrier를 기록하지도 보지도 않는다.
- stamp 0 IMU·CAN 표본은 source 시각이 없다. 수신 callback의 ROS 시각과 함께, 도착 시점에 이미 받아 둔 가장 최근
  정상 표본의 stamp에도 barrier를 둔다. 그래서 수신이 scan stamp보다 몇 ms 늦어도 그 정상 표본을 다시 쓰지 않는다.

map pose(GPS)는 `asmc_lidar::BoundedPoseHistory`가 정한다.

- source stamp가 scan 이전이고 실제 수신도 scan 수신 이전인 GPS 표본만 쓴다. 늦게 도착한 GPS로 지난 scan의
  map 가용성을 바꾸지 않는다.
- 표본이 없으면 `gps_missing`이다. 무효 status, 0 좌표, 비유한 값, 잘못된 map offset은 `gps_invalid`다.
  마지막 표본의 age가 300ms(`lidar_pose_gps_max_age_s`)를 넘으면 `gps_stale`이다.
  연속 정상 3표본 전에는 `gps_recovering`이다. 무효 표본이나 300ms를 넘는 정상 표본 간격은 연속 횟수를 0으로 되돌린다.
- 정상 표본이 3개 연속으로 모이면 map pose가 유효해지고 그 scan부터 local과 `/lidar/map_object`가 함께 나간다.
  음영 중에는 map 위치를 추정하지 않으며 단절 전후 GPS 차분으로 속도를 만들지 않는다.
- map attitude는 같은 scan의 IMU 표본(roll·pitch·yaw 각 1회)이고, 짧은 위치 보정에는 CAN 속도를 쓴다.
  CAN 속도가 비유한이거나 35m/s(`lidar_pose_maximum_speed_mps`)를 넘으면 `map_speed_invalid`이고 N6이다.
  GPS 음영이 아니므로 local로 내보내지 않는다.
- GPS 음영은 local tracker·ID·검출 설정을 바꾸지 않는다. map 기반 motion 이력만 지운다. 그래서 L1·L2의
  local 객체는 모두 `MOTION_UNKNOWN`이다. 복구 후에는 track마다 warm-up(`motion_warmup_observations=3`)을
  다시 거친다. 그동안 `/lidar/map_object`는 나가지만 해당 객체가 `MOTION_UNKNOWN`이므로 legacy는 M1로 보류된다.
- callback 수신 시각보다 `gps_sanity_maximum_future_lead_s`(기본 100ms) 넘게 미래인 GPS source stamp는
  즉시 `gps_invalid` barrier로 거부하며 sanity anchor를 미래로 옮기지 않는다. 이 구간에는 local만 나오고,
  정상 시각 GPS가 돌아오면 기존 3표본 복구 latch를 거쳐 local+map 출력으로 복귀한다.

## 5. 수평화와 yaw 계약

`main_integration.launch`는 `enable_common_geometry=true`와 `lidar_level_roll=true`를 고정한다. arg로 바꿀 수 없다.

- IMU roll과 pitch는 forward 수평화 `CorrectingPoints`(`Lidar/ImuRansac/ImuFusion.cpp`)에서 각각 정확히 한 번
  적용된다. 적용한 회전은 `frame_boundary::StoreLeveling`으로 scan마다 정확히 한 번 기록된다. 기록이 0회이거나
  2회 이상이거나 회전이 무효이면 `leveling_source`가 비고 `base_from_input`이 실패한다. 그 scan은 N5다.
- local(`base_link`) 객체는 기록된 수평화의 역변환과 센서 장착 변환으로 만든다. IMU roll/pitch의 순 적용 횟수는 0이다.
  map 객체는 같은 scan의 local 결과에 `map_from_base`(같은 scan IMU의 roll·pitch·yaw)를 곱한다. 순 적용 횟수는 1이다.
  다시 수평화하지 않는다.
- 수평화는 GPS와 무관하다. GPS 정상·무효·stale·없음·복구 중·복구 후에 수평화 행렬, 수평화한 점군, local 결과가
  bit 단위로 같다.
- yaw: 이전 scan의 OBB yaw는 현재 프레임으로 한 번만 다시 표현한다(정렬 사본, 미관측 track 예측). 현재 검출의
  OBB yaw는 Kalman update에서 다시 돌리지 않는다(`AlignTrackingFrame(..., rotate_obb_yaw=false)`).
  따라서 발행 yaw 오차가 scan마다 누적되지 않는다.
- 진단 `leveling_store_count`는 1, `leveling_source`는 `Main::CorrectingPoints`여야 한다.

legacy 경로의 한계: `enable_common_geometry=false`(production 기본)는 이 계약 밖이다.

- 이 경로는 pitch만 수평화한다.
- legacy 내부 helper `Body2ENU`·`MinMax2ENU`(`Global/Global.cpp`), `FillMapObbBounds`(`MotionClassifier`)와
  legacy 카메라 투영(`SensorFusion/FusionCamera.cpp`)은 pitch를 한 번 더 적용한다. 20m·3°에서 약 0.16m다.
- 이 결함은 특성 시험 `LevelingContract.LegacyBody2EnuAppliesPitchTwice_KnownDefect`로만 고정했고 고치지 않았다.
- 통합 launch는 공통 경로만 쓴다. legacy 기하 경로는 새 소비자에게 권하지 않으며(deprecated) B1 이후 정리 대상이다.

## 6. 시험

ROS master가 필요한 시험(rostest)은 격리된 network namespace의 전용 master에서만 실행한다.

| 범위 | 시험 |
|---|---|
| message·계약 | `tools/audit/test_p1b_contract.py`, `src/asmc_msgs/test/test_message_serialization.py`, `src/asmc_msgs/test/test_lidar_object_v2.py`, Main gtest `LidarObjectV2Contract.*` |
| N1 | `OutputGateB0.LocalPoseInvalidPublishesNothing`, `GpsDenialRecovery.ImuInvalidStaleOrSilentIsNoneNeverLocal`, `CanStaleSilentOrInvalidIsNoneNeverLocal`, `ParserCallbacksTurnEveryMalformedInputIntoABarrier`, `InvalidBarrierOrderB0.*` |
| N2–N5, N7 | `OutputGateB0.LidarTimeLevelingContextOrMapTransformInvalidPublishesNothing`, `LevelingContract.StoreLevelingExactlyOncePerScan`, `LidarObjectV2Builder.GateInputsFromRealContextsFollowTheTruthTable` |
| N6 | `OutputGateB0.EveryGpsFaultClassMapsToItsOutput`, `GpsDenialRecovery.CanSpeedAboveLimitIsNoneNotLocal`, `ClockMismatchNeverProducesMapOrLocalFromInconsistentTime` |
| N8 | `FrameBoundaryV2.V2StampGuardIsIndependentOfV1`, `OutputGateB0.DowngradeAfterBuilderOrStampRejectionPublishesNothing` |
| N9 | `LidarObjectV2Builder.InvalidObjectGeometryRejectsWholeFrame`, `FrameBoundaryV2.InvalidGeometryPublishesNothingOnV2` |
| L1·L2 | `OutputGateB0.GpsDeniedOrRecoveringSelectsLocalOnly`, `BoundedPoseRecoveryDrivesLocalThenMap`, `GpsDenialRecovery.*`(시작 3표본, 무효 class별 첫 scan, 300ms 침묵·stale, 정확히 3표본 복구, 복구 중 무효·간격, 늦은 도착) |
| M1 | `OutputGateB0.HealthySelectsMapOnlyAndKeepsFrameWideLegacyMotionGate`, `LegacyCompat.LegacyMotionReadyIsFrameWide`, `CommonModeWithholdsTheWholeLegacyFrameWhileAnyObjectWarmsUp`, `FrameBoundaryV2.WarmupObjectIsCarriedOnMapV2AsUnknown` |
| M2 | `LidarObjectV2Builder.MapModeEqualsMapPoseTimesLocalFromTheSameScan`, `FrameBoundaryV2.ExactlyOneV2MessagePerScanAndV1Unchanged`, `SelectionIgnoresSubscriberDemand`, `EmptySceneIsObservedEmptyOnActiveTopicOnly` |
| 전체 조합 | `OutputGateB0.ExhaustiveInvariants`, `DeleteAllOnEveryScanWhileMapOutputIsDeactivated`, `LegacyCompat.LegacyIsNeverPublishedWithoutMapV2EvenWhenMotionReady`, 실제 Main rostest `main_integration_gating.test` |
| 수평화·yaw | `LevelingContract.*`(`RollPitchExactlyOnceInLocalAndMap`, `GpsStateDoesNotChangeLevelingOrLocalOutput`, `NoObbYawDoubleRotationAcrossTwoScans`, `PublishedYawErrorDoesNotAccumulate` 등) |
| production 기본값 | `OutputGateB0.ProductionModeHasNoV2AndKeepsPreB0RuleForMotionReadyFrames`, `FrameBoundaryV2.CommonGeometryOffAdvertisesNoV2AndPassesDecisionThrough`, `test_main_launch_contract.py`, `PlannerUdpEnable.*` |

nested workspace의 Main 시험은 CI에서 빌드하지 않는다. CI가 기계적으로 지키는 것은 계약 validator와
`tools/audit/test_p1b_contract.py`, topic policy, 문서 검사, `tools/audit/check_legacy_lidar_publishers.py`다.
Main 시험은 변경 PR에서 로컬 gate로 실행한다.
