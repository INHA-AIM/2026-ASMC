# LiDAR v2 출력 rollout과 남은 차단 항목

> **문서 역할:** Active Handoff — LiDAR v2 최종 출력의 단계별 적용, 실행 방법, legacy·v1 호환 상태와 남은 차단 항목
>
> **상태(2026-10-15):** branch `feature/lidar-nogps-main-integration`(기준 `fcfe8c58`)의 opt-in 구현이다.
> production 기본값과 판제 입력은 바뀌지 않았고 소비자 수락 전이다. 계약 정본은
> [LiDAR object v2](../contracts/lidar-object-v2.md)다.

이 문서에서 "B0"는 LiDAR v2 출력 통합 묶음(commit 5개)을 가리킨다.
[Camera·LiDAR B0 기준선 프로토콜](../records/camera/camera-lidar-b0.md)의 B0와는 다른 작업이다.

## 1. B0 묶음

| commit | 내용 |
|---|---|
| `feat(asmc_msgs): add lossless lidar object v2 motion contract` | `LidarObjectV2`·`LidarObjectArrayV2`, 새 계약 파일 `lidar_candidate_topic_contract_v2.yaml`과 registry 1행, validator와 시험 |
| `fix(lidar): unify leveling and remove duplicate yaw transform` | scan당 `StoreLeveling` 1회 계약, Kalman update의 OBB yaw 이중 회전 제거. 둘 다 공통 경로에만 적용된다 |
| `feat(lidar): enforce local/map output gating` | `OutputGate`, v2 publisher, legacy gate, `planner_udp_enabled`, `main_integration.launch`, topic policy |
| `test(lidar): cover gps denial recovery and legacy compatibility` | GPS·IMU·CAN 음영·복구 gtest, legacy 호환 gtest, 실제 Main을 띄우는 rostest |
| `docs(lidar): document b0 rollout and remaining blockers` | 계약·인계 문서, `drivable_filter_node.py` 정적 검사 |

동결 message 4개(`asmc_msgs/LidarObject*`, `Msg/TrackedObject*`)와 기존 bool의 의미, production UDP 기본값은
바꾸지 않았다. `/last_lidar`, `/lidar/motion_debug`, `planning_inputs` bridge, `/lidar/tracked_objects_map`은 제거하지 않았고
형식도 그대로다. 통합 프로필에서만 4절 규칙으로 발행 scan이 줄어든다. TunnelFilter, P4, 구조물 필터, 카메라 보호,
LP seeding, ML 코드는 건드리지 않았다.

## 2. 단계

| 단계 | launch | 최종 LiDAR 출력 | legacy와 UDP | 상태 |
|---|---|---|---|---|
| 0 production | `roslaunch Main main.launch` | v2 없음 | 처리한 scan마다 발행, UDP는 `setting.yaml`대로 127.0.0.1:7510 | 현행 운영 경로. 변경 없음. 5절 KNOWN_GAP |
| 1 통합 프로필 | `roslaunch Main main_integration.launch` | 유효한 local scan마다 `/lidar/local_object`; map pose가 유효하면 같은 stamp의 `/lidar/map_object` 추가 | `/lidar/map_object` scan 가운데 객체가 모두 `motion_valid`인 scan에서만 | B0 구현, opt-in |
| 2 소비자 수락(B1 진입) | 1과 같음 | 1과 같음 | 1과 같음 | 미착수. 9절 조건 |
| 3 판제 v2 이전 | 별도 결정 | v2 | 병행 유지 | 미착수, B0 범위 밖. 8절 |
| 4 정리 | 별도 결정 | v2 | `/lidar/tracked_objects_map`, v1 typed, legacy 기하 경로 제거 검토 | 모든 소비자 이전 후 |

## 3. 통합 프로필 실행

빌드 순서는 [RUN_SENSOR_FUSION](../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md)과 같다. 바깥 workspace를 먼저
빌드해 `asmc_msgs` v2 header를 만든 뒤 nested workspace를 `catkin_make --force-cmake`로 다시 빌드한다.
함수 기본 인자가 추가돼 symbol이 바뀌었으므로 예전 object가 남은 증분 빌드는 링크에 실패할 수 있다.

```bash
roslaunch Main main_integration.launch planner_udp_enabled:=false
```

- 오프라인 재생, bag, 시험에서는 항상 `planner_udp_enabled:=false`를 넘긴다. 이 값이면 planner UDP sender socket을
  만들지 않고 아무것도 보내지 않는다. roscpp 자체의 UDPROS server socket은 남을 수 있지만 아무것도 보내지 않는다.
  기본값 true는 production과 같아 127.0.0.1:7510으로 보낸다.
- `enable_common_geometry=true`와 `lidar_level_roll=true`는 launch 안에 고정돼 있다. 명령행에서 넘겨도
  무시된다(`roslaunch --dump-params`로 확인). 다른 launch의 `<include>`에서 넘기면 roslaunch가 unused arg 오류를 낸다.
- 나머지 arg(`enable_sensor_fusion`, `rviz`, `fusion_trace_path`, `drivable_filter_enabled`,
  `known_static_filter_enabled`, `static_structure_exclusion_enabled`)의 기본값은 `main.launch`와 같다.
- 확인할 것: `rostopic echo /perception/lidar/diagnostics`에서 `v2_output`, `v2_reason`, `gps_state`,
  `leveling_store_count`(1), `leveling_source`(`Main::CorrectingPoints`)를 본다. `/lidar/local_object`와
  `/lidar/map_object`를 함께 구독해 map의 모든 stamp에 같은 stamp의 local 출력이 있는지 확인한다.
- rollback: `main_integration.launch`를 끝내고 `main.launch`를 다시 실행한다. 객체 생산자는 한 번에 하나만 띄운다.

ROS master가 필요한 시험은 공유 master(11311)가 아니라 격리된 network namespace의 전용 master에서만 실행한다.
실제 Main을 띄우는 rostest는 `src/lidar+sensorfusion/src/Main/test/main_integration_gating.test`다.

## 4. scan별 출력 요약

정확한 행, 사유 문자열과 시험은 [계약 3절](../contracts/lidar-object-v2.md#3-scan별-진리표)이 정본이다.

- **NONE(아무것도 없음):** LiDAR·scan 시각·IMU·CAN·좌표 변환·객체 기하 중 하나라도 무효이거나 GPS가 아닌 이유로
  map pose가 무효인 scan이다. v2 두 topic에 아무것도 나가지 않는다. 빈 배열이나 `INPUT_INVALID` 배열도 내지 않는다.
  사유는 진단(`v2_output`, `v2_reason`, `v2_none_frames`)과 trace `lidar_process_end`에만 남는다.
- **LOCAL:** local 입력이 유효한 모든 scan에 `/lidar/local_object`(`base_link`)가 나온다. GPS가 없거나 무효이거나
  stale·미래 시각이거나 복구 중이면 local만 나오고
  legacy는 보류된다. 지도 marker에는 DELETEALL만 나간다.
- **MAP:** 그 scan의 map pose가 유효하면 같은 stamp의 `/lidar/map_object`(`map`)가 local에 추가로 나온다.
- `OBSERVED_EMPTY` 빈 배열은 입력이 유효하고 활성 topic의 장면에 객체가 실제로 0개일 때만 나간다.
  모드 신호로 빈 배열을 반복하지 않으며 좌표 0의 더미 객체를 만들지 않는다.
- v1 `/perception/lidar/objects_local`·`objects_map`은 지금도 scan마다 두 topic 모두 나간다(구독자가 있을 때).
  호환 전용이며 최종 출력이 아니다.
- legacy(`/last_lidar`, `/lidar/tracked_objects_map`, `/lidar/motion_debug`, UDP)는 `/lidar/map_object` scan 가운데
  발행 대상 객체가 모두 `motion_valid`인 scan에서만 나간다. warm-up 객체가 하나라도 있으면 프레임 전체를 보류한다.
- GPS 복구 직후에는 track이 다시 warm-up을 거친다. 그래서 `/lidar/map_object`가 돌아와도 정지 track 기준으로 약 9 scan 동안
  legacy가 나가지 않는다(B-10). 단위 시나리오 `LegacyCompat.CommonModeWithholdsTheWholeLegacyFrameWhileAnyObjectWarmsUp`의
  `legacy_pattern`(음영 5 scan 뒤 `dddddwwwwwwwwwPPP…`)과 runtime S22·S17–S20에서 확인했다. GPS 3표본 복구 시간은 여기에 더해진다.

## 5. production 기본값과 KNOWN_GAP

production `main.launch`의 기존 arg와 param은 그대로다. `enable_common_geometry=false`(4행)와 `lidar_level_roll=false`(5행)를
유지한다. 추가한 것은 `planner_udp_enabled` arg(8행, 기본 true)와 param 1줄뿐이다. `setting.yaml`의
`PlannerInterface`(Enabled true, 127.0.0.1:7510, MaxObjects 20)도 바꾸지 않았다.
변경 전 `main.launch`와 B0 `main.launch`의 `roslaunch --dump-params`를 비교하면 `/Main/planner_udp_enabled: true`
한 줄만 추가됐다. `test_main_launch_contract.py`가 이 기본값을 고정한다.

**KNOWN_GAP (B1 차단):** production(common OFF)에서는 warm-up 객체가 지금도 `/last_lidar`와 UDP(기본 ON)에
속도 0인 객체로 나간다. legacy wire(`Msg/TrackedObject`)에는 motion 상태 필드가 없으므로 소비자는 이 객체를 정지 객체와
구분할 수 없다. B0는 production 동작을 바꾸지 않는다는 결정에 따라 이 경로를 고치지 않았다. 이 동작을 옳다고
단언하는 시험도 두지 않았다. 격리 runtime의 production 대조(S22)에서 warm-up 상태 객체 17개가 모두 속도 0으로
`/last_lidar`에 나갔다. 같은 시나리오를 포함한 모든 통합 프로필 실행에서는 warm-up 객체가 legacy에 나간 scan이 0이었다.

## 6. legacy·v1 topic 상태

### `/lidar/tracked_objects_map`

- 생산자: Main `PlannerInterface/PlannerObjectRosPublisher.cpp` 677–678행. B0에서 제거하지 않았다. 통합 프로필에서는 4절
  legacy 규칙에 따라 보류되는 scan이 있다.
- runtime 판제 구독자는 기본 구성에 없다. `planning_inputs`의 `objects.launch`(2행)와 `asmc_planner`의
  `autonomy_no_bridges_v2.launch`(8행)는 `/last_lidar`와 `/lidar/motion_debug`를 쓴다.
  `lidar_source_policy:=main_raw lidar_input_topic:=/lidar/tracked_objects_map`을 명시할 때만
  이 topic을 구독한다(`src/planning_inputs/README.md` 46–49행).
- 시험·도구·수동 개발 소비자가 있다.
  - 시험: `src/planning_inputs/test/ros_boundary_smoke_test.py`(41·69행), `src/lidar+sensorfusion/src/Main/test/test_main_integration_gating_ros.py`(100행, 기록용)
  - 도구: `src/lidar+sensorfusion/analyze_velocity_bag.py`(35행), `src/lidar+sensorfusion/src/Main/scripts/build_known_static_object_catalog.py`(55행),
    `tools/perception/replay_camera_objects.py`(129행), `run_frame_boundary_trial.py`(116행), `run_b1a_trial.py`(89행), `run_camera_lidar_b0.py`(189행)
  - 수동 개발: `src/asmc_hd_map/development/won_tae_link_node/drivable_filter_node.py`(76행 입력 기본값)
  - 계약·감사: `config/contracts/perception_runtime_topic_contract_v1.yaml`(86행, 종료 조건 bag parity와 모든 소비자의 `asmc_msgs` 사용),
    `tools/audit/validate_perception_contract.py`(37행), `tools/audit/build_perception_runtime_baseline.py`(121·383행)
- `tools/perception/b0_measurements.py`는 이 topic을 "처리한 stamp마다 한 번"으로 센다. production `main.launch`에서는 맞지만
  통합 프로필에서는 gate 때문에 성립하지 않는다. 통합 프로필로 측정하려면 v2 topic으로 바꿔야 한다.
- 장래 제거 후보다. 제거하려면 위 소비자를 v2 또는 `/last_lidar`로 먼저 옮기고 계약의 종료 조건을 만족해야 한다.

### v1 typed `/perception/lidar/objects_local`, `objects_map`

호환용(deprecated)이며 최종 출력이 아니다. 동작은 변경 전과 같고 기존 rostest `frame_boundary.test`(14건, 파일 무수정)가
같은 scan의 dual publish를 고정한다. topic policy의 producer anchor도 v1 literal을 요구한다. 제거나 배타화는 소비자
수락 이후에 정한다(B-5).

### legacy 기하 경로(common OFF)

pitch만 수평화하며 내부 helper가 pitch를 한 번 더 적용하는 알려진 결함이 있다(B-8,
[계약 5절](../contracts/lidar-object-v2.md#5-수평화와-yaw-계약)). 통합 프로필은 공통 경로만 쓴다.
새 소비자는 legacy 기하에 의존하지 않는다. 정리는 B1 이후다.

### `drivable_filter_node.py` 경고

`src/asmc_hd_map/development/won_tae_link_node/drivable_filter_node.py`는 `/lidar/tracked_objects_map`을 받아
`last_lidar`로 다시 발행하는 수동 대조용 노드다. 어떤 launch도 이 노드를 띄우지 않는다. Main과 함께 실행하면
`/last_lidar` publisher가 둘이 되므로 함께 실행하지 않는다. `tools/audit/check_legacy_lidar_publishers.py`가 CI에서
모든 launch XML(`*.launch`, `*.launch.xml`, `*.test`)과 shell launcher(`*.sh`)를 검사한다. 주석이 아닌 곳에서 이 노드를
참조하면 실패한다(`tools/audit/test_legacy_lidar_publishers.py`).

## 7. UDP와 도구 영향

- `planner_udp_enabled:=false`이면 planner UDP sender socket을 만들지 않고 아무것도 보내지 않는다. roscpp 자체의
  UDPROS server socket은 남을 수 있지만 아무것도 보내지 않는다. ROS legacy 발행과 UDP 전송은 같은 불리언 하나로 묶여 있다.
- 변경 전 Main은 이 param을 무시했다. 그래서 `tools/perception/run_frame_boundary_trial.py`(68행), `run_b1a_trial.py`(56행),
  `b1a_protocol.py`(80행)는 false를 넘겨도 UDP를 보냈다. B0 이후에는 실제로 UDP가 꺼진다.
- `tools/perception/b0_main.launch`처럼 arg를 넘기지 않는 launch는 이전과 같이 UDP를 보낸다.

## 8. 판제 v2 이전은 별도 단계

B0는 판제 입력을 바꾸지 않는다. 판제가 v2를 소비하려면 별도 PR에서 다음을 정한다.

- `planning_inputs`에 `asmc_msgs/LidarObjectArrayV2` 입력 경로를 만든다. `motion_state` 값은
  `planning_inputs`의 `DetectedObject` `MOTION_*`와 같다. 소비자는 `motion_state`로 분기한다.
- 두 v2 topic을 모두 구독한다. NONE scan의 침묵은 timeout과 진단으로 처리한다.
- GPS 음영 중 `/lidar/local_object`(`base_link`)를 쓰는 판제·음영 어댑터의 좌표 정합과 복귀 전환 정책을 정한다.
- v2 속도 필드는 아직 모두 무효다. 판제가 쓰는 속도 계약을 먼저 정해야 한다.
- 전환 기간에는 legacy와 v2를 병행하고 같은 scan의 객체 수·ID·OBB를 비교한다. legacy와 UDP의 제거는 그 다음이다.

## 9. B1 진입 조건

1. 실데이터 검증: 통합 프로필을 실제 MORAI 주행과 정답이 있는 bag으로 검증한다. B0 runtime 근거는 합성 feeder 장면이다.
   bag smoke 1회는 참고용이다.
2. 구조물 필터 정책: 터널·벽·가드레일 처리(TunnelFilter, P4, 구조물 필터, 카메라 보호)는 B0에 없다. GPS/미션에 따라
   경로를 바꾸지 않는 단일 경로 정책([asmc_lidar 인계](lidar-migration.md))과 그 결과의 v2 표현을
   판제와 합의한다.
3. 무효 barrier 수정(F1, 10절)은 B0 무수정 목록에 있던 `Parser/SensorPoseHistory.cpp`를 바꾼다. owner 리뷰를 받고
   IMU 무효 시나리오(S11)의 반복 실행 결과를 확인한다.
4. production warm-up 노출(KNOWN_GAP)의 처리 방법과 production 동작 변경 승인을 받는다.
5. owner 리뷰를 받는다(B-1). v2 파생 필드 확정(B-2), NONE 침묵 정책(B-6), `is_dynamic` 의미 차이(B-27)는 소비자 리뷰를 거친다.
6. 수평화: production roll 기본값(B-7), 실데이터에서 roll·pitch 수평화가 지면·군집에 주는 영향, IMU 부호 규약(B-24),
   legacy pitch 이중 적용(B-8)을 결정한다.
7. 카메라·fusion을 함께 켠 통합 프로필의 CPU·지연 부하를 측정한다.
8. 복구 직후 legacy 공백(B-10)과 미래 stamp GPS 이후의 map 공백(B-11)을 판제가 받아들일지 정한다.

## 10. 남은 차단 항목

| ID | 상태 | 내용과 근거 |
|---|---|---|
| F1 | 수정함(2026-10-15), owner 리뷰 필요 | 변경 전부터 있던 결함이다. IMU 무효가 시작된 직후 첫 scan이 NONE이 아니라 `/lidar/local_object`로 나갈 수 있었다. `SensorPoseHistory`가 IMU·CAN마다 최대 stamp인 무효 barrier 하나만 두어, 50ms 대기(`WaitForLocalLookupNs`) 중에 scan 뒤 stamp의 무효 표본이 오면 barrier가 scan 뒤로 밀렸다. callback 지연이 run마다 달라 격리 runtime S11에서 간헐적으로 재현됐다. 이제 공통 경로는 무효 source stamp를 모두 보관하고(5초, 최대 1024개) [선택 정상 표본, scan stamp] 구간에 하나라도 있으면 NONE이다. GPS 자세 gate도 같은 규칙이다. ROS 없는 결정적 회귀 시험 `InvalidBarrierOrderB0.*`는 현재 11개 case다. 수정 전 `SensorPoseHistory`에서는 F1 6개와 F1b 2개, 모두 8개가 실패한다. 수정 후에는 11개 모두 통과한다. `--gtest_repeat=30`과 shuffle seed 16개로도 확인했다. F1만 다루던 시점의 시험은 9개 중 6개가 실패했었다. legacy 경로(`enable_common_geometry=false`)는 barrier를 기록하지도 보지도 않는다 |
| F1b | 수정함(2026-10-15), owner 리뷰 필요 | 변경 전부터 있던 결함이다. stamp 0 CAN·IMU 표본은 source 시각이 없어 수신 callback의 ROS 시각으로 barrier를 매겼다. 수신이 scan stamp보다 몇 ms 늦으면 barrier가 scan 뒤에 놓여 105ms 된 정상 표본이 다시 쓰였다(격리 runtime S16에서 run마다 위상에 따라 실패). 이제 도착 시점에 이미 받은 최신 정상 표본 stamp에도 barrier를 둔다(`InvalidBarrierOrderB0.ZeroHeaderCanReceivedJustAfterTheScanStampStillBlocksTheOldSample`, `ParserPathZeroHeaderImuBlocksTheOldSampleWhateverTheReceiptClock`) |
| KNOWN_GAP/B1 | 미해결, B1 차단 | 5절. production warm-up 객체가 `/last_lidar`와 UDP에 속도 0으로 나간다 |
| B-11 | 특성 기록 | GPS 시계가 10초 앞섰다가 정상으로 돌아오면 sanity filter가 미래에 고정돼 map이 25 scan 동안 나오지 않는다. stale·미래 map은 나가지 않아 안전 쪽이다. gtest property로만 기록하고 단언하지 않는다 |
| B-15 | 수정함(2026-10-15) | 변경 전부터 있던 결함이다(변경 전 바이너리도 같은 스택). common geometry에서 종료할 때 `frame_boundary::Publisher`의 `ros::WallTimer`가 roscpp `TimerManager` 싱글턴보다 늦게 파괴되어 `boost::lock_error`로 abort했다. 이제 `main.cpp`가 worker thread join과 집계 출력 뒤에 Publisher를 먼저 해제한다. 이 abort 때문에 돌지 못하던 실제 Main의 LeakSanitizer 종료 검사가 실행된다 |
| B-5 | 결정 필요 | v1 typed topic의 유지·배타화·제거 |
| B-8 | B1 | legacy 기하 경로의 pitch 이중 적용 |
| B-9 | 미측정 | 객체 하나의 기하가 무효이면 통합 프로필은 legacy 프레임 전체를 보류한다. 가용률 영향은 재지 않았다 |
| B-10 | 판제 확인 필요 | GPS 복구 직후 약 9 scan의 legacy 공백 |
| B-18 | 후속 | legacy 변환(`PlannerObjectAdapter`)은 아직 `frame_boundary::Current()`를 읽는다. 현재 동기 구조에서는 v2와 같은 scan context다 |
| B-19 | 후속 | GPS 표본 append 거부 사유가 진단에 남지 않는다 |
| B-21 | 기록 | 이전 증거 manifest가 `main.launch` SHA를 고정하고 있어 B0 이후 값과 다르다. CI에서 참조하지 않는다 |
| B-22 | 기록 | 진단 `legacy_output`에 새 값 `frame_invalid`가 생겼다. 저장소 안에서 이 값을 프로그램으로 읽는 곳은 없다 |
| B-25 | 기록 | nested workspace는 CI에서 빌드하지 않는다. Main gtest·rostest·nose는 로컬 gate다 |
| B-26 | 확인 필요 | `/lidar/object(_boxes)`를 최종 출력에서 뺐다(시각화). owner 확인이 필요하다 |

## 11. 검증 기록 — 2026-10-15

원본 로그와 runtime 하네스는 저장소 밖 호스트의 감사 디렉터리 `audit_work/asmc_b0_integration_20261015_103640`에 있다.
runtime 하네스는 WSL 전용(user·network namespace, private binfmt_misc)이라 저장소에 넣지 않았다. 아래 숫자는 이 날짜의
기록이며 계약값이 아니다.

> **정본:** commit 전 최종 검증(G8, 2026-10-16 KST)의 기계용 결과는 감사 디렉터리의 `results/TEST_RESULTS.json`이다.
> 아래 숫자와 다르면 그 JSON을 따른다.

### 2026-10-17 local 상시 출력 계약 보정

아래 G7/G8 수치는 당시의 상호 배타적 local/map 계약으로 측정한 **과거 증거**다. 현재 계약은 사용자의
최종 요구대로 유효한 local scan마다 `/lidar/local_object`를 발행하고, map pose가 유효한 같은 scan에는
`/lidar/map_object`를 추가 발행한다. 따라서 아래의 “동시 발행 0”과 local/map 개별 개수는 현재 합격 기준이나
예상 개수가 아니다. S08 미래 GPS(+10초)는 수신 시각보다 100ms 넘게 앞선 비정상 stamp로 거부하며 sanity
anchor를 오염시키지 않는다. 2026-10-17 재검증에서는 S08 50 scan 전부 local, map 41 scan, 미래·복구 구간
map 위반 0, 정상 GPS 복귀 후 세 번째 표본부터 local+map 복구를 확인했다. 전체 격리 행렬 27/27도 통과했다.

아래 값은 bag smoke 항목을 빼면 **G7 재검증** 기록이다.
- G7은 F1·F1b·B-15 수정(10절) 뒤의 소스를 처음부터 다시 빌드해 2026-10-15 19:20–21:20 KST에 측정했다.
- G8은 시험 주석, 시험 격리, 문서만 고친 뒤 같은 절차로 처음부터 다시 측정했다. production 바이너리 기계어는 G7과 같다.
- G8은 Main gtest 23/206, 행렬 30/30, `/lidar/local_object` 223, `/lidar/map_object` 950, S11·S16 각 20/20을 재현했다.

- G7 Main gtest: 23 binary / 206 case, 실패 0. 기존 15 binary / 132 case는 기준선과 같게 통과했다.
- G7 rostest(격리 namespace, 전용 master): `frame_boundary` 14/14(파일 무수정), `frame_boundary_v2` 8/8,
  `camera_objects_adapter` 1/1, `main_integration_gating` 1/1.
- G7 nose: Main 26(6+15+5), `asmc_msgs` 1+9. 동결 message 6개의 sha256과 MD5는 변경 전과 같다.
- G7 CI offline Python: 67 명령 / 562 case, 실패 0. `check_documentation_migration.py`와 모든 계약 validator가 통과했다.
- G7 `-Wall -Wextra` 진단: 고유 위치 44개로 기준선과 같고 새 경고 키는 0이다.
- G7 격리 runtime 행렬: B0 측정 30회(시나리오 27개와 S11 반복 3회), 1,287 scan. 30회 모두 PASS다(FAIL 0, INCONCLUSIVE 0).
  발행 수는 `/lidar/local_object` 223, `/lidar/map_object` 950이다. v2 두 topic 동시 발행 0,
  `/lidar/local_object`와 legacy 동시 발행 0, map pose 무효 scan의 map 출력 0, 좌표 0 더미 객체 0이었다.
  B0 측정 실행은 모두 `planner_udp_enabled:=false`였고 IPv4 `Udp.OutDatagrams` 증가 0, IPv6 `Udp6OutDatagrams` 증가 0,
  strace UDP 송신 0이었다.
- G7 반복 실행(재시도 없음, 실행마다 새 namespace): S11(IMU 무효, F1) 연속 20회 20/20 PASS, S16(stamp 0 CAN, F1b)
  연속 20회 20/20 PASS. 두 시나리오 모두 20회의 관측 순서가 한 가지이고 oracle과 같았다.
- G7 양성 대조(별도 namespace, loopback만, 라벨 `POSITIVE_CONTROL_S26`): production UDP 기본값으로 38 datagram이
  127.0.0.1:7510으로 나갔고 legacy를 발행한 scan 38개와 같았다.
- G8 bag smoke(최종 검증, `tunnel_yolo_fusion.bag`, production 필터, fusion ON, UDP OFF): 402 scan,
  `/lidar/map_object` 170, `/lidar/local_object` 119, NONE 113. 동시 발행 0, 좌표 0 더미 객체 0, UDP 송신 0이었다.
  G7 실행 값은 171/117/114였다. bag 재생은 실시간이라 GPS 상태 전환 경계의 scan 1~2개가 실행마다 달라질 수 있다.
  bag sha256은 실행 전후가 같았다. 실데이터 검증이 아니라 참고용 smoke다.
