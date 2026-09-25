# Perception runtime topic contract v1

> **상태:** Frozen additive interface; P4-C6 package closeout, bag parity와 MORAI 차량 loop 검수 완료
> **기계 정본:** `config/contracts/perception_runtime_topic_contract_v1.yaml`
> **실행 façade:** `src/asmc_perception/launch/perception.launch`

이 계약은 신호등과 LiDAR 객체 결과를 판제가 소비할 canonical ROS1 타입으로 옮기기 위한
경계다. P4-B에서 타입과 topic 의미를 먼저 고정했고 P4-C1에서 신호등 typed producer를
병행 연결했으며 P4-C2에서 대표 bag·stale·recovery parity를 검증했다. P4-C3에서 판제
`behavior_planner`가 typed 결과를 우선 소비하도록 연결했다. P4-C4에서 실제 ROS
`behavior_node`를 사용한 `GREEN→KEEP→RED→STOP→GREEN→KEEP` 판단 loop를 고정하고, 별도 opt-in
시험으로 Frenet/MPC/UDP와 실제 MORAI 차량 정지·재출발까지 확인했다. P4-C5에서 camera/BEV/lane
runtime support와 config 소유권을 `asmc_perception`으로 옮기고 representative bag의
canonical/compatibility payload parity를 통과했다. P4-C6에서 offline·training·model
자산까지 `asmc_perception`으로 이관하고 구 camera/BEV/traffic package를 제거했지만,
topic·node·message 계약은 그대로 유지했다. LiDAR canonical
topic은 여전히 **발행된다고 가정하면 안 되며** 기계 정본의
`producer_status`가 실제 producer 상태의 권위다.

## 1. 메시지

| 타입 | 역할 | 핵심 규칙 |
|---|---|---|
| `asmc_msgs/TrafficLightDetection` | 신호등 분류·신뢰도·freshness | 실제 추론 입력 image header를 보존 |
| `asmc_msgs/TrackedObject` | LiDAR/fusion tracked object | array header frame의 OBB·절대속도, SI 권위 |
| `asmc_msgs/TrackedObjectArray` | 동일 시점 객체 배열 | LiDAR source header 보존 |

`TrackedObject*` 필드와 MD5는 nested `Msg/TrackedObject*`와 같게 설계했다. 그러나 ROS datatype
이름은 다르므로 이름이 다른 publisher/subscriber가 자동 연결된다고 가정하지 않는다. producer
이관 또는 명시적 adapter와 bag parity가 필요하다.

`TrafficLightDetection`의 `state`는 다음과 같다.

| 값 | 의미 | `valid` |
|---:|---|---|
| 0 | UNKNOWN | false |
| 1 | NOT_DETECTED | false |
| 2 | RED | true |
| 3 | YELLOW | true |
| 4 | GREEN | true |
| 5 | RED_YELLOW | true |
| 6 | RED_LEFT | true |
| 7 | GREEN_LEFT | true |
| 8 | LEFT | true |

`stale=true`인 결과는 항상 무효다. `candidate_labels`와
`candidate_confidences`는 길이가 같고 confidence 내림차순이어야 한다. `label`은 기존 문자열
consumer와 비교하기 위한 사람이 읽는 값이지 enum 대신 사용하는 별도 권위가 아니다.

## 2. Canonical topic

| Topic | Type | Frame/stamp | 현재 상태 |
|---|---|---|---|
| `/perception/traffic_light/detection` | `asmc_msgs/TrafficLightDetection` | `camera_extra`, 실제 입력 image header | P4-C4 behavior 및 MORAI actuator closed-loop 통과 |
| `/perception/lidar/tracked_objects` | `asmc_msgs/TrackedObjectArray` | `map`, LiDAR source stamp | 선언됨, 미발행 |
| `/perception/lidar/drivable_tracked_objects` | `asmc_msgs/TrackedObjectArray` | `map`, 같은 source stamp | 선언됨, 미발행 |

세 topic은 동적 인지 결과라 latch하지 않는다. 신호등 adapter가 단순 ROS 수신 시각이나 가장
가까운 camera header를 붙이는 것은 허용하지 않는다. 실제 추론에 쓴 정확한 image header를
producer가 전달해야 한다.

Tracked object의 `cluster_center`, `obb_center`, `obb_corners`, `obb_yaw`, `velocity_mps`는 모두
array `header.frame_id` 기준이다. canonical runtime은 `map`을 사용하며 위치·속도는 SI가
권위다. `speed_kph`는 화면과 회귀 확인 편의를 위한 파생값이다.

## 3. 호환 topic

다음 topic은 consumer 전환이 끝날 때까지 유지한다.

| Topic | Type | 종료 조건 |
|---|---|---|
| `/traffic_light/state` | `std_msgs/String` | 모든 consumer가 typed detection 사용 |
| `/traffic_light/detail` | `std_msgs/String` JSON | String/detail parity와 consumer 전환 |
| `/lidar/tracked_objects_map` | `Msg/TrackedObjectArray` | `asmc_msgs` bag parity와 참조 0 |
| `/last_lidar` | `Msg/TrackedObjectArray` | 최종 HD Map filter parity와 참조 0 |

기존 topic을 remap으로 숨기지 않는다. 일정 기간 legacy와 canonical을 병행 발행하고 같은 source
frame에서 객체 수, ID, OBB, 속도, dynamic state 및 신호 label/confidence/stale을 비교한다.

## 4. Façade 실행 경계

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_perception perception.launch
roslaunch asmc_perception perception.launch start_visualization:=true
```

기본 실행은 v3 sensor-set provenance를 가진 3-camera lane pipeline이며 traffic-light만 선택적으로
추가한다. LiDAR/fusion은 아직 nested workspace, recorder는 `data_collection`이 독립적으로
소유한다. `asmc_perception`은 두 component를 시작하거나 `Main`, `Msg`, 중복 `morai_msgs`를
소유·삭제하지 않는다.

v2와 v3는 VLP-16 주기가 다르지만 네 camera의 pose, intrinsics, period와 UDP 설정은 같다.
`cameras_v3.yaml`은 v3 원본 전체 SHA를 기록하고 unit test가 camera parity를 고정한다.

## 5. 이관 gate

Traffic-light producer 전환:

1. **완료:** 실제 추론 image header를 typed 결과에 복사한다.
2. **완료:** representative bag에서 기존/new String과 typed label/confidence/stale/recovery parity를 검증한다.
3. **완료:** 판제 consumer를 typed 우선으로 바꾸고 invalid/stale 시 legacy로 우회하지 않는 unit·live 연결을 확인한다.
4. **완료:** 격리된 실제 ROS behavior decision loop에서 적색 STOP·녹색 새 KEEP을 확인한다.
5. **완료:** 실제 simulator에서 Frenet/MPC/UDP를 포함한 제동 정지·재출발과 원상 복구를 확인한다.
6. 모든 named consumer와 launch의 compatibility topic 참조가 0인 뒤 retire 여부를 결정한다.

2번의 기계 판독 근거는
`config/baselines/perception_traffic_light_parity_v1.yaml`이다. 3번의 구현 위치는
`behavior_planner/src/signal/ros_traffic_light_provider.*`이며, 4번은 제어 명령까지 연결하는
별도 수락시험이므로 producer·adapter 통합과 구분한다. 4번 근거는
`config/baselines/perception_traffic_light_closed_loop_v1.yaml`이며 trajectory/controller/UDP는
명시적으로 제외한다. 5번 근거는
`config/baselines/perception_traffic_light_vehicle_closed_loop_v1.yaml`이다. 실제 controller 출력은
저장 전 `/acceptance/ctrl_cmd`로 격리하고, 명시적 gate를 연 뒤에만 bridge `/ctrl_cmd`로 전달한다.
이 시험의 입력은 synthetic typed detection이므로 camera classifier 정확도 근거로 사용하지 않는다.

LiDAR producer 전환:

1. nested `Msg`와 `asmc_msgs`의 field, MD5, 단위와 frame parity를 검증한다.
2. bag에서 객체 수·ID·OBB·속도·dynamic state를 비교한다.
3. 구 `link_node` mask filter를 최종 `/hd_map` 영역 consumer로 교체하고 parity를 확인한다.
4. 모든 consumer 참조가 0이 된 뒤에만 nested `Msg`를 삭제한다.

## 6. 검증

```bash
python3 tools/audit/validate_p1b_contract.py
python3 tools/audit/validate_perception_contract.py
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
python3 -m unittest tools/audit/test_perception_contract.py
python3 -m unittest src/asmc_perception/test/test_facade_contract.py
```

ROS 환경에서는 `asmc_msgs` generated serialization과 façade launch parsing을 추가로 확인한다.
