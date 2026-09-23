# `asmc_perception` façade

> **상태:** P4-B compatibility façade — 기존 인지 source는 아직 이동하지 않음

분산된 camera semantic, metric BEV/lane, traffic-light, nested LiDAR/fusion과 recorder를
하나의 운영 진입점에서 조합한다. 이 패키지는 아직 각 구현을 복사하거나 소유하지 않으며,
기존 executable·topic을 그대로 사용한다.

## 실행

기본값은 competition-legal 3-camera lane pipeline만 실행한다.

```bash
roslaunch asmc_perception perception.launch
```

필요한 component만 명시적으로 추가한다.

```bash
roslaunch asmc_perception perception.launch \
  start_traffic_light:=true

roslaunch asmc_perception perception.launch \
  start_camera_lane:=false \
  start_recorder:=true \
  recorder_profile:=full_raw
```

LiDAR/fusion은 아직 별도 nested workspace의 `Main`, `Msg`와 Velodyne overlay를 요구한다.
그 overlay를 먼저 source한 환경에서만 다음처럼 켠다.

```bash
roslaunch asmc_perception perception.launch \
  start_camera_lane:=false \
  start_lidar_fusion:=true
```

## P4-B 경계

- camera lane은 v3 센서셋과 SHA로 고정한 `cameras_v3.yaml`을 사용한다.
- v2와 v3의 네 camera pose, intrinsics, period와 UDP 설정이 동일한지는 unit test가 검사한다.
- 신호등은 기존 `/traffic_light/state`, `/traffic_light/detail`을 유지한다.
- LiDAR는 기존 `/lidar/tracked_objects_map`, `/last_lidar`를 유지한다.
- 신규 `asmc_msgs/TrafficLightDetection`, `asmc_msgs/TrackedObjectArray`는 additive 계약이다.
- typed producer가 아직 없으므로 canonical typed topic을 발행한다고 가정하면 안 된다.
- source-header propagation과 consumer cutover를 검증한 후 후속 PR에서 병행 발행한다.
- Capture/gRPC GT, simulator object/traffic truth는 deployment 입력으로 사용하지 않는다.

기계 판독 상태는 `config/runtime.yaml`, 파트 간 topic·message 정본은
[`perception_runtime_topic_contract_v1.yaml`](../../config/contracts/perception_runtime_topic_contract_v1.yaml),
전체 이관 gate는 [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md)를 따른다.

## 검증

```bash
python3 -m unittest src/asmc_perception/test/test_facade_contract.py
python3 tools/audit/validate_perception_contract.py
roslaunch --nodes asmc_perception perception.launch
```

P4-C 이전에는 기존 package를 삭제하거나 이 façade가 legacy topic을 remap해서 숨기지 않는다.
