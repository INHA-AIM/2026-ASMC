# `asmc_perception` runtime

> **상태:** P4-C2 — canonical traffic-light producer + bag parity baseline

분산된 camera semantic, metric BEV/lane, traffic-light, nested LiDAR/fusion과 recorder를
하나의 운영 진입점에서 조합한다. camera/BEV/recorder는 아직 기존 executable을 사용하며,
신호등 producer는 이 패키지로 먼저 이식됐다. 기존 executable·topic은 parity 확인 전까지 보존한다.

## 실행

기본값은 competition-legal 3-camera lane pipeline만 실행한다. CUDA, `twinlite_morai`, 외부
TwinLite 소스가 필요한 까닭에 WSL/Linux host에서는 다음 wrapper를 사용할 수 있다.

```bash
./scripts/perception.sh
```

이 wrapper는 runtime 기능이 아니라 편의·오실행 방지 도구다. GPU 컨테이너가 꺼져 있으면
시작하고 그 안에서 아래 `roslaunch`를 실행할 뿐이며, 어떤 component를 켤지는 전달한
launch argument가 결정한다. 지금까지처럼 컨테이너에 직접 들어가 실행해도 결과는 같다.

```bash
docker exec -it asmc-perception-train bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_perception perception.launch
```

`roslaunch asmc_perception perception.launch`를 직접 실행하려면 반드시
`asmc-perception-train` 컨테이너 안에서 실행한다. `asmc-ros-noetic`은 bridge, build와 GUI용이며
camera inference 실행 환경이 아니다.

필요한 component만 명시적으로 추가한다.

```bash
./scripts/perception.sh \
  start_traffic_light:=true

./scripts/perception.sh \
  start_camera_lane:=false \
  start_recorder:=true \
  recorder_profile:=full_raw
```

LiDAR/fusion은 아직 별도 nested workspace의 `Main`, `Msg`와 Velodyne overlay를 요구한다.
그 overlay를 먼저 source한 환경에서만 다음처럼 켠다.

```bash
./scripts/perception.sh \
  start_camera_lane:=false \
  start_lidar_fusion:=true
```

## 시각화

debug image publisher를 함께 켠다.

```bash
./scripts/perception.sh start_visualization:=true
```

화면은 GUI 환경이 있는 `asmc-ros-noetic` 컨테이너에서 연다. 아래 명령은 먼저
`docker exec -it asmc-ros-noetic bash`로 들어간 각각의 터미널에서 실행한다.

```bash
asmc-ros-gui rqt-image /perception/camera/debug/mosaic
asmc-ros-gui rqt-image /perception/bev/debug/fused
asmc-ros-gui rqt-image /perception/bev/debug/fused_quality
asmc-ros-gui rqt-image /perception/lane/debug/boundaries_bev
asmc-ros-gui rqt-image /perception/lane/debug/camera_reprojection_mosaic
```

`rqt_image_view`는 한 창에 한 topic을 선택한다. 추론 자체만 확인할 때는 camera mosaic,
차로 fitting은 `boundaries_bev`, 카메라 정합은 `camera_reprojection_mosaic`가 가장 직접적이다.
`start_visualization`은 개발·검수용이고 기본값은 false다. 평상시 주행에는 필요하지 않다.

`start_traffic_light`은 실제 producer를 켜는 옵션이다. 현재는 true로 켜면 전 구간에서 계속
추론하며, HD맵의 `TRAFFIC_LIGHT_REQUIRED` 구간(A236~A308)에 따른 process/compute 활성화는
판제 supervisor와 함께 연결할 후속 항목이다.

## P4-C1 경계

- camera lane은 v3 센서셋과 SHA로 고정한 `cameras_v3.yaml`을 사용한다.
- v2와 v3의 네 camera pose, intrinsics, period와 UDP 설정이 동일한지는 unit test가 검사한다.
- 신호등은 기존 `/traffic_light/state`, `/traffic_light/detail`과 typed
  `/perception/traffic_light/detection`을 병행 발행한다.
- typed 결과는 실제 추론에 사용한 `camera_extra` image header를 그대로 보존한다.
- LiDAR는 기존 `/lidar/tracked_objects_map`, `/last_lidar`를 유지한다.
- 신규 `asmc_msgs/TrafficLightDetection`, `asmc_msgs/TrackedObjectArray`는 additive 계약이다.
- LiDAR typed producer는 아직 없으므로 해당 canonical topic을 발행한다고 가정하면 안 된다.
- 기존 `traffic_light` package는 대표 bag parity와 consumer cutover 전까지 삭제하지 않는다.
- Capture/gRPC GT, simulator object/traffic truth는 deployment 입력으로 사용하지 않는다.

## P4-C2 parity 회귀

대표 raw bag의 `camera_extra` 프레임을 기존 producer와 canonical producer에 한 장씩 같은
순서로 넣어 legacy String, typed result, source header, stale 전환과 복구를 비교한다.

```bash
# asmc-perception-train 컨테이너
roslaunch asmc_perception traffic_light_parity.launch \
  bag_path:=/data/rosbags/hd_map_validation/20260921_174512_kcity2025_manual_route_hdmap/20260921_174512_kcity2025_manual_route_hdmap_0.bag

# host
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
```

기준 결과는 `config/baselines/perception_traffic_light_parity_v1.yaml`이다. 이 통과는 producer
병행 발행의 근거이며 판제 consumer cutover 완료를 뜻하지는 않는다.

기계 판독 상태는 `config/runtime.yaml`, 파트 간 topic·message 정본은
[`perception_runtime_topic_contract_v1.yaml`](../../config/contracts/perception_runtime_topic_contract_v1.yaml),
전체 이관 gate는 [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md)를 따른다.

## 검증

```bash
python3 -m unittest src/asmc_perception/test/test_facade_contract.py
python3 -m unittest src/asmc_perception/test/test_traffic_light_contract.py
python3 -m unittest src/asmc_perception/test/test_traffic_light_parity.py
python3 tools/audit/validate_perception_contract.py
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
roslaunch --nodes asmc_perception perception.launch
```

P4-C parity 완료 전에는 기존 package를 삭제하거나 legacy topic을 remap해서 숨기지 않는다.
