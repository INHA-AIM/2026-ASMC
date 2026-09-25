# `asmc_perception` runtime

> **상태:** P4-C5 — 신호 차량 loop 수락 + camera/BEV/lane runtime·support·config 통합

분산된 camera semantic, metric BEV/lane과 traffic-light를 하나의 인지 실행 경계로 조합한다.
camera semantic, BEV/fusion/lane과 신호등 ROS node 구현 및 실행 support library·config는 이
패키지가 소유한다. 기존 package의 같은
executable과 Python import는 호환 전달자로, 기존 config 경로는 SHA로 고정한 mirror로 보존하며
topic과 node 이름은 바꾸지 않았다. `data_collection`과 팀원 개발 중인 LiDAR/fusion은 독립
패키지·workspace이며 이 launch가 소유하거나 시작하지 않는다.

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

신호등이 필요한 경우에만 명시적으로 추가한다.

```bash
./scripts/perception.sh \
  start_traffic_light:=true
```

bag 기록·재생은 별도 `data_collection` 진입점을 사용한다.

```bash
roslaunch data_collection bag_recorder.launch profile:=full_raw
```

LiDAR/fusion은 별도 nested workspace의 `Main`, `Msg`와 Velodyne overlay를 사용하며 담당 팀원
handoff 전에는 `asmc_perception` launch에 포함하지 않는다.

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

## P4-C3 판제 consumer

`behavior_planner`는 이제 `/perception/traffic_light/detection`을 권위 입력으로 구독한다. 기존
`/traffic_light/state` 구독도 유지하지만 typed message를 한 번도 받지 못한 구 실행을 위한
fallback일 뿐이다. typed message를 받은 뒤에는 다음 규칙을 적용한다.

- `valid=false` 또는 `stale=true`: `UNKNOWN`, legacy String으로 우회하지 않음
- typed 수신이 local stale timeout을 넘김: `UNKNOWN`, legacy String으로 우회하지 않음
- typed producer를 한 번도 보지 못함: fresh legacy String 사용

기존 판제 통합 launch가 시작하는 executable은
`traffic_light/live_infer.py`에서 `asmc_perception/traffic_light_node.py`로 바뀌었다. topic 이름은
삭제하거나 remap하지 않았으며 producer 경로 변경은 두 launch 파일의 migration 주석과
`behavior_planner/README.md`에 기록돼 있다. unit/build, live ROS 연결과 MORAI 차량
정지·재출발까지 통과했다. 다만 모든 consumer/launch의 legacy 참조가 없어지기 전에는 기존
topic을 제거하지 않는다.

## P4-C4 신호 수락시험

production `behavior_node`, 현재 MGeo와 전역경로를 쓰되 Frenet/MPC/UDP 출력은 시작하지 않는
격리된 ROS decision-loop 시험을 제공한다.

```bash
# asmc-ros-noetic 컨테이너
roslaunch asmc_perception traffic_light_closed_loop_acceptance.launch
cat /tmp/asmc_traffic_light_closed_loop_acceptance.json
```

고정 pose는 A728 정지선 8.95 m 전방이다. 시험은 같은 typed stream에서
`GREEN → KEEP`, `RED → STOP(map 목표점 포함)`, `GREEN → 새 KEEP command_id`를 검사한다.
기준은 `config/baselines/perception_traffic_light_closed_loop_v1.yaml`이다. 이 시험은 판제 판단
loop의 수락 근거이며 실제 simulator의 제동거리·MPC·UDP actuator까지 통과했다는 뜻은 아니다.

실제 `behavior → Frenet → MPC → /ctrl_cmd → UDP → MORAI` 차량 시험은 다음처럼 별도 opt-in한다.
기본 launch는 제어가 잠겨 있으며 아래 두 flag가 모두 있어야 Ego를 텔레포트하고 움직인다.

```bash
# controlled simulator only
roslaunch asmc_perception traffic_light_vehicle_acceptance_stack.launch \
  run_acceptance:=true allow_vehicle_control:=true \
  repo_root:=/root/ws grpc_host:=host.docker.internal
cat /tmp/asmc_traffic_light_vehicle_acceptance.json
```

MPC 출력은 `/acceptance/ctrl_cmd`에 격리되고 시험 node가 원래 Ego 상태를 저장한 뒤에만
`/ctrl_cmd`로 전달한다. 종료 시 full brake, gate close, pose·speed·gear·control mode 복원을
수행한다. 26.R1 local 실측에서 적색은 물리 정지선 5.21 m 전에 `0.0 m/s`로 멈췄고, 녹색에서
1.33 m 재진행·2.27 m/s를 확인했다. 근거는
`config/baselines/perception_traffic_light_vehicle_closed_loop_v1.yaml`이다. 이 결과는 synthetic
typed 신호 이후 제어 loop 수락이며 camera 분류 정확도 시험은 아니다.

## P4-C4/C5 물리 이식 경계

다음 실제 node 구현은 `src/asmc_perception/src/asmc_perception/nodes/`로 이동했다.

- camera semantic inference와 visualizer
- per-view BEV, 3-view fusion과 BEV visualizer
- lane vectorizer와 lane visualizer

canonical launch는 모두 `pkg="asmc_perception"`으로 실행한다. 기존
`camera_semantic_perception`, `drivable_bev` executable은 같은 이름의 얇은 호환 전달자다.
P4-C5에서 camera backend, runtime BEV/lane 수학과 config 정본도 각각
`asmc_perception.camera`, `asmc_perception.bev`, `asmc_perception/config`으로 옮겼다. 기존 Python namespace는 canonical
symbol을 다시 export하고 기존 YAML은 byte-identical mirror로 유지한다.

live 3-camera→BEV→lane 출력은 `config/baselines/perception_runtime_move_smoke_v1.yaml`에
고정한다. 입력 bag과 recorder의 소유권은 계속 `data_collection`에 있다.

P4-C5 representative raw bag을 0.5배속으로 두 진입점에 각각 재생한 결과 front class-ID 79,
fused BEV 38, lane observation 43개의 공통 source stamp에서 payload mismatch가 모두 0이었다.
ROS connection-local `header.seq`만 비교에서 제외하고 frame·stamp·config SHA와 전체 payload는
그대로 비교한다. 실행 도구는 `tools/audit/run_perception_runtime_parity.py`, ownership·config SHA와
재검증 근거는 `config/baselines/perception_support_config_parity_v1.yaml`이다.

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
# GPU container; choose a new output directory on every run
python3 tools/audit/run_perception_runtime_parity.py \
  --bag /path/to/camera_raw.bag --output-dir /tmp/asmc_p4c5_parity
roslaunch asmc_perception traffic_light_closed_loop_acceptance.launch
# controlled simulator only; explicit opt-in required
roslaunch asmc_perception traffic_light_vehicle_acceptance_stack.launch \
  run_acceptance:=true allow_vehicle_control:=true
```

support/config parity는 완료됐지만 offline 도구 이관과 모든 legacy consumer 전환은 남아 있다.
따라서 기존 package를 아직 삭제하거나 legacy topic을 remap해서 숨기지 않는다. 다음 P4-C6은
기존 camera/BEV/traffic package의 남은 runtime 참조와 offline 자산을 분류해 호환 package
closeout 조건을 확정한다. `data_collection`은 독립 package로 유지한다.
`src/lidar+sensorfusion`은 담당 팀원이 개발 중인 독립 workspace이므로
그 파일과 내부 `Msg`·`morai_msgs`는 건드리지 않는다. LiDAR 완성·handoff 뒤에만 별도 P4-D로
canonical adapter와 message 전환을 진행한다.
