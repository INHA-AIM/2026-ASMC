# `asmc_perception` runtime

> **상태:** P4-C6 — camera/BEV/lane/traffic runtime·offline·training 패키지 통합 완료

분산된 camera semantic, metric BEV/lane과 traffic-light를 하나의 인지 실행 경계로 조합한다.
camera semantic, BEV/fusion/lane과 신호등 ROS node 구현, 실행 support library·config, 모델,
TwinLite 학습 adapter와 offline BEV audit은 이 패키지가 소유한다. P4-C6에서 기존
`camera_semantic_perception`, `drivable_bev`, `traffic_light` ROS 패키지와 `src/perception` 그룹을
제거했지만 topic과 node 이름은 바꾸지 않았다. `data_collection`과 팀원 개발 중인 LiDAR/fusion은 독립
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

### `camera_bev`와 `camera_bev_fusion`

`camera_bev` 노드는 front/left/right 카메라의 semantic probability·road-marking을
카메라별 calibration으로 `base_link` 평면에 각각 투영한다. 출력은
`/perception/bev/debug/{view}/*`이며, 현재는 후단 fusion의 내부 입력이자 개발
비교 계층이다.

`camera_bev_fusion`은 세 투영 결과의 source timestamp를 맞추고, calibration 품질·
가시 영역 가중치로 겹침을 해결해 하나의 `/perception/bev/camera/*`를 만든다.
`coverage`, `source_count`도 이 단계에서 만들며 lane vectorizer가 소비할 운영 정본은
이 fused output이다. 두 노드는 중복 알고리즘이 아니라 **카메라별 투영 → 다중
카메라 융합**의 순차 단계다.

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

## P4 closeout MORAI live 확인

이 절은 package 통합 PR 전 사람이 MORAI를 주행하며 확인하는 최소 절차다.
MORAI UI에서 실행 release에 맞는 `*_local.json`과
`ASMC_sensor_set_v3_local.json`을 불러온 뒤 Play/Connect한다.

```bash
# Terminal A — asmc-ros-noetic container, local UDP bridge
docker exec -it asmc-ros-noetic bash
cd /root/ws
./scripts/udp_bridge.sh

# Terminal B — WSL/Linux host (container 안이 아님), canonical perception
cd /home/aim06/projects/2026-ai-sw-mobility-competition/2026-ASMC
./scripts/perception.sh \
  start_traffic_light:=true \
  start_visualization:=true \
  lane_enable_shared_family_debug:=true \
  lane_enable_boundary_family_candidate:=true
```

별도 터미널에서 ROS 토픽을 확인한다. `rostopic hz`는 계속 대기하므로 항목별로
`Ctrl+C`로 나오거나 터미널을 나눠 실행한다.

```bash
# Terminal C — asmc-ros-noetic container
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# MORAI v3 camera input: front/left/right 약 20 Hz, extra 약 10 Hz
rostopic hz /image_jpeg/compressed
rostopic hz /cam_left/image_jpeg/compressed
rostopic hz /cam_right/image_jpeg/compressed
rostopic hz /cam_extra/image_jpeg/compressed

# canonical runtime output: GPU 상태에 따라 입력보다 느릴 수 있지만 계속 갱신되어야 함
rostopic hz /perception/camera/front/drivable/probability
rostopic hz /perception/bev/camera/drivable_probability
rostopic hz /perception/lane/boundary_observations
rostopic hz /perception/lane/boundary_family
rostopic hz /perception/traffic_light/detection

# 계약·상태 sample
rostopic echo -n 1 /perception/lane/boundary_family/header
rostopic echo -n 1 /perception/lane/boundary_family/operating_domain
rostopic echo -n 1 /perception/lane/boundary_family/algorithm_id
rostopic echo -n 1 /perception/lane/boundary_family/deployment_allowed
rostopic echo -n 1 /perception/lane/diagnostics
rostopic echo -n 1 /perception/bev/diagnostics
rostopic echo -n 1 /perception/traffic_light/detection
```

음영구간 adapter는 제어와 연결하지 않고 별도로 켠다.

```bash
# Terminal D — asmc-ros-noetic container
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch

# Terminal E — adapter output
rostopic hz /planning/debug/lane_boundary_family/strips
rostopic echo -n 1 /planning/diagnostics/lane_boundary_family_adapter
```

시각화는 각각 별도 `asmc-ros-noetic` 터미널에서 연다.

```bash
asmc-ros-gui rqt-image /perception/camera/debug/mosaic
asmc-ros-gui rqt-image /perception/bev/debug/fused
asmc-ros-gui rqt-image /perception/lane/debug/boundaries_bev
asmc-ros-gui rqt-image /perception/lane/debug/camera_reprojection_mosaic
asmc-ros-gui rviz
```

RViz의 Fixed Frame을 `base_link`로 두고 MarkerArray
`/perception/lane/debug/shared_family/markers`와
`/planning/debug/lane_boundary_family/strips`를 추가한다. 음영 ODD 밖의
`valid=false` 또는 정지·disconnect 후 stale/marker clear는 정상 fail-safe다.
음영구간은 A448 종료 후부터 A126 종료까지이며 회피용 반대 차로는 A131이다.
이 구간에서는
`frame_id=base_link`, `operating_domain=shadow_two_way_v1`, 동결 algorithm ID,
`deployment_allowed=false`를 확인한다. observed는 청록, structure-inferred는 주황,
invalid는 연결하지 않은 실제 공백으로 보여야 한다.

신호등은 A236~A308에서 실제 색과 typed
`/perception/traffic_light/detection`, 호환 `/traffic_light/state`의 전환을 같이 확인한다.
이 live 검사에서는 `/ctrl_cmd`를 발행하지 않으므로 수동 주행 중에 사용해도 된다.

## P4-C1 경계

- camera lane은 v3 센서셋과 SHA로 고정한 `cameras_v3.yaml`을 사용한다.
- v2와 v3의 네 camera pose, intrinsics, period와 UDP 설정이 동일한지는 unit test가 검사한다.
- 신호등은 기존 `/traffic_light/state`, `/traffic_light/detail`과 typed
  `/perception/traffic_light/detection`을 병행 발행한다.
- typed 결과는 실제 추론에 사용한 `camera_extra` image header를 그대로 보존한다.
- LiDAR는 기존 `/lidar/tracked_objects_map`, `/last_lidar`를 유지한다.
- 신규 `asmc_msgs/TrafficLightDetection`, `asmc_msgs/TrackedObjectArray`는 additive 계약이다.
- LiDAR typed producer는 아직 없으므로 해당 canonical topic을 발행한다고 가정하면 안 된다.
- traffic-light 체크포인트는 `models/traffic_light/best.pt`가 정본이다.
- Capture/gRPC GT, simulator object/traffic truth는 deployment 입력으로 사용하지 않는다.

## P4-C2 parity 회귀

대표 raw bag의 `camera_extra` 프레임을 기존 producer와 canonical producer에 한 장씩 같은
순서로 넣어 legacy String, typed result, source header, stale 전환과 복구를 비교한 동결 시험이다.
호환 producer 패키지는 P4-C6에서 제거했으므로 이 A/B harness를 현재 HEAD에서 다시 실행하지
않는다. 결과와 입력·체크포인트 SHA는 baseline validator로 확인한다.

```bash
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

canonical launch는 모두 `pkg="asmc_perception"`으로 실행한다. P4-C5에서 camera backend,
runtime BEV/lane 수학과 config 정본도 각각
`asmc_perception.camera`, `asmc_perception.bev`, `asmc_perception/config`으로 옮겼다. 기존 Python namespace는 canonical
symbol을 다시 export하던 호환 package와 config mirror는 P4-C6 검증 후 제거했다.

재학습 코드는 `asmc_perception.training.twinlite`, offline replay·pseudo-GT·audit 코드는
`asmc_perception.offline.bev`와 `tools/bev/`에 분리했다. 이 namespace는 차량 runtime launch가
import하거나 시작하지 않는다. 긴 실험 이력은 `docs/`에 보존한다.

live 3-camera→BEV→lane 출력은 `config/baselines/perception_runtime_move_smoke_v1.yaml`에
고정한다. 입력 bag과 recorder의 소유권은 계속 `data_collection`에 있다.

P4-C5 representative raw bag을 0.5배속으로 두 진입점에 각각 재생한 결과 front class-ID 79,
fused BEV 38, lane observation 43개의 공통 source stamp에서 payload mismatch가 모두 0이었다.
ROS connection-local `header.seq`만 비교에서 제외하고 frame·stamp·config SHA와 전체 payload는
그대로 비교했다. 당시 도구는 호환 package 제거와 함께 retired 처리했고, 측정값·input SHA와
canonical config/model SHA는 `config/baselines/perception_support_config_parity_v1.yaml`에 보존한다.

기계 판독 상태는 `config/runtime.yaml`, 파트 간 topic·message 정본은
[`perception_runtime_topic_contract_v1.yaml`](../../config/contracts/perception_runtime_topic_contract_v1.yaml),
전체 이관 gate는 [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md)를 따른다.

## 검증

```bash
python3 -m unittest src/asmc_perception/test/test_facade_contract.py
python3 -m unittest src/asmc_perception/test/test_traffic_light_contract.py
python3 tools/audit/validate_perception_contract.py
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
roslaunch --nodes asmc_perception perception.launch
roslaunch asmc_perception traffic_light_closed_loop_acceptance.launch
# controlled simulator only; explicit opt-in required
roslaunch asmc_perception traffic_light_vehicle_acceptance_stack.launch \
  run_acceptance:=true allow_vehicle_control:=true
```

P4-C6에서 camera/BEV/traffic 호환 package를 제거하고 offline·training 자산까지 이 패키지로
이관했다. legacy topic은 외부 소비자 호환 계약이므로 package 삭제와 무관하게 계속 병행한다.
`data_collection`은 독립 package로 유지한다.
`src/lidar+sensorfusion`은 담당 팀원이 개발 중인 독립 workspace이므로
그 파일과 내부 `Msg`·`morai_msgs`는 건드리지 않는다. LiDAR 완성·handoff 뒤에만 별도 P4-D로
canonical adapter와 message 전환을 진행한다.
