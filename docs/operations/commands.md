# ASMC 실행 명령

`asmc-ros-noetic`의 `/root/ws`는 실행할 checkout입니다. 현재 worktree를 유지하고
다른 checkout을 빌드한 setup과 섞지 않습니다. 인자는 `name:=value`이며
[실제 launch 인자/기본값](launch-reference.md)은 소스에서 자동 생성합니다.

## 1. 개별 구성으로 전체 스택 실행

### Host와 빌드

```bash
export ASMC=$(git rev-parse --show-toplevel)
cd "$ASMC"
# GUI를 쓸 때만
xhost +local:docker
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash
```

LiDAR·판제·수집은 outer workspace만 빌드합니다. 주행 중 container 재생성이나 빌드를 하지 않습니다. 구 패키지는 폐기했고 재현은 고정 revision의 별도 checkout을 사용합니다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2 -l2 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

각 터미널은 아래 outer 환경만 적용합니다.
```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
```

PR #122의 터널 최적화·3뷰 fusion도 위 `/root/ws` 빌드에 포함됩니다.
개인 M1 overlay를 자동 source하는 과거 설정은 해제하고 같은 workspace로 통일합니다.
병합·설정 파일 변경만으로 기존 실행파일이나 실행 중인 노드가 갱신되지는 않습니다.
재빌드 후 다음 점검이 `source_matches: true`인지 확인합니다.

```bash
python3 /root/ws/src/asmc_lidar/tools/check_runtime_build.py
```

이 점검은 ROS 노드를 시작하지 않습니다. 실행 후에는 기존 LiDAR diagnostics의
`build_package_source_sha256`, `fusion_views=all`, `original_tunnel_pipeline=true`로 확인합니다.
[빌드·실행파일 확인과 재시작 주의](../../src/asmc_lidar/docs/three-view-fusion.md#빌드와-실제-실행파일-확인)를 따릅니다.

### 터미널별 실행

각 행은 별도 터미널에서 foreground로 실행합니다. 기본 실행 명령은 인자 없이 사용하고,
일부 모델 점검·복구에 필요한 옵션은 아래 별도 표에서 확인합니다. 이 구성과 아래 시나리오 통합 구성을 동시에 켜지 않습니다.

| 순서 | 실행 | 포함·주의 |
|---|---|---|
| A | `./scripts/udp_bridge.sh` | 허용 센서·기본 `/vehicle/state`, UDP 포트당 수신자 하나 |
| B | `roslaunch asmc_camera camera.launch` | 세 모델 상시 ACTIVE·발행, E8 기본; GUI 별도 |
| C | `roslaunch asmc_lidar lidar.launch` | 공용 local/map 추적·속도/운동·전달 필터; 카메라 모델 미실행 |
| D | `roslaunch asmc_planner autonomy_no_bridges_v2.launch` | **차량 제어 포함**, 새 공용 관측 직접 소비; GPS/IMU 위치와 Competition 종방향 실측 속도 조합, MPC/selector/재밍 어댑터 |

고주로 통합 브랜치에서는 D에 `highway_merge:=true`를 지정합니다.
판제 대시보드는 D에 포함되어 별도 실행하지 않습니다. 실행·보존·검증 범위는
[통합 안내](../../src/asmc_planner/docs/main-integration-20261026.md)를 참고하세요.

C/D는 기본 정본 조합입니다. source 선택 인자는 제거됐으며 [실행 기준선](../../src/asmc_lidar/docs/planning-runtime.md)을 따릅니다. 전체 미션·속도 정확도 인수는 후속입니다.

`roslaunch asmc_hd_map competition_map.launch`는 **선택 실행**입니다. 지도·미션 ROS 토픽과
현재 lane/context 또는 지도 RViz가 필요할 때 사용합니다. 현재 판제/Frenet과 LiDAR는 각 설정에
지정된 지도 파일을 직접 읽으므로 이 launch 없이도 지도 기반 주행·객체 필터가 작동합니다.
지도 토픽을 켜는 것만으로 판제의 파일 bundle이 자동 교체되지는 않습니다.
[역할과 소비 경계](../../src/asmc_lidar/docs/planning-runtime.md#hd맵-launch의-역할)를 참고하세요.

신호등의 단일 추론·발행자는 카메라 담당의 `asmc_camera`입니다. 판제는 결과만 소비하고
`traffic_light:=false` 인자는 제거되어 지정하지 않습니다. 중단/재개·주기 제어는 카메라의
`/perception/camera/traffic_light/set_mode`를 사용합니다. 판제 신호 규칙은 유지합니다. bridge `/vehicle/state`와
판제 `/planning/vehicle/state`는 다른 토픽·추정기이며 같은 토픽의 중복 producer는 금지합니다. 기본 D는 `/Ego_topic`의 허용된 종방향 속도만 `/planning/vehicle/gps_imu_state`의 GPS/IMU 위치와 결합합니다. 두 입력의 시각이 맞지 않거나 끊기면 상태를 무효로 처리합니다.

기본 카메라는 세 모델 모두 추론·발행합니다. 준비가 끝나기 전에 판제가 결과를 사용하지 않도록
상태와 새 결과의 validity/freshness를 확인합니다. [구간별 소비 안내](../handoffs/perception-to-planning.md)는
판제 사용 범위이며 생산자를 끄는 설정이 아닙니다. Lane 주행 소비는 음영 ODD에 한정하고 A126 진입 전에
유효 family를 확인합니다. A448은 접근 구간, 주변 차량 소비는 A153 끝/A451 진입까지입니다.

현재 차선 기본 모델은 Large v7 E8 TensorRT다. 사용자가 알려진 공급·freshness 제한을 확인한 뒤 전환을 승인했다.
[선정·실측·Medium 복구](../records/camera/2026-10-18-twinlite-large-deployment.md)를 확인한다.
공식 BDD100K Large의 추가학습 전 결과는 [별도 실험 실행 안내](../../src/asmc_camera/docs/bdd-pretrained-experiment.md)를
따릅니다. 이진 차선 시각화 실험이며 위 B의 운영 family를 대체하지 않습니다.

### 개별 디버그·시각화

일반 사용은 위 B의 `camera.launch`와 아래 `view.launch`를 각각 한 번 실행합니다.
두 launch 모두 `models:=all`이 기본이며, `camera.launch`는 `mode:=active`도 기본이므로
이 인자를 반복 지정할 필요가 없습니다. 추론과 시각화 선택은 서로 독립적입니다.

```bash
# B에서 이미 camera.launch를 실행했다면 추론을 또 시작하지 않음
roslaunch asmc_camera view.launch
# 별도 터미널: rqt는 한 개만 실행
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

E8은16종, pretrained는COCO80 checkpoint에서7종만 선택합니다. 입력은 YOLO/TwinLite 모두
전방544×960 B1·좌우480×640 B2, 목표15/15Hz·ResNet10Hz, confidence0.40·NMS0.50입니다.
Ego mask·보정·engine은 YAML/manifest가 소유합니다. 자동 PyTorch fallback은 없습니다.

같은 rqt에서 `/perception/camera/debug/mosaic/compressed`는 주행가능영역·차선 semantic,
`/perception/lane/debug/shared_family_bev/compressed`는 피팅된 family를 표시합니다.
`/perception/bev/debug/comparison/compressed`의 RGB·semantic은 같은 family 프레임의 IMU 지면을
재사용합니다. 별도 추론·피팅을 하지 않으며 `IMU shared`/`M0: 사유`로 적용 상태를 표시합니다.
Family 화면은 새 family 도착 즉시 갱신합니다. 신호등도 결과 도착 즉시 표시합니다.
큰 객체·semantic 합성과 RGB BEV 비교 화면은 최대10Hz이며, 추론/판제 발행 주기와 독립적입니다.
최신 결과 하나만 대기시키며 같은 source 결과 재표시를 신규 인지 공급으로 세지 않습니다.
정본 지면은 `cameras_v3.yaml`의 z=-0.36m이며 변경 후 카메라/투영 노드를 다시 시작해야 합니다.
Family는 원본 시각 IMU 자세·공동 폭 피팅을 기본 사용합니다. IMU/timing 근거가 부족하면
세 뷰 모두 고정 평면·폭 prior0.35로 처리하며 명목 거리는0.36m 그대로입니다.
[기본 전환과10분 검증](../records/camera/2026-10-18-joint-family-stabilization.md)을 확인합니다.
고정 평면 복구는 [IMU 비교·복구 안내](../records/camera/2026-10-18-joint-family-stabilization.md)를
따릅니다. [0.36m·표시 검증](../records/camera/2026-10-18-nominal-ground-and-arrival-view.md)을 함께 확인합니다.
`view.launch`는 시각화 worker만 시작하며, 추론 모델이나 rqt 창을 시작하지 않습니다.
일부 모델만 추론하거나 표시하는 `models` 옵션은 아래 선택 옵션 표에서 안내합니다.

`visualization.launch`는 `view.launch`와 구형 실행 경로가 공유하는 내부 시각화 구성입니다.
단독 기본 실행은 세 표시 플래그가 모두 false라 시각화 노드를 만들지 않습니다. 일반 사용자는
`view.launch`를 사용하며 둘을 동시에 실행하지 않습니다. 상세 legacy/raw 옵션은 재현·진단용입니다.

GUI 종료 후 사용한 process가 남지 않았는지
`rosnode list`, `pgrep -af rqt_image_view`로 확인하고 해당 터미널의 Ctrl+C로 정리합니다.
사용자/다른 작업의 프로세스를 일괄 종료하지 않습니다.

```bash
rostopic info /perception/lidar/planning_observations_map
rostopic info /ctrl_cmd
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /planning/objects
rostopic hz /planning/vehicle/state
rostopic echo -n 1 /morai_sensor_bridge/diagnostics
```

웹 화면은 아래 기본 명령 중 필요한 것을 별도 터미널에서 실행하고 주소를 직접 엽니다.
판제 웹은 모드에 따라 map/local을 자동 선택하며 수동 선택도 가능합니다. 음영 패널에 실제 선택 제어기,
정지 사유, bbox 판정, family·실제 선택 경로·local LiDAR 참조 객체를 표시합니다.
GPS 무효 때 마지막 map 위치로 올리지 않으며 debug OFF에서도 typed 기하를 볼 수 있습니다.
local 객체를 현재 음영 제어기가 회피에 사용한다는 의미는 아닙니다.
private 메시지 갱신 후에는 위 전체 빌드를 수행하고 adapter·판제·웹을 함께 재시작합니다.
[표시·시간 계약과 후속 계획](../architecture/shadow-planning-refactoring.md)을 참고하세요.

| 화면 | 기본 실행 | 주소 |
|---|---|---|
| 판제 웹 콘솔 | `roslaunch asmc_planner dashboard.launch` | `http://127.0.0.1:8088/web/index_v2.html` |
| LiDAR 웹 시각화 | `roslaunch asmc_lidar view.launch` | `http://127.0.0.1:8091/` |

LiDAR `view.launch`는 모니터만 시작합니다. 기본 `debug:=false`에서도 발행 객체 수·진단 상태를
확인할 수 있습니다. 웹의 OBB·점군 그림과 필터 전후·ROI 단계를 보려면 처리 노드의
`debug` 옵션을 켭니다. 판제 주행에는 이 옵션이 필요하지 않습니다.
기존 `lidar.launch`가 있다면 두 번째 처리 노드를 시작하지 않습니다.
판제는 C의 `planning_observations_map`을 사용합니다. 상세는 [웹 모니터](../../src/asmc_lidar/docs/web-monitor.md)를 봅니다.
`asmc_lidar`를 고르는 별도 source 옵션은 없습니다. 기본 연결은
`/perception/lidar/planning_observations_map` (`asmc_msgs/LidarPlanningObservation`)
→ `planning_objects_adapter` → `/planning/objects`입니다. `tracks_map`은 다른 타입의
분석용 출력이므로 판제 입력으로 대신 지정하지 않습니다. 수집용 planner도 같은 기본 입력을 상속합니다.

판제 웹은 `http://127.0.0.1:8088/web/index_v2.html`, LiDAR 웹은 `http://127.0.0.1:8091/`입니다.
기본 포트·ROS 노드 이름이 서로 다르고 두 웹 모두 관측만 합니다. 하나씩 사용하려면 해당 웹 launch
터미널만 Ctrl+C로 종료하고 다른 웹을 실행합니다. 브라우저 탭 종료는 서버 종료가 아닙니다.
동일한 웹 launch를 중복 실행하지 않습니다. `autonomy_no_bridges_v2.launch`에 음영 어댑터가
포함돼 있으므로 별도 차선변경 제어 launch도 추가하지 않습니다.
LiDAR `debug:=true`는 중간 점군과 필터 전후 표현을 위한 샘플 발행입니다. 기본은 OFF이며
추적·속도·HD맵/높이 필터·판제 발행은 유지됩니다. 웹 체크박스는 표시만 바꾸고 실제 발행 필터를 변경하지 않습니다.

### 선택 옵션·수동 모드 제어

아래는 기본 실행에서 바꿀 필요가 있을 때만 사용하는 인자입니다. 일반 주행은 위 기본 명령을
그대로 사용합니다. 전체 목록은 [launch 인자](launch-reference.md)를 봅니다.
`camera.launch`의 `models`는 **추론할 모델**, `view.launch`의 `models`는 **표시할 결과**를 선택합니다.
예를 들어 `models:=objects`는 객체만, `models:=objects,lane`은 객체와 차선을 선택하며
`view.launch`에서 선택을 바꿔도 추론·발행 모델은 바뀌지 않습니다.
실행 선택을 바꿀 때는 해당 launch를 종료한 뒤 대체하며 같은 생산자나 시각화 worker를 중복 시작하지 않습니다.

| 실행 | 옵션·기본값 | 용도 |
|---|---|---|
| `asmc_camera camera.launch` | `models:=all` | 기본 객체·차선·신호등 모두 추론. 선택값은 `objects`, `lane`, `traffic_light`의 쉼표 조합 |
| `asmc_camera camera.launch` | `mode:=active` | 기본 상시 추론·발행. `standby`는 로드·예열 후 대기, `off`는 모델 미실행 |
| `asmc_camera camera.launch` | `detector:=e8` | `pretrained`는 사전학습 COCO7 대조용. 가중치 변경 시 재시작 |
| `asmc_camera camera.launch` | `confidence:=0.40`, `nms_iou:=0.50` | 현재 검증된 객체 검출 기본값 |
| `asmc_camera view.launch` | `models:=all` | 기본 세 모델의 결과 표시. 일부 화면만 선택해도 추론은 유지 |
| `asmc_lidar lidar.launch` | `fusion:=true` | 카메라 분류 연결. `false`는 LiDAR 단독 진단용 |
| `asmc_lidar lidar.launch` | `debug:=false` | 웹 OBB·점군 그림과 필터 전후 비교가 필요할 때 `true`; 주행에는 불필요 |
| `asmc_lidar lidar.launch` | `rviz:=false` | 웹만 쓸 때 그대로 유지 |
| `asmc_lidar lidar.launch` | `debug_rate_hz:=5.0`, `debug_max_points:=20000` | 화면 샘플 상한. 실제 검출·판제 발행 주기나 처리 점 수를 바꾸지 않음 |
| `asmc_planner dashboard.launch` | `dashboard_port:=8088` | 판제 웹, 기본 state/객체 토픽은 정본 |
| `asmc_lidar view.launch` | `dashboard_port:=8091`, `open_browser:=false` | LiDAR 웹. 처리 노드와 독립적인 관측용 실행; 주소는 직접 열기 |
| `asmc_hd_map competition_map.launch` | `rviz:=false` | 선택 지도 publisher. `true`면 지도 RViz도 실행; 기존 지도 launch를 대체하고 중복 시작하지 않음 |

명시적으로 대기 실행이 필요하면 `camera.launch`의 `mode`를 `standby`로 지정합니다. 다음은 그 경우에만 필요한 객체/신호등 복구 예입니다. 모드 요청자는 하나만 둡니다.

```bash
rosservice call /perception/camera/objects/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/objects/status
# prepared=true 확인 후 echo 종료, ACTIVE
rosservice call /perception/camera/objects/set_mode '{mode: 2, target_hz: 0.0}'
rosservice call /perception/camera/traffic_light/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/traffic_light/status
# 같은 generation 준비 확인 후
rosservice call /perception/camera/traffic_light/set_mode '{mode: 2, target_hz: 0.0}'
```

Lane도 기본 상시 ACTIVE입니다. 수동으로 대기시켰다면 다음 PREPARE → prepared/pipeline_ready
확인 → ACTIVE 순서로 복구합니다. 현재 구간 진입/이탈로 자동 중단하지 않습니다.

```bash
rosservice call /perception/camera/lane/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/lane/status
# 준비 확인 후
rosservice call /perception/camera/lane/set_mode '{mode: 2, target_hz: 0.0}'
# 수동 중단이 필요한 경우에만 (기본 운용에서는 호출하지 않음)
rosservice call /perception/camera/lane/set_mode '{mode: 0, target_hz: 0.0}'
```

원본 좌표·빈/무효·세대/시간 규칙은 [카메라 계약](../contracts/camera-runtime-v1.md),
카메라 공용 관측과 기본3뷰 matching 계약은 [LiDAR 인계](../handoffs/camera-to-lidar.md),
판제 단독/통합 차이는 [판제 운영](planning-control.md)을 봅니다.

## 2. Container·build 명령

### ROS container

```bash
./scripts/docker_ros_up.sh <command>
```

| command | 기본값·의미 |
|---|---|
| `config [show]` | compose의 host network·port publish 여부 검사; `show`면 전체 출력 |
| `build` | RTX 40 기본 image build |
| `up` | RTX 40 기본 container 재생성·실행 |
| `rebuild` | cache 없이 build 후 재생성 |
| `build-rtx50` | 기본 image + RTX 50 파생 image build |
| `up-rtx50` | RTX 50 container 재생성·실행 |
| `rebuild-rtx50` | RTX 50 파생 image no-cache rebuild |
| `status` | `network_mode=host`, published port 없음 검사 |
| `check [port]` | Windows↔container loopback UDP. 기본 `39199` |
| `verify [seconds] [molitcomp03\|26r1]` | competition allowlist topic. 기본 `8 molitcomp03` |
| `verify-field [seconds] [molitcomp03]` | 주최판 Competition Status까지 필수. 기본 `8 molitcomp03` |
| `verify-gt [seconds] [molitcomp03\|26r1]` | local development oracle topic 검사 |
| `down` | ROS container 종료·제거 |

### Perception training container

```bash
./scripts/docker_perception_up.sh <config|build|up|down|rebuild>
```

`rebuild`는 no-cache build다. 이 컨테이너는 모델 학습과 학습 중 validation만 담당한다.
live 또는 bag replay ROS 추론·시각화는 모두 `asmc-ros-noetic`에서 실행한다. 학습
컨테이너는 host ROS master에 연결하지 않는다.

### ROS workspace build

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh
source devel/setup.bash
```

## 3. UDP bridge 옵션

권장 진입점:

```bash
./scripts/udp_bridge.sh [options] [bridge.launch override:=value ...]
```

| option | 기본 | 허용값·의미 |
|---|---:|---|
| `--transport` | `local` | `local`, `lan2lan`; Ctrl 목적지 IP를 결정 |
| `--gt` | OFF | 일반 입력에 Object·Semantic·Instance LiDAR 추가; local 전용 |
| `--require-competition-status` | false | 9099 Competition → `/Ego_topic`을 필수로 검사 (MolitComp03 확인) |
| `-h`, `--help` | — | 도움말 |

추가로 덮어쓸 수 있는 `bridge.launch` 인자:

| launch arg | 기본값 | 설명 |
|---|---:|---|
| `runtime_profile` | `morai_udp` | 두 MORAI release가 공유하는 고정 runtime; wrapper override 금지 |
| `transport_profile` | `local` | 직접 launch용 `local`, `lan2lan`; wrapper에서는 `--transport` 사용 |
| `stream_profile` | `competition` | 내부 `competition`, `development`; wrapper에서는 `--gt`로 GT만 추가 |
| `timestamp_mode` | `receive` | `receive` 또는 source timestamp 실험용 `source` |
| `publish_map_tf` | `false` | raw GPS/IMU `map→base_link` TF; 권위 localization과 동시 금지 |
| `lidar_frame_mode` | `azimuth` | VLP16 frame assembly 모드; 현행은 `azimuth` |
| `lidar_rotation_rate` | `15.0` | v3/GT 조립 참고값. 실제 송신 주기는 MORAI 센서셋 설정; v2 비교는10.0 |
| `cmd_ip` | transport에서 결정 | local `127.0.0.1`, LAN2LAN `192.168.0.1` |
| `cmd_port` | `9093` | MORAI Ego Ctrl Cmd listen port |
| `competition_port` | `9798` | 로컬 Detail 진단 호환 port; LAN에서는 비활성 |
| `competition_status_required` | `false` | true면9099 `/Ego_topic` 입력을 요구; Detail9798로 대체하지 않음 |
| `enable_vehicle_state` | `true` | GPS/IMU 기반 `/vehicle/state` producer |

`runtime_profile`, `transport_profile`, `stream_profile`은 wrapper가 관리한다.
`transport_profile:=...`를 직접 넘기지 말고 `--transport`를 사용한다.
`--streams`와 `ASMC_STREAMS`는 폐기했다. GT 개발은 `./scripts/udp_bridge.sh --gt`를 사용하고,
MORAI UI에서 `config/morai/sensors/261027_ASMC_Local_SensorGT_25.S4.MolitComp03.json`을 로드한다. Intensity·Instance
LiDAR 모두15Hz로 설정되어 있다. 주기 변경은 JSON 재로드 후 적용되며 실제 수신률은 별도로 확인한다.

직접 launch도 가능하지만 local 디버그 외에는 wrapper가 더 안전하다.

```bash
roslaunch asmc_bridge bridge.launch
```

LAN2LAN을 직접 launch할 때는 `transport_profile` 만 바꾸면 Ctrl IP가 자동으로
바뀌지 않는다. 반드시 두 값을 함께 주거나 wrapper를 사용한다.

```bash
roslaunch asmc_bridge bridge.launch \
  transport_profile:=lan2lan \
  cmd_ip:=192.168.0.1
```


## 4. 팀 시나리오 통합 실행

[scen_runner](../../tools/scen_runner/README.md)는 #86의 전체 씬·단일구간·충돌/정체 재현 경로입니다.
개인 `local.yaml`을 작성한 뒤 같은 컨테이너의 네 터미널에서 실행합니다.
앞 절의 수동 인지/판제 실행과 동시에 켜지 않습니다.

```bash
# 터미널1
./tools/scen_runner/run_bridge.sh
# 터미널2
./tools/scen_runner/run_perception.sh
# 터미널3: 요청 대기 후 판제/MPC 시작·재시작
./tools/scen_runner/run_planning.sh
# 터미널4: 객체/환경 배치·차량 주행을 포함한 개발 검증
./tools/scen_runner/run_scenario.sh --mode scene --seed 42
```

구간만 시험할 때는 터미널4를 다음 중 하나로 대체합니다.

```bash
./tools/scen_runner/run_scenario.sh --mode section --section tunnel
./tools/scen_runner/run_scenario.sh --replay-browser
# 키보드/외부 제어기 검증에는 run_scenario 대신 사용
./tools/scen_runner/run_validation.sh --mode section --section dynamic
```

`--section`은 static/dynamic/roundabout/high_speed/tunnel입니다.
난수·날씨·주행 횟수·정체 시간은 공유 YAML, PC 주소/DLL 경로는 Git 제외 local.yaml이 소유합니다.
시나리오 자체의 gRPC/DLL/신호 고정은 개발 검증용입니다. 본선 입력 경로가 아닙니다.
현재 시나리오 perception 구성도 카메라 전체 ACTIVE입니다. 판제가 lane 결과를 음영 구간에서
소비하는 정책과 모델 활성화는 구분하며, full-stack 부하/모드 gate는 별도 확인합니다.

## 5. 수집·bag·오프라인 진입점

수집은 명시적으로 실행할 때만 시작합니다. 현재 문서 작업은 이 명령을 실행하지 않습니다.
Capture는 학습 GT, raw bag은 실제 비동기 지연/재현 측정이며 서로를 대체하지 않습니다.

```bash
# 수동 Capture 요청만: 차량 제어를 시작하지 않음
roslaunch data_collection capture_mode_collector.launch capture_hz:=1.0 max_captures:=100
# 자동 Capture 주행: 기존 주행 제어를 종료하고 승인된 장부/설정으로만 실행
roslaunch data_collection capture_route_collection.launch controller:=mpc target_kph:=20
# raw bag
roslaunch data_collection bag_recorder.launch profile:=full_raw scenario:=manual_route controller:=wheel
# live bridge를 종료한 별도 재현 환경; paused 상태로 시작
roslaunch data_collection bag_replay.launch bag_path:=/data/rosbags/raw/<run_id> required_profile:=camera_raw
```

[수집 README](../../src/data_collection/README.md)의 센서셋/파일 QA와 [bag 재현](rosbag-evaluation.md)의
시계·consumer·result recorder 준비 순서를 따릅니다. raw gRPC 자동수집과 운영 판제를 동시에 실행하지 않습니다.
개별 component·전체 인자는 [launch 목록](launch-reference.md)에서 찾습니다.

## 6. LAN2LAN 연습 사전 준비

처음 연결은 [팀원용 안내](lan2lan-teammate.md), 채점 프로그램 경계는
[두 PC 준비](lan2lan-rehearsal.md)를 따른다.

| 역할 | 전용 NIC IPv4 | 현재 MolitComp03 설정 |
|---|---|---|
| Windows MORAI PC | 192.168.0.1/24 | CompetitionInfoPublisher Host9088 → .10:9099 |
| 실행 PC | 192.168.0.10/24 | Windows/Hyper-V 수신 → bridge → /Ego_topic |

전용 NIC는 gateway/DNS를 비우고 MAC·ARP로 상대를 확인한다. 인터넷 NIC의 IP를 대신
바꾸지 않는다. 설정 도구는 미리보기 후 `-Apply`로 적용하며 기존 ASMC 규칙도 갱신한다.
시뮬레이터에는 release별 Network JSON과 `261027_ASMC_LAN2LAN_Sensor_<시뮬레이터버전>.json`을 사용한다.
이미 LAN 센서셋이 적용돼 있으면 Competition 수정 때문에 다시 로드하지 않는다.

Windows/Hyper-V의 팀 수신 UDP는 다음과 같다. 상대 `.1`, 로컬 `.10`, 전용 NIC로 범위를 제한한다.

```text
9099, 9092, 9291, 9293, 9295, 9297, 9299, 9301, 9303
```

실행 PC의 WSL은 mirrored, Docker Desktop은 host networking 활성화 및 컨테이너 host mode를
확인한다. 설정을 바꾼 경우 실행 PC의 작업을 저장하고 반영하며, 시뮬레이터 PC는 재시작하지 않는다.
TCP용 portproxy로 UDP 전달을 해결하지 않는다. 별도 채점 프로그램의 endpoint는 별도로 확인한다.

```bash
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
```

별도 컨테이너 터미널에서 실제 메시지를 검사한다.

```bash
python3 scripts/verify_morai_topics.py --profile competition --transport lan2lan --require-competition-status --duration 30
./scripts/docker_ros_up.sh verify-field 30 molitcomp03
```

`--require-competition-status`, `competition_status_required`, `verify-field`는 실제9099의
`/Ego_topic`을 검사한다. 과거9798 검사 의미를 정정했다. 9798 Detail은 LAN에서 수신·발행하지
않고 속도 fallback으로 사용하지 않는다. 기존 `competition_port=9798`은 로컬 진단 호환 인자다.
local·`--gt` 동작은 유지하며, 이번 MolitComp03 허용 판단을26.R1에 일반화하지 않는다.
새 기록을 할 때만 [rosbag 안내](rosbag-evaluation.md)의 `competition_raw`와 저장소 사전 점검을 따른다.


## 7. 종료

각 foreground `roslaunch`는 해당 terminal에서 `Ctrl+C`로 종료한다. container를 내릴 때:

```bash
# 학습 container를 별도로 실행했을 때만:
./scripts/docker_perception_up.sh down
./scripts/docker_ros_up.sh down
xhost -local:docker
```

주행·녹화 중에 `up`, `down`, `rebuild`로 container를 재생성하지 않는다.


## LiDAR 병행 비교 실행

일반 주행은 앞의 기본 명령과 `/perception/lidar` namespace를 사용합니다.
구 이관용 `/migration/lidar`는 운영 경로·별칭이 아니며 더 이상 현행 실행 예시로 사용하지 않습니다.
과거 bag·실험 기록의 토픽 이름은 당시 근거로 보존합니다. `/test/lidar`는 아래 격리 시험의 예시 이름일 뿐,
새 파이프라인이나 자동 실행 노드를 추가하는 기능이 아닙니다.
운영 LiDAR와 별도 시험 처리를 **동시에 비교할 때만** 아래처럼 시험 namespace를 명시합니다.
여기서는 출력 격리를 위한 인자가 필수입니다. 센서·카메라 입력은 기존 정본을 공유하며
판제는 운영 namespace를 계속 소비합니다.

```bash
roslaunch asmc_lidar lidar.launch namespace:=/test/lidar
```

이 시험 결과를 웹으로 볼 때는 기존 웹 launch를 종료한 뒤 같은 namespace로 실행합니다.
포트가 같은 웹 서버를 중복 실행하지 않습니다.

```bash
roslaunch asmc_lidar view.launch namespace:=/test/lidar
```

선택 옵션은 앞의 표와 동일합니다. `fusion:=false`는 카메라 연결 없는 비교,
`debug:=true`는 중간 점군·필터 전후 표시, `rviz:=true`는 해당 namespace의 처리된
OBB·ID·분류와 ego box 표시입니다. 원본 `/velodyne_points`와 처리 결과를 구분합니다.
리셋이 필요한 시험에서만 다음 서비스를 호출합니다. 대기·추적·pose 이력이 초기화되고
processing generation이 증가합니다.

```bash
rosservice call /test/lidar/lidar_pipeline/reset "{}"
```

보정·시간·터널 비교 등의 내부 옵션과 관측·진단 규칙은
[asmc_lidar README](../../src/asmc_lidar/README.md),
[웹 모니터](../../src/asmc_lidar/docs/web-monitor.md)를 따릅니다.
