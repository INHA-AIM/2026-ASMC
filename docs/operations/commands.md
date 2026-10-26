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

새 LiDAR 연결은 컨테이너의 outer workspace만 빌드합니다. 주행 중 container 재생성이나 빌드를 하지 않습니다. 구 Main 실행/복구에만 별도 nested 빌드가 필요합니다. 절차는 아래 구 Main 안내 링크를 따릅니다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2
source devel/setup.bash
```

이후 새 경로의 각 터미널에서는 outer 환경을 적용합니다. 구 Main 실행/복구만 바깥→nested 순서를 지킵니다([구 Main 안내](../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md)).

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
```

### 터미널별 실행

각 행은 별도 터미널에서 foreground로 실행합니다. 이 구성과 아래 시나리오 통합 구성을 동시에 켜지 않습니다.

| 순서 | 실행 | 포함·주의 |
|---|---|---|
| A | `./scripts/udp_bridge.sh` | 허용 센서·기본 `/vehicle/state`, UDP 포트당 수신자 하나 |
| B | `roslaunch asmc_hd_map competition_map.launch` | 정적 지도·metadata; 현재 Planner local map copy를 자동 대체하지 않음 |
| C | `roslaunch asmc_camera camera.launch` | 세 모델 상시 ACTIVE·발행, E8 기본; GUI 별도 |
| D | `roslaunch asmc_lidar lidar.launch` | 공용 local/map 추적·속도/운동·전달 필터; 카메라 모델 미실행 |
| E | `roslaunch asmc_planner autonomy_no_bridges_v2.launch lidar_source_policy:=asmc_lidar` | **차량 제어 포함**, 새 공용 관측 직접 소비; GPS/IMU 위치와 Competition 종방향 실측 속도 조합, MPC/selector/재밍 어댑터 |

위 D/E는 새 LiDAR 경로의 조합입니다. 통합 판제 인자를 생략한 기본은 아직 구 `validated_lidar`입니다. 구 Main 실행·복구와 새 조합을 섞지 않습니다. 실제 미션 인수/구 패키지 폐기는 후속이며 [실행 기준선](../../src/asmc_lidar/docs/planning-runtime.md)을 확인합니다.

신호등의 단일 추론·발행자는 카메라 담당의 `asmc_camera`입니다. 판제는 결과만 소비하고
`traffic_light:=false` 인자는 제거되어 지정하지 않습니다. 중단/재개·주기 제어는 카메라의
`/perception/camera/traffic_light/set_mode`를 사용합니다. 판제 신호 규칙은 유지합니다. bridge `/vehicle/state`와
판제 `/planning/vehicle/state`는 다른 토픽·추정기이며 같은 토픽의 중복 producer는 금지합니다. 기본 E는 `/Ego_topic`의 허용된 종방향 속도만 `/planning/vehicle/gps_imu_state`의 GPS/IMU 위치와 결합합니다. 두 입력의 시각이 맞지 않거나 끊기면 상태를 무효로 처리합니다.

기본 카메라는 세 모델 모두 추론·발행합니다. 준비가 끝나기 전에 판제가 결과를 사용하지 않도록
상태와 새 결과의 validity/freshness를 확인합니다. [구간별 소비 안내](../handoffs/perception-to-planning.md)는
판제 사용 범위이며 생산자를 끄는 설정이 아닙니다. Lane 주행 소비는 음영 ODD에 한정하고 A126 진입 전에
유효 family를 확인합니다. A448은 접근 구간, 주변 차량 소비는 A153 끝/A451 진입까지입니다.

현재 차선 기본 모델은 Large v7 E8 TensorRT다. 사용자가 알려진 공급·freshness 제한을 확인한 뒤 전환을 승인했다.
[선정·실측·Medium 복구](../records/camera/2026-10-18-twinlite-large-deployment.md)를 확인한다.
공식 BDD100K Large의 추가학습 전 결과는 [별도 실험 실행 안내](../../src/asmc_camera/docs/bdd-pretrained-experiment.md)를
따릅니다. 이진 차선 시각화 실험이며 위 C의 운영 family를 대체하지 않습니다.

명시적으로 대기 실행이 필요하면 C 대신 `roslaunch asmc_camera camera.launch mode:=standby`를
사용합니다. 다음은 그 경우에만 필요한 객체/신호등 복구 예입니다. 모드 요청자는 하나만 둡니다.

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

### 개별 디버그·시각화

일반 사용은 위 C의 `camera.launch`와 아래 `view.launch`를 각각 한 번 실행합니다.
두 launch 모두 `models:=all`이 기본이며, `camera.launch`는 `mode:=active`도 기본이므로
이 인자를 반복 지정할 필요가 없습니다. 추론과 시각화 선택은 서로 독립적입니다.

```bash
# C에서 이미 camera.launch를 실행했다면 추론을 또 시작하지 않음
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
고정 평면 복구가 필요하면 기존 `camera.launch`를 종료한 뒤
`roslaunch asmc_camera _imu_compare.launch enabled:=false`로 대체합니다. `view.launch`는 그대로 사용합니다.
[0.36m·표시 검증](../records/camera/2026-10-18-nominal-ground-and-arrival-view.md)을 함께 확인합니다.
`view.launch`는 시각화 worker만 시작하며, 추론 모델이나 rqt 창을 시작하지 않습니다.

일부 모델만 점검할 때는 기존 `camera.launch`를 종료한 뒤 다음 중 하나로 대체합니다.
아래 두 줄은 서로 다른 추론 실행 예시입니다.

```bash
# YOLO만 추론: lane·신호등 모델은 시작하지 않음
roslaunch asmc_camera camera.launch models:=objects
# 위 추론 실행을 종료한 뒤 사전학습7종만 대조
roslaunch asmc_camera camera.launch models:=objects detector:=pretrained
```

추론은 기본 전체 실행을 유지하고 화면만 객체+차선으로 제한하려면, 기존 `view.launch`만
종료한 뒤 다음으로 대체합니다. 신호등 추론·발행은 계속 동작합니다.

```bash
roslaunch asmc_camera view.launch models:=objects,lane
```

`visualization.launch`는 `view.launch`와 구형 실행 경로가 공유하는 내부 시각화 구성입니다.
단독 기본 실행은 세 표시 플래그가 모두 false라 시각화 노드를 만들지 않습니다. 일반 사용자는
`view.launch`를 사용하며 둘을 동시에 실행하지 않습니다. 상세 legacy/raw 옵션은 재현·진단용입니다.

GUI 종료 후 사용한 process가 남지 않았는지
`rosnode list`, `pgrep -af rqt_image_view`로 확인하고 해당 터미널의 Ctrl+C로 정리합니다.
사용자/다른 작업의 프로세스를 일괄 종료하지 않습니다.

```bash
roslaunch asmc_hd_map competition_map.launch rviz:=true
roslaunch asmc_planner dashboard.launch state_topic:=/planning/vehicle/state
rostopic info /YOLO/BBox
rostopic info /ctrl_cmd
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /planning/objects
rostopic hz /planning/vehicle/state
rostopic echo -n 1 /morai_sensor_bridge/diagnostics
```

HD Map RViz 명령은 기존 지도 launch를 대체합니다. GUI만 추가한다고 같은 publisher를 하나 더 띄우지 않습니다.

이관 검증 LiDAR(`asmc_lidar`)의 필터 단계·OBB·추적 결과를 브라우저에서 보려면 별도 터미널에서
`roslaunch asmc_lidar view.launch open_browser:=true`를 실행합니다(처리는 별도 `lidar.launch debug:=true`)(`http://127.0.0.1:8091/`, 읽기 전용).
이 launch는 모니터만 켭니다. 별도 `lidar.launch`와 기존 Main을 함께 실행하면 LiDAR 처리가 두 번 수행됩니다.
판제는 계속 D의 Main을 사용합니다. 상세는 [웹 모니터](../../src/asmc_lidar/docs/web-monitor.md)를 봅니다.
원본 좌표·빈/무효·세대/시간 규칙은 [카메라 계약](../contracts/camera-runtime-v1.md),
전방 어댑터와 nested source 순서는 [LiDAR 인계](../handoffs/camera-to-lidar.md),
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
| `--require-competition-status` | false | 별도9798 경로를 필수화. 현재9099 Competition 설정에는 사용하지 않음 |
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
| `competition_port` | `9798` | 별도 상태 수신 port; 현재 UI Competition은 ego_port9099 경로 |
| `competition_status_required` | `false` | true면 별도 competition 경로 미수신/미지원 형식을 실패로 처리 |
| `enable_vehicle_state` | `true` | GPS/IMU 기반 `/vehicle/state` producer |

`runtime_profile`, `transport_profile`, `stream_profile`은 wrapper가 관리한다.
`transport_profile:=...`를 직접 넘기지 말고 `--transport`를 사용한다.
`--streams`와 `ASMC_STREAMS`는 폐기했다. GT 개발은 `./scripts/udp_bridge.sh --gt`를 사용하고,
MORAI UI에서 `config/morai/sensors/ASMC_sensor_set_GT.json`을 로드한다. Intensity·Instance
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
현재 시나리오 perception 구성은 카메라 전체 ACTIVE입니다. 음영에서만 lane을 활성화하는
판제 운영 정책과 구분하고 full-stack 부하/모드 gate는 별도 확인합니다.

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

역할별 Windows·장치관리자·WSL2·Wireshark 확인과 주최 채점 채널의 차이는
[LAN2LAN 상세](lan2lan-rehearsal.md)를 먼저 확인한다. 아래 팀 포트 목록을 채점 프로그램
전체의 방화벽 목록으로 사용하지 않으며 현장 주최 network JSON을 팀 생성본으로 덮지 않는다.
사용자 확인상 채점·녹화도 추론 PC에서 실행한다. 필요한 채점 channel은 그 PC까지 전달하고,
Windows 프로그램 수신과 WSL/Docker 알고리즘 입력을 구분한다.

연습과 현장은 포트를 바꾸지 않고 IP만 local에서 LAN2LAN으로 바꾼다.

| 장치 | 전용 Ethernet NIC IPv4 | prefix/netmask |
|---|---:|---:|
| MORAI simulator PC | `192.168.0.1` | `/24` (`255.255.255.0`) |
| Autonav code PC | `192.168.0.10` | `/24` (`255.255.255.0`) |

가정에서는 전용 케이블/스위치의 두 NIC에만 고정 IP를 설정하고 gateway·DNS는 비운다.
현장에서는 **주최측이 제공한 NIC·netmask·gateway 값을 우선**하며, 이 문서를
근거로 정상 설정된 현장 PC를 초기화하지 않는다.

### Simulator PC

1. MORAI 전용 Ethernet NIC를 `192.168.0.1/24`로 고정한다.
2. Wi-Fi, VPN, Hyper-V adapter에 동일 `192.168.0.0/24` route가 없는지 확인한다.
3. simulator release에 맞는 network JSON 하나와 v3 sensor JSON을 import한다.

```text
config/morai/network/generated/ASMC_network_set_26r1_lan2lan.json
또는
config/morai/network/generated/ASMC_network_set_molitcomp03_lan2lan.json

config/morai/sensors/generated/ASMC_sensor_set_v3_lan2lan.json
```

4. Windows Firewall에서 Autonav `192.168.0.10`에서 MORAI로 들어오는 Ctrl UDP
   `9093`만 필요한 프로필에 허용한다. 전체 firewall을 끄지 않는다.
5. MORAI를 열기 전·후 `route print`, `Get-NetIPConfiguration`으로 route가 변하지
   않았는지 남긴다.

### Autonav code PC — Windows + WSL + Docker Desktop

1. Ethernet NIC를 `192.168.0.10/24`로 고정하고 동일 subnet route 충돌을 없앤다.
2. `%UserProfile%\.wslconfig`에 다음을 적용한다.

```ini
[wsl2]
networkingMode=mirrored
firewall=true
```

3. PowerShell에서 `wsl --shutdown` 후 Docker Desktop을 재시작한다.
4. Docker Desktop 4.34+ Linux container, WSL Integration, **Enable host networking**을
   켜고 compose의 `network_mode: host`를 유지한다. Docker `ports:`로 UDP를 다시
   매핑하지 않는다.
5. Windows Firewall과 Hyper-V firewall에서 source `192.168.0.1`의 다음 inbound UDP만
   허용한다.

```text
9798, 9092, 9099,
9291, 9293, 9295, 9297, 9299, 9301, 9303
```

TrafficLight oracle `7502`, Object oracle `7505`, GT `9305/9307/9309/9311`은 팀 bridge/알고리즘
입력에 추가하지 않는다. 별도 주최 채점 프로그램의 필요한 수신 경로까지 차단하라는 뜻이
아니며 [채점 endpoint 확인](lan2lan-rehearsal.md#채점-전용-채널-아직-닫지-않은-경계)을 따른다.

6. WSL과 container에서 실제 NIC·route·bind를 확인한다.

```bash
ip -br addr
ip route get 192.168.0.1
./scripts/docker_ros_up.sh status
ss -lunp
```

7. bridge를 명시적 LAN2LAN으로 실행한다.

```bash
./scripts/udp_bridge.sh --transport lan2lan
```

별도9798에 허용 상태가 송신되는 것을 확인한 환경에서만 아래 검사를 쓴다.
현재 CompetitionInfoPublisher9099→`/Ego_topic` 설정에는 적용하지 않는다
([2026-10-23 확인](../records/bridge/2026-10-23-competition-channel-audit.md)).

```bash
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
./scripts/docker_ros_up.sh verify-field 30 molitcomp03
```

### Native Linux code PC

Windows/WSL 단계만 제외하고 `192.168.0.10/24`, firewall allowlist,
`network_mode: host`, route·bind 검사를 동일하게 적용한다. Docker bridge network나
port translation을 새로 넣지 않는다.

### 연습·현장 공통 gate

1. 양방향 IP reachability와 `192.168.0.0/24` route를 확인한다.
2. code PC에서 sensor destination port의 packet count·size·rate를 확인한다.
3. bridge diagnostics의 parse error, staleness, LiDAR raw packet/frame 조립률을 확인한다.
4. Ctrl mode 2의 `192.168.0.1:9093` 전송과 실제 brake/steer 응답을 확인한다.
5. bridge·container·MORAI를 하나씩 재시작해 복구를 검사한다.
6. 15분 이상 CPU/GPU/RAM, packet drop과 topic Hz를 기록한다.

generated JSON, 포트, source IP를 현장에서 즉석 편집하지 않는다. 연습을 끝낸 뒤
local로 돌아갈 때는 MORAI에 `*_local.json`을 다시 import하고 bridge를 인자 없이
실행한다. LAN2LAN 옵션을 환경변수로 숨기거나 수신 sender IP로 자동 판별하지
않는다.

## 7. 종료

각 foreground `roslaunch`는 해당 terminal에서 `Ctrl+C`로 종료한다. container를 내릴 때:

```bash
# 학습 container를 별도로 실행했을 때만:
./scripts/docker_perception_up.sh down
./scripts/docker_ros_up.sh down
xhost -local:docker
```

주행·녹화 중에 `up`, `down`, `rebuild`로 container를 재생성하지 않는다.


## LiDAR 이관 검증 실행

기존 Main은 운영 판제 입력을 계속 제공한다. 아래 명령은 드라이버·카메라 추론을 시작하지 않고
기존 센서/카메라 정본 입력을 구독해 `/migration/lidar/*`에만 발행한다.

```bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar lidar.launch
# 카메라 없이 비교할 때 위 실행 대신 사용
roslaunch asmc_lidar lidar.launch fusion:=false
# 명시적 초기화: 대기·추적·pose 이력을 지우고 처리 generation 증가
rosservice call /migration/lidar/lidar_pipeline/reset "{}"
```

일반 실행 인자는 `fusion`(기본 true), `namespace`(기본 `/migration/lidar`), `pipeline_config`다.
`fusion:=false`는 카메라 보정 조회·구독을 끄며 카메라 패키지가 없는 환경에서도 동작한다.
fusion은 카메라 추론을 시작하는 옵션이 아니며 현재 연결은 전방 분류에 한정된다.
기존 입력/출력 토픽·`lidar_level_roll`·`camera_calibration_config`·`experimental_tunnel_filter`·
`publish_tf`(기본 false)·`tf_child_frame`은 내부 대조·배선 조정용으로 유지한다.
빈 roll/tunnel override는 YAML 값을 사용하고 fusion은 launch 선택을 우선한다.
`/perception/lidar/*` 운영 전환은 기본 차단하며 L7에서 별도 확인한다.
처리 시작 때 RViz도 함께 켜려면 `roslaunch asmc_lidar lidar.launch rviz:=true`를 사용한다.
기본은 OFF다. 지정한 namespace의 객체/ego Marker에 자동 연결하며 웹 화면은 켜지 않는다.
이미 열린 RViz를 유지하려면 이 옵션을 켜지 않고 아래 수동 표시 도구를 사용한다.

PR #105의 localhost 웹 화면은 `view.launch`로 별도 실행하며 처리 노드를 중복 시작하지 않는다.
단계별 점군을 보려면 기존 처리 실행을 `roslaunch asmc_lidar lidar.launch debug:=true`로 준비한 뒤
별도 터미널에서 `roslaunch asmc_lidar view.launch`를 실행하고 `http://127.0.0.1:8091/`을 연다.
새 관측·배열·진단과 복구는 [asmc_lidar README](../../src/asmc_lidar/README.md)를 따른다.

RViz에서 **처리된 OBB·ID·분류**를 보려면 별도 터미널에서 아래 변환기를 실행한다.
기존 RViz의 Fixed Frame=`base_link`, MarkerArray=`/migration/lidar/markers`로 설정하거나
`File → Open Config`에서 `/root/ws/src/asmc_lidar/rviz/lidar.rviz`를 연다.

```bash
rosrun asmc_lidar lidar_markers.py
```

이 도구는 `/migration/lidar/observations_local`만 읽고 Marker만 발행한다. 카메라 추론·검출·추적을
재실행하지 않는다. `/velodyne_points`는 원본 점군이며 처리 결과와 구분한다. 중간 지면 제거/군집 점군은
현재 공개되지 않는다. 이미 RViz가 열려 있으면 새 GUI를 중복 실행하지 않는다.
