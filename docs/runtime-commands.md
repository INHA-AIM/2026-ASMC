# ASMC 실행 명령어 모음

> **문서 역할:** Active Operator Reference — 현재 runtime 실행·시각화·녹화 명령과 공개 옵션
>
> **범위:** 본선·local 공통 실행 진입점. 학습·offline 분석·HD Map 생성용 내부
> launch는 각 package README를 따른다.

명령은 host에서 현재 저장소 root를 ASMC로 잡고, container에서는 `/root/ws`인
기준이다. ROS launch 인자는 `name:=value` 형태로 덮어쓴다.

## 1. 가장 빠른 local 실행

### Host — container 준비

```bash
# 먼저 host에서 이 저장소 안의 아무 디렉터리로 이동
export ASMC=$(git rev-parse --show-toplevel)
cd "$ASMC"
xhost +local:docker

./scripts/docker_ros_up.sh up
```

### Terminal A — UDP bridge는 단독 foreground로 유지

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./scripts/udp_bridge.sh
```

bridge는 perception·planning 통합 명령에 포함하지 않는다. 항상 별도 terminal의 log와
diagnostics를 보며 입력 Hz를 검사한다.

### LiDAR 이전용 골격과 현재 실행 경로

`asmc_lidar`는 좌표·시간 라이브러리와 계약 시험만 제공한다. 손재호가 기존
`lidar+sensorfusion`의 검출·추적·fusion을 이전할 예정이며 현재 객체 publisher는 없다.
기존 실행은 [원본 sensor-fusion guide](../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md),
새 골격의 빌드·시험은 [asmc_lidar README](../src/asmc_lidar/README.md),
이전·삭제 조건은 [LiDAR 인계](./packages/asmc_lidar/README.md)를 따른다.
이전 개인 연구용 비교·후보 launch는 삭제됐다. 옛 devel에 실행파일이 남아 있어도
현재 소스에서 지원하는 기능으로 사용하지 않는다.

### 입력 Hz·브릿지 진단 (별도 터미널)

```bash
rostopic hz /Ego_topic
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /gps
rostopic hz /imu
rostopic hz /vehicle/state
rostopic echo -n 1 /morai_udp_bridge/diagnostics
rostopic echo -n 1 /morai_sensor_bridge/diagnostics
```

### Terminal B — 최종 HD Map

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_hd_map competition_map.launch
```

RViz까지 열려면:

```bash
roslaunch asmc_hd_map competition_map.launch rviz:=true
```

### Terminal C — camera

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_camera camera.launch models:=objects,lane
```

기본은 STANDBY다. 객체 PREPARE/ACTIVE와 음영 lane 준비는 [카메라 계약](./contracts/perception/camera-runtime-v1.md)을 따른다.
신호등을 카메라에서 소유하려면 `models:=all`을 선택하고 판제의 신호등 생산자는 끈다.
현재 LiDAR는 별도 nested Main이며 [카메라→LiDAR 인계](./packages/asmc_lidar/camera-handoff.md)의 빌드/source 순서를 따른다.

### Terminal D — Planning/Control

bridge를 이미 실행했으므로 `no_bridges` launch를 사용한다. 신호등 producer를
perception에서 이미 켰다면 중복 실행을 막기 위해 `traffic_light:=false`로 둔다.

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch behavior_planner autonomy_no_bridges_v2.launch traffic_light:=false
```

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
./scripts/docker_perception_up.sh <build|up|down|rebuild>
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
| `--streams` | `competition` | `competition`, `development`; development는 local 전용 |
| `--require-competition-status` | false | 주최 현장판 type 787을 필수 gate로 전환 |
| `-h`, `--help` | — | 도움말 |

추가로 덮어쓸 수 있는 `bridge.launch` 인자:

| launch arg | 기본값 | 설명 |
|---|---:|---|
| `runtime_profile` | `morai_udp` | 두 MORAI release가 공유하는 고정 runtime; wrapper override 금지 |
| `transport_profile` | `local` | 직접 launch용 `local`, `lan2lan`; wrapper에서는 `--transport` 사용 |
| `stream_profile` | `competition` | 직접 launch용 `competition`, `development`; wrapper에서는 `--streams` 사용 |
| `timestamp_mode` | `receive` | `receive` 또는 source timestamp 실험용 `source` |
| `publish_map_tf` | `false` | raw GPS/IMU `map→base_link` TF; 권위 localization과 동시 금지 |
| `lidar_frame_mode` | `azimuth` | VLP16 frame assembly 모드; 현행은 `azimuth` |
| `cmd_ip` | transport에서 결정 | local `127.0.0.1`, LAN2LAN `192.168.0.1` |
| `cmd_port` | `9093` | MORAI Ego Ctrl Cmd listen port |
| `competition_port` | `9798` | Competition Vehicle Status 수신 port |
| `competition_status_required` | `false` | true면 type 787 미수신을 field gate 실패로 처리 |
| `enable_vehicle_state` | `true` | GPS/IMU 기반 `/vehicle/state` producer |

`runtime_profile`, `transport_profile`, `stream_profile`은 wrapper가 관리한다.
`transport_profile:=...`를 직접 넘기지 말고 `--transport`를 사용한다.

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

## 4. HD Map 옵션

```bash
roslaunch asmc_hd_map competition_map.launch [arg:=value ...]
```

| arg | 기본값 | 설명 |
|---|---:|---|
| `rviz` | `false` | 최종 map RViz preset 동시 실행 |
| `data_root` | `/data` | 외부 data root |
| `debug_republish_hz` | `1.0` | latched/static debug marker 재발행 Hz |
| `baseline_path` | empty | 검증 baseline override |
| `publish_local` | `false` | 선택적 local lane/grid 크롭 발행. 필요할 때만 `true`; 기본 RViz는 전역 외곽선 marker 사용 |
| `local_pose_source` | `gps_imu` | `gps_imu`, `ego_status`, `pose_stamped` |
| `local_ego_topic` | `/Ego_topic` | `ego_status` 입력 |
| `local_pose_topic` | `/localization/pose` | `pose_stamped` 입력 |
| `local_gps_topic` | `/gps` | GPS 입력 |
| `local_imu_topic` | `/imu` | IMU 입력 |
| `local_extent_m` | `120.0` | local 정사각 crop 한 변(m); 현행 호환값 |
| `local_publish_hz` | `5.0` | local topic 최대 발행 Hz |

운영 topic은 `/hd_map/global/*`, `/hd_map/local/*`, `/hd_map/meta`다. debug marker를
planning data로 사용하지 않는다.

## 5. Camera 실행·LiDAR 연결

일반 실행은 `camera.launch`, 이미 실행 중인 생산자의 시각화는 `view.launch`다.
기존 `perception.launch`의 상세 옵션은 호환/과거 실험용이며 일반 실행에 반복 지정하지 않는다.
현재 인수 계약은 [카메라 runtime](./contracts/perception/camera-runtime-v1.md), Main 연결은 [LiDAR 인계](./packages/asmc_lidar/camera-handoff.md)가 소유한다.

| 공개 옵션 | 기본값 | 의미 |
|---|---|---|
| mode | standby | off/standby/active |
| models | all | objects,lane,traffic_light 중 쉼표로 조합 |
| detector | e8 | e8/pretrained, 변경 시 노드 재시작 |
| confidence | 0.40 | 객체 생산자 cutoff |
| nms_iou | 0.50 | 원본 클래스별 NMS |

```bash
roslaunch asmc_camera camera.launch
# 객체만 즉시 확인할 때
roslaunch asmc_camera camera.launch mode:=active models:=objects
# 기존 실행 종료 후 사전학습 대조
roslaunch asmc_camera camera.launch mode:=active models:=objects detector:=pretrained
```

E8/사전학습 YOLO와 TwinLite는 TensorRT다. 전방544×960 B1·좌우480×640 B2,
목표 YOLO/TwinLite/ResNet=15/15/10Hz이며 실제 최소 공급과 목표 달성은 따로 검사한다.
Mask·보정·모델 경로·세부 입력은 패키지 YAML로 관리한다. Engine 환경 불일치에 자동 fallback하지 않는다.
모델별 독립 프로세스이며 같은 입력 크기/Hz가 같은 stamp나 tensor 공유를 뜻하지 않는다.

모드 요청은 별도 터미널에서 한다. 같은 generation의 prepared=true를 확인한 뒤 ACTIVE를 요청한다.

```bash
rosservice call /perception/camera/objects/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/objects/status
# prepared=true 확인 후 Ctrl-C
rosservice call /perception/camera/objects/set_mode '{mode: 2, target_hz: 0.0}'
```

Lane은 일반 구간 STANDBY → 음영 진입 전 PREPARE → prepared/pipeline_ready 확인 → ACTIVE → 이탈 후 STANDBY다.
서비스 루트는 `/perception/camera/lane/set_mode`이며 구간 판단은 판제 책임이다. Family 정본 결과를 소비하고 영상 BEV는 수요가 있을 때만 만든다.

```bash
roslaunch asmc_camera view.launch models:=objects,lane
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

한 rqt에서 family 화면 `/perception/lane/debug/shared_family_bev/compressed`로 선택을 바꿀 수 있다.
시각화는 추론 생산자를 시작하지 않으며 기존 rqt를 중복 생성하지 않는다.

Main은 별도 실행하고 **독립 YOLO를 시작하지 않는다**. `/perception/camera/objects`의 전방만
`/YOLO/BBox`로 변환한다. `start_yolo` 및 구형 장치/GPU 옵션은 제거됐다.
현재 판제 입력과 전방 unique·±50ms·200ms 대기는 유지한다. 3뷰 fusion/전체 LiDAR 이전은 손재호의 후속 작업이다.
빌드·source 순서, 원본 이름/그룹·confidence 손실 경계와 잔류 실행파일 정리는 위 LiDAR 인계를 따른다.

[Family 최신 검증](./handoffs/perception/camera-family-supply-optimization.md)의 순간 공백/freshness 제한은 유지한다.
과거 P4/P5·YOLO11m 수락 도구와 기록은 당시 입력/프로필의 증거이며 현재 설정 전체 수락으로 재사용하지 않는다.

## 6. Planning/Control 옵션

```bash
roslaunch behavior_planner autonomy_no_bridges_v2.launch [arg:=value ...]
```

| arg | 기본값 | 설명 |
|---|---:|---|
| `state_source` | `gps_imu` | `gps_imu`, local 개발용 `ego_dev` |
| `dashboard` | `false` | behavior dashboard |
| `traffic_light` | `false` | 독립 호환 시험에서만 판제 launch가 신호등 producer를 시작할지 |
| `traffic_light_device` | `auto` | 신호등 inference device |
| `camera_topic` | `/cam_extra/image_jpeg/compressed` | 신호등 input |
| `traffic_light_checkpoint` | `asmc_camera/models/traffic_light/best.pt` | checkpoint |

신호등 producer의 운영 소유자는 `asmc_camera`이고 planning launch 기본값은 `false`다.
`traffic_light:=true`는 perception launch를 켜지 않은 독립 호환 시험에서만 사용한다. bridge와
`/vehicle/state` producer도 중복으로 켜지 않는다.

## 7. Bag 녹화·재생

### 녹화

```bash
roslaunch data_collection bag_recorder.launch [arg:=value ...]
```

| arg | 기본값 | 설명 |
|---|---:|---|
| `profile` | `full_raw` | `config/bag_profiles/*.yaml`의 profile 이름 |
| `scenario`, `controller`, `run_id` | empty | run metadata; `run_id` empty이면 자동 생성 |
| `rosbag_root` | empty | empty이면 `$ASMC_DATA/rosbags` 정책 사용 |
| `map_slug` | `kcity2025` | map ID |
| `weather`, `sim_hour` | empty | 지정하면 simulator 환경 metadata override |
| `grpc_address`, `grpc_port` | `127.0.0.1`, `7789` | 개발용 환경 metadata gRPC |
| `grpc_timeout_sec` | `1.0` | 개별 RPC timeout |
| `grpc_connect_timeout_sec` | `2.0` | gRPC 연결 timeout |
| `environment_poll_sec` | `2.0` | 환경 metadata polling |
| `topic_timeout_sec` | `10.0` | 필수 topic 대기 timeout |
| `parent_raw_run_id` | empty | result bag의 부모 raw run |
| `model_id`, `checkpoint` | empty | result profile에서 필수 |
| `min_free_gb` | `20.0` | 최소 남은 disk |
| `split_gb` | `4.0` | bag split 크기 |
| `buffer_mb` | `1024` | rosbag buffer |

예:

```bash
roslaunch data_collection bag_recorder.launch \
  profile:=full_raw scenario:=manual_route controller:=wheel
```

### 재생

```bash
roslaunch data_collection bag_replay.launch \
  bag_path:=/data/rosbags/<run>/<file>.bag [arg:=value ...]
```

| arg | 기본값 | 설명 |
|---|---:|---|
| `bag_path` | 필수 | replay bag |
| `required_profile` | `camera_raw` | bag manifest에서 요구할 profile |
| `rate` | `1.0` | replay 배속 |
| `allow_privileged_gt` | `false` | oracle bag을 명시적으로 허용 |
| `wait_for_start` | `true` | consumer 준비 전 paused wait |
| `post_roll_sec` | `2.0` | 마지막 message 후 대기 |

## 8. LAN2LAN 연습 사전 준비

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

TrafficLight oracle `7502`, Object oracle `7505`, GT `9305/9307/9309/9311`은 본선
allowlist에 추가하지 않는다.

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

주최 현장판에서 type 787을 수락할 때만:

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

## 9. 종료

각 foreground `roslaunch`는 해당 terminal에서 `Ctrl+C`로 종료한다. container를 내릴 때:

```bash
# 학습 container를 별도로 실행했을 때만:
./scripts/docker_perception_up.sh down
./scripts/docker_ros_up.sh down
xhost -local:docker
```

주행·녹화 중에 `up`, `down`, `rebuild`로 container를 재생성하지 않는다.
