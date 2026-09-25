# ASMC 실행 명령어 모음

> **문서 역할:** Active Operator Reference — 현재 runtime 실행·시각화·녹화 명령과 공개 옵션
>
> **범위:** 본선·local 공통 실행 진입점. 학습·offline 분석·HD Map 생성용 내부
> launch는 각 package README를 따른다.

명령은 저장소 root가 host에서
`/home/aim06/projects/2026-ai-sw-mobility-competition/2026-ASMC`, container에서
`/root/ws`인 기준이다. ROS launch 인자는 `name:=value` 형태로 덮어쓴다.

## 1. 가장 빠른 local 실행

### Host — container 준비

```bash
cd /home/aim06/projects/2026-ai-sw-mobility-competition/2026-ASMC
xhost +local:docker

./scripts/docker_ros_up.sh up
./scripts/docker_perception_up.sh up
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

### Terminal C — camera·BEV·lane perception

```bash
docker exec -it asmc-perception-train bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_perception perception.launch
```

신호등과 debug image publisher까지 필요하면:

```bash
roslaunch asmc_perception perception.launch \
  start_traffic_light:=true \
  start_visualization:=true \
  lane_enable_shared_family_debug:=true
```

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

### Perception GPU container

```bash
./scripts/docker_perception_up.sh <build|up|down|rebuild>
```

`rebuild`는 no-cache build다. 소스만 바뀐 경우 container 안의 catkin build만 다시
실행하며 image를 불필요하게 rebuild하지 않는다.

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
| `publish_local` | `true` | 위치 기반 local lane/grid 발행 |
| `local_pose_source` | `gps_imu` | `gps_imu`, `ego_status`, `pose_stamped` |
| `local_ego_topic` | `/Ego_topic` | `ego_status` 입력 |
| `local_pose_topic` | `/localization/pose` | `pose_stamped` 입력 |
| `local_gps_topic` | `/gps` | GPS 입력 |
| `local_imu_topic` | `/imu` | IMU 입력 |
| `local_extent_m` | `120.0` | local 정사각 crop 한 변(m); 현행 호환값 |
| `local_publish_hz` | `5.0` | local topic 최대 발행 Hz |

운영 topic은 `/hd_map/global/*`, `/hd_map/local/*`, `/hd_map/meta`다. debug marker를
planning data로 사용하지 않는다.

## 5. Perception 옵션

Host wrapper:

```bash
./scripts/perception.sh [perception.launch arg:=value ...]
```

container 직접 실행:

```bash
docker exec -it asmc-perception-train bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_perception perception.launch [arg:=value ...]
```

| arg | 기본값 | 설명 |
|---|---:|---|
| `start_camera_lane` | `true` | semantic, per-view BEV, fused BEV, lane vectorizer 실행 |
| `start_traffic_light` | `false` | extra camera 신호등 추론 |
| `start_visualization` | `false` | debug image publisher 실행; GUI 창을 자동으로 열지 않음 |
| `visualization_legacy_rqt` | `false` | 개별 camera·중간/fitting BEV 등 이전 rqt 영상 호환 발행 |
| `visualization_calibration_grid` | `false` | camera별 지면 calibration grid overlay 발행 |
| `publish_bev_debug_metadata` | legacy rqt값(기본 `false`) | per-view coverage, source count/view 발행; 평가·bag·구 시각화용 |
| `camera_checkpoint` | v6 `best_mean.pt` | camera semantic checkpoint |
| `camera_device` | `cuda` | inference device |
| `camera_warmup_iterations` | `1` | model warm-up 횟수 |
| `camera_batch_wait_ms` | `20.0` | multi-view batch 대기(ms) |
| `camera_model_grid_max_hz` | `15.0` | model-grid road-marking 최대 Hz |
| `camera_sensor_set_path` | `ASMC_sensor_set_v3.json` | calibration 검증 sensor snapshot |
| `camera_calibration_config` | `config/cameras_v3.yaml` | BEV camera calibration |
| `bev_grid_config` | `config/bev/bev_grid.yaml` | `base_link` BEV 범위·해상도 |
| `lane_operating_domain` | `shadow_two_way_v1` | typed lane-family ODD ID |
| `lane_algorithm_id` | `twinlite_v6_shadow_boundary_contract_v1` | algorithm/version ID |
| `lane_enable_shared_family_debug` | `false` | shared-family debug fitting·marker |
| `lane_enable_boundary_family_candidate` | shared debug값 | provisional typed family candidate 발행 |
| `lane_publish_pipeline_audit` | `false` | frame별 상세 lane audit; 평가·result bag에서만 명시적으로 활성화 |
| `traffic_light_checkpoint` | package `models/traffic_light/best.pt` | 신호등 checkpoint |
| `traffic_light_device` | `auto` | `auto`, CPU/GPU device override |
| `traffic_light_image_topic` | `/cam_extra/image_jpeg/compressed` | 신호등 camera input |
| `traffic_light_state_topic` | `/traffic_light/state` | legacy String state |
| `traffic_light_detail_topic` | `/traffic_light/detail` | legacy detail |
| `traffic_light_typed_topic` | `/perception/traffic_light/detection` | typed canonical output |
| `traffic_light_hz` | `10.0` | 추론 최대 Hz |
| `traffic_light_min_detect` | `0.40` | 최소 detection confidence |
| `traffic_light_stale_sec` | `1.0` | 입력 stale timeout(s) |

기능별 분리 실행:

```bash
# camera·BEV·lane만
roslaunch asmc_perception perception.launch \
  start_traffic_light:=false start_visualization:=false

# 신호등만
roslaunch asmc_perception perception.launch \
  start_camera_lane:=false start_traffic_light:=true

# 이미 실행 중인 producer의 시각화 publisher만
roslaunch asmc_perception perception.launch \
  start_camera_lane:=false start_visualization:=true
```

`camera_lane.launch`는 전체 lane chain이 아니라 semantic inference node만 실행한다.
일반 주행에서는 항상 최상위 `perception.launch`를 사용한다.

### rqt/RViz 표시

먼저 perception을 시각화 publisher와 함께 실행한다. 이 명령은 GUI 창을 직접 열지는
않는다.

```bash
# repository root의 host terminal
./scripts/perception.sh start_visualization:=true
```

팀 공통 rqt 확인 대상은 아래 두 영상이다.

| rqt topic | 확인 내용 |
|---|---|
| `/perception/camera/debug/mosaic/compressed` | 좌·전방·우 camera의 원본·주행가능·road-marking 9칸 영상 |
| `/perception/bev/debug/comparison/compressed` | 원본 RGB BEV와 fused semantic BEV의 좌우 비교 영상 |

GUI는 `asmc-ros-noetic` container에서 연다. 두 rqt 창과 RViz는 각각 프로세스를 점유하므로
필요한 창마다 별도 host terminal을 열고 다음 공통 준비를 수행한다.

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
```

각 terminal에서 원하는 명령 하나를 실행한다.

```bash
# rqt: camera 9-panel mosaic
asmc-ros-gui rqt-image /perception/camera/debug/mosaic/compressed

# rqt: RGB BEV | semantic BEV comparison
asmc-ros-gui rqt-image /perception/bev/debug/comparison/compressed

# RViz: 기존 marker 확인 방식은 변경 없음
asmc-ros-gui rviz
```

- `camera/debug/mosaic/compressed`: 좌·전방·우 각각의 원본·주행가능·road-marking,
  총 9칸이다.
- `bev/debug/comparison/compressed`: 같은 metric crop의 원본 RGB BEV(왼쪽)와 fused semantic
  결과(오른쪽)다. 오른쪽은 주행가능 확률을 0.5에서 이진화하고 차선 class를 고정
  색으로 그리므로 확률·confidence 밝기를 표현하지 않는다. 별도 확률/품질·fitting·
  grid 영상은 기본으로 광고하지 않는다.
- image publisher는 subscriber-lazy이므로 rqt 창을 열기 전 `rostopic hz`에 아무 값이
  없어도 정상이다.

이전 개별·중간 영상을 잠시 비교해야 할 때만 호환 profile을 켠다.

```bash
./scripts/perception.sh \
  start_visualization:=true \
  visualization_legacy_rqt:=true
```

이 옵션은 camera별 panel, 기존 fused/fused-quality와 lane evidence/fitted/boundary/
reprojection 영상을 다시 광고한다. camera calibration grid는 호환 profile과 분리되어
있으며 다음처럼 명시적으로 켠다.

```bash
./scripts/perception.sh \
  start_visualization:=true \
  visualization_calibration_grid:=true
```

RViz marker 구성은 이번 rqt 정리에서 변경하지 않았다.

기본 실행에서는 per-view projected `coverage`, fused `source_count`, `source_view`도
광고하지 않는다. 평가 bag이나 overlap/calibration 분석에서 필요하면 다음처럼 독립적으로
다시 켠다. `visualization_legacy_rqt:=true`를 사용하면 이 값도 기본 true가 된다.

```bash
./scripts/perception.sh publish_bev_debug_metadata:=true
```

`road_marking/model_grid/*`와 `/perception/bev/debug/{view}`의 semantic raster는 이름에
`grid`·`debug`가 있어도 rqt 화면이 아니라 각각 lane vectorizer와 camera fusion의 내부
입력이다. `rostopic list`에 계속 보이는 것이 정상이며 시각화 정리 목적으로 끄지 않는다.

`/perception/lane/pipeline_audit`도 일반 주행에서는 광고하지 않는다. raw bag 평가나
rejection 원인 분석에서만 다음처럼 켠다. 공식 batch 도구는 이 인자를 자동으로 전달한다.

```bash
./scripts/perception.sh lane_publish_pipeline_audit:=true
```

### Perception runtime 수락검사

bridge와 검사할 perception/planning process를 먼저 각자의 foreground terminal에서 실행한다.
아래 verifier는 인지 노드를 자동 시작하지 않으며, 현재 ROS graph를 15초간 측정한다.

```bash
# 기본 camera semantic → per-view BEV → fused BEV → lane graph
./scripts/verify_perception.sh

# 신호등 producer와 canonical rqt 두 화면도 함께 실행했을 때
./scripts/verify_perception.sh --traffic-light --visualization

# behavior_planner까지 실행한 상태에서 typed + legacy 신호등 구독 확인
./scripts/verify_perception.sh --traffic-light --require-planning

# 판제 팀원이 read-only adapter를 함께 검토할 때만
./scripts/verify_perception.sh --require-lane-adapter
```

| verifier 옵션 | 의미 |
|---|---|
| `--duration SEC` | 측정 시간, 기본 15초 |
| `--max-stale-sec SEC` | 일반 topic의 마지막 수신 허용 age, 기본 1초 |
| `--traffic-light` | typed detection과 legacy state/detail producer 검사 |
| `--visualization` | subscriber-lazy인 canonical mosaic/comparison 2종 검사 |
| `--require-planning` | `/behavior_planner`의 typed/legacy 신호등 구독 검사; `--traffic-light` 필수 |
| `--require-lane-adapter` | `/lane_boundary_monitor`의 typed lane 구독 검사 |
| `--allow-opt-in-debug` | 의도적으로 켠 기본-OFF debug publisher를 정보로만 보고 실패시키지 않음 |
| `--yaml-out PATH` | machine-readable 결과 저장; ROS container 안의 경로 |

기본 검사는 node, message type, 유일 publisher, 필수 내부 subscriber, rate, source stamp,
`frame_id`, staleness와 diagnostics ERROR를 함께 본다. P5-B에서 기본-OFF로 바꾼 19개 topic이
옵션 없이 다시 광고되어도 실패한다. 계약 정본과 판제/launch 경로의 정적 연결만 빠르게
확인하려면 다음을 실행한다.

```bash
python3 tools/audit/validate_perception_runtime_acceptance.py
python3 -m unittest tools/audit/test_perception_runtime_acceptance.py
```

세부 기준과 해석은
[P5-C runtime acceptance](./records/baselines/perception-p5c-runtime-acceptance-v1.md)를 따른다.

## 6. Planning/Control 옵션

```bash
roslaunch behavior_planner autonomy_no_bridges_v2.launch [arg:=value ...]
```

| arg | 기본값 | 설명 |
|---|---:|---|
| `state_source` | `gps_imu` | `gps_imu`, local 개발용 `ego_dev` |
| `dashboard` | `false` | behavior dashboard |
| `traffic_light` | `true` | 판제 launch가 신호등 producer를 시작할지 |
| `traffic_light_device` | `auto` | 신호등 inference device |
| `camera_topic` | `/cam_extra/image_jpeg/compressed` | 신호등 input |
| `traffic_light_checkpoint` | `asmc_perception/models/traffic_light/best.pt` | checkpoint |

`asmc_perception` runtime에서 신호등을 켰다면 반드시 `traffic_light:=false`로 중복
producer를 막는다. bridge와 `/vehicle/state` producer도 중복으로 켜지 않는다.

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
./scripts/docker_perception_up.sh down
./scripts/docker_ros_up.sh down
xhost -local:docker
```

주행·녹화 중에 `up`, `down`, `rebuild`로 container를 재생성하지 않는다.
