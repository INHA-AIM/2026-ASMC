# ASMC 실행 명령어 모음

> **문서 역할:** Active Operator Reference — 현재 runtime 실행·시각화·녹화 명령과 공개 옵션
>
> **범위:** 본선·local 공통 실행 진입점. 학습·offline 분석·HD Map 생성용 내부
> launch는 각 package README를 따른다.

명령은 host의 팀 저장소 root `$ASMC`와 container의 `/root/ws`를 기준으로 한다.
host의 `$ASMC`는 실제 checkout한 `2026-ASMC`를 가리켜야 한다.
ROS launch 인자는 `name:=value` 형태로 덮어쓴다.

## 1. 가장 빠른 local 실행

### Host — container 준비

```bash
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
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_perception perception.launch
```

신호등과 debug image publisher까지 필요하면:

```bash
roslaunch asmc_perception perception.launch \
  start_traffic_light:=true \
  start_visualization:=true
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

운영 추론은 bridge·GUI와 같은 `asmc-ros-noetic` 컨테이너의 별도 terminal에서 직접
실행한다.

```bash
docker exec -it asmc-ros-noetic bash
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
| `lane_enable_shared_family_debug` | `true` | 음영구간 shared-family 보강 fitting·근거 marker; 성능 측정에서만 `false` |
| `lane_enable_boundary_family_candidate` | `true` | 판제 소비 정본 `/perception/lane/boundary_family` 발행 |
| `lane_enable_independent_boundary_pipeline` | `false` | legacy 개별 spline·tracker·`boundary_observations`; 호환 평가에서만 `true` |
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

### 음영구간 shared-family와 판제 adapter RViz

일반 `perception.launch`를 옵션 없이 실행해도 음영구간 보강
`/perception/lane/boundary_family`와 그 근거 marker가 기본 생성된다. 판제는 이 family를
차선 인지 정본으로 사용한다. legacy 개별 spline·tracker와
`/perception/lane/boundary_observations`는 기본 비활성화다. 기존 호환·진단 재현이
필요할 때만 `lane_enable_independent_boundary_pipeline:=true`로 켠다.

```bash
# asmc-ros-noetic container
roslaunch asmc_perception perception.launch
```

bag 처리시간 비교처럼 family 근거 marker 비용을 제외하려면
`lane_enable_shared_family_debug:=false`만 주고, family 출력은 그대로 유지한다.

별도 `asmc-ros-noetic` terminal에서 adapter를 실행하면 Fixed Frame이 `base_link`인 전용
RViz가 자동으로 열린다. 최종 accepted strip과 perception support/gap marker가 이미 등록돼
있어 별도 RViz terminal이나 display 추가가 필요 없다.

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch
```

bag replay·처리시간 측정처럼 GUI가 필요 없을 때만 `rviz:=false`를 추가한다. adapter와 RViz는
read-only이며 Behavior/Frenet/제어 명령을 발행하지 않는다.

### rqt/RViz 표시

먼저 perception을 시각화 publisher와 함께 실행한다. 이 명령은 GUI 창을 직접 열지는
않는다.

```bash
# asmc-ros-noetic container
roslaunch asmc_perception perception.launch start_visualization:=true
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
roslaunch asmc_perception perception.launch \
  start_visualization:=true \
  visualization_legacy_rqt:=true
```

이 옵션은 camera별 panel, 기존 fused/fused-quality와 lane evidence/fitted/boundary/
reprojection 영상을 다시 광고한다. camera calibration grid는 호환 profile과 분리되어
있으며 다음처럼 명시적으로 켠다.

```bash
roslaunch asmc_perception perception.launch \
  start_visualization:=true \
  visualization_calibration_grid:=true
```

RViz marker 구성은 이번 rqt 정리에서 변경하지 않았다.

기본 실행에서는 per-view projected `coverage`, fused `source_count`, `source_view`도
광고하지 않는다. 평가 bag이나 overlap/calibration 분석에서 필요하면 다음처럼 독립적으로
다시 켠다. `visualization_legacy_rqt:=true`를 사용하면 이 값도 기본 true가 된다.

```bash
roslaunch asmc_perception perception.launch publish_bev_debug_metadata:=true
```

`road_marking/model_grid/*`와 `/perception/bev/debug/{view}`의 semantic raster는 이름에
`grid`·`debug`가 있어도 rqt 화면이 아니라 각각 lane vectorizer와 camera fusion의 내부
입력이다. `rostopic list`에 계속 보이는 것이 정상이며 시각화 정리 목적으로 끄지 않는다.

`/perception/lane/pipeline_audit`도 일반 주행에서는 광고하지 않는다. raw bag 평가나
rejection 원인 분석에서만 다음처럼 켠다. 공식 batch 도구는 이 인자를 자동으로 전달한다.

```bash
roslaunch asmc_perception perception.launch lane_publish_pipeline_audit:=true
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

# perception·신호등·rqt·planning·lane adapter를 모두 띄운 최종 P5 gate
./scripts/verify_perception.sh \
  --traffic-light --visualization --require-planning --require-lane-adapter
```

| verifier 옵션 | 의미 |
|---|---|
| `--duration SEC` | 측정 시간, 기본 15초 |
| `--max-stale-sec SEC` | 일반 topic의 마지막 수신 허용 age, 기본 1초 |
| `--master-timeout-sec SEC` | ROS master XML-RPC 호출 제한, 기본 3초; master가 없을 때 무한 대기 방지 |
| `--traffic-light` | typed detection과 legacy state/detail producer 검사 |
| `--visualization` | subscriber-lazy인 canonical mosaic/comparison 2종 검사 |
| `--require-planning` | `/behavior_planner`의 typed/legacy 신호등 구독 검사; `--traffic-light` 필수 |
| `--require-lane-adapter` | `/boundary_family_monitor`의 canonical `boundary_family` 구독 검사 |
| `--allow-opt-in-debug` | 의도적으로 켠 기본-OFF debug publisher를 정보로만 보고 실패시키지 않음 |
| `--yaml-out PATH` | machine-readable 결과 저장; ROS container 안의 경로 |

기본 검사는 node, message type, 유일 publisher, 필수 내부 subscriber, rate, source stamp,
`frame_id`, staleness와 diagnostics ERROR를 함께 본다. P5-B와 shared-family closeout에서
기본-OFF로 정한 20개 topic이
옵션 없이 다시 광고되어도 실패한다. 계약 정본과 판제/launch 경로의 정적 연결만 빠르게
확인하려면 다음을 실행한다.

```bash
python3 tools/audit/validate_perception_runtime_acceptance.py
python3 -m unittest tools/audit/test_perception_runtime_acceptance.py
```

세부 기준과 해석은
[P5-C runtime acceptance](./records/baselines/perception-p5c-runtime-acceptance-v1.md)와
[P5-D/E closeout](./records/baselines/perception-p5de-closeout-v1.md)을 따른다.

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
| `traffic_light_checkpoint` | `asmc_perception/models/traffic_light/best.pt` | checkpoint |

신호등 producer의 운영 소유자는 `asmc_perception`이고 planning launch 기본값은 `false`다.
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

## 10. 개발용 LiDAR 실시간 지면 비교

독립 LiDAR worktree가 마운트된 `asmc-ros-noetic` 컨테이너 안에서 실행한다.
GT 센서·YOLO·기존 Main·카메라 perception은 필요 없다. UDP bridge가 이미
실행 중이면 중복 실행하지 않는다. 운영 장애물 토픽을 바꾸는 명령이 아니다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

# 기존 bridge 사용, 세 방법과 RViz
roslaunch asmc_lidar ground_compare_live.launch

# bridge까지 함께 시작할 때만
roslaunch asmc_lidar ground_compare_live.launch start_bridge:=true

# 한 방법만 계산
roslaunch asmc_lidar ground_compare_live.launch method:=linefit
```

| 옵션 | 기본값 | 선택 |
|---|---|---|
| `method` | `all` | `all / jaeho / patchwork / linefit` |
| `patchwork_variant` | `guard25` | 고정 높이 1.80m + 25cm 보호. 보호 전 결과는 `fixed_height` |
| `rviz` | `true` | `false`면 점군·진단만 발행 |
| `start_bridge` | `false` | `true`면 기존 `asmc_bridge/bridge.launch` 포함 |
| `transport_profile` | `local` | 함께 시작하는 bridge에 적용, `lan2lan`도 지원 |
| `stream_profile` | `competition` | 함께 시작하는 bridge에 적용 |
| `timestamp_mode` | `receive` | 함께 시작하는 bridge는 `receive / source`; 기존 bridge는 변경하지 않음 |
| `cmd_ip` | local `127.0.0.1` | LAN2LAN 기본 `192.168.0.1` |
| `cloud_topic / imu_topic` | `/velodyne_points / /imu` | 다른 입력 이름으로 remap |

RViz fixed frame은 `base_link`다. Jaeho / Patchwork / Linefit 그룹을 토글해
**같은 위치**에서 비교한다. `nonground`는 장애물 확정이 아니다.
재호의 `structural_removed`와 `native_retained`를 통해 지면 탐색 도중 제거된
수직면을 지면 삭제와 구분할 수 있다.

```bash
rostopic echo /lidar_candidate/ground_live/status
rostopic hz /lidar_candidate/ground_live/linefit/nonground
```

[실시간 비교와 이전 단일 프레임 도구](packages/asmc_lidar/ground-method-comparison-visualization.md),
[측정 보고서](packages/asmc_lidar/l2a2-v1-live-ground-review-20261004.md)를 함께 확인한다.
