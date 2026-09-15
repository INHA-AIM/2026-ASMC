# rosbag 평가·Streaming GT 가이드

> **문서 역할:** Active Guide — 재현 가능한 raw/result bag과 개발용 Streaming GT
> **기준일:** 2026-09-14
> **대회 운용 경계:** MORAI↔팀 PC는 UDP만 사용. Semantic·Instance·Object GT는 개발 전용
> **관련:** [UDP↔ROS 네트워크](./network-udp-ros.md) · [인지 융합 로드맵](./perception-map-fusion-roadmap.md) · [data_collection README](../src/data_collection/README.md)

## 1. 어떤 수집 방식을 언제 쓰는가

MORAI의 시간 진행, 외부 Sync, Capture, 센서 transport와 기록 형식은 서로 독립된 설정이다.
현재 실측과 목적에 따라 기본 경로를 다음처럼 구분한다.

| 목적 | 정본 경로 | 이유 |
|---|---|---|
| 카메라/LiDAR 단일-frame 학습 GT | v2 + Capture 1 Hz | 장착 센서 수를 늘리지 않고 RGB·Semantic·Instance와 box 파일을 같은 Capture로 보존 |
| 고주기 RGB sequence | v2 + Simulation Time Sensor Record | 장착된 RGB·일반 센서 파일을 공통 주기 후보로 저장 |
| 고주기 RGB–Semantic supervised sequence | 최소 RGB+Semantic 센서셋 + Simulation Time Sensor Record | v2에 없는 Semantic 센서를 명시적으로 장착해야 함 |
| 입력 센서 skew를 배제한 알고리즘 검증 | Simulation Time Sensor Sync + UDP source-time bag/replay | 이상적인 동시 입력에서 알고리즘 자체를 분리 평가 |
| 대회 운용·실시간 지연 평가 | v2 + UDP Connect + `full_raw`/파트별 raw bag | 실제 비동기 주기·drop·latency 보존 |
| 고주기 개발용 oracle | GT 센서셋 + UDP source-time bag | Capture 정지 없이 RGB/Semantic, Intensity/Instance 스트림 기록 |
| 모델 A/B | 동일 raw bag replay + 모델별 result bag | 입력을 고정하고 checkpoint별 결과를 분리 |

Capture 요청은 2026-09-09 pilot에서 요청 주기 2/5/10 Hz와 무관하게 약 1.4~1.7 Hz에
포화됐고 요청마다 시뮬레이션이 약 0.5초 멈췄다. 따라서 wheel/MPC 주행의 temporal 성능
측정에는 쓰지 않는다. Capture v2의 한 LiDAR 파일 안에는 Intensity·Semantic·Instance와
Instance box 텍스트가 함께 생성되고 XYZ가 일치했으므로 학습 정답의 1순위는 Capture다.

2026-09-14 v2 Sensor Record 실측에서는 `CAMERA_1~4`, `LIDAR_5`, `GPS_6`, `IMU_7`만
새로 기록됐고, 과거 GT 센서셋 디렉터리인 `CAMERA_8~10`, `LIDAR_11`은 갱신되지 않았다.
Sensor Record는 장착된 센서의 고주기 파일 저장 기능이지 Capture의 oracle channel 생성
기능이 아니다. 따라서 v2 Sensor Record는 RGB sequence에는 쓸 수 있지만 segmentation
학습 label을 제공하지 않는다. 고주기 supervised camera data가 필요할 때만
`RGB front/left/right + Semantic front/left/right + 필요한 상태 센서`로 제한한 별도
sync-training 센서셋을 만들고, 기존 v2/대회 설정과 provenance를 분리한다.

Simulation Time Sensor Synchronization은 실제 UDP 운용의 대체물이 아니다. 센서 생성 skew를
배제하려면 브릿지를 `timestamp_mode:=source`로 실행해 canonical topic과 `/clock`, Ego/IMU를
bag으로 남기거나 Sensor Record 파일을 offline replay해야 한다. `receive` stamp에는 도착
시각 차이가 남는다. 이 결과는 `controlled/ideal-input`, Real Time UDP bag 결과는
`deployment/operational`로 표기하고 두 기준을 함께 보고한다.

MORAI는 카메라와 3D LiDAR에 ROS와 UDP를 지원하지만, ROS 선택이 외부 tick 동기화나
GT 정합을 자동 보장하지 않는다. 현재 ROS1 연결은 `ws://127.0.0.1:9090` rosbridge를
경유하므로 실제 수신률·timestamp·부하로 판단한다. 근거는 MORAI
[센서 통신 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/-35)과
[Capture Mode](https://morai-sim--drive-user-manual--en-22-r2.scrollhelp.site/msdume2/capture-sensor-data)를 따른다.

## 2. Streaming GT 브릿지

공용 UDP bridge는 v2와 GT 포트 합집합을 항상 연다. v2를 로드하면 선택
GT 포트가 대기하고, GT를 로드하면 Semantic과 Instance가 자동 수신된다.
SIM 버전은 `udp_bridge.sh` 자체에서 선택해 provenance를 남긴다.

```bash
# 26.R1.H3 일반 개발: receive stamp. v2/GT 모두 수신
./scripts/udp_bridge.sh --simulator 26r1

# 정합이 필요한 Streaming GT 녹화: source stamp
./scripts/udp_bridge.sh --simulator 26r1 timestamp_mode:=source

# 대회판: privileged GT 없이 UDP만 사용
./scripts/udp_bridge.sh --simulator competition
```

| 센서 | UDP | canonical topic | 네 번째 필드 |
|---|---:|---|---|
| ID 5 Intensity | 9299 | `/velodyne_points` | `intensity` float32 |
| ID 11 Instance | 9311 | `/velodyne_points_instance` | `instance_id` uint32 |

두 LiDAR는 publisher, UDP socket, packet buffer와 scan 통계를 공유하지 않는다. 같은 pose이므로
`frame_id=velodyne`과 `base_link→velodyne` TF는 공유한다. Instance ID는 정답 전용이며 모델
입력으로 쓰지 않는다. 0은 비대상 point, 1 이상은 객체 ID 후보이나 ID만으로 class를 알 수
없어 `/Object_topic` 또는 Capture의 `_instance.txt`와 결합해야 한다.

### timestamp mode

| mode | stamp | 용도 |
|---|---|---|
| `receive` | 브릿지가 message를 만든 ROS 시각 | 실제 end-to-end latency와 대회 운용 |
| `source` | MORAI camera/IMU/Ego/Object sec·nsec와 Velodyne device time | 개발용 stream 정합 |

카메라는 timestamp·fragment index·payload size가 일치하는 fragment만 한 frame으로 조립한다.
Velodyne은 microseconds-since-hour 값을 현재 ROS hour에 unwrap해 마지막 유효 packet 시각을
cloud stamp로 쓴다. source 값이 잘못됐거나 역행하면 receive time으로 fallback하고
`/morai_sensor_bridge/diagnostics`, `/morai_udp_bridge/diagnostics`에 카운터를 남긴다. GPS NMEA에는
source stamp가 없으므로 두 mode 모두 receive time이며 metadata에 이 예외를 기록한다.

## 3. raw/result recorder

profile은 snapshot용 `config/profiles/`와 분리된
`src/data_collection/config/bag_profiles/`에 있다. recorder는 `rosbag record`를 감싸되
센서 message를 재직렬화하거나 온라인 동기화하지 않는다.

| profile | 주요 내용 | transport | GT/배포 |
|---|---|---|---|
| `full_raw` | RGB 4, ID 5, GPS/IMU/Ego/TF/control | UDP receive | 배포 가능 |
| `camera_raw` | RGB 전·좌·우와 차량 상태 | UDP receive | 배포 가능 |
| `lidar_raw` | ID 5와 GPS/IMU/Ego | UDP receive | 배포 가능 |
| `lidar_dev_gt_udp` | ID 5·11, Object, GPS/IMU/Ego | UDP source | privileged |
| `dev_oracle_udp` | RGB/Semantic, ID 5·11, Object와 상태 | UDP source | privileged |
| `dev_oracle_raw` | `dev_oracle_udp`의 canonical alias | UDP source | privileged |
| `camera_semantic_result` | 카메라 추론 결과와 diagnostics | replay clock | 결과 |

일반 profile에 Semantic·Instance·`/Object_topic`을 넣으면 시작 전에 실패한다. GT profile은
`privileged_gt: true`, `deployment_allowed: false`다. 같은 canonical topic에 ROS/UDP publisher가
둘 이상 있거나 profile의 timestamp mode와 bridge mode가 다르면 녹화를 시작하지 않는다.

### 실행

사용자는 보통 scenario와 controller만 적는다.

```bash
# 브릿지는 receive mode로 실행
roslaunch data_collection bag_recorder.launch \
  profile:=full_raw scenario:=comp_sample controller:=wheel

# 브릿지는 GT/source mode로 실행
roslaunch data_collection bag_recorder.launch \
  profile:=lidar_dev_gt_udp scenario:=jamming_modified controller:=morai_auto
```

run ID는 KST 시작 시각, map, 초기 날씨, simulation hour와 controller로 자동 생성된다.

```text
YYYYMMDD_HHMMSS_kcity2025_<scenario>_<weather>_<sim-time>_<controller>
20260909_143012_kcity2025_comp_sample_foggy_1pm_wheel
```

MORAI gRPC 조회가 실패하면 녹화를 막지 않고 `unknown`과 오류를 남긴다. scenario와 controller는
MORAI가 확실히 제공하지 않으므로 사용자가 지정한다. 녹화 중 환경은 2초마다
`environment_timeline.jsonl`에 저장한다. 같은 run ID나 result model ID는 덮어쓰지 않는다.
gRPC를 쓸 수 없는 상태에서 실제 조건을 알고 있으면 `weather:=foggy sim_hour:=13`처럼 시작
값만 명시할 수 있으며, 자동 조회값을 덮었다는 launch 설정도 metadata에 남는다.

```text
/data/rosbags/
├── raw/<run_id>/
│   ├── metadata.yaml
│   ├── environment_timeline.jsonl
│   └── bags/<run_id>_0.bag
├── results/<raw_run_id>/<model_id>/
│   ├── metadata.yaml
│   └── bags/<model_id>_0.bag
├── audits/<comparison-or-run>/audit.json
└── rejected/<run_id>/
    ├── rejection.yaml
    └── ...
```

raw는 무압축, result는 LZ4, 기본 split은 4 GiB, rosbag buffer는 1 GiB이며 시작 전에 20 GiB를
확인한다. Ctrl-C는 rosbag에 SIGINT를 전달한 뒤 closed bag의 SHA-256과 종료 수신률을
metadata에 남긴다. 강제 중단 시 `.active`와 `status: interrupted`를 보존한다.
단, 컨테이너나 WSL 자체가 즉시 강제 종료돼 recorder가 finalization을 실행하지 못하면
`metadata.yaml`이 `recording`으로 남을 수 있다. 이 경우 `.active`를 원본 그대로 보존하고
`rosbag reindex`/`fix` 여부를 별도 복구 절차에서 판단한다.

잘못된 closed run은 삭제하거나 덮어쓰지 않고 다음처럼 옮긴다.
해당 raw를 parent로 둔 result가 있으면 rejected run 아래 `results/`로 함께 이동한다.

```bash
rosrun data_collection reject_bag_run.py \
  --run-id <run_id> --reason "scenario setup before recording was incomplete"
```

## 4. replay와 result lineage

replay 전에는 live UDP/ROS bridge를 끈다. 같은 raw topic publisher가 이미 있으면 replay가
시작을 거부한다. `/use_sim_time`은 replay controller가 먼저 켜고 종료 후 원래 값으로
복원한다. 추론이나 recorder를 먼저 실행하면 두 노드가 wall clock을 고정해 result bag의
시각과 지연 값이 깨질 수 있으므로 실행 순서는 반드시 **paused replay controller → 추론
노드 → result recorder → start service**로 한다.

첫 터미널에서 replay controller를 실행한다. 기본 `wait_for_start:=true`라서 raw 메시지를
즉시 내보내지 않고 `/use_sim_time=true`만 준비한 채 대기한다. live publisher 검사는 이
시점에 수행된다.

```bash
roslaunch data_collection bag_replay.launch \
  bag_path:=/data/rosbags/raw/<raw_run_id> \
  required_profile:=camera_raw
```

두 번째 터미널에서 추론 노드를 시작한다. GPU/CUDA 첫 호출 때문에 초반 프레임이 유실되지
않도록 카메라 추론 노드는 구독 시작 전에 기본 1회 warmup을 수행한다. 필요할 때만
`warmup_iterations:=0`으로 끌 수 있다. 세 번째 터미널에서 result recorder를 시작한다.

```bash
roslaunch data_collection bag_recorder.launch \
  profile:=camera_semantic_result \
  parent_raw_run_id:=<raw_run_id> \
  model_id:=twinlite_marking_v1_epoch18 \
  checkpoint:=/artifacts/runs/<checkpoint>.pth
```

result recorder는 `/use_sim_time=true`가 아니면 시작을 거부한다. recorder가 `Recording to`
또는 clock 대기 상태를 출력하면 네 번째 터미널에서 재생을 연다.

```bash
rosservice call /asmc_bag_replay/start
```

재생 완료 후 기본 2초 post-roll 동안 sim clock을 유지해 마지막 추론 결과를 배출한 다음
원래 clock 설정을 복원한다. 그 뒤 result recorder와 추론 노드를 Ctrl-C로 닫는다.
자동화 환경에서만 `wait_for_start:=false`를 사용할 수 있지만, downstream readiness를 별도
보장해야 한다.

result metadata에는 parent raw ID, model ID, checkpoint path/SHA-256, Git 상태와 추론
diagnostics가 남는다. `/perception/camera/diagnostics`에는 backend/config/checkpoint, view와
입력 크기, view별 input/output/replaced frame, 오류 수, stage별 시간, processing FPS,
queue/e2e latency와 source/receive/result stamp가 기록된다.

result recorder의 `model_id`와 실제로 존재하는 `checkpoint` 경로는 필수다. 존재하지 않는
가중치 경로를 주면 lineage가 불완전해지므로 녹화를 시작하지 않는다.

GT bag은 기본 replay가 차단된다. oracle 분석에서만 `allow_privileged_gt:=true`를 명시한다.
GT topic을 모델 입력으로 연결하는 것은 profile과 별개로 금지한다.

## 5. 정량 감사와 현재 실측

bag 자체를 읽어 rate, header 역행, bag-header age, RGB–Semantic/LiDAR 최근접 gap과 Instance
schema를 계산한다. raw 디렉터리는 불변으로 두고 결과는 `audits/`에 쓴다.
설정 Hz 대비 실측 Hz, 긴 arrival gap과 실제 packet/frame drop은 구분해야 한다. sequence가
없는 stream에서는 auditor가 정확한 drop 수를 복원할 수 없으며, 현재 bridge 처리 지연과
수신률 기준선의 정의는 [UDP↔ROS §6.4](./network-udp-ros.md#64-2026-09-09-v2-udpros-지연수신률-기준선)를
따른다.

```bash
rosrun data_collection audit_rosbag_run.py \
  /data/rosbags/raw/<run_id> \
  --json-out /data/rosbags/audits/<run_id>/audit.json

# gate 실패 시 exit 2가 필요한 CI/선정 단계
rosrun data_collection audit_rosbag_run.py /data/rosbags/raw/<run_id> --strict
```

### raw → replay → result smoke 기준선 (2026-09-09)

`camera_raw` 30초 run을 동일 4-class checkpoint로 1.0배속 재생해 전체 lineage를 검증했다.
초기 구현처럼 downstream을 먼저 띄운 뒤 `/use_sim_time`을 바꾸면 result bag time이 raw
header보다 약 716초 늦어지는 오류가 재현됐다. 위 paused 순서와 warmup을 적용한 재시험은
다음 결과를 보였다.

| 항목 | 결과 |
|---|---:|
| raw RGB 전/좌/우 | 20.0 / 20.0 / 20.0 Hz |
| result 전/좌/우 | 18.76 / 19.78 / 19.71 Hz |
| 처리량(3 view 합계) | 최근 구간 약 56–59 frame/s |
| 모델 inference stage | 최근 구간 약 23–25 ms/batch |
| end-to-end p50 / p95 | 53.8 / 84.8 ms |
| output bag-header p95 | view별 약 77–80 ms |
| decode / inference error | 0 / 0 |
| header timestamp regression | 0 |

`processing_fps`는 세 view의 출력 frame 합계이므로 카메라당 FPS와 혼동하지 않는다.
`replaced_by_view`는 latest-frame 처리에서 새 프레임으로 교체된 수이며 네트워크 packet drop과
다르다. raw 수신률, 모델 출력률, replaced 수, 오류 수와 end-to-end latency를 함께 보고
모델·센서 설정을 비교한다.

같은 날 Foggy 13시에서 별도 live smoke를 수행해 자동 run ID가
`..._pipeline_smoke_foggy_1pm_morai_auto`로 생성되고 2초 주기 timeline도 계속
`weather=foggy`, `sim_hour=13`을 기록함을 확인했다. 이 표의 성능 수치는 앞서 저장한
Cloudy 13시 raw를 고정 입력으로 재생한 결과이며 날씨 조건을 섞어 표기하지 않는다.

Foggy 13시의 `full_raw` 19초 smoke에서도 전/좌/우 RGB 20.01Hz, 후방 RGB 10.01Hz,
Intensity LiDAR 8.45Hz, GPS 5.00Hz, IMU 48.58Hz, Ego 48.67Hz가 기록됐다. 모든 header는
단조 증가했고 Intensity LiDAR의 bag-header p95는 1.51ms였다. v2 센서셋이므로 Semantic과
Instance topic은 profile에 포함되지 않았으며 raw metadata에는 `transport=udp`,
`timestamp_mode=receive`, `bridge_configuration_source=live_ros_parameters`, bag SHA-256과
`status=complete`가 남았다.

### GT UDP source-time 기준선 (2026-09-09)

| 항목 | 결과 | 판단 |
|---|---:|---|
| RGB/Semantic 전·좌·우 | 각각 약 20 Hz | 통과 |
| RGB extra | 약 10 Hz | 설정값과 일치 |
| ID 5 / ID 11 | actor 포함 34초 bag 8.61 / 7.63 Hz | 7 Hz 이상, 차이 15% 이내 |
| front/left/right RGB–Semantic p95 | 최신 20초 34/35/31 ms | 30 ms 기준 초과 |
| camera pairing 변동 | 다른 20초 구간 3/3/3 ms | 센서 위상에 따라 변해 5분 gate 필요 |
| ID 5–11 p50 / p95 | 12.0 / 101.6 ms | p95 50 ms 기준 초과 |
| source regressions/fallbacks | 관측 구간 0 / 0 | 통과 |
| ID 11 schema | `x,y,z,float32 + instance_id,uint32` | 통과 |
| Instance ↔ Object ID | Instance `[2,3,4]`, Pedestrian `[2,4]`, NPC `[3]` | 미해결 ID 0개, 통과 |

따라서 현재 UDP Streaming GT는 rate/schema gate는 통과했지만 정합 gate는 통과하지 못했다.
**학습 pair를 자동 확정하는 경로가 아니라 평가·디버깅용 oracle**로 취급한다. offline
pairing에서 허용 gap을 넘는 frame은 버리고, LiDAR detector의 정식
학습 label은 Capture의 Instance point와 `_instance.txt`/`_instance_8Points.txt`를 쓴다.
Foggy 13시 통제 장면의 `lidar_dev_gt_udp` bag에서 364만 sampled point 중 41,551개가
0이 아닌 Instance였고, unique ID `[2,3,4]`가 `/Object_topic`의 NPC·Pedestrian ID와 모두
대응했다. recorder는 `privileged_gt=true`, `deployment_allowed=false`, `timestamp_mode=source`로
분리됐으며 정상 종료와 bag hash도 남았다. 따라서 **Streaming GT transport와 객체 ID
보존 검증은 완료**됐지만, LiDAR p95 정합 gate는 여전히 실패한다. 이 실패를 숨기지 않고
auditor가 `ok=false`로 보고하므로 학습 pair로 자동 승격하지 않는다. Capture box와의 수치
교차검증은 LiDAR detector 학습 데이터셋을 실제 만들 때 수행한다.

## 6. 후순위 ROS GT 검토 기록

MORAI의 Network Settings와 센서를 ROS로 전환하고 `rosbridge_websocket`을
`0.0.0.0:9090`에서 실행해 확인했다. Network Settings는 21개 WebSocket client로 연결됐고
`/Object_topic`은 약 39~40 Hz로 수신됐다. 정지 장면에 배치한 NPC 차량은 unique ID 3,
보행자는 ID 2와 4로 class/name/size/pose를 포함해 정상 수신됐다.

그러나 RGB·Semantic·ID 5·ID 11·GPS·IMU 센서 topic은 ROS master에 하나도 advertise되지
않았다. `/Ego_topic`은 advertise됐지만 MORAI payload의 `angular_velocity`를 현재 팀의
beta `EgoVehicleStatus.msg`가 갖고 있지 않아 rosbridge가 각 message를 거부했다. 즉 필수
topic 연속 수신 조건을 시작 단계에서 만족하지 못했으므로 이 조건에서는 5분 부하·정합
비교를 진행하지 않고 ROS gate 실패로 판정했다. `dev_oracle_raw`는 검증된 UDP alias다.

현재 저장소에는 ROS GT용 센서셋·adapter·bag profile을 포함하지 않는다. 향후 UDP만으로
해결할 수 없는 요구가 확인될 때 별도 실험 PR에서 MORAI UI export 센서셋과 메시지 adapter를
함께 설계한다. 팀 공용 `morai_msgs`를 실험 목적으로 즉시 바꾸지는 않는다.

완료한 항목:

- NPC·보행자를 둔 `lidar_dev_gt_udp` bag 기록과 Instance schema/rate 검증
- ID 11의 비영 Instance ID와 `/Object_topic` unique ID 대응 검증

후순위 항목:

1. LiDAR 학습 데이터셋을 만들 때 같은 Capture의 `_instance.txt`/8Points box와 수치 교차검증
2. UDP만으로 부족한 지표가 확인될 때 별도 ROS GT 실험 범위와 5분 A/B gate 재수립

현재 UDP는 대회 운용과 raw/result 재현 평가에 문제가 없고 ROS 1차 gate는 필수 센서
advertise 단계에서 실패했다. 따라서 ROS GT 재실험은 카메라 BEV·판제 전달 인터페이스보다
후순위로 둔다.
