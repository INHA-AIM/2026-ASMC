# 개발·시뮬레이터 운영 가이드

> **2026-10-27 현재 실행 기준:** CompetitionInfoPublisher는9088→9099→`/Ego_topic`이다.
> `--require-competition-status`/`verify-field`도 이9099 입력을 검사하도록 정정했다.
> 9798은 로컬 Detail 진단이며 LAN 운영·수락에서 제외한다. 아래 과거 P3 설명과 현재 역할은
> [정정 기록](../records/bridge/2026-10-27-competition-9099-correction.md)을 기준으로 구분한다.


> **문서 역할:** Canonical — Docker, MORAI UDP profile, bridge 기동과 시뮬 검증
>
> **현재 기준:** 공통 `morai_udp` bridge + 본선 `ASMC_sensor_set_v3.json` + ROS1 Noetic + Docker host network

대회 허용 범위는 [대회 환경](../competition/README.md), Planning/Control의 실행은
[Planning/Control 규약](./planning-control.md), bag 기록·재생은
[rosbag 가이드](./rosbag-evaluation.md)를 따른다. parser·timestamp·TF·`vehicle_state`를
수정할 때는 [MORAI bridge guide](../../src/asmc_bridge/docs/integration.md)를
함께 본다.

## 1. 현재값과 남은 승인 항목

P3-A에서 공통 runtime과 profile 계약, Ctrl 포트를 적용했고 P3-B에서
Competition Vehicle Status decoder, P3-C에서 stream allowlist와 센서 transport 파생본을
추가했다. 파일이 존재하는 것과 현장
live acceptance가 끝난 것을 구분한다.

| 항목 | 현재 적용값 | 남은 gate |
|---|---|---|
| 개발 IP | Host/Destination `127.0.0.1` | 동일 |
| 현장 IP | profile 계약·생성본: MORAI `192.168.0.1`, Autonav `192.168.0.10` | 실제 두 PC 리허설 |
| Ego Ctrl Cmd | `9093/9094`, type 1, mode 2 | local brake closed loop 통과; LAN2LAN 재검증 |
| 차량 상태 | Ego `9088/9099`; MolitComp Competition `9797/9798` | Competition mode live packet·field 확인 |
| TrafficLight simulator oracle | `258`, `7603/7502` | team production bridge에서 소비 금지 |
| CollisionData | destination `9092` | `9091/9092` pair |
| ObjectInfo | `7505`, 임시 개발 oracle | competition profile에서 제거 |
| 센서 | v2/v3 local·lan2lan 파생본, `9290~9303` | 실제 LAN packet rate |
| privileged stream | 기본 차단, local `development`에서만 활성 | 본선 launch topic absence 재확인 |
| VLP16 | v2 10 Hz 회귀, v3 설정·소비 15 Hz | local 완성 scan 약 12 Hz 허용; LAN 자원·지연 재검증 |

정본은 [`bridge_profiles_v1.json`](../../config/bridge/bridge_profiles_v1.json)이다. generated
network/sensor JSON·decoder·synthetic UDP publish·allowlist 계약은 검증하지만,
실측된 `7502` packet은 Competition이 아닌 `#TrafficLight$`였다. 실제 Competition type 787의
`9798`은 아직 0건이므로 차단 상태다. 실제 LAN 검증도 남았다. v3는 설정과 소비 loop가
15 Hz이고 완성 scan 약 12 Hz 입력은 허용한다.

type 258 TrafficLight status와 `GetTrafficLightStatus` 조회 서비스는 서로 다르다. 조사한 주최
MolitComp03, 팀 MolitComp03, 26.R1 network export에는 type 778 MoraiTLServiceProvider가 없고
참가팀 MolitComp UI에도 GetTrafficLight 항목이 없었다. 26.R1의 gRPC 연결 항목이나 저장소 proto
존재만으로 query 가능성을 가정하지 않는다. production 신호등 입력은 카메라 인지 결과이며,
TrafficLight oracle/GetTrafficLight는 bridge 필수 토픽이 아니다.

## 2. 런타임 경계

```text
MORAI --UDP--> asmc_bridge의 sensor/network node --> 팀 내부 ROS topic
MORAI <--UDP-- asmc_bridge network node <--ROS-- /ctrl_cmd

/planning/vehicle/state + /planning/objects --> asmc_planner(Behavior→Frenet)
  --> MPC /ctrl_cmd/normal + 음영 /ctrl_cmd/jamming --> selector --> /ctrl_cmd
```

- MORAI와 팀 PC 사이에는 허용 UDP만 쓴다.
- ROS는 팀 PC 내부의 정규화·인지·디버깅 경계다.
- Behavior/Frenet은 asmc_planner 내부 직접 호출이며 MPC와는 ROS trajectory로 연결된다. 직접 UDP socket을 열지 않는다.
- `rosbridge.sh` direct WebSocket 실험은 종료됐으며 현재 개발·본선 경로가 아니다.
- gRPC scenario runner와 GT topic은 개발·데이터 수집 전용이다.
- `asmc_bridge/bridge.launch`가 Network UDP, Sensor UDP, VehicleState node를 한 번에 기동한다.
  P3-F부터 세 구현과 component launch도 모두 `asmc_bridge` 패키지 안에 있다.
- 두 simulator release는 `morai_udp` runtime 하나를 쓴다. release는 MORAI UI에 import할 JSON으로,
  transport는 `udp_bridge.sh`의 `local`/`lan2lan`으로 선택한다.
- stream policy는 `competition`(기본)과 `development`(로컬 oracle)로 나뉘며 release와 독립적이다.

## 3. Docker 기동

Windows는 Docker Desktop 4.34 이상에서 Linux containers, WSL Integration과
`Settings → Resources → Network → Enable host networking`을 켠 뒤 재시작한다.

```bash
cd "$ASMC"
xhost +local:docker
./scripts/docker_ros_up.sh build       # 최초 또는 image 의존성 변경
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh status      # mode=host, published={}
./scripts/docker_ros_up.sh check       # Windows↔container UDP 왕복
docker exec -it asmc-ros-noetic bash
```

| 환경 | 명령 | 기준 |
|---|---|---|
| RTX 40 기본 | `build`, `up` → `asmc/ros-noetic:dev` | Python 3.8, PyTorch 2.1.2+cu118 |
| RTX 50 opt-in | `build-rtx50`, `up-rtx50` → `asmc/ros-noetic:rtx50` | 추론 venv Python 3.9, PyTorch 2.7.1+cu128 |
| 학습 | `docker_perception_up.sh build/up` → `asmc/perception-train:dev` | `$ASMC_DATA` read-only |

코드만 바뀌면 image를 재빌드하지 않고 컨테이너에서 `./scripts/build_ws.sh`를 실행한다.
Dockerfile/requirements가 바뀌었을 때만 `rebuild`를 사용한다. `up`은 컨테이너를 재생성하므로
주행·녹화 중 실행하지 않는다.

종료할 때는 `./scripts/docker_ros_up.sh down`과 `xhost -local:docker`를 실행한다. 데이터는
저장소 밖 `$ASMC_DATA`를 ROS 컨테이너 `/data`에 rw, 학습 컨테이너에는 ro로 마운트한다.
미설정 시 스크립트는 저장소와 나란한 `data/`를 사용한다. TwinLiteNet+ upstream은
`external/baselines/TwinLiteNetPlus`에서 ROS runtime과 학습 컨테이너의
`/opt/baselines/TwinLiteNetPlus`로 모두 read-only 마운트하고 loader가 pinned commit을
검사한다. live와 bag replay ROS 추론은 `asmc-ros-noetic`, 모델 학습과 학습 중
validation은 `asmc-perception-train`에서 실행한다. 학습 컨테이너는 host ROS master와
분리한다. NVIDIA driver는 호스트에만 설치한다.

## 4. 현재 개발 UDP profile

MORAI Network/Sensor의 Host와 Destination IP는 모두 `127.0.0.1`, protocol은 UDP다.
Compose는 `network_mode: host`이고 개별 `ports:` forwarding을 사용하지 않는다.

### Network와 control

| 채널 | 현재 포트 | ROS/소비자 |
|---|---:|---|
| Ego Vehicle Status | Host/Dest `9088/9099` | `/Ego_topic` |
| Competition Vehicle Status | MolitComp Host/Dest `9797/9798`, type 787 | `/Competition_topic`; 26r1 export에는 없음 |
| TrafficLight status | Host/Dest `7603/7502`, type 258 | simulator oracle; canonical bridge가 소비하지 않음 |
| Object Info | Dest `7505` | `/Object_topic`, 임시 oracle |
| Collision Data | Host/Dest `9091/9092` | `/CollisionData` |
| Ego Ctrl Cmd | Host/Dest `9093/9094`, mode 2 | `asmc_bridge` network node가 `/ctrl_cmd`를 UDP로 전송 |

### 센서

| ID | 센서 | Host→Dest | ROS topic | 설정 Hz |
|---:|---|---|---|---:|
| 1 | front camera | `9290→9291` | `/image_jpeg/compressed` | 20 |
| 2 | left camera | `9292→9293` | `/cam_left/image_jpeg/compressed` | 20 |
| 3 | right camera | `9294→9295` | `/cam_right/image_jpeg/compressed` | 20 |
| 4 | extra camera | `9296→9297` | `/cam_extra/image_jpeg/compressed` | 10 |
| 5 | VLP16 Intensity | `9298→9299` | `/velodyne_points` | v3 15 (v2 10) |
| 6 | GPS | `9300→9301` | `/gps` | 5 |
| 7 | IMU | `9302→9303` | `/imu` | 50 |

GT 센서셋에서만 Semantic camera `9305/9307/9309`, Instance LiDAR `9311`을 사용한다. bridge는
이 optional port를 열 수 있지만 competition profile에서는 GT publisher와 profile을 시작하지
않는다. LiDAR destination은 `9299`이며 과거 `2368`을 쓰지 않는다.

한 장짜리 UI 입력표는 [UDP 치트시트](../../config/morai/udp-sim-cheat-sheet.md)를 사용한다.

## 5. 빌드·실행·상태 확인

컨테이너 안에서:

```bash
cd /root/ws
./scripts/build_ws.sh
source devel/setup.bash
./scripts/udp_bridge.sh
```

이 명령은 `/vehicle/state`까지 포함한다. 기존 stack이 vehicle-state node를 별도로 기동하는
일시적 호환 실행에서만 `enable_vehicle_state:=false`를 사용하고, 동일 topic의 publisher를
두 개 실행하지 않는다.

26r1과 대회판 모두 같은 `morai_udp` launch를 사용한다. 현장에서는 별도로
`ASMC_network_set_molitcomp03_lan2lan.json`을 MORAI에 로드해야 하며, 실제 LAN gate 전에는
이를 현장 설정 완료로 간주하지 않는다.

현장 후보 명령은 명시적 transport를 함께 쓴다.

```bash
./scripts/udp_bridge.sh --transport lan2lan
```

수신 port와 bridge 구현은 두 transport가 같다. transport 인자는 Ctrl 송신 IP를 local
`127.0.0.1` 또는 현장 `192.168.0.1`로 안전하게 확정한다. sender IP 기반 자동 판정은 첫 수신
전에는 결정할 수 없고 WSL mirrored/NAT·다중 NIC에서 오판할 수 있어 제공하지 않는다.

기본 명령은 `local + competition`이다. Object/Semantic/Instance가 필요한 개발 실험만
`./scripts/udp_bridge.sh --gt`로 실행한다. development와 lan2lan을 결합하면
스크립트와 두 bridge node가 모두 기동을 거부한다.

다른 터미널에서:

```bash
./scripts/docker_ros_up.sh verify 8 molitcomp03
./scripts/docker_ros_up.sh verify 8 26r1
# 실제 CompetitionInfoPublisher9099 /Ego_topic LAN 수락:
./scripts/docker_ros_up.sh verify-field 30 molitcomp03

rostopic hz /Ego_topic
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /gps
rostopic hz /imu
rostopic echo /Ego_topic -n1
```

`verify`는 RGB 4대, LiDAR, GPS, IMU, Ego, 유효 `/vehicle/state`와 privileged publisher
부재를 확인한다. 현재 정상 Competition은 `/Ego_topic` 경로다. 정정된 `verify-field`는
LAN의9099 `/Ego_topic`을 검사하며 `/Competition_topic` Detail publisher는 거부한다.
release 인자는 provenance label이며 `verify-field` 수락 범위는 확인된 `molitcomp03`으로 제한한다.
GT 실험 재현에서만 `--gt`와 `verify-gt 20 molitcomp03`을 함께 사용한다.

LiDAR 실제 도착률만 별도로 검사할 때는 다음처럼 실행한다. 이 exact-arrival 진단은 v3의
설정·소비 15 Hz 요구와 다르며, 현재 MolitComp03에서는 약 12 Hz를 넣어 사용한다.

```bash
python3 scripts/verify_morai_topics.py \
  --duration 30 --required-topic /velodyne_points \
  --lidar-target-hz 12 --strict-thresholds
```

설정 Hz보다 낮은 관측값을 바로 packet loss로 단정하지 않는다. sequence가 없는 stream은
SIM 송신 cadence, Windows ingress, UDP, bridge와 ROS scheduling을 단일 rate만으로 분리할 수
없다. 날짜가 있는 기준선은 [transport 검증 기록](../records/bridge/runtime-transport-2026-09.md)에
보존한다.

## 6. 문제 해결

### LiDAR만 수신되지 않을 때

1. Docker Desktop host networking 적용 후 재시작했는지 확인한다.
2. `docker_ros_up.sh status`가 `mode=host`, `published={}`인지 확인한다.
3. MORAI LiDAR Connect와 `127.0.0.1:9299`를 확인한다.
4. `ss -lunp`로 다른 9299 bind가 없는지 확인한다.
5. bridge foreground의 `[SENSORS]` 요약 블록에서 `lidar=...Hz`를 확인한다. 콘솔은5초, diagnostics는1초 주기다.
6. `rotation_hz`와 PointCloud의 `interval_hz`를 비교한다. 두 값 모두 약12Hz이고
   `socket_drops=0`이면 ROS 변환 이전의 입력 cadence를 먼저 조사한다. `raw_packet_hz`는
   최근 구간, `packets_per_frame_median`은 최대4096프레임이므로 두 값을 같은 창으로 단정하지 않는다.
   v3 local P3-E 표본은 약 `600 packet/s`, `50 packet/frame`, `12 Hz`였다.
7. 최초 미수신 `WAITING`, 수신 중단 `STALE`, 잘못된 입력 `INVALID`, 수신기 실패 `ERROR`를
   구분한다. Local 선택 Competition의 알려진 형식은1회 INFO와 `UNSUPPORTED` 진단으로
   분리하며 정상 decode로 세지 않는다. 현재 설정과 로그 규칙은 [bridge README](../../src/asmc_bridge/README.md)를 따른다.

### ROS master 충돌

레거시 `aim-noetic-pc`와 `asmc-ros-noetic`을 동시에 실행하지 않는다.

```bash
docker exec -it asmc-ros-noetic bash
pkill -9 -f rosmaster
pkill -9 -f roscore
```

### RViz/rqt가 열리지 않을 때

```bash
xhost +local:docker
asmc-ros-gui rqt-image /perception/bev/debug/shadow_map_rgb
asmc-ros-gui rviz -d /root/ws/<config>.rviz
```

`DISPLAY`가 `:0` 또는 `:1`인지 확인한다. GUI helper는 Qt window state와 image transport를
정규화하며 지정한 `.rviz` 파일은 수정하지 않는다. image는 `XDG_RUNTIME_DIR`와
`QT_X11_NO_MITSHM=1`, 2 GiB shared memory를 제공한다.

## 7. 변경 범위별 시뮬 검증

| 범위 | 최소 gate |
|---|---|
| 공통 | submodule `45c6baf`, `build_ws.sh`, host network check, 핵심 sensor topic |
| bridge | packet parse error 0, topic rate/staleness, Ctrl 목적지·응답 |
| HD Map | global/local topic, RViz, ego crop, geometry/checkpoint 회귀 |
| Behavior/Frenet/MPC | catkin build, v2 topic transaction, trajectory provenance·fail-safe, Ctrl tracking |
| camera | frozen checkpoint, 3-view output, timestamp skew, FPS/latency |
| LiDAR/fusion | VLP16 topic, detector/tracker output, camera sync와 resource |
| scenario/gRPC | 개발 profile에서만 scenario 시작·종료·NPC spawn |
| full stack | 모든 node 동시 기동, 15분 이상 주행, restart recovery |

압축 전 체크리스트에 있던 판정 가능한 세부 기준 중 현재도 유효한 항목은 다음과 같다.

- Behavior/Frenet/MPC: route·GPS/IMU 유효성, command ID와 feedback lifecycle,
  trajectory source time·연속성, `/ctrl_cmd` 단일 publisher·목적지, 급커브·저속 구간
  이탈/조향 진동과 `Ctrl+C` 뒤 node·bridge 종료를 확인한다.
- camera: checkpoint/model source SHA, front·left·right 출력, view 간 timestamp skew,
  stale/drop, FPS, p50/p95 latency와 VRAM을 기록한다.
- LiDAR/fusion: detector/tracker 출력, camera sync 허용값과 CPU/GPU 사용량을 함께 기록한다.
- scenario/gRPC: 개발 profile에서 route drive, NPC spawn과 정상 종료를 확인한다. gRPC host와
  route 시각화는 본선 profile의 gate가 아니다.

검증 결과에는 환경, MORAI/profile, Git commit, 실행 명령, PASS/FAIL, 로그와 미검증 항목을
남긴다. LBC teacher BEV는 privileged-state 연구·검수 도구이고 direct rosbridge는
회귀 재현용일 뿐, 둘 다 현재 full-stack gate가 아니다.

```markdown
## Sim verify — <파트> — <이름> — <날짜>

- 환경: OS/Docker, MORAI·sensor/network profile, map/scenario
- commit과 실행 명령:
- 범위와 결과: PASS / FAIL
- rate·latency·자원·로그:
- 미검증 항목과 이유:
```

## 8. 현장 전환 gate

현재 구현된 profile을 실제 현장에 적용할 때 다음을 함께 검증한다.

1. `local`과 `lan2lan`의 port·channel·Ctrl mode가 같고 IP만 다른지 확인한다.
2. Ctrl `9093/9094`, Ego Vehicle Status `9088/9099`, MolitComp Competition Vehicle Status
   `9797/9798`, Collision `9091/9092` packet 방향을 UI와 capture로 확인한다. `7603/7502`는
   TrafficLight status이며 Competition Vehicle Status로 해석하지 않는다.
3. MORAI UI의 VLP16 15 Hz `sensorPeriod`·`frequencyIndex`와 손재호 소비 loop 15 Hz를 유지한다.
4. `/velodyne_points`의 실제 완성 scan률(현재 약 12 Hz)과 packet/parse 오류를 확인한다.
5. 손재호 tracking, latency와 CPU/GPU를 관찰하되 실제 scan을 중복 재발행하지 않는다.
6. Windows+WSL mirrored 또는 native Linux의 실제 두 PC에서 firewall, route, restart와 offline
   복구를 리허설한다.

Ctrl은 `9093/9094`로 전환됐고 Ego Vehicle Status는 두 release 모두 `9088/9099`로 통일됐다.
Ego Vehicle Status는 Competition Vehicle Status live capture·field 검증이 끝날 때까지 상태 입력 회귀 기준으로,
LiDAR v2 10 Hz는 과거 비교·회귀용으로 유지하고, 배포 센서 설정은 v3 15 Hz를 사용한다.
