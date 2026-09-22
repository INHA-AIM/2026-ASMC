# 개발·시뮬레이터 운영 가이드

> **문서 역할:** Canonical — Docker, MORAI UDP profile, bridge 기동과 시뮬 검증
>
> **현재 기준:** 26.R1.H3 + `ASMC_sensor_set_v2.json` + ROS1 Noetic + Docker host network

대회 허용 범위는 [대회 환경](../competition/README.md), Planning/Control의 독립 실행은
[Planning/Control 규약](./planning-control.md), bag 기록·재생은
[rosbag 가이드](./rosbag-evaluation.md)를 따른다. parser·timestamp·TF·`vehicle_state`를
수정하거나 Phase 3 이관을 준비할 때는 [MORAI bridge guide](../packages/asmc_bridge/README.md)를
함께 본다.

## 1. 현재값과 승인된 후속 목표

아래 두 열을 섞지 않는다. 3단계 `asmc_bridge` 전까지 launch와 MORAI UI에는 **현재 적용값**을
사용한다.

| 항목 | 현재 적용값 | 3단계 승인 목표 |
|---|---|---|
| 개발 IP | Host/Destination `127.0.0.1` | 동일 |
| 현장 IP | 아직 공용 profile 없음 | MORAI `192.168.0.1`, Autonav `192.168.0.10` |
| Ego Ctrl Cmd | `9111/9112`, type 1, mode 2 | `9093/9094`, type 1, mode 2 |
| 차량 상태 | Ego Vehicle Status destination `909` | Competition Vehicle Status `7603/7502` |
| CollisionData | destination `9092` | `9091/9092` pair |
| ObjectInfo | `7505`, 임시 개발 oracle | competition profile에서 제거 |
| 센서 | `9290~9303` | 같은 포트, IP만 profile별 변경 |
| VLP16 | 설정 10 Hz | UI 저장본 15 Hz, 실측 14~16 Hz |

네트워크와 LiDAR 목표는 승인됐지만 **아직 코드·센서셋에 적용하지 않았다**. Phase 3에서
`dev_local`과 `competition_lan` profile을 만들고 packet fixture, 30초 topic rate, 손재호
tracking 결과, CPU와 latency를 함께 비교한 뒤 전환한다.

## 2. 런타임 경계

```text
MORAI --UDP--> morai_sensor_bridge/morai_udp_bridge --> 팀 내부 ROS topic
MORAI <--UDP-- /ctrl_cmd adapter

MORAI GPS/IMU --UDP--> standalone Frenet --UDP :9300--> MPC --UDP--> MORAI
```

- MORAI와 팀 PC 사이에는 허용 UDP만 쓴다.
- ROS는 팀 PC 내부의 정규화·인지·디버깅 경계다.
- `rosbridge.sh` direct WebSocket 실험은 종료됐으며 현재 개발·본선 경로가 아니다.
- gRPC scenario runner와 GT topic은 개발·데이터 수집 전용이다.
- 현재 bridge는 `morai_udp_bridge`, `morai_sensor_bridge`, `vehicle_state`로 나뉘어 있다.
  단일 `asmc_bridge`는 Phase 3 parity 이후에만 교체한다.
- `udp_bridge.sh --simulator competition`은 현재 packet provenance/diagnostics 라벨만 바꾼다.
  현장 IP·port·privileged-stream allowlist를 적용하는 배포 profile이 아니다.

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
`external/baselines/TwinLiteNetPlus`에서 학습 컨테이너 `/opt/baselines/TwinLiteNetPlus`로
read-only 마운트하고 loader가 pinned commit을 검사한다. NVIDIA driver는 호스트에만 설치한다.

## 4. 현재 개발 UDP profile

MORAI Network/Sensor의 Host와 Destination IP는 모두 `127.0.0.1`, protocol은 UDP다.
Compose는 `network_mode: host`이고 개별 `ports:` forwarding을 사용하지 않는다.

### Network와 control

| 채널 | 현재 포트 | ROS/소비자 |
|---|---:|---|
| Ego Vehicle Status | Dest `909` | `/Ego_topic` |
| Object Info | Dest `7505` | `/Object_topic`, 임시 oracle |
| Collision Data | Host/Dest `9091/9092` | `/CollisionData` |
| Ego Ctrl Cmd | Host/Dest `9111/9112` | `/ctrl_cmd` 또는 standalone MPC |
| Planner → MPC | `9300` | standalone 내부 UDP |

### 센서

| ID | 센서 | Host→Dest | ROS topic | 설정 Hz |
|---:|---|---|---|---:|
| 1 | front camera | `9290→9291` | `/image_jpeg/compressed` | 20 |
| 2 | left camera | `9292→9293` | `/cam_left/image_jpeg/compressed` | 20 |
| 3 | right camera | `9294→9295` | `/cam_right/image_jpeg/compressed` | 20 |
| 4 | extra camera | `9296→9297` | `/cam_extra/image_jpeg/compressed` | 10 |
| 5 | VLP16 Intensity | `9298→9299` | `/velodyne_points` | 10 |
| 6 | GPS | `9300→9301` | `/gps` | 5 |
| 7 | IMU | `9302→9303` | `/imu` | 50 |

GT 센서셋에서만 Semantic camera `9305/9307/9309`, Instance LiDAR `9311`을 사용한다. bridge는
이 optional port를 열 수 있지만 competition profile에서는 GT publisher와 profile을 시작하지
않는다. LiDAR destination은 `9299`이며 과거 `2368`을 쓰지 않는다.

한 장짜리 UI 입력표는 [UDP 치트시트](../../config/morai_udp_sim_cheat_sheet.md)를 사용한다.

## 5. 빌드·실행·상태 확인

컨테이너 안에서:

```bash
cd /root/ws
./scripts/build_ws.sh
source devel/setup.bash
./scripts/udp_bridge.sh --simulator 26r1
```

현재 `--simulator competition`도 같은 launch 기본값을 사용한다. Phase 3의
`competition_lan` profile이 구현되기 전에는 이를 현장 설정 완료 명령으로 사용하지 않는다.

다른 터미널에서:

```bash
./scripts/docker_ros_up.sh verify 8

rostopic hz /Ego_topic
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /gps
rostopic hz /imu
rostopic echo /Ego_topic -n1
```

`verify`는 Ego, RGB 4대, LiDAR, GPS, IMU의 count/rate를 확인한다. GT 실험 재현에서만
`verify-gt 20`을 사용한다. v2에서 GT-only stream이 idle인 것은 오류가 아니다.

설정 Hz보다 낮은 관측값을 바로 packet loss로 단정하지 않는다. sequence가 없는 stream은
SIM 송신 cadence, Windows ingress, UDP, bridge와 ROS scheduling을 단일 rate만으로 분리할 수
없다. 날짜가 있는 기준선은 [transport 검증 기록](../records/simulator/runtime-transport-2026-09.md)에
보존한다.

## 6. 문제 해결

### LiDAR만 수신되지 않을 때

1. Docker Desktop host networking 적용 후 재시작했는지 확인한다.
2. `docker_ros_up.sh status`가 `mode=host`, `published={}`인지 확인한다.
3. MORAI LiDAR Connect와 `127.0.0.1:9299`를 확인한다.
4. `ss -lunp`로 다른 9299 bind가 없는지 확인한다.
5. bridge foreground의 `rx ... lidar=...`와 diagnostics를 확인한다.

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
| Frenet/MPC | standalone build, global path, GPS/IMU, `:9300`, Ctrl tracking |
| camera | frozen checkpoint, 3-view output, timestamp skew, FPS/latency |
| LiDAR/fusion | VLP16 topic, detector/tracker output, camera sync와 resource |
| scenario/gRPC | 개발 profile에서만 scenario 시작·종료·NPC spawn |
| full stack | 모든 node 동시 기동, 15분 이상 주행, restart recovery |

압축 전 체크리스트에 있던 판정 가능한 세부 기준 중 현재도 유효한 항목은 다음과 같다.

- Frenet/MPC: global path load, GPS/IMU 유효성, planner→MPC `:9300`, Ctrl 목적지,
  급커브·저속 구간 이탈/조향 진동과 `Ctrl+C` 뒤 process/socket 종료를 확인한다.
- camera: checkpoint/model source SHA, front·left·right 출력, view 간 timestamp skew,
  stale/drop, FPS, p50/p95 latency와 VRAM을 기록한다.
- LiDAR/fusion: detector/tracker 출력, camera sync 허용값과 CPU/GPU 사용량을 함께 기록한다.
- scenario/gRPC: 개발 profile에서 route drive, NPC spawn과 정상 종료를 확인한다. gRPC host와
  route 시각화는 본선 profile의 gate가 아니다.

검증 결과에는 환경, MORAI/profile, Git commit, 실행 명령, PASS/FAIL, 로그와 미검증 항목을
남긴다. LBC와 direct rosbridge는 회귀 재현용일 뿐 현재 full-stack gate가 아니다.

```markdown
## Sim verify — <파트> — <이름> — <날짜>

- 환경: OS/Docker, MORAI·sensor/network profile, map/scenario
- commit과 실행 명령:
- 범위와 결과: PASS / FAIL
- rate·latency·자원·로그:
- 미검증 항목과 이유:
```

## 8. 현장 전환 gate

Phase 3에서 현재값을 바꿀 때 다음을 한 변경으로 검증한다.

1. `dev_local`과 `competition_lan`의 port·channel·Ctrl mode가 같고 IP만 다른지 확인한다.
2. Ctrl `9093/9094`, Status `7603/7502`, Collision `9091/9092` packet 방향을 UI와 capture로
   확인한다.
3. MORAI UI에서 VLP16 15 Hz로 저장하고 `sensorPeriod`·`frequencyIndex`를 비교한다.
4. `/velodyne_points`를 30초 이상 측정해 평균 14~16 Hz와 packet/parse 오류를 확인한다.
5. 10 Hz baseline 대비 손재호 tracking, latency와 CPU/GPU를 비교한다.
6. native Linux의 실제 두 PC에서 firewall, route, restart와 offline 복구를 리허설한다.

전환 전에는 `9111/9112`, Ego `909`와 LiDAR 10 Hz를 현재 회귀 기준으로 유지한다.
