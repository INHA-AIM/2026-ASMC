# MORAI UDP — 시뮬 UI 치트시트

> **공통 런타임:** `morai_udp` — 26.R1.H3와 25.S4.MolitComp03가 같은 bridge를 사용한다.
> Docker Desktop + `./scripts/docker_ros_up.sh` + `./scripts/udp_bridge.sh` 기준.
> 상세: [개발·시뮬레이터 운영 가이드](../../docs/operations/README.md)
> bridge 구현·통합 gate: [MORAI bridge guide](../../docs/packages/asmc_bridge/README.md)
> 본선 센서셋: [`ASMC_sensor_set_v3.json`](./sensors/ASMC_sensor_set_v3.json) — 설정·소비 15 Hz, 완성 scan 약 12 Hz 허용<br>
> 회귀 센서셋: [`ASMC_sensor_set_v2.json`](./sensors/ASMC_sensor_set_v2.json) — 10 Hz 비교용<br>
> GT용: [`ASMC_sensor_set_GT.json`](./sensors/ASMC_sensor_set_GT.json) (v3 15 Hz + Semantic Camera×3 + Semantic/Instance LiDAR)

---

## 팀원이 할 일 (체크리스트)

Docker·Dockerfile에 **개인 IP를 넣을 필요 없음** (Host/Dest IP = `127.0.0.1`).

1. Docker Desktop + WSL Integration ON → `./scripts/docker_ros_up.sh up`
2. MORAI **Host/Dest IP 전부 `127.0.0.1`**
3. **센서셋** `sensors/generated/ASMC_sensor_set_v3_local.json` Import
   (10 Hz 과거 회귀가 필요할 때만 `...v2_local.json`)
4. **Network Settings**: `network/generated/ASMC_network_set_26r1_local.json` Import
   (현재 bridge 호환 Ego **9099** · Object **7505** · Collision **9092** 포함)
5. **Cmd Control**: 타입 **Ego Ctrl Cmd**, Host/Dest **9093 / 9094**, mode **2**
6. 컨테이너: `source devel/setup.bash && ./scripts/udp_bridge.sh`
7. 확인: `rostopic hz /Ego_topic` · `/velodyne_points` · `/vehicle/state`

26.R1에는 `/Competition_topic`의 원천인 type 787이 없고, 참가팀 MolitComp03 local은 JSON에
type 787 항목이 있어도 packet을 발행하지 않았다. 두 release의 local 검사는
`./scripts/docker_ros_up.sh verify 8 <release>`를 사용한다. 주최 현장판 type 787 수락에서만
`./scripts/docker_ros_up.sh verify-field 30 molitcomp03`을 사용한다. release 인자는 provenance
label이며 bridge 동작이나 필수 토픽을 자동 변경하지 않는다.

**LiDAR만 0이고 cam/gps/imu는 될 때:** `docker_ros_up.sh status`의 host mode, MORAI
Connect/Dest 9299와 다른 9299 listener를 순서대로 확인한다. 개별 Docker port forwarding은
사용하지 않는다. **9299를 다른 포트로 옮기지 말 것.** 상세:
[운영 가이드 §6](../../docs/operations/README.md#6-문제-해결)

bridge 명령에는 simulator 선택이 없다. 26r1과 대회판은 같은 runtime을 쓰고, release 차이는
MORAI UI에 import하는 network JSON에서만 선택한다. 아래 표는 `local`이다. `lan2lan` 파일은
[MORAI config 안내](./README.md)에서 별도로 고른다.

---

## IP (Network · Sensor · Cmd 공통)

```
Host IP:          127.0.0.1
Destination IP:   127.0.0.1
```

---

## Cmd Control (팀 표준)

```
타입:     Ego Ctrl Cmd          ← TrajectoryTrackingCmd 금지
Host:     9093
Dest:     9094
Mode:     2
```

---

## Network Settings — Destination PORT

| 채널 | Dest | (참고 Host) |
|------|------|-------------|
| Ego Vehicle Status | **9099** | 9088 |
| Competition Vehicle Status (MolitComp only) | **9798** | 9797 |
| TrafficLight status oracle | **7502** | 7603 |
| Object Info | **7505** | 7605 |
| Collision Data | **9092** | 9091 |

TrafficLight status(type 258)는 simulator oracle이며 `GetTrafficLightStatus` 조회 서비스와 다른
채널이다. 조사한 MolitComp03/26.R1 export에는 type 778 MoraiTLServiceProvider가 없으므로
GetTrafficLight를 필수 채널로 가정하거나 본선 planning 입력으로 사용하지 않는다.

---

## Sensor Setting — Destination PORT (v2)

| 센서 | Host / Dest | ROS 토픽 |
|------|-------------|----------|
| camera-1 | 9290 / **9291** | `/image_jpeg/compressed` |
| camera-2 | 9292 / **9293** | `/cam_left/image_jpeg/compressed` |
| camera-3 | 9294 / **9295** | `/cam_right/image_jpeg/compressed` |
| camera-4 | 9296 / **9297** | `/cam_extra/image_jpeg/compressed` |
| 3D LiDAR | 9298 / **9299** | `/velodyne_points` |
| GPS | 9300 / **9301** | `/gps` |
| IMU | 9302 / **9303** | `/imu` |

권장 Host = Dest − 1. 순서: cam1–4 → lidar → gps → imu.  
**LiDAR Velodyne 기본 2368/2369 쓰지 말 것.**

---

## GT Semantic Camera (ASMC_sensor_set_GT.json)

기본 competition profile은 아래 포트나 publisher를 열지 않는다. GT 센서셋은
local 개발 profile을 명시해야 수신한다.

```bash
./scripts/udp_bridge.sh --streams development
```

| 센서 | Host / Dest | ROS 토픽 |
|------|-------------|----------|
| Semantic front (Camera-8) | 9304 / **9305** | `/sem_front/image_jpeg/compressed` |
| Semantic left (Camera-9) | 9306 / **9307** | `/sem_left/image_jpeg/compressed` |
| Semantic right (Camera-10) | **9308 / 9309** | `/sem_right/image_jpeg/compressed` |

Camera-10은 예전 **9090**이 아님 (rosbridge와 충돌).

## GT Semantic/Instance LiDAR (ASMC_sensor_set_GT.json)

ID 11·12는 local development profile에서만 독립 수신한다. 정합용 source stamp가 필요하면:

```bash
./scripts/udp_bridge.sh --streams development timestamp_mode:=source
```

| 센서 | Host / Dest | ROS 토픽 | field |
|---|---|---|---|
| Intensity LiDAR ID 5 | 9298 / **9299** | `/velodyne_points` | `intensity float32` |
| Semantic LiDAR ID 11 | 9310 / **9311** | `/velodyne_points_semantic` | `semantic_id uint32` |
| Instance LiDAR ID 12 | 9312 / **9313** | `/velodyne_points_instance` | `instance_id uint32` |

확인: `./scripts/docker_ros_up.sh verify-gt 20`. 세 LiDAR의 실효 주기·source stamp·PointCloud2 schema를 검사한다. 동일 장착 위치가 점 인덱스의 일대일 대응을 보장하지는 않는다. Instance/Object/Semantic은 개발 oracle이며
대회 launch와 일반 raw bag에 포함하지 않는다.

---

## 실행 한 줄

```bash
./scripts/docker_ros_up.sh up && docker exec -it asmc-ros-noetic bash
# 컨테이너 안
source devel/setup.bash && ./scripts/udp_bridge.sh
```
