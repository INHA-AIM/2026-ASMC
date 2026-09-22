# MORAI UDP — 시뮬 UI 치트시트

> **공통 런타임:** `morai_udp` — 26.R1.H3와 25.S4.MolitComp03가 같은 bridge를 사용한다.
> Docker Desktop + `./scripts/docker_ros_up.sh` + `./scripts/udp_bridge.sh --simulator 26r1` 기준.
> 상세: [개발·시뮬레이터 운영 가이드](../../docs/operations/README.md)
> bridge 구현·통합 gate: [MORAI bridge guide](../../docs/packages/asmc_bridge/README.md)
> 센서셋: [`ASMC_sensor_set_v2.json`](./sensors/ASMC_sensor_set_v2.json) (현재 10 Hz 회귀 기준)<br>
> 15 Hz 후보: [`ASMC_sensor_set_v3.json`](./sensors/ASMC_sensor_set_v3.json)<br>
> GT용: [`ASMC_sensor_set_GT.json`](./sensors/ASMC_sensor_set_GT.json) (v2 + Semantic Camera×3 + Instance LiDAR)

---

## 팀원이 할 일 (체크리스트)

Docker·Dockerfile에 **개인 IP를 넣을 필요 없음** (Host/Dest IP = `127.0.0.1`).

1. Docker Desktop + WSL Integration ON → `./scripts/docker_ros_up.sh up`
2. MORAI **Host/Dest IP 전부 `127.0.0.1`**
3. **센서셋** `ASMC_sensor_set_v2.json` Import (또는 15 Hz 시험용 v3, 개발 GT용 GT)
4. **Network Settings**: `network/generated/ASMC_network_set_26r1_dev_local.json` Import
   (현재 bridge 호환 Ego **909** · Object **7505** · Collision **9092** 포함)
5. **Cmd Control**: 타입 **Ego Ctrl Cmd**, Host/Dest **9093 / 9094**, mode **2**
6. 컨테이너: `source devel/setup.bash && ./scripts/udp_bridge.sh --simulator 26r1`
7. 확인: `rostopic hz /Ego_topic` · `/velodyne_points` · `/image_jpeg/compressed`

**LiDAR만 0이고 cam/gps/imu는 될 때:** `docker_ros_up.sh status`의 host mode, MORAI
Connect/Dest 9299와 다른 9299 listener를 순서대로 확인한다. 개별 Docker port forwarding은
사용하지 않는다. **9299를 다른 포트로 옮기지 말 것.** 상세:
[운영 가이드 §6](../../docs/operations/README.md#6-문제-해결)

`--simulator`는 release별 MORAI import 파일과 provenance를 고르며 bridge 구현은 바꾸지 않는다.
아래 표는 `dev_local`이다. 현장 파일은 [MORAI config 안내](./README.md)에서 별도로 고른다.

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
| Ego Vehicle Status | **909** | 908 |
| Object Info | **7505** | 7605 |
| Collision Data | **9092** | 9091 |

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

브릿지는 v2·GT 포트 합집합을 항상 연다. v2에서는 아래 포트가 대기하고,
GT 센서셋을 로드하면 별도 옵션 없이 자동 수신한다.

```bash
./scripts/udp_bridge.sh --simulator 26r1
```

| 센서 | Host / Dest | ROS 토픽 |
|------|-------------|----------|
| Semantic front (Camera-8) | 9304 / **9305** | `/sem_front/image_jpeg/compressed` |
| Semantic left (Camera-9) | 9306 / **9307** | `/sem_left/image_jpeg/compressed` |
| Semantic right (Camera-10) | **9308 / 9309** | `/sem_right/image_jpeg/compressed` |

Camera-10은 예전 **9090**이 아님 (rosbridge와 충돌).

## GT Instance LiDAR (ASMC_sensor_set_GT.json)

브릿지는 ID 5와 선택 GT ID 11 포트를 항상 독립적으로 연다.
v2에서 ID 11은 대기하고 GT에서 자동 수신한다. 정합용 source stamp가 필요하면:

```bash
./scripts/udp_bridge.sh --simulator 26r1 timestamp_mode:=source
```

| 센서 | Host / Dest | ROS 토픽 | field |
|---|---|---|---|
| Instance LiDAR ID 11 | 9310 / **9311** | `/velodyne_points_instance` | `instance_id uint32` |

확인: `./scripts/docker_ros_up.sh verify-gt 20`. Instance/Object/Semantic은 개발 oracle이며
대회 launch와 일반 raw bag에 포함하지 않는다.

---

## 실행 한 줄

```bash
./scripts/docker_ros_up.sh up && docker exec -it asmc-ros-noetic bash
# 컨테이너 안
source devel/setup.bash && ./scripts/udp_bridge.sh --simulator 26r1
```
