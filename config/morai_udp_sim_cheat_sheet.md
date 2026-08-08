# MORAI UDP — 시뮬 UI 치트시트 (26.R1.h3 개발용)

> **개발 SIM: 26.R1.h3** (대회 배포 SIM 아님)  
> Docker Desktop + `./scripts/docker_ros_up.sh` + `./scripts/udp_bridge.sh` 기준.  
> 상세: [docs/network-udp-ros.md](../docs/network-udp-ros.md)  
> 센서셋: [`ASMC_sensor_set_v1.json`](./ASMC_sensor_set_v1.json) (주행용 Camera×4/LiDAR/GPS/IMU)  
> GT용: [`ASMC_sensor_set_GT.json`](./ASMC_sensor_set_GT.json) (v1 + Semantic Camera×3 · 브릿지 기본 **OFF**)

---

## 팀원이 할 일 (체크리스트)

Docker·Dockerfile에 **개인 IP를 넣을 필요 없음** (Host/Dest IP = `127.0.0.1`).

1. Docker Desktop + WSL Integration ON → `./scripts/docker_ros_up.sh up`
2. MORAI **Host/Dest IP 전부 `127.0.0.1`**
3. **센서셋** `ASMC_sensor_set_v1.json` Import (또는 GT용 `ASMC_sensor_set_GT.json`)
4. **Network Settings** (센서셋에 없음): Ego **909** · Object **7505** · Collision **9092**
5. **Cmd Control**: 타입 **Ego Ctrl Cmd**, Host/Dest **9111 / 9112**
6. 컨테이너: `source devel/setup.bash && ./scripts/udp_bridge.sh`
7. 확인: `rostopic hz /Ego_topic` · `/velodyne_points` · `/image_jpeg/compressed`

**LiDAR만 0이고 cam/gps/imu는 될 때:** Docker Desktop의 **9299** 포워딩이 끊긴 것.
컨테이너 쪽 9299에 리스너가 없는 동안 MORAI가 쏘면 ~30s 후 포워딩이 철거된다.

1. `./scripts/docker_ros_up.sh up` → **곧바로** 브릿지 실행 (늦게 올리면 그 사이에 또 죽는다)
2. 그래도 0이면 Docker Desktop 재시작 —
   `"/mnt/c/Program Files/Docker/Docker/resources/bin/docker.exe" desktop restart`
   (WSL 안의 `docker desktop restart` 는 실패하면서 성공처럼 보인다)

판단은 `docker_ros_up.sh`의 `[OK]`가 아니라 브릿지 로그 `rx ... lidar=` 카운터로 한다.
**9299를 다른 포트로 옮기지 말 것.** 상세: [docs/network-udp-ros.md §6.1](../docs/network-udp-ros.md#61-lidar만-0일-때-9299-포워딩)

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
Host:     9111
Dest:     9112
```

---

## Network Settings — Destination PORT

| 채널 | Dest | (참고 Host) |
|------|------|-------------|
| Ego Vehicle Status | **909** | 908 |
| Object Info | **7505** | 7605 |
| Collision Data | **9092** | 9091 |

---

## Sensor Setting — Destination PORT (v1)

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

팀 공용 브릿지 기본은 **OFF** (`enable_semantic_camera:=false`).  
개발 시 GT 센서셋 Import 후:

```bash
./scripts/udp_bridge.sh enable_semantic_camera:=true
```

| 센서 | Host / Dest | ROS 토픽 |
|------|-------------|----------|
| Semantic front (Camera-8) | 9304 / **9305** | `/sem_front/image_jpeg/compressed` |
| Semantic left (Camera-9) | 9306 / **9307** | `/sem_left/image_jpeg/compressed` |
| Semantic right (Camera-10) | **9308 / 9309** | `/sem_right/image_jpeg/compressed` |

Camera-10은 예전 **9090**이 아님 (rosbridge와 충돌).

---

## 실행 한 줄

```bash
./scripts/docker_ros_up.sh up && docker exec -it asmc-ros-noetic bash
# 컨테이너 안
source devel/setup.bash && ./scripts/udp_bridge.sh
```
