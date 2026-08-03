# MORAI UDP — 시뮬 UI 치트시트 (26.R1.h3 개발용)

> **개발 SIM: 26.R1.h3** (대회 배포 SIM 아님)  
> Docker Desktop + `./scripts/docker_ros_up.sh` + `./scripts/udp_bridge.sh` 기준.  
> 상세: [docs/network-udp-ros.md](../docs/network-udp-ros.md)  
> 차량 센서 프리셋: [`ASMC_v1_vehicle_sensors.json`](./ASMC_v1_vehicle_sensors.json) (MORAI에 Import)

---

## 팀원이 할 일 (체크리스트)

개인 Windows/WSL IP를 Docker·compose에 넣을 필요 **없음**.

1. Docker Desktop + WSL Integration ON → `./scripts/docker_ros_up.sh up`
2. MORAI **Host/Dest IP 전부 `127.0.0.1`**
3. 아래 **Network / Sensor Dest 포트**를 치트시트와 동일하게 (또는 `ASMC_v1_vehicle_sensors.json` 로드)
4. **Cmd Control** 타입 = **Ego Ctrl Cmd**, Host/Dest = **9111 / 9112** (팀 표준)
5. 컨테이너: `source devel/setup.bash && ./scripts/udp_bridge.sh`
6. 확인: `rostopic hz /Ego_topic` · `/velodyne_points` · `/image_jpeg/compressed`

---

## IP (Network · Sensor · Cmd 공통)

```
Host IP:          127.0.0.1
Destination IP:   127.0.0.1
```

---

## Cmd Control (팀 표준 — 개인 포트 쓰지 말 것)

서로 다른 PC에서 MORAI를 켜도 **같은 숫자**를 쓴다.  
(예전에 9093/9094, 9097/9098, 9111/9112가 섞인 것은 각자 UI에서 임의 배정했기 때문. SIM끼리 충돌하지 않음 — 각자 localhost.)

```
타입:     Ego Ctrl Cmd          ← TrajectoryTrackingCmd 금지
Host:     9111                  ← 브릿지 기본 cmd_port
Dest:     9112
```

다른 값을 이미 쓰고 있으면 시뮬을 **9111/9112로 맞추거나**:

```bash
./scripts/udp_bridge.sh cmd_port:=<내_Host>
```

---

## Network Settings — Destination PORT (브릿지가 listen)

| 채널 | Dest | (참고 Host) |
|------|------|-------------|
| Ego Vehicle Status | **909** | 908 |
| Object Info | **7505** | 7605 |
| Collision Data | **9092** | 9091 |

---

## Sensor Setting — Destination PORT

| 센서 | Dest | ROS 토픽 |
|------|------|----------|
| camera-1 | **9291** | `/image_jpeg/compressed` |
| camera-2 | **9293** | `/cam_left/image_jpeg/compressed` |
| camera-3 | **9295** | `/cam_right/image_jpeg/compressed` |
| camera-4 | **9297** | `/cam_extra/image_jpeg/compressed` |
| 3D LiDAR | **9299** | `/velodyne_points` (Host **9298**) |
| GPS | **9301** | `/gps` (Host **9300**) |
| IMU | **9303** | `/imu` (Host **9302**) |

권장 Host = Dest − 1. 순서: cam1–4 → lidar → gps → imu.  
**LiDAR는 Velodyne 기본 2368/2369 쓰지 말 것** (Docker Desktop 미개방 · Host 충돌).

---

## 실행 한 줄

```bash
./scripts/docker_ros_up.sh up && docker exec -it asmc-ros-noetic bash
# 컨테이너 안
source devel/setup.bash && ./scripts/udp_bridge.sh
```
