# UDP ↔ ROS 네트워크 가이드

> **최종 수정:** 2026-08-04  
> **지위:** 팀 공식 권장  
> **개발 SIM (현재):** **MORAI 26.R1.h3** — 대회 배포 SIM 아님  
> **msg:** `src/MORAI-ROS_morai_msgs` submodule **`beta_drive`**  
> 구현: `src/morai_udp_bridge/` · `src/morai_sensor_bridge/` · `./scripts/udp_bridge.sh`  
> 규정: [로컬 03-vehicle-sensors-network](../../docs/competition/regulations/03-vehicle-sensors-network.md)

---

## 0. 버전 · 용어 (필독)

| 항목 | 내용 |
|------|------|
| **일상 개발** | **26.R1.h3** + 본 브릿지 + `beta_drive` |
| **대회 당일/게이트** | 대회 MORAI · 허용 **UDP만** (토픽·기능 제약 많음 → 별도 재검증) |
| **UDP 브릿지** | `./scripts/udp_bridge.sh` — 바이너리 UDP ↔ ROS |
| **rosbridge** | `./scripts/rosbridge.sh` — WebSocket · **개발 편의 · 당일 금지** |

이 문서의 포트/토픽은 **26.R1.h3 개발 스택** 기준이다. 대회 SIM에 그대로 꽂기 전에 패킷·포트를 다시 맞춰야 한다.

---

## 1. 규정 요약 (당일)

시뮬↔팀 PC는 **허용 UDP만**. 내부 ROS는 실격 대상이 아님.  
당일 **rosbridge로 시뮬 연결 금지**.

---

## 2. 아키텍처

```
MORAI (Windows)  --UDP-->  morai_udp_bridge / morai_sensor_bridge  --> ROS topics
                              ^
/ctrl_cmd ---------------------+  (UDP Ctrl → SIM)
```

---

## 3. Windows + WSL + Docker: IP를 `127.0.0.1`로 쓰는 방법

### 왜 개인 Windows/WSL IP를 안 넣나?

Docker Desktop의 `network_mode: host`는 **WSL eth0과 다른 netns**라, Dest를 WSL IP로 넣어도 컨테이너에 UDP가 안 들어온다.

### 팀 표준 (권장)

`docker/ros-noetic/docker-compose.yaml`이 **UDP Destination 포트를 publish**한다.  
Windows localhost → Docker 포워딩 → 컨테이너.

| MORAI 필드 (Network · Sensor 공통) | 값 |
|--------------------------------------|-----|
| **Host IP** | `127.0.0.1` |
| **Destination IP** | `127.0.0.1` |

| 브릿지 (컨테이너) | 값 |
|-------------------|-----|
| Ctrl 송신 `cmd_ip` | `host.docker.internal` (`udp_bridge.sh` 기본) |

**각자 PC에서 할 일:** Docker Desktop + WSL Integration ON → compose `up` → 시뮬에 위 IP만 넣으면 됨.  
이더넷/WSL IP(`172.24.x` 등)를 일일이 맞출 필요 없음.

```bash
cd "$ASMC"
xhost +local:docker
./scripts/docker_ros_up.sh up          # 최초: build
docker exec -it asmc-ros-noetic bash
./scripts/build_ws.sh                  # 최초·패키지 추가 시
source devel/setup.bash
./scripts/udp_bridge.sh
```

포트 목록을 본인 UI와 다르게 쓰면: launch arg로 덮거나 `docker-compose.yaml`의 `ports:` / 센서 `cameras` rosparam을 맞춘 뒤 `docker_ros_up.sh up`으로 재생성.

---

## 4. 시뮬레이터에 적을 IP · 포트 (치트시트)

한 장으로 복사: [config/morai_udp_sim_cheat_sheet.md](../config/morai_udp_sim_cheat_sheet.md)

### 4.1 공통 IP

```
Host IP:        127.0.0.1
Destination IP: 127.0.0.1
Protocol:       UDP
```

### 4.2 Cmd Control (팀 표준)

| 항목 | 값 | 비고 |
|------|-----|------|
| 타입 | **Ego Ctrl Cmd** | TrajectoryTrackingCmd 아님 |
| Host PORT | **9111** | 브릿지 기본 `cmd_port` |
| Dest PORT | **9112** | 페어 |

팀원이 서로 다른 숫자(9093/9094, 9097/9098 …)를 쓰던 것은 **각자 UI에서 임의 배정**한 결과다.  
PC마다 MORAI를 단독 실행하므로 포트 숫자가 같아도 **충돌하지 않는다** — 문서·launch를 맞추려면 **9111/9112로 통일**한다.

예외로 다른 Host를 쓸 때만:

```bash
./scripts/udp_bridge.sh cmd_port:=<내_Host_PORT>
```

### 4.3 Network → 브릿지 listen (Destination PORT)

레포 기본값 (launch / compose와 동기):

| 채널 | Dest PORT | ROS |
|------|-----------|-----|
| Ego Vehicle Status | **909** | `/Ego_topic` |
| Object Info | **7505** | `/Object_topic` (oracle) |
| Collision Data | **9092** | `/CollisionData` |

### 4.4 Sensor → Destination PORT

| 센서 | Dest | ROS 토픽 |
|------|------|----------|
| camera-1 | **9291** | `/image_jpeg/compressed` |
| camera-2 | **9293** | `/cam_left/image_jpeg/compressed` |
| camera-3 | **9295** | `/cam_right/image_jpeg/compressed` |
| camera-4 | **9297** | `/cam_extra/image_jpeg/compressed` |
| 3D LiDAR | **9299** | `/velodyne_points` (Host **9298**) |
| GPS | **9301** | `/gps` |
| IMU | **9303** | `/imu` |

Host PORT는 Dest−1 패턴(9290/9291 …) 권장. cam1–3이 주최 고정이면 그 값을 유지하고 cam4·GPS·IMU만 이어 배정.

---

## 5. 패키지 · msg

| 패키지 | 역할 |
|--------|------|
| `morai_udp_bridge` | Ego/Object/Collision IN · `/ctrl_cmd` OUT |
| `morai_sensor_bridge` | Cam×4 · LiDAR(VLP-16) · GPS · IMU |

- Ego: UDP → **`wheel_angle`** (beta_drive; va의 `front_steer_angle` 아님)  
- Ctrl: **`steering`** (rad) → UDP 정규화  
- Object/Collision은 학습·eval용. **배포 추론 경로 privileged 금지**

---

## 6. 검증

```bash
rostopic hz /Ego_topic
rostopic hz /image_jpeg/compressed
rostopic hz /cam_left/image_jpeg/compressed
rostopic hz /cam_right/image_jpeg/compressed
rostopic hz /cam_extra/image_jpeg/compressed
rostopic hz /gps
rostopic hz /imu
rostopic hz /velodyne_points
rostopic echo /Ego_topic -n1   # wheel_angle, position, velocity
```

참고 실측(26 + AIM06): Ego ~47 · cam1–3 ~20 · cam4 ~10 · gps ~5 · imu ~50 · lidar `/velodyne_points` ~7 (SIM 10 Hz·Docker UDP).

센서셋 JSON:
- 주행: [`config/ASMC_sensor_set_v1.json`](../config/ASMC_sensor_set_v1.json) (Camera×4 · LiDAR · GPS · IMU · Network/Cmd는 별도)
- GT 수집: [`config/ASMC_sensor_set_GT.json`](../config/ASMC_sensor_set_GT.json) (위 + Semantic Camera×3 Dest **9305/9307/9090** · 브릿지 수신 미구현, 후속)

LiDAR TF (`base_link`→`velodyne`, v1 기준): **x=0.80 y=0.0 z=1.45**

---

## 7. FAQ (요약)

**Q. rosbridge랑 뭐가 다르나?**  
A. rosbridge=`rosbridge.sh`(WebSocket). 대회 경로=`udp_bridge.sh`(UDP).

**Q. 전부 UDP가 더 빠르지 않나?**  
A. 제어만 보면 홉이 줄 수 있으나 6인 인터페이스·디버깅 비용이 큼. 경계만 UDP.

**Q. 대회 SIM에서도 이 포트 그대로?**  
A. **보장 안 됨.** 26 개발용 기준. 대회 빌드에서 Dest/패킷 재확인.

---

## 8. 관련

- [getting-started.md](./getting-started.md) · [docker-guide.md](./docker-guide.md) · [morai-msgs-versions.md](./morai-msgs-versions.md)  
- [config/morai_udp_sim_cheat_sheet.md](../config/morai_udp_sim_cheat_sheet.md)  
- 로컬 상세: [udp-bridge-porting.md](../../docs/competition/udp-bridge-porting.md)
