# UDP ↔ ROS 네트워크 가이드

> **문서 역할:** Canonical — 시뮬 UDP 경계, 포트, bridge와 Docker 네트워크 정본
> **문서 허브:** [README.md](./README.md)
> **최종 수정:** 2026-09-05
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

시뮬↔팀 PC는 **v1.1에 열거된 UDP만** 허용된다. 내부 ROS는 팀 PC 내부 처리에만 쓴다.
Ego Ctrl Cmd는 **cmd type 1 (accel/brake) + ctrl mode 2**로 고정하고, 당일
**rosbridge로 시뮬 연결하지 않는다**. 수신 상태는 Competition Vehicle Status의
자세·x방향 속도·각속도·제어 입력 범위에 맞춘다.

---

## 2. 아키텍처

Planning/Control 자체는 이 ROS bridge 구조를 사용하지 않는다. GPS/IMU UDP를 직접 수신하고 CtrlCmd UDP를 직접 송신하며, 상세는 [planning-control-udp.md](./planning-control-udp.md)를 따른다. 아래 구조는 ROS가 필요한 기타 모듈용이다.

```
MORAI (Windows)  --UDP-->  morai_udp_bridge / morai_sensor_bridge  --> ROS topics
                              ^
/ctrl_cmd ---------------------+  (UDP Ctrl → SIM)
```

---

## 3. Windows + WSL + Docker host networking

### 팀 표준

`docker/ros-noetic/docker-compose.yaml`은 Docker Desktop의 `network_mode: host`를 사용한다.
개별 `ports:` 포워딩을 두지 않으므로 브릿지를 늦게 켜거나 재시작해도 LiDAR `9299/udp`
포워딩만 사라지는 문제가 발생하지 않는다.

| MORAI 필드 (Network · Sensor 공통) | 값 |
|--------------------------------------|-----|
| **Host IP** | `127.0.0.1` |
| **Destination IP** | `127.0.0.1` |

| 브릿지 (컨테이너) | 값 |
|-------------------|-----|
| Ctrl 송신 `cmd_ip` | `127.0.0.1` (`udp_bridge.sh` 기본) |

각자 PC에서는 Docker Desktop의 **Settings → Resources → Network → Enable host
networking**과 WSL Integration을 활성화한다. MORAI의 기존 Host/Destination IP와 센서 포트는
바꾸지 않는다.

```bash
cd "$ASMC"
xhost +local:docker
./scripts/docker_ros_up.sh up          # 최초: build
docker exec -it asmc-ros-noetic bash
./scripts/build_ws.sh                  # 최초·패키지 추가 시
source devel/setup.bash
./scripts/udp_bridge.sh
```

Host mode는 포트 번호 자체를 없애는 기능이 아니다. MORAI와 브릿지는 기존 Camera·LiDAR·
GPS·IMU·Ego·Ctrl 포트 번호를 동일하게 사용하되, Compose의 host↔container 포트 변환만
사용하지 않는다.

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
| 명령 설정 | **cmd type 1 + ctrl mode 2** | v1.1 허용값 |

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

MORAI Sensor 목록 순서 = **camera 1–4 → 3D LiDAR → GPS → IMU → semantic camera 8–10.**
Dest는 이 순서대로 홀수로 배정하고 Host는 Dest−1이다.

| # | 센서 | Host | Dest | ROS 토픽 |
|---|------|------|------|----------|
| 1 | camera-1 | 9290 | **9291** | `/image_jpeg/compressed` |
| 2 | camera-2 | 9292 | **9293** | `/cam_left/image_jpeg/compressed` |
| 3 | camera-3 | 9294 | **9295** | `/cam_right/image_jpeg/compressed` |
| 4 | camera-4 | 9296 | **9297** | `/cam_extra/image_jpeg/compressed` |
| 5 | 3D LiDAR | 9298 | **9299** | `/velodyne_points` |
| 6 | GPS | 9300 | **9301** | `/gps` |
| 7 | IMU | 9302 | **9303** | `/imu` |
| 8 | semantic camera-8 | 9304 | **9305** | `/sem_front/image_jpeg/compressed` |
| 9 | semantic camera-9 | 9306 | **9307** | `/sem_left/image_jpeg/compressed` |
| 10 | semantic camera-10 | 9308 | **9309** | `/sem_right/image_jpeg/compressed` |

**LiDAR 2368 금지.** semantic 8–10은 GT 센서셋에서만 쓰고 기본 OFF (§6).

cam/gps/imu는 되는데 **lidar만 0**이면 [§6.1](#61-host-network-검사와-lidar-장애-대응)을 참고한다.
포트 배정 자체는 문제가 아니므로 **9299를 다른 번호로 옮기지 말 것.**

---

## 5. 패키지 · msg

| 패키지 | 역할 |
|--------|------|
| `morai_udp_bridge` | Ego/Object/Collision IN · `/ctrl_cmd` OUT |
| `morai_sensor_bridge` | Cam×4 · LiDAR(VLP-16) · GPS · IMU |

- Ego: UDP → **`wheel_angle`** (beta_drive; va의 `front_steer_angle` 아님)  
- Ctrl: **`steering`** (rad) → UDP 정규화  
- Object/Collision은 학습·eval용. **배포 추론 경로 privileged 금지**
- standalone Frenet Planner는 센서 객체 입력 연동 전 대회 경로 검증을 위해 `ObjectInfo:7505`를 임시 사용한다. perception 연동 후 해당 receiver를 제거한다.

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
rostopic hz /sem_front/image_jpeg/compressed   # GT 센서셋 사용 시
rostopic echo /Ego_topic -n1   # wheel_angle, position, velocity
```

참고 실측(26 + AIM06): Ego ~47 · cam1–3 ~20 · cam4 ~10 · gps ~5 · imu ~50 · lidar `/velodyne_points` ~7 (SIM 10 Hz·Docker UDP).

센서셋 JSON:
- 주행: [`config/ASMC_sensor_set_v1.json`](../config/ASMC_sensor_set_v1.json) (Camera×4 · LiDAR Dest **9299** · GPS · IMU · Network/Cmd는 별도)
- GT 수집: [`config/ASMC_sensor_set_GT.json`](../config/ASMC_sensor_set_GT.json) (위 + Semantic Camera×3 Dest **9305/9307/9309** → `/sem_{front,left,right}/image_jpeg/compressed`)

LiDAR TF (`base_link`→`velodyne`, v1 기준): **x=0.80 y=0.0 z=1.45**  
LiDAR Dest **2368 쓰지 말 것.**

시맨틱(개발용): `./scripts/udp_bridge.sh enable_semantic_camera:=true`  
(기본 OFF — 팀 공용 주행 브릿지와 분리)

### 6.1 host network 검사와 LiDAR 장애 대응

컨테이너 생성과 브릿지 실행은 분리한다. `docker_ros_up.sh`는 이미지·컨테이너·마운트와
host 네트워크 검사를 담당하고, `udp_bridge.sh`는 별도 터미널의 foreground에서 실행해
`rx ... lidar=...` 로그를 직접 확인한다.

```bash
# Compose에 ports가 없고 host mode인지 정적 검사
./scripts/docker_ros_up.sh config

# 컨테이너 생성 후 실제 network mode 검사
./scripts/docker_ros_up.sh up

# 실제 Windows <-> 컨테이너 양방향 UDP 검사(임시 39199/39200 사용)
./scripts/docker_ros_up.sh check

# 별도 foreground 터미널에서 브릿지 실행
docker exec -it asmc-ros-noetic bash
source devel/setup.bash
./scripts/udp_bridge.sh

# 다른 WSL 터미널에서 필수 MORAI 토픽 검사
./scripts/docker_ros_up.sh verify 8
```

`verify`는 Ego·카메라 4대·LiDAR·GPS·IMU의 메시지 수와 수신률을 한 번에 출력한다.
LiDAR만 `MISSING`이면 다음을 순서대로 확인한다.

1. Docker Desktop의 `Enable host networking`이 켜졌는지 확인한다.
2. `docker_ros_up.sh status`가 `mode=host`, `published={}`를 출력하는지 확인한다.
3. MORAI LiDAR Connect와 Dest `127.0.0.1:9299`를 확인한다.
4. 다른 프로세스가 UDP 9299를 먼저 bind하지 않았는지 `ss -lunp`로 확인한다.

Bridge-mode 시절 사용하던 Windows 포트 리스너 검사, 컨테이너 반복 재생성, Docker Desktop
재시작 우회는 기본 운용에서 제거했다. Host mode에서도 MORAI 포트 번호는 그대로 필요하며,
LiDAR Dest를 다른 번호로 옮기지 않는다.

### 6.2 2026-09-05 host mode 실측

26.R1.H3 MORAI가 실행 중인 상태에서 시뮬레이터 네트워크·센서 설정을 변경하지 않고
시험했다.

| 항목 | 결과 |
|---|---|
| 합성 Compose 설정 | `network_mode: host`, published port 없음 |
| Windows → 컨테이너 UDP (`39199`) | 통과 |
| 컨테이너 → Windows UDP (`39200`) | 통과 |
| Ego·카메라 4대·LiDAR·GPS·IMU | 모두 수신 |
| LiDAR 수신률 | 최초 약 8.23 Hz, 브릿지 재시작 후 약 8.27 Hz |
| 브릿지 42초 중단 후 재실행 | Docker Desktop/컨테이너 재시작 없이 즉시 복구 |

이 실측으로 host mode가 목표로 한 **브릿지 재시작 뒤 LiDAR `9299/udp` 수신 단절 회피**를
확인해 로컬 기본 구성으로 승격했다. 실제 차량 제어 UDP와 Capture Mode gRPC는 안전한
운용 조건 및 Capture Mode가 준비됐을 때 각각 추가 확인한다.

시험 중 `udp_bridge.sh`가 전달한 `cmd_ip`가 통합 launch에서 누락되는 문제도 발견해
수정했다. 현재 기본값 `127.0.0.1`이 실제 송신 노드까지 전달된다.

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
