# UDP ↔ ROS 네트워크 가이드

> **최종 수정:** 2026-08-30
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

cam/gps/imu는 되는데 **lidar만 0**인 경우가 자주 나온다 → [§6.1](#61-lidar만-0일-때-9299-포워딩) 참고.
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

### 6.1 lidar만 0일 때 (9299 포워딩)

`rostopic hz /velodyne_points`가 `no new messages`이고 브릿지 로그가 이렇게 나오는 증상:

```
rx cam1=3797  cam2=3800  cam3=3800  cam4=1900  gps=950  imu=8853  lidar=0
lidar 수신 0건 - MORAI Sensor Destination Port 확인.
```

**포트 배정·MORAI 설정·브릿지 코드 문제가 아니다.** Docker Desktop의 9299 UDP 포워딩이
끊긴 것이고, 컨테이너 안에서는 정상적으로 `0.0.0.0:9299`에 bind되어 있다 (`ss -lunp`로 확인 가능).

**원인.** MORAI가 스트림을 쏘는 동안 컨테이너 쪽 9299에 리스너가 없으면
Docker Desktop이 약 30초 후 이 포워딩을 스스로 철거한다. 한 번 철거되면 되살아나지 않는다.
컨테이너를 먼저 띄우고 브릿지를 늦게 실행하면 그 사이에 죽는다. cam/gps/imu가 멀쩡한데
lidar만 죽는 이유는 LiDAR 패킷 레이트가 훨씬 높아서다.

**확인.** 브릿지 로그의 `rx ... lidar=` 카운터를 본다.
`docker_ros_up.sh`가 찍는 `[OK] Windows UDP Dest open`은 **판단 근거가 못 된다** —
`up` 2초 뒤 검사인데 포워딩은 죽기까지 30초쯤 살아 있어서 깨진 상태에서도 통과한다.
Windows에서 직접 보려면:

```powershell
netstat -ano -p UDP | Select-String ':929[0-9]|:930[0-9]'
```

9291·9293·…·9309는 `com.docker.backend` PID로 떠 있는데 **9299만 없으면** 이 증상이다.

**해결 1 — 컨테이너 재생성 (대부분).** 재생성한 뒤 **곧바로** 브릿지를 실행한다.
컨테이너만 띄워두고 몇 분 뒤에 브릿지를 올리면 같은 증상이 재발한다.

```bash
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash
source devel/setup.bash && ./scripts/udp_bridge.sh   # 지체 없이
```

**해결 2 — Docker Desktop 재시작 (재생성으로도 안 될 때).**
Docker Desktop 내부 포워딩 상태가 깨지면 컨테이너를 몇 번 재생성해도 9299가 살아나지 않는다
(측정: 300패킷 송신 → 0 수신, 같은 조건에서 9305는 정상 수신). 이때는 재시작해야 한다.

**WSL 안에서 `docker desktop restart`를 쓰지 말 것** —
`fork/exec /opt/docker-desktop/bin/com.docker.backend: no such file or directory`로 실패하면서
아무것도 재시작하지 않는데 `docker ps`는 계속 동작해서 성공한 것처럼 보인다. Windows CLI를 쓴다:

```bash
"/mnt/c/Program Files/Docker/Docker/resources/bin/docker.exe" desktop restart
```

재시작 후 컨테이너는 restart policy가 `no`라 자동으로 안 올라온다.
`./scripts/docker_ros_up.sh up`으로 다시 띄우고, 다른 컨테이너(`hmobility-sim` 등)도 `docker start`한다.

**하지 말 것.** LiDAR Dest를 9299 외의 번호로 옮기는 것 (과거 `f47340a`에서 9311로 옮겼다가
`db854dd`에서 되돌림). 포트 번호 문제가 아니라서 옮겨도 같은 증상이 재발하고,
센서셋 JSON·launch·compose·문서를 전부 갈아야 하며 팀 포트 표만 어긋난다.
compose의 `0.0.0.0:9299:9299/udp` 명시 bind도 이 증상에 효과가 없다 (축약형과 동일하게 0 수신).

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
