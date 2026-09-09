# UDP ↔ ROS 네트워크 가이드

> **문서 역할:** Canonical — 시뮬 UDP 경계, 포트, bridge와 Docker 네트워크 정본
> **문서 허브:** [README.md](./README.md)
> **최종 수정:** 2026-09-09
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
| **권장 진입점** | `./scripts/udp_bridge.sh --simulator 26r1\|competition` |

이 문서의 포트/토픽은 **26.R1.h3 개발 스택** 기준이다. 대회 SIM에 그대로 꽂기 전에 패킷·포트를 다시 맞춰야 한다.

### 0.1 공용 UDP bridge와 SIM profile

26.R1.H3와 대회판을 별도 bridge로 복제하지 않는다. 센서 UDP는 ROS 메시지
이름과 무관하고, 현재 bridge는 Ego 패킷 181/229/245-byte 변형을 패킷 길이로
구분한다. 카메라·GPS·IMU와 Velodyne 센서 패킷, 포트가 같으면 같은 파서와
canonical ROS topic을 사용한다.

| 선택 | SIM | transport | 용도 |
|---|---|---|---|
| `--simulator 26r1` | 26.R1.H3 | UDP | 일반 개발, v2/GT 센서셋 |
| `--simulator competition` | 25.S4.MolitComp03 | UDP | 대회판 이식·게이트·당일 |

```bash
./scripts/udp_bridge.sh --simulator 26r1
./scripts/udp_bridge.sh --simulator competition
```

`--simulator`는 현재 파서를 분기하기보다 `simulator_profile`을 ROS parameter,
diagnostics, rosbag metadata에 남긴다. 이후 대회판 실증에서 제어 payload나 패킷
변형이 확인될 때만 같은 bridge의 profile 파라미터로 흡수한다. 대회판은 반드시
수신 패킷 헤더·길이와 CtrlCmd 반응을 실차 gate한다.

bridge는 v2와 GT 센서 포트의 **합집합을 항상 bind**한다. v2를 로드하면
Semantic 9305/9307/9309와 Instance LiDAR 9311은 대기하고, GT를 로드하면 별도
bridge 옵션 없이 같은 토픽으로 발행한다. MORAI UI의 sensor set/Connect 상태 자체를
UDP로 직접 조회할 수는 없으므로 이 방식이 자동 선택의 기준이다.
v2의 GT-only 포트 무수신은 diagnostics에 `optional GT stream idle`로 남고 장애 경고를
발생시키지 않는다. GT profile 수집 시에는 `verify-gt`가 해당 토픽을 필수로 검사한다.

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

### 3.1 Docker Desktop 최초 설정(팀원별 1회)

Host networking은 Docker Desktop **4.34 이상**에서 지원됩니다. 팀원 PC마다 다음 순서로
설정합니다.

1. Docker Desktop을 Linux containers/WSL 2 모드로 실행합니다.
2. **Settings → Resources → Network**를 엽니다.
3. **Enable host networking**을 체크합니다.
4. **Apply and restart**를 눌러 Docker Desktop 재시작을 완료합니다.
5. **Settings → Resources → WSL Integration**에서 개발에 사용하는 WSL 배포판을 켭니다.

`Network` 메뉴가 보이지 않으면 Windows containers 모드인지 확인하고 Linux containers로
전환합니다. `Enable host networking`이 없으면 Docker Desktop을 업데이트합니다. Docker의
[공식 host network driver 문서](https://docs.docker.com/engine/network/drivers/host/)에도 같은
활성화 절차와 TCP/UDP 양방향 지원이 명시되어 있습니다.

설정 적용 후 WSL에서 다음 합격 조건을 확인합니다.

```bash
cd "$ASMC"
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh status
./scripts/docker_ros_up.sh check
```

- `status`: `mode=host`, `published={}`
- `check`: Windows → 컨테이너와 컨테이너 → Windows가 모두 `[OK]`

`check` 실패 시 MORAI 설정부터 바꾸지 않습니다. Docker Desktop에서 host networking 체크와
**Apply and restart** 완료 여부, WSL Integration 대상 배포판, Linux containers 모드를 먼저
확인한 뒤 `docker_ros_up.sh up`을 다시 실행합니다.

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

MORAI의 기존 Host/Destination IP와 센서 포트는 바꾸지 않는다.

```bash
cd "$ASMC"
xhost +local:docker
./scripts/docker_ros_up.sh up          # 최초: build
docker exec -it asmc-ros-noetic bash
./scripts/build_ws.sh                  # 최초·패키지 추가 시
source devel/setup.bash
./scripts/udp_bridge.sh --simulator 26r1
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
./scripts/udp_bridge.sh --simulator 26r1 cmd_port:=<내_Host_PORT>
```

### 4.3 Network → 브릿지 listen (Destination PORT)

레포 기본값 (launch / compose와 동기):

| 채널 | Dest PORT | ROS |
|------|-----------|-----|
| Ego Vehicle Status | **909** | `/Ego_topic` |
| Object Info | **7505** | `/Object_topic` (oracle) |
| Collision Data | **9092** | `/CollisionData` |

### 4.4 Sensor → Destination PORT

MORAI Sensor 목록 순서 = **camera 1–4 → 3D LiDAR → GPS → IMU → semantic camera 8–10**이며,
GT 개발셋에는 동일 위치의 instance LiDAR 11이 추가된다.
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
| 11 | instance GT LiDAR | 9310 | **9311** | `/velodyne_points_instance` (GT 송신 시) |

**LiDAR 2368 금지.** semantic 8–10과 instance 11은 GT 센서셋에서만 송신되지만,
공용 bridge는 해당 포트를 항상 열어 둔다 (§6).

cam/gps/imu는 되는데 **lidar만 0**이면 [§6.1](#61-host-network-검사와-lidar-장애-대응)을 참고한다.
포트 배정 자체는 문제가 아니므로 **9299를 다른 번호로 옮기지 말 것.**

---

## 5. 패키지 · msg

| 패키지 | 역할 |
|--------|------|
| `morai_udp_bridge` | Ego/Object/Collision IN · `/ctrl_cmd` OUT |
| `morai_sensor_bridge` | Cam×4 · optional Semantic×3 · ID 5/optional ID 11 LiDAR · GPS · IMU |

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
rostopic hz /velodyne_points_instance          # GT 센서셋 사용 시
rostopic echo /Ego_topic -n1   # wheel_angle, position, velocity
```

참고 실측(26 + AIM06): cam1–3 ~20 · cam4 ~10 · gps ~5 · imu/Network status
~46 · lidar `/velodyne_points` ~8.2 Hz다. 설정 주기보다 낮은 값은 먼저 **실효 수신률**로
부르고, 원본 sequence나 송신 측 카운터 없이 곧바로 UDP frame drop으로 단정하지 않는다.

센서셋 JSON:
- 현재 개발: [`config/ASMC_sensor_set_v2.json`](../config/ASMC_sensor_set_v2.json) (Camera×4 · Intensity LiDAR Dest **9299** · GPS · IMU · Network/Cmd는 별도)
- 과거 재현: [`config/ASMC_sensor_set_v1.json`](../config/ASMC_sensor_set_v1.json) (v2와 rear camera pitch만 다름: `340°→328°`)
- GT 수집: [`config/ASMC_sensor_set_GT.json`](../config/ASMC_sensor_set_GT.json) (v2 + Semantic Camera×3 Dest **9305/9307/9309** + Instance LiDAR Dest **9311**)

LiDAR TF (`base_link`→`velodyne`, v2 기준): **x=0.80 y=0.0 z=1.45**
LiDAR Dest **2368 쓰지 말 것.**

시맨틱(개발용)은 GT 센서셋에서만 송신되지만 bridge port는 항상 열려 있다.

GT LiDAR(ID 11)는 ID 5와 같은 위치·10 Hz에서 `intensityType=2` Instance로 송신한다.
학습 입력은 ID 5의 실제 Intensity이고 ID 11의 point ID와 3D box 파일은 정답으로만 쓴다.
브릿지는 9311을 항상 독립 bind하며 v2에서는 데이터 없이 대기한다. GT 정합 수집에서
source timestamp가 필요할 때만 다음 파라미터를 추가한다.

```bash
./scripts/udp_bridge.sh --simulator 26r1 timestamp_mode:=source
```

ID 11은 `x,y,z float32 + instance_id uint32`로 발행된다. ID 5와 socket, packet buffer,
publisher를 공유하지 않는다. source timestamp와 raw/result bag 운용은
[rosbag 평가·Streaming GT 가이드](./rosbag-evaluation-streaming-gt.md)를 따른다.

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
./scripts/udp_bridge.sh --simulator 26r1

# 다른 WSL 터미널에서 필수 MORAI 토픽 검사
./scripts/docker_ros_up.sh verify 8

# GT 센서셋+GT bridge의 Semantic/ID 11 포함 health check
./scripts/docker_ros_up.sh verify-gt 20
```

`verify`는 Ego·카메라 4대·LiDAR·GPS·IMU의 메시지 수와 수신률을 한 번에 출력한다.
`verify-gt`는 Object, Semantic 3대와 ID 11 schema/source pairing도 함께 보고한다. 엄격한
선정 gate는 컨테이너에서 `verify_morai_topics.py --profile gt --strict-thresholds` 또는
closed bag의 `audit_rosbag_run.py --strict`로 수행한다.
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

### 6.3 2026-09-09 Streaming GT 실측

GT sensor set과 source timestamp bridge를 Real Time/UDP Connect에서 검증했다.

| 항목 | 결과 |
|---|---|
| RGB/Semantic 전·좌·우 | 각 약 20 Hz |
| ID 5 / ID 11 | 최신 20초 8.03 / 7.43 Hz |
| ID 11 schema | `instance_id uint32`, `intensity` 없음 |
| source timestamp regression/fallback | 관측 구간 0 / 0 |
| front/left/right RGB–Semantic p95 | 최신 20초 34 / 35 / 31 ms, 다른 구간 3 / 3 / 3 ms |
| ID 5–11 p95 | 최신 102 ms, 이전 약 106~108 ms |

9311 수신과 stream 분리는 통과했지만 두 LiDAR와 최신 camera 표본의 strict pairing 기준은
통과하지 못했다. 따라서 Streaming GT는 평가·디버깅에 쓰고, 학습 label은 Capture 파일을
우선한다. 현재 장애물이 없는 장면에서는 Instance ID가 모두 0이었으므로 actor 장면에서
Object/box ID 대응 검증이 남아 있다.

### 6.4 2026-09-09 v2 UDP↔ROS 지연·수신률 기준선

26.R1.H3, v2 sensor set, Docker host network, `timestamp_mode=receive`에서 60초 동안
브릿지 내부 처리 시간과 ROS subscriber 관측값을 함께 측정했다. 측정 도구는 다음처럼
JSON을 남길 수 있다.

```bash
python3 scripts/verify_morai_topics.py \
  --profile runtime --duration 60 \
  --json-out /data/rosbags/audits/udp_bridge_v2_60s.json
```

#### 지연과 누락의 정의를 분리한다

| 지표 | 의미 | 포함하지 않는 것 |
|---|---|---|
| bridge `processing_*_ms` | 검증된 UDP datagram/frame을 parse·ROS message화하고 `publish()`가 반환될 때까지의 callback wall time | MORAI 생성 지연, Windows→WSL 전송 전 구간, 알고리즘 시간 |
| camera/LiDAR `assembly_*_ms` | 첫 fragment/packet부터 완성 frame/cloud publish까지 | 순수 변환 연산만을 뜻하지 않음 |
| verifier `header_age_ms` | receive mode에서는 브릿지가 stamp를 만든 뒤 subscriber callback까지 | source-to-host 전체 지연 |
| subscriber rate/arrival gap | 해당 subscriber가 실제 받은 message의 주기·jitter | 원인이 SIM 송신, UDP, bridge, ROS queue 중 어디인지는 단독 판별 불가 |
| kernel/socket drop | Linux UDP stack에서 확인되는 명시적 오류·버퍼 overflow | 시뮬레이터가 애초에 송신하지 않은 frame |

`processing`과 `header_age`는 일부 구간이 겹치므로 합산하지 않는다. 특히 `assembly`는
카메라 fragment나 LiDAR scan을 기다린 시간이 들어가며, 연산 병목 수치가 아니다.

#### 60초 수신률과 완전성

| ROS topic | 설정/기대 Hz | subscriber 실측 Hz | 관찰 |
|---|---:|---:|---|
| RGB front/left/right | 20 | 20.003/20.004/20.004 | 설정값과 일치 |
| RGB extra | 10 | 10.003 | 설정값과 일치 |
| GPS | 5 | 5.001 | 설정값과 일치 |
| IMU | 50 | 46.029 | 약 8% 낮은 실효 주기 |
| Intensity LiDAR | 10 | 8.244 | 약 17.6% 낮은 실효 주기 |
| Ego | 50 | 40.441 | 같은 시점 bridge interval은 약 46.2 Hz |

관측 종료 시 Linux UDP `InErrors`, `RcvbufErrors`, `SndbufErrors`, `InCsumErrors`,
`MemErrors`와 해당 UDP socket의 drop은 모두 0이었고, bridge parse error도 0이었다.
따라서 카메라·GPS에는 로컬 유실 증거가 없고, LiDAR/IMU의 nominal 대비 부족분도 현재
자료만으로 UDP 유실이라고 판정할 수 없다. LiDAR packet에는 scan sequence가 없어 SIM의
실제 송신 cadence와 Windows 이전 손실을 분리할 수 없기 때문이다.

Ego는 bridge의 유효 UDP 처리율 약 46.2 Hz보다 별도 subscriber 관측률 40.4 Hz가 낮았다.
`queue_size=1`과 burst/jitter에 따른 latest-value 교체 또는 subscriber scheduling 영향이
유력하지만, 서로 다른 계층의 같은 sequence를 추적하지 않았으므로 확정 drop률로 기록하지
않는다. 상태·제어에는 최신값 우선 정책이 합리적일 수 있으나, 50 Hz 전체 보존이 필요하면
publisher sequence와 subscriber sequence gap을 같은 5분 run에서 추가 측정한다.

검증 JSON의 `long_gap_equivalent_periods`도 **drop count가 아니다**. subscriber 도착 간격이
그 topic의 중앙 주기보다 길었던 정도를 주기 수로 표현한 jitter/burst 지표다.

#### 변환·전달 지연

| 경계 | p50 | p95 | max | 해석 |
|---|---:|---:|---:|---|
| RGB front bridge 처리 | 0.094 ms | 0.258 ms | 1.509 ms | JPEG payload를 ROS CompressedImage로 publish |
| RGB left bridge 처리 | 0.079 ms | 0.271 ms | 4.636 ms | 같은 경계 |
| RGB right bridge 처리 | 0.062 ms | 0.210 ms | 0.666 ms | 같은 경계 |
| Intensity LiDAR bridge 처리 | 1.142 ms | 1.583 ms | 4.150 ms | point 계산·PointCloud2 생성 포함 |
| Ego UDP→ROS bridge 처리 | 0.119 ms | 0.423 ms | 0.832 ms | packet parse·ROS publish enqueue |
| Ctrl ROS callback→UDP `sendto` | 0.080 ms | 0.110 ms | 0.364 ms | 중립/제동 명령 193표본 |

receive stamp 기준 ROS message age p95는 RGB front/left/right `0.690/0.653/0.578 ms`,
LiDAR `1.522 ms`, Ego `0.603 ms`, GPS `0.572 ms`, IMU `0.500 ms`였다. 이 값은 브릿지에서
stamp를 설정한 뒤 subscriber까지의 age이며 MORAI가 sensor frame을 생성한 시각부터의
end-to-end 지연은 아니다. LiDAR scan assembly는 p50 `109.7 ms`, p95 `215.8 ms`로 변환
연산보다 scan packet 축적과 실제 도착 cadence가 신선도를 지배했다.

현재 측정으로 확인된 제어 경계는 다음 세 부분뿐이다.

```text
MORAI Ego UDP --[parse+publish p95 0.423 ms]--> ROS subscriber
ROS algorithm --[이번 측정에서 제외]--> /ctrl_cmd
/ctrl_cmd callback --[packet+sendto p95 0.110 ms]--> Windows UDP socket
```

따라서 “UDP→ROS→제어→UDP 전체가 정확히 몇 ms”라고 두 값을 더해 말할 수 없다. ROS
publisher에서 bridge callback까지, 인지·판단·MPC 실행시간, Windows network stack과 MORAI
actuation 반영시간이 남아 있다. 완전한 closed-loop 지연은 실제 controller를 켠 동일 bag/run에서
Ego source stamp, command 식별자와 조향/가감속 응답을 상관시켜 별도로 측정한다.

#### 브릿지 제거·직접 UDP 처리 판단은 통합 이후로 보류

모든 센서 데이터가 bridge를 거치므로 위 수치는 향후 병목 분석의 기준선으로 보존한다.
그러나 현재 수치만으로 bridge를 제거하거나 각 모듈이 MORAI UDP를 직접 받도록 바꾸지 않는다.

LiDAR의 p95 `1.583 ms` 전체가 제거 가능한 ROS overhead는 아니다. 이 값에는 raw Velodyne
packet 해석, 거리·각도 좌표 계산과 point 배열 생성처럼 직접 UDP consumer에도 필요한 일이
포함된다. 직접 UDP로 바꿨을 때 실제로 없어지는 후보는 PointCloud2 구성 일부, ROS 직렬화·복사와
publish/subscribe scheduling이며, 절감 가능치는 `1.583 ms`보다 작다. 약 `109.7/215.8 ms`
(p50/p95)의 LiDAR assembly 시간도 packet 도착과 scan 축적 시간이므로 bridge 제거만으로
사라지지 않는다.

카메라 bridge p95는 `0.21~0.27 ms`, Ego UDP→ROS는 `0.423 ms`, Ctrl ROS→UDP는
`0.110 ms`다. 20 Hz 카메라의 50 ms frame budget이나 8.24 Hz LiDAR의 약 121 ms 실효
주기에 비해 현재 확인된 bridge 처리비용은 작다. 또한 bridge를 유지하면 다음 이점이 있다.

- 모든 팀원이 같은 canonical ROS topic과 좌표계를 사용한다.
- raw rosbag replay와 모델 A/B 비교가 가능하다.
- simulator version과 transport 차이를 adapter 경계에 격리한다.
- rate, stale, parse 오류와 p50/p95 진단을 공통으로 수집한다.
- 센서 UDP parser가 팀 모듈마다 중복되는 것을 막는다.

따라서 이 항목은 **현재 최적화 작업이 아니라 통합 후 재검토할 후보**다. 전체 스택에서 먼저
인지·판단·MPC, queue 대기, sensor assembly와 MORAI actuation을 포함한 end-to-end profile을
수집한다. 다음 중 하나가 같은 재현 run에서 확인될 때만 직접 UDP A/B를 연다.

- bridge/ROS 경계가 전체 deadline의 5~10% 이상을 반복해서 차지한다.
- publisher sequence와 subscriber sequence 비교로 ROS 구간의 실제 drop이 확인된다.
- bridge 때문에 control deadline을 1~2 ms 차이로 반복해서 놓친다.
- serialization·copy가 CPU/메모리 병목이라는 profiler 근거가 있다.

재검토 시에도 bridge 제거를 첫 선택으로 두지 않는다. queue depth와 latest-value 정책,
`tcp_nodelay`, 불필요한 image/PointCloud 복사, buffer 재사용 및 intra-process 구성을 먼저
비교한다. 직접 UDP 후보는 기존 ROS 경로와 동일 입력·동일 출력 표현을 사용해
`UDP ingress → consumer-ready`를 측정해야 하며, 기능·진단·재현성 손실까지 함께 평가한다.
대회판에서도 MORAI 경계의 UDP 사용 여부와 팀 PC 내부 ROS bridge 제거 여부는 별개의 결정이다.

#### v2↔GT hot reload 로그 해석

bridge console의 숫자는 프로세스 시작 후 **누적 message 수**다. 기존 출력에서 GT로 바꿀 때
`lidar_instance=951`, `sem_front=2575`까지 증가한 뒤 v2로 되돌려도 값은 0으로 초기화되지
않았다. 두 5초 로그에서 값이 똑같았으므로 그 구간의 GT 입력은 실제로 0 Hz였다. 현재 출력은
혼선을 막기 위해 `누적값(최근 5초 Hz)` 형태다.

```text
lidar_instance=951(0.0Hz) sem_front=2575(0.0Hz)
```

즉 v2가 GT를 송신한 것이 아니며, sensor set hot reload 전에 GT로 받은 누적 이력만 남은
것이다. 확실한 현재 상태는 괄호의 interval Hz와 `last_message_age_sec`로 판정한다.

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
