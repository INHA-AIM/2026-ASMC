# MORAI network and sensor presets

> **2026-10-27 현재 실행 기준:** CompetitionInfoPublisher는9088→9099→`/Ego_topic`이다.
> `--require-competition-status`/`verify-field`도 이9099 입력을 검사하도록 정정했다.
> 9798은 로컬 Detail 진단이며 LAN 운영·수락에서 제외한다. 아래 과거 P3 설명과 현재 역할은
> [정정 기록](../../docs/records/bridge/2026-10-27-competition-9099-correction.md)을 기준으로 구분한다.


## 어떤 파일을 로드하는가

| MORAI release | 개발 PC 한 대 | 현장 두 PC |
|---|---|---|
| 26.R1.H3 | `network/generated/ASMC_network_set_26r1_local.json` | 검증용 `..._26r1_lan2lan.json` |
| 25.S4.MolitComp03 | `network/generated/ASMC_network_set_molitcomp03_local.json` | `network/generated/ASMC_network_set_molitcomp03_lan2lan.json` |

파일 로드 뒤에는 **Network Settings의 Cmd Control 표시 이름이 `MoraiCmdController`인지**
확인한다. `TrajectoryTrackingCmd`는 다른 제어 프로토콜이며 포트가9093으로 같아도 현재
`/ctrl_cmd` 패킷을 소비하지 않는다. 2026-10-26 실제 UI와 현재 PC 저장본을 대조해
26R1 생성본의 제어 선택을 수정했다. `templates/`의 과거 export는 출처로 보존하고,
`bridge_profiles_v1.json`의 `control_import`가 검증된 제어 선택만 적용한다.
기존에 Windows SaveFile로 복사한 JSON은 저장소 수정으로 자동 갱신되지 않는다.
새 `generated/` 파일을 불러온 뒤 실제 드롭다운 이름·포트·연결 상태를 다시 확인한다.
2026-10-26에는 현재 PC의 `SaveFile/Network/26.R1.H3`에 있는 생성본4개도 저장소와
바이트 일치를 확인했다. 과거 원본 export는 수정하지 않았으며, 이 복사는 다른 PC에 자동 전파되지 않는다.
실제 응답 확인과 정지 순서는 [판제 실행 안내](../../docs/operations/planning-control.md#morai-제어-프로토콜과-정지-순서)를 따른다.

현장 정본 조합은 `molitcomp03 + lan2lan`이다. 두 release의 import schema와
`PUBSUB_TYPE`은 다르므로 파일을 교차 import하지 않는다. 반면 ROS bridge runtime은 둘 다
`morai_udp` 하나다. `generated/`는 `templates/`와
[`bridge_profiles_v1.json`](../bridge/bridge_profiles_v1.json)에서 결정적으로 생성한다.

```bash
./scripts/udp_bridge.sh
./scripts/udp_bridge.sh --gt
./scripts/udp_bridge.sh --transport lan2lan
```

인자가 없는 첫 명령은 `runtime=morai_udp`, `transport=local`, GT OFF,
Ctrl target `127.0.0.1:9093`으로 실행된다. Object/Semantic/Instance는 발행하지 않으며
`--gt`에서만 local 개발 oracle로 연다. 이전 `--streams`/`ASMC_STREAMS`는 제거했다.
내부 `stream_profile=competition/development`는 일반/GT 입력 구분이며 MORAI 버전이 아니다.
sensor set이나 MORAI release를 bridge가
선택하지는 않는다. sensor set과
release network JSON은 MORAI UI에서 불러온 것이 기준이다.

LAN2LAN 명령은 Ctrl 대상 IP와 diagnostics를 `lan2lan`으로 맞추지만 실제 LAN gate를 대신하지
않는다. `lan2lan` network JSON은 Ctrl, Ego Vehicle Status, Competition Vehicle Status,
CollisionData만 LAN IP를 쓰고
Object/legacy/미확인 채널은 MORAI PC loopback에 둔다. renderer는 UDP field만 바꾸며
gRPC/rosbridge 설정은 별도 개발 도구 소유다.

```bash
python3 tools/bridge/render_morai_network_profiles.py --write
python3 tools/bridge/render_morai_network_profiles.py --check
python3 tools/bridge/render_morai_sensor_profiles.py --write
python3 tools/bridge/render_morai_sensor_profiles.py --check
```

`archive/ASMC_network_set_26r1_local_legacy_9111.json`은 과거 bag/run 재현용이다. 새 실행에
로드하지 않는다.

## Network export 차이

release JSON은 단순 IP 변형이 아니다. 팀이 26r1 UI에서 저장한 과거 v2 export는 23개 entry,
MolitComp local dev export는 12개 entry다. 과거 26R1의 type786은 TrajectoryTrackingCmd로,
현재 UDP bridge와 맞지 않았다. 생성본은 두 release 모두 type771 MoraiCmdController를 사용한다.
Ego status의 type770/295와 나머지 release별 schema는 유지한다.

| 제어 설정 | 26.R1.H3 생성본 | 25.S4.MolitComp03 생성본 |
|---|---|---|
| Cmd Control | MoraiCmdController (`PUBSUB_TYPE=771`) | MoraiCmdController (`PUBSUB_TYPE=771`) |
| import의 `egoCtrlMode` | 3 (현재 PC UI 저장본으로 확인) | 2 (기존 팀/주최 import 계약) |
| import의 `receiveBufferSize` | 59 | 55 |
| Host/Destination PORT | 9093/9094 | 9093/9094 |
| UDP payload의 CtrlMode | 2=AutoMode | 2=AutoMode |

import 선택값과 UDP payload의 AutoMode 바이트는 서로 다른 enum이다. 이를 같은 숫자로
일괄 정규화하지 않는다. 현재 bridge는 검증된23바이트 payload(전체55B)를 유지한다.
26R1의 조향·제동은 제한 대조로 확인했다. 2026-10-26에는 사용자가 대회용
MolitComp03에서 기본 설정으로 새 LiDAR 판제 주행이 동작함을 확인했다. 실행 중인
`Simulator_v.S4.251001.MolitComp03_Window`, `invert_steer=false`, local UDP와
`/perception/lidar/tracks_map` 직접 소비도 읽기 전용 조회로 확인했다.
주행 거리·전 미션·LAN2LAN 수락을 이 확인으로 대체하지 않는다.
ROS 메시지·bridge runtime을 release별로 나누지 않는다.

ROS 조향은 좌회전 양수 rad다. **대회용 MolitComp03는 검증된 기본값 `invert_steer=false`를 유지**한다.
같은 PC라도 확인한 R1.H3·IONIQ5·UDP 조합은 반대 yaw 응답을 내어 `invert_steer:=true`가 필요했다.
두 결과를 PC 이름으로 선택하거나 모든 release에 true를 적용하지 않는다. 동일 실행 조합에서는
일관된 부호를 사용하고, 부호 변환은 bridge에서 한 번만 수행한다. 최대40도와 판제 내부 계약은 유지한다.
팀원의 과거 false 주행 기록(`f84a62ce`)도 보존한다. ROS WebSocket은 이 UDP 변환을 거치지 않는다.
실행 중인 bridge와 중복 시작하지 않는다.

```bash
# 대회용 MolitComp03: 공용 기본값 false
./scripts/udp_bridge.sh
# 확인한 26R1 개발 조합: 명시적 override
./scripts/udp_bridge.sh invert_steer:=true
```

주최 PC MolitComp network의 10개 type은 팀 local dev의 공통 부분이며, 팀 dev에만 type 266
SensorSync와 type 783 MoraiSL subscriber가 더 있다. 이 둘은 LAN으로 승격하지 않는다. 두
MolitComp 원본에서 공통 entry의 종류 차이는 없고, 설정 차이는 TF2(type 267)의 network 활성
상태와 Ctrl mode(주최 2, 팀 원본 1)다. MolitComp 생성기는 Ctrl을 mode2로 정규화한다.

현재 MolitComp03의 CompetitionInfoPublisher는 type295 `9088→9099`다. type787 `9797→9798`은 별도 Detail이다. type 258
`7603→7502`는 연속 송신형 TrafficLight Status publisher다. 이것은
`GetTrafficLightStatus`/`MoraiTLServiceProvider`와 같은 조회 서비스가 아니다. 조사한 주최
MolitComp, 팀 MolitComp, 26.R1 export 모두 type 778 MoraiTLServiceProvider를 포함하지 않으며,
참가팀 배포판의 Network UI에도 GetTrafficLight가 없었다. 26.R1 export에 gRPC service(type
4865)가 있거나 저장소에 gRPC proto가 있다는 사실도 실제 실행판에서 traffic-light query가
허용·제공된다는 증거가 아니다. 따라서 canonical bridge와 검증기는 GetTrafficLight를 필수
입력으로 요구하지 않고, type 258도 production planning 입력으로 소비하지 않는다. 실제 신호등
판단은 카메라 인지 결과를 사용한다. 주최 network에 있는 항목 전체를 팀 runtime 필수 입력으로
해석하지 않는다.

팀 MolitComp 직접 저장본과 주최 저장본은 type787 항목을 포함하지만, 현재 실측은 별도
Detail138B다. v1의 `competition_vehicle_status`는 이 endpoint의 호환 키이고 실제 역할은
verified_status_bindings의 UI·실측 매핑을 따른다. LAN에서는 이 endpoint를 loopback에 두며
브릿지 수신·발행과 수락에서 제외한다. 실제9099 Competition은 LAN에 포함한다.

정확한 원본 inventory는 다음과 같다. `Division_TYPE` 1/2/3/4는 각각 publisher,
subscriber, controller, service다.

| export | publisher (1) | subscriber (2) | controller (3) | service (4) | 별도 gRPC |
|---|---|---|---|---|---|
| 주최 MolitComp03 | 258, 261, 267, 295, 787, 770, 772 | 259 | 771 | 780 | 없음 |
| 팀 MolitComp03 local | 위와 같음 | 259, **266, 783** | 771 | 780 | 없음 |
| 팀 26.R1 local | 258, 261, 265, 263, 267, 271, 770, 772, 779 | 259, 262, 264, 266, 776, 783 | 786 | 269, 270, 274, 275, 780, 782 | 4865 |

주요 이름은 `258=TLStatusPublisher`, `259=TLCtrlSubscriber`, `261=CollisionData`,
`262=SensorPosSubscriber`, `263/264/265=GlobalIntersection publisher/control/status`,
`266=SensorSyncDataSubscriber`, `267=TF2Publisher`, `269/270/274/275=SyncMode service`,
`271=SyncModeInfoPublisher`, `770=MoraiInfoPublisher`, `772=MoraiObjectInfoPublisher`,
`776=MoraiLampsSubscriber`, `779=MoraiReplayInfoPublisher`, `780=MoraiEventCmdServiceProvider`,
`782=MoraiMapSpecServiceProvider`, `783=MoraiSLSubscriber`, `4865=MoraiService(gRPC)`다.
MolitComp 전용 `295/771/787`은 해당 실행판 UI/export와 live packet을 기준으로 각각 CompetitionInfoPublisher,
Ego Ctrl Cmd, 별도 Detail로 확인했다. 릴리스가 다른 enum 이름을
숫자만 보고 교차 적용하지 않는다.

26.R1의 type 770은 Ego/MoraiInfo이고 MolitComp03의 type295는 CompetitionInfoPublisher다.
type787 Detail은26.R1 원본에 없다. 따라서 release JSON 하나로의 병합은 하지
않지만 ROS bridge binary, 포트 계약, ROS topic API는 공통으로 유지한다.

## 센서셋

파일명은 팀 명명 규칙인 `ASMC_sensor_set_*`를 유지한다.

| 파일 | Camera-4 pitch | VLP16 | 용도 |
|---|---:|---:|---|
| `ASMC_sensor_set_v1.json` | 340° | 10 Hz | 최초 설정·과거 재현 |
| `ASMC_sensor_set_v2.json` | 328° | 10 Hz | 현재 회귀 기준 |
| `ASMC_sensor_set_v3.json` | 328° | 15 Hz | 본선 설정, 실제 완성 scan 약 12 Hz 허용 |
| `ASMC_sensor_set_GT.json` | 328° | Intensity·Instance 모두15 Hz | 일반 센서 + Semantic Camera 3대 + Instance LiDAR, 개발 전용 |

v3는 MORAI UI가 저장한 `sensorPeriod≈0.06666667`, `frequencyIndex=2`를 가진다. v2와 센서 pose,
Camera×4, GPS/IMU, UDP `9290~9303`은 같다. export release 차이로 v3에는 일부 ROS2Native 빈
필드가 없고 LiDAR의 `fireDistance`, `noiseRatio`, `velodyneUnit`가 명시돼 있다. 이 schema 차이를
센서 구성 변경으로 오해하지 않는다.

v3의 팀 요구는 UI 저장 주기와 손재호 코드의 소비 loop가 15 Hz인 것이다. 2026-09-23
MolitComp03 local 실측은 raw 약 600 packet/s, 50 packet/revolution, 완성 scan 약 12 Hz였고
이는 허용된다. bridge는 azimuth 기준으로 8 Hz 오조립을 제거하며, 시뮬레이터에 없는 scan을
복제해 15 Hz로 만들지 않는다. packet/parse error, latency·CPU와 tracking 품질은 계속 관찰한다.

GT는 2026-10-17에 두 LiDAR의 `sensorPeriod≈0.06666667`·`frequencyIndex=2`만 수정했다.
pose·포트·나머지 센서는 보존했으며 이전10Hz SHA는 bridge 계약과 Git 이력에 남긴다.
MORAI UI에서 GT JSON을 다시 로드해야 새 주기가 적용된다. Bridge의 조립 참고값도15Hz지만
실제 송신 주기를 원격 변경하거나 완성 scan을 복제하지 않는다.

v1/v2/v3 파일은 UI export·provenance 기준이다. `sensors/generated/`의 v2/v3
`local`/`lan2lan` 파일은 포트·pose·주기를 고정하고 UDP IP만 transport 계약으로 바꾸어
생성한 MORAI import용이다. GT는 local 개발 전용이며 LAN2LAN 파생본을 만들지 않는다.
v3 LAN2LAN 파일의 존재는 실제 LAN2LAN 배포 승인을 의미하지 않는다.

## IP와 Docker/WSL

| transport | MORAI | Autonav |
|---|---|---|
| `local` | `127.0.0.1` | `127.0.0.1` |
| `lan2lan` | `192.168.0.1` | `192.168.0.10` |

Docker Desktop 개발 compose는 `network_mode: host`이므로 Docker `ports:`를 열거하지 않는다.
`./scripts/docker_ros_up.sh check`로 Windows↔container loopback UDP를 검사한다.

Windows 11 22H2 이상의 WSL 개발 PC에서는 `%UserProfile%\.wslconfig`의 mirrored networking을
LAN2LAN 후보로 사용한다.

```ini
[wsl2]
networkingMode=mirrored
firewall=true
```

변경 후 PowerShell에서 `wsl --shutdown`하고 Docker Desktop 4.34 이상의 host networking을
활성화한다. mirrored mode는 Windows NIC를 WSL에 반영하고 Windows↔WSL localhost 통신 및 LAN의
WSL 접근을 지원한다. Docker Desktop host networking은 그 위에서 container와 host 사이의
TCP/UDP를 연결한다. 둘은 별도 계층이므로 둘 다 켜고 실제 UDP 왕복으로 검증해야 한다.

bridge 수신 socket은 `0.0.0.0`과 기존 destination port에 bind하므로 포트를 변환하지 않는다.
따라서 sensor/network 수신은 local과 LAN2LAN에서 같은 bridge binary와 port를 쓴다. 다만
`/ctrl_cmd`의 송신 목적지 IP는 수신 전에도 확정돼야 하므로 기본 local은 인자 없이,
현장은 `--transport lan2lan`을 명시한다. sender IP 자동 추론은 다중 NIC·NAT·mirrored 환경과
첫 status packet 이전 제어에서 모호하므로 기본 정책으로 사용하지 않는다.
Windows Firewall과 Hyper-V firewall에서는 MORAI `192.168.0.1` source에 대해 Autonav inbound UDP
`9092`, `9099`, `9291/9293/9295/9297/9299`, `9301/9303`만 허용한다. TrafficLight
oracle `7502`, GT 전용
`9305/9307/9309/9311`과 Object oracle `7505`는 본선 allowlist에 넣지 않는다. Ctrl `9093`은
Autonav에서 MORAI로 나가는 목적지다. 기본 inbound action 전체를 Allow로 바꾸지 않는다.

- WSL mirrored networking: <https://learn.microsoft.com/windows/wsl/networking#mirrored-mode-networking>
- Docker Desktop host networking: <https://docs.docker.com/engine/network/drivers/host/#docker-desktop>

Windows loopback 검증은 실제 LAN ingress를 증명하지 않는다. `lan2lan`은 Windows+WSL+Docker
구성과 최종 native Linux 구성 각각에서 route, firewall, bind, packet rate와 재시작 복구를
통과해야 한다.
