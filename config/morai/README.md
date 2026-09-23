# MORAI network and sensor presets

## 어떤 파일을 로드하는가

| MORAI release | 개발 PC 한 대 | 현장 두 PC |
|---|---|---|
| 26.R1.H3 | `network/generated/ASMC_network_set_26r1_local.json` | 검증용 `..._26r1_lan2lan.json` |
| 25.S4.MolitComp03 | `network/generated/ASMC_network_set_molitcomp03_local.json` | `network/generated/ASMC_network_set_molitcomp03_lan2lan.json` |

현장 정본 조합은 `molitcomp03 + lan2lan`이다. 두 release의 import schema와
`PUBSUB_TYPE`은 다르므로 파일을 교차 import하지 않는다. 반면 ROS bridge runtime은 둘 다
`morai_udp` 하나다. `generated/`는 `templates/`와
[`bridge_profiles_v1.json`](../bridge/bridge_profiles_v1.json)에서 결정적으로 생성한다.

```bash
./scripts/udp_bridge.sh
./scripts/udp_bridge.sh --streams development
./scripts/udp_bridge.sh --transport lan2lan
```

인자가 없는 첫 명령은 `runtime=morai_udp`, `transport=local`, `streams=competition`,
Ctrl target `127.0.0.1:9093`으로 실행된다. Object/Semantic/Instance는 발행하지 않으며
두 번째 명령에서만 local 개발 oracle로 연다. sensor set이나 MORAI release를 bridge가
선택하지는 않는다. sensor set과
release network JSON은 MORAI UI에서 불러온 것이 기준이다.

두 번째 명령은 Ctrl 대상 IP와 diagnostics를 `lan2lan`으로 맞추지만 실제 LAN gate를 대신하지
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

## 센서셋

파일명은 팀 명명 규칙인 `ASMC_sensor_set_*`를 유지한다.

| 파일 | Camera-4 pitch | VLP16 | 용도 |
|---|---:|---:|---|
| `ASMC_sensor_set_v1.json` | 340° | 10 Hz | 최초 설정·과거 재현 |
| `ASMC_sensor_set_v2.json` | 328° | 10 Hz | 현재 회귀 기준 |
| `ASMC_sensor_set_v3.json` | 328° | 15 Hz | 본선 후보, live acceptance 대기 |
| `ASMC_sensor_set_GT.json` | 328° | 10 Hz | v2 + Semantic Camera 3대 + Instance LiDAR, 개발 전용 |

v3는 MORAI UI가 저장한 `sensorPeriod≈0.06666667`, `frequencyIndex=2`를 가진다. v2와 센서 pose,
Camera×4, GPS/IMU, UDP `9290~9303`은 같다. export release 차이로 v3에는 일부 ROS2Native 빈
필드가 없고 LiDAR의 `fireDistance`, `noiseRatio`, `velodyneUnit`가 명시돼 있다. 이 schema 차이를
센서 구성 변경으로 오해하지 않는다.

v3 승인 조건은 `/velodyne_points`를 30초 이상 측정한 평균 14~16 Hz, packet/parse error,
latency·CPU, 손재호 tracking의 10 Hz 대비 결과다.

원본 네 파일은 UI export·provenance 기준이다. `sensors/generated/`의 v2/v3
`local`/`lan2lan` 파일은 포트·pose·주기를 고정하고 UDP IP만 transport 계약으로 바꾸어
생성한 MORAI import용이다. GT는 local 개발 전용이며 LAN2LAN 파생본을 만들지 않는다.
v3 LAN2LAN 파일의 존재는 15 Hz live acceptance나 실제 LAN 승인을 의미하지 않는다.

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
Windows Firewall과 Hyper-V firewall에서는 MORAI `192.168.0.1` source에 대해 Autonav inbound UDP
`7502`, `9092`, `9099`, `9291/9293/9295/9297/9299`, `9301/9303`만 허용한다. GT 전용
`9305/9307/9309/9311`과 Object oracle `7505`는 본선 allowlist에 넣지 않는다. Ctrl `9093`은
Autonav에서 MORAI로 나가는 목적지다. 기본 inbound action 전체를 Allow로 바꾸지 않는다.

- WSL mirrored networking: <https://learn.microsoft.com/windows/wsl/networking#mirrored-mode-networking>
- Docker Desktop host networking: <https://docs.docker.com/engine/network/drivers/host/#docker-desktop>

Windows loopback 검증은 실제 LAN ingress를 증명하지 않는다. `lan2lan`은 Windows+WSL+Docker
구성과 최종 native Linux 구성 각각에서 route, firewall, bind, packet rate와 재시작 복구를
통과해야 한다.
