# MORAI network and sensor presets

## 어떤 파일을 로드하는가

| MORAI release | 개발 PC 한 대 | 현장 두 PC |
|---|---|---|
| 26.R1.H3 | `network/generated/ASMC_network_set_26r1_dev_local.json` | 검증용 `..._26r1_competition_lan.json` |
| 25.S4.MolitComp03 | `network/generated/ASMC_network_set_molitcomp03_dev_local.json` | `network/generated/ASMC_network_set_molitcomp03_competition_lan.json` |

현장 정본 조합은 `molitcomp03 + competition_lan`이다. 두 release의 import schema와
`PUBSUB_TYPE`은 다르므로 파일을 교차 import하지 않는다. 반면 ROS bridge runtime은 둘 다
`morai_udp` 하나다. `generated/`는 `templates/`와
[`bridge_profiles_v1.json`](../bridge/bridge_profiles_v1.json)에서 결정적으로 생성한다.

```bash
./scripts/udp_bridge.sh --simulator 26r1 --transport dev_local
./scripts/udp_bridge.sh --simulator competition --transport competition_lan
```

두 번째 명령은 Ctrl 대상 IP와 diagnostics를 현장 profile로 맞추지만 실제 LAN gate를 대신하지
않는다. 현장 network JSON은 Ctrl, Competition Vehicle Status, CollisionData만 LAN IP를 쓰고
Object/legacy/미확인 채널은 MORAI PC loopback에 둔다. renderer는 UDP field만 바꾸며
gRPC/rosbridge 설정은 별도 개발 도구 소유다.

```bash
python3 tools/bridge/render_morai_network_profiles.py --write
python3 tools/bridge/render_morai_network_profiles.py --check
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

현재 sensor JSON 네 개의 UDP IP는 `dev_local` export다. `competition_lan` sensor JSON 생성과
검증은 v3 live acceptance 작업에서 함께 수행하며, 그 전에는 network JSON만 보고 현장 센서
transport가 완료됐다고 판단하지 않는다.

## IP와 Docker/WSL

| transport | MORAI | Autonav |
|---|---|---|
| `dev_local` | `127.0.0.1` | `127.0.0.1` |
| `competition_lan` | `192.168.0.1` | `192.168.0.10` |

Docker Desktop 개발 compose는 `network_mode: host`이므로 Docker `ports:`를 열거하지 않는다.
`./scripts/docker_ros_up.sh check`로 Windows↔container loopback UDP를 검사한다. WSL/Windows의
모든 포트를 여는 것은 필요하지 않고 공격 면만 넓힌다. 현장에서는 MORAI `192.168.0.1`을
source로 제한해 Autonav inbound UDP `7502`, `9092`, `9291/9293/9295/9297/9299`, `9301/9303`
만 허용한다. GT 전용 `9305/9307/9309/9311`과 legacy oracle `909/7505`는 본선 allowlist에
넣지 않는다. Ctrl `9093`은 Autonav에서 MORAI로 나가는 목적지다.

Windows loopback 검증은 실제 LAN ingress를 증명하지 않는다. `competition_lan`은 native Linux
두 PC에서 route, firewall, bind, packet rate와 재시작 복구를 별도로 통과해야 한다.
