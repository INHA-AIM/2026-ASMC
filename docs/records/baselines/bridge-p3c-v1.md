# P3-C deployment stream and sensor transport baseline

> 기준일: 2026-09-23
>
> 기계 판독 정본: [`config/bridge/bridge_profiles_v1.json`](../../../config/bridge/bridge_profiles_v1.json)

## 실행 계약

bridge runtime은 여전히 release와 무관한 `morai_udp` 하나다. transport와 input policy를
다음처럼 독립 축으로 둔다.

| 축 | 값 | 기본 |
|---|---|---|
| transport | `local`, `lan2lan` | `local` |
| streams | `competition`, `development` | `competition` |

`competition`은 Ego/Competition/Collision, RGB 4대, VLP16, GPS, IMU만 열고 Object,
Semantic 3대, Instance LiDAR는 socket과 ROS publisher를 생성하지 않는다. `development`는
이 oracle를 추가하지만 `local`에서만 허용한다. CLI와 두 bridge node가 모두
`lan2lan + development`를 거부한다.

## 센서 transport 파생본

v2·v3 원본 UI export에서 다음 네 파일을 결정적으로 생성한다.

- `ASMC_sensor_set_v2_local.json`
- `ASMC_sensor_set_v2_lan2lan.json`
- `ASMC_sensor_set_v3_local.json`
- `ASMC_sensor_set_v3_lan2lan.json`

renderer는 모든 typed `udpConfig`를 찾아 host/destination IP만 바꾸고, 포트·pose·rate·sensor
schema가 원본과 같은지 검사한다. GT는 privileged local 전용이므로 LAN2LAN 파생본을
생성하지 않는다.

## 남은 gate

- Competition Vehicle Status live packet capture·field 교차 검증
- v3 설정·손재호 소비 loop 15 Hz 확인(후속 P3-E에서 실제 완성 scan 약 12 Hz 허용으로 확정)
- Windows/WSL/Docker 또는 native Linux의 실제 두 PC LAN2LAN
- firewall·route·packet rate·restart recovery·offline 리허설

```bash
python3 tools/bridge/render_morai_sensor_profiles.py --check
python3 tools/audit/validate_bridge_profiles.py
./scripts/udp_bridge.sh
python3 scripts/verify_morai_topics.py --profile competition
```
