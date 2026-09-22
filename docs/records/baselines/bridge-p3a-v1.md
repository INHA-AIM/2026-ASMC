# P3-A bridge baseline and profile contract

> 기준일: 2026-09-23
> 기계 판독 정본: [`config/bridge/bridge_profiles_v1.json`](../../../config/bridge/bridge_profiles_v1.json)

## 동결한 결정

26.R1.H3와 25.S4.MolitComp03의 현재 Python bridge에는 packet parser나 topic 동작 분기가 없고
`simulator_profile`은 diagnostics 문자열뿐이었다. P3-A부터 runtime protocol을 `morai_udp` 하나로
고정한다. `--simulator` 선택은 release별 MORAI import와 실행 provenance를 고르는 용도이며 새
bridge 구현을 선택하지 않는다.

MORAI network export 자체는 동일하지 않다. 26r1 export는 22개 entry와 Ctrl `PUBSUB_TYPE=786`,
MolitComp03 export는 12개 entry와 Ctrl `PUBSUB_TYPE=771`을 가진다. 따라서 release template은
분리하고, IP만 `dev_local` 또는 `competition_lan`으로 렌더링한다.

현장 renderer는 allowlist 방식이다. Ctrl, Competition Vehicle Status, CollisionData만 LAN IP로
승격하고 Object/legacy/미확인 network entry는 loopback에 남긴다. 센서셋의 현장 IP 파생본은 v3
live acceptance와 함께 만드는 후속 범위다.

Ctrl 기본값은 이번 변경부터 `9093/9094`, mode 2다. 과거 `9111/9112` 파일은 archive로
격리했다. Competition Vehicle Status `7603/7502`는 network 계약만 동결됐으며 현재 bridge의
legacy Ego Status `908/909` decoder를 대체하는 작업은 아직 남았다.

현재 구현 기준선은 `integration_launch`, `morai_udp_bridge`, `morai_sensor_bridge`,
`vehicle_state` 네 package와 그 entrypoint/launch/node SHA, 필수 publish/subscribe topic을 기계
판독 계약에 포함한다. `/Ego_topic`, `/Object_topic`, `/CollisionData`, Camera×4, LiDAR, GPS/IMU,
`/vehicle/state`, 두 diagnostics가 현재 핵심 출력이고 Semantic×3/Instance LiDAR는 개발 optional로
분리한다.

## 센서 기준

- v1: Camera-4 pitch 340°, VLP16 10 Hz, 과거 재현
- v2: Camera-4 pitch 328°, VLP16 10 Hz, 현재 회귀 기준
- v3: v2 geometry/ports + VLP16 15 Hz UI export, live acceptance 대기
- GT: v2 + Semantic Camera×3 + Instance LiDAR, 개발 전용

v3의 파일 값과 실제 packet rate를 구분한다. 30초 평균 14~16 Hz, error/latency/CPU와 손재호
tracking 비교를 통과하기 전에는 배포 승인이 아니다.

## WSL/Docker 판단

개발 compose는 host networking이고 개별 Docker port publishing을 쓰지 않는다. 따라서 “WSL의
모든 포트 개방”은 해결책이 아니다. loopback은 왕복 probe로 확인하고, 현장 firewall은 필요한
inbound UDP destination만 MORAI PC source IP에 한정해 허용한다. 실제 LAN은 native Linux 두 PC
리허설을 별도 gate로 둔다.

## 재현·검증

```bash
python3 tools/bridge/render_morai_network_profiles.py --check
python3 tools/audit/validate_bridge_profiles.py
python3 -m unittest tools/audit/test_bridge_profiles.py
./scripts/docker_ros_up.sh config
```

다음 P3 작업은 Competition Vehicle Status decoder/fixture, transport-aware launch, privileged
stream allowlist다. 그 뒤 v3 15 Hz live acceptance와 실제 LAN 리허설로 넘어간다.
