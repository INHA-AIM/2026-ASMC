# P3-B Competition Vehicle Status decoder baseline

> 기준일: 2026-09-23
>
> 기계 판독 정본: [`config/bridge/bridge_profiles_v1.json`](../../../config/bridge/bridge_profiles_v1.json)
>
> **P3-E 정정:** 이 baseline의 `7603/7502` 매핑은 잘못됐다. type 258은 TrafficLight
> status이고 MolitComp Competition Vehicle Status는 type 787, `9797/9798`이다. decoder
> fixture 자체는 유지하되 live 승인 대상 port는 P3-E 계약을 따른다.

## 적용 범위

P3-A의 단일 `morai_udp` runtime과 `local`/`lan2lan` 분리를 유지하며 Competition
Vehicle Status `7603→7502` 수신을 추가했다. bridge는 Ego Vehicle Status를
`/Ego_topic`, Competition Vehicle Status를 `/Competition_topic`으로 독립 발행하고 수신
통계와 diagnostics도 분리한다.

대회판 assembly 정적 분석으로 152-byte vehicle-status body와 181-byte datagram을
확인했다. parser는 `#MoraiInfo$`, declared body length, CRLF tail, finite numeric value와
지원하는 packet variant를 검사한다. synthetic fixture는 이 계약의 회귀 테스트이며
실제 대회 packet capture로 위장하지 않는다.

## 제공 필드와 제약

- ROS type: `morai_msgs/EgoVehicleStatus`
- position, velocity, heading, acceleration, accel/brake, wheel angle을 변환한다.
- velocity는 bridge 계약에 따라 km/h에서 m/s로 변환한다.
- UDP body에 없는 `unique_id`는 `-1`, `lateral_offset`은 기본 `0`이다.
- `/Ego_topic`을 즉시 제거하거나 Competition을 authoritative localization으로 간주하지 않는다.

## 검증과 남은 gate

```bash
python3 tools/audit/validate_bridge_profiles.py
python3 -m unittest tools/audit/test_bridge_profiles.py
catkin_make run_tests_morai_udp_bridge
rostopic hz /Competition_topic
rostopic echo -n1 /Competition_topic
```

정적 unit test와 synthetic UDP→ROS publish 테스트는 필수 gate다. 배포 승인을 위해서는
대회 시뮬레이터에서 live packet을 캡처해 field·rate·timestamp·staleness를 비교하고,
Ego fallback과 상태 입력 우선순위를 판제 팀과 확정해야 한다.
