# asmc_msgs

ASMC 내부 ROS1 메시지의 정본 패키지다. 패키지 버전 `1.0.0`부터 기존 planning/perception
메시지 13개와 HD Map·competition metadata 메시지 12개의 MD5를 동결한다.

- 사람이 읽는 규약: [HD Map topic contract v1](../../docs/contracts/hd-map-topic-contract-v1.md)
- 기계 판독 규약: [hd_map_topic_contract_v1.yaml](../../config/contracts/hd_map_topic_contract_v1.yaml)
- 검증: `python3 tools/audit/validate_p1b_contract.py`

기존 `.msg`에 필드를 추가하거나 순서를 바꾸면 ROS1 MD5와 wire compatibility가 깨진다. 변경이
필요하면 기존 타입을 유지하고 새 contract/version/type을 추가한다.

ROS Noetic 환경의 단독 build 명령은 다음과 같다.

```bash
catkin_make --pkg asmc_msgs
catkin_make run_tests_asmc_msgs
```

이 패키지는 interface만 제공하며 publisher, metadata 생성기와 최종 geometry는
`asmc_hd_map`이 소유한다. 판제는 `/hd_map/global/*` latched 토픽을 한 번 받아 위치 기반으로
조회하는 방식을 기본으로 하고, `/hd_map/local/lanes`와 `/hd_map/local/drivable_grid`는
선택적 파생 캐시로만 사용한다. `/hd_map/local/current_lane`은 후륜축 중심 `base_link`를
기준으로 한 상태 포함(`INSIDE/BOUNDARY/NEARBY/STALE`) 차로 association이다. local crop에는
mission/checkpoint/marking metadata가 중복되지 않는다.
