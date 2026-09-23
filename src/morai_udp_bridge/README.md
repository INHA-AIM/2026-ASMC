# morai_udp_bridge

MORAI Network UDP의 Ego/Competition/Object/Collision packet을 ROS1으로 변환하고 `/ctrl_cmd`를 Ego Ctrl Cmd
UDP로 보내는 Network adapter다. P3-D의 `asmc_bridge` façade가 이 node를 조합하지만,
rollback·packet parity를 위해 구현 패키지 경계는 유지한다.

- 구현·단위·검증: [bridge package-group guide](../../docs/packages/asmc_bridge/README.md)
- 현재 포트와 실행: [운영 가이드](../../docs/operations/README.md)
- 외부 message 정책: [MORAI message policy](../../docs/contracts/simulator/morai-msgs-beta-policy.md)

```bash
roslaunch morai_udp_bridge morai_udp_bridge.launch
```

현재 기본값은 Ego `9099`, MolitComp Competition `9798`, Object `7505`, Collision `9092`,
Ctrl target `127.0.0.1:9093`이다.
`ObjectInfo`는 개발 oracle이며 본선 profile에 포함하면 안 된다. 진단의 `runtime_profile`은
`morai_udp` 하나다. release별 MORAI import와 transport IP는
[`config/morai`](../../config/morai/)에서 분리한다.
기본 `stream_profile=competition`은 Object socket과 `/Object_topic` publisher를 생성하지 않는다.
Object oracle은 local에서 `--streams development`로만 활성화한다.

입출력:

- publish `/Ego_topic`, `/Competition_topic`, `/Object_topic`, `/CollisionData`
- subscribe `/ctrl_cmd`
- publish `/morai_udp_bridge/diagnostics`

Competition Vehicle Status는 MolitComp type 787 (`9797→9798`)과 대회판 assembly의
152-byte payload 정적 분석 및 synthetic
fixture를 기준으로 엄격하게 길이·header·tail을 검사한다. `unique_id`는 UDP body에 없어
`-1`, `lateral_offset`은 제공되지 않아 기본 `0`이다. 현장 승인 전까지 Ego Vehicle Status
`9099`도 독립 회귀 기준으로 유지한다. type 258 (`7603→7502`)은 TrafficLight status이며 이
bridge의 Competition parser에 연결하지 않는다.

```bash
rostopic hz /Competition_topic
rostopic echo -n1 /Competition_topic
rostopic echo /morai_udp_bridge/diagnostics
```
