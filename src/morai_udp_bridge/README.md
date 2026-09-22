# morai_udp_bridge

MORAI Network UDP의 Ego/Object/Collision packet을 ROS1으로 변환하고 `/ctrl_cmd`를 Ego Ctrl Cmd
UDP로 보내는 현재 bridge다. 단일 `asmc_bridge`로의 통합은 아직 시작하지 않았다.

- 구현·단위·검증: [bridge package-group guide](../../docs/packages/asmc_bridge/README.md)
- 현재 포트와 실행: [운영 가이드](../../docs/operations/README.md)
- 외부 message 정책: [MORAI message policy](../../docs/contracts/simulator/morai-msgs-beta-policy.md)

```bash
roslaunch morai_udp_bridge morai_udp_bridge.launch
```

현재 기본값은 Ego `909`, Object `7505`, Collision `9092`, Ctrl target `127.0.0.1:9093`이다.
`ObjectInfo`는 개발 oracle이며 본선 profile에 포함하면 안 된다. `simulator_profile`은 진단
라벨이며 지원 release 모두 `morai_udp` 하나를 쓴다. release별 MORAI import와 transport IP는
[`config/morai`](../../config/morai/)에서 분리한다.

입출력:

- publish `/Ego_topic`, `/Object_topic`, `/CollisionData`
- subscribe `/ctrl_cmd`
- publish `/morai_udp_bridge/diagnostics`

Competition Vehicle Status decoder 전환 전까지 Ego `909`는 개발 회귀 기준으로 유지한다.
