# integration_launch

현재 MORAI network/sensor bridge를 함께 실행하는 launch-only ROS1 package다. Planning/Control
공동 소유이므로 Phase 3 통합 때 일방적으로 이동하거나 삭제하지 않는다.

- bridge 구조와 통합 gate: [bridge package-group guide](../../docs/packages/asmc_bridge/README.md)
- Docker·MORAI UI·진단: [운영 가이드](../../docs/operations/README.md)

```bash
roslaunch integration_launch morai_udp_bridges.launch
# 또는 저장소 진입점
./scripts/udp_bridge.sh --simulator 26r1
```

현재 `--simulator competition`은 provenance label만 바꾸며 현장 transport profile을 적용하지
않는다. 현장 IP/포트와 privileged stream allowlist가 구현되기 전에는 이 명령을 본선 준비
완료의 근거로 사용하지 않는다.
