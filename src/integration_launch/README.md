# integration_launch

현재 MORAI network/sensor bridge를 함께 실행하는 launch-only ROS1 package다. Planning/Control
공동 소유이므로 Phase 3 통합 때 일방적으로 이동하거나 삭제하지 않는다.

- bridge 구조와 통합 gate: [bridge package-group guide](../../docs/packages/asmc_bridge/README.md)
- Docker·MORAI UI·진단: [운영 가이드](../../docs/operations/README.md)

```bash
roslaunch integration_launch morai_udp_bridges.launch
# 또는 저장소 진입점
./scripts/udp_bridge.sh
```

26r1과 대회판은 같은 launch를 사용한다. release는 MORAI UI의 network JSON에서 고르고,
bridge에서는 기본 `local` 또는 `--transport lan2lan`만 선택한다. 실제 LAN gate와 privileged
stream allowlist가 완료되기 전에는 `lan2lan` 선택만으로 본선 준비가 끝난 것은 아니다.
