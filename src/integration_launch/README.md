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
bridge에서 simulator release는 선택하지 않고 transport와 stream policy만 분리한다.
기본은 `local + competition`이며 `--streams development`는 Object/Semantic/Instance를 여는
local 전용 oracle다. `lan2lan + development`는 fail-closed로 거부한다. 실제 LAN gate 전에는
`lan2lan` 선택만으로 본선 준비가 끝난 것은 아니다.
