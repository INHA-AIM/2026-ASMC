# asmc_bridge

MORAI Network/Sensor UDP와 GPS/IMU 기반 `/vehicle/state`를 한 명령으로 기동하는 canonical
launch façade다. P3-D에서는 기존 구현을 이동하거나 삭제하지 않으며 다음 패키지를 조합한다.

- `morai_udp_bridge`
- `morai_sensor_bridge`
- `vehicle_state`
- 호환 composition인 `integration_launch`

```bash
roslaunch asmc_bridge bridge.launch
# 저장소 표준 진입점
./scripts/udp_bridge.sh
```

기본값은 `morai_udp + local + competition`이고 `/vehicle/state`까지 발행한다. 기존 stack이
`vehicle_state.launch`를 별도로 소유하는 일시적 호환 실행에서만
`enable_vehicle_state:=false`를 사용한다. 같은 `/vehicle/state` publisher를 두 개 실행하면 안 된다.

Object/Semantic/Instance oracle은 `./scripts/udp_bridge.sh --streams development`로만 열 수 있고
LAN2LAN과 함께 사용할 수 없다. simulator release는 launch 인자가 아니며 MORAI UI에 import한
network JSON schema로만 구분한다.
