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

수신은 두 transport 모두 같은 UDP port에서 이뤄지지만 Ctrl 목적지 IP가 다르다. local은
인자 없이 실행하고 현장은 `--transport lan2lan`을 명시한다. P3-E local 실측에서 v3 LiDAR는
azimuth 조립 뒤 약 12 Hz였고 아직 배포 미승인이다. MolitComp Competition Vehicle Status는
type 787의 `9797→9798`이며, type 258의 `7603→7502` TrafficLight oracle과 구분한다.
type 258은 GetTrafficLight 조회 서비스가 아니며, 조사한 두 release export에는 해당 service
type 778이 없다. production bridge는 어느 쪽도 신호등 판단 입력으로 사용하지 않는다.
